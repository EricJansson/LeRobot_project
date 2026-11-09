"""WSL2 stdio worker for official LeRobot IK. This process never opens serial."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np

from robot_program.lerobot_ik_controller import DEFAULT_URDF, MAX_ITERATIONS, LeRobotPoseSolver
from robot_program.remote_ik_protocol import (
    MAX_FRAME, MOTION_POLICY, PROTOCOL_VERSION, calibrated_follower, calibrated_channel_limits,
    channel_limits, command_limits, decode_frame, encode_frame, finite_vector, kinematic_urdf, model_id,
)


USEFUL_PROGRESS_FRACTION = .125
NORMAL_PATH_BOUND_DEG = 45.
ESCAPE_PATH_BOUND_DEG = 75.
SEARCH_FRACTIONS = tuple(2. ** -attempt for attempt in range(9))
APPROXIMATE_ITERATIONS = 4
JOINT_STEP_EPS_DEG = 1e-8
TRANSLATION_EPS_M = 1e-8
CONE_EPS_M = 1e-10
RECOVERY_TRANSLATION_M = .001
RECOVERY_ITERATIONS = 4
JACOBIAN_PROBE_DEG = .001
JACOBIAN_RCOND = 1e-4


@dataclass
class MotionCandidate:
    score: tuple
    joints_deg: np.ndarray
    pose: np.ndarray
    approximate: bool
    bound_deg: float


@dataclass
class RecoveryCandidate:
    error_m: float
    joints_deg: np.ndarray
    xyz_m: np.ndarray


def solve_intent(solver, measured, delta, gripper_limits, *, sent=None, desired_xyz_m=None,
                 joint_step_limits=None, recovery_step_m=.001, correction=False):
    """Bounded Cartesian search; no pending input is accumulated on rejection.

    Recovery from a measured stop is explicitly separated from operator progress.
    Its translation is bounded before applying the same joint-rate feasibility test.
    """
    reference = measured if sent is None else sent
    joints = np.asarray(reference[:5], dtype=float)
    pose = solver.kinematics.forward_kinematics(joints).copy()
    measured_pose = (pose.copy() if sent is None else
                     solver.kinematics.forward_kinematics(np.asarray(measured[:5], dtype=float)).copy())
    baseline = np.asarray(measured_pose[:3, 3] if desired_xyz_m is None else desired_xyz_m, dtype=float)
    response = {"xyz_m": measured_pose[:3, 3].tolist(), "baseline_xyz_m": baseline.tolist(),
                "requested_xyz_m": (baseline + delta[:3]).tolist(), "target": None, "reason": "",
                "progress_fraction": 0., "candidate_xyz_m": None, "recovering": False,
                "issued_xyz_m": pose[:3, 3].tolist(),
                "motion_status": "Armed—constrained" if any(delta[:5]) or correction else None}
    if not any(delta) and not correction:
        return response
    arm_input = any(delta[:5]) or correction
    if joint_step_limits is not None and hasattr(solver, "orientation_weight"):
        # A five-joint arm cannot keep an arbitrary six-DOF orientation during
        # lateral translation. Prioritize XYZ then retain the existing local
        # pitch/roll behavior for explicit orientation-only input.
        orientation_only = any(delta[3:5]) and not any(delta[:3])
        solver.orientation_weight = .01 if orientation_only else .0001
        solver.position_tolerance_m = .0005 if orientation_only else .00005
    jaw = float(np.clip(reference[5] + delta[5], *gripper_limits))
    if not arm_input:
        response["target"] = [*joints.tolist(), jaw] if jaw != reference[5] else None
        return response
    recovery = baseline - pose[:3, 3]
    length = float(np.linalg.norm(recovery))
    if joint_step_limits is not None and length > recovery_step_m:
        recovery *= recovery_step_m / length
    response["recovering"] = length > .0005
    precise_candidate = None
    # First try the entire input. Halving bounds solver work and releases a
    # rejected outward request immediately when fresh inward input arrives.
    for attempt in range(9 if joint_step_limits is not None else 1):
        fraction = 2. ** -attempt
        target = pose.copy()
        target[:3, 3] += fraction * (recovery + np.asarray(delta[:3]))
        pitch, roll = np.deg2rad(np.asarray(delta[3:5]) * fraction)
        cp, sp, cr, sr = np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
        target[:3, :3] = pose[:3, :3] @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array(
            [[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
        result = solver.solve_pose(target, joints)
        if not result.accepted:
            response["reason"] = result.reason
            continue
        if joint_step_limits is not None and np.any(
                np.abs(result.joints_deg - joints) > np.asarray(joint_step_limits[:5]) + 1e-8):
            response["reason"] = "Cartesian increment exceeds Pi rate allowance"
            continue
        achieved = result.pose.copy()
        translation = np.asarray(delta[:3], dtype=float)
        if (joint_step_limits is not None and not correction and np.linalg.norm(translation) > 0
                and (achieved[:3, 3] - pose[:3, 3]) @ translation / np.linalg.norm(translation)
                <= max(1e-9, min(1e-6, np.linalg.norm(translation) * .02))):
            response["reason"] = "No forward Cartesian progress"
            continue
        if correction and (np.linalg.norm(achieved[:3, 3] - pose[:3, 3]) > recovery_step_m + 1e-8
                           or np.linalg.norm(achieved[:3, 3] - baseline) >= length):
            response["reason"] = "No bounded stationary correction progress"
            continue
        translation = np.asarray(delta[:3], dtype=float)
        norm2 = float(translation @ translation)
        # Solver tolerance is not issued progress. In particular, tiny accepted
        # boundary steps must not grow an unreachable Cartesian reference.
        issued_fraction = (float(np.clip((achieved[:3, 3] - baseline) @ translation / norm2, 0., fraction))
                           if norm2 else 0.)
        accepted_xyz = baseline + issued_fraction * translation
        # Recheck FK against the retained line, not only the short solver
        # request. Otherwise solver tolerance can accumulate sideways at reach.
        if np.linalg.norm(achieved[:3, 3] - accepted_xyz) > max(.0005, length) + 1e-8:
            response["reason"] = "Issued FK would increase retained-path error"
            continue
        response["accepted_xyz_m"] = accepted_xyz.tolist()
        response.update(target=[*result.joints_deg.tolist(), jaw], reason="",
                        motion_status="Following path",
                        progress_fraction=fraction, candidate_xyz_m=achieved[:3, 3].tolist(),
                        requested_xyz_m=(baseline + fraction * np.asarray(delta[:3])).tolist(),
                        search_attempts=attempt + 1)
        # Near a boundary, accepting a tolerance-sized fraction forever can
        # look like a lock. Keep that option, but first seek a useful escape.
        if ((achieved[:3, 3] - pose[:3, 3]) @ translation < USEFUL_PROGRESS_FRACTION * norm2
                and joint_step_limits is not None and not correction
                and any(delta[:3]) and hasattr(solver, "limits_deg")):
            if precise_candidate is None:
                precise_candidate = response.copy()
            response["target"] = None
            continue
        return response
    response["search_attempts"] = attempt + 1
    # Only the remote rate-aware path opts into approximation. Legacy callers
    # and stationary correction keep their strict Cartesian contract.
    if (joint_step_limits is not None and not correction and any(delta[:3])
            and hasattr(solver, "limits_deg")):
        adapted = adapt_translation(solver, joints, pose, delta, jaw, joint_step_limits, response)
        if adapted["target"] is not None:
            translation = np.asarray(delta[:3])
            if (precise_candidate is None or
                    np.dot(np.array(adapted["candidate_xyz_m"]) - pose[:3, 3], translation) >
                    np.dot(np.array(precise_candidate["candidate_xyz_m"]) - pose[:3, 3], translation)):
                return adapted
    if precise_candidate is not None:
        precise_candidate["search_attempts"] = response["search_attempts"]
        return precise_candidate
    return response


def _candidate_offsets(stage, direction, length, unit, sideways):
    offsets = [fraction * direction for fraction in SEARCH_FRACTIONS]
    if stage == "path":
        offsets += [scale * length * (unit + side) / 2 for scale in (1., .25) for side in sideways]
    elif stage == "escape":
        angle = np.deg2rad(ESCAPE_PATH_BOUND_DEG)
        offsets = [scale * length * (np.cos(angle) * unit + np.sin(angle) * side)
                   for scale in (.95, .475) for side in sideways]
    return offsets


def _bounded_candidate(solver, result, joints, pose, direction, length, steps, stage, bound_deg):
    q = np.asarray(result.joints_deg)
    if (not result.accepted or q.shape != (5,) or not np.isfinite(q).all()
            or np.any(q < solver.limits_deg[:, 0]) or np.any(q > solver.limits_deg[:, 1])
            or np.any(np.abs(q - joints) > np.asarray(steps[:5]) + JOINT_STEP_EPS_DEG)):
        return None
    achieved = solver.kinematics.forward_kinematics(q).copy()
    if achieved.shape != (4, 4) or not np.isfinite(achieved).all():
        return None
    movement = achieved[:3, 3] - pose[:3, 3]
    distance = float(np.linalg.norm(movement))
    forward = float(movement @ direction / length)
    if (forward <= max(1e-9, min(1e-6, length * .02)) or distance > length + TRANSLATION_EPS_M
            or forward < distance * np.cos(np.deg2rad(bound_deg)) - CONE_EPS_M):
        return None
    # Wide escape ranks by forward gain; narrow stages retain closest-path rank.
    score = ((False, -forward) if stage == "escape" else
             (forward < length * USEFUL_PROGRESS_FRACTION, float(np.linalg.norm(movement - direction))))
    return MotionCandidate(score, q.copy(), achieved.copy(), stage != "pitch", bound_deg)


def adapt_translation(solver, joints, pose, delta, jaw, steps, response):
    """Bounded position-only search, then closest useful nearby translation."""
    direction = np.asarray(delta[:3], dtype=float)
    length = float(np.linalg.norm(direction))
    attempts = response["search_attempts"]
    unit = direction / length
    axis = np.eye(3)[np.argmin(np.abs(unit))]
    lateral = np.cross(unit, axis)
    lateral /= np.linalg.norm(lateral)
    sideways = (lateral, -lateral, np.cross(unit, lateral), -np.cross(unit, lateral))
    fallback = None
    for stage in ("pitch", "path", "escape"):
        approximate = stage != "pitch"
        bound_deg = ESCAPE_PATH_BOUND_DEG if stage == "escape" else NORMAL_PATH_BOUND_DEG
        best = None
        for offset in _candidate_offsets(stage, direction, length, unit, sideways):
            requested = pose.copy()
            requested[:3, 3] += offset
            result = solver.solve_pose(requested, joints, position_only=True, approximate=approximate,
                                       max_iterations=APPROXIMATE_ITERATIONS if approximate else MAX_ITERATIONS)
            attempts += 1
            candidate = _bounded_candidate(solver, result, joints, pose, direction, length, steps, stage, bound_deg)
            if candidate is None:
                continue
            if best is None or candidate.score < best.score:
                best = candidate
            forward = float((candidate.pose[:3, 3] - pose[:3, 3]) @ direction / length)
            if not approximate and forward >= length * USEFUL_PROGRESS_FRACTION:
                break  # A precise position-only escape has priority over drift.
        if best is None:
            continue
        forward = float((best.pose[:3, 3] - pose[:3, 3]) @ unit)
        previous_forward = (float((fallback.pose[:3, 3] - pose[:3, 3]) @ unit) if fallback is not None else 0.)
        if fallback is None or forward > previous_forward:
            fallback = best
        # Try the wider cone only when precise and narrow adaptation both make
        # less than one eighth of the requested forward progress.
        if forward >= length * USEFUL_PROGRESS_FRACTION:
            break
    if fallback is not None:
        q, achieved = fallback.joints_deg, fallback.pose
        movement = achieved[:3, 3] - pose[:3, 3]
        forward = float(movement @ unit)
        response.update(target=[*q.tolist(), jaw], reason="", accepted_xyz_m=achieved[:3, 3].tolist(),
                        candidate_xyz_m=achieved[:3, 3].tolist(), progress_fraction=forward / length,
                        motion_status="Adapting path" if fallback.approximate else "Adapting pitch",
                        requested_xyz_m=(pose[:3, 3] + direction).tolist(), search_attempts=attempts,
                        path_bound_deg=fallback.bound_deg,
                        path_angle_deg=float(np.rad2deg(np.arccos(np.clip(forward / np.linalg.norm(movement), -1., 1.)))))
        return response
    response.update(reason="No legal forward movement", search_attempts=attempts)
    return response


def solve_recovery(solver, sent, delta, automatic, anchor, goal, steps, limits):
    """Bounded differential IK around the Pi's mandatory inward joint step.

    The violating axes and jaw are fixed to the Pi target. Only legal arm axes
    can coordinate, with FK checked after projection into the slow joint box.
    """
    response = {"target": None, "reason": "", "motion_status": "Recovering"}
    direction = np.asarray(delta[:3], dtype=float)
    size = float(np.linalg.norm(direction))
    if not size:
        return response
    direction *= min(1., RECOVERY_TRANSLATION_M / size)
    seed = np.asarray(automatic[:5], dtype=float)
    before = solver.kinematics.forward_kinematics(np.asarray(sent[:5], dtype=float))[:3, 3].copy()
    base = solver.kinematics.forward_kinematics(seed)[:3, 3].copy()
    requested = before + direction
    free = [i for i in range(5) if anchor[i] == goal[i]]
    if not free:
        return response
    bounds = np.asarray([[max(limits[i][0], sent[i] - steps[i]), min(limits[i][1], sent[i] + steps[i])]
                         for i in free])
    if np.any(bounds[:, 0] > bounds[:, 1]):
        return response
    q, best = seed.copy(), None
    baseline_error = float(np.linalg.norm(base - requested))
    for _ in range(RECOVERY_ITERATIONS):
        xyz = solver.kinematics.forward_kinematics(q)[:3, 3].copy()
        jacobian = []
        for i in free:
            probe = q.copy()
            probe[i] += JACOBIAN_PROBE_DEG  # Joint API consumes degrees, FK returns meters.
            jacobian.append((solver.kinematics.forward_kinematics(probe)[:3, 3].copy() - xyz) / JACOBIAN_PROBE_DEG)
        change = np.linalg.lstsq(np.asarray(jacobian).T, requested - xyz, rcond=JACOBIAN_RCOND)[0]
        trial = q.copy()
        trial[free] = np.clip(q[free] + change, bounds[:, 0], bounds[:, 1])
        for fraction in (1., .5, .25):
            candidate = q + fraction * (trial - q)
            pose = solver.kinematics.forward_kinematics(candidate).copy()
            achieved = pose[:3, 3]
            error = float(np.linalg.norm(achieved - requested))
            if (not np.isfinite(pose).all() or error >= baseline_error - 1e-8
                    or np.linalg.norm(achieved - before) > np.linalg.norm(base - before) + np.linalg.norm(direction) + 1e-8
                    or (achieved - base) @ direction <= 1e-12):
                continue
            if best is None or error < best.error_m:
                best = RecoveryCandidate(error, candidate.copy(), achieved.copy())
        if best is None:
            break
        q = best.joints_deg.copy()
    if best is not None:
        q, xyz = best.joints_deg, best.xyz_m
        response.update(target=[*q.tolist(), automatic[5]], candidate_xyz_m=xyz.tolist(),
                        issued_xyz_m=before.tolist(), requested_xyz_m=requested.tolist())
    return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_URDF.parent / "calibration")
    parser.add_argument("--analyze-trace", type=Path, metavar="PATH",
                        help="Write the saved trace with issued and measured XYZ to stdout")
    args = parser.parse_args()
    urdf = kinematic_urdf()
    follower = calibrated_follower("unused", args.calibration_dir)
    limits = channel_limits(follower)
    calibrated_limits = calibrated_channel_limits(follower)
    normal_limits = command_limits(limits)
    model = model_id(urdf, args.calibration_dir)

    def reply(data):
        sys.stdout.buffer.write(encode_frame(data))
        sys.stdout.buffer.flush()

    with TemporaryDirectory(prefix="lerobot-remote-ik-") as directory:
        path = Path(directory) / "kinematic.urdf"
        path.write_text(urdf, encoding="utf-8")
        solver = LeRobotPoseSolver(normal_limits[:5], urdf_path=path, position_tolerance_m=.00005, orientation_weight=.0001)
        if args.analyze_trace is not None:
            trace = json.loads(args.analyze_trace.read_text(encoding="utf-8"))
            if trace["model"] != model:
                parser.error("Trace model differs from this URDF or calibration")
            for tick in trace["ticks"]:
                for channel in ("issued_before", "issued", "measured", "candidate"):
                    joints = tick.get(channel)
                    pose = (solver.kinematics.forward_kinematics(np.asarray(joints[:5], dtype=float))
                            if finite_vector(joints, 6) else None)
                    tick[f"{channel}_xyz_m"] = pose[:3, 3].tolist() if pose is not None else None
            json.dump(trace, sys.stdout, indent=2)
            print()
            return
        reply({"model": model, "limits": limits, "calibrated_limits": calibrated_limits,
               "command_limits": normal_limits, "v": PROTOCOL_VERSION, "motion_policy": MOTION_POLICY})
        while frame := sys.stdin.buffer.readline(MAX_FRAME + 1):
            request = decode_frame(frame)
            measured, delta = request.get("measured"), request.get("delta")
            sent = request.get("sent")
            desired_xyz_m = request.get("desired_xyz_m")
            if not finite_vector(measured, 6) or not finite_vector(delta, 6):
                raise ValueError("Expected six finite measured channels and six finite increments")
            if sent is not None and not finite_vector(sent, 6):
                raise ValueError("Expected six finite sent channels")
            if desired_xyz_m is not None and not finite_vector(desired_xyz_m, 3):
                raise ValueError("Expected three finite desired XYZ values")
            if request.get("op") == "recover":
                for key in ("automatic", "anchor", "goal", "steps"):
                    if not finite_vector(request.get(key), 6):
                        raise ValueError(f"Expected six finite recovery {key} values")
                if sent is None or any(x <= 0 for x in request["steps"]):
                    raise ValueError("Recovery requires sent joints and positive step allowances")
                reply(solve_recovery(solver, sent, delta, request["automatic"], request["anchor"],
                                     request["goal"], request["steps"], limits))
                continue
            recovery_step = request.get("recovery_step_m", .001)
            if not isinstance(recovery_step, (int, float)) or not np.isfinite(recovery_step) or not 0 < recovery_step <= .001:
                raise ValueError("Recovery step must be positive and at most 1 mm")
            if type(request.get("correction", False)) is not bool:
                raise ValueError("Correction must be boolean")
            steps = request.get("joint_step_limits")
            if steps is not None and (not finite_vector(steps, 6) or any(x <= 0 for x in steps)):
                raise ValueError("Expected six positive joint step limits")
            reply(solve_intent(solver, measured, delta, limits[5], sent=sent, desired_xyz_m=desired_xyz_m,
                               joint_step_limits=steps, recovery_step_m=request.get("recovery_step_m", .001),
                               correction=request.get("correction", False)))


if __name__ == "__main__":
    main()
