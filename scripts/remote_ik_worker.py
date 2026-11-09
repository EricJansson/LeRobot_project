"""WSL2 stdio worker for official LeRobot IK. This process never opens serial."""

import argparse
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np

from robot_program.lerobot_ik_controller import DEFAULT_URDF, LeRobotPoseSolver
from robot_program.remote_ik_protocol import (
    MAX_FRAME, calibrated_follower, channel_limits, decode_frame, encode_frame, finite_vector,
    kinematic_urdf, model_id,
)


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
                "issued_xyz_m": pose[:3, 3].tolist()}
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
                        progress_fraction=fraction, candidate_xyz_m=achieved[:3, 3].tolist(),
                        requested_xyz_m=(baseline + fraction * np.asarray(delta[:3])).tolist(),
                        search_attempts=attempt + 1)
        return response
    response["search_attempts"] = attempt + 1
    return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_URDF.parent / "calibration")
    parser.add_argument("--analyze-trace", type=Path, metavar="PATH",
                        help="Write the saved trace with issued and measured XYZ to stdout")
    args = parser.parse_args()
    urdf = kinematic_urdf()
    limits = channel_limits(calibrated_follower("unused", args.calibration_dir))
    model = model_id(urdf, args.calibration_dir)

    def reply(data):
        sys.stdout.buffer.write(encode_frame(data))
        sys.stdout.buffer.flush()

    with TemporaryDirectory(prefix="lerobot-remote-ik-") as directory:
        path = Path(directory) / "kinematic.urdf"
        path.write_text(urdf, encoding="utf-8")
        solver = LeRobotPoseSolver(limits[:5], urdf_path=path, position_tolerance_m=.00005, orientation_weight=.0001)
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
        reply({"model": model, "limits": limits})
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
