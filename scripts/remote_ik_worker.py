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


def solve_intent(solver, measured, delta, gripper_limits, *, sent=None, desired_xyz_m=None):
    """Apply base-frame XYZ, local pitch/roll and normalized jaw increments."""
    joints = np.asarray(measured[:5], dtype=float)
    pose = solver.kinematics.forward_kinematics(joints)
    measured_xyz_m = pose[:3, 3].tolist()
    # Continue from the target the Pi actually issued. Motor feedback rounds to
    # encoder ticks, so restarting each increment from it can erase small moves.
    reference = measured if sent is None else sent
    if sent is not None:
        joints = np.asarray(sent[:5], dtype=float)
    target = (solver.kinematics.forward_kinematics(joints) if sent is not None else pose).copy()
    arm_input = any(delta[:5])
    baseline_xyz_m = measured_xyz_m if desired_xyz_m is None else desired_xyz_m
    if arm_input:
        target[:3, 3] = baseline_xyz_m
        target[:3, 3] += delta[:3]
    pitch, roll = np.deg2rad(delta[3:5])
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    target[:3, :3] = target[:3, :3] @ np.array([
        [cp, 0, sp], [0, 1, 0], [-sp, 0, cp],
    ]) @ np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])

    response = {"xyz_m": measured_xyz_m, "baseline_xyz_m": baseline_xyz_m,
                "requested_xyz_m": target[:3, 3].tolist(), "target": None, "reason": ""}
    if not any(delta):
        return response
    if arm_input:
        result = solver.solve_pose(target, joints)
        if not result.accepted:
            return {**response, "reason": result.reason}
        joints = result.joints_deg
    jaw = float(np.clip(reference[5] + delta[5], *gripper_limits))
    if arm_input or jaw != reference[5]:
        response["target"] = [*joints.tolist(), jaw]
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
        solver = LeRobotPoseSolver(limits[:5], urdf_path=path)
        if args.analyze_trace is not None:
            trace = json.loads(args.analyze_trace.read_text(encoding="utf-8"))
            if trace["model"] != model:
                parser.error("Trace model differs from this URDF or calibration")
            for tick in trace["ticks"]:
                for channel in ("issued_before", "issued", "measured"):
                    joints = tick[channel]
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
            reply(solve_intent(solver, measured, delta, limits[5], sent=sent, desired_xyz_m=desired_xyz_m))


if __name__ == "__main__":
    main()
