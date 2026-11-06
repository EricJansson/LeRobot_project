"""WSL2 stdio worker for official LeRobot IK. This process never opens serial."""

import argparse
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np

from robot_program.lerobot_ik_controller import DEFAULT_URDF, LeRobotPoseSolver
from robot_program.remote_ik_protocol import (
    MAX_FRAME, calibrated_follower, channel_limits, decode_frame, encode_frame, finite_vector,
    kinematic_urdf, model_id,
)


def solve_intent(solver, measured, delta, gripper_limits):
    """Apply base-frame XYZ, local pitch/roll and normalized jaw increments."""
    joints = np.asarray(measured[:5], dtype=float)
    pose = solver.kinematics.forward_kinematics(joints)
    target = pose.copy()
    target[:3, 3] += delta[:3]
    pitch, roll = np.deg2rad(delta[3:5])
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    target[:3, :3] = target[:3, :3] @ np.array([
        [cp, 0, sp], [0, 1, 0], [-sp, 0, cp],
    ]) @ np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])

    response = {"xyz_m": pose[:3, 3].tolist(), "requested_xyz_m": target[:3, 3].tolist(),
                "target": None, "reason": ""}
    if not any(delta):
        return response
    if any(delta[:5]):
        result = solver.solve_pose(target, joints)
        if not result.accepted:
            return {**response, "reason": result.reason}
        joints = result.joints_deg
    jaw = float(np.clip(measured[5] + delta[5], *gripper_limits))
    if any(delta[:5]) or jaw != measured[5]:
        response["target"] = [*joints.tolist(), jaw]
    return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_URDF.parent / "calibration")
    args = parser.parse_args()
    urdf = kinematic_urdf()
    limits = channel_limits(calibrated_follower("unused", args.calibration_dir))

    def reply(data):
        sys.stdout.buffer.write(encode_frame(data))
        sys.stdout.buffer.flush()

    with TemporaryDirectory(prefix="lerobot-remote-ik-") as directory:
        path = Path(directory) / "kinematic.urdf"
        path.write_text(urdf, encoding="utf-8")
        solver = LeRobotPoseSolver(limits[:5], urdf_path=path)
        reply({"model": model_id(urdf, args.calibration_dir), "limits": limits})
        while frame := sys.stdin.buffer.readline(MAX_FRAME + 1):
            request = decode_frame(frame)
            measured, delta = request.get("measured"), request.get("delta")
            if not finite_vector(measured, 6) or not finite_vector(delta, 6):
                raise ValueError("Expected six finite measured channels and six finite increments")
            reply(solve_intent(solver, measured, delta, limits[5]))


if __name__ == "__main__":
    main()
