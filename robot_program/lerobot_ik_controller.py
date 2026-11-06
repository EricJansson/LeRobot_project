"""URDF-based LeRobot IK, independent of the legacy RobotArm implementation.

Arm joints use LeRobot bus degrees; Cartesian transforms use meters. The
normalized gripper is deliberately excluded from the kinematic chain.
Constructing an adapter validates its model/dependencies without opening a port.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TARGET_FRAME = "gripper_frame_link"
DEFAULT_URDF = Path(__file__).resolve().parents[1] / "so101_new_calib.urdf"
POSITION_TOLERANCE_M = 0.0005
POSITION_TOLERANCE_REASON = "position tolerance not reached"
MAX_ITERATIONS = 20
JOINT_ROUNDOFF_DEG = 1e-8
GRIPPER_RANGE = (0.0, 50.0)


@dataclass
class IKResult:
    accepted: bool
    reason: str
    joints_deg: np.ndarray
    pose: np.ndarray


def _pose(value: np.ndarray) -> np.ndarray:
    pose = np.asarray(value, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError("Expected a finite 4x4 pose transform")
    return pose.copy()


def _joint_limits(bus, urdf_path: Path) -> np.ndarray:
    """Intersect URDF limits with endpoints normalized by the installed bus.

    _normalize is LeRobot's hardware-free conversion used by sync_read. Calling
    it here avoids copying the 0.6 bus's midpoint/resolution conventions or the
    legacy degree export. This integration is intentionally pinned to 0.6.1.
    """
    from lerobot.motors import MotorNormMode

    root = ET.parse(urdf_path).getroot()
    if root.find(f"link[@name='{TARGET_FRAME}']") is None:
        raise ValueError(f"URDF is missing {TARGET_FRAME}")
    endpoints = []
    for name in (*ARM_JOINTS, "gripper"):
        motor = bus.motors[name]
        calibration = bus.calibration[name]
        expected_mode = MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.DEGREES
        if motor.norm_mode != expected_mode or calibration.range_min >= calibration.range_max:
            raise ValueError(f"Invalid calibration or normalization mode for {name}")
        if name == "gripper":
            continue
        lo = bus._normalize({motor.id: calibration.range_min})[motor.id]
        hi = bus._normalize({motor.id: calibration.range_max})[motor.id]
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.attrib.get("type") != "revolute":
            raise ValueError(f"URDF is missing revolute joint {name}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"URDF is missing limits for {name}")
        bounds = [max(lo, np.rad2deg(float(limit.attrib["lower"]))),
                  min(hi, np.rad2deg(float(limit.attrib["upper"])))]
        if not np.isfinite(bounds).all() or bounds[0] >= bounds[1]:
            raise ValueError(f"Invalid intersected limits for {name}: {bounds}")
        endpoints.append(bounds)
    return np.asarray(endpoints)


class LeRobotPoseSolver:
    """Official LeRobot FK/IK with explicit joint bounds and no motor access."""

    def __init__(self, limits_deg, *, urdf_path: Path = DEFAULT_URDF, kinematics=None):
        self.limits_deg = np.asarray(limits_deg, dtype=float)
        if (self.limits_deg.shape != (5, 2) or not np.isfinite(self.limits_deg).all()
                or np.any(self.limits_deg[:, 0] >= self.limits_deg[:, 1])):
            raise ValueError("Expected five finite, nonempty arm joint ranges")
        if kinematics is None:
            from lerobot.model.kinematics import RobotKinematics

            kinematics = RobotKinematics(
                str(Path(urdf_path).resolve()), target_frame_name=TARGET_FRAME,
                joint_names=list(ARM_JOINTS),
            )
        if list(kinematics.joint_names) != list(ARM_JOINTS):
            raise ValueError("Kinematics must use the explicit five-joint arm order")
        self.kinematics = kinematics
        for name, (lo, hi) in zip(ARM_JOINTS, self.limits_deg):
            self.kinematics.robot.set_joint_limits(name, *np.deg2rad([lo, hi]))
        # A separate jaw joint must never become an unconstrained IK variable.
        self.kinematics.solver.mask_dof("gripper")
        self.kinematics.solver.enable_joint_limits(True)

    def solve_pose(self, target: np.ndarray, measured_deg: np.ndarray) -> IKResult:
        """Solve and validate without reading or writing hardware."""
        target = _pose(target)
        measured = np.asarray(measured_deg, dtype=float)
        if measured.shape != (5,) or not np.isfinite(measured).all():
            raise ValueError("Expected five finite measured arm joint degrees")
        q = measured.copy()
        measured_pose = _pose(self.kinematics.forward_kinematics(measured))
        achieved = measured_pose.copy()
        reason = POSITION_TOLERANCE_REASON
        accepted = False
        try:
            for _ in range(MAX_ITERATIONS):
                q = np.asarray(self.kinematics.inverse_kinematics(
                    q, target, position_weight=1.0, orientation_weight=0.01,
                ), dtype=float)
                if q.shape != (5,) or not np.isfinite(q).all():
                    reason = "nonfinite or malformed IK solution"
                    break
                if (np.any(q < self.limits_deg[:, 0] - JOINT_ROUNDOFF_DEG)
                        or np.any(q > self.limits_deg[:, 1] + JOINT_ROUNDOFF_DEG)):
                    reason = "joint limit exceeded"
                    break
                # Snap solver roundoff, then verify FK of the exact bounded command.
                q = np.clip(q, self.limits_deg[:, 0], self.limits_deg[:, 1])
                achieved = _pose(self.kinematics.forward_kinematics(q))
                error = float(np.linalg.norm(achieved[:3, 3] - target[:3, 3]))
                if error <= POSITION_TOLERANCE_M:
                    accepted, reason = True, "accepted"
                    break
        except (RuntimeError, ValueError) as exc:
            reason = f"IK solver failed: {exc}"
        if not accepted:
            q, achieved = measured.copy(), measured_pose
            self.kinematics.forward_kinematics(measured)
        return IKResult(accepted, reason, q.copy(), achieved.copy())


class LeRobotIKController(LeRobotPoseSolver):
    """Legacy local controller with one hardware owner."""

    def __init__(self, follower, *, urdf_path: Path = DEFAULT_URDF, kinematics=None):
        self.follower = follower
        super().__init__(_joint_limits(follower.bus, Path(urdf_path).resolve()),
                         urdf_path=urdf_path, kinematics=kinematics)
        self.target_pose: np.ndarray | None = None
        self._gripper = 0.0
        self._connected = False

    @classmethod
    def from_port(cls, port: str | None = None):
        from importlib.metadata import version

        if version("lerobot") != "0.6.1":
            raise RuntimeError("This IK mode requires LeRobot 0.6.1; see requirements-pi-ik.txt")
        from robot_program.lerobot_compat import SO101FollowerConfig
        from robot_program.so101_follower import SO101Follower
        from robot_program.utils.ports import auto_port, normalize_port

        config = SO101FollowerConfig(
            port=normalize_port(port or auto_port()), id="lerobot_arm", use_degrees=True,
        )
        return cls(SO101Follower(config))

    def connect(self):
        if self._connected:
            return
        try:
            self.follower.connect()
            self._connected = True
            self.follower.bus.enable_torque()
            self.sync_from_hardware()
        except BaseException:
            if self.follower.bus.is_connected:
                self._connected = True
                self.disconnect()
            raise

    def disconnect(self):
        if self._connected:
            try:
                self.follower.bus.disable_torque()
            finally:
                try:
                    self.follower.disconnect()
                finally:
                    self._connected = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    def _read_joints(self) -> np.ndarray:
        positions = self.follower.bus.sync_read("Present_Position")
        joints = np.array([positions[name] for name in ARM_JOINTS], dtype=float)
        self._gripper = float(positions["gripper"])
        if not np.isfinite([*joints, self._gripper]).all():
            raise ValueError("Hardware reported nonfinite joint positions")
        return joints

    def sync_from_hardware(self) -> np.ndarray:
        self.target_pose = _pose(self.kinematics.forward_kinematics(self._read_joints()))
        return self.target_pose.copy()

    def read_telemetry(self) -> tuple[dict[str, float], float, dict[str, float]]:
        """Read joint degrees, normalized gripper position, and base-frame XYZ in meters."""
        joints = self._read_joints()
        pose = _pose(self.kinematics.forward_kinematics(joints))
        xyz_m = dict(zip(("x", "y", "z"), pose[:3, 3].tolist()))
        return dict(zip(ARM_JOINTS, joints.tolist())), self._gripper, xyz_m

    def move_to_pose(self, target: np.ndarray, *, gripper_delta: float = 0.0) -> IKResult:
        result = self.solve_pose(target, self._read_joints())
        if result.accepted:
            action = {f"{name}.pos": float(value) for name, value in zip(ARM_JOINTS, result.joints_deg)}
            if gripper_delta:
                action["gripper.pos"] = self._gripper_target(gripper_delta)
            self.follower.send_action(action)
        self.target_pose = result.pose.copy()
        return result

    def _gripper_target(self, delta: float) -> float:
        if not np.isfinite(delta):
            raise ValueError("Expected finite gripper delta")
        return float(np.clip(self._gripper + delta, *GRIPPER_RANGE))

    def move_gripper(self, delta: float):
        """A jaw-only command leaves all arm targets untouched and bypasses IK."""
        if not delta:
            return
        self._read_joints()
        target = self._gripper_target(delta)
        if target != self._gripper:
            self.follower.send_action({"gripper.pos": target})
