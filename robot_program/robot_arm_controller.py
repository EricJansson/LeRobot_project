"""
robot_program/robot_arm_controller.py

Combines RobotArm (IK/FK) with SO101Follower (hardware control).

Joint name mapping between SO101 motors and RobotArm / RobotState:
  shoulder_pan  <-> base_yaw_deg
  shoulder_lift <-> theta1_deg
  elbow_flex    <-> theta2_deg
  wrist_flex    <-> theta3_deg
  wrist_roll    -> extra DOF (not part of IK model)
  gripper       -> extra DOF (not part of IK model)
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Tuple

from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

from robot_program.RobotArm import ArmModel, RobotArm, RobotState, load_all_motor_limits
from robot_program.utils.ports import normalize_port, auto_port


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# ---------------------------------------------------------------------------
# Main class
# ---------------------------------------------------------------------------

class RobotArmController:
    """
    High-level controller that fuses:
    - RobotArm  : geometric model, IK/FK, joint-limit awareness
    - SO101Follower : Dynamixel bus communication for the physical arm

    Typical usage
    -------------
        with RobotArmController.from_port("COM3") as robot:
            robot.sync_from_hardware()           # seed state from real motors
            robot.move_to_xyz(30, 0, 20)         # IK → send to hardware
            x, y, z, phi = robot.end_effector_pose()
    """

    # Ordered list of the six SO101 joint names
    JOINT_NAMES = (
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    )

    # ------------------------------------------------------------------ init

    def __init__(
        self,
        follower: SO101Follower,
        arm: Optional[RobotArm] = None,
        calibration_path: Optional[Path] = None,
    ) -> None:
        """
        Args:
            follower: A (possibly not yet connected) SO101Follower instance.
            arm: Pre-built RobotArm.  If None, one is created automatically
                 with default ArmModel and calibration auto-loaded if available.
            calibration_path: Passed to RobotArm when arm=None.
        """
        self.follower = follower

        # Load calibration once — used both for the IK arm and for clamping all 6 joints.
        # Raises immediately if the file is missing or incomplete.
        _REQUIRED = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        self._motor_limits: dict = load_all_motor_limits(
            calibration_path, required_motors=_REQUIRED
        )

        # Build RobotArm, passing the already-loaded planar limits so the file
        # is not read a second time.
        if arm is not None:
            self.arm: RobotArm = arm
        else:
            planar_limits = [
                self._motor_limits["shoulder_lift"],
                self._motor_limits["elbow_flex"],
                self._motor_limits["wrist_flex"],
            ]
            self.arm = RobotArm(
                model=ArmModel(),
                joint_limits_deg=planar_limits,
                auto_load_limits=False,
            )

        # Extra DOFs not modelled by the IK
        self._wrist_roll: float = 0.0
        self._gripper: float = 0.0

        self._connected = False

    # --------------------------------------------------------- factory helpers

    @classmethod
    def from_port(
        cls,
        port: Optional[str] = None,
        robot_id: str = "lerobot_arm",
        arm: Optional[RobotArm] = None,
        calibration_path: Optional[Path] = None,
    ) -> "RobotArmController":
        """
        Convenience factory that builds the SO101Follower from a port string.

        Args:
            port: Serial port string, e.g. ``"COM3"`` or ``"/dev/ttyACM0"``.
                  If None, :func:`auto_port` is used.
            robot_id: ``id`` field forwarded to SO101FollowerConfig.
            arm: Optional pre-built RobotArm (see __init__).
            calibration_path: Optional path to calibration JSON.
        """
        resolved_port = normalize_port(port) if port else normalize_port(auto_port())
        cfg = SO101FollowerConfig(port=resolved_port, id=robot_id, use_degrees=True)
        follower = SO101Follower(cfg)
        return cls(follower=follower, arm=arm, calibration_path=calibration_path)

    # ------------------------------------------------- connection life-cycle

    def connect(self) -> None:
        """Connect to the robot and enable torque."""
        if self._connected:
            return
        self.follower.connect()
        self.follower.bus.enable_torque()
        self._connected = True
        # Seed RobotArm state from the actual motor positions so the IK
        # continuity heuristic starts from a meaningful reference posture.
        self.sync_from_hardware()

    def disconnect(self) -> None:
        """Disable torque and disconnect from the robot."""
        if not self._connected:
            return
        try:
            self.follower.bus.disable_torque()
        finally:
            self.follower.disconnect()
            self._connected = False

    def __enter__(self) -> "RobotArmController":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    # ------------------------------------------------------- state sync helpers

    def _hw_pos_to_state(self, pos: dict) -> None:
        """Update internal state from a hardware position dict."""
        s = self.arm.state
        s.base_yaw_deg = pos.get("shoulder_pan", s.base_yaw_deg)
        s.theta1_deg   = pos.get("shoulder_lift", s.theta1_deg)
        s.theta2_deg   = pos.get("elbow_flex",    s.theta2_deg)
        s.theta3_deg   = pos.get("wrist_flex",    s.theta3_deg)
        self._wrist_roll = pos.get("wrist_roll", self._wrist_roll)
        self._gripper    = pos.get("gripper",    self._gripper)

    # ------------------------------------------------------ public API

    def sync_from_hardware(self) -> dict:
        """
        Read present positions from the hardware and update internal state.

        Returns the raw position dict from the bus.
        """
        pos = self.follower.bus.sync_read("Present_Position")
        self._hw_pos_to_state(pos)
        return pos

    def send_state(self) -> None:
        """Send the current internal state to the hardware.

        All six joint values are clamped against calibration limits immediately
        before transmission — this is the final safety barrier regardless of
        how the internal state was set (IK, direct assignment, hardware sync).
        """
        lim = self._motor_limits
        s = self.arm.state
        action = {
            "shoulder_pan.pos":  _clamp(s.base_yaw_deg,    *lim["shoulder_pan"]),
            "shoulder_lift.pos": _clamp(s.theta1_deg,      *lim["shoulder_lift"]),
            "elbow_flex.pos":    _clamp(s.theta2_deg,      *lim["elbow_flex"]),
            "wrist_flex.pos":    _clamp(s.theta3_deg,      *lim["wrist_flex"]),
            "wrist_roll.pos":    _clamp(self._wrist_roll,  *lim["wrist_roll"]),
            "gripper.pos":       _clamp(self._gripper,     *lim["gripper"]),
        }
        self.follower.send_action(action)

    # .......................  IK-based movement  .......................

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        phi_deg: float = 0.0,
        wrist_roll: Optional[float] = None,
        gripper: Optional[float] = None,
        allow_phi_adaptation: bool = True,
        phi_adaptation_range: float = 45.0,
        phi_adaptation_step: float = 5.0,
        lateral_tol: float = 0.5,
    ) -> bool:
        """
        Solve IK for the target Cartesian position and send to hardware.

        Args:
            x, y, z: Target end-effector position in **cm** (world frame).
            phi_deg: Desired end-effector orientation (degrees).
            wrist_roll: If given, override the wrist-roll joint value.
            gripper: If given, override the gripper value (0-50).
            allow_phi_adaptation: Search nearby phi values if exact phi fails.
            phi_adaptation_range: Max phi deviation to search (degrees).
            phi_adaptation_step: Step size for phi search (degrees).
            lateral_tol: Lateral positioning tolerance for yaw solver (cm).

        Returns:
            True if IK succeeded and command was sent; False if unreachable.
        """
        success = self.arm.move_end_effector(
            target_xyz=(x, y, z),
            phi_deg=phi_deg,
            lateral_tol=lateral_tol,
            allow_phi_adaptation=allow_phi_adaptation,
            phi_adaptation_range=phi_adaptation_range,
            phi_adaptation_step=phi_adaptation_step,
        )
        if not success:
            return False

        if wrist_roll is not None:
            self._wrist_roll = _clamp(wrist_roll, *self._motor_limits["wrist_roll"])
        if gripper is not None:
            self._gripper = _clamp(gripper, *self._motor_limits["gripper"])

        self.send_state()
        return True

    # .......................  Direct joint control  ....................

    def set_joint_angles(
        self,
        shoulder_pan: Optional[float] = None,
        shoulder_lift: Optional[float] = None,
        elbow_flex: Optional[float] = None,
        wrist_flex: Optional[float] = None,
        wrist_roll: Optional[float] = None,
        gripper: Optional[float] = None,
    ) -> None:
        """
        Override individual joints and send to hardware.
        Joints left as None keep their current value.
        The RobotArm state is updated to stay consistent.
        """
        lim = self._motor_limits
        s = self.arm.state
        if shoulder_pan  is not None:
            s.base_yaw_deg = _clamp(shoulder_pan,  *lim["shoulder_pan"])
        if shoulder_lift is not None:
            s.theta1_deg   = _clamp(shoulder_lift, *lim["shoulder_lift"])
        if elbow_flex    is not None:
            s.theta2_deg   = _clamp(elbow_flex,    *lim["elbow_flex"])
        if wrist_flex    is not None:
            s.theta3_deg   = _clamp(wrist_flex,    *lim["wrist_flex"])
        if wrist_roll    is not None:
            self._wrist_roll = _clamp(wrist_roll,  *lim["wrist_roll"])
        if gripper       is not None:
            self._gripper    = _clamp(gripper,     *lim["gripper"])
        self.send_state()

    def set_gripper(self, value: float) -> None:
        """Set gripper value (clamped to calibration limits) and send."""
        self._gripper = _clamp(value, *self._motor_limits["gripper"])
        self.send_state()

    def set_wrist_roll(self, value: float) -> None:
        """Set wrist roll angle (degrees, clamped to calibration limits) and send."""
        self._wrist_roll = _clamp(value, *self._motor_limits["wrist_roll"])
        self.send_state()

    # .......................  Queries / FK  ...........................

    def end_effector_pose(self) -> Tuple[float, float, float, float]:
        """
        Compute forward kinematics for the current internal state.

        Returns:
            (x, y, z, phi_deg) in cm / degrees.
        """
        s = self.arm.state
        return self.arm.forward_kinematics(
            s.base_yaw_deg,
            s.theta1_deg,
            s.theta2_deg,
            s.theta3_deg,
        )

    @property
    def state(self) -> RobotState:
        """Current internal RobotState (angles in degrees)."""
        return self.arm.state

    @property
    def wrist_roll(self) -> float:
        return self._wrist_roll

    @property
    def gripper(self) -> float:
        return self._gripper


# ---------------------------------------------------------------------------
# CLI smoke-test
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="RobotArmController smoke test")
    ap.add_argument("--port", default=None, help="Serial port")
    args = ap.parse_args()

    with RobotArmController.from_port(port=args.port) as robot:
        print("Connected.")
        x, y, z, phi = robot.end_effector_pose()
        print(f"Current end-effector: x={x:.2f} y={y:.2f} z={z:.2f} phi={phi:.2f}")

        target = (30.0, 0.0, 20.0)
        print(f"\nMoving to xyz={target} ...")
        ok = robot.move_to_xyz(*target)
        print("Reached" if ok else "IK failed (unreachable)")

        x2, y2, z2, phi2 = robot.end_effector_pose()
        print(f"New end-effector:     x={x2:.2f} y={y2:.2f} z={z2:.2f} phi={phi2:.2f}")

    print("Disconnected.")


if __name__ == "__main__":
    main()