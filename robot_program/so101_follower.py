"""Repository-owned SO101 motor tuning.

The configure sequence preserves the previously modified LeRobot 0.4.0
installation. All connection, calibration, action, and disconnect behavior
remains inherited from upstream. Importing this module never opens a port.
"""

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.so101_follower.so101_follower import SO101Follower as UpstreamSO101Follower


PID = {
    "shoulder_pan": (10, 0, 48),
    "shoulder_lift": (12, 0, 44),
    "elbow_flex": (14, 0, 40),
    "wrist_flex": (14, 0, 40),
    "wrist_roll": (16, 0, 36),
    "gripper": (18, 0, 32),
}


class SO101Follower(UpstreamSO101Follower):
    """Tuned follower with upstream calibration and connection behavior."""

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                for register, value in zip(
                    ("P_Coefficient", "I_Coefficient", "D_Coefficient"), PID[motor]
                ):
                    self.bus.write(register, motor, value)
                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, 500)
                    self.bus.write("Protection_Current", motor, 250)
                    self.bus.write("Overload_Torque", motor, 25)
