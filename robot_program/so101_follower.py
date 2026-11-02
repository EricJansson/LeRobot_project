"""Repository-owned SO101 motor tuning.

The configure sequence preserves the previously modified LeRobot 0.4.0
installation. Calibration must already match the arm; migration never starts
an interactive calibration workflow. Importing this module never opens a port.
"""

from contextlib import suppress
from dataclasses import replace
from pathlib import Path

from lerobot.motors.feetech import OperatingMode
from robot_program.lerobot_compat import SO101FollowerConfig, UpstreamSO101Follower


PID = {
    "shoulder_pan": (10, 0, 48),
    "shoulder_lift": (12, 0, 44),
    "elbow_flex": (14, 0, 40),
    "wrist_flex": (14, 0, 40),
    "wrist_roll": (16, 0, 36),
    "gripper": (18, 0, 32),
}


class CalibrationRequiredError(RuntimeError):
    """Existing calibration is missing, incomplete, or does not match the arm."""


class SO101Follower(UpstreamSO101Follower):
    """Tuned follower loading hardware calibration from the project by default."""

    name = "so101_follower"

    def __init__(self, config: SO101FollowerConfig):
        if config.calibration_dir is None:
            config = replace(
                config, calibration_dir=Path(__file__).resolve().parents[1] / "calibration"
            )
        super().__init__(config)

    def _calibration_error(self) -> CalibrationRequiredError:
        return CalibrationRequiredError(
            f"Missing, incomplete, or mismatched hardware calibration: {self.calibration_fpath}. "
            "Restore the authoritative calibration for robot ID "
            f"'{self.id}' without changing its values. Recalibration requires explicit approval; "
            "the repository degree-limit export is not a substitute."
        )

    def connect(self, calibrate: bool = True) -> None:
        if not calibrate:
            raise ValueError("Calibration verification cannot be skipped by this follower.")
        if set(self.calibration) != set(self.bus.motors):
            raise self._calibration_error()
        try:
            super().connect(calibrate=True)
        except CalibrationRequiredError:
            # Upstream has opened the bus to compare calibration. Close it
            # without changing torque or writing calibration on this failure.
            if self.bus.is_connected:
                with suppress(Exception):
                    self.bus.disconnect(disable_torque=False)
            raise

    def calibrate(self) -> None:
        # Upstream connect calls this on a mismatch, before configure().
        raise self._calibration_error()

    def configure(self) -> None:
        with self.bus.torque_disabled():
            # Preserve the 0.4 configuration sequence. Newer upstream helpers
            # also inspect/change Phase, which is outside this migration.
            for motor in self.bus.motors:
                self.bus.write("Return_Delay_Time", motor, 0)
                if self.bus.protocol_version == 0:
                    self.bus.write("Maximum_Acceleration", motor, 254)
                self.bus.write("Acceleration", motor, 254)
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
