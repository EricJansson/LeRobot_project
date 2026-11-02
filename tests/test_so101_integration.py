"""Offline follower and entry-point checks; no attached devices required."""

from contextlib import contextmanager
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

# Pure mathematics tests can still run without the optional hardware runtime.
pytest.importorskip("lerobot")

from lerobot.motors.feetech import OperatingMode
from robot_program import so101_follower as integration


ROOT = Path(__file__).resolve().parents[1]


class RecordingBus:
    def __init__(self, fail=False):
        self.motors = dict.fromkeys(integration.PID)
        self.protocol_version = 0
        self.events = []
        self.fail = fail

    @contextmanager
    def torque_disabled(self):
        self.events.append("torque context entered")
        try:
            yield
        finally:
            self.events.append("torque context exited")

    def configure_motors(self):
        raise AssertionError("Upstream configuration must not add new register writes")

    def write(self, register, motor, value):
        self.events.append((register, motor, value))
        if self.fail:
            raise OSError("simulated bus failure")


def test_configuration_preserves_motor_tuning():
    bus = RecordingBus()
    integration.SO101Follower.configure(SimpleNamespace(bus=bus))
    expected = ["torque context entered"]
    tuning = {
        "shoulder_pan": (10, 0, 48), "shoulder_lift": (12, 0, 44),
        "elbow_flex": (14, 0, 40), "wrist_flex": (14, 0, 40),
        "wrist_roll": (16, 0, 36), "gripper": (18, 0, 32),
    }
    for motor in tuning:
        expected.extend([("Return_Delay_Time", motor, 0),
                         ("Maximum_Acceleration", motor, 254), ("Acceleration", motor, 254)])
    for motor, (p, i, d) in tuning.items():
        expected.extend([("Operating_Mode", motor, OperatingMode.POSITION.value),
                         ("P_Coefficient", motor, p), ("I_Coefficient", motor, i),
                         ("D_Coefficient", motor, d)])
        if motor == "gripper":
            expected.extend([("Max_Torque_Limit", motor, 500),
                             ("Protection_Current", motor, 250), ("Overload_Torque", motor, 25)])
    expected.append("torque context exited")
    assert bus.events == expected


def test_configuration_failure_exits_torque_context():
    bus = RecordingBus(fail=True)
    with pytest.raises(OSError, match="simulated"):
        integration.SO101Follower.configure(SimpleNamespace(bus=bus))
    assert bus.events[-1] == "torque context exited"


@pytest.mark.parametrize("module", [
    "scripts.teleop_axis_control", "scripts.main_teleop_ik", "scripts.main_teleop_plan",
    "scripts.controller_dashboard", "robot_program.read_motors",
    "robot_program.keyboard_control", "robot_program.input_control",
])
def test_cli_help_without_hardware(module):
    code = """
import runpy, sys
from lerobot.motors.feetech import FeetechMotorsBus
import serial
def forbidden(*args, **kwargs):
    raise AssertionError('CLI help must not open hardware')
FeetechMotorsBus.connect = forbidden
serial.Serial.open = forbidden
sys.modules['keyboard'] = None
sys.argv = [sys.argv[1], '--help']
runpy.run_module(sys.argv[0], run_name='__main__')
"""
    result = subprocess.run([sys.executable, "-c", code, module], cwd=ROOT,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout.lower()
