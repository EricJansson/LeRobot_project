"""Migration checks use fake robots/buses and never open a serial device."""

import builtins
import importlib
import json
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

import pytest

pytest.importorskip("lerobot")

from lerobot.motors.feetech import FeetechMotorsBus
from robot_program import so101_follower as follower
from robot_program.lerobot_compat import SO101FollowerConfig
from robot_program.utils import ports

ROOT = Path(__file__).resolve().parents[1]


def forbidden(*args, **kwargs):
    raise AssertionError("Unexpected hardware or dashboard access")


@pytest.fixture(autouse=True)
def block_hardware(monkeypatch):
    import serial
    monkeypatch.setattr(FeetechMotorsBus, "connect", forbidden)
    monkeypatch.setattr(serial.Serial, "open", forbidden)


@pytest.mark.parametrize("layout", ["old", "new", "missing_dependency"])
def test_import_layouts(monkeypatch, layout):
    original = builtins.__import__
    upstream, config = object(), object()
    requested = []

    def fake_import(name, *args, **kwargs):
        if name.startswith("lerobot.robots.so101_follower"):
            requested.append(name)
            if layout != "old":
                missing = "unrelated_dependency" if layout == "missing_dependency" else "lerobot.robots.so101_follower"
                raise ModuleNotFoundError(name=missing)
            return SimpleNamespace(SO101Follower=upstream, SO101FollowerConfig=config)
        if name == "lerobot.robots.so_follower":
            requested.append(name)
            return SimpleNamespace(SO101Follower=upstream, SO101FollowerConfig=config)
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    if layout == "missing_dependency":
        with pytest.raises(ModuleNotFoundError) as error:
            runpy.run_path(str(ROOT / "robot_program/lerobot_compat.py"))
        assert error.value.name == "unrelated_dependency"
        assert "lerobot.robots.so_follower" not in requested
    else:
        result = runpy.run_path(str(ROOT / "robot_program/lerobot_compat.py"))
        assert result["UpstreamSO101Follower"] is upstream
        assert result["SO101FollowerConfig"] is config
        assert ("lerobot.robots.so_follower" in requested) == (layout == "new")


def make_follower(monkeypatch, tmp_path):
    import lerobot.robots.robot as robot_module
    monkeypatch.setattr(robot_module, "HF_LEROBOT_CALIBRATION", tmp_path)
    return follower.SO101Follower(SO101FollowerConfig(
        port="/dev/fake", id="lerobot_arm", use_degrees=True, calibration_dir=tmp_path,
    ))


def test_identity_motor_ids_and_units_preserved(monkeypatch, tmp_path):
    robot = make_follower(monkeypatch, tmp_path)
    assert robot.calibration_fpath == tmp_path / "lerobot_arm.json"
    assert {name: motor.id for name, motor in robot.bus.motors.items()} == {
        "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
        "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
    }
    assert {motor.norm_mode.value for name, motor in robot.bus.motors.items()
            if name != "gripper"} == {"degrees"}
    assert robot.bus.motors["gripper"].norm_mode.value == "range_0_100"
    assert robot.cameras == {}


def test_default_calibration_comes_from_project_not_cache_or_cwd(monkeypatch, tmp_path):
    import lerobot.robots.robot as robot_module
    monkeypatch.setattr(robot_module, "HF_LEROBOT_CALIBRATION", tmp_path / "unused-cache")
    monkeypatch.chdir(tmp_path)
    config = SO101FollowerConfig(port="/dev/fake", id="lerobot_arm", use_degrees=True)
    robot = follower.SO101Follower(config)
    assert robot.calibration_fpath == ROOT / "calibration/lerobot_arm.json"
    assert config.calibration_dir is None  # Do not mutate the caller's config.
    assert not (tmp_path / "unused-cache").exists()
    raw = json.loads(robot.calibration_fpath.read_text())
    for name, values in raw.items():
        for key, value in values.items():
            assert getattr(robot.calibration[name], key) == value


def test_degree_export_matches_project_hardware_calibration():
    raw = json.loads((ROOT / "calibration/lerobot_arm.json").read_text())
    exported = json.loads((ROOT / "calibration/lerobot_arm_with_degrees.json").read_text())
    motors = {motor["name"]: motor for motor in exported["motors"]}
    assert set(raw) == set(motors)
    for name, values in raw.items():
        assert all(motors[name][key] == value for key, value in values.items())
        for endpoint in ("min", "max"):
            assert motors[name][f"degree_{endpoint}"] == pytest.approx(
                (values[f"range_{endpoint}"] - 2048) / 4096 * 360, abs=0.01,
            )


@pytest.mark.parametrize("incomplete", [False, True])
def test_missing_calibration_rejected_before_connect(monkeypatch, tmp_path, incomplete):
    robot = make_follower(monkeypatch, tmp_path)
    if incomplete:
        robot.calibration = {"shoulder_pan": object()}
    with pytest.raises(follower.CalibrationRequiredError, match="explicit approval"):
        robot.connect()
    assert not robot.calibration_fpath.exists()


def test_mismatched_calibration_closes_bus_without_writes(monkeypatch, tmp_path):
    robot = make_follower(monkeypatch, tmp_path)
    robot.calibration = dict.fromkeys(robot.bus.motors, object())
    robot.bus = MagicMock(is_connected=True, motors=robot.bus.motors)

    def upstream_connect(self, calibrate=True):
        assert calibrate
        self.calibrate()

    monkeypatch.setattr(follower.UpstreamSO101Follower, "connect", upstream_connect)
    with pytest.raises(follower.CalibrationRequiredError, match="mismatched"):
        robot.connect()
    robot.bus.disconnect.assert_called_once_with(disable_torque=False)
    robot.bus.write.assert_not_called()
    robot.bus.write_calibration.assert_not_called()
    robot.bus.enable_torque.assert_not_called()
    robot.bus.disable_torque.assert_not_called()
    # Avoid the upstream destructor treating this mock as still connected.
    robot.bus.is_connected = False


def test_matching_calibration_delegates_connection(monkeypatch, tmp_path):
    robot = make_follower(monkeypatch, tmp_path)
    robot.calibration = dict.fromkeys(robot.bus.motors, object())
    connect = Mock()
    monkeypatch.setattr(follower.UpstreamSO101Follower, "connect", connect)
    robot.connect()
    connect.assert_called_once_with(calibrate=True)
    with pytest.raises(ValueError, match="cannot be skipped"):
        robot.connect(calibrate=False)


@pytest.mark.parametrize("mode", ["ik", "plan"])
@pytest.mark.parametrize("dashboard, platform", [(False, "linux"), (True, "linux"), (True, "win32")])
@pytest.mark.parametrize("loop_failure", [False, True])
def test_launchers_preserve_dashboard_default_and_allow_headless(monkeypatch, tmp_path, mode, dashboard, platform, loop_failure):
    module = importlib.import_module(f"scripts.main_teleop_{mode}")
    argv = ["teleop", "--port", "/dev/fake"] + ([] if dashboard else ["--no-dashboard"])
    monkeypatch.setattr(sys, "argv", argv)
    monkeypatch.setattr(module, "sys", SimpleNamespace(platform=platform, executable=sys.executable))
    monkeypatch.setattr(module, "Path", lambda path: tmp_path / Path(path).name)
    for key in ("default_command_file", "default_telemetry_file", "default_plan_cmd_file"):
        if hasattr(module, key):
            monkeypatch.setattr(module, key, (lambda key=key: tmp_path / key) if dashboard else forbidden)
    context = MagicMock()
    monkeypatch.setattr(module.RobotArmController, "from_port", Mock(return_value=context))
    monkeypatch.setattr(module.TeleopState, "from_robot", Mock(return_value=module.TeleopState()))
    monkeypatch.setattr(module, "setup_gamepad", Mock(return_value=MagicMock()))
    if mode == "ik":
        monkeypatch.setattr(module, "setup_teleop_bindings", Mock())
    process = MagicMock(pid=123)
    process.poll.return_value = None
    popen = Mock(return_value=process) if dashboard else Mock(side_effect=forbidden)
    monkeypatch.setattr(module.subprocess, "Popen", popen)
    loop = Mock(side_effect=RuntimeError("loop failed") if loop_failure else None)
    monkeypatch.setattr(module, f"run_{mode}_teleop_loop", loop)
    if loop_failure:
        with pytest.raises(RuntimeError, match="loop failed"):
            module.main()
    else:
        module.main()
    assert loop.call_args.kwargs["dashboard_enabled"] is dashboard
    if dashboard:
        assert loop.call_args.kwargs["dashboard_alive"]()
        assert popen.call_args.args[0][0] == sys.executable
        assert popen.call_args.kwargs["creationflags"] == (0x08000000 if platform == "win32" else 0)
        process.terminate.assert_called_once()
        process.wait.assert_called_once_with(timeout=3)
    else:
        popen.assert_not_called()
        assert loop.call_args.kwargs["dashboard_alive"] is None
        assert not list(tmp_path.iterdir())
    assert context.__exit__.call_count == 1
    assert context.__exit__.call_args.args[0] is (RuntimeError if loop_failure else None)


@pytest.mark.parametrize("mode", ["ik", "plan"])
def test_headless_loops_ignore_ipc_and_allow_gamepad_input(monkeypatch, mode):
    module = importlib.import_module(f"scripts.teleop_{mode}_control")
    for name in ("handle_dashboard_command", "push_telemetry", "handle_plan_dashboard_cmd",
                 "push_plan_telemetry", "_pending_manual_cmd"):
        if hasattr(module, name):
            monkeypatch.setattr(module, name, forbidden)
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    drive = Mock(return_value=True)
    monkeypatch.setattr(module, "apply_controller_input", drive)
    gp = MagicMock()
    gp.step.side_effect = [True, False]
    args = [MagicMock(), module.TeleopState(), gp, {"LB"}, {}, [0, 0], None, None]
    if mode == "ik":
        monkeypatch.setattr(module, "_print_status", Mock())
        monkeypatch.setattr(module, "motor_degrees", Mock(return_value={}))
    else:
        args.extend([None, module.Plan(), None])
    getattr(module, f"run_{mode}_teleop_loop")(
        *args, dashboard_alive=forbidden, dashboard_enabled=False,
    )
    drive.assert_called_once()
    assert gp.step.call_count == 2


def test_headless_plan_records_plays_and_cancels(monkeypatch):
    from scripts import teleop_plan_control as module
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    for name in ("handle_dashboard_command", "handle_plan_dashboard_cmd",
                 "push_plan_telemetry", "_pending_manual_cmd"):
        monkeypatch.setattr(module, name, forbidden)
    action = module.PlanAction(joints={"shoulder_pan": 10})
    snapshot = Mock(return_value=action)
    tick = Mock(return_value=True)
    cancel = Mock()
    monkeypatch.setattr(module, "snapshot_plan_action", snapshot)
    monkeypatch.setattr(module, "_tick_playback", tick)
    monkeypatch.setattr(module, "_cancel_playback", cancel)

    class Gamepad:
        def set_bindings(self, bindings, profile):
            self.bindings = bindings

        def switch_profile(self, profile):
            pass

        def step(self):
            try:
                presses, releases = next(events)
            except StopIteration:
                return False
            for name in releases:
                self.bindings.buttons_up[name](name, False)
            for name in presses:
                self.bindings.buttons_down[name](name, True)
            return True

    events = iter([(["RB", "Cross"], []), (["LB", "Start"], ["RB", "Cross"]),
                   (["Back"], ["LB", "Start"])])
    plan = module.Plan()
    module.run_plan_teleop_loop(
        MagicMock(), module.TeleopState(), Gamepad(), set(), {}, [0, 0],
        None, None, None, plan, None, dashboard_enabled=False,
    )
    assert plan.actions == [action]
    snapshot.assert_called_once()
    tick.assert_called_once()
    cancel.assert_called_once()


@pytest.mark.parametrize("system, raw, expected", [
    ("Windows", " com3 ", "COM3"),
    ("Windows", "com12", r"\\.\COM12"),
    ("Linux", "/dev/serial/by-id/robot", "/dev/serial/by-id/robot"),
])
def test_port_compatibility(monkeypatch, system, raw, expected):
    monkeypatch.setattr(ports.platform, "system", lambda: system)
    assert ports.normalize_port(raw) == expected
