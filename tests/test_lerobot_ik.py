"""Offline regression tests for the LeRobot control path; serial is forbidden."""

import builtins
from contextlib import contextmanager
from dataclasses import replace
from importlib.metadata import version
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

import numpy as np
import pytest

pytest.importorskip('lerobot')
from lerobot.motors.feetech import FeetechMotorsBus
from robot_program.lerobot_compat import SO101FollowerConfig
from robot_program.so101_follower import SO101Follower
from robot_program.lerobot_ik_controller import ARM_JOINTS, MAX_ITERATIONS, LeRobotIKController
from scripts import main_teleop_lerobot_ik as teleop

pytestmark = pytest.mark.skipif(version('lerobot') != '0.6.1', reason='Pi IK targets LeRobot 0.6.1')


@pytest.fixture(autouse=True)
def no_serial(monkeypatch):
    import serial
    def forbidden(*args, **kwargs):
        raise AssertionError('Tests must not open serial devices')
    monkeypatch.setattr(FeetechMotorsBus, 'connect', forbidden)
    monkeypatch.setattr(serial.Serial, 'open', forbidden)


@pytest.fixture
def follower():
    robot = SO101Follower(SO101FollowerConfig(port='/dev/forbidden', id='lerobot_arm', use_degrees=True))
    robot.send_action = Mock(side_effect=lambda action: dict(action))
    robot.bus.sync_read = Mock(return_value={**dict.fromkeys(ARM_JOINTS, 0.0), 'gripper': 20.0})
    return robot


class FakeKinematics:
    joint_names = list(ARM_JOINTS)

    def __init__(self):
        self.robot = SimpleNamespace(set_joint_limits=Mock())
        self.solver = SimpleNamespace(mask_dof=Mock(), enable_joint_limits=Mock())
        self.inverse_kinematics = Mock(side_effect=lambda q, target, **kw: np.r_[target[:3, 3] * 1000, q[3:]])

    def forward_kinematics(self, q):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(q)[:3] / 1000
        return pose


@pytest.fixture
def controller(follower):
    robot = LeRobotIKController(follower, kinematics=FakeKinematics())
    robot.sync_from_hardware()
    return robot


def test_limits_use_bus_calibration_and_explicit_joint_order(controller):
    bus = controller.follower.bus
    bus.calibration['shoulder_pan'] = replace(bus.calibration['shoulder_pan'], range_min=1600, range_max=2400)
    narrowed = LeRobotIKController(controller.follower, kinematics=FakeKinematics())
    assert narrowed.limits_deg[0] == pytest.approx(np.array([-400, 400]) * 360 / 4095)
    assert narrowed.limits_deg[4] == pytest.approx(np.rad2deg([-2.74385, 2.84121]))
    bus.sync_read.return_value = dict(zip(reversed((*ARM_JOINTS, 'gripper')), [20, 5, 4, 3, 2, 1]))
    np.testing.assert_equal(controller._read_joints(), [1, 2, 3, 4, 5])


@pytest.mark.parametrize('bad', [np.full(5, np.nan), np.zeros(6), np.array([500, 0, 0, 0, 0])])
def test_invalid_solutions_never_send(controller, bad):
    controller.kinematics.inverse_kinematics.side_effect = lambda *args, **kw: bad
    result = controller.move_to_pose(np.eye(4), gripper_delta=2)
    assert not result.accepted
    controller.follower.send_action.assert_not_called()
    np.testing.assert_equal(controller.target_pose, np.eye(4))


def test_unreachable_target_has_bounded_iterations(controller):
    controller.kinematics.inverse_kinematics.side_effect = lambda q, *args, **kw: q
    target = np.eye(4)
    target[0, 3] = 10
    assert not controller.move_to_pose(target).accepted
    assert controller.kinematics.inverse_kinematics.call_count == MAX_ITERATIONS
    controller.follower.send_action.assert_not_called()


def test_converged_solution_over_five_degrees_is_sent_in_full(controller):
    solved_joints = np.array([8.0, -4.0, 2.0, 0.0, 0.0])
    controller.kinematics.inverse_kinematics.side_effect = [solved_joints * 2, solved_joints]
    target = np.eye(4)
    target[:3, 3] = solved_joints[:3] / 1000
    result = controller.move_to_pose(target)
    assert result.accepted
    assert controller.kinematics.inverse_kinematics.call_count == 2
    np.testing.assert_allclose(result.joints_deg, solved_joints)
    np.testing.assert_allclose(result.pose, target)
    np.testing.assert_allclose(controller.target_pose, result.pose)
    controller.follower.send_action.assert_called_once_with({
        f'{name}.pos': value for name, value in zip(ARM_JOINTS, solved_joints)
    })


def test_gripper_is_normalized_and_bypasses_ik(controller):
    controller.follower.bus.sync_read.return_value['gripper'] = 49
    controller.move_gripper(0.7)
    controller.move_gripper(2)
    assert controller.follower.send_action.call_args_list[0].args[0] == {'gripper.pos': 49.7}
    assert controller.follower.send_action.call_args.args[0] == {'gripper.pos': 50.0}
    controller.kinematics.inverse_kinematics.assert_not_called()


def test_neutral_and_released_shoulder_send_nothing(controller):
    state = teleop.InputState()
    state.axes['RY'] = -1
    teleop.tick(controller, state)
    state.axes['RY'] = 0
    state.active = True
    teleop.tick(controller, state)
    state.active = False
    teleop.tick(controller, state)
    controller.follower.send_action.assert_not_called()
    controller.follower.bus.sync_read.return_value['shoulder_pan'] = 20
    state.active = True
    teleop.tick(controller, state)
    assert controller.target_pose[0, 3] == 0.020


@pytest.mark.parametrize('yaw_deg', [0, 90, -90, 180])
def test_fine_translation_stays_fixed_to_base(controller, yaw_deg):
    a = np.deg2rad(yaw_deg)
    controller.target_pose[:3, :3] = [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]
    state = teleop.InputState(active=True, was_active=True)
    state.axes.update(LX=1, LY=-1, RY=-1)
    controller.move_to_pose = Mock()
    teleop.tick(controller, state)
    target = controller.move_to_pose.call_args.args[0]
    np.testing.assert_allclose(target[:3, 3], [0.002, -0.002, 0.002])
    np.testing.assert_allclose(target[:3, :3], controller.target_pose[:3, :3])


def test_pitch_roll_and_gripper_increments(controller):
    state = teleop.InputState(active=True, was_active=True, hat=(0, 1))
    state.axes.update(RX=1, RT=1)
    controller.move_to_pose = Mock()
    teleop.tick(controller, state)
    pitch, roll = np.deg2rad([0.4, 0.75])
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    expected = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([
        [cr, -sr, 0], [sr, cr, 0], [0, 0, 1],
    ])
    np.testing.assert_allclose(controller.move_to_pose.call_args.args[0][:3, :3], expected)
    assert controller.move_to_pose.call_args.kwargs['gripper_delta'] == 1.0


@pytest.mark.parametrize('stop', ['disconnect', 'interrupt', 'read_error'])
def test_connection_cleanup(controller, monkeypatch, stop):
    events = []
    follower = controller.follower
    monkeypatch.setattr(type(follower.bus), 'is_connected', property(lambda _: True))
    follower.connect = lambda: events.append('connect')
    follower.disconnect = lambda: events.append('disconnect')
    follower.bus.enable_torque = lambda: events.append('enable')
    follower.bus.disable_torque = lambda: events.append('disable')
    if stop == 'read_error':
        follower.bus.sync_read.side_effect = OSError('read failed')
        with pytest.raises(OSError):
            controller.connect()
    else:
        gp = SimpleNamespace(step=Mock(side_effect=KeyboardInterrupt) if stop == 'interrupt' else Mock(return_value=False))
        try:
            with controller:
                teleop.run_loop(controller, gp, teleop.InputState())
        except KeyboardInterrupt:
            pass
    assert events == ['connect', 'enable', 'disable', 'disconnect']
    follower.send_action.assert_not_called()


def test_missing_placo_fails_before_gamepad_or_serial(monkeypatch):
    original = builtins.__import__
    def importing(name, *args, **kwargs):
        if name == 'lerobot.model.kinematics':
            raise ImportError('placo unavailable')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', importing)
    monkeypatch.setattr(sys, 'argv', ['teleop'])
    gp = Mock()
    monkeypatch.setattr(teleop, 'setup_gamepad', gp)
    with pytest.raises(SystemExit) as exc:
        teleop.main()
    assert exc.value.code == 1
    gp.assert_not_called()


@pytest.fixture
def real_controller(follower):
    pytest.importorskip('placo')
    return LeRobotIKController(follower)


def test_real_fk_ik_roundtrips_steps_and_boundaries(real_controller):
    r = real_controller
    poses = [np.zeros(5), np.array([10., -15., 20., 10., -20.])]
    for joint in range(5):
        for bound in (0, 1):
            q = np.zeros(5)
            q[joint] = r.limits_deg[joint, bound]
            poses.append(q)
    for q in poses:
        target = r.kinematics.forward_kinematics(q).copy()
        result = r.solve_pose(target, q)
        assert result.accepted, result.reason
        np.testing.assert_allclose(result.pose, target, atol=0.0005)
    q = poses[1]
    for axis in range(3):
        for direction in (-1, 1):
            target = r.kinematics.forward_kinematics(q).copy()
            target[axis, 3] += direction * 0.0008
            result = r.solve_pose(target, q)
            assert result.accepted, result.reason
            assert np.linalg.norm(result.pose[:3, 3] - target[:3, 3]) <= 0.0005


def test_real_extended_retraction_keeps_sending_full_commands_with_stalled_feedback(real_controller):
    r = real_controller
    measured = np.array([0.0, 25.0, -12.0, 3.0, 0.0])
    r.follower.bus.sync_read.return_value.update(dict(zip(ARM_JOINTS, measured)))
    state = teleop.InputState(active=True)
    state.axes['LY'] = 1.0
    start = r.kinematics.forward_kinematics(measured)
    for _ in range(8):
        result = teleop.tick(r, state)
        assert result.accepted, result.reason
    assert r.follower.send_action.call_count == 8
    assert np.max(np.abs(result.joints_deg - measured)) > 5.0
    assert result.pose[0, 3] < start[0, 3] - 0.014
    r.follower.send_action.assert_called_with({
        f'{name}.pos': value for name, value in zip(ARM_JOINTS, result.joints_deg)
    })


def test_real_outward_rejections_allow_retraction_without_releasing_lb(real_controller):
    r = real_controller
    measured = np.array([0.0, 25.0, -12.0, 3.0, 0.0])
    r.follower.bus.sync_read.return_value.update(dict(zip(ARM_JOINTS, measured)))
    state = teleop.InputState(active=True)
    state.axes['LY'] = -1.0
    for _ in range(100):
        result = teleop.tick(r, state)
        if not result.accepted:
            break
        r.follower.bus.sync_read.return_value.update(dict(zip(ARM_JOINTS, result.joints_deg)))
    else:
        pytest.fail('Outward jog never reached the modeled boundary')

    boundary = r.target_pose.copy()
    r.follower.send_action.reset_mock()
    for _ in range(3):
        assert not teleop.tick(r, state).accepted
        np.testing.assert_allclose(r.target_pose, boundary)
    r.follower.send_action.assert_not_called()
    state.axes['LY'] = 1.0
    inward = teleop.tick(r, state)
    assert inward.accepted, inward.reason
    assert inward.pose[0, 3] < boundary[0, 3]
    r.follower.send_action.assert_called_once()


@pytest.mark.parametrize("enabled", [False, True])
def test_dashboard_samples_on_teleop_loop_only(controller, monkeypatch, enabled):
    gp = SimpleNamespace(
        step=Mock(side_effect=[True, True, True, False]), name="Fixture controller",
        num_axes=6, num_buttons=14, num_hats=1, deadzone=0.1,
    )
    dashboard = Mock() if enabled else None
    clock = iter([0, 0, 0.05, 0.05, 0.11, 0.11, 0.15])
    monkeypatch.setattr(teleop, "perf_counter", lambda: next(clock))
    monkeypatch.setattr(teleop, "sleep", Mock())
    controller.follower.bus.sync_read.reset_mock()
    joints = dict(zip(ARM_JOINTS, [10.0, -20.0, 30.0, 0.0, 0.0]))
    controller.follower.bus.sync_read.return_value = {**joints, "gripper": 20.0}
    target = controller.target_pose.copy()
    state = teleop.InputState()
    state.axes["LX"] = -0.5
    state.buttons["Cross"] = True
    state.hat = (-1, 1)
    teleop.run_loop(controller, gp, state, dashboard)
    np.testing.assert_array_equal(controller.target_pose, target)
    controller.follower.send_action.assert_not_called()
    assert controller.follower.bus.sync_read.call_count == (2 if enabled else 0)
    if enabled:
        assert dashboard.publish.call_count == 2
        dashboard.publish.assert_called_with(
            joints, 20.0, {"x": 0.01, "y": -0.02, "z": 0.03}, False,
            gamepad={
                "name": "Fixture controller", "axis_count": 6, "button_count": 14,
                "hat_count": 1, "deadzone": 0.1, "axes": state.axes,
                "buttons": state.buttons, "dpad": (-1, 1),
            },
        )


@pytest.mark.parametrize("mode", ["headless", "normal", "legacy_flag", "loop_error", "bind_error"])
def test_web_dashboard_launcher_lifecycle(monkeypatch, mode):
    events = []
    robot = Mock()
    robot_context = MagicMock()
    robot_context.__enter__.side_effect = lambda: events.append("connect")
    robot_context.__exit__.side_effect = lambda *_: events.append("disconnect")
    robot.from_port.return_value = robot_context
    monkeypatch.setattr(teleop, "LeRobotIKController", robot)
    monkeypatch.setattr(teleop, "setup_gamepad", Mock())

    @contextmanager
    def dashboard(host, port):
        assert (host, port) == ("0.0.0.0", 8000)
        events.append("bind")
        if mode == "bind_error":
            raise OSError("address in use")
        try:
            yield SimpleNamespace(port=port)
        finally:
            events.append("close")

    def loop(_robot, _gp, _state, web):
        assert (web is not None) == (mode != "headless")
        events.append("loop")
        if mode == "loop_error":
            raise OSError("loop failed")

    monkeypatch.setattr(teleop, "WebDashboard", dashboard)
    monkeypatch.setattr(teleop, "run_loop", loop)
    args = ["teleop"]
    if mode == "headless":
        args.append("--no-web-dashboard")
    if mode == "legacy_flag":
        args.append("--no-joint-step-limit")
    monkeypatch.setattr(sys, "argv", args)
    if mode.endswith("_error"):
        with pytest.raises(SystemExit, match="1"):
            teleop.main()
    else:
        teleop.main()
    robot.from_port.assert_called_once_with(None)
    if mode == "headless":
        assert events == ["connect", "loop", "disconnect"]
    elif mode == "bind_error":
        assert events == ["bind"]
    else:
        assert events == ["bind", "connect", "loop", "disconnect", "close"]
