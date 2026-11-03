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


@pytest.mark.parametrize('bad', [np.full(5, np.nan), np.zeros(6), np.array([500, 0, 0, 0, 0]), np.array([6, 0, 0, 0, 0])])
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


def test_joint_step_rejection_keeps_previous_goal_until_joints_catch_up(controller):
    target = np.eye(4)
    target[0, 3] = 0.004
    assert controller.move_to_pose(target).accepted
    controller.follower.send_action.reset_mock()
    target[0, 3] = 0.006
    for _ in range(3):
        assert not controller.move_to_pose(target).accepted
        assert controller.target_pose[0, 3] == 0.004
    controller.follower.send_action.assert_not_called()
    controller.follower.bus.sync_read.return_value['shoulder_pan'] = 3
    assert controller.move_to_pose(target).accepted
    assert controller.target_pose[0, 3] == 0.006


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


def test_fine_pitch_roll_and_gripper(controller):
    state = teleop.InputState(active=True, was_active=True, hat=(0, 1))
    state.axes.update(RX=1, RT=1)
    controller.move_to_pose = Mock()
    teleop.tick(controller, state)
    c, s = np.cos(np.deg2rad(0.4)), np.sin(np.deg2rad(0.4))
    expected = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    np.testing.assert_allclose(controller.move_to_pose.call_args.args[0][:3, :3], expected)
    assert controller.move_to_pose.call_args.kwargs['gripper_delta'] == 0.7


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


def test_real_unequal_tracking_does_not_ratchet_pitch(real_controller):
    r = real_controller
    measured = np.array([5.36, -77.63, 46.02, 49.41, -9.63])
    goal = measured.copy()
    rate = np.array([2, 0.3, 0.3, 2, 2])
    def read(*args, **kwargs):
        return {**dict(zip(ARM_JOINTS, measured)), 'gripper': 20.0}
    def send(action):
        for i, name in enumerate(ARM_JOINTS):
            if name + '.pos' in action:
                goal[i] = action[name + '.pos']
        return action
    r.follower.bus.sync_read.side_effect = read
    r.follower.send_action.side_effect = send
    start = r.sync_from_hardware()
    state = teleop.InputState(active=True, was_active=True)
    state.axes['RY'] = -1
    rejected = 0
    for _ in range(50):
        result = teleop.tick(r, state)
        rejected += not result.accepted
        measured += np.clip(goal - measured, -rate, rate)
    for _ in range(100):
        measured += np.clip(goal - measured, -rate, rate)
    end = r.kinematics.forward_kinematics(measured).copy()
    angle = np.arccos(np.clip((np.trace(start[:3, :3].T @ end[:3, :3]) - 1) / 2, -1, 1))
    assert rejected > 0  # Exercise step-cap recovery with assumed, unequal rates.
    assert np.rad2deg(angle) < 0.1
    assert end[2, 3] > start[2, 3] + 0.015


@pytest.mark.parametrize("enabled", [False, True])
def test_dashboard_samples_on_teleop_loop_only(controller, monkeypatch, enabled):
    gp = SimpleNamespace(step=Mock(side_effect=[True, True, True, False]))
    dashboard = Mock() if enabled else None
    clock = iter([0, 0, 0.05, 0.05, 0.11, 0.11, 0.15])
    monkeypatch.setattr(teleop, "perf_counter", lambda: next(clock))
    monkeypatch.setattr(teleop, "sleep", Mock())
    controller.follower.bus.sync_read.reset_mock()
    joints = dict(zip(ARM_JOINTS, [10.0, -20.0, 30.0, 0.0, 0.0]))
    controller.follower.bus.sync_read.return_value = {**joints, "gripper": 20.0}
    target = controller.target_pose.copy()
    teleop.run_loop(controller, gp, teleop.InputState(), dashboard)
    np.testing.assert_array_equal(controller.target_pose, target)
    controller.follower.send_action.assert_not_called()
    assert controller.follower.bus.sync_read.call_count == (2 if enabled else 0)
    if enabled:
        assert dashboard.publish.call_count == 2
        dashboard.publish.assert_called_with(joints, 20.0, {"x": 0.01, "y": -0.02, "z": 0.03}, False)


@pytest.mark.parametrize("mode", ["headless", "normal", "loop_error", "bind_error"])
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
    monkeypatch.setattr(sys, "argv", ["teleop"] + (["--no-web-dashboard"] if mode == "headless" else []))
    if mode.endswith("_error"):
        with pytest.raises(SystemExit, match="1"):
            teleop.main()
    else:
        teleop.main()
    if mode == "headless":
        assert events == ["connect", "loop", "disconnect"]
    elif mode == "bind_error":
        assert events == ["bind"]
    else:
        assert events == ["bind", "connect", "loop", "disconnect", "close"]
