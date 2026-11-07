import ctypes
from contextlib import nullcontext
import errno
from io import BytesIO
from importlib.metadata import version
import os
import shutil
import socket
import subprocess
import sys
import tarfile
from threading import Event, Thread
from time import sleep
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import webbrowser

import numpy as np
import pytest

from controller.input.xinput import XInputGamepad, XInputPad, XInputState
from robot_program.lerobot_ik_controller import DEFAULT_URDF, LeRobotPoseSolver
from robot_program.remote_ik_protocol import (
    CHANNELS, PROTOCOL_VERSION, calibrated_follower, channel_limits, kinematic_urdf,
)
from scripts import main_teleop_remote_ik as remote_teleop
from scripts.install_remote_ik_pi import bundle
from scripts.main_teleop_remote_ik import ControlLoop, LatestInput, MotorLink, PiTunnel
from scripts.remote_ik_motor_service import MotorSession, PiMotorBus, ProtocolError, serve
from scripts.remote_ik_worker import solve_intent


LIMITS = [[-90., 90.]] * 5 + [[0., 50.]]
TOKEN = "sample-token"


@pytest.mark.parametrize("options, expected_port", [
    ([], 18743), (["--no-browser"], 18743),
    (["--dashboard-port", "18744"], 18744), (["--dashboard-port", "0"], 0),
])
def test_launcher_dashboard_port_and_manual_browser(monkeypatch, capsys, options, expected_port):
    ports = []
    dashboards = []
    dashboard_type = remote_teleop.WebDashboard

    def dashboard(host, port):
        ports.append(port)
        server = dashboard_type(host, 0)
        dashboards.append(server)
        return server

    monkeypatch.setattr(remote_teleop, "WebDashboard", dashboard)
    monkeypatch.setattr(remote_teleop, "XInputGamepad", lambda index: None)
    monkeypatch.setattr(remote_teleop, "load_key_state", lambda: lambda key: -1)
    monkeypatch.setattr(remote_teleop, "Worker", lambda *args: nullcontext(
        SimpleNamespace(metadata={"model": "fixture", "limits": LIMITS})))
    monkeypatch.setattr(remote_teleop, "ControlLoop", lambda *args, **kwargs:
                        SimpleNamespace(run=lambda: None, error=None))
    monkeypatch.setattr(webbrowser, "open", lambda *args, **kwargs: pytest.fail("Browser opened automatically"))
    monkeypatch.setattr(sys, "argv", ["remote-ik", "--dry-run", *options])
    remote_teleop.main()
    assert ports == [expected_port]
    assert f"http://127.0.0.1:{dashboards[0].port}/" in capsys.readouterr().out


def test_launcher_busy_dashboard_port(monkeypatch, capsys):
    monkeypatch.setattr(remote_teleop, "XInputGamepad", lambda index: None)
    monkeypatch.setattr(remote_teleop, "load_key_state", lambda: lambda key: -1)
    monkeypatch.setattr(remote_teleop, "Worker", lambda *args: pytest.fail("Worker started with busy dashboard port"))
    with remote_teleop.WebDashboard("127.0.0.1", 0) as dashboard:
        monkeypatch.setattr(sys, "argv", ["remote-ik", "--dry-run", "--dashboard-port", str(dashboard.port)])
        with pytest.raises(SystemExit) as error:
            remote_teleop.main()
    assert error.value.code == 1
    message = capsys.readouterr().err
    assert "Cannot start dashboard" in message
    assert "--dashboard-port PORT" in message


class FakeBus:
    measured = [0.] * 6

    def __init__(self):
        self.writes = []

    def read(self):
        return self.measured.copy()

    def arm(self, target):
        return self.write(target)

    def write(self, target):
        self.writes.append(target.copy())
        return target.copy()


@pytest.fixture
def rig():
    now = [0.]
    bus = FakeBus()
    motor = MotorSession(bus, limits=LIMITS, model="fixture", token=TOKEN, watchdog=.15,
                         hz=50, arm_rate=100, gripper_rate=50, clock=lambda: now[0])
    return motor, bus, now


def hello(motor):
    return motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": TOKEN})


def command(motor, prior, op, **fields):
    return motor.tick({"v": PROTOCOL_VERSION, "op": op, "session": prior["session"],
                       "seq": prior["ack"] + 1, "feedback_seq": prior["feedback_seq"], **fields})


def test_urdf_and_install_bundle_use_only_kinematics():
    source = ET.parse(DEFAULT_URDF).getroot()
    stripped = ET.fromstring(kinematic_urdf())
    assert not any(stripped.findall(f".//{tag}") for tag in ("visual", "collision", "mesh"))
    for tag in ("joint", "transmission"):
        assert [ET.canonicalize(ET.tostring(node), strip_text=True) for node in source.findall(tag)] == [
            ET.canonicalize(ET.tostring(node), strip_text=True) for node in stripped.findall(tag)
        ]
    with tarfile.open(fileobj=BytesIO(bundle())) as archive:
        assert not any(name.startswith("assets/") for name in archive.getnames())
        assert archive.extractfile("so101_new_calib.urdf").read() == kinematic_urdf().encode()


def test_disarmed_bounds_and_separate_gripper_rate(rig):
    motor, bus, _ = rig
    feedback = hello(motor)
    target = [10., 0., 0., 0., 0., 5.]
    feedback = command(motor, feedback, "move", target=target)
    assert not bus.writes
    feedback = command(motor, feedback, "arm")
    assert feedback["armed"]
    feedback = command(motor, feedback, "move", target=[0.] * 5 + [55.])
    assert not feedback["moving"]
    feedback = command(motor, feedback, "move", target=target)
    assert feedback["sent"] == [2., 0., 0., 0., 0., 1.]
    assert feedback["measured"] == [0.] * 6


@pytest.mark.parametrize("failure", ["watchdog", "disconnect", "release", "stale_feedback", "duplicate"])
def test_stop_latches_measured_hold_once(rig, failure):
    motor, bus, now = rig
    feedback = command(motor, hello(motor), "arm")
    feedback = command(motor, feedback, "move", target=[1.] * 6)
    bus.measured = [.5] * 6
    if failure == "watchdog":
        now[0] += .16
        motor.tick()
    elif failure == "disconnect":
        motor.tick(lost=True)
    elif failure == "release":
        command(motor, feedback, "hold")
    elif failure == "stale_feedback":
        command(motor, feedback, "move", target=[1.] * 6, feedback_seq=feedback["feedback_seq"] - 1)
    else:
        command(motor, feedback, "move", target=[1.] * 6, seq=feedback["ack"])
    assert not motor.advancing
    assert motor.armed is (failure == "release")
    assert bus.writes[-1] == [.5] * 6
    bus.measured = [.4] * 6
    motor.tick()
    assert len(bus.writes) == 3


def test_expired_arm_cannot_rearm(rig):
    motor, bus, now = rig
    first = hello(motor)
    now[0] += .16
    feedback = command(motor, first, "arm")
    assert not feedback["armed"] and not bus.writes
    assert "stale" in feedback["reason"]


def test_physical_session_observes_without_writing_by_default():
    bus = FakeBus()
    motor = MotorSession(bus, limits=LIMITS, model="fixture", token=TOKEN, watchdog=.15,
                         hz=50, arm_rate=100, gripper_rate=50, simulated=False)
    feedback = hello(motor)
    assert not feedback["simulated"] and not feedback["motion_enabled"]
    assert feedback["measured"] == [0.] * 6 and feedback["read_max_ms"] >= 0
    for op in ("arm", "move", "hold", "disarm"):
        feedback = command(motor, feedback, op, target=[1.] * 6)
        assert not feedback["armed"] and not feedback["moving"]
        assert "Observe only" in feedback["reason"]
        assert feedback["sent"] is None and feedback["write_max_ms"] is None
    motor.tick(lost=True)
    assert not bus.writes


def test_pi_motor_bus_uses_normalized_channels_and_goals_before_torque():
    values = [12., -11., 10., -9., 8., 25.]
    positions = dict(zip(CHANNELS, values))
    calls = []
    bus = SimpleNamespace(sync_read=lambda register: positions if register == "Present_Position" else {},
                          sync_write=lambda register, targets: calls.append((register, targets)),
                          enable_torque=lambda *, num_retry: calls.append(("torque", num_retry)))
    adapter = PiMotorBus(SimpleNamespace(bus=bus), __file__)
    assert adapter.read() == values
    assert adapter.arm(values) == values
    assert calls == [("Goal_Position", positions), ("torque", 1)]


def test_failed_torque_enable_leaves_real_arm_disarmed():
    writes = []

    def enable_torque(*, num_retry):
        raise ConnectionError("Incorrect status packet")

    bus = SimpleNamespace(sync_read=lambda register: dict.fromkeys(CHANNELS, 0.),
                          sync_write=lambda register, targets: writes.append(targets),
                          enable_torque=enable_torque)
    adapter = PiMotorBus(SimpleNamespace(bus=bus), __file__)
    motor = MotorSession(adapter, limits=LIMITS, model="fixture", token=TOKEN, watchdog=.15,
                         hz=50, arm_rate=100, gripper_rate=50, simulated=False, allow_motion=True)
    feedback = command(motor, hello(motor), "arm")
    assert feedback["fault"] and "Incorrect status packet" in feedback["reason"]
    assert not feedback["armed"] and not adapter.torque_enabled
    feedback = command(motor, feedback, "move", target=[1.] * 6)
    assert feedback["fault"] and not feedback["moving"]
    assert writes == [dict.fromkeys(CHANNELS, 0.)]


def test_pi_motor_bus_attempts_to_relax_every_motor():
    calls = []

    def disable_torque(channel, *, num_retry):
        calls.append(channel)
        if channel == "shoulder_lift":
            raise OSError("no reply")

    adapter = PiMotorBus(SimpleNamespace(bus=SimpleNamespace(disable_torque=disable_torque)), __file__)
    with pytest.raises(RuntimeError, match="shoulder_lift: no reply"):
        adapter.relax()
    assert calls == list(CHANNELS)


@pytest.mark.parametrize(("allow_motion", "opt_in", "requested", "releases"), [
    (True, True, True, True),
    (True, True, False, False),
    (True, False, True, False),
    (False, True, True, False),
])
def test_torque_release_requires_opt_in_and_orderly_real_arm_shutdown(allow_motion, opt_in, requested, releases):
    bus = FakeBus()
    events = []
    bus.relax = lambda: events.append("torque released")
    motor = MotorSession(bus, limits=LIMITS, model="fixture", token=TOKEN, watchdog=.15,
                         hz=50, arm_rate=100, gripper_rate=50, simulated=False, allow_motion=allow_motion)
    feedback = hello(motor)
    if opt_in:
        feedback = command(motor, feedback, "relax_on_exit")
    assert feedback["relax_on_exit"] is (allow_motion and opt_in)
    assert not events

    if allow_motion:
        feedback = command(motor, feedback, "arm")
        command(motor, feedback, "move", target=[1.] * 6)
    motor.tick(lost=True)
    assert not events
    if allow_motion:
        assert bus.writes[-1] == [0.] * 6

    motor.shutdown(requested=requested)
    assert events == (["torque released"] if releases else [])


@pytest.mark.parametrize("failure", ["read", "write"])
def test_bus_error_latches_fault_and_prevents_fresh_arm(rig, monkeypatch, failure):
    motor, bus, _ = rig
    feedback = command(motor, hello(motor), "arm")
    feedback = command(motor, feedback, "move", target=[1.] * 6)

    def fail(*_):
        raise OSError("serial failure")

    with monkeypatch.context() as patch:
        patch.setattr(bus, failure, fail)
        feedback = command(motor, feedback, "move", target=[1.5] * 6)
        assert feedback["fault"] and not feedback["armed"]
    feedback = command(motor, feedback, "arm")
    assert feedback["fault"] and not feedback["armed"]
    assert feedback["sent"] == [0.] * 6
    assert bus.writes == [[0.] * 6, [1.] * 6, [0.] * 6]


def gamepad_input(*, lx=0., start=False, lb=False):
    return {"axes": {"LX": lx, "LY": 0., "RX": 0., "RY": 0., "LT": 0., "RT": 0.},
            "buttons": {"Start": start, "LB": lb, "Back": False}, "dpad": (0, 0)}


@pytest.mark.parametrize(("scenario", "expected", "mode"), [
    ("release during IK", "hold", "Real arm enabled"),
    ("brief neutral during IK", "hold", "Real arm enabled"),
    ("jaw replaces arm during IK", "hold", "Real arm enabled"),
    ("disconnect during IK", "disarm", "Real arm enabled"),
    ("reconnect while holding LB", "poll", "Real arm enabled"),
    ("resume after hold", "move", "Real arm enabled"),
    ("observe only", "poll", "Observe only"),
])
def test_control_loop_drops_stale_intent_and_reports_pi_mode(scenario, expected, mode):
    latest = LatestInput()
    latest.publish(gamepad_input(lx=.5, lb=True))
    stop = Event()
    operations = []
    moving = scenario.endswith("during IK")
    armed = moving or scenario == "resume after hold"
    feedback = {"simulated": False, "motion_enabled": scenario != "observe only",
                "armed": armed, "moving": moving, "measured": [0.] * 6, "hz": 50,
                "watchdog_s": .15, "sent": [2.] * 6 if armed else None, "reason": "", "fault": False}

    def solve(measured, delta, *, sent, desired_xyz_m):
        assert desired_xyz_m is None
        if moving:
            assert any(delta)
            assert sent == [2.] * 6
            if scenario == "jaw replaces arm during IK":
                jaw = gamepad_input(lb=True)
                jaw["dpad"] = (0, 1)
                latest.publish(jaw)
            else:
                latest.publish(None if scenario == "disconnect during IK"
                               else gamepad_input(lb=scenario == "brief neutral during IK"))
                latest.publish(gamepad_input(lx=.5, lb=True))
            return {"target": [1.] * 6}
        assert sent is None
        if armed:
            assert any(delta)
            return {"target": [1.] * 6}
        assert not any(delta)
        return {}

    def request(op, **values):
        operations.append(op)
        stop.set()
        return {**feedback, "moving": False, "armed": armed and op != "disarm"}

    control = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, .25, "Real arm")
    control.session(SimpleNamespace(feedback=feedback, request=request))
    assert operations == [expected]
    assert control.status["mode"] == mode


def test_arm_start_ramp_restarts_and_traces_issued_steps(monkeypatch):
    now = [0.]
    monkeypatch.setattr(remote_teleop, "monotonic", lambda: now[0])
    latest, stop, trace = LatestInput(), Event(), []
    measured = [0.] * 6
    feedback = {"armed": True, "moving": False, "measured": measured, "sent": measured,
                "hz": 50, "watchdog_s": .2, "fault": False, "reason": ""}
    # Time, arm input, jaw input, expected ramp, expected operation.
    steps = [
        (0., True, True, .1, "move"),
        (.15, True, True, .55, "move"),
        (.30, True, False, 1., "move"),
        (.32, False, True, None, "hold"),
        (.34, False, True, None, "move"),
        (.36, True, True, .1, "move"),
        (.38, True, False, None, "hold"),  # Neutral and re-press between control ticks.
        (.40, True, False, .1, "move"),
        (.42, True, False, .1 + .9 * (1 / 15) ** 2 * (3 - 2 / 15), "hold"),  # IK rejects.
        (.44, True, False, .1, "move"),
        (.46, True, False, .1 + .9 * (1 / 15) ** 2 * (3 - 2 / 15), "hold"),
        (.48, True, False, .1, "move"),
    ]
    index = 0

    def publish():
        at, arm, jaw, *_ = steps[index]
        now[0] = at
        if index == 6:
            # The input poller sees neutral, but the control loop only sees the next tap.
            now[0] -= remote_teleop.GAMEPAD_POLL_S
            latest.publish(gamepad_input(lb=True))
            now[0] = at
        sample = gamepad_input(lx=.5 if arm else 0., lb=True)
        sample["axes"]["LY"] = -1. if arm else 0.
        sample["dpad"] = (0, int(jaw))
        latest.publish(sample)

    def solve(measured, delta, *, sent, desired_xyz_m):
        if index == 8:
            return {"reason": "joint limit exceeded"}
        if not any(delta):
            return {}
        if index == 10:
            latest.publish(gamepad_input(lb=True))
            publish()
        if index in (7, 11):
            assert desired_xyz_m is None
        baseline = desired_xyz_m if desired_xyz_m is not None else [0.] * 3
        target = [prior + step * 1000 for prior, step in zip(feedback["sent"], delta)]
        return {"baseline_xyz_m": baseline, "requested_xyz_m": np.add(baseline, delta[:3]).tolist(),
                "target": target}

    def request(op, **values):
        nonlocal index
        assert op == steps[index][4]
        if op == "move":
            feedback.update(moving=True, sent=values["target"])
        elif op == "hold":
            feedback.update(moving=False, sent=measured)
        index += 1
        if index == len(steps):
            stop.set()
        else:
            publish()
        return feedback

    publish()
    control = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, 1., "Sim", trace=trace)
    control.session(SimpleNamespace(feedback=feedback, request=request))
    for tick, (_, _, _, scale, op) in zip(trace, steps, strict=True):
        assert tick["op"] == op
        if scale is None:
            assert tick["arm_scale"] is None
        else:
            assert tick["arm_scale"] == pytest.approx(scale)
            assert tick["delta"][0] == pytest.approx(-2 * tick["delta"][1])
    assert trace[0]["delta"][5] == trace[4]["delta"][5] == 1.
    assert trace[0]["issued"][0] - trace[0]["issued_before"][0] == pytest.approx(.2)
    assert trace[2]["issued"][0] - trace[2]["issued_before"][0] == pytest.approx(4.)
    assert trace[7]["issued"][0] - trace[7]["issued_before"][0] == pytest.approx(.2)
    assert trace[8]["ik_reason"] == "joint limit exceeded"
    for hold_index in (6, 10):
        hold, restart = trace[hold_index:hold_index + 2]
        assert hold["xyz_reset"] == "arm input neutral"
        assert hold["desired_before_m"] is not None
        assert hold["desired_xyz_m"] is restart["desired_before_m"] is None
        assert restart["arm_scale"] == pytest.approx(.1)


@pytest.mark.parametrize("delta", [
    [.001, 0., 0., 0., 0., 0.],
    [.001, -.002, 0., 0., 0., 0.],
    [.001, -.002, .003, 0., 0., 0.],
    [0., 0., 0., 1., -2., 0.],
])
@pytest.mark.parametrize("desired", [None, [.3, .2, .1]])
def test_worker_uses_cartesian_reference_independently_of_joint_seed(delta, desired):
    measured = [.1, .2, .3, 0., 0., 10.]
    issued = [.2, .3, .4, 0., 0., 12.]

    def forward(joints):
        pose = np.eye(4)
        pose[:3, 3] = joints[:3]
        return pose

    baseline = measured[:3] if desired is None else desired
    requested = np.add(baseline, delta[:3])

    def solve_pose(target, seed):
        np.testing.assert_allclose(seed, issued[:5])
        np.testing.assert_allclose(target[:3, 3], requested)
        if any(delta[3:5]):
            assert not np.allclose(target[:3, :3], np.eye(3))
        return SimpleNamespace(accepted=True, joints_deg=seed)

    solver = SimpleNamespace(kinematics=SimpleNamespace(forward_kinematics=forward), solve_pose=solve_pose)
    result = solve_intent(solver, measured, delta, LIMITS[5], sent=issued, desired_xyz_m=desired)
    assert result["baseline_xyz_m"] == baseline
    assert result["requested_xyz_m"] == pytest.approx(requested)
    assert result["target"] == issued
    jaw_only = solve_intent(solver, measured, [0., 0., 0., 0., 0., 1.], LIMITS[5],
                            sent=issued, desired_xyz_m=desired)
    assert jaw_only["target"] == [*issued[:5], 13.]


def test_control_loop_preserves_xyz_through_rejection_and_slew_until_input_stops(monkeypatch):
    now = [0.]
    monkeypatch.setattr(remote_teleop, "monotonic", lambda: now[0])
    latest = LatestInput()
    stop = Event()
    measured = [.1, .2, .3, 0., 0., 10.]
    target = [1., 2., 3., 0., 0., 10.]
    start, advanced, retracted = ([.1, .2, .3], [.101, .2, .3], [.1005, .2, .3])
    settled = [.1, .2, .295]
    feedback = {"simulated": True, "motion_enabled": True, "armed": True, "moving": False,
                "measured": measured, "hz": 50, "watchdog_s": .15, "sent": measured,
                "reason": "", "fault": False}
    # Time, X input, LB, desired seen by the arm solver, solve outcome, Pi operation.
    steps = [
        (0., 1., True, None, "full", "move"),
        (.02, 1., True, advanced, "reject", "hold"),
        (.04, 1., True, advanced, "reject", "poll"),
        (.06, -1., True, advanced, "reject", "poll"),
        (.08, -1., True, None, "slew", "move"),
        (.10, -1., True, start, "full", "move"),
        (.12, 0., True, None, None, "hold"),
        (.14, -1., True, None, "slew", "move"),
        (.16, -1., True, settled, "full", "move"),
        (.18, -1., False, None, None, "hold"),
        (.20, 1., True, None, "full", "move"),
    ]
    index = 0
    statuses = []

    def publish():
        at, x, lb, *_ = steps[index]
        now[0] = at
        gamepad = gamepad_input(lb=lb)
        gamepad["axes"]["LY"] = np.float64(-x)
        latest.publish(gamepad)

    def solve(measured, delta, *, sent, desired_xyz_m):
        at, x, _, expected_desired, outcome, _ = steps[index]
        assert desired_xyz_m == expected_desired
        if outcome is None:
            assert not any(delta)
            return {}
        assert np.sign(delta[0]) == np.sign(x)
        assert not any(delta[1:])
        baseline = measured[:3] if desired_xyz_m is None else desired_xyz_m
        if at == .14:
            assert baseline == settled and sent is None
        return {"xyz_m": measured[:3], "baseline_xyz_m": baseline,
                "requested_xyz_m": advanced if x > 0 else retracted,
                "target": None if outcome == "reject" else target,
                "reason": remote_teleop.POSITION_TOLERANCE_REASON if outcome == "reject" else ""}

    def request(op, **values):
        nonlocal index
        _, _, _, _, outcome, expected_op = steps[index]
        assert op == expected_op
        statuses.append(control.status.get("desired_xyz_m"))
        if op == "move":
            assert values["target"] == target
            issued = [*target[:5], 9.]
            issued[0] += -.5 if outcome == "slew" else 1e-10
            feedback.update(moving=True, sent=issued)
        elif op == "hold":
            feedback.update(moving=False, sent=measured)
            if now[0] == .12:
                feedback["measured"] = [*settled, 0., 0., 10.]
        index += 1
        if index == len(steps):
            stop.set()
        else:
            publish()
        return feedback

    publish()
    control = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, .25, "Sim")
    control.session(SimpleNamespace(feedback=feedback, request=request))
    assert statuses[1:] == [
        advanced, advanced, advanced, None, start, retracted, None, settled, retracted, None,
    ]
    assert control.status["desired_xyz_m"] == advanced


@pytest.mark.skipif(version("lerobot") != "0.6.1", reason="Remote IK targets LeRobot 0.6.1")
def test_small_sustained_ik_input_survives_encoder_quantization(tmp_path):
    follower = calibrated_follower("unused", DEFAULT_URDF.parent / "calibration")
    limits = channel_limits(follower)
    bus = follower.bus
    ids = [bus.motors[name].id for name in CHANNELS]
    urdf = tmp_path / "model.urdf"
    urdf.write_text(kinematic_urdf(), encoding="utf-8")
    solver = LeRobotPoseSolver(limits[:5], urdf_path=urdf)

    def quantize(target):
        readings = bus._normalize(bus._unnormalize(dict(zip(ids, target))))
        return [float(readings[motor]) for motor in ids]

    def y_position(joints):
        return solver.kinematics.forward_kinematics(np.asarray(joints[:5]))[1, 3]

    measured = quantize([-4.7, -65.7, 53.8, 68.4, -20.1, 3.4])
    start_y = y_position(measured)
    sent = None
    desired = None
    for _ in range(25):
        result = solve_intent(solver, measured, [0., .00005, 0., 0., 0., 0.], limits[5],
                              sent=sent, desired_xyz_m=desired)
        sent = result["target"]
        assert sent is not None, result["reason"]
        desired = result["requested_xyz_m"]
        measured = quantize(sent)
    assert y_position(measured) - start_y == pytest.approx(25 * .00005, abs=.00025)


@pytest.mark.parametrize("started_here", [True, False])
def test_pi_tunnel_stops_only_its_own_service(monkeypatch, started_here):
    invocation = "a" * 32
    commands = []
    tunnel = PiTunnel("eric@rpi", simulated=not started_here)

    if started_here:
        tunnel.hardware_config = 'REMOTE_IK_HARDWARE_ARGS="--serial-port /dev/serial/by-id/test"\n'

    def service_command(command, *, timeout, config=None):
        commands.append(command)
        if len(commands) == 1:
            assert config == tunnel.hardware_config
            assert ('cmp -s "$tmp" "$config"' in command) is started_here
            return SimpleNamespace(stdout=f"{invocation if started_here else ''}\n{'b' * 64}\n",
                                   stderr="", returncode=0)
        return SimpleNamespace(stdout="", stderr="", returncode=0)

    monkeypatch.setattr(tunnel, "service_command", service_command)
    with tunnel:
        tunnel.start_service()
    assert f"systemctl --user start {tunnel.unit}" in commands[0]
    if started_here:
        assert len(commands) == 2
        assert invocation in commands[1] and f"systemctl --user stop {tunnel.unit}" in commands[1]
    else:
        assert len(commands) == 1


def test_hardware_config_imports_once_then_uses_local_edits(tmp_path, monkeypatch):
    path = tmp_path / "remote_ik.hardware.conf"
    monkeypatch.setattr(remote_teleop, "HARDWARE_CONFIG", path)
    tunnel = PiTunnel("eric@rpi", simulated=False)
    original = 'REMOTE_IK_HARDWARE_ARGS="--serial-port /dev/serial/by-id/test --arm-rate 20"\n'
    reads = []

    def service_command(command, *, timeout):
        reads.append(command)
        return SimpleNamespace(stdout=original, stderr="", returncode=0)

    monkeypatch.setattr(tunnel, "service_command", service_command)
    assert tunnel.load_hardware_config() == original
    assert path.read_text(encoding="utf-8") == original

    edited = original.replace("--arm-rate 20", "--arm-rate 40")
    path.write_bytes(("# Saved on Windows\n" + edited).replace("\n", "\r\n").encode("utf-8-sig"))
    assert tunnel.load_hardware_config() == edited
    assert len(reads) == 1

    path.write_text(edited + "UNEXPECTED_SETTING=1\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="one REMOTE_IK_HARDWARE_ARGS"):
        tunnel.load_hardware_config()


def test_dashboard_hardware_settings_only_exposes_supported_fields():
    config = ('REMOTE_IK_HARDWARE_ARGS="--serial-port /dev/serial/by-id/test --allow-motion '
              '--hz 50 --watchdog-ms 200 --arm-rate=40 --gripper-rate 10 --token-file /private/token"\n')
    assert remote_teleop.dashboard_hardware_settings(config) == {
        "serial-port": "/dev/serial/by-id/test", "allow-motion": True, "hz": "50",
        "watchdog-ms": "200", "arm-rate": "40", "gripper-rate": "10",
    }


def test_auth_and_reconnect(rig):
    motor, bus, _ = rig
    with pytest.raises(ProtocolError, match="credentials"):
        motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "wrong"})
    first = command(motor, hello(motor), "arm")
    motor.tick(lost=True)
    second = hello(motor)
    assert first["session"] != second["session"] and not second["armed"]
    rejected = command(motor, second, "move", target=[1.] * 6, session=first["session"])
    assert "wrong session" in rejected["reason"]
    assert len(bus.writes) == 1


def test_start_edge_requires_neutral_input():
    latest = LatestInput()
    latest.publish(gamepad_input())
    latest.publish(gamepad_input(lx=.5, start=True))
    assert latest.sample.start == 0
    latest.publish(gamepad_input(start=True))
    assert latest.sample.start == 0
    latest.publish(gamepad_input())
    latest.publish(gamepad_input(start=True))
    assert latest.sample.start == 1
    latest.publish(gamepad_input(lb=True))
    latest.publish(None)
    assert latest.sample.release == 1


def test_xinput_maps_a_single_slot_and_clears_disconnected_state(monkeypatch):
    pad = XInputPad(lx=-32768, ly=32767, rx=12, ry=-32768, lt=9, rt=255, buttons=0x4109)
    connected = True

    def get_state(slot, pointer):
        assert slot == 2
        if not connected:
            return 1167
        ctypes.cast(pointer, ctypes.POINTER(XInputState)).contents.pad = pad
        return 0

    monkeypatch.setattr("controller.input.xinput.load_xinput", lambda: get_state)
    reader = XInputGamepad(2)
    latest = LatestInput()
    sample = reader.read()
    assert sample["axes"] == {"LX": -1., "LY": -1., "RX": 0., "RY": 1., "LT": 0., "RT": 1.}
    assert {label for label, pressed in sample["buttons"].items() if pressed} == {"Square", "LB"}
    assert sample["dpad"] == (1, 1)
    latest.publish(sample)

    connected = False
    latest.publish(reader.read())
    assert latest.sample.gamepad is None and latest.sample.release == 1

    connected = True
    pad = XInputPad(buttons=0x0010)
    latest.publish(reader.read())
    assert latest.sample.neutral and latest.sample.start == 0
    pad.buttons = 0
    latest.publish(reader.read())
    pad.buttons = 0x0010
    latest.publish(reader.read())
    assert latest.sample.start == 1


@pytest.mark.parametrize("simulated", [True, False])
def test_tcp_reconnect_and_mode_gate(simulated):
    bus = FakeBus()
    releases = []
    bus.relax = lambda: releases.append(True)
    motor = MotorSession(bus, limits=LIMITS, model="fixture", token=TOKEN, watchdog=.15,
                         hz=50, arm_rate=100, gripper_rate=50, simulated=simulated, allow_motion=True)
    stop = Event()
    with socket.socket() as listener:
        try:
            listener.bind(("127.0.0.1", 0))
            listener.listen(4)
            with socket.create_connection(listener.getsockname(), timeout=.5):
                pass
        except PermissionError:
            pytest.skip("Local sockets are disabled in this environment")
        pending, _ = listener.accept()
        pending.close()
        server = Thread(target=serve, args=(listener, motor, stop))
        server.start()
        port = listener.getsockname()[1]
        metadata = {"model": "fixture", "limits": LIMITS}
        try:
            with pytest.raises(RuntimeError, match="mode"):
                with MotorLink(port, TOKEN, metadata, not simulated):
                    pytest.fail("Client accepted the wrong motor service mode")
            with MotorLink(port, TOKEN, metadata, simulated, relax_on_exit=not simulated) as first:
                assert first.feedback["relax_on_exit"] is (not simulated)
                assert first.request("arm")["armed"]
                assert first.request("move", target=[1.] * 6)["moving"]
                sleep(.22)
                paused = first.request("poll")
                assert not paused["armed"] and not paused["moving"] and "watchdog" in paused["reason"]
            with MotorLink(port, TOKEN, metadata, simulated) as second:
                assert first.session != second.session
                assert not second.feedback["armed"]
                assert not second.request("move", target=[1.] * 6)["moving"]
            assert bus.writes[-1] == [0.] * 6
            assert not releases
        finally:
            stop.set()
            server.join(2)
            assert not server.is_alive()
        assert releases == ([] if simulated else [True])


@pytest.mark.skipif(sys.platform != "linux" or not shutil.which("fuser"), reason="Linux tty ownership guard")
def test_pi_serial_exclusivity_on_pseudo_terminal():
    master, slave = os.openpty()
    path = os.ttyname(slave)
    os.close(slave)
    handler = SimpleNamespace(ser=None)

    def open_port():
        handler.ser = open(path, "rb", buffering=0)
        return True

    def close_port(*, disable_torque):
        assert not disable_torque
        handler.ser.close()
        bus.is_connected = False

    def connect():
        bus.is_connected = True
        handler.openPort()

    handler.openPort = open_port
    bus = SimpleNamespace(port_handler=handler, is_connected=False, connect=connect, disconnect=close_port)
    follower = SimpleNamespace(bus=bus, is_calibrated=True)
    other = subprocess.Popen([sys.executable, "-c", "import sys; open_file=open(sys.argv[1]); print('ready', flush=True); input()", path],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    try:
        assert other.stdout.readline() == b"ready\n"
        with pytest.raises(RuntimeError, match="already in use"):
            with PiMotorBus(follower, path):
                pytest.fail("Opened a tty held by a legacy process")
        other.communicate(b"stop\n", timeout=2)
        with PiMotorBus(follower, path):
            with pytest.raises(OSError) as exc:
                os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            assert exc.value.errno == errno.EBUSY
    finally:
        if other.poll() is None:
            other.kill()
            other.wait()
        os.close(master)
