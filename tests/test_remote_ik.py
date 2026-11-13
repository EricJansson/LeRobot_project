import ctypes
from contextlib import nullcontext
import errno
from io import BytesIO
from importlib.metadata import version
import json
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

from controller.input import windows_gamepad
from controller.input.xinput import XInputGamepad, XInputPad, XInputState
from robot_program.lerobot_ik_controller import DEFAULT_URDF, LeRobotPoseSolver
from robot_program.remote_ik_protocol import (
    CHANNELS, MOTION_POLICY, PROTOCOL_VERSION, calibrated_follower, channel_limits, command_limits, kinematic_urdf,
)
from scripts import main_teleop_remote_ik as remote_teleop
from scripts.install_remote_ik_pi import bundle
from scripts.main_teleop_remote_ik import MotorLink, PiTunnel
from robot_program import remote_ik_control
from robot_program.remote_ik_control import ControlLoop, LatestInput
from scripts.remote_ik_motor_service import MotorSession, PiMotorBus, ProtocolError, motor_listener, serve
from scripts.remote_ik_worker import solve_intent


LIMITS = [[-90., 90.]] * 5 + [[0., 50.]]
METADATA = {"model": "fixture", "limits": LIMITS, "calibrated_limits": LIMITS,
            "command_limits": command_limits(LIMITS), "v": PROTOCOL_VERSION, "motion_policy": MOTION_POLICY}
TOKEN = "sample-token"


def test_motor_listener_preserves_exclusive_active_owner():
    with motor_listener(0) as first:
        with pytest.raises(OSError, match="already in use"):
            with motor_listener(first.getsockname()[1]):
                pytest.fail("A second live motor listener acquired the port")


@pytest.mark.skipif(sys.platform != "linux", reason="Pi/Linux TIME_WAIT restart behavior")
def test_motor_listener_restarts_immediately_after_server_closed_connection():
    with motor_listener(0) as first:
        port = first.getsockname()[1]
        with socket.create_connection(("127.0.0.1", port), timeout=2) as client:
            accepted, _ = first.accept()
            with accepted:
                accepted.settimeout(2)
                # Make the server the active closer so its address enters
                # TIME_WAIT before the new service attempts to bind it.
                accepted.shutdown(socket.SHUT_WR)
                assert client.recv(1) == b""
                client.shutdown(socket.SHUT_WR)
                assert accepted.recv(1) == b""
    with motor_listener(port) as restarted:
        assert restarted.getsockname() == ("127.0.0.1", port)


@pytest.mark.parametrize("existing, record_tick", [(False, True), (True, True), (True, False)])
def test_launcher_trace_overwrite(monkeypatch, tmp_path, capsys, existing, record_tick):
    trace_path = tmp_path / "trace.json"
    old_data = '{"previous_run": true, "padding": "old data must be truncated"}'
    if existing:
        trace_path.write_text(old_data, encoding="utf-8")
    monkeypatch.setattr(sys, "argv", ["remote-ik", "--pi-sim", "--trace", str(trace_path)])
    monkeypatch.setattr(remote_teleop, "open_gamepad", lambda *args: nullcontext(lambda: None))
    monkeypatch.setattr(remote_teleop, "load_key_state", lambda: lambda key: -1)
    monkeypatch.setattr(remote_teleop, "WebDashboard", lambda *args: nullcontext(
        SimpleNamespace(port=18743, exit_reason=None)))
    monkeypatch.setattr(remote_teleop, "Worker", lambda *args: nullcontext(
        SimpleNamespace(metadata={"model": "fixture"})))
    monkeypatch.setattr(remote_teleop, "ensure_installed", lambda host: "a" * 64)
    monkeypatch.setattr(remote_teleop, "PiTunnel", lambda *args, **kwargs: nullcontext(
        SimpleNamespace(connect=None)))

    def control(*args, trace, **kwargs):
        def run():
            if record_tick:
                trace.append({"tick_s": 1})
        return SimpleNamespace(run=run, error=None)

    monkeypatch.setattr(remote_ik_control, "ControlLoop", control)
    remote_teleop.main()
    if record_tick:
        assert {k: v for k, v in json.loads(trace_path.read_text(encoding="utf-8")).items() if k != "provenance"} == {
            "model": "fixture", "ticks": [{"tick_s": 1}],
        }
        assert "Remote IK trace saved to" in capsys.readouterr().out
    else:
        assert trace_path.read_text(encoding="utf-8") == old_data
        assert "Remote IK trace saved to" not in capsys.readouterr().out


@pytest.mark.parametrize("options, expected_port", [
    ([], 18743),
    (["--dashboard-port", "18744"], 18744), (["--dashboard-port", "0"], 0),
    (["--advance-button", "Start"], 18743),
    (["--advance-button", "Back"], 18743),
    (["--gamepad", "sdl", "--index", "4"], 18743),
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

    def open_gamepad(backend, index):
        assert (backend, index) == (("sdl", 4) if "--gamepad" in options else ("auto", None))
        return nullcontext(lambda: None)

    monkeypatch.setattr(remote_teleop, "open_gamepad", open_gamepad)
    monkeypatch.setattr(remote_teleop, "load_key_state", lambda: lambda key: -1)
    monkeypatch.setattr(remote_teleop, "Worker", lambda *args: nullcontext(
        SimpleNamespace(metadata=METADATA)))
    monkeypatch.setattr(remote_ik_control, "ControlLoop", lambda *args, **kwargs:
                        SimpleNamespace(run=lambda: None, error=None))
    monkeypatch.setattr(webbrowser, "open", lambda *args, **kwargs: pytest.fail("Browser opened automatically"))
    monkeypatch.setattr(remote_teleop.subprocess, "run", lambda *args, **kwargs: pytest.fail("Git called without tracing"))
    monkeypatch.setattr(sys, "argv", ["remote-ik", "--dry-run", *options])
    remote_teleop.main()
    assert ports == [expected_port]
    output = capsys.readouterr().out
    assert f"http://127.0.0.1:{dashboards[0].port}/" in output
    expected_advance = options[options.index("--advance-button") + 1] if "--advance-button" in options else "Start"
    assert f"{expected_advance}: Released" in output


def test_launcher_busy_dashboard_port(monkeypatch, capsys):
    monkeypatch.setattr(remote_teleop, "open_gamepad", lambda *args: nullcontext(lambda: None))
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


def test_pi_bundle_imports_without_the_source_checkout(tmp_path):
    with tarfile.open(fileobj=BytesIO(bundle())) as archive:
        archive.extractall(tmp_path, filter="data")
    script = '''
import sys
sys.path.insert(0, sys.argv[1])
from scripts.remote_ik_motor_service import MotorSession, SimulatedBus
limits = [[-90., 90.]] * 5 + [[0., 50.]]
motor = MotorSession(SimulatedBus(limits), limits=limits, model="fixture", token="test",
                     watchdog=.2, hz=50, arm_rate=200, gripper_rate=30)
assert motor.recovery.goal is None
'''
    result = subprocess.run([sys.executable, "-I", "-c", script, str(tmp_path)],
                            cwd=tmp_path, capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr


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
    torque = [0]
    def enable_torque(*, num_retry):
        calls.append(("torque", num_retry))
        torque[0] = 1
    bus = SimpleNamespace(sync_read=lambda register, *args, **kwargs: positions if register == "Present_Position"
                          else dict.fromkeys(CHANNELS, torque[0]),
                          sync_write=lambda register, targets: calls.append((register, targets)),
                          enable_torque=enable_torque)
    adapter = PiMotorBus(SimpleNamespace(bus=bus), __file__)
    assert adapter.read() == values
    assert adapter.arm(values) == values
    assert calls == [("Goal_Position", positions), ("torque", 1)]


def test_failed_torque_enable_leaves_real_arm_disarmed():
    writes = []

    def enable_torque(*, num_retry):
        raise ConnectionError("Incorrect status packet")

    bus = SimpleNamespace(sync_read=lambda register, *args, **kwargs: dict.fromkeys(CHANNELS, 0.),
                          sync_write=lambda register, targets: writes.append(targets),
                          enable_torque=enable_torque)
    adapter = PiMotorBus(SimpleNamespace(bus=bus), __file__)
    motor = MotorSession(adapter, limits=LIMITS, model="fixture", token=TOKEN, watchdog=.15,
                         hz=50, arm_rate=100, gripper_rate=50, simulated=False, allow_motion=True)
    feedback = command(motor, hello(motor), "capture")
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

    def solve(measured, delta, *, sent, desired_xyz_m, joint_step_limits, correction, recovery_step_m):
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
            return {"target": [1.] * 6, "accepted_xyz_m": [0.] * 3}
        assert sent == ([2.] * 6 if armed else None)
        if armed:
            assert any(delta)
            return {"target": [1.] * 6, "accepted_xyz_m": [0.] * 3}
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
    latest, stop, trace = LatestInput(clock=lambda: now[0]), Event(), []
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

    def solve(measured, delta, *, sent, desired_xyz_m, joint_step_limits, correction, recovery_step_m):
        if index == 8:
            return {"reason": "joint limit exceeded"}
        if not any(delta):
            return {}
        if index == 10:
            latest.publish(gamepad_input(lb=True))
            publish()
        if index in (7, 11):
            assert desired_xyz_m is not None
        baseline = desired_xyz_m if desired_xyz_m is not None else [0.] * 3
        target = [prior + step * 1000 for prior, step in zip(feedback["sent"], delta)]
        return {"baseline_xyz_m": baseline, "requested_xyz_m": np.add(baseline, delta[:3]).tolist(),
                "target": target, "accepted_xyz_m": np.add(baseline, delta[:3]).tolist()}

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
    control = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, 1., "Sim", trace=trace, clock=lambda: now[0])
    control.session(SimpleNamespace(feedback=feedback, request=request))
    for tick, (_, _, _, scale, op) in zip(trace, steps, strict=True):
        assert tick["op"] == op
        if scale is None:
            assert tick["arm_scale"] is None
        else:
            assert tick["arm_scale"] == pytest.approx(scale)
            assert tick["delta"][0] == pytest.approx(-2 * tick["delta"][1])
    assert [trace[i]["delta"][5] for i in (0, 4)] == pytest.approx([1., 1.])
    assert trace[0]["issued"][0] - trace[0]["issued_before"][0] == pytest.approx(.2)
    assert trace[2]["issued"][0] - trace[2]["issued_before"][0] == pytest.approx(4.)
    assert trace[7]["issued"][0] - trace[7]["issued_before"][0] == pytest.approx(.2)
    assert trace[8]["ik_reason"] == "joint limit exceeded"
    for hold_index in (6, 10):
        hold, restart = trace[hold_index:hold_index + 2]
        assert hold["xyz_reset"] is None
        assert hold["desired_before_m"] is not None
        assert hold["desired_xyz_m"] == restart["desired_before_m"] == hold["desired_before_m"]
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
        return SimpleNamespace(accepted=True, joints_deg=seed, pose=forward(seed))

    solver = SimpleNamespace(kinematics=SimpleNamespace(forward_kinematics=forward), solve_pose=solve_pose)
    result = solve_intent(solver, measured, delta, LIMITS[5], sent=issued, desired_xyz_m=desired)
    assert result["baseline_xyz_m"] == baseline
    assert result["requested_xyz_m"] == pytest.approx(requested)
    assert result["target"] == issued
    jaw_only = solve_intent(solver, measured, [0., 0., 0., 0., 0., 1.], LIMITS[5],
                            sent=issued, desired_xyz_m=desired)
    assert jaw_only["target"] == [*issued[:5], 13.]


def test_control_loop_preserves_xyz_through_rejection_slew_neutral_and_lb_release(monkeypatch):
    now = [0.]
    latest = LatestInput(clock=lambda: now[0])
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
        (.08, -1., True, advanced, "slew", "move"),
        (.10, -1., True, advanced, "full", "move"),
        (.12, 0., True, retracted, None, "hold"),
        (.14, -1., True, retracted, "slew", "move"),
        (.16, -1., True, retracted, "full", "move"),
        (.18, -1., False, None, None, "hold"),
        (.20, 1., True, retracted, "full", "move"),
    ]
    index = 0
    statuses = []

    def publish():
        at, x, lb, *_ = steps[index]
        now[0] = at
        gamepad = gamepad_input(lb=lb)
        gamepad["axes"]["LY"] = np.float64(-x)
        latest.publish(gamepad)

    def solve(measured, delta, *, sent, desired_xyz_m, joint_step_limits, correction, recovery_step_m):
        at, x, _, expected_desired, outcome, _ = steps[index]
        assert desired_xyz_m == expected_desired
        if outcome is None:
            assert not any(delta)
            return {}
        assert np.sign(delta[0]) == np.sign(x)
        assert not any(delta[1:])
        baseline = measured[:3] if desired_xyz_m is None else desired_xyz_m
        if at == .14:
            assert baseline == retracted and sent is not None
        return {"xyz_m": measured[:3], "baseline_xyz_m": baseline,
                "requested_xyz_m": advanced if x > 0 else retracted,
                "accepted_xyz_m": advanced if x > 0 else retracted,
                "target": None if outcome == "reject" else target,
                "reason": "position tolerance not reached" if outcome == "reject" else ""}

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
    control = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, .25, "Sim", clock=lambda: now[0])
    control.session(SimpleNamespace(feedback=feedback, request=request))
    assert statuses[1:] == [
        advanced, advanced, advanced, advanced, advanced, retracted, retracted, retracted, retracted, retracted,
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
        desired = result.get("accepted_xyz_m", result["requested_xyz_m"])
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


@pytest.mark.parametrize("backend, index, expected_slot", [("auto", None, 2), ("xinput", None, 0)])
def test_windows_gamepad_uses_xinput_without_opening_sdl(monkeypatch, backend, index, expected_slot):
    from controller.input import sdl_gamepad

    slots_read = []

    def xinput(slot):
        def read():
            slots_read.append(slot)
            return {"name": f"XInput {slot}"} if slot == expected_slot else None

        return SimpleNamespace(read=read)

    monkeypatch.setattr(windows_gamepad, "XInputGamepad", xinput)
    monkeypatch.setattr(sdl_gamepad, "open_sdl_gamepad", lambda _: pytest.fail("SDL opened"))
    with windows_gamepad.open_gamepad(backend, index) as read:
        assert read() == {"name": f"XInput {expected_slot}"}
        assert read() == {"name": f"XInput {expected_slot}"}
    assert slots_read == [*range(expected_slot + 1), expected_slot]


def test_windows_gamepad_waits_for_sdl_and_keeps_its_selected_device(monkeypatch):
    from controller.input import sdl_gamepad

    now = 10.
    sdl_connected = False
    xinput_connected = False
    opens = []
    monkeypatch.setattr(windows_gamepad, "monotonic", lambda: now)
    monkeypatch.setattr(windows_gamepad, "XInputGamepad", lambda _: SimpleNamespace(
        read=lambda: {"name": "XInput"} if xinput_connected else None))

    def open_sdl(index):
        opens.append(index)
        if not sdl_connected:
            raise sdl_gamepad.NoSDLGamepad()
        return nullcontext(SimpleNamespace(
            name="PS4 Controller", index=1,
            read=lambda: {"name": "PS4 Controller"} if sdl_connected else None,
        ))

    monkeypatch.setattr(sdl_gamepad, "open_sdl_gamepad", open_sdl)
    with windows_gamepad.open_gamepad("auto", None) as read:
        assert read() is None
        sdl_connected = True
        assert read() is None
        now += 1
        assert read() == {"name": "PS4 Controller"}
        sdl_connected = False
        xinput_connected = True
        now += 1
        assert read() is None
    assert opens == [None, None]


def test_sdl_maps_controls_and_drops_detached_controller(monkeypatch):
    import pygame
    from pygame._sdl2 import controller

    from controller.input.sdl_gamepad import open_sdl_gamepad

    axes = {
        pygame.CONTROLLER_AXIS_LEFTX: -32768, pygame.CONTROLLER_AXIS_LEFTY: -32768,
        pygame.CONTROLLER_AXIS_RIGHTX: 15, pygame.CONTROLLER_AXIS_RIGHTY: 32767,
        pygame.CONTROLLER_AXIS_TRIGGERLEFT: 500, pygame.CONTROLLER_AXIS_TRIGGERRIGHT: 32767,
    }
    buttons = {pygame.CONTROLLER_BUTTON_LEFTSHOULDER, pygame.CONTROLLER_BUTTON_BACK,
               pygame.CONTROLLER_BUTTON_DPAD_UP, pygame.CONTROLLER_BUTTON_DPAD_RIGHT}
    connected = True
    opened = []
    closed = []

    class Pad:
        name = "PS4 Controller"

        def __init__(self, index):
            opened.append(index)

        def attached(self):
            return connected

        def get_axis(self, axis):
            return axes[axis]

        def get_button(self, button):
            return button in buttons

        def quit(self):
            closed.append(self.name)

    monkeypatch.setattr(pygame.event, "clear", lambda: None)
    monkeypatch.setattr(pygame.display, "init", lambda: None)
    monkeypatch.setattr(pygame.display, "quit", lambda: None)
    monkeypatch.setattr(controller, "init", lambda: None)
    monkeypatch.setattr(controller, "quit", lambda: None)
    monkeypatch.setattr(controller, "get_count", lambda: 2)
    monkeypatch.setattr(controller, "is_controller", lambda index: index == 1)
    monkeypatch.setattr(controller, "name_forindex", lambda index: Pad.name)
    monkeypatch.setattr(controller, "Controller", Pad)

    with pytest.raises(RuntimeError, match="Available gamepads: 1: PS4 Controller"):
        with open_sdl_gamepad(0):
            pytest.fail("Unmapped device was opened")
    latest = LatestInput()
    with open_sdl_gamepad() as reader:
        assert reader.index == 1
        sample = reader.read()
        assert sample["name"] == Pad.name
        assert sample["axes"] == {"LX": -1., "LY": -1., "RX": 0., "RY": 1., "LT": 0., "RT": 1.}
        assert {label for label, pressed in sample["buttons"].items() if pressed} == {"Back", "LB"}
        assert sample["dpad"] == (1, 1)
        latest.publish(sample)

        axes[pygame.CONTROLLER_AXIS_TRIGGERLEFT] = 32767
        axes[pygame.CONTROLLER_AXIS_TRIGGERRIGHT] = 0
        buttons.clear()
        buttons.update((pygame.CONTROLLER_BUTTON_START, pygame.CONTROLLER_BUTTON_DPAD_DOWN))
        sample = reader.read()
        assert (sample["axes"]["LT"], sample["axes"]["RT"]) == (1., 0.)
        assert {label for label, pressed in sample["buttons"].items() if pressed} == {"Start"}
        assert sample["dpad"] == (0, -1)

        connected = False
        latest.publish(reader.read())
        assert latest.sample.gamepad is None and latest.sample.release == 1 and latest.sample.disconnect == 1
        assert reader.read() is None
    assert opened == [1] and closed == [Pad.name]


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
        metadata = METADATA
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


@pytest.mark.parametrize("flag", ["--no-browser", "--commission-ik"])
def test_removed_launcher_flags_are_rejected(monkeypatch, capsys, flag):
    monkeypatch.setattr(sys, "argv", ["remote-ik", "--dry-run", flag])
    with pytest.raises(SystemExit) as exc:
        remote_teleop.main()
    assert exc.value.code == 2
    assert "unrecognized arguments: " + flag in capsys.readouterr().err


def test_mode_remains_explicit(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["remote-ik"])
    with pytest.raises(SystemExit) as exc:
        remote_teleop.main()
    assert exc.value.code == 2
    assert "Choose --dry-run, --pi-sim, or --real-arm" in capsys.readouterr().err


def test_worker_request_parameters_are_explicit_per_call():
    worker = remote_teleop.Worker("Ubuntu", "python")
    stream = BytesIO()
    worker.process = SimpleNamespace(stdin=stream)
    worker.receive = lambda timeout: {}
    for correction, step in [(True, .00002), (False, .001)]:
        worker.solve([0.]*6, [0.]*6, sent=None, desired_xyz_m=None,
                     joint_step_limits=[1.]*6, correction=correction, recovery_step_m=step)
    frames = [json.loads(line) for line in stream.getvalue().splitlines()]
    assert [frame["correction"] for frame in frames] == [True, False]
    assert [frame["recovery_step_m"] for frame in frames] == [.00002, .001]
    assert not hasattr(worker, "correction")


def test_worker_recovery_request_carries_pi_step_without_cartesian_backlog():
    worker = remote_teleop.Worker("Ubuntu", "python")
    stream = BytesIO()
    worker.process = SimpleNamespace(stdin=stream)
    worker.receive = lambda timeout: {"target": None}
    worker.recover([0.]*6, [0.,0.,.001,0.,0.,0.], sent=[0.]*6,
                   automatic=[.1]*5+[0.], anchor=[0.]*6, goal=[1.]*5+[0.], steps=[.1]*6)
    frame = json.loads(stream.getvalue())
    assert frame["op"] == "recover"
    assert frame["automatic"] == [.1]*5+[0.]
    assert frame["steps"] == [.1]*6
    assert "desired_xyz_m" not in frame
