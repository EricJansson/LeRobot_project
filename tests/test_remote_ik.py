import ctypes
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

import numpy as np
import pytest

from controller.input.xinput import XInputGamepad, XInputPad, XInputState
from robot_program.lerobot_ik_controller import DEFAULT_URDF, LeRobotPoseSolver
from robot_program.remote_ik_protocol import (
    CHANNELS, PROTOCOL_VERSION, calibrated_follower, channel_limits, kinematic_urdf,
)
from scripts.install_remote_ik_pi import bundle
from scripts.main_teleop_remote_ik import ControlLoop, LatestInput, MotorLink, PiTunnel
from scripts.remote_ik_motor_service import MotorSession, PiMotorBus, ProtocolError, serve
from scripts.remote_ik_worker import solve_intent


LIMITS = [[-90., 90.]] * 5 + [[0., 50.]]
TOKEN = "sample-token"


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
                          enable_torque=lambda: calls.append("torque"))
    adapter = PiMotorBus(SimpleNamespace(bus=bus), __file__)
    assert adapter.read() == values
    assert adapter.arm(values) == values
    assert calls == [("Goal_Position", positions), "torque"]


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

    def solve(measured, delta, *, sent):
        if moving:
            assert any(delta)
            assert sent == [2.] * 6
            latest.publish(None if scenario == "disconnect during IK" else gamepad_input())
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
    for _ in range(25):
        result = solve_intent(solver, measured, [0., .00005, 0., 0., 0., 0.], limits[5], sent=sent)
        sent = result["target"]
        assert sent is not None, result["reason"]
        measured = quantize(sent)
    assert y_position(measured) - start_y == pytest.approx(25 * .00005, abs=.00025)


@pytest.mark.parametrize("started_here", [True, False])
def test_pi_tunnel_stops_only_its_own_service(monkeypatch, started_here):
    invocation = "a" * 32
    commands = []
    tunnel = PiTunnel("eric@rpi", simulated=not started_here)

    def service_command(command, *, timeout):
        commands.append(command)
        if len(commands) == 1:
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
    def gamepad(*, lx=0., start=False, lb=False):
        return {"axes": {"LX": lx}, "buttons": {"Start": start, "LB": lb, "Back": False}, "dpad": (0, 0)}

    latest = LatestInput()
    latest.publish(gamepad())
    latest.publish(gamepad(lx=.5, start=True))
    assert latest.sample.start == 0
    latest.publish(gamepad(start=True))
    assert latest.sample.start == 0
    latest.publish(gamepad())
    latest.publish(gamepad(start=True))
    assert latest.sample.start == 1
    latest.publish(gamepad(lb=True))
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
