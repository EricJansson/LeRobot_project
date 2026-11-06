import ctypes
import errno
from io import BytesIO
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

import pytest

from controller.input.xinput import XInputGamepad, XInputPad, XInputState
from robot_program.lerobot_ik_controller import DEFAULT_URDF
from robot_program.remote_ik_protocol import PROTOCOL_VERSION, kinematic_urdf
from scripts.install_remote_ik_pi import bundle
from scripts.main_teleop_remote_ik import LatestInput, MotorLink
from scripts.remote_ik_motor_service import MotorSession, PiMotorBus, ProtocolError, serve


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


def test_tcp_simulation_reconnect_and_mode_gate():
    bus = FakeBus()
    motor = MotorSession(bus, limits=LIMITS, model="fixture", token=TOKEN, watchdog=.15,
                         hz=50, arm_rate=100, gripper_rate=50)
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
                with MotorLink(port, TOKEN, metadata, False):
                    pytest.fail("Live client accepted a simulated service")
            with MotorLink(port, TOKEN, metadata, True) as first:
                assert first.request("arm")["armed"]
                assert first.request("move", target=[1.] * 6)["moving"]
                sleep(.22)
                paused = first.request("poll")
                assert not paused["armed"] and not paused["moving"] and "watchdog" in paused["reason"]
            with MotorLink(port, TOKEN, metadata, True) as second:
                assert first.session != second.session
                assert not second.feedback["armed"]
                assert not second.request("move", target=[1.] * 6)["moving"]
            assert bus.writes[-1] == [0.] * 6
        finally:
            stop.set()
            server.join(2)
            assert not server.is_alive()


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
