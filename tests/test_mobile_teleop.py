import asyncio
from contextlib import nullcontext
import sys
from threading import Event
from time import monotonic, sleep
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, Mock

import pytest

pytest.importorskip("websockets", minversion="16")
from websockets.exceptions import ConnectionClosedError, InvalidStatus
from websockets.sync.client import connect

from robot_program.remote_ik_control import ControlLoop, motion_delta
from scripts import main_mobile_teleop as launcher
from scripts import mobile_teleop_gateway as gateway_module
from scripts.mobile_teleop_gateway import (
    ADVANCE, ARM_CONTROLS_HEADER, ARM_INPUT_FRAME, ARM_INPUT_PATH, DEADMAN, FRESHNESS_S, INPUT_FRAME, INPUT_PATH,
    POSE_FRAME, POSE_PATH, POSE_STALE_S, STOP, TOUCHSCREEN_IDLE_S, TOUCHSCREEN_PATH, TOUCHSCREEN_STALE_S,
    TRACKING, MarkerPose, MobileGateway, MobileInput, TouchscreenInput, mobile_arm_status,
)


def test_launcher_reads_the_same_user_token_on_each_start(tmp_path):
    path = tmp_path / "mobile-teleop.token"
    with pytest.raises(ValueError, match="Add your phone's shared bearer token"):
        launcher.load_phone_token(path)
    path.write_text("manual-phone-token\n", encoding="utf-8-sig")
    assert launcher.load_phone_token(path) == launcher.load_phone_token(path) == "manual-phone-token"
    path.write_text("Bearer manual-phone-token\n", encoding="utf-8")
    with pytest.raises(ValueError, match="no 'Bearer ' prefix"):
        launcher.load_phone_token(path)


@pytest.mark.parametrize(("route_ip", "expected"), [("192.168.1.5", "192.168.1.5"), ("203.0.113.1", "127.0.0.1")])
def test_launcher_defaults_to_the_private_route_address(monkeypatch, route_ip, expected):
    probe = MagicMock()
    probe.__enter__.return_value.getsockname.return_value = (route_ip, 0)
    monkeypatch.setattr(launcher.socket, "socket", lambda *_: probe)
    assert launcher.default_input_address() == expected
    probe.__enter__.return_value.send.assert_not_called()


@pytest.mark.parametrize("flags,mode,arm_status,speed", [
    ([], "Observe only", "Unavailable", 1.),
    (["--real-arm"], "Observe only", "Unavailable", 1.),
    (["--speed", "0.1"], "Observe only", "Unavailable", .1),
    (["--dry-run"], "Local simulation", "Holding", 1.),
    (["--pi-sim"], "Pi simulation", "Holding", .1),
])
def test_launcher_selects_motor_link_and_publishes_arm_metrics(monkeypatch, flags, mode, arm_status, speed):
    status = {"mode": mode, "connection": "Connected", "state": "Holding", "control_state": "Holding",
              "measured": [1., 2., 3., 4., 5., 0.], "xyz_m": [.1, -.2, .3], "feedback_hz": 50.}
    gateway = MagicMock(address="127.0.0.1", port=18744)
    phone = {"phone_mode": "aruco", "phone": {}, "marker_pose": {"state": "Waiting for marker pose"}}
    gateway.telemetry.return_value = phone
    dashboard = MagicMock(port=18745)
    keys = iter([0, -1])
    tunnel = SimpleNamespace(connect=Mock())
    open_tunnel = Mock(return_value=nullcontext(tunnel))
    deploy = Mock(return_value="test-fingerprint")
    local = Mock(return_value=Mock())
    loop = Mock(return_value=SimpleNamespace(status=status, run=lambda: None, error=None))
    monkeypatch.setattr(sys, "argv", ["mobile-teleop", "--listen", "127.0.0.1", *flags])
    monkeypatch.setattr(launcher, "load_phone_token", lambda _: "mobile-test-token")
    monkeypatch.setattr(launcher, "load_key_state", lambda: lambda _: next(keys))
    monkeypatch.setattr(launcher, "MobileGateway", lambda *args, **kwargs: nullcontext(gateway))
    monkeypatch.setattr(launcher, "WebDashboard", lambda *args, **kwargs: nullcontext(dashboard))
    monkeypatch.setattr(launcher, "Worker", lambda *args: nullcontext(None))
    monkeypatch.setattr(launcher, "local_simulated_link", local)
    monkeypatch.setattr(launcher, "ensure_installed", deploy)
    monkeypatch.setattr(launcher, "PiTunnel", open_tunnel)
    monkeypatch.setattr(launcher, "ControlLoop", loop)

    launcher.main()

    assert loop.call_args.args[4] == speed
    if mode != "Local simulation":
        deploy.assert_called_once_with("eric@rpi")
        open_tunnel.assert_called_once_with("eric@rpi", flags == ["--pi-sim"], fingerprint="test-fingerprint")
        assert loop.call_args.args[1] is tunnel.connect
        assert loop.call_args.args[2].joysticks_only
        assert not loop.call_args.kwargs["allow_limit_recovery"]
        assert loop.call_args.kwargs["on_link_lost"] == loop.call_args.args[2].end
        local.assert_not_called()
    else:
        deploy.assert_not_called()
        open_tunnel.assert_not_called()
        assert loop.call_args.args[1] is local.return_value
    dashboard.publish.assert_called_once_with(
        {"shoulder_pan": 1., "shoulder_lift": 2., "elbow_flex": 3., "wrist_flex": 4., "wrist_roll": 5.},
        0., {"x": .1, "y": -.2, "z": .3}, False, gamepad=None,
        remote={**status, **phone, "arm_status": arm_status},
    )


@pytest.mark.parametrize("flags", [[], ["--real-arm"]])
def test_launcher_restricts_real_arm_listen_address(monkeypatch, capsys, flags):
    monkeypatch.setattr(sys, "argv", ["mobile-teleop", "--listen", "0.0.0.0", *flags])
    with pytest.raises(SystemExit) as error:
        launcher.main()
    assert error.value.code == 2
    assert "Use a private LAN IPv4 address or 127.0.0.1 in real-arm mode" in capsys.readouterr().err


def arm_frame(seq, flags=0, x=0., y=0., z=0., roll=0., pitch=0., gripper=0):
    return ARM_INPUT_FRAME.pack(seq, flags, x, y, z, roll, pitch, gripper)


def test_pi_gateway_advertises_joysticks_and_invalidation_retires_the_socket():
    phone = MobileInput(joysticks_only=True)
    headers = {"Authorization": "Bearer test-token"}
    with MobileGateway("127.0.0.1", 0, phone, "test-token", arm_status=lambda: "Holding") as gateway:
        base_url = f"ws://127.0.0.1:{gateway.port}"
        with pytest.raises(InvalidStatus) as refused:
            connect(base_url + INPUT_PATH, additional_headers=headers, proxy=None)
        assert refused.value.response.status_code == 404
        with connect(base_url + ARM_INPUT_PATH, additional_headers=headers, proxy=None) as client:
            assert client.response.headers[ARM_CONTROLS_HEADER] == "joysticks"
            assert client.recv(timeout=.5) == "arm-status:Holding"
            phone.end()  # Motor connection dropped before the first phone frame.
            client.send(arm_frame(0))
            with pytest.raises(ConnectionClosedError, match="Input expired"):
                client.recv(timeout=.5)
        with connect(base_url + ARM_INPUT_PATH, additional_headers=headers, proxy=None) as client:
            client.send(arm_frame(0))
            client.send(arm_frame(1, DEADMAN, roll=.5))
            wait_for_telemetry(gateway, lambda status: status["phone"]["seq"] == 1)
            assert not phone.sample.lb


def test_pi_joysticks_drive_four_axes_and_hold_on_either_release():
    now = 1.
    phone = MobileInput(clock=lambda: now, joysticks_only=True)
    phone.begin(arm_controls=True)
    phone.publish(arm_frame(0))
    stop = Event()
    measured = [0.] * 6
    feedback = {"armed": False, "moving": False, "measured": measured, "sent": measured,
                "hz": 50, "watchdog_s": .2, "fault": False, "reason": ""}
    frames = iter((arm_frame(1, ADVANCE),
                   arm_frame(2, DEADMAN, .25, -.5, .25, .5, .5, 1),
                   arm_frame(3, DEADMAN), arm_frame(4, DEADMAN, roll=.5),
                   arm_frame(5, roll=.5), arm_frame(6, STOP), arm_frame(7, DEADMAN, x=.5)))
    operations, deltas = [], []

    def solve(_measured, delta, **_):
        if not any(delta):
            return {}
        deltas.append(delta)
        return {"target": [1.] * 5 + [0.], "accepted_xyz_m": [0., 0., 0.]}

    def request(op, **values):
        nonlocal now
        operations.append(op)
        if op == "arm":
            feedback["armed"] = True
        elif op == "move":
            feedback.update(moving=True, sent=values["target"])
        elif op in ("hold", "disarm"):
            feedback["moving"] = False
            if op == "disarm":
                feedback["armed"] = False
        frame = next(frames, None)
        if frame is None:
            stop.set()
        else:
            now += .02
            assert phone.publish(frame)
        return dict(feedback)

    loop = ControlLoop(SimpleNamespace(solve=solve), None, phone, stop, .1, "Pi simulation",
                       clock=lambda: now, allow_limit_recovery=False)
    loop.session(SimpleNamespace(feedback=dict(feedback), request=request))
    assert operations == ["poll", "arm", "move", "hold", "move", "hold", "disarm", "poll"]
    assert deltas[0] == pytest.approx([.000005, -.00001, .000005, 0., .00375, 0.])
    assert deltas[1] == pytest.approx([0., 0., 0., 0., .00375, 0.])


def test_limit_handoff_disables_phone_start():
    assert mobile_arm_status({"control_state": "Holding", "limit_handoff": True}) == "Unavailable"


def test_arm_receive_deadline_rechecks_an_early_timer_and_rejects_late_frames(monkeypatch):
    now = 10.
    timeouts = []
    frames = iter(((10., 0), (10.070, 1), (10.146, 2)))

    class Timer:
        def __init__(self, delay):
            timeouts.append(delay)
            self.early = len(timeouts) == 2

        async def __aenter__(self):
            nonlocal now
            if self.early:
                now = 10.066
                raise TimeoutError

        async def __aexit__(self, *_):
            pass

        def expired(self):
            return self.early

    async def recv():
        nonlocal now
        now, seq = next(frames)
        return arm_frame(seq)

    monkeypatch.setattr(gateway_module, "monotonic", lambda: now)
    monkeypatch.setattr(asyncio, "timeout", Timer)
    monkeypatch.setattr(asyncio, "timeout_at", Timer)
    phone = MobileInput(clock=lambda: now)
    gateway = MobileGateway("127.0.0.1", 0, phone, "test-token")
    connection = SimpleNamespace(request=SimpleNamespace(path=ARM_INPUT_PATH), recv=recv,
                                 send=AsyncMock(), close=AsyncMock())

    asyncio.run(gateway._receive(connection))

    assert timeouts == pytest.approx([2., FRESHNESS_S, FRESHNESS_S - .066, FRESHNESS_S])
    assert phone.telemetry()["seq"] == 1
    assert not gateway.active
    connection.close.assert_awaited_once_with(1008, "Input expired")


def test_arm_input_mapping_and_fresh_start_after_stopping_or_reconnecting():
    now = 1.
    phone = MobileInput(clock=lambda: now)
    phone.begin(arm_controls=True)
    assert phone.publish(arm_frame(0))
    assert phone.publish(arm_frame(1, DEADMAN, x=.25))
    assert not phone.sample.lb
    assert phone.publish(arm_frame(2, ADVANCE))
    assert phone.sample.start == 1
    wire = bytes.fromhex("00000003 01 3e800000 bf000000 3e800000 3f000000 be800000 ff")
    assert phone.publish(wire)
    assert phone.sample.lb
    assert motion_delta(phone.sample.gamepad) == pytest.approx([.0005, -.001, .0005, -.1, .375, -1.])
    assert phone.telemetry()["wrist"] == (.5, -.25)
    assert phone.telemetry()["gripper"] == -1
    assert phone.publish(arm_frame(4, STOP))
    assert phone.sample.stop == 1 and phone.sample.neutral and not phone.sample.lb
    for seq in range(5, 125):
        now += .02
        assert phone.publish(arm_frame(seq, STOP))
        assert phone.sample.stop_since is None
    assert phone.publish(arm_frame(125, DEADMAN, gripper=1))
    assert not phone.sample.lb
    assert phone.publish(arm_frame(126, ADVANCE))
    assert phone.publish(arm_frame(127, DEADMAN, gripper=1))
    assert phone.sample.lb and motion_delta(phone.sample.gamepad)[5] == 1
    now += FRESHNESS_S + .01
    with pytest.raises(TimeoutError, match="Input expired"):
        phone.publish(arm_frame(128, DEADMAN))
    phone.end()
    phone.begin(arm_controls=True)
    assert phone.publish(arm_frame(0))
    assert phone.publish(arm_frame(1, DEADMAN, pitch=1))
    assert not phone.sample.lb and phone.sample.start == 2


@pytest.mark.parametrize("frame", [
    INPUT_FRAME.pack(1, 0, 0, 0, 0), arm_frame(1, DEADMAN | ADVANCE), arm_frame(1, STOP | ADVANCE),
    arm_frame(1, STOP, gripper=1), arm_frame(1, ADVANCE, roll=.1), arm_frame(1, pitch=float("nan")),
    arm_frame(1, roll=1.1), arm_frame(1, gripper=2), arm_frame(1, flags=8),
])
def test_arm_input_rejects_invalid_frames_without_refreshing(frame):
    phone = MobileInput()
    phone.begin(arm_controls=True)
    with pytest.raises(ValueError, match="sequence-zero"):
        phone.publish(arm_frame(1, DEADMAN))
    assert phone.publish(arm_frame(0))
    before = phone.sample
    with pytest.raises(ValueError):
        phone.publish(frame)
    assert phone.sample is before


def test_arm_socket_reports_simulator_feedback_without_adding_messages_on_v1():
    phone = MobileInput()
    arm_state = "Holding"
    headers = {"Authorization": "Bearer test-token"}
    with MobileGateway("127.0.0.1", 0, phone, "test-token", arm_status=lambda: arm_state) as gateway:
        base_url = f"ws://127.0.0.1:{gateway.port}"
        with connect(base_url + ARM_INPUT_PATH, additional_headers=headers, close_timeout=.2, proxy=None) as client:
            client.send(arm_frame(0))
            assert client.recv(timeout=.5) == "arm-status:Holding"
            arm_state = "Control enabled"
            for seq in range(1, 15):
                client.send(arm_frame(seq))
                sleep(.012)
            assert client.recv(timeout=.5) == "arm-status:Control enabled"
            assert gateway.telemetry()["phone_mode"] == "arm"
        deadline = monotonic() + 1
        while gateway.active and monotonic() < deadline:
            sleep(.001)
        assert not gateway.active
        with connect(base_url + INPUT_PATH, additional_headers=headers, close_timeout=.2, proxy=None) as client:
            client.send(INPUT_FRAME.pack(0, 0, 0, 0, 0))
            with pytest.raises(TimeoutError):
                client.recv(timeout=.02)


def test_expiry_between_ik_samples_requires_new_connection_and_advance():
    now = 1.
    phone = MobileInput(clock=lambda: now)
    assert phone.publish(INPUT_FRAME.pack(0, 0, 0, 0, 0))
    assert phone.publish(INPUT_FRAME.pack(1, ADVANCE, 0, 0, 0))
    assert phone.publish(INPUT_FRAME.pack(2, DEADMAN, 1, -1, .5))
    before = phone.sample
    assert before.start == 1 and before.lb
    assert motion_delta(before.gamepad) == pytest.approx([.002 / 1.5, -.002 / 1.5, .001 / 1.5, 0, 0, 0])

    now += FRESHNESS_S + .01
    with pytest.raises(TimeoutError, match="expired"):
        phone.publish(INPUT_FRAME.pack(3, DEADMAN, 1, 0, 0))
    phone.end()
    phone.begin()
    assert phone.publish(INPUT_FRAME.pack(0, DEADMAN, 1, 0, 0))
    assert phone.sample.disconnect == before.disconnect + 1
    assert phone.sample.start == before.start
    assert phone.publish(INPUT_FRAME.pack(1, 0, 0, 0, 0))
    assert phone.publish(INPUT_FRAME.pack(2, ADVANCE, 0, 0, 0))
    assert phone.sample.start == before.start + 1


@pytest.mark.parametrize("frame", ["{}", b"\x00", INPUT_FRAME.pack(1, 4, 0, 0, 0),
                                   INPUT_FRAME.pack(1, 0, float("nan"), 0, 0),
                                   INPUT_FRAME.pack(1, ADVANCE | DEADMAN, 0, 0, 0)])
def test_invalid_frames_do_not_refresh_or_advance_input(frame):
    phone = MobileInput()
    assert phone.publish(INPUT_FRAME.pack(0, 0, 0, 0, 0))
    before = phone.sample
    with pytest.raises(ValueError):
        phone.publish(frame)
    assert phone.sample is before


def test_websocket_auth_exclusivity_and_timeout():
    phone = MobileInput()
    token = "mobile-test-token"
    headers = {"Authorization": f"Bearer {token}"}
    with MobileGateway("127.0.0.1", 0, phone, token) as gateway:
        url = f"ws://127.0.0.1:{gateway.port}{INPUT_PATH}"
        with pytest.raises(InvalidStatus) as rejected:
            connect(url, open_timeout=2, proxy=None)
        assert rejected.value.response.status_code == 401

        with connect(url, additional_headers=headers, open_timeout=2, close_timeout=.2, proxy=None) as client:
            deadline = monotonic() + 1
            while not gateway.active and monotonic() < deadline:
                sleep(.001)
            with pytest.raises(InvalidStatus) as rejected:
                connect(url, additional_headers=headers, open_timeout=2, proxy=None)
            assert rejected.value.response.status_code == 409
            first = INPUT_FRAME.pack(0, DEADMAN, .25, 0, 0)
            client.send(first)
            client.send(first)  # A duplicate cannot keep the stream alive.
            with pytest.raises(ConnectionClosedError) as expired:
                client.recv(timeout=1)
            assert expired.value.rcvd.code == 1008
            assert phone.telemetry() == {"seq": 0, "fresh": False, "deadman": False, "xyz": (0., 0., 0.)}

        with connect(url, additional_headers=headers, open_timeout=2, close_timeout=.2, proxy=None) as client:
            client.send(INPUT_FRAME.pack(0, 0, 0, .25, 0))
            deadline = monotonic() + 1
            while not phone.telemetry()["fresh"] and monotonic() < deadline:
                sleep(.001)
            assert phone.telemetry()["xyz"] == (0., .25, 0.)
    assert not phone.telemetry()["fresh"]


def test_marker_pose_preserves_units_and_expires_without_a_new_sequence():
    now = 1.
    feed = MarkerPose(clock=lambda: now)
    feed.begin()
    frame = POSE_FRAME.pack(5, TRACKING, -12.5, 18, 55, 9.5, -20, 170)
    assert feed.publish(frame)
    assert feed.telemetry() == {"state": "Tracking", "seq": 5, "frame": "camera",
                                "xyz_units": "cm",
                                "xyz_cm": (-12.5, 18., 55.), "rpy_deg": (9.5, -20., 170.)}

    now += POSE_STALE_S / 2
    assert not feed.publish(frame)
    assert not feed.publish(POSE_FRAME.pack(4, 0, *([0.] * 6)))
    now += POSE_STALE_S
    assert feed.telemetry() == {"state": "Pose timed out", "seq": 5, "frame": "camera",
                                "xyz_units": "cm",
                                "xyz_cm": (-12.5, 18., 55.), "rpy_deg": (9.5, -20., 170.)}

    # A stationary marker can report identical coordinates with a new sequence.
    assert feed.publish(POSE_FRAME.pack(6, TRACKING, -12.5, 18, 55, 9.5, -20, 170))
    assert feed.telemetry()["state"] == "Tracking"


def test_marker_loss_retains_detected_pose_until_reacquisition_or_disconnect():
    feed = MarkerPose(clock=lambda: 1.)
    feed.begin("robot-axes", "normalized")
    assert feed.telemetry()["xyz_cm"] is None
    assert feed.publish(POSE_FRAME.pack(0, 0, *([0.] * 6)))
    assert feed.telemetry() == {"state": "Marker lost", "seq": 0, "frame": "robot-axes",
                                "xyz_units": "normalized",
                                "xyz_cm": None, "rpy_deg": None}

    assert feed.publish(POSE_FRAME.pack(1, TRACKING, .125, -.25, .5, 9.5, -20, 170))
    for seq in (2, 3):
        assert feed.publish(POSE_FRAME.pack(seq, 0, *([0.] * 6)))
        assert feed.telemetry() == {"state": "Marker lost", "seq": seq, "frame": "robot-axes",
                                    "xyz_units": "normalized",
                                    "xyz_cm": (.125, -.25, .5), "rpy_deg": (9.5, -20., 170.)}

    assert feed.publish(POSE_FRAME.pack(4, TRACKING, .25, -.5, .75, 10, -15, 90))
    assert feed.telemetry() == {"state": "Tracking", "seq": 4, "frame": "robot-axes",
                                "xyz_units": "normalized",
                                "xyz_cm": (.25, -.5, .75), "rpy_deg": (10., -15., 90.)}
    feed.end()
    assert feed.telemetry()["state"] == "Disconnected"
    assert feed.telemetry()["frame"] is None
    assert feed.telemetry()["xyz_units"] is None
    assert feed.telemetry()["xyz_cm"] is None
    assert feed.telemetry()["rpy_deg"] is None
    feed.begin()
    assert feed.telemetry() == {"state": "Waiting for marker pose", "seq": None, "frame": "camera",
                                "xyz_units": "cm", "xyz_cm": None, "rpy_deg": None}
    assert feed.publish(POSE_FRAME.pack(0, TRACKING, -12.5, 18, 55, 9.5, -20, 170))
    assert feed.telemetry()["xyz_cm"] == (-12.5, 18., 55.)


@pytest.mark.parametrize("frame", [INPUT_FRAME.pack(0, 0, 0, 0, 0),
                                   POSE_FRAME.pack(0, 2, *([0.] * 6)),
                                   POSE_FRAME.pack(0, TRACKING, float("inf"), 0, 0, 0, 0, 0)])
def test_marker_pose_rejects_invalid_frames(frame):
    feed = MarkerPose()
    feed.begin()
    with pytest.raises(ValueError):
        feed.publish(frame)
    assert feed.telemetry()["seq"] is None


def test_marker_websocket_reports_frame_and_xyz_units_across_reconnect():
    phone = MobileInput()
    auth = ("Authorization", "Bearer mobile-test-token")
    frame_header = "X-Marker-Pose-Frame"
    robot_header = (frame_header, "robot-axes")
    units_header = "X-Marker-Pose-XYZ-Units"
    normalized = (units_header, "normalized")
    centimeters = (units_header, "cm")
    with MobileGateway("127.0.0.1", 0, phone, "mobile-test-token") as gateway:
        url = f"ws://127.0.0.1:{gateway.port}{POSE_PATH}"

        for invalid_headers in ([(frame_header, "unknown")], [robot_header, robot_header],
                                [(units_header, "mm")], [normalized, normalized]):
            with pytest.raises(InvalidStatus) as rejected:
                connect(url, additional_headers=[auth, *invalid_headers], open_timeout=2, proxy=None)
            assert rejected.value.response.status_code == 400

        for headers, frame, units in (([], "camera", "cm"),
                                      ([robot_header, normalized], "robot-axes", "normalized"),
                                      ([robot_header, centimeters], "robot-axes", "cm")):
            with connect(url, additional_headers=[auth, *headers], open_timeout=2, proxy=None) as client:
                client.send(POSE_FRAME.pack(0, TRACKING, .125, -.25, .5, 30, 9, -45))
                deadline = monotonic() + 1
                while gateway.marker_pose.telemetry()["state"] != "Tracking" and monotonic() < deadline:
                    sleep(.001)
                assert gateway.marker_pose.telemetry() == {
                    "state": "Tracking", "seq": 0, "frame": frame, "xyz_units": units,
                    "xyz_cm": (.125, -.25, .5), "rpy_deg": (30., 9., -45.),
                }

            deadline = monotonic() + 1
            while gateway.marker_pose.telemetry()["frame"] is not None and monotonic() < deadline:
                sleep(.001)
            assert gateway.marker_pose.telemetry()["frame"] is None
            assert gateway.marker_pose.telemetry()["xyz_units"] is None


def test_marker_websocket_keeps_pose_out_of_control_input():
    phone = MobileInput()
    initial_input = phone.sample
    headers = {"Authorization": "Bearer mobile-test-token"}
    with MobileGateway("127.0.0.1", 0, phone, "mobile-test-token") as gateway:
        base_url = f"ws://127.0.0.1:{gateway.port}"
        with pytest.raises(InvalidStatus) as rejected:
            connect(base_url + POSE_PATH, open_timeout=2, proxy=None)
        assert rejected.value.response.status_code == 401

        with connect(base_url + POSE_PATH, additional_headers=headers, open_timeout=2, proxy=None) as client:
            client.send(POSE_FRAME.pack(0, TRACKING, 15, -12, 50, 30, 9, -45))
            deadline = monotonic() + 1
            while gateway.marker_pose.telemetry()["seq"] is None and monotonic() < deadline:
                sleep(.001)
            assert gateway.marker_pose.telemetry()["rpy_deg"] == (30., 9., -45.)
            sleep(FRESHNESS_S + .025)
            # The monitoring stream stays open across a velocity watchdog interval.
            assert client.ping().wait(1)
            assert phone.sample is initial_input
            with pytest.raises(InvalidStatus) as rejected:
                connect(base_url + INPUT_PATH, additional_headers=headers, open_timeout=2, proxy=None)
            assert rejected.value.response.status_code == 409


@pytest.mark.parametrize("frame", [
    "{}", b"\x00", arm_frame(0), INPUT_FRAME.pack(2, DEADMAN, 0, 0, 0),
    INPUT_FRAME.pack(2, ADVANCE, 0, 0, 0), INPUT_FRAME.pack(2, STOP, 0, 0, 0),
    INPUT_FRAME.pack(2, 0, float("nan"), 0, 0), INPUT_FRAME.pack(2, 0, 0, 1.1, 0),
])
def test_touchscreen_rejects_invalid_frames_without_refreshing(frame):
    now = 1.
    touchscreen = TouchscreenInput(clock=lambda: now)
    assert touchscreen.publish(INPUT_FRAME.pack(1, 0, .25, 0, 0))
    now += TOUCHSCREEN_STALE_S / 2
    with pytest.raises(ValueError):
        touchscreen.publish(frame)
    now += TOUCHSCREEN_STALE_S / 2
    assert touchscreen.telemetry() == {"seq": 1, "fresh": False, "deadman": False, "xyz": (0., 0., 0.)}


def wait_for_telemetry(gateway, predicate):
    deadline = monotonic() + 1
    while monotonic() < deadline:
        readings = gateway.telemetry()
        if predicate(readings):
            return readings
        sleep(.001)
    pytest.fail(f"Unexpected gateway telemetry {gateway.telemetry()}")


def test_touchscreen_socket_recovers_after_stale_input_and_expires_without_a_new_sequence():
    motion = MobileInput()
    initial_motion = motion.sample
    headers = {"Authorization": "Bearer mobile-test-token"}
    with MobileGateway("127.0.0.1", 0, motion, "mobile-test-token") as gateway:
        base = f"ws://127.0.0.1:{gateway.port}"
        with pytest.raises(InvalidStatus) as rejected:
            connect(base + TOUCHSCREEN_PATH, open_timeout=2, proxy=None)
        assert rejected.value.response.status_code == 401

        with connect(base + TOUCHSCREEN_PATH, additional_headers=headers, open_timeout=2,
                     close_timeout=.2, proxy=None) as client:
            client.send(INPUT_FRAME.pack(0, 0, .25, 0, 0))
            first = wait_for_telemetry(gateway, lambda readings: readings["phone"]["seq"] == 0)
            assert first["phone_mode"] == "xyz"
            assert first["phone"]["xyz"] == (.25, 0., 0.)
            with pytest.raises(InvalidStatus) as rejected:
                connect(base + ARM_INPUT_PATH, additional_headers=headers, open_timeout=2, proxy=None)
            assert rejected.value.response.status_code == 409

            sleep(.15)
            client.send(INPUT_FRAME.pack(1, 0, 0, .5, 0))
            receiving = wait_for_telemetry(gateway, lambda readings: readings["phone"]["seq"] == 1)
            assert receiving["phone"]["fresh"] and receiving["phone"]["xyz"] == (0., .5, 0.)

            stale = wait_for_telemetry(gateway, lambda readings: not readings["phone"]["fresh"])
            assert stale["phone_mode"] == "xyz"
            assert stale["phone"] == {"seq": 1, "fresh": False, "deadman": False, "xyz": (0., 0., 0.)}

            client.send(INPUT_FRAME.pack(2, 0, 0, 0, -.5))
            recovered = wait_for_telemetry(gateway, lambda readings: readings["phone"]["seq"] == 2)
            assert recovered["phone"]["fresh"] and recovered["phone"]["xyz"] == (0., 0., -.5)
            assert motion.sample is initial_motion

            wait_for_telemetry(gateway, lambda readings: not readings["phone"]["fresh"])
            client.send(INPUT_FRAME.pack(2, 0, 1, 0, 0))
            client.send(INPUT_FRAME.pack(1, 0, 1, 0, 0))
            with pytest.raises(ConnectionClosedError) as expired:
                client.recv(timeout=TOUCHSCREEN_IDLE_S - TOUCHSCREEN_STALE_S / 2)
            assert expired.value.rcvd.code == 1008
            assert expired.value.rcvd.reason == "Input expired"

        assert not gateway.active
        assert motion.sample is initial_motion


def test_websocket_dashboard_mode_follows_the_active_endpoint():

    headers = {"Authorization": "Bearer mobile-test-token"}
    with MobileGateway("127.0.0.1", 0, MobileInput(), "mobile-test-token") as gateway:
        base = f"ws://127.0.0.1:{gateway.port}"

        def dial(path):
            return connect(base + path, additional_headers=headers, open_timeout=2, close_timeout=.2, proxy=None)

        assert gateway.telemetry()["phone_mode"] is None
        with dial(POSE_PATH) as pose:
            waiting = wait_for_telemetry(gateway, lambda readings: readings["phone_mode"] == "aruco")
            assert waiting["marker_pose"]["state"] == "Waiting for marker pose"
            pose.send(POSE_FRAME.pack(0, TRACKING, 12., 15., 18., 6., 8., 10.))
            pose.send(POSE_FRAME.pack(1, 0, 0., 0., 0., 0., 0., 0.))
            lost = wait_for_telemetry(gateway, lambda readings: readings["marker_pose"]["seq"] == 1)
            assert lost["phone_mode"] == "aruco"
            assert lost["marker_pose"]["state"] == "Marker lost"
            assert lost["marker_pose"]["rpy_deg"] == (6., 8., 10.)
        wait_for_telemetry(gateway, lambda readings: readings["phone_mode"] is None)

        with dial(INPUT_PATH) as xyz:
            waiting = wait_for_telemetry(gateway, lambda readings: readings["phone_mode"] == "xyz")
            assert waiting["phone"]["seq"] is None
            assert waiting["marker_pose"]["xyz_cm"] is None
            xyz.send(INPUT_FRAME.pack(0, 0, .25, 0., 0.))
            receiving = wait_for_telemetry(gateway, lambda readings: readings["phone"]["fresh"])
            assert receiving["phone_mode"] == "xyz"
            assert receiving["phone"]["xyz"] == (.25, 0., 0.)
        wait_for_telemetry(gateway, lambda readings: readings["phone_mode"] is None)

        with dial(POSE_PATH):
            waiting = wait_for_telemetry(gateway, lambda readings: readings["phone_mode"] == "aruco")
            assert waiting["marker_pose"]["seq"] is None
            assert waiting["marker_pose"]["xyz_cm"] is None
            assert not waiting["phone"]["fresh"]
