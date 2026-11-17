from time import monotonic, sleep
from unittest.mock import MagicMock

import pytest

pytest.importorskip("websockets", minversion="16")
from websockets.exceptions import ConnectionClosedError, InvalidStatus
from websockets.sync.client import connect

from robot_program.remote_ik_control import motion_delta
from scripts import main_mobile_teleop as launcher
from scripts.mobile_teleop_gateway import (
    ADVANCE, DEADMAN, FRESHNESS_S, INPUT_FRAME, INPUT_PATH, POSE_FRAME, POSE_PATH, POSE_STALE_S,
    TRACKING, MarkerPose, MobileGateway, MobileInput,
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
