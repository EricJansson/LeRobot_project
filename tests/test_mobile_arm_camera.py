import json
from unittest.mock import Mock

import pytest

from scripts import mobile_arm_camera as camera


PI = "eric@192.168.0.190"
CONNECTION = "192.168.0.10 56789 192.168.0.190 22"


def test_reuses_live_camera_without_starting_a_process(monkeypatch):
    ssh = Mock(return_value=CONNECTION)
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_camera_status", lambda _: True)

    assert camera.ensure_arm_camera(PI, "/dev/video0") == "http://192.168.0.190:8080/stream?extra_headers=1"
    ssh.assert_called_once_with(PI, 'printf "%s\\n" "$SSH_CONNECTION"')


def test_starts_a_user_service_and_waits_for_frames(monkeypatch):
    ssh = Mock(side_effect=[CONNECTION, ""])
    state = Mock(side_effect=[None, False, True])
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_camera_status", state)
    monkeypatch.setattr(camera, "sleep", Mock())

    device = "/dev/v4l/by-id/USB camera"
    camera.ensure_arm_camera(PI, device)

    host, command = ssh.call_args.args
    assert host == PI
    assert "systemctl --user is-active --quiet lerobot-arm-camera.service" in command
    assert "systemd-run --user --collect --service-type=exec --unit=lerobot-arm-camera.service" in command
    assert "'--device=/dev/v4l/by-id/USB camera'" in command
    assert "--host=192.168.0.190 --port=8080" in command
    assert state.call_count == 3


def test_reports_offline_camera_without_replacing_existing_server(monkeypatch):
    ssh = Mock(return_value=CONNECTION)
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_camera_status", lambda _: False)
    monkeypatch.setattr(camera, "monotonic", Mock(side_effect=[0, 6]))

    with pytest.raises(RuntimeError, match="/dev/video0 is producing no video"):
        camera.ensure_arm_camera(PI, "/dev/video0")
    assert ssh.call_count == 1


def test_rejects_a_non_lan_pi_address_before_starting_camera(monkeypatch):
    ssh = Mock(return_value="192.168.0.10 56789 203.0.113.9 22")
    monkeypatch.setattr(camera, "_ssh", ssh)

    with pytest.raises(RuntimeError, match="private LAN IPv4 address"):
        camera.ensure_arm_camera(PI, "/dev/video0")
    assert ssh.call_count == 1


@pytest.mark.parametrize("source,expected", [
    ({"online": True, "captured_fps": 30}, True),
    ({"online": False, "captured_fps": 30}, False),
    ({"online": True, "captured_fps": 0}, False),
    ({"online": True}, None),
])
def test_state_requires_online_source_with_a_nonzero_frame_rate(monkeypatch, source, expected):
    connection = Mock()
    response = connection.getresponse.return_value
    response.status = 200
    response.read.return_value = json.dumps({"ok": True, "result": {"source": source}}).encode()
    monkeypatch.setattr(camera, "HTTPConnection", Mock(return_value=connection))

    assert camera._camera_status("192.168.0.190") is expected
    connection.request.assert_called_once_with("GET", "/state")
    connection.close.assert_called_once()
