import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from scripts import mobile_arm_camera as camera


PI = "eric@192.168.0.190"
CONNECTION = "192.168.0.10 56789 192.168.0.190 22"


def test_usb_service_reuses_matching_profile_or_manual_stream(monkeypatch):
    ssh = Mock(side_effect=[CONNECTION, ""])
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_camera_status", lambda _: True)

    assert camera.ensure_arm_camera(PI, "/dev/video0") == "http://192.168.0.190:8080/stream?extra_headers=1"
    command = ssh.call_args.args[1]
    assert 'case " $environment " in *" LEROBOT_CAMERA_PROFILE=control "*) exit 0;; esac' in command
    assert 'systemctl --user stop lerobot-arm-camera.service; else exit 0; fi;' in command


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
    assert "--resolution=640x480 --desired-fps=15" in command
    assert "--setenv=LEROBOT_CAMERA_PROFILE=control --property=Nice=5" in command
    assert state.call_count == 3


def test_usb_camera_defaults_to_the_stable_innomaker_capture_device(monkeypatch):
    ssh = Mock(side_effect=[CONNECTION, ""])
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_camera_status", Mock(side_effect=[None, True]))
    monkeypatch.setattr(camera, "sleep", Mock())

    camera.ensure_arm_camera(PI, None)

    command = ssh.call_args.args[1]
    assert "/dev/v4l/by-id/usb-*Innomaker*-video-index0" in command
    assert 'device=/dev/video0;' in command
    assert '"--device=$device"' in command


def test_waits_for_frames_from_an_existing_stream(monkeypatch):
    ssh = Mock(side_effect=[CONNECTION, ""])
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_camera_status", lambda _: False)
    monkeypatch.setattr(camera, "monotonic", Mock(side_effect=[0, 6]))

    with pytest.raises(RuntimeError, match="/dev/video0 is producing no video"):
        camera.ensure_arm_camera(PI, "/dev/video0")
    assert 'else exit 0; fi;' in ssh.call_args.args[1]


@pytest.mark.parametrize("start,args", [(camera.ensure_arm_camera, ("/dev/video0",)), (camera.ensure_rpi_camera, ())])
def test_rejects_a_non_lan_pi_address_before_starting_camera(monkeypatch, start, args):
    ssh = Mock(return_value="192.168.0.10 56789 203.0.113.9 22")
    monkeypatch.setattr(camera, "_ssh", ssh)

    with pytest.raises(RuntimeError, match="private LAN IPv4 address"):
        start(PI, *args)
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


def test_rpi_camera_reuses_live_service_with_matching_profile(monkeypatch):
    ssh = Mock(side_effect=[CONNECTION, ""])
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_rpi_camera_is_live", lambda _: True)

    assert camera.ensure_rpi_camera(PI) == "http://192.168.0.190:8081/stream"
    command = ssh.call_args.args[1]
    assert 'case " $environment " in *" LEROBOT_CAMERA_PROFILE=control "*) exit 0;; esac' in command
    assert 'systemctl --user stop lerobot-rpi-camera.service; else exit 0; fi;' in command


def test_rpi_camera_uploads_then_verifies_the_separate_user_service(monkeypatch):
    ssh = Mock(side_effect=[CONNECTION, ""])
    live = Mock(side_effect=[False, False, True])
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_rpi_camera_is_live", live)
    monkeypatch.setattr(camera, "sleep", Mock())

    assert camera.ensure_rpi_camera(PI) == "http://192.168.0.190:8081/stream"

    host, command = ssh.call_args.args
    assert host == PI
    assert "systemctl --user is-active --quiet lerobot-rpi-camera.service" in command
    assert 'cat > "$tmp"; mv -f -- "$tmp" "$dir/pi_camera_stream.py"' in command
    assert "systemd-run --user --collect --service-type=exec --unit=lerobot-rpi-camera.service" in command
    assert '/usr/bin/python3 "$dir/pi_camera_stream.py" 192.168.0.190 8081 640 480 15' in command
    assert ssh.call_args.kwargs["data"] == Path(camera.__file__).with_name("pi_camera_stream.py").read_bytes()
    assert live.call_count == 3


def test_balanced_profile_preserves_the_original_feed_sizes_and_rates(monkeypatch):
    ssh = Mock(side_effect=[CONNECTION, "", CONNECTION, ""])
    monkeypatch.setattr(camera, "_ssh", ssh)
    monkeypatch.setattr(camera, "_camera_status", lambda _: True)
    monkeypatch.setattr(camera, "_rpi_camera_is_live", lambda _: True)

    camera.ensure_arm_camera(PI, None, profile="balanced")
    camera.ensure_rpi_camera(PI, profile="balanced")

    usb_command = ssh.call_args_list[1].args[1]
    csi_command = ssh.call_args_list[3].args[1]
    assert "--resolution=1280x720 --desired-fps=30" in usb_command
    assert '192.168.0.190 8081 1296 972 45' in csi_command
    assert "--setenv=LEROBOT_CAMERA_PROFILE=balanced" in usb_command
    assert "--setenv=LEROBOT_CAMERA_PROFILE=balanced" in csi_command


def test_rpi_camera_times_out_with_diagnostic_when_no_frames_arrive(monkeypatch):
    monkeypatch.setattr(camera, "_ssh", Mock(side_effect=[CONNECTION, ""]))
    monkeypatch.setattr(camera, "_rpi_camera_is_live", lambda _: False)
    monkeypatch.setattr(camera, "monotonic", Mock(side_effect=[0, 6]))

    with pytest.raises(RuntimeError, match=r"No live frames.*journalctl.*lerobot-rpi-camera.service"):
        camera.ensure_rpi_camera(PI)
