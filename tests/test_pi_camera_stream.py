from contextlib import closing
from http.client import HTTPConnection
import sys
from threading import Thread
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

from scripts import mobile_arm_camera, pi_camera_stream


def test_mjpeg_serves_latest_frames_and_closes_when_capture_stalls(monkeypatch):
    monkeypatch.setattr(pi_camera_stream, "FRAME_TIMEOUT", 1)
    frames = pi_camera_stream.LatestFrame()
    first = b"\xff\xd8older\xff\xd9"
    latest = b"\xff\xd8latest\xff\xd9"
    next_frame = b"\xff\xd8next\xff\xd9"
    boundary = pi_camera_stream.BOUNDARY
    with pi_camera_stream.CameraServer(("127.0.0.1", 0), frames) as server:
        monkeypatch.setattr(mobile_arm_camera, "RPI_CAMERA_PORT", server.server_port)
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            frames.write(b"incomplete frame")
            assert not mobile_arm_camera._rpi_camera_is_live("127.0.0.1")
            frames.write(first)
            frames.write(latest)
            assert mobile_arm_camera._rpi_camera_is_live("127.0.0.1")
            with closing(HTTPConnection("127.0.0.1", server.server_port, timeout=3)) as connection:
                connection.request("GET", "/stream")
                response = connection.getresponse()
                assert response.status == 200
                assert response.getheader("Content-Type") == f"multipart/x-mixed-replace; boundary={boundary}"
                assert response.read(len(part(boundary, latest))) == part(boundary, latest)
                frames.write(next_frame)
                assert response.read(len(part(boundary, next_frame))) == part(boundary, next_frame)
                assert response.read() == f"--{boundary}--\r\n".encode("ascii")
            assert not mobile_arm_camera._rpi_camera_is_live("127.0.0.1")
        finally:
            server.shutdown()
            worker.join(2)


def part(boundary, jpeg):
    return (f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
            + jpeg + b"\r\n")


def test_camera_module_uses_csi_and_balanced_capture_configuration(monkeypatch):
    camera_class = MagicMock()
    camera_class.global_camera_info.return_value = [{"Id": "/sys/usb/innomaker"}, {"Id": "/base/i2c/imx219"}]
    camera = camera_class.return_value.__enter__.return_value
    encoder = Mock()
    monkeypatch.setitem(sys.modules, "picamera2", SimpleNamespace(Picamera2=camera_class))
    monkeypatch.setitem(sys.modules, "picamera2.encoders", SimpleNamespace(JpegEncoder=Mock(), MJPEGEncoder=lambda: encoder))
    monkeypatch.setitem(sys.modules, "picamera2.outputs", SimpleNamespace(FileOutput=lambda frames: frames))
    server_class = MagicMock()
    monkeypatch.setattr(pi_camera_stream, "CameraServer", server_class)
    monkeypatch.setattr(sys, "argv", ["pi_camera_stream.py", "192.168.0.190", "8081"])

    pi_camera_stream.main()

    camera_class.assert_called_once_with(1)
    camera.create_video_configuration.assert_called_once_with(
        main={"size": (1296, 972), "format": "YUV420"},
        sensor={"output_size": (1296, 972), "bit_depth": 10},
        controls={"FrameRate": 45},
        display=None,
    )
    assert encoder.framerate == 45
    address, frames = server_class.call_args.args
    assert address == ("192.168.0.190", 8081)
    camera.start_recording.assert_called_once_with(encoder, frames, config=camera.create_video_configuration.return_value)
    camera.stop_recording.assert_called_once_with()
