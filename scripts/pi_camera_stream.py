"""Serve the CSI camera as MJPEG on the Pi's private LAN address."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BufferedIOBase
import socket
import sys
from threading import Condition
from time import monotonic


FRAME_TIMEOUT = 2
MAX_FRAME_BYTES = 4 * 1024 * 1024
BOUNDARY = "lerobot-rpi-camera"
SENSOR_SIZE = (1296, 972)


class LatestFrame(BufferedIOBase):
    """Receive one JPEG per encoder write and retain only the newest frame."""

    def __init__(self):
        super().__init__()
        self._condition = Condition()
        self._sequence = 0
        self._jpeg = b""
        self._received_at = float("-inf")

    def write(self, frame):
        if 4 <= len(frame) <= MAX_FRAME_BYTES and frame[:2] == b"\xff\xd8" and frame[-2:] == b"\xff\xd9":
            with self._condition:
                self._jpeg = bytes(frame)
                self._received_at = monotonic()
                self._sequence += 1
                self._condition.notify_all()
        return len(frame)

    def _fresh(self):
        return monotonic() - self._received_at < FRAME_TIMEOUT

    def is_live(self):
        with self._condition:
            return self._fresh()

    def wait_for_frame(self, after):
        with self._condition:
            if self._condition.wait_for(lambda: self._sequence != after and self._fresh(), timeout=FRAME_TIMEOUT):
                return self._sequence, self._jpeg
        return None


class CameraServer(ThreadingHTTPServer):
    def __init__(self, address, frames):
        self.frames = frames
        super().__init__(address, CameraHandler)


class CameraHandler(BaseHTTPRequestHandler):
    def setup(self):
        self.request.settimeout(FRAME_TIMEOUT)
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        super().setup()

    def log_message(self, format, *args):
        # Polling and normal viewer reconnects do not need journal entries.
        pass

    def do_GET(self):
        try:
            if self.path == "/health":
                live = self.server.frames.is_live()
                body = b"lerobot-rpi-camera\n" if live else b"No live frames\n"
                self.send_response(200 if live else 503)
                self.send_header("Content-Type", "text/plain")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self._write_frames()
            else:
                self.send_error(404)
        except (ConnectionError, TimeoutError):
            # A slow or disconnected viewer can reconnect without affecting capture.
            pass

    def _write_frames(self):
        sequence = 0
        while (frame := self.server.frames.wait_for_frame(sequence)) is not None:
            sequence, jpeg = frame
            self.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                             f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
            self.wfile.write(jpeg)
            self.wfile.write(b"\r\n")
        self.wfile.write(f"--{BOUNDARY}--\r\n".encode("ascii"))


def main():
    from picamera2 import Picamera2
    from picamera2.encoders import JpegEncoder, MJPEGEncoder
    from picamera2.outputs import FileOutput

    host, port, width, height, fps = sys.argv[1:]
    size = (int(width), int(height))
    fps = int(fps)
    cameras = Picamera2.global_camera_info()
    # Picamera2 uses this ID convention to identify USB cameras in its ordered list.
    camera_index = next((index for index, info in enumerate(cameras) if "/usb" not in info["Id"]), None)
    if camera_index is None:
        raise RuntimeError("No CSI camera detected. Check rpicam-hello --list-cameras on the Pi.")

    pixel_format = "YUV420"
    try:
        encoder = MJPEGEncoder()
    except RuntimeError as exc:
        if "Hardware MJPEG not available" not in str(exc):
            raise
        # Older Picamera2 releases need the software JPEG encoder on Pi 5.
        encoder = JpegEncoder()
        pixel_format = "XBGR8888"
    else:
        encoder.framerate = fps

    frames = LatestFrame()
    with CameraServer((host, int(port)), frames) as server, Picamera2(camera_index) as camera:
        config = camera.create_video_configuration(
            main={"size": size, "format": pixel_format},
            sensor={"output_size": SENSOR_SIZE, "bit_depth": 10},
            controls={"FrameRate": fps},
            display=None,
        )
        camera.start_recording(encoder, FileOutput(frames), config=config)
        try:
            server.serve_forever()
        finally:
            camera.stop_recording()


if __name__ == "__main__":
    main()
