"""Serve read-only LeRobot IK telemetry on a trusted local network."""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import sys
from threading import Event, Thread
from time import monotonic


class DashboardHandler(BaseHTTPRequestHandler):
    timeout = 3

    def do_GET(self):
        if self.path == "/":
            self.respond(HTTPStatus.OK, "text/html; charset=utf-8", self.server.page)
        elif self.path == "/api/telemetry":
            payload = json.dumps(self.server.telemetry()).encode()
            self.respond(HTTPStatus.OK, "application/json", payload)
        elif self.path == "/api/exit":
            self.watch_exit()
        else:
            self.respond(HTTPStatus.NOT_FOUND, "text/plain", b"Not found")

    def do_POST(self):
        self.respond(HTTPStatus.METHOD_NOT_ALLOWED, "text/plain", b"Read-only dashboard")

    def watch_exit(self):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            # An open stream can deliver the final message even when the
            # browser has slowed its telemetry polling in a background tab.
            while not self.server.closing.is_set():
                self.wfile.write(b": waiting for exit\n\n")
                self.server.closing.wait(5)
            if self.server.exit_reason is not None:
                reason = json.dumps(self.server.exit_reason).encode()
                self.wfile.write(b"event: stopped\ndata: " + reason + b"\n\n")
        except OSError:
            pass  # Browser closed the stream.

    def respond(self, status, content_type, body):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if status == HTTPStatus.METHOD_NOT_ALLOWED:
            self.send_header("Allow", "GET")
        self.end_headers()
        self.wfile.write(body)

    def log_request(self, code="-", size="-"):
        pass  # Routine polling should not fill the teleop console.


class WebDashboard(ThreadingHTTPServer):
    allow_reuse_address = sys.platform != "win32"
    daemon_threads = False  # Flush the exit event before the process ends.

    def __init__(self, host, port):
        self.page = Path(__file__).with_suffix(".html").read_bytes()
        self._sample = None
        self.exit_reason = None
        self.closing = Event()
        super().__init__((host, port), DashboardHandler)

    @property
    def port(self):
        return self.server_port

    def server_bind(self):
        if sys.platform == "win32":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def publish(self, joints, gripper, xyz_m, gamepad_enabled, *, gamepad=None, remote=None):
        # Replace the entire snapshot. HTTP handlers never mutate it.
        readings = {
            "joints_deg": dict(joints),
            "gripper_normalized": gripper,
            "xyz_m": dict(xyz_m),
            "gamepad_enabled": gamepad_enabled,
        }
        if gamepad is not None:
            readings["gamepad"] = {
                **gamepad,
                "axes": dict(gamepad["axes"]),
                "buttons": dict(gamepad["buttons"]),
                "dpad": list(gamepad["dpad"]),
            }
        if remote is not None:
            readings["remote"] = dict(remote)
        self._sample = (monotonic(), readings)

    def telemetry(self):
        sample = self._sample
        if sample is None:
            return None
        sampled_at, readings = sample
        return {**readings, "age_s": monotonic() - sampled_at}

    def __enter__(self):
        self._thread = Thread(target=self.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True)
        try:
            self._thread.start()
        except BaseException:
            self.server_close()
            raise
        return self

    def __exit__(self, exc_type, *_):
        if exc_type is KeyboardInterrupt and self.exit_reason is None:
            self.exit_reason = "Ctrl+C pressed"
        elif exc_type is None and self.exit_reason is None:
            self.exit_reason = "Session ended"
        self.closing.set()
        self.shutdown()
        self.server_close()
        self._thread.join()
