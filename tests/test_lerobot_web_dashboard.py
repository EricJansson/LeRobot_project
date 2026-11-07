from http.client import HTTPConnection
from contextlib import nullcontext
import json
from pathlib import Path
import shutil
import socket
import subprocess

import pytest

from scripts.lerobot_web_dashboard import WebDashboard


def test_dashboard_restart_on_same_port():
    def snapshot(server):
        connection = HTTPConnection("127.0.0.1", server.port, timeout=2)
        try:
            connection.request("GET", "/api/telemetry")
            response = connection.getresponse()
            return response.getheader("X-Dashboard-Session"), json.loads(response.read())
        finally:
            connection.close()

    with WebDashboard("127.0.0.1", 0) as first:
        first.publish({"shoulder_pan": 10}, 20, {}, True)
        first_session, readings = snapshot(first)
        assert readings["joints_deg"] == {"shoulder_pan": 10}
        assert snapshot(first)[0] == first_session
        port = first.port
    with WebDashboard("127.0.0.1", port) as second:
        second_session, readings = snapshot(second)
        assert second_session and second_session != first_session
        assert readings is None


def test_dashboard_frontend_reconnect():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is needed for the frontend lifecycle check")
    result = subprocess.run([node, str(Path(__file__).with_name("dashboard_reconnect.cjs"))],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_dashboard_http_and_shutdown():
    try:
        dashboard = WebDashboard("127.0.0.1", 0)
    except PermissionError:
        pytest.skip("Local sockets are disabled in this environment")

    with dashboard:
        with pytest.raises(OSError):
            with WebDashboard("127.0.0.1", dashboard.port):
                pytest.fail("Second bind unexpectedly succeeded")

        def request(method, path):
            conn = HTTPConnection("127.0.0.1", dashboard.port, timeout=2)
            try:
                conn.request(method, path)
                response = conn.getresponse()
                assert response.getheader("Cache-Control") == "no-store"
                return response.status, response.read()
            finally:
                conn.close()

        assert request("GET", "/")[0] == 200
        assert request("GET", "/api/telemetry") == (200, b"null")
        xyz_m = {"x": 0.15, "y": -0.02, "z": 0.18}
        dashboard.publish({"shoulder_pan": 12.5}, 30.0, xyz_m, True)
        status, body = request("GET", "/api/telemetry")
        assert status == 200
        sample = json.loads(body)
        assert sample.pop("age_s") >= 0
        assert sample == {
            "joints_deg": {"shoulder_pan": 12.5},
            "gripper_normalized": 30.0,
            "xyz_m": xyz_m,
            "gamepad_enabled": True,
        }
        gamepad = {
            "name": "Fixture controller",
            "axis_count": 6,
            "button_count": 14,
            "hat_count": 1,
            "deadzone": 0.1,
            "axes": {"LX": -0.5, "LT": 0.75},
            "buttons": {"LB": False, "Cross": True},
            "dpad": [-1, 1],
        }
        dashboard.publish({"shoulder_pan": 12.5}, 30.0, xyz_m, False, gamepad=gamepad)
        gamepad["axes"]["LX"] = 0
        gamepad["buttons"]["Cross"] = False
        gamepad["dpad"][0] = 0
        status, body = request("GET", "/api/telemetry")
        assert status == 200
        sample = json.loads(body)
        assert sample["gamepad_enabled"] is False
        assert sample["gamepad"] == {
            **gamepad,
            "axes": {"LX": -0.5, "LT": 0.75},
            "buttons": {"LB": False, "Cross": True},
            "dpad": [-1, 1],
        }
        assert request("POST", "/api/telemetry")[0] == 405
        assert request("GET", "/README.md")[0] == 404

    with pytest.raises(ConnectionRefusedError):
        with socket.create_connection(("127.0.0.1", dashboard.port), timeout=4):
            pytest.fail("Dashboard is still accepting connections")


@pytest.mark.parametrize(("error", "reason"), [
    (None, "Esc pressed"),
    (KeyboardInterrupt, "Ctrl+C pressed"),
    (RuntimeError, None),
])
def test_dashboard_exit_event_distinguishes_stops_from_unexpected_errors(error, reason):
    try:
        dashboard = WebDashboard("127.0.0.1", 0)
    except PermissionError:
        pytest.skip("Local sockets are disabled in this environment")

    connection = HTTPConnection("127.0.0.1", dashboard.port, timeout=2)
    try:
        with (pytest.raises(error) if error else nullcontext()), dashboard:
            connection.request("GET", "/api/exit")
            response = connection.getresponse()
            assert response.status == 200
            assert response.getheader("Content-Type") == "text/event-stream"
            assert response.readline() == b": waiting for exit\n"
            assert response.readline() == b"\n"
            if error:
                raise error()
            dashboard.exit_reason = reason
        if reason is None:
            assert response.read() == b""
        else:
            assert response.readline() == b"event: stopped\n"
            assert json.loads(response.readline().removeprefix(b"data: ")) == reason
    finally:
        connection.close()
