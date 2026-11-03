from http.client import HTTPConnection
import json
import socket

import pytest

from scripts.lerobot_web_dashboard import WebDashboard


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
        assert request("POST", "/api/telemetry")[0] == 405
        assert request("GET", "/README.md")[0] == 404

    with pytest.raises(ConnectionRefusedError):
        with socket.create_connection(("127.0.0.1", dashboard.port), timeout=2):
            pytest.fail("Dashboard is still accepting connections")
