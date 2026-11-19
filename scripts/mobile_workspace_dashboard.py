"""Workspace editor available only on the loopback mobile dashboard."""

from http import HTTPStatus
import json

from scripts.lerobot_web_dashboard import DashboardHandler, WebDashboard


class WorkspaceHandler(DashboardHandler):
    def do_GET(self):
        if self.path == "/api/robot-workspace":
            self.respond(HTTPStatus.OK, "application/json", json.dumps(self.server.workspace.telemetry()).encode())
        else:
            super().do_GET()

    def do_POST(self):
        if self.path != "/api/robot-workspace":
            return super().do_POST()
        authority = f"127.0.0.1:{self.server.port}"
        if (self.headers.get("Origin") != f"http://{authority}" or self.headers.get("Host") != authority
                or self.headers.get("X-Dashboard-Session") != self.server.session_id):
            return self.respond(HTTPStatus.FORBIDDEN, "text/plain", b"Dashboard origin and session required")
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096 or self.headers.get_content_type() != "application/json":
                raise ValueError("Expected a JSON workspace, at most 4096 bytes")
            data = json.loads(self.rfile.read(length))
            result = self.server.workspace.save(data)
        except (ValueError, TypeError, OverflowError) as exc:
            return self.respond(HTTPStatus.BAD_REQUEST, "text/plain; charset=utf-8", str(exc).encode())
        except OSError:
            return self.respond(HTTPStatus.INTERNAL_SERVER_ERROR, "text/plain", b"Could not save robot workspace")
        self.respond(HTTPStatus.OK, "application/json", json.dumps(result).encode())


class MobileWorkspaceDashboard(WebDashboard):
    def __init__(self, host, port, *, page=None, workspace):
        if host != "127.0.0.1":
            raise ValueError("Workspace dashboard must listen on 127.0.0.1")
        self.workspace = workspace
        super().__init__(host, port, page=page)
        self.RequestHandlerClass = WorkspaceHandler
