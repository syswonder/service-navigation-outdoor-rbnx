"""HTTP transport for the goal/task contract. No ROS or legacy route submission."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import re
import threading


class ApiError(ValueError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def serve(backend, port=8765, host="127.0.0.1"):

    class Handler(BaseHTTPRequestHandler):
        def respond(self, code, payload):
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def dispatch(self, method):
            try:
                path = self.path.split("?", 1)[0]
                if method == "GET":
                    match = re.fullmatch(r"/api/costmaps/(local|global)", path)
                    if match and hasattr(backend, "costmap_snapshot"):
                        return self.respond(200, backend.costmap_snapshot(match[1]))
                    if path == "/api/status":
                        return self.respond(200, backend.status())
                    match = re.fullmatch(r"/api/tasks/([a-f0-9]{32})", path)
                    if match:
                        return self.respond(200, backend.tasks.snapshot(match[1]))
                elif method == "POST":
                    match = re.fullmatch(r"/api/tasks/([a-f0-9]{32})/cancel", path)
                    if match:
                        return self.respond(202, backend.tasks.cancel(match[1]))
                    if path in (
                        "/api/tasks",
                        "/api/activation",
                    ):
                        size = int(self.headers.get("Content-Length", "0"))
                        if not 0 < size <= 2*1024*1024:
                            raise ApiError(400, "JSON request must be 1..2097152 bytes")
                        payload = json.loads(self.rfile.read(size))
                        if path == "/api/activation":
                            if not hasattr(backend, "set_active"):
                                raise ApiError(404, "Activation is unavailable")
                            if (
                                not isinstance(payload, dict)
                                or type(payload.get("active")) is not bool
                            ):
                                raise ValueError("active must be a boolean")
                            backend.set_active(payload["active"])
                            return self.respond(200, {"active": payload["active"]})
                        return self.respond(202, backend.submit(payload))
                self.respond(404, {"error": "Unknown endpoint"})
            except ApiError as error:
                self.respond(error.status, {"error": str(error)})
            except KeyError:
                self.respond(404, {"error": "Resource does not exist"})
            except (ValueError, TypeError) as error:
                self.respond(400, {"error": str(error)})
            except Exception:
                self.respond(
                    500,
                    {"error": "Navigation request failed; inspect the navigation log"},
                )

        def do_GET(self):
            self.dispatch("GET")

        def do_POST(self):
            self.dispatch("POST")

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server
