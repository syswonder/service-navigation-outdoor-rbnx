"""HTTP transport for the goal/task contract. No ROS or legacy route submission."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import re
import threading
import mimetypes
from pathlib import Path
from urllib.parse import unquote


class ApiError(ValueError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def serve(backend, port=8765, host="127.0.0.1", web_root=None):
    root = Path(web_root).resolve() if web_root else None

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
                    if not path.startswith("/api/") and root is not None:
                        target = (root / unquote(path).lstrip("/")).resolve()
                        if not target.is_relative_to(root):
                            return self.respond(404, {"error": "Not found"})
                        if target.is_dir():
                            target = target / "index.html"
                        if not target.is_file():
                            return self.respond(
                                404, {"error": "Web asset not found; build web first"}
                            )
                        body = target.read_bytes()
                        self.send_response(200)
                        self.send_header(
                            "Content-Type",
                            mimetypes.guess_type(target.name)[0]
                            or "application/octet-stream",
                        )
                        self.send_header("Content-Length", str(len(body)))
                        self.send_header("Cache-Control", "no-cache")
                        self.end_headers()
                        self.wfile.write(body)
                        return
                    if (
                        path == "/api/places"
                        and getattr(backend, "places", None) is not None
                    ):
                        return self.respond(200, backend.places.snapshot())
                    if path == "/api/map":
                        return self.respond(200, backend.map_snapshot())
                    match = re.fullmatch(r"/api/costmaps/(local|global)", path)
                    if match and hasattr(backend, "costmap_snapshot"):
                        return self.respond(200, backend.costmap_snapshot(match[1]))
                    if path == "/api/status":
                        return self.respond(200, backend.status())
                    if path == "/api/obstacles":
                        if backend.obstacles is None:
                            return self.respond(200, {"obstacles": []})
                        return self.respond(
                            200, {"obstacles": backend.obstacles.snapshot()}
                        )
                    match = re.fullmatch(r"/api/tasks/([a-f0-9]{32})", path)
                    if match:
                        return self.respond(200, backend.tasks.snapshot(match[1]))
                elif method == "POST":
                    match = re.fullmatch(r"/api/tasks/([a-f0-9]{32})/cancel", path)
                    if match:
                        return self.respond(202, backend.tasks.cancel(match[1]))
                    if path in (
                        "/api/plans",
                        "/api/tasks",
                        "/api/obstacles",
                        "/api/activation",
                    ):
                        size = int(self.headers.get("Content-Length", "0"))
                        if not 0 < size <= 65536:
                            raise ApiError(400, "JSON request must be 1..65536 bytes")
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
                        if path == "/api/obstacles":
                            if backend.obstacles is None:
                                raise ApiError(
                                    403,
                                    "Obstacle injection is only available in simulation",
                                )
                            if not isinstance(payload, dict):
                                raise ValueError("Request must be an object")
                            if payload.get("frame_id") != "map":
                                raise ValueError("Obstacle coordinates must use map")
                            for key in ("scene_version", "route_version"):
                                if payload.get(key) != backend.bundle[key]:
                                    raise ApiError(409, "Scene version mismatch")
                            return self.respond(201, backend.obstacles.add(payload))
                        if path == "/api/plans":
                            return self.respond(200, backend.preview(payload))
                        return self.respond(202, backend.submit(payload))
                elif method == "DELETE":
                    match = re.fullmatch(
                        r"/api/obstacles/(test_[A-Za-z0-9_]{1,48})", path
                    )
                    if match:
                        if backend.obstacles is None:
                            raise ApiError(
                                403,
                                "Obstacle injection is only available in simulation",
                            )
                        return self.respond(200, backend.obstacles.delete(match[1]))
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

        def do_DELETE(self):
            self.dispatch("DELETE")

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server
