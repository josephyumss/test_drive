"""Dependency-free local HTTP server for the Guide AMR dashboard."""

from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from typing import Any
from urllib.parse import urlparse

from .store import MonitoringStore


STATIC_ROOT = Path(__file__).with_name("static")
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
}


class MonitoringRequestHandler(BaseHTTPRequestHandler):
    store: MonitoringStore
    server_version = "GuideAMRMonitoring/0.1"

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/status":
            self._json_response(HTTPStatus.OK, self.store.snapshot())
            return
        if path == "/api/health":
            self._json_response(HTTPStatus.OK, {"ok": True})
            return

        static_path = "/index.html" if path == "/" else path
        self._serve_static(static_path)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path != "/api/status":
            self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 64 * 1024:
                raise ValueError("invalid request size")
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            result = self.store.update(body)
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._json_response(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        self._json_response(HTTPStatus.OK, result)

    def log_message(self, format: str, *args: Any) -> None:
        # Keep the terminal quiet during 2 Hz dashboard polling.
        if not (self.command == "GET" and self.path == "/api/status"):
            super().log_message(format, *args)

    def _serve_static(self, request_path: str) -> None:
        relative = request_path.lstrip("/")
        candidate = (STATIC_ROOT / relative).resolve()
        try:
            candidate.relative_to(STATIC_ROOT.resolve())
        except ValueError:
            self._json_response(HTTPStatus.FORBIDDEN, {"error": "forbidden"})
            return

        if not candidate.is_file() or candidate.suffix not in CONTENT_TYPES:
            self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return

        content = candidate.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", CONTENT_TYPES[candidate.suffix])
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src 'self'; img-src 'self' data:; object-src 'none'",
        )
        self.end_headers()
        self.wfile.write(content)

    def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)


def make_server(
    host: str = "127.0.0.1",
    port: int = 8080,
    *,
    store: MonitoringStore | None = None,
) -> ThreadingHTTPServer:
    dashboard_store = store or MonitoringStore()

    class BoundHandler(MonitoringRequestHandler):
        pass

    BoundHandler.store = dashboard_store
    return ThreadingHTTPServer((host, port), BoundHandler)


def run_mock_data(store: MonitoringStore, stop: threading.Event) -> None:
    """Publish a repeating demo sequence without touching robot hardware."""

    phases = [
        (
            7,
            {
                "system_state": "RUN",
                "state_reason": "정상 주행",
                "connection": {
                    "jetson": True,
                    "stm32": True,
                    "lidar": True,
                    "camera": True,
                },
                "velocity": {
                    "target_linear_mps": 0.45,
                    "actual_linear_mps": 0.43,
                    "angular_rad_s": 0.02,
                    "left_mps": 0.42,
                    "right_mps": 0.44,
                },
                "obstacle": {
                    "detected": False,
                    "object_class": "없음",
                    "distance_m": 3.4,
                    "ttc_s": 7.9,
                    "direction": "전방",
                },
                "battery": {"voltage_v": 23.8, "percent": 82, "warning": False},
                "diagnostics": {"uptime_ms": 12000},
            },
        ),
        (
            5,
            {
                "system_state": "SLOW",
                "state_reason": "전방 보행자 접근",
                "velocity": {
                    "target_linear_mps": 0.22,
                    "actual_linear_mps": 0.24,
                },
                "obstacle": {
                    "detected": True,
                    "object_class": "보행자",
                    "distance_m": 1.65,
                    "ttc_s": 3.1,
                    "direction": "전방 좌측",
                },
            },
        ),
        (
            4,
            {
                "system_state": "CONTROLLED_STOP",
                "state_reason": "장애물 정지 구역 진입",
                "velocity": {
                    "target_linear_mps": 0.0,
                    "actual_linear_mps": 0.0,
                    "left_mps": 0.0,
                    "right_mps": 0.0,
                },
                "obstacle": {
                    "detected": True,
                    "object_class": "보행자",
                    "distance_m": 0.95,
                    "ttc_s": None,
                },
                "last_stop_reason": "전방 보행자",
            },
        ),
        (
            4,
            {
                "system_state": "READY",
                "state_reason": "장애물 해제·재출발 대기",
                "obstacle": {
                    "detected": False,
                    "object_class": "없음",
                    "distance_m": 2.8,
                    "ttc_s": None,
                },
            },
        ),
    ]
    index = 0
    while not stop.is_set():
        duration, update = phases[index % len(phases)]
        store.update(update)
        for _ in range(duration * 2):
            if stop.wait(0.5):
                return
            snapshot = store.snapshot()["status"]
            uptime = snapshot["diagnostics"]["uptime_ms"] + 500
            store.update({"diagnostics": {"uptime_ms": uptime}})
        index += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Guide AMR local monitoring UI")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--mock", action="store_true", help="publish safe simulated robot data"
    )
    args = parser.parse_args()

    store = MonitoringStore()
    stop_mock = threading.Event()
    mock_thread: threading.Thread | None = None
    if args.mock:
        mock_thread = threading.Thread(
            target=run_mock_data, args=(store, stop_mock), daemon=True
        )
        mock_thread.start()

    server = make_server(args.host, args.port, store=store)
    print(f"Guide AMR monitoring: http://{args.host}:{args.port}")
    print("Mock data enabled." if args.mock else "Waiting for POST /api/status data.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_mock.set()
        server.shutdown()
        server.server_close()
        if mock_thread is not None:
            mock_thread.join(timeout=1)


if __name__ == "__main__":
    main()
