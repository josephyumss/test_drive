import json
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from monitoring.server import make_server
from monitoring.store import MonitoringStore


class MonitoringStoreTests(unittest.TestCase):
    def test_partial_update_preserves_other_fields(self) -> None:
        store = MonitoringStore()
        result = store.update(
            {
                "system_state": "RUN",
                "state_reason": "정상 주행",
                "velocity": {"actual_linear_mps": 0.3},
            }
        )
        self.assertEqual(result["status"]["system_state"], "RUN")
        self.assertEqual(result["status"]["velocity"]["actual_linear_mps"], 0.3)
        self.assertEqual(result["status"]["velocity"]["left_mps"], 0.0)

    def test_state_change_creates_event(self) -> None:
        store = MonitoringStore()
        result = store.update(
            {"system_state": "EMERGENCY_STOP", "state_reason": "낙차 감지"}
        )
        self.assertTrue(result["events"])
        self.assertEqual(result["events"][-1]["level"], "critical")

    def test_invalid_state_is_rejected(self) -> None:
        store = MonitoringStore()
        with self.assertRaisesRegex(ValueError, "system_state"):
            store.update({"system_state": "FLY"})

    def test_invalid_battery_percent_is_rejected(self) -> None:
        store = MonitoringStore()
        with self.assertRaisesRegex(ValueError, "battery.percent"):
            store.update({"battery": {"percent": 120}})


class MonitoringServerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.store = MonitoringStore()
        self.server = make_server("127.0.0.1", 0, store=self.store)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        host, port = self.server.server_address
        self.base_url = f"http://{host}:{port}"

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def test_health_endpoint(self) -> None:
        with urlopen(f"{self.base_url}/api/health", timeout=2) as response:
            payload = json.load(response)
        self.assertEqual(payload, {"ok": True})

    def test_status_post_and_get(self) -> None:
        body = json.dumps(
            {
                "system_state": "SLOW",
                "state_reason": "장애물 접근",
                "obstacle": {"detected": True, "distance_m": 1.5, "ttc_s": 3.0},
            }
        ).encode()
        request = Request(
            f"{self.base_url}/api/status",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=2) as response:
            self.assertEqual(response.status, 200)

        with urlopen(f"{self.base_url}/api/status", timeout=2) as response:
            payload = json.load(response)
        self.assertEqual(payload["status"]["system_state"], "SLOW")
        self.assertEqual(payload["status"]["obstacle"]["distance_m"], 1.5)

    def test_invalid_status_returns_400(self) -> None:
        request = Request(
            f"{self.base_url}/api/status",
            data=b'{"system_state":"INVALID"}',
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(HTTPError) as caught:
            urlopen(request, timeout=2)
        self.assertEqual(caught.exception.code, 400)
        caught.exception.close()

    def test_dashboard_static_files(self) -> None:
        with urlopen(f"{self.base_url}/", timeout=2) as response:
            html = response.read().decode()
        self.assertIn("통합 안전 관제", html)
        self.assertIn("/styles.css", html)


if __name__ == "__main__":
    unittest.main()
