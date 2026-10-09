"""Exercise live adapter boundaries, UART shutdown and echo interpretation."""
from dataclasses import asdict, replace
import importlib.util
import json
import math
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

from jetson.amr_core.full_run import FullRunConfig, SideReading
from jetson.amr_core.full_run_ultrasonic import UltrasonicWorker, McuUltrasonicFeed
from jetson.amr_core.full_run_legacy import validate_legacy_status

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("full_run_adapter_test", ROOT / "scripts/full_run_controller.py")
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class ScanTests(unittest.TestCase):
    def scan(self, ranges):
        return types.SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi/180,
                                     range_min=0.05, range_max=12.0, ranges=ranges)

    def test_infinity_is_coverage_but_zero_nan_are_not(self):
        config = FullRunConfig()
        points, front = adapter.parse_scan(self.scan([math.inf]*360), config)
        self.assertEqual(points, [])
        self.assertEqual(front, 12)
        for ranges in ([0.0]*360, [math.nan]*360):
            with self.assertRaises(ValueError):
                adapter.parse_scan(self.scan(ranges), config)

    def test_lidar_extrinsics_apply_to_points(self):
        values = [math.inf]*360
        values[180] = 2
        points, front = adapter.parse_scan(self.scan(values), replace(FullRunConfig(), lidar_x_m=0.3))
        self.assertAlmostEqual(front, 2.3)
        self.assertAlmostEqual(points[0][1], 2.3)

    def test_configured_chassis_returns_are_self_filtered_without_hiding_external_points(self):
        values = [math.inf]*360
        values[180] = 0.20  # x=+0.20: physically inside the configured chassis.
        values[181] = 2.0   # Outside the chassis and must remain an obstacle point.
        metrics = {}
        points, front = adapter.parse_scan(self.scan(values), FullRunConfig(), metrics)
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][1], 2.0)
        self.assertEqual(metrics["valid_points"], 2)
        self.assertEqual(metrics["self_filtered_points"], 1)
        self.assertEqual(metrics["retained_points"], 1)
        self.assertGreater(front, 1.9)


class EchoTests(unittest.TestCase):
    def worker(self, stuck=False, generate=False):
        levels = {31: 1 if stuck else 0}
        worker = UltrasonicWorker.__new__(UltrasonicWorker)
        worker.c = FullRunConfig(ultrasonic_timeout_s=0.001)
        worker.edge_lock = threading.Lock()
        worker.echo_done = threading.Event()
        worker.active_echo = worker.rise_ns = worker.fall_ns = None
        worker.pins = {"left": (33, 31)}
        worker.stats = {"left": {"triggers": 0, "rising_edges": 0, "falling_edges": 0,
                                 "edge_callbacks": 0, "ignored_edges": 0}}
        worker.edge_error = None
        worker.emit = lambda *_args, **_kwargs: None
        def output(_pin, value):
            if generate:
                levels[31] = value
                worker.on_edge(31)
        worker.gpio = types.SimpleNamespace(HIGH=1, LOW=0, input=lambda pin: levels.get(pin, 0), output=output)
        return worker

    def test_missing_echo_is_ambiguous_not_zero_range(self):
        reading = self.worker().measure("left")
        self.assertEqual(reading.status, "NO_ECHO")
        self.assertIsNone(reading.distance_m)
        self.assertIn("disconnected", reading.detail)

    def test_stuck_high_is_fault(self):
        self.assertEqual(self.worker(stuck=True).measure("left").status, "FAULT")

    def test_valid_pulse_converts_to_meters(self):
        with mock.patch("jetson.amr_core.full_run_ultrasonic.time.monotonic_ns", side_effect=[1000000, 3000000]):
            reading = self.worker(generate=True).measure("left")
        self.assertEqual(reading.status, "VALID")
        self.assertAlmostEqual(reading.distance_m, 0.343)

    def test_zero_length_pulse_is_fault(self):
        with mock.patch("jetson.amr_core.full_run_ultrasonic.time.monotonic_ns", side_effect=[1000000, 1000000]):
            reading = self.worker(generate=True).measure("left")
        self.assertEqual(reading.status, "FAULT")


class McuSideFeedTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.feed = McuUltrasonicFeed(FullRunConfig(), lambda event, **data: self.events.append((event, data)))

    def reading(self, left, right):
        raw = validate_legacy_status(f"$STATUS,0,0,0,{left},{right},80")
        self.assertTrue(self.feed.update(raw, 100))
        return self.feed.results.get_nowait(), self.feed.results.get_nowait()

    def test_actual_centimetres_become_correct_side_metres(self):
        left, right = self.reading(35, 140)
        self.assertEqual((left.side, right.side), ("left", "right"))
        self.assertEqual((left.status, right.status), ("VALID", "VALID"))
        self.assertAlmostEqual(left.distance_m, .35)
        self.assertAlmostEqual(right.distance_m, 1.40)
        self.assertFalse(self.feed.snapshot()["Jetson_GPIO_owner"])

    def test_minus_one_is_ambiguous_NO_ECHO_not_zero_distance(self):
        for reading in self.reading(-1, -1):
            self.assertEqual(reading.status, "NO_ECHO")
            self.assertIsNone(reading.distance_m)
            self.assertIn("ambiguous", reading.detail)

    def test_zero_or_dangerously_short_echo_is_a_fault(self):
        for reading in self.reading(0, 1):
            self.assertEqual(reading.status, "FAULT")

    def test_above_configured_range_is_distinct_from_numeric_zero(self):
        for reading in self.reading(301, 400):
            self.assertEqual(reading.status, "NO_ECHO")
            self.assertGreater(reading.distance_m, 3)
            self.assertIn("outside", reading.detail)

    def test_old_format_cannot_fabricate_missing_MCU_side_distances(self):
        raw = validate_legacy_status("$STATUS,0,1234,45,0,0,0,0,0,0,0")
        self.assertFalse(self.feed.update(raw, 100))
        self.assertTrue(self.feed.results.empty())
        self.assertTrue(any(e == "mcu_side_input_invalid" for e, _ in self.events))

    def test_backlogged_STATUS_does_not_create_unbounded_side_queue(self):
        raw = validate_legacy_status("$STATUS,0,0,0,35,140,80")
        for n in range(100):
            self.feed.update(raw, n)
        self.assertEqual(self.feed.results.qsize(), 64)
        self.assertTrue(any(e == "ultrasonic_queue_overflow" for e, _ in self.events))


class AdapterTests(unittest.TestCase):
    def run_preflight(self, firmware=True, fail_write=False, protocol="ctrl", raw_base=0, same_port=False,
                      live_stop_cycle=None, resume_after_stop=False, initial_emergency=False,
                      status_on_command=False, command_status_fallback=True, fail_cleanup=False,
                      fail_uart_open=False, fail_shutdown_write=False, compact_status=False, unknown_status=False):
        uarts = []
        captured = {}
        class Uart:
            def __init__(self, *_args, **_kwargs):
                if fail_uart_open:
                    raise PermissionError("simulated UART permission error")
                self.incoming = bytearray()
                self.writes = []
                self.closed = False
                uarts.append(self)
            @property
            def in_waiting(self):
                return len(self.incoming)
            def reset_input_buffer(self):
                self.incoming.clear()
            def read(self, count):
                result = bytes(self.incoming[:count])
                del self.incoming[:count]
                return result
            def write(self, data):
                self.writes.append(data)
                if fail_shutdown_write and data == b"$CMD,0,0,1\r\n":
                    raise OSError("simulated shutdown UART failure")
                if fail_write and data.startswith(b"$CMD") and data.endswith(b",0\r\n"):
                    raise OSError("simulated UART write failure")
                if data.startswith(b"$FULL,") and firmware:
                    session = int(data.decode().strip().split(",")[1])
                    (uarts[0] if status_on_command else uarts[-1]).incoming.extend(
                        f"$CTRL,1,{session},0,0,0,0,0,0,0,0,1000\r\n".encode())
                elif protocol == "legacy" and firmware and data.startswith(b"$CMD"):
                    base = raw_base
                    if live_stop_cycle is not None:
                        cycle = captured.get("cycle", 0)
                        base = 0 if cycle <= 2 else (15 if resume_after_stop and cycle >= 6 else 10)
                        if compact_status and resume_after_stop and cycle >= 13:
                            base = 20
                    emergency = int(initial_emergency and len(self.writes) == 1)
                    frame = (f"$STATUS,{base},0,0,-1,-1,80\r\n" if compact_status
                             else f"$STATUS,{base},1234,45,0,0,0,0,0,0,{emergency}\r\n")
                    if unknown_status:
                        frame = f"$STATUS,{base},0,0,-1,-1,80,0\r\n"
                    (uarts[0] if status_on_command else uarts[-1]).incoming.extend(
                        frame.encode())
                return len(data)
            def close(self):
                self.closed = True
        class Worker:
            def __init__(self, *_):
                self.results = queue.Queue()
                self.inside = None
                captured["worker"] = self
            def close(self):
                if fail_cleanup:
                    raise RuntimeError("simulated GPIO cleanup failure")
                pass
        def make_node(config, core, recorder):
            captured["core"] = core
            # Simulate a slow import / device setup before the main loop.
            time.sleep(0.30)
            return types.SimpleNamespace(destroy_node=lambda: None)
        def spin_once(*_args, **_kwargs):
            captured["cycle"] = captured.get("cycle", 0) + 1
            core = captured["core"]
            now = time.monotonic()
            core.update_camera([], now)
            core.update_scan([], 4, now)
            if "worker" in captured:
                for side in ("left", "right"):
                    captured["worker"].results.put(SideReading(side, now, "NO_ECHO"))
        serial = types.ModuleType("serial")
        serial.Serial = Uart
        ros = types.ModuleType("rclpy")
        ros.init = ros.try_shutdown = lambda: None
        ros.ok = lambda: live_stop_cycle is None or captured.get("cycle", 0) < (16 if compact_status else 10)
        ros.spin_once = spin_once
        jetson = types.ModuleType("Jetson")
        jetson.__path__ = []
        gpio = types.ModuleType("Jetson.GPIO")
        original_read = Path.read_text
        def read_text(path, *args, **kwargs):
            if path == Path("/proc/321/stat"):
                return "321 (bash) S " + "0 " * 18 + "456 0"
            return original_read(path, *args, **kwargs)
        def stop_request(_path, _pid, _ticks, last_id):
            if live_stop_cycle is not None and captured.get("cycle", 0) >= live_stop_cycle and last_id is None:
                return {"request_id": "test-stop", "action": "instant_stop", "pid": 321, "start_ticks": "456"}
            return None
        with tempfile.TemporaryDirectory() as folder:
            config = asdict(replace(FullRunConfig(), startup_timeout_s=0.5, acceleration_rpm_s=1000))
            config_path = Path(folder) / "config.json"
            config_path.write_text(json.dumps(config))
            with mock.patch.dict(sys.modules, {"serial": serial, "rclpy": ros, "Jetson": jetson, "Jetson.GPIO": gpio}), \
                    mock.patch.object(adapter, "make_node", side_effect=make_node), \
                    mock.patch.object(adapter, "UltrasonicWorker", Worker), \
                    mock.patch.object(Path, "read_text", read_text), \
                    mock.patch.object(adapter, "read_stop_request", side_effect=stop_request), \
                    mock.patch.object(adapter.signal, "signal"), mock.patch("builtins.print"):
                arguments = ["--config", str(config_path), "--port", "commands", "--status-port", "commands" if same_port else "status",
                             "--log-dir", folder, "--side-sensor-source", "mcu" if compact_status else "gpio"]
                if live_stop_cycle is None:
                    arguments += ["--preflight-only"]
                else:
                    arguments += ["--supervisor-pid", "321", "--supervisor-start-ticks", "456",
                                  "--user-stop-file", str(Path(folder) / "stop.json")]
                if protocol != "legacy":
                    arguments += ["--mcu-protocol", protocol]
                if not command_status_fallback:
                    arguments += ["--no-command-status-fallback"]
                result = adapter.main(arguments)
            records = [json.loads(line) for line in (Path(folder) / "events.jsonl").read_text().splitlines()]
            captured["automatic_request"] = (Path(folder) / "auto_bundle.request.json").exists()
        self.automatic_request = captured["automatic_request"]
        return result, uarts, records

    def test_preflight_is_always_zero_and_closes_both_uarts(self):
        result, uarts, records = self.run_preflight()
        self.assertEqual(result, 0, records)
        for message in uarts[0].writes:
            if message.startswith(b"$CMD"):
                self.assertTrue(message.startswith(b"$CMD,0,0,"), message)
        self.assertTrue(all(u.closed for u in uarts))
        self.assertTrue(any(r["event"] == "ready" for r in records))
        self.assertFalse(any(r["event"] == "exception" for r in records))
        self.assertFalse(self.automatic_request)

    def test_missing_extended_firmware_fails_preflight_without_motion(self):
        result, uarts, records = self.run_preflight(firmware=False)
        self.assertEqual(result, 1)
        self.assertTrue(all(b"$CMD,0,0," in m for m in uarts[0].writes if m.startswith(b"$CMD")))
        self.assertTrue(any(r.get("after") == "FAULT_STOP" for r in records))
        self.assertTrue(self.automatic_request)

    def test_unhandled_uart_exception_requests_bundle_and_still_stops(self):
        result, uarts, records = self.run_preflight(fail_write=True)
        self.assertEqual(result, 1)
        self.assertTrue(self.automatic_request)
        self.assertIn(b"$CMD,0,0,1\r\n", uarts[0].writes)
        self.assertTrue(all(u.closed for u in uarts))

    def test_default_preflight_accepts_old_firmware_and_never_sends_FULL(self):
        result, uarts, records = self.run_preflight(protocol="legacy", raw_base=40)
        self.assertEqual(result, 0, records)
        self.assertFalse(any(message.startswith(b"$FULL") for message in uarts[0].writes))
        self.assertTrue(all(message.startswith(b"$CMD,0,0,") for message in uarts[0].writes))
        self.assertTrue(any(r["event"] == "legacy_status_normalized" and r["selected_rpm"] == 0 for r in records))
        self.assertFalse(self.automatic_request)

    def test_legacy_same_uart_for_commands_and_status_is_supported(self):
        result, uarts, records = self.run_preflight(protocol="legacy", same_port=True)
        self.assertEqual(result, 0, records)
        self.assertEqual(len(uarts), 1)
        self.assertTrue(uarts[0].closed)

    def test_legacy_status_on_command_RX_passes_without_firmware_update_or_motion(self):
        result, uarts, records = self.run_preflight(protocol="legacy", status_on_command=True)
        self.assertEqual(result, 0, records)
        sources = [r for r in records if r["event"] == "uart_status_source"]
        self.assertEqual(len(sources), 1)
        self.assertEqual(sources[0]["port"], "commands")
        self.assertTrue(sources[0]["fallback"])
        self.assertTrue(all(m.startswith(b"$CMD,0,0,") for m in uarts[0].writes))
        self.assertFalse(uarts[1].writes)
        self.assertFalse(self.automatic_request)

    def test_UART_open_failure_records_exact_stage_and_requests_automatic_bundle(self):
        result, uarts, records = self.run_preflight(protocol="legacy", fail_uart_open=True)
        self.assertEqual(result, 1)
        errors = [r for r in records if r["event"] == "exception"]
        self.assertEqual(errors[0]["startup_stage"], "open_command_UART")
        self.assertIn("PermissionError", errors[0]["traceback"])
        self.assertEqual(uarts, [])
        self.assertTrue(self.automatic_request)

    def test_actual_compact_packet_passes_preflight_without_firmware_update_or_motion(self):
        result, uarts, records = self.run_preflight(protocol="legacy", compact_status=True, status_on_command=True)
        self.assertEqual(result, 0, records)
        self.assertTrue(any(r["event"] == "ready" for r in records))
        self.assertFalse(any(r.get("after") == "FAULT_STOP" for r in records))
        self.assertFalse(any(r["event"] == "MCU_schema_mismatch" for r in records))
        self.assertTrue(all(m.startswith(b"$CMD,0,0,") for m in uarts[0].writes))
        self.assertFalse(self.automatic_request)
        sources = [r for r in records if r["event"] == "side_sensor_source"]
        self.assertEqual(sources[0]["source"], "STM32_STATUS")
        self.assertFalse(any(r.get("stage") in ("import_Jetson_GPIO", "initialize_ultrasonic_GPIO") for r in records))
        sides = [r for r in records if r["event"] == "ultrasonic"]
        self.assertTrue(sides)
        self.assertTrue(all(r["source"] == "STM32_STATUS" and r["raw_cm"] == -1 for r in sides))

    def test_unsupported_packet_fails_with_schema_reason_without_motion_or_timeout(self):
        result, uarts, records = self.run_preflight(protocol="legacy", unknown_status=True, status_on_command=True)
        self.assertEqual(result, 1)
        faults = [r for r in records if r.get("after") == "FAULT_STOP"]
        self.assertTrue(faults)
        self.assertIn("STM32_STATUS_unsupported_layout", faults[0]["reason"])
        self.assertIn("received_fields=8", faults[0]["reason"])
        self.assertNotIn("startup_timeout", faults[0]["reason"])
        self.assertTrue(any(r["event"] == "MCU_schema_mismatch" for r in records))
        self.assertTrue(all(m.startswith(b"$CMD,0,0,") for m in uarts[0].writes))
        self.assertTrue(self.automatic_request)

    def test_compact_terminal_stop_waits_for_standstill_then_new_UP_resumes(self):
        result, uarts, records = self.run_preflight(protocol="legacy", compact_status=True, status_on_command=True,
                                                   live_stop_cycle=5, resume_after_stop=True)
        self.assertEqual(result, 0, records)
        states = [r["after"] for r in records if r["event"] == "state"]
        self.assertEqual(states[:4], ["READY", "RUNNING", "PAUSED", "RUNNING"])
        self.assertTrue(any(r["event"] == "legacy_stop_standstill_observed" for r in records))
        self.assertTrue(any(r["event"] == "legacy_stop_UP_consumed" for r in records))
        self.assertFalse(any(r["event"] == "legacy_stop_acknowledged" for r in records))
        self.assertFalse(self.automatic_request)

    def test_cleanup_failure_cannot_be_reported_as_success(self):
        result, uarts, records = self.run_preflight(protocol="legacy", fail_cleanup=True)
        self.assertEqual(result, 1)
        self.assertTrue(any(r.get("context") == "gpio" for r in records))
        self.assertTrue(all(u.closed for u in uarts))
        self.assertTrue(self.automatic_request)

    def test_shutdown_UART_error_records_trace_and_does_not_skip_other_cleanup(self):
        result, uarts, records = self.run_preflight(protocol="legacy", fail_shutdown_write=True)
        self.assertEqual(result, 1)
        self.assertTrue(any(r.get("context") == "shutdown_stop" for r in records))
        self.assertTrue(all(u.closed for u in uarts))
        self.assertTrue(self.automatic_request)

    def test_command_RX_fallback_can_be_disabled_and_missing_status_still_faults(self):
        result, uarts, records = self.run_preflight(protocol="legacy", status_on_command=True,
                                                   command_status_fallback=False)
        self.assertEqual(result, 1)
        self.assertTrue(all(m.startswith(b"$CMD,0,0,") for m in uarts[0].writes))
        self.assertFalse(any(r["event"] == "uart_status_source" for r in records))
        health = [r for r in records if r["event"] == "uart_health"][-1]
        self.assertEqual(list(health["ports"]), ["status"])
        self.assertEqual(health["ports"]["status"]["rx_bytes"], 0)
        self.assertTrue(self.automatic_request)

    def test_no_RX_fault_records_both_ports_before_automatic_bundle(self):
        result, uarts, records = self.run_preflight(protocol="legacy", firmware=False)
        self.assertEqual(result, 1)
        health_index = max(i for i, r in enumerate(records) if r["event"] == "uart_health")
        bundle_index = next(i for i, r in enumerate(records) if r["event"] == "auto_bundle_requested")
        self.assertLess(health_index, bundle_index)
        health = records[health_index]
        self.assertEqual(set(health["ports"]), {"commands", "status"})
        self.assertTrue(all(p["rx_bytes"] == 0 for p in health["ports"].values()))
        self.assertTrue(health["diagnosis"].startswith("no_RX_bytes_"))
        self.assertTrue(all(m.startswith(b"$CMD,0,0,") for m in uarts[0].writes))

    def test_legacy_preflight_recovers_prior_shutdown_ESTOP_without_button_press(self):
        result, uarts, records = self.run_preflight(protocol="legacy", initial_emergency=True)
        self.assertEqual(result, 0, records)
        self.assertTrue(any(r["event"] == "legacy_startup_ESTOP" for r in records))
        self.assertFalse(self.automatic_request)

    def test_legacy_missing_status_is_a_fault_not_an_open_loop_fallback(self):
        result, uarts, records = self.run_preflight(protocol="legacy", firmware=False)
        self.assertEqual(result, 1)
        self.assertFalse(any(message.startswith(b"$FULL") for message in uarts[0].writes))
        self.assertTrue(any("STM32_STATUS_missing_or_stale" in str(r) for r in records))
        self.assertTrue(self.automatic_request)

    def test_terminal_stop_at_READY_consumes_simultaneous_UP(self):
        result, uarts, records = self.run_preflight(protocol="legacy", live_stop_cycle=4)
        self.assertEqual(result, 0, records)
        self.assertFalse(any(r.get("after") == "RUNNING" for r in records))
        self.assertTrue(all(m.startswith(b"$CMD,0,0,") for m in uarts[0].writes))
        self.assertTrue(any(r.get("after") == "PAUSED" for r in records))

    def test_live_terminal_stop_then_UP_restores_saved_speed_without_exit(self):
        result, uarts, records = self.run_preflight(protocol="legacy", live_stop_cycle=5, resume_after_stop=True)
        self.assertEqual(result, 0, records)
        states = [r["after"] for r in records if r["event"] == "state"]
        self.assertEqual(states[:4], ["READY", "RUNNING", "PAUSED", "RUNNING"])
        self.assertTrue(any(r["event"] == "legacy_status_normalized" and r["base_delta"] == 5 and r["selected_rpm"] == 10 for r in records))
        paused_tx = [r["text"] for r in records if r["event"] == "uart_tx" and r.get("state") == "PAUSED"]
        self.assertTrue(paused_tx)
        self.assertTrue(all(text.startswith("$CMD,0,0,") for text in paused_tx))
        self.assertFalse(self.automatic_request)


if __name__ == "__main__":
    unittest.main()
