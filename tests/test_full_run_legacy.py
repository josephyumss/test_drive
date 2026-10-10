"""Existing STM32 firmware, inferred buttons, saved manoeuvres and local stop."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from jetson.amr_core.full_run import FullRunConfig, FullRunController, SideReading
from jetson.amr_core.full_run_legacy import LegacyControlAdapter, LegacyStatusLayoutChanged, validate_legacy_status
from jetson.amr_core.full_run_user_stop import request_stop, read_stop_request, process_start_ticks


def status_line(base=0, left=0, right=0, emergency=0):
    return f"$STATUS,{base},1234,45,0,0,{left},{right},0,0,{emergency}"


class LegacyAdapterTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.adapter = LegacyControlAdapter(123, emit=lambda event, **data: self.events.append((event, data)))
        self.now = 100.0

    def feed(self, base=0, allow_controls=True, **kwargs):
        self.now += .10
        return self.adapter.decode(status_line(base, **kwargs), self.now, allow_controls=allow_controls)

    def test_old_selected_speed_never_starts_new_process(self):
        first = self.feed(55)
        self.assertEqual(first.base_rpm, 0)
        self.assertEqual(first.up_count, 0)
        self.assertEqual(self.feed(55).base_rpm, 0)
        self.assertEqual(self.feed(60).base_rpm, 5)

    def test_previous_shutdown_ESTOP_does_not_deadlock_startup(self):
        first = self.feed(10, emergency=1, allow_controls=False)
        self.assertEqual(first.stop_flags, 2)
        cleared = self.feed(10, emergency=0, allow_controls=False)
        self.assertEqual((cleared.stop_flags, cleared.base_rpm), (0, 0))
        self.assertEqual(self.feed(15).base_rpm, 5)

    def test_startup_UP_is_consumed_and_never_queues_departure(self):
        self.feed(0, allow_controls=False)
        self.assertEqual(self.feed(5, allow_controls=False).base_rpm, 0)
        self.assertEqual(self.feed(5).base_rpm, 0)
        self.assertEqual(self.feed(10).base_rpm, 5)

    def test_base_changes_infer_UP_and_DOWN_without_new_protocol(self):
        self.feed()
        up = self.feed(15)
        self.assertEqual((up.base_rpm, up.up_count), (15, 3))
        down = self.feed(5)
        self.assertEqual((down.base_rpm, down.down_count), (5, 2))
        self.assertEqual(self.feed(5).up_count, 3)
        self.assertEqual(self.feed(0).base_rpm, 0)

    def test_software_limit_does_not_require_many_DOWN_presses(self):
        self.feed()
        self.assertEqual(self.feed(60).base_rpm, 25)
        self.assertEqual(self.feed(55).base_rpm, 20)

    def test_stop_preserves_speed_and_first_UP_restores_it(self):
        self.feed()
        self.feed(10)
        self.adapter.request_instant_stop()
        stopped = self.feed(10)
        self.assertEqual((stopped.base_rpm, stopped.stop_flags, stopped.stop_count), (10, 2, 1))
        resumed = self.feed(15)
        self.assertEqual((resumed.base_rpm, resumed.stop_flags), (10, 0))

    def test_UP_in_first_stop_acknowledgement_is_consumed_not_resumed(self):
        self.feed()
        self.feed(10)
        self.adapter.request_instant_stop()
        self.assertEqual(self.feed(15).stop_flags, 2)
        self.assertFalse(self.adapter.await_zero_command)
        self.assertEqual(self.feed(15).stop_flags, 2)
        resumed = self.feed(20)
        self.assertEqual((resumed.base_rpm, resumed.stop_flags), (10, 0))

    def test_legacy_emergency_stale_RPM_is_not_integrated(self):
        self.feed()
        emergency = self.feed(0, left=12, right=12, emergency=1)
        self.assertEqual((emergency.left_rpm, emergency.right_rpm), (0, 0))
        self.assertEqual(emergency.stop_flags, 2)
        self.assertEqual(self.feed(0, emergency=0).stop_flags, 2)
        self.assertEqual(self.feed(5, emergency=0).stop_flags, 0)

    def test_UP_while_emergency_still_reported_cannot_resume(self):
        self.feed()
        self.feed(0, emergency=1)
        self.assertEqual(self.feed(5, emergency=1).stop_flags, 2)
        self.assertEqual(self.feed(5, emergency=0).stop_flags, 2)
        self.assertEqual(self.feed(10).stop_flags, 0)

    def test_at_firmware_limit_UP_is_invisible_and_DOWN_UP_recovers(self):
        self.feed(80)
        self.adapter.request_instant_stop()
        self.assertEqual(self.feed(80).up_count, 0)
        self.feed(75)
        resumed = self.feed(80)
        self.assertEqual((resumed.base_rpm, resumed.stop_flags, resumed.up_count), (5, 0, 1))
        self.assertTrue(any(name == "legacy_button_limit" for name, _ in self.events))

    def test_invalid_STATUS_is_rejected(self):
        for line in ("$CTRL,1,123", status_line(256), status_line(3), status_line(-5),
                     status_line(left=1001), status_line(emergency=2)):
            with self.subTest(line=line), self.assertRaises(ValueError):
                self.adapter.decode(line, self.now)

    def test_observable_limits_and_host_clock_are_not_claimed_as_MCU_data(self):
        self.feed(0)
        normalized = self.feed(5, left=4, right=3)
        self.assertIsNone(normalized.left_counts)
        self.assertIsNone(normalized.right_counts)
        self.assertEqual(self.adapter.snapshot()["clock"], "Jetson_receive_time")
        self.assertEqual(self.adapter.snapshot()["button_events"], "inferred_from_base_RPM")


class CompactLegacyAdapterTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.adapter = LegacyControlAdapter(123, emit=lambda event, **data: self.events.append((event, data)))
        self.now = 100.0

    def feed(self, base=0, left=0, right=0, allow_controls=True):
        self.now += .10
        return self.adapter.decode(f"$STATUS,{base},{left},{right},-1,-1,80", self.now,
                                   allow_controls=allow_controls)

    def test_confirmed_positions_and_absent_fields_are_not_fabricated(self):
        raw = validate_legacy_status("$STATUS,30,12,-13,35,40,80")
        self.assertEqual((raw.base_rpm, raw.left_rpm, raw.right_rpm), (30, 12, -13))
        self.assertEqual((raw.left_us_cm, raw.right_us_cm, raw.sharp_distance_cm), (35, 40, 80))
        for name in ("emergency", "left_target_rpm", "right_target_rpm", "left_pwm", "right_pwm", "sharp_adc"):
            self.assertIsNone(getattr(raw, name))

    def test_old_base_is_ignored_and_new_UP_DOWN_control_speed(self):
        self.assertEqual(self.feed(80).base_rpm, 0)
        self.assertEqual(self.feed(85).base_rpm, 5)
        self.assertEqual(self.feed(90).base_rpm, 10)
        self.assertEqual(self.feed(80).base_rpm, 0)

    def test_real_RPM_is_used_not_confused_with_ultrasonic_minus_one(self):
        result = self.feed(0, left=12, right=13)
        self.assertEqual((result.left_rpm, result.right_rpm), (12, 13))
        self.assertEqual(self.adapter.raw.left_us_cm, -1)
        self.assertIsNone(self.adapter.snapshot()["MCU_ESTOP"])
        self.assertFalse(self.adapter.snapshot()["MCU_command_ACK_available"])

    def test_zero_feedback_without_a_host_zero_write_cannot_release_stop_barrier(self):
        self.feed()
        self.feed(10)
        self.adapter.request_instant_stop()
        for _ in range(5):
            self.assertEqual(self.feed(10).stop_flags, 2)
        self.assertTrue(self.adapter.await_zero_command)

    def test_stop_waits_for_three_zero_samples_and_discards_queued_UP(self):
        self.feed()
        self.feed(10)
        self.adapter.request_instant_stop()
        self.adapter.note_command_written(0, 0, 0, self.now)
        self.assertEqual(self.feed(15).stop_flags, 2)
        self.assertEqual(self.feed(15).stop_flags, 2)
        self.assertEqual(self.feed(15).stop_flags, 2)
        self.assertFalse(self.adapter.await_zero_command)
        self.assertEqual(self.feed(15).stop_flags, 2)
        resumed = self.feed(20)
        self.assertEqual((resumed.base_rpm, resumed.stop_flags), (10, 0))
        self.assertTrue(any(e == "legacy_stop_standstill_observed" for e, _ in self.events))
        self.assertFalse(any(e == "legacy_stop_acknowledged" for e, _ in self.events))

    def test_coasting_RPM_resets_standstill_count(self):
        self.feed()
        self.adapter.request_instant_stop()
        self.adapter.note_command_written(0, 0, 0, self.now)
        self.feed()
        self.feed(left=2)
        self.assertEqual(self.adapter.zero_rpm_samples, 0)
        self.assertTrue(self.adapter.await_zero_command)
        for _ in range(3):
            self.feed()
        self.assertFalse(self.adapter.await_zero_command)

    def test_batched_identical_frames_do_not_count_as_independent_zero_samples(self):
        self.feed()
        self.adapter.request_instant_stop()
        self.adapter.note_command_written(0, 0, 0, self.now)
        self.now += .01
        for _ in range(30):
            self.adapter.decode("$STATUS,0,0,0,-1,-1,80", self.now)
        self.assertEqual(self.adapter.zero_rpm_samples, 1)
        self.assertTrue(self.adapter.await_zero_command)

    def test_format_changes_are_not_silently_mixed_with_button_state(self):
        self.feed()
        with self.assertRaises(LegacyStatusLayoutChanged):
            self.adapter.decode(status_line(), self.now)

    def test_malformed_RPM_and_ranges_never_become_valid_zero(self):
        for line in ("$STATUS,0,NaN,0,-1,-1,80", "$STATUS,0,1001,0,-1,-1,80",
                     "$STATUS,0,0,0,-2,-1,80", "$STATUS,3,0,0,-1,-1,80",
                     "$STATUS,256,0,0,-1,-1,80"):
            with self.subTest(line=line), self.assertRaises(ValueError):
                validate_legacy_status(line)


class LegacyCoreTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.core = FullRunController(FullRunConfig(), 123, self.now, mcu_protocol="legacy")
        self.adapter = LegacyControlAdapter(123)
        self.feed()
        self.assertEqual(self.core.state, "READY")

    def feed(self, base=0, left=0, right=0, emergency=0, front=4):
        self.now += .02
        self.core.update_status(self.adapter.decode(status_line(base, left, right, emergency), self.now,
                                                    allow_controls=self.core.state != "STARTUP"), self.now)
        self.core.update_camera([], self.now)
        self.core.update_scan([], front, self.now)
        for side in ("left", "right"):
            self.core.update_side(SideReading(side, self.now, "NO_ECHO"))
        return self.core.tick(self.now)

    def test_zero_pause_then_UP_preserves_the_curve_and_phase(self):
        self.feed(10)
        self.assertEqual(self.core.state, "RUNNING")
        self.core.make_path(1.1, .9)
        self.core.set_phase("ENTRY")
        follower = self.core.follower
        self.feed(0)
        self.assertEqual(self.core.state, "PAUSED")
        self.feed(5)
        self.assertEqual((self.core.state, self.core.phase), ("RUNNING", "ENTRY"))
        self.assertIs(self.core.follower, follower)

    def test_terminal_stop_is_zero_then_UP_resumes_saved_speed(self):
        self.feed(10)
        self.adapter.request_instant_stop()
        self.core.change("PAUSED", "user_instant_stop_terminal")
        self.assertEqual(self.feed(10), (0, 0))
        self.feed(15)
        self.assertEqual(self.core.state, "RUNNING")
        self.assertEqual(self.core.status.base_rpm, 10)

    def test_blocked_UP_is_consumed_not_used_when_space_later_clears(self):
        self.feed(5)
        self.feed(5, front=.2)
        self.feed(10, front=.2)
        self.feed(10, front=4)
        self.assertEqual(self.core.state, "PAUSED")
        self.feed(15, front=4)
        self.assertEqual(self.core.state, "RUNNING")

    def test_distance_uses_measured_RPM_not_requested_command(self):
        self.feed(5)
        for _ in range(50):
            self.feed(5, left=6, right=6)
        expected = 6 / 60 * 3.141592653589793 * .2032
        self.assertAlmostEqual(self.core.odom.x_m, expected, places=6)
        self.assertEqual(self.core.snapshot(self.now)["odometry_source"], "reported_RPM_integral")

    def test_missing_status_is_a_latched_device_fault(self):
        self.now += .10
        while self.now - self.core.status_stamp <= .6:
            self.core.tick(self.now)
            self.now += .10
        self.assertEqual(self.core.state, "FAULT_STOP")
        self.assertIn("STM32_STATUS", self.core.fault_reason)
        self.feed(5)
        self.assertEqual(self.core.state, "FAULT_STOP")


class UserStopTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="full-run-user-stop-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / "project"
        (self.project / ".run").mkdir(parents=True)
        (self.project / "scripts").mkdir()
        (self.project / "scripts/start_full_run.sh").touch()
        (self.project / ".run/full_run.pid").write_text("123 456")
        process = self.root / "proc/123"
        process.mkdir(parents=True)
        (process / "cwd").mkdir()
        (process / "stat").write_text("123 (bash with spaces) S " + "0 " * 18 + "456 0\n")
        (process / "cmdline").write_bytes(b"bash\0" + str(self.project / "scripts/start_full_run.sh").encode() + b"\0")
        self.process = process

    def request(self):
        return request_stop(self.project, proc_root=self.root / "proc")

    def test_request_is_atomic_identified_and_consumed_once(self):
        request = self.request()
        target = self.project / ".run/full_run.user_stop.json"
        self.assertEqual(read_stop_request(target, 123, "456", None), request)
        self.assertIsNone(read_stop_request(target, 123, "456", request["request_id"]))
        self.assertEqual(list((self.project / ".run").glob(".full-run-user-stop-*")), [])

    def test_other_process_or_old_run_request_is_ignored(self):
        self.request()
        target = self.project / ".run/full_run.user_stop.json"
        self.assertIsNone(read_stop_request(target, 999, "456", None))
        self.assertIsNone(read_stop_request(target, 123, "different", None))

    def test_reused_PID_or_wrong_launcher_is_refused_without_writing(self):
        for filename, content in (("stat", "123 (bash) S " + "0 " * 18 + "999 0"),
                                  ("cmdline", "bash\0other-script.sh\0")):
            path = self.process / filename
            original = path.read_bytes()
            path.write_bytes(content.encode())
            with self.assertRaises(ValueError):
                self.request()
            path.write_bytes(original)
        self.assertFalse((self.project / ".run/full_run.user_stop.json").exists())

    def test_missing_active_process_is_refused(self):
        (self.project / ".run/full_run.pid").unlink()
        with self.assertRaises(FileNotFoundError):
            self.request()

    def test_malformed_request_for_this_live_process_is_not_silently_ignored(self):
        payload = self.request()
        del payload["request_id"]
        target = self.project / ".run/full_run.user_stop.json"
        target.write_text(json.dumps(payload))
        with self.assertRaises(ValueError):
            read_stop_request(target, 123, "456", None)


if __name__ == "__main__":
    unittest.main()
