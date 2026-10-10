"""Deterministic state/actuator checks without a robot or ROS."""
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

from jetson.amr_core.full_run import ControlStatus, FullRunConfig, FullRunController, SideReading
from jetson.amr_core.full_run_log import FlightRecorder
from jetson.amr_core.reactive_avoidance import CubicBezierPath, BezierPathFollower, FusedObject


ROOT = Path(__file__).resolve().parents[1]


class FullRunTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.config = FullRunConfig.read(ROOT / "config/full_run.json")
        self.events = []
        self.core = FullRunController(self.config, 123, self.now,
                                      lambda event, **data: self.events.append((event, data)))
        self.status = ControlStatus(123, 0, 0, 0, 0, 0, 0, 0, 0, 1000)
        self.feed()
        self.step()
        self.assertEqual(self.core.state, "READY")

    def feed(self, points=None, front=4.0, left=None, right=None, detections=None):
        self.status = replace(self.status, uptime_ms=int(self.now * 1000))
        self.core.update_status(self.status, self.now)
        self.core.update_camera(detections or [], self.now)
        self.core.update_scan(points or [], front, self.now)
        for side, distance in (("left", left), ("right", right)):
            self.core.update_side(SideReading(side, self.now, "NO_ECHO" if distance is None else "VALID", distance))

    def step(self, refresh=True, **feeds):
        self.now += 0.02
        if refresh:
            self.feed(**feeds)
        return self.core.tick(self.now)

    def up(self, base=10):
        self.status = replace(self.status, up_count=self.status.up_count + 1, base_rpm=base, stop_flags=0)
        return self.step()

    def start(self):
        self.up()
        self.assertEqual(self.core.state, "RUNNING")

    def target(self, distance=1.95):
        return FusedObject(1, "box", 0.9, 3, distance, distance, 0, -0.08, 0.08,
                           -0.16, 0.16, True)

    def passing(self):
        self.start()
        self.core.target = self.target()
        self.core.target_world = (1.95, 0.0)
        self.core.turn_left = True
        self.core.lane_y = 0.86
        self.core.odom.y_m = 0.86
        self.core.set_phase("BASELINE")
        for _ in range(5):
            self.step()
        self.assertEqual(self.core.phase, "PASS")

    def echo(self, distance=0.45, corroborate=True):
        points = [(math.radians(-90), distance + self.config.side_sensor_y_m)] if corroborate else []
        return self.step(points=points, right=distance)

    def clear_tail(self):
        self.core.odom.distance_travelled_m += 0.10
        for _ in range(self.config.tail_clear_samples):
            self.step()

    def test_configuration_geometry(self):
        self.assertAlmostEqual(self.config.expected_side_range_m, 0.40)
        self.assertAlmostEqual(self.config.rear_clearance_m, 0.575)
        self.assertEqual(self.config.side_safety_margin_m, 0.20)
        self.assertEqual(self.config.maximum_rpm, 75)
        self.assertEqual(self.config.entry_rpm, 36)
        self.assertEqual(self.config.bypass_rpm, 48)
        self.assertEqual(self.config.return_rpm, 42)
        self.assertEqual(self.config.acceleration_rpm_s, 10.0)
        self.assertEqual(self.config.turn_acceleration_rpm_s, 15.0)
        self.assertEqual(self.config.turn_deceleration_rpm_s, 25.0)
        self.assertEqual(self.config.avoidance_handle_ratio, 0.30)
        self.assertEqual(self.config.minimum_entry_forward_m, 0.30)

    def test_config_rejects_overlapping_thresholds_and_duplicate_pins(self):
        for config in (replace(self.config, side_stop_m=0.45), replace(self.config, right_echo=31),
                       replace(self.config, maximum_rpm=80), replace(self.config, wheel_base_m=0)):
            with self.assertRaises(ValueError):
                config.validate()

    def test_idle_never_auto_starts(self):
        for _ in range(200):
            self.assertEqual(self.step(), (0, 0))
        self.assertEqual(self.core.state, "READY")

    def test_stale_pre_start_up_not_reused(self):
        core = FullRunController(self.config, 123, 100)
        status = replace(self.status, base_rpm=5, up_count=1)
        core.update_status(status, 100.01)
        self.assertEqual(core.state, "FAULT_STOP")

    def test_speed_ramps_and_is_capped(self):
        self.start()
        previous = 0
        for _ in range(100):
            command = self.step()
            self.assertLessEqual(max(command), 10)
            self.assertGreaterEqual(command[0], previous)
            previous = command[0]
        self.assertEqual(command, (10, 10))

    def test_phase_speed_limits_scale_with_75_rpm_drive_limit(self):
        self.start()
        self.status = replace(self.status, base_rpm=75)
        self.feed()
        self.assertEqual(self.core.desired_command(), (75, 75))

        self.core.set_phase("ENTRY")
        self.core.make_path(2.0, 0.0)
        self.assertEqual(self.core.desired_command(), (36, 36))

        self.core.lane_y = 0.0
        self.core.set_phase("PASS")
        self.assertEqual(self.core.desired_command(), (48, 48))

        self.core.set_phase("RETURN")
        self.core.make_path(2.0, 0.0)
        self.assertEqual(self.core.desired_command(), (42, 42))

    def test_entry_and_return_use_faster_but_still_limited_steering_slew(self):
        self.start()
        self.core.ramped = [0.0, 0.0]
        self.core.set_phase("ENTRY")
        entry = self.core.ramp_command((1, 23), 0.1)
        self.assertEqual(entry, (1, 2))
        self.assertEqual(self.core.ramped, [1.0, 1.5])

        self.core.ramped = [16.0, 16.0]
        self.core.set_phase("RETURN")
        returning = self.core.ramp_command((23, 1), 0.1)
        self.assertEqual(returning, (18, 14))
        self.assertEqual(self.core.ramped, [17.5, 13.5])

    def test_entry_and_return_paths_use_sharper_shared_curve(self):
        self.start()
        self.core.make_path(1.2, 0.9)
        chord = math.hypot(1.2, 0.9)
        expected_handle = chord * self.config.avoidance_handle_ratio
        self.assertAlmostEqual(self.core.follower.path.p1[0], expected_handle)
        self.assertAlmostEqual(self.core.follower.path.p2[0], 1.2 - expected_handle)

    def test_slightly_reduced_forward_room_can_start_entry(self):
        self.start()
        self.core.target = self.target()
        obstacle_x = (self.config.robot_length_m / 2
                      + self.config.side_safety_margin_m + 0.31)
        self.core.target_world = (obstacle_x, 0.0)
        self.core.choose_entry()
        self.assertEqual(self.core.state, "RUNNING")
        self.assertEqual(self.core.phase, "ENTRY")

    def test_too_little_forward_room_still_pauses(self):
        self.start()
        self.core.target = self.target()
        obstacle_x = (self.config.robot_length_m / 2
                      + self.config.side_safety_margin_m + 0.29)
        self.core.target_world = (obstacle_x, 0.0)
        self.core.choose_entry()
        self.assertEqual(self.core.state, "PAUSED")
        self.assertEqual(self.core.reason, "insufficient_entry_forward_room")

    def test_environmental_pause_decelerates_and_resume_accelerates(self):
        self.start()
        for _ in range(120):
            command = self.step()
        self.assertEqual(command, (10, 10))
        first = self.step(front=0.20)
        self.assertEqual(self.core.state, "PAUSED")
        self.assertGreater(first[0], 0)
        self.assertLess(self.core.ramped[0], 10)
        previous = first[0]
        for _ in range(80):
            command = self.step(front=0.20)
            self.assertLessEqual(command[0], previous)
            previous = command[0]
        self.assertEqual(command, (0, 0))
        self.step(front=4.0)
        resumed = self.up(10)
        self.assertEqual(self.core.state, "RUNNING")
        self.assertLess(resumed[0], 10)

    def test_faults_are_suppressed_while_paused_and_recovery_can_resume(self):
        self.start()
        self.step(front=0.20)
        self.assertEqual(self.core.state, "PAUSED")
        self.core.update_side(SideReading("right", self.now, "FAULT", detail="lifted"))
        self.assertEqual(self.core.state, "PAUSED")
        self.assertEqual(self.core.paused_fault_reason, "ultrasonic_right:lifted")
        self.status = replace(self.status, up_count=self.status.up_count + 1, base_rpm=10)
        self.now += 0.02
        self.status = replace(self.status, uptime_ms=int(self.now * 1000))
        self.core.update_status(self.status, self.now)
        self.core.update_camera([], self.now)
        self.core.update_scan([], 4.0, self.now)
        self.core.update_side(SideReading("left", self.now, "NO_ECHO"))
        self.core.update_side(SideReading("right", self.now, "FAULT", detail="lifted"))
        self.core.tick(self.now)
        self.assertEqual(self.core.state, "PAUSED")
        self.assertFalse(self.core.pending_up)
        self.feed(front=4.0)
        self.step()
        self.assertEqual(self.core.state, "PAUSED")
        self.up(10)
        self.assertEqual(self.core.state, "RUNNING")
        self.assertIsNone(self.core.paused_fault_reason)

    def test_user_pause_preserves_follower_and_phase(self):
        self.start()
        path = CubicBezierPath.from_poses(start_x_m=0, start_y_m=0, start_yaw_rad=0,
                                         goal_x_m=1.2, goal_y_m=0.86, goal_yaw_rad=0)
        follower = BezierPathFollower(path)
        self.core.follower = follower
        self.core.set_phase("ENTRY")
        self.status = replace(self.status, base_rpm=0, down_count=1)
        self.assertEqual(self.step(), (0, 0))
        active = self.core.active_s
        for _ in range(500):
            self.step()
        self.assertEqual(self.core.active_s, active)
        self.assertIs(self.core.follower, follower)
        self.up(5)
        self.assertEqual(self.core.state, "RUNNING")
        self.assertEqual(self.core.phase, "ENTRY")
        self.assertIs(self.core.follower, follower)

    def test_instant_stop_resumes_with_saved_speed(self):
        self.start()
        self.status = replace(self.status, stop_count=1, stop_flags=3)
        self.assertEqual(self.step(), (0, 0))
        self.assertEqual(self.core.state, "PAUSED")
        self.status = replace(self.status, stop_flags=2)
        self.step()
        self.up(10)
        self.assertEqual(self.core.state, "RUNNING")
        self.assertEqual(self.status.base_rpm, 10)

    def test_held_stop_blocks_up_and_requires_new_press_after_release(self):
        self.start()
        self.status = replace(self.status, stop_count=1, stop_flags=3)
        self.step()
        self.status = replace(self.status, up_count=1)
        self.step()
        self.assertEqual(self.core.state, "PAUSED")
        self.status = replace(self.status, stop_flags=0)
        self.step()
        self.assertEqual(self.core.state, "PAUSED")
        self.up()
        self.assertEqual(self.core.state, "RUNNING")

    def test_environmental_stop_up_is_not_queued(self):
        self.start()
        self.step(front=0.20)
        self.assertEqual(self.core.state, "PAUSED")
        self.status = replace(self.status, up_count=2)
        self.step(front=0.20)
        self.step(front=4)
        self.assertEqual(self.core.state, "PAUSED")
        self.up()
        self.assertEqual(self.core.state, "RUNNING")

    def test_missing_mcu_stops_and_cannot_resume_with_up(self):
        self.start()
        for _ in range(30):
            self.now += 0.02
            self.core.update_camera([], self.now)
            self.core.update_scan([], 4, self.now)
            for side in ("left", "right"):
                self.core.update_side(SideReading(side, self.now, "NO_ECHO"))
            self.core.tick(self.now)
        self.assertEqual(self.core.state, "FAULT_STOP")
        self.assertEqual(self.up(), (0, 0))

    def test_transient_camera_staleness_pauses_and_consumes_up_until_recovery(self):
        self.start()
        for _ in range(60):
            self.now += 0.02
            self.status = replace(self.status, uptime_ms=int(self.now * 1000))
            self.core.update_status(self.status, self.now)
            self.core.update_scan([], 4, self.now)
            for side in ("left", "right"):
                self.core.update_side(SideReading(side, self.now, "NO_ECHO"))
            command = self.core.tick(self.now)
        self.assertEqual(command, (0, 0))
        self.assertEqual(self.core.state, "PAUSED")
        self.assertIsNone(self.core.fault_reason)

        # An UP while the camera is still stale is consumed, not queued for
        # an automatic departure when frames return.
        self.status = replace(self.status, up_count=self.status.up_count + 1, base_rpm=10)
        self.now += 0.02
        self.core.update_status(self.status, self.now)
        self.core.update_scan([], 4, self.now)
        for side in ("left", "right"):
            self.core.update_side(SideReading(side, self.now, "NO_ECHO"))
        self.assertEqual(self.core.tick(self.now), (0, 0))
        self.feed()
        self.assertEqual(self.step(), (0, 0))
        self.assertEqual(self.core.state, "PAUSED")
        self.up(10)
        self.assertEqual(self.core.state, "RUNNING")

    def test_startup_waits_for_external_clearance_and_reports_blocker(self):
        config = replace(self.config, startup_timeout_s=0.05)
        core = FullRunController(config, 123, self.now,
                                 lambda event, **data: self.events.append((event, data)))
        point = (math.atan2(0.4, 0.3), 0.5)
        core.update_status(self.status, self.now)
        core.update_camera([], self.now)
        core.update_scan([point], 4, self.now)
        for side in ("left", "right"):
            core.update_side(SideReading(side, self.now, "NO_ECHO"))
        self.assertEqual(core.tick(self.now + 0.02), (0, 0))
        self.assertEqual(core.state, "STARTUP")
        self.assertEqual(core.tick(self.now + 0.06), (0, 0))
        self.assertEqual(core.state, "FAULT_STOP")
        self.assertIn("resume_blocker:front_clearance", core.fault_reason)

    def test_full_width_front_corner_guard_uses_chassis_length(self):
        self.start()
        point = (math.atan2(0.4, 0.3), 0.5)
        self.assertEqual(self.step(points=[point]), (0, 0))
        self.assertEqual(self.core.state, "PAUSED")
        self.assertLess(self.core.front_clearance_m(), 0)

    def test_watchdog_fault_is_distinct_from_user_stop(self):
        self.start()
        self.status = replace(self.status, fault=1)
        self.assertEqual(self.step(), (0, 0))
        self.assertEqual(self.core.state, "FAULT_STOP")

    def test_reboot_or_foreign_session_after_start_is_fault(self):
        self.start()
        self.core.update_status(replace(self.status, session=456), self.now)
        self.assertEqual(self.core.state, "FAULT_STOP")

    def test_cumulative_encoders_including_wrap_and_pause(self):
        self.start()
        uptime = self.status.uptime_ms
        self.core.update_status(replace(self.status, left_counts=0xfffffff0, right_counts=0xfffffff0,
                                        uptime_ms=uptime + 100), self.now)
        initial_x = self.core.odom.x_m
        self.core.update_status(replace(self.status, left_counts=0x100, right_counts=0x100,
                                        uptime_ms=uptime + 200, base_rpm=0), self.now + 0.1)
        expected = (0x100 + 0x10) * math.pi * self.config.wheel_diameter_m / 40000
        self.assertAlmostEqual(self.core.odom.x_m - initial_x, expected)
        self.assertEqual(self.core.state, "PAUSED")

    def test_loop_stall_and_no_wheel_progress_stop(self):
        self.start()
        self.now += 1
        self.feed()
        self.assertEqual(self.core.tick(self.now), (0, 0))
        self.assertEqual(self.core.state, "FAULT_STOP")

    def test_no_wheel_feedback_progress_is_fault(self):
        self.start()
        for _ in range(400):
            self.step()
        self.assertEqual(self.core.state, "FAULT_STOP")
        self.assertIn("wheel_feedback", self.core.fault_reason)

    def test_side_echo_fault_is_not_clear(self):
        self.start()
        self.core.update_side(SideReading("right", self.now, "FAULT", detail="stuck_high"))
        self.assertEqual(self.core.state, "FAULT_STOP")

    def test_constant_no_echo_cannot_confirm_tail(self):
        self.passing()
        for _ in range(40):
            self.core.odom.distance_travelled_m += 0.01
            self.step()
        self.assertFalse(self.core.side_seen)
        self.assertIsNone(self.core.tail_at)
        self.assertEqual(self.core.phase, "PASS")

    def test_thin_pole_one_echo_with_lidar_is_armed(self):
        self.passing()
        self.echo()
        self.assertTrue(self.core.side_seen)
        self.clear_tail()
        self.assertEqual(self.core.phase, "REAR_CLEARANCE")

    def test_single_echo_without_lidar_cannot_arm(self):
        self.passing()
        self.echo(corroborate=False)
        self.clear_tail()
        self.assertFalse(self.core.side_seen)
        self.assertEqual(self.core.phase, "PASS")

    def test_long_object_waits_until_far_then_adds_rear_length(self):
        self.passing()
        for _ in range(30):
            self.core.odom.distance_travelled_m += 0.05
            self.echo()
        self.assertEqual(self.core.phase, "PASS")
        self.clear_tail()
        self.assertEqual(self.core.phase, "REAR_CLEARANCE")
        self.core.odom.x_m += 0.57
        self.step()
        self.assertEqual(self.core.phase, "REAR_CLEARANCE")
        self.core.odom.x_m += 0.01
        self.step()
        self.assertEqual(self.core.phase, "RETURN")

    def test_no_echo_with_lidar_still_close_cannot_confirm_tail(self):
        self.passing()
        self.echo()
        self.core.odom.distance_travelled_m += 0.1
        for _ in range(10):
            self.step(points=[(-math.pi / 2, 0.70)])
        self.assertEqual(self.core.phase, "PASS")
        self.assertIsNone(self.core.tail_at)

    def test_paused_samples_cannot_advance_tail_or_reset_path(self):
        self.passing()
        self.echo()
        self.status = replace(self.status, base_rpm=0, down_count=1)
        self.step()
        for _ in range(20):
            self.step()
        self.assertTrue(self.core.side_seen)
        self.assertIsNone(self.core.tail_at)
        self.up()
        self.assertEqual(self.core.clear_count, 0)

    def test_backward_movement_does_not_count_as_rear_clearance(self):
        self.passing()
        self.echo()
        self.clear_tail()
        self.core.odom.distance_travelled_m += 1
        self.core.odom.x_m -= 1
        self.step()
        self.assertEqual(self.core.phase, "REAR_CLEARANCE")

    def test_second_object_cancels_rear_clearance(self):
        self.passing()
        self.echo()
        self.clear_tail()
        self.echo()
        self.assertEqual(self.core.phase, "PASS")
        self.assertIsNone(self.core.tail_at)

    def test_near_baseline_does_not_count_as_returned_background(self):
        self.start()
        self.core.set_phase("BASELINE")
        for _ in range(5):
            self.echo()
        self.assertAlmostEqual(self.core.baseline_value, 0.45)
        for _ in range(10):
            self.core.odom.distance_travelled_m += 0.02
            self.echo()
        self.assertIsNone(self.core.tail_at)

    def test_seek_timeout_is_a_sensor_fault_not_auto_return(self):
        self.passing()
        self.core.odom.distance_travelled_m = self.core.pass_start + 3.1
        self.step()
        self.assertEqual(self.core.state, "FAULT_STOP")

    def test_entry_frozen_after_detection_lost(self):
        self.start()
        self.core.freeze_target(self.target())
        for _ in range(20):
            self.step()
        self.assertEqual(self.core.phase, "ENTRY")
        self.assertIsNotNone(self.core.follower)

    def test_new_object_in_passing_lane_is_not_hidden_by_long_object_matching(self):
        self.passing()
        new = replace(self.target(), track_id=8, distance_m=1.0, forward_distance_m=1.0)
        self.core.objects = [new]
        self.assertIs(self.core.new_target(), new)

    def test_full_ideal_wheel_sequence_with_pause_and_repeated_avoidance(self):
        self.start()
        for run in range(2):
            self.core.freeze_target(self.target())
            paused = False
            initial_x = self.core.odom.x_m
            for tick in range(20000):
                self.status = replace(self.status, left_rpm=self.core.command[0], right_rpm=self.core.command[1])
                feeds = {}
                if self.core.phase == "PASS":
                    travelled = self.core.odom.distance_travelled_m - self.core.pass_start
                    if travelled < 0.35:
                        feeds = {"right": 0.45, "points": [(-math.pi / 2, 0.70)]}
                if self.core.phase == "ENTRY" and not paused and self.core.follower.progress_ratio > 0.25:
                    follower = self.core.follower
                    self.status = replace(self.status, base_rpm=0, down_count=self.status.down_count + 1,
                                          left_rpm=0, right_rpm=0)
                    self.step()
                    for _ in range(20):
                        self.step()
                    self.up(10)
                    self.assertIs(self.core.follower, follower)
                    paused = True
                self.step(**feeds)
                self.assertEqual(self.core.state, "RUNNING", self.core.snapshot(self.now))
                if self.core.phase == "DRIVE":
                    self.assertLessEqual(abs(self.core.odom.y_m), self.config.path_completion_m)
                    self.assertGreater(self.core.odom.x_m, initial_x)
                    break
            else:
                self.fail(f"Did not complete avoidance: {self.core.snapshot(self.now)}")


class ProtocolAndLogTests(unittest.TestCase):
    def test_ctrl_exact_format(self):
        status = ControlStatus.decode("$CTRL,1,123,10,2,1,0,0,0,8,9,123456\r\n")
        self.assertEqual(status.base_rpm, 10)
        self.assertEqual(status.right_rpm, 9)
        counted = ControlStatus.decode("$CTRL,1,123,10,2,1,0,0,0,8,9,123456,1000,2000")
        self.assertEqual(counted.left_counts, 1000)
        for line in ("$STATUS,0,0,0,0,0,0,0,0,0,0", "$CTRL,2,123", "$CTRL,1,0,10,2,1,0,0,0,8,9,123456",
                     "$CTRL,1,123,65,2,1,0,0,0,8,9,123456"):
            with self.assertRaises(ValueError):
                ControlStatus.decode(line)

    def test_flight_recorder_rotation_timestamp_and_nonfinite(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = FlightRecorder(folder, max_bytes=500, backups=3)
            for index in range(15):
                recorder.emit("test", index=index, distance=math.inf, data="x" * 50)
            recorder.close()
            files = list(Path(folder).glob("events.jsonl*"))
            self.assertGreater(len(files), 1)
            self.assertLessEqual(len(files), 4)
            for file in files:
                for line in file.read_text(encoding="utf8").splitlines():
                    record = json.loads(line)
                    self.assertEqual(record["distance"], "+Inf")
                    self.assertIn("monotonic_s", record)
                    self.assertIn("utc", record)

    def test_flight_recorder_prints_human_readable_startup_and_ready(self):
        import contextlib
        import io

        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()) as output:
            recorder = FlightRecorder(folder)
            recorder.emit("startup_step", stage="open_command_UART")
            recorder.emit("state", before="STARTUP", after="READY", reason="fresh_feeds_wait_new_UP")
            recorder.close()

        console = output.getvalue()
        self.assertIn("[STARTUP] open_command_UART", console)
        self.assertIn("[STATE] STARTUP -> READY: fresh_feeds_wait_new_UP", console)
        self.assertIn("[READY] Sensors and STM32 are live", console)


if __name__ == "__main__":
    unittest.main()
