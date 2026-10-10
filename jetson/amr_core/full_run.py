"""Continuous hardware-independent controller. No ROS/GPIO/serial imports.

Pause keeps the follower, frozen obstacle and side-passing evidence. New UP
events are required after a stop; a blocked UP is consumed, never queued.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path
import statistics

from .reactive_avoidance import (
    BezierPathFollower, CubicBezierPath, DetectionTracker, FusionConfig,
    WheelOdometry, fuse_confirmed_tracks,
    line_heading_error_rad, local_point_to_world, passing_lateral_offset_m,
    select_priority_target,
)


@dataclass(frozen=True)
class FullRunConfig:
    wheel_diameter_m: float = 0.2032
    wheel_base_m: float = 0.85
    encoder_counts_per_output_rev: int = 40000
    robot_width_m: float = 0.90
    robot_length_m: float = 0.85
    side_safety_margin_m: float = 0.25
    rear_safety_margin_m: float = 0.15
    side_sensor_x_m: float = 0.0
    side_sensor_y_m: float = 0.25
    lidar_x_m: float = 0.0
    lidar_y_m: float = 0.0
    lidar_yaw_deg: float = 0.0
    image_width_px: int = 640
    camera_horizontal_fov_deg: float = 69.0
    camera_lidar_yaw_offset_deg: float = 0.0
    minimum_confidence: float = 0.45
    detection_frames: int = 3
    trigger_distance_m: float = 2.0
    maximum_rpm: int = 25
    entry_rpm: int = 12
    bypass_rpm: int = 16
    return_rpm: int = 14
    acceleration_rpm_s: float = 5.0
    deceleration_rpm_s: float = 10.0
    turn_acceleration_rpm_s: float = 15.0
    turn_deceleration_rpm_s: float = 25.0
    avoidance_handle_ratio: float = 0.30
    lookahead_m: float = 0.25
    path_completion_m: float = 0.08
    sensor_timeout_s: float = 1.0
    mcu_timeout_s: float = 0.5
    startup_timeout_s: float = 90.0
    front_stop_m: float = 0.25
    front_resume_m: float = 0.40
    side_stop_m: float = 0.30
    side_resume_m: float = 0.36
    entry_side_clear_m: float = 0.80
    side_object_max_m: float = 0.70
    side_clear_min_m: float = 0.85
    baseline_tolerance_m: float = 0.10
    baseline_samples: int = 5
    tail_clear_samples: int = 5
    tail_min_travel_m: float = 0.03
    seek_max_distance_m: float = 3.0
    pass_max_distance_m: float = 10.0
    path_timeout_s: float = 120.0
    no_progress_timeout_s: float = 5.0
    left_trig: int = 33
    left_echo: int = 31
    right_trig: int = 7
    right_echo: int = 15
    ultrasonic_interval_s: float = 0.06
    ultrasonic_timeout_s: float = 0.03
    ultrasonic_max_m: float = 3.0
    log_max_bytes: int = 16 * 1024 * 1024
    log_backup_count: int = 24

    @classmethod
    def read(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        unknown = set(data) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown configuration keys: {sorted(unknown)}")
        config = cls(**data)
        config.validate()
        return config

    def validate(self):
        signed = {"side_sensor_x_m", "lidar_x_m", "lidar_y_m", "lidar_yaw_deg",
                  "camera_lidar_yaw_offset_deg"}
        zero_allowed = {"side_safety_margin_m", "rear_safety_margin_m"}
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{key}: finite numeric value required")
            if key not in signed and (value < 0 if key in zero_allowed else value <= 0):
                raise ValueError(f"{key}: invalid sign")
        for key in ("image_width_px", "encoder_counts_per_output_rev", "maximum_rpm", "entry_rpm", "bypass_rpm", "return_rpm",
                    "detection_frames", "baseline_samples", "tail_clear_samples", "left_trig",
                    "left_echo", "right_trig", "right_echo", "log_max_bytes", "log_backup_count"):
            if type(getattr(self, key)) is not int:
                raise ValueError(f"{key}: integer required")
        if not 1 <= self.maximum_rpm <= 25:
            raise ValueError("maximum_rpm must be 1..25 (Jetson full-run speed limit)")
        if not 0 < self.minimum_confidence <= 1 or not 1 < self.camera_horizontal_fov_deg < 179:
            raise ValueError("Invalid confidence/camera FOV")
        if not 0.1 <= self.avoidance_handle_ratio <= 0.5:
            raise ValueError("avoidance_handle_ratio must be within 0.1..0.5")
        if len({self.left_trig, self.left_echo, self.right_trig, self.right_echo}) != 4:
            raise ValueError("GPIO pins must be distinct")
        if any(pin > 40 for pin in (self.left_trig, self.left_echo, self.right_trig, self.right_echo)):
            raise ValueError("BOARD GPIO pins must be within the 40-pin header")
        if not self.front_stop_m < self.front_resume_m:
            raise ValueError("front stop/resume thresholds overlap")
        if not self.side_stop_m < self.side_resume_m < self.side_object_max_m < self.side_clear_min_m:
            raise ValueError("side stop/resume/object/clear thresholds overlap")
        if self.side_clear_min_m >= self.ultrasonic_max_m:
            raise ValueError("Side clear threshold exceeds ultrasonic range")
        if self.side_sensor_y_m > self.robot_width_m / 2:
            raise ValueError("Side sensor is outside robot footprint")
        if self.side_sensor_x_m < -self.robot_length_m / 2:
            raise ValueError("Side sensor is behind robot rear")
        if self.expected_side_range_m <= self.side_resume_m:
            raise ValueError("Planned side distance is too close to the stop/resume threshold")

    @property
    def expected_side_range_m(self):
        return self.robot_width_m / 2 - self.side_sensor_y_m + self.side_safety_margin_m

    @property
    def rear_clearance_m(self):
        return self.robot_length_m / 2 + self.side_sensor_x_m + self.rear_safety_margin_m


@dataclass(frozen=True)
class ControlStatus:
    session: int
    base_rpm: int
    up_count: int
    down_count: int
    stop_count: int
    stop_flags: int  # bit 0: button held, bit 1: instant-stop latched
    fault: int       # 1: command watchdog, 2: firmware/hardware fault
    left_rpm: float
    right_rpm: float
    uptime_ms: int
    left_counts: int | None = None
    right_counts: int | None = None

    @classmethod
    def decode(cls, line: str):
        parts = line.strip().split(",")
        if len(parts) not in (12, 14) or parts[:2] != ["$CTRL", "1"]:
            raise ValueError("Expected $CTRL version 1, 12/14 fields; use legacy mode for existing $STATUS firmware")
        vals = [int(v) for v in parts[2:]]
        s = cls(*vals)
        if (not 1 <= s.session <= 0x7fffffff or not 0 <= s.base_rpm <= 25
                or not 0 <= s.stop_flags <= 3 or s.fault not in (0, 1, 2)
                or any(not 0 <= v <= 0xffffffff for v in
                       (s.up_count, s.down_count, s.stop_count, s.uptime_ms))
                or abs(s.left_rpm) > 1000 or abs(s.right_rpm) > 1000):
            raise ValueError("Invalid CTRL range")
        if any(v is not None and not 0 <= v <= 0xffffffff for v in (s.left_counts, s.right_counts)):
            raise ValueError("Invalid encoder count")
        return s


@dataclass(frozen=True)
class SideReading:
    side: str
    stamp: float
    status: str  # VALID, NO_ECHO, FAULT
    distance_m: float | None = None
    pulse_s: float | None = None
    detail: str = ""


class FullRunPathFollower(BezierPathFollower):
    def heading_error_rad(self, *, pose_x_m, pose_y_m, pose_yaw_rad):
        self._update_progress(pose_x_m, pose_y_m)
        if self.progress_ratio >= 0.90:
            # All full-run goals face the original +x axis. Keep following the
            # endpoint's forward line even after overshoot; a fixed point
            # behind the robot would otherwise cause a U-turn.
            return line_heading_error_rad(pose_y_m=pose_y_m, target_y_m=self.path.p3[1],
                                           pose_yaw_rad=pose_yaw_rad, lookahead_m=self.lookahead_m)
        return super().heading_error_rad(pose_x_m=pose_x_m, pose_y_m=pose_y_m,
                                         pose_yaw_rad=pose_yaw_rad)


class FullRunController:
    def __init__(self, config: FullRunConfig, session: int, now: float, emit=None, *, mcu_protocol="ctrl"):
        config.validate()
        if mcu_protocol not in ("legacy", "ctrl"):
            raise ValueError("mcu_protocol must be legacy or ctrl")
        self.c = config
        self.mcu_protocol = mcu_protocol
        self.session = session
        self.emit = emit or (lambda *_args, **_kwargs: None)
        self.state = "STARTUP"
        self.phase = "DRIVE"
        self.reason = "waiting_for_fresh_sensors_and_" + ("STATUS" if mcu_protocol == "legacy" else "CTRL")
        self.created = self.last_tick = now
        self.active_s = self.phase_started = 0.0
        self.status = None
        self.status_stamp = self.camera_stamp = self.scan_stamp = None
        self.front = None
        self.points = []
        self.sides = {}
        self.tracker = DetectionTracker(minimum_confidence=config.minimum_confidence,
                                        required_frames=config.detection_frames)
        self.fusion = FusionConfig(image_width_px=config.image_width_px,
                                   camera_horizontal_fov_deg=config.camera_horizontal_fov_deg,
                                   camera_lidar_yaw_offset_deg=config.camera_lidar_yaw_offset_deg,
                                   robot_width_m=config.robot_width_m,
                                   safety_margin_m=config.side_safety_margin_m,
                                   avoidance_trigger_distance_m=config.trigger_distance_m)
        self.odom = WheelOdometry(wheel_diameter_m=config.wheel_diameter_m,
                                  wheel_base_m=config.wheel_base_m)
        self.objects = []
        self.target = None
        self.target_world = None
        self.follower = None
        self.turn_left = True
        self.lane_y = 0.0
        self.baseline = []
        self.baseline_value = None
        self.side_seen = False
        self.near_samples = 0
        self.clear_count = 0
        self.seen_at = self.tail_at = None
        self.tail_x_m = None
        self.pass_start = 0.0
        self.last_side_stamp = -math.inf
        self.ramped = [0.0, 0.0]
        self.command = (0, 0)
        self.progress_at = now
        self.progress_distance = 0.0
        self.motion_expected_since = None
        self.resume_up_floor = 0
        self.pending_up = False
        self.fault_reason = None
        self.paused_fault_reason = None
        self.preflight_ready = False

    def stop_now(self):
        self.ramped = [0.0, 0.0]
        self.command = (0, 0)
        self.motion_expected_since = None

    def ramp_command(self, target, dt):
        turning = (self.state == "RUNNING" and self.phase in ("ENTRY", "RETURN")
                   and max(target) > 0)
        acceleration = self.c.turn_acceleration_rpm_s if turning else self.c.acceleration_rpm_s
        deceleration = self.c.turn_deceleration_rpm_s if turning else self.c.deceleration_rpm_s
        for i, value in enumerate(target):
            rate = acceleration if value > self.ramped[i] else deceleration
            delta = max(-rate * dt, min(rate * dt, value - self.ramped[i]))
            self.ramped[i] += delta
        self.command = tuple(round(value) for value in self.ramped)
        return self.command

    def change(self, state, reason, *, immediate=False):
        old = self.state
        self.state, self.reason = state, reason
        if state != "RUNNING":
            if immediate or state in ("STARTUP", "READY", "FAULT_STOP"):
                self.stop_now()
            self.pending_up = False
            if self.status:
                self.resume_up_floor = self.status.up_count
        if state == "RUNNING":
            self.paused_fault_reason = None
        self.emit("state", before=old, after=state, phase=self.phase, reason=reason,
                  immediate_stop=immediate, pose=self.pose())

    def set_phase(self, phase):
        before = self.phase
        self.phase, self.phase_started = phase, self.active_s
        self.emit("phase", before=before, after=phase, pose=self.pose())

    def fault(self, reason):
        if self.state == "PAUSED":
            if reason != self.paused_fault_reason:
                self.paused_fault_reason = reason
                self.emit("fault_suppressed_while_paused", reason=reason, phase=self.phase,
                          pose=self.pose())
            return
        if self.state != "FAULT_STOP":
            self.fault_reason = reason
            self.change("FAULT_STOP", reason, immediate=True)

    def pause(self, reason, *, immediate=False):
        if self.state == "RUNNING":
            self.change("PAUSED", reason, immediate=immediate)
            self.clear_count = 0  # Old samples must not count after resume.

    def pose(self):
        return {"x": self.odom.x_m, "y": self.odom.y_m, "yaw": self.odom.yaw_rad,
                "distance": self.odom.distance_travelled_m}

    def update_status(self, status, now):
        if status.session != self.session:
            self.emit("foreign_CTRL", received_session=status.session, expected=self.session)
            if self.status is not None:
                self.fault("STM32_session_changed_or_rebooted")
            return
        previous = self.status
        self.status, self.status_stamp = status, now
        if previous is None:
            self.resume_up_floor = status.up_count
            if self.mcu_protocol == "ctrl" and (status.base_rpm != 0 or status.stop_flags):
                self.fault("handshake_must_reset_speed_and_stop_flags")
        else:
            if status.left_counts is not None and previous.left_counts is not None:
                elapsed = ((status.uptime_ms - previous.uptime_ms) & 0xffffffff) / 1000
                if 0 < elapsed <= self.c.mcu_timeout_s:
                    left_delta = ((status.left_counts - previous.left_counts + 0x80000000) & 0xffffffff) - 0x80000000
                    right_delta = ((status.right_counts - previous.right_counts + 0x80000000) & 0xffffffff) - 0x80000000
                    factor = 60 / (self.c.encoder_counts_per_output_rev * elapsed)
                    if max(abs(left_delta * factor), abs(right_delta * factor)) > 1000:
                        self.fault("encoder_counter_jump_or_wrong_counts_per_revolution")
                        return
                    self.odom.update(left_delta * factor, right_delta * factor, elapsed)
            if status.uptime_ms < previous.uptime_ms and previous.uptime_ms - status.uptime_ms < 0x80000000:
                self.fault("STM32_uptime_regressed")
            if status.stop_count != previous.stop_count or status.stop_flags:
                self.pause("user_instant_stop", immediate=True)
            elif status.base_rpm == 0:
                self.pause("user_speed_zero")
            if status.up_count != previous.up_count:
                self.emit("up", count=status.up_count, base_rpm=status.base_rpm)
                if self.state in ("READY", "PAUSED") and status.up_count != self.resume_up_floor:
                    self.pending_up = True
            if status.down_count != previous.down_count:
                self.emit("down", count=status.down_count, base_rpm=status.base_rpm)
        if status.fault:
            self.fault(f"STM32_fault_{status.fault}")

    def update_camera(self, detections, now):
        for item in detections:
            box = item.get("xyxy", [])
            confidence = float(item.get("confidence", 0))
            if (len(box) != 4 or not all(math.isfinite(float(v)) for v in box)
                    or box[2] <= box[0] or box[3] <= box[1]
                    or not math.isfinite(confidence) or not 0 <= confidence <= 1):
                raise ValueError("Invalid YOLO detection")
        self.tracker.update(detections)
        self.camera_stamp = now

    def update_scan(self, points, front, now):
        self.points, self.front, self.scan_stamp = points, front, now
        self.objects = fuse_confirmed_tracks(self.tracker.confirmed_visible(), points, self.fusion)

    def update_side(self, reading):
        if reading.side not in ("left", "right") or reading.status not in ("VALID", "NO_ECHO", "FAULT"):
            raise ValueError("Invalid side reading")
        if reading.status == "VALID" and (reading.distance_m is None or not math.isfinite(reading.distance_m)
                                          or reading.distance_m < 0.02):
            self.fault(f"ultrasonic_{reading.side}:invalid_numeric_range")
            return
        self.sides[reading.side] = reading
        if reading.status == "FAULT" and self.state != "STARTUP":
            self.fault(f"ultrasonic_{reading.side}:{reading.detail}")

    @property
    def inside(self):
        return "right" if self.turn_left else "left"

    def fresh(self, stamp, now, timeout=None):
        return stamp is not None and 0 <= now - stamp <= (timeout or self.c.sensor_timeout_s)

    def health_errors(self, now):
        errors = []
        mcu_name = "STM32_STATUS" if self.mcu_protocol == "legacy" else "STM32_CTRL"
        for name, stamp in (("camera", self.camera_stamp), ("lidar", self.scan_stamp),
                            (mcu_name, self.status_stamp)):
            timeout = self.c.mcu_timeout_s if name == mcu_name else self.c.sensor_timeout_s
            if not self.fresh(stamp, now, timeout):
                errors.append(f"{name}_missing_or_stale")
        for side in ("left", "right"):
            r = self.sides.get(side)
            if r is None or not self.fresh(r.stamp, now) or r.status == "FAULT":
                errors.append(f"ultrasonic_{side}_missing_stale_or_fault")
        if self.status is not None and self.status.fault:
            errors.append(f"STM32_fault_{self.status.fault}")
        if self.front is None:
            errors.append("lidar_front_invalid_coverage")
        return errors

    def lateral_clearance(self, side):
        sign = 1 if side == "left" else -1
        values = [sign * d * math.sin(a) - self.c.side_sensor_y_m
                  for a, d in self.points
                  if abs(d * math.cos(a) - self.c.side_sensor_x_m) <= 0.18
                  and sign * math.sin(a) > 0]
        return min(values, default=math.inf)

    def side_safe(self, side, threshold):
        r = self.sides.get(side)
        return (r is not None and r.status in ("VALID", "NO_ECHO")
                and (r.status == "NO_ECHO" or r.distance_m > threshold)
                and self.lateral_clearance(side) > threshold)

    def entry_clear(self, side):
        sign = 1 if side == "left" else -1
        diagonal = [d for a, d in self.points if math.radians(20) <= sign * a <= math.radians(75)]
        return self.side_safe(side, self.c.side_resume_m) and min(diagonal, default=math.inf) >= self.c.entry_side_clear_m

    def blocking_reason(self, resume=False):
        front_threshold = self.c.front_resume_m if resume else self.c.front_stop_m
        if self.front is None or self.front_clearance_m() <= front_threshold:
            return "front_clearance"
        threshold = self.c.side_resume_m if resume else self.c.side_stop_m
        for side in ("left", "right"):
            if not self.side_safe(side, threshold):
                return f"{side}_clearance"
        if self.phase == "WAIT_DIRECTION" and self.target and not any(self.entry_clear(s) for s in ("left", "right")):
            return "no_entry_direction"
        return None

    def front_clearance_m(self):
        # Measure from the bumper across the whole body width. A narrow
        # +/-15deg radial sector alone misses the corners of a 90cm chassis.
        values = [d * math.cos(a) - self.c.robot_length_m / 2
                  for a, d in self.points if d * math.cos(a) > 0.01
                  and abs(d * math.sin(a)) <= self.c.robot_width_m / 2 + self.c.rear_safety_margin_m]
        return min(values, default=(self.front - self.c.robot_length_m / 2) if self.front is not None else -math.inf)

    def freeze_target(self, target):
        self.target = target
        self.target_world = local_point_to_world(origin_x_m=self.odom.x_m,
                                                origin_y_m=self.odom.y_m,
                                                origin_yaw_rad=self.odom.yaw_rad,
                                                forward_m=target.forward_distance_m,
                                                lateral_m=target.distance_m * math.sin(target.bearing_rad))
        self.emit("frozen_obstacle", target=asdict(target), world=self.target_world)
        self.set_phase("STOPPING")

    def new_target(self):
        candidates = []
        for target in self.objects:
            if not target.intersects_corridor or target.forward_distance_m > self.c.trigger_distance_m:
                continue
            world = local_point_to_world(origin_x_m=self.odom.x_m, origin_y_m=self.odom.y_m,
                                         origin_yaw_rad=self.odom.yaw_rad,
                                         forward_m=target.forward_distance_m,
                                         lateral_m=target.distance_m * math.sin(target.bearing_rad))
            if self.target_world and math.dist(world, self.target_world) <= 0.75:
                continue
            # A long face's matched LiDAR median moves along that face while
            # the robot passes; YOLO may also assign it a new ID. Recognize
            # continuation inside the frozen obstacle's lateral strip, without
            # ignoring a new object in the actual passing lane.
            if self.target_world and self.phase in ("ENTRY", "BASELINE", "PASS", "REAR_CLEARANCE"):
                half_span = abs(self.target.lateral_max_m - self.target.lateral_min_m) / 2 + 0.25
                if (target.label == self.target.label
                        and abs(world[1] - self.target_world[1]) <= half_span
                        and self.target_world[0] - 0.75 <= world[0] <= self.target_world[0] + self.c.pass_max_distance_m):
                    continue
            candidates.append(target)
        return min(candidates, key=lambda t: t.distance_m, default=None)

    def make_path(self, x, y):
        path = CubicBezierPath.from_poses(start_x_m=self.odom.x_m, start_y_m=self.odom.y_m,
                                         start_yaw_rad=self.odom.yaw_rad, goal_x_m=x,
                                         goal_y_m=y, goal_yaw_rad=0.0,
                                         handle_ratio=self.c.avoidance_handle_ratio)
        self.follower = FullRunPathFollower(path, lookahead_m=self.c.lookahead_m,
                                           completion_radius_m=self.c.path_completion_m,
                                           completion_heading_tolerance_rad=math.radians(3))
        self.emit("path", phase=self.phase, control_points=asdict(path))

    def choose_entry(self):
        preferred = "right" if self.target.bearing_rad > 0 else "left"
        if abs(self.target.bearing_rad) < math.radians(3):
            preferred = max(("left", "right"), key=self.lateral_clearance)
        options = [s for s in (preferred, "left" if preferred == "right" else "right") if self.entry_clear(s)]
        if not options:
            self.set_phase("WAIT_DIRECTION")
            self.pause("no_entry_direction")
            return
        # The frozen obstacle position, not a fresh detection, survives pauses.
        obstacle_local_x = ((self.target_world[0] - self.odom.x_m) * math.cos(self.odom.yaw_rad)
                            + (self.target_world[1] - self.odom.y_m) * math.sin(self.odom.yaw_rad))
        forward = obstacle_local_x - self.c.robot_length_m / 2 - self.c.side_safety_margin_m
        if forward < 0.65:
            self.pause("insufficient_entry_forward_room")
            return
        self.turn_left = options[0] == "left"
        lateral = passing_lateral_offset_m(self.target, turn_left=self.turn_left,
                                           corridor_half_width_m=self.fusion.corridor_half_width_m)
        x, y = local_point_to_world(origin_x_m=self.odom.x_m, origin_y_m=self.odom.y_m,
                                    origin_yaw_rad=self.odom.yaw_rad, forward_m=forward, lateral_m=lateral)
        self.lane_y = y
        self.baseline = []
        self.baseline_value = None
        self.side_seen = False
        self.near_samples = 0
        self.clear_count = 0
        self.seen_at = self.tail_at = None
        self.last_side_stamp = -math.inf
        self.set_phase("ENTRY")
        self.make_path(x, y)

    def side_evidence(self):
        reading = self.sides[self.inside]
        if reading.stamp <= self.last_side_stamp:
            return
        self.last_side_stamp = reading.stamp
        valid = reading.status == "VALID"
        near = valid and self.c.side_stop_m < reading.distance_m <= self.c.side_object_max_m
        # One valid close echo plus matching LiDAR is sufficient for a thin
        # pole. With no LiDAR corroboration require two consecutive echoes.
        lidar = self.lateral_clearance(self.inside)
        if near:
            self.clear_count = 0
            previous = getattr(self, "near_samples", 0)
            self.near_samples = previous + 1
            if not self.side_seen and (lidar <= self.c.side_object_max_m or self.near_samples >= 2):
                self.side_seen, self.seen_at = True, self.odom.distance_travelled_m
                self.emit("side_object_seen", side=self.inside, range_m=reading.distance_m, lidar_m=lidar)
        else:
            self.near_samples = 0
        if self.phase == "BASELINE":
            self.baseline.append(reading.distance_m if valid else None)
            if len(self.baseline) >= self.c.baseline_samples:
                values = [v for v in self.baseline if v is not None]
                self.baseline_value = statistics.median(values) if len(values) >= len(self.baseline) / 2 else None
                self.pass_start = self.odom.distance_travelled_m
                self.emit("side_baseline", side=self.inside, readings=self.baseline,
                          baseline_m=self.baseline_value, object_already_seen=self.side_seen)
                self.set_phase("PASS")
            return
        if not self.side_seen or self.phase != "PASS":
            return
        # A nearby baseline could itself be the obstacle. Returning to that
        # close value cannot prove the tail. Clear is always a FAR condition.
        far = reading.status == "NO_ECHO" or (valid and reading.distance_m >= self.c.side_clear_min_m)
        if self.baseline_value is not None and self.baseline_value >= self.c.side_clear_min_m and valid:
            far = far and reading.distance_m >= self.baseline_value - self.c.baseline_tolerance_m
        clear = (far and lidar >= self.c.side_clear_min_m
                 and self.odom.distance_travelled_m - self.seen_at >= self.c.tail_min_travel_m)
        self.clear_count = self.clear_count + 1 if clear else 0
        if self.clear_count >= self.c.tail_clear_samples:
            self.tail_at = self.odom.distance_travelled_m
            self.tail_x_m = self.odom.x_m
            self.emit("tail_confirmed", side=self.inside, range_m=reading.distance_m,
                      lidar_m=lidar, tail_at=self.tail_at, extra_m=self.c.rear_clearance_m)
            self.set_phase("REAR_CLEARANCE")

    def update_phase(self):
        if self.phase == "DRIVE":
            target = select_priority_target(self.objects, self.fusion)
            if target:
                self.freeze_target(target)
        elif self.phase in ("STOPPING", "WAIT_DIRECTION"):
            if max(abs(value) for value in self.ramped) < 0.5 and self.active_s - self.phase_started >= 0.3:
                self.choose_entry()
        elif self.phase in ("ENTRY", "RETURN"):
            if self.active_s - self.phase_started > self.c.path_timeout_s:
                self.pause("path_time_limit")
                self.phase_started = self.active_s
            elif self.follower.completed(pose_x_m=self.odom.x_m, pose_y_m=self.odom.y_m,
                                         pose_yaw_rad=self.odom.yaw_rad):
                if self.phase == "ENTRY":
                    self.set_phase("BASELINE")
                else:
                    self.target = self.target_world = self.follower = None
                    self.side_seen = False
                    self.set_phase("DRIVE")
        elif self.phase in ("BASELINE", "PASS", "REAR_CLEARANCE"):
            if self.phase != "BASELINE" or max(abs(value) for value in self.ramped) < 0.5:
                self.side_evidence()
            if self.phase == "PASS":
                distance = self.odom.distance_travelled_m - self.pass_start
                if not self.side_seen and distance > self.c.seek_max_distance_m:
                    self.fault("side_obstacle_never_seen_check_sensor_source_wiring_orientation")
                elif distance > self.c.pass_max_distance_m:
                    self.pause("pass_distance_limit")
                    self.pass_start = self.odom.distance_travelled_m
            if self.phase == "REAR_CLEARANCE":
                # A second close face cancels clearance; concave/segmented
                # obstacles must not trigger an early return.
                r = self.sides[self.inside]
                if ((r.status == "VALID" and r.distance_m <= self.c.side_object_max_m)
                        or self.lateral_clearance(self.inside) <= self.c.side_object_max_m):
                    self.tail_at = None
                    self.clear_count = 0
                    self.set_phase("PASS")
                elif self.odom.x_m - self.tail_x_m >= self.c.rear_clearance_m:
                    self.set_phase("RETURN")
                    self.make_path(self.odom.x_m + max(1.0, abs(self.odom.y_m) * 2), 0.0)
        if self.phase in ("ENTRY", "BASELINE", "PASS", "REAR_CLEARANCE", "RETURN"):
            if self.new_target() is not None:
                self.pause("new_obstacle_on_saved_path")

    def desired_command(self):
        base = min(self.status.base_rpm, self.c.maximum_rpm)
        if self.phase in ("STOPPING", "WAIT_DIRECTION", "BASELINE"):
            return 0, 0
        if self.phase == "DRIVE":
            error = line_heading_error_rad(pose_y_m=self.odom.y_m, target_y_m=0,
                                            pose_yaw_rad=self.odom.yaw_rad, lookahead_m=0.8)
            cap = base
        elif self.phase in ("ENTRY", "RETURN"):
            cap = min(base, self.c.entry_rpm if self.phase == "ENTRY" else self.c.return_rpm)
            error = self.follower.heading_error_rad(pose_x_m=self.odom.x_m, pose_y_m=self.odom.y_m,
                                                    pose_yaw_rad=self.odom.yaw_rad)
        else:
            cap = min(base, self.c.bypass_rpm)
            error = line_heading_error_rad(pose_y_m=self.odom.y_m, target_y_m=self.lane_y,
                                            pose_yaw_rad=self.odom.yaw_rad, lookahead_m=0.8)
        # Pure pursuit uses the measured wheel track, rather than a gain tuned
        # for the old 50cm chassis. Scale BOTH wheels to preserve curvature
        # when the user's speed cap is reached.
        lookahead = self.c.lookahead_m if self.phase in ("ENTRY", "RETURN") else 0.8
        delta = cap * self.c.wheel_base_m * math.sin(error) / lookahead
        delta = max(-0.95 * cap, min(0.95 * cap, delta))
        wheels = [cap - delta, cap + delta]
        scale = min(1.0, base / max(wheels))
        return tuple(round(v * scale) for v in wheels)

    def tick(self, now):
        dt = now - self.last_tick
        self.last_tick = now
        if self.status and self.status.left_counts is None and self.fresh(self.status_stamp, now, self.c.mcu_timeout_s):
            self.odom.update(self.status.left_rpm, self.status.right_rpm, dt)
        if self.state == "FAULT_STOP":
            return 0, 0
        if dt < 0 or dt > 0.25:
            self.fault("control_loop_gap_over_250ms")
            self.stop_now()
            return 0, 0
        errors = self.health_errors(now)
        if self.state == "STARTUP":
            blocker = self.blocking_reason(resume=True) if not errors else None
            if (not errors and blocker is None and self.status
                    and self.status.base_rpm == 0 and not self.status.stop_flags):
                self.preflight_ready = True
                self.change("READY", "fresh_feeds_wait_new_UP")
            elif now - self.created >= self.c.startup_timeout_s:
                reasons = list(errors)
                if blocker:
                    reasons.append(f"resume_blocker:{blocker}")
                if self.status and self.status.base_rpm != 0:
                    reasons.append("nonzero_start_speed")
                if self.status and self.status.stop_flags:
                    reasons.append("stop_flag_active")
                self.fault("startup_timeout:" + ",".join(reasons or ["readiness_not_established"]))
            return 0, 0
        if errors:
            reason = "sensor_health:" + ",".join(errors)
            # Camera/LiDAR processing can have a short scheduling/inference
            # gap. Stop immediately, but make that stop recoverable and
            # require a fresh UP after valid data returns. MCU/status/side
            # sensor loss remains a latched FAULT_STOP.
            recoverable = all(error.startswith(("camera_", "lidar_")) for error in errors)
            if not recoverable:
                self.fault(reason)
            elif self.state == "RUNNING":
                self.pause(reason, immediate=True)
            if self.state != "RUNNING" and self.pending_up:
                self.pending_up = False
                if self.status:
                    self.resume_up_floor = self.status.up_count
                self.emit("resume_denied", reason=reason)
            self.stop_now()
            return 0, 0
        blocker = self.blocking_reason(resume=self.state != "RUNNING")
        if self.state in ("READY", "PAUSED"):
            if self.pending_up:
                self.pending_up = False
                self.resume_up_floor = self.status.up_count
                extra = None
                if self.reason == "insufficient_entry_forward_room":
                    # No entry curve has been executed yet. If the object was
                    # removed, UP may return to straight driving.
                    if select_priority_target(self.objects, self.fusion) is None:
                        self.target = self.target_world = None
                        self.set_phase("DRIVE")
                    else:
                        extra = "insufficient_entry_forward_room"
                if self.phase != "DRIVE" and self.new_target() is not None:
                    extra = "new_obstacle_on_saved_path"
                if blocker or extra or self.status.stop_flags or self.status.base_rpm == 0:
                    self.emit("resume_denied", reason=blocker or extra or "user_stop_held_or_speed_zero")
                else:
                    self.clear_count = 0
                    self.last_side_stamp = self.sides.get(self.inside, SideReading("", now, "NO_ECHO")).stamp
                    self.progress_at = now
                    self.progress_distance = self.odom.distance_travelled_m
                    self.change("RUNNING", "new_UP_resume_saved_phase")
            if self.state != "RUNNING":
                return self.ramp_command((0, 0), dt)
        if blocker:
            self.pause(blocker)
            return self.ramp_command((0, 0), dt)
        self.active_s += max(0, dt)
        self.update_phase()
        if self.state != "RUNNING":
            return self.ramp_command((0, 0), dt)
        desired = self.desired_command()
        self.ramp_command(desired, dt)
        if max(self.command) >= 3:
            if self.motion_expected_since is None:
                self.motion_expected_since = now
                self.progress_at = now
                self.progress_distance = self.odom.distance_travelled_m
            if self.odom.distance_travelled_m - self.progress_distance >= 0.01:
                self.progress_at, self.progress_distance = now, self.odom.distance_travelled_m
            elif now - self.progress_at > self.c.no_progress_timeout_s:
                self.fault("wheel_feedback_no_progress")
                return 0, 0
        else:
            self.motion_expected_since = None
        return self.command

    def snapshot(self, now):
        corridor = [(d * math.cos(a), d * math.sin(a)) for a, d in self.points
                    if d * math.cos(a) > 0.01
                    and abs(d * math.sin(a)) <= self.c.robot_width_m / 2 + self.c.rear_safety_margin_m]
        nearest = min(corridor, default=None, key=lambda point: point[0])
        return {"state": self.state, "phase": self.phase, "reason": self.reason,
                "mcu_protocol": self.mcu_protocol,
                "odometry_source": "encoder_counts" if self.status and self.status.left_counts is not None else "reported_RPM_integral",
                "pose": self.pose(), "command": self.command, "ramp": self.ramped,
                "status": asdict(self.status) if self.status else None,
                "ages_s": {"camera": None if self.camera_stamp is None else now - self.camera_stamp,
                           "lidar": None if self.scan_stamp is None else now - self.scan_stamp,
                           "MCU_status": None if self.status_stamp is None else now - self.status_stamp},
                "sides": {k: asdict(v) for k, v in self.sides.items()}, "front_m": self.front,
                "front_bumper_clearance_m": self.front_clearance_m(),
                "health_errors": self.health_errors(now), "fault_reason": self.fault_reason,
                "paused_fault_reason": self.paused_fault_reason,
                "clearance_evidence": {"nearest_front_corridor_point_xy_m": nearest,
                                       "points_inside_configured_body": sum(
                                           abs(d * math.cos(a)) <= self.c.robot_length_m / 2
                                           and abs(d * math.sin(a)) <= self.c.robot_width_m / 2 for a, d in self.points),
                                       "left_lidar_m": self.lateral_clearance("left"),
                                       "right_lidar_m": self.lateral_clearance("right"),
                                       "resume_blocker": self.blocking_reason(resume=True)},
                "inside": self.inside, "baseline_m": self.baseline_value, "side_seen": self.side_seen,
                "baseline_samples": len(self.baseline), "near_samples": self.near_samples,
                "seen_at": self.seen_at, "pass_start": self.pass_start,
                "rear_progress_m": None if self.tail_at is None else self.odom.distance_travelled_m - self.tail_at,
                "rear_required_m": self.c.rear_clearance_m,
                "last_side_stamp": self.last_side_stamp,
                "pending_up": self.pending_up, "resume_up_floor": self.resume_up_floor,
                "motion_expected_since": self.motion_expected_since,
                "no_progress_age_s": now - self.progress_at,
                "frozen_target_world": self.target_world,
                "lane_y_m": self.lane_y, "turn_left": self.turn_left,
                "saved_path": asdict(self.follower.path) if self.follower else None,
                "clear_count": self.clear_count, "tail_at": self.tail_at, "tail_x_m": self.tail_x_m,
                "active_s": self.active_s, "follower_progress": self.follower.progress_ratio if self.follower else None,
                "target": asdict(self.target) if self.target else None,
                "objects": [asdict(v) for v in self.objects]}
