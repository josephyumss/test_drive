#!/usr/bin/env python3
"""One-shot avoidance with YOLO and LiDAR as the only obstacle sensors.

Wait for /yolo/detections, /scan and STM32 wheel/emergency telemetry, drive
forward, avoid a confirmed fused obstacle, recover the original path, then
drive forward for two seconds and exit with stopped motors. This script does
not import GPIO or read SHARP/ultrasonic values. Vehicle geometry and fusion
calibration match test_fused_path_avoidance.py.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from jetson.amr_core.reactive_avoidance import (
    BezierPathFollower,
    CubicBezierPath,
    DetectionTracker,
    FusedObject,
    FusionConfig,
    WheelOdometry,
    differential_rpm_command,
    fuse_confirmed_tracks,
    line_heading_error_rad,
    local_point_to_world,
    passing_lateral_offset_m,
    return_heading_error_rad,
    select_priority_target,
    wrap_angle,
)


# Existing hardware and communication settings.
PORT = "/dev/ttyTHS1"
BAUDRATE = 115200
LIDAR_SIDE_CLEAR_M = 0.8
LIDAR_SIDE_STOP_M = 0.35
LIDAR_FRONT_HALF_DEG = 15.0
LIDAR_SIDE_MIN_DEG = 20.0
LIDAR_SIDE_MAX_DEG = 75.0
LIDAR_HARD_STOP_M = 0.25


# Vehicle values: measure these on the real robot before ground operation.
WHEEL_DIAMETER_M = 0.20
WHEEL_BASE_M = 0.50
ROBOT_WIDTH_M = 0.50
ROBOT_LENGTH_M = 0.65
SAFETY_MARGIN_M = 0.15


# Camera/LiDAR calibration. Positive yaw offset means the camera center points
# left of LiDAR zero. Keep IMAGE_WIDTH equal to the Docker camera output width.
IMAGE_WIDTH = 640.0
CAMERA_HORIZONTAL_FOV_DEG = 69.0
CAMERA_LIDAR_YAW_OFFSET_DEG = 0.0
BBOX_ANGULAR_PADDING_DEG = 1.5


# Detection, fusion, and avoidance tuning.
MIN_CONFIDENCE = 0.45
REQUIRED_DETECTION_FRAMES = 3
TRACK_IOU_THRESHOLD = 0.30
TRACK_MAX_MISSED_FRAMES = 2

AVOIDANCE_TRIGGER_DISTANCE_M = 2.0
LIDAR_CLUSTER_DISTANCE_GAP_M = 0.30
LIDAR_CLUSTER_ANGLE_GAP_DEG = 2.0
LIDAR_MINIMUM_CLUSTER_POINTS = 2

CRUISE_RPM = 20
AVOIDANCE_FORWARD_RPM = 12
BYPASS_FORWARD_RPM = 16
RETURN_FORWARD_RPM = 14
MAXIMUM_MOTOR_RPM = 65

AVOIDANCE_STEERING_GAIN_RPM_PER_RAD = 18.0
PATH_STEERING_GAIN_RPM_PER_RAD = 16.0
MAXIMUM_AVOIDANCE_DELTA_RPM = 10.0
MAXIMUM_PATH_DELTA_RPM = 8.0

SENSOR_TIMEOUT_S = 1.0
STOP_BEFORE_TURN_S = 0.30
REQUIRED_CLEAR_SAMPLES = 5
MAXIMUM_ENTRY_PATH_S = 20.0

# A frozen S-curve reaches the passing lane before the obstacle's near face.
ENTRY_FRONT_CLEARANCE_M = ROBOT_LENGTH_M * 0.5 + SAFETY_MARGIN_M
MINIMUM_ENTRY_FORWARD_M = 0.65
ENTRY_PATH_LOOKAHEAD_M = 0.25
PATH_COMPLETION_RADIUS_M = 0.14

# Keep the existing obstacle-passing distance, then use another fixed S-curve
# to return to the original y=0 path.
BYPASS_DISTANCE_M = 0.70
RETURN_LOOKAHEAD_M = 0.80
RETURN_MINIMUM_FORWARD_M = 1.00
RETURN_FORWARD_PER_LATERAL = 2.0
TARGET_REIDENTIFICATION_RADIUS_M = 0.75
PATH_OFFSET_TOLERANCE_M = 0.08
HEADING_TOLERANCE_DEG = 6.0
AVOIDANCE_COOLDOWN_S = 0.5
POST_AVOIDANCE_FORWARD_S = 2.0
STARTUP_TIMEOUT_S = 30.0
MAXIMUM_RUNTIME_S = 180.0
NO_PROGRESS_TIMEOUT_S = 10.0
WAIT_CLEAR_TIMEOUT_S = 15.0
MAXIMUM_RETURN_PATH_S = 40.0
CONTROL_PERIOD_S = 0.02
OPEN_LOOP_ARM_S = 1.0


class YoloLidarAvoidance(Node):
    WAIT_SENSORS = "Waiting for sensors"
    POST_AVOIDANCE = "Final two seconds forward"
    COMPLETE = "Complete"
    ABORTED = "Aborted"
    DRIVE = "직진"
    STOPPING = "회피 전 정지"
    WAIT_CLEAR = "공간 대기"
    TURN_LEFT = "좌측 회피"
    TURN_RIGHT = "우측 회피"
    BYPASS = "장애물 통과"
    RETURN_PATH = "원경로 복귀"

    def __init__(
        self,
        *,
        port: str = PORT,
        status_port: str | None = None,
        baudrate: int = BAUDRATE,
        startup_timeout_s: float = STARTUP_TIMEOUT_S,
        max_runtime_s: float = MAXIMUM_RUNTIME_S,
        no_progress_timeout_s: float = NO_PROGRESS_TIMEOUT_S,
        preflight_only: bool = False,
        clear_startup_emergency: bool = False,
        open_loop: bool = False,
    ) -> None:
        if not all(math.isfinite(value) and value > 0 for value in
                   (startup_timeout_s, max_runtime_s, no_progress_timeout_s)):
            raise ValueError("Timeouts must be finite and positive")
        super().__init__("yolo_lidar_path_avoidance")
        self.serial = None
        self.status_serial = None
        self.rx_buffer = bytearray()
        self.startup_timeout_s = startup_timeout_s
        self.max_runtime_s = max_runtime_s
        self.no_progress_timeout_s = no_progress_timeout_s
        self.preflight_only = preflight_only
        self.clear_startup_emergency = clear_startup_emergency
        self.open_loop = open_loop
        self.created_s = time.monotonic()
        self.ready_started_s: float | None = None
        self.maneuver_started = False
        self.post_avoidance_started_s: float | None = None
        self.abort_reason: str | None = None
        self.last_progress_s = self.created_s
        self.last_progress_distance_m = 0.0
        self.last_command = (0, 0)
        self.closed = False

        self.fusion_config = FusionConfig(
            image_width_px=IMAGE_WIDTH,
            camera_horizontal_fov_deg=CAMERA_HORIZONTAL_FOV_DEG,
            camera_lidar_yaw_offset_deg=CAMERA_LIDAR_YAW_OFFSET_DEG,
            bbox_angular_padding_deg=BBOX_ANGULAR_PADDING_DEG,
            lidar_cluster_distance_gap_m=LIDAR_CLUSTER_DISTANCE_GAP_M,
            lidar_cluster_angle_gap_deg=LIDAR_CLUSTER_ANGLE_GAP_DEG,
            minimum_cluster_points=LIDAR_MINIMUM_CLUSTER_POINTS,
            robot_width_m=ROBOT_WIDTH_M,
            safety_margin_m=SAFETY_MARGIN_M,
            avoidance_trigger_distance_m=AVOIDANCE_TRIGGER_DISTANCE_M,
        )
        self.tracker = DetectionTracker(
            minimum_confidence=MIN_CONFIDENCE,
            required_frames=REQUIRED_DETECTION_FRAMES,
            iou_threshold=TRACK_IOU_THRESHOLD,
            maximum_missed_frames=TRACK_MAX_MISSED_FRAMES,
        )
        self.odometry = WheelOdometry(
            wheel_diameter_m=WHEEL_DIAMETER_M,
            wheel_base_m=WHEEL_BASE_M,
        )

        self.stamps: dict[str, float | None] = {
            "camera": None,
            "lidar": None,
        }
        if not self.open_loop:
            self.stamps["stm32"] = None
        self.detections: list[dict] = []
        self.labels: list[str] = []
        self.lidar_points: list[tuple[float, float]] = []
        self.fused_objects: list[FusedObject] = []
        self.priority_target: FusedObject | None = None

        self.left_wheel_rpm = 0.0
        self.right_wheel_rpm = 0.0
        self.stm32_emergency = False
        self.telemetry_seen = False
        self.lidar_front_m: float | None = None
        self.lidar_left_m: float | None = None
        self.lidar_right_m: float | None = None

        self.state = self.WAIT_SENSORS
        self.state_started_s = time.monotonic()
        self.cooldown_until_s = 0.0
        self.clear_count = 0
        self.target_track_id: int | None = None
        self.pending_target_track_id: int | None = None
        self.planned_obstacle_x_m: float | None = None
        self.planned_obstacle_y_m: float | None = None
        self.entry_follower: BezierPathFollower | None = None
        self.return_follower: BezierPathFollower | None = None
        self.bypass_target_y_m = 0.0
        self.paused_state: str | None = None
        self.bypass_start_distance_m = 0.0
        self.last_odometry_s = time.monotonic()
        self.last_report: tuple | None = None

        self.create_subscription(String, "/yolo/detections", self.on_yolo, 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

        # Open the actuator link last, after all ROS subscriptions exist. A
        # write timeout bounds shutdown even if the UART stops responding.
        self.serial = serial.Serial(port, baudrate, timeout=0.01,
                                    write_timeout=0.2, exclusive=True)
        try:
            self.serial.reset_input_buffer()
            if status_port is not None and status_port != port:
                self.status_serial = serial.Serial(
                    status_port,
                    baudrate,
                    timeout=0.01,
                    write_timeout=0.2,
                    exclusive=True,
                )
                self.status_serial.reset_input_buffer()
            else:
                self.status_serial = self.serial
        except Exception:
            if self.status_serial is not None and self.status_serial is not self.serial:
                self.status_serial.close()
            self.serial.close()
            raise

    @property
    def finished(self) -> bool:
        return self.state in (self.COMPLETE, self.ABORTED)

    def abort(self, reason: str, now: float) -> None:
        if self.finished:
            return
        self.abort_reason = reason
        self.transition(self.ABORTED, now)
        print(f"ABORT: {reason}", flush=True)

    def on_yolo(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, list):
                raise ValueError
            for item in payload:
                if not isinstance(item, dict):
                    raise ValueError("Invalid detection entry")
                confidence = float(item.get("confidence", 0.0))
                box = item.get("xyxy", [])
                if (not math.isfinite(confidence) or not 0 <= confidence <= 1
                        or not isinstance(box, (list, tuple)) or len(box) != 4
                        or not all(math.isfinite(float(value)) for value in box)
                        or float(box[2]) <= float(box[0])
                        or float(box[3]) <= float(box[1])):
                    raise ValueError("Invalid detection geometry/confidence")
            self.detections = payload
            self.tracker.update(self.detections)
            self.labels = [
                str(item.get("class", "object"))
                for item in self.detections
                if float(item.get("confidence", 0.0)) >= MIN_CONFIDENCE
            ]
            self.stamps["camera"] = time.monotonic()
        except (json.JSONDecodeError, TypeError, ValueError):
            self.detections = []
            self.labels = []
            self.tracker.update([])
            self.stamps["camera"] = None

    @staticmethod
    def sector_minimum(
        message: LaserScan, low_deg: float, high_deg: float
    ) -> float | None:
        """Return clearance only for sectors with valid samples.

        LaserScan +inf means no return within range_max. NaN, zero, negative,
        below-minimum samples and absent scan coverage never mean clear.
        Normalize bearings so scanners using 0..2pi cover the right side too.
        """
        low, high = math.radians(low_deg), math.radians(high_deg)
        values = []
        for index, distance in enumerate(message.ranges):
            angle = wrap_angle(message.angle_min + index * message.angle_increment)
            if not low <= angle <= high:
                continue
            if (math.isfinite(distance) and distance > 0
                    and message.range_min <= distance <= message.range_max):
                values.append(float(distance))
            elif distance == math.inf and math.isfinite(message.range_max):
                values.append(float(message.range_max))
        return min(values) if values else None

    def on_scan(self, message: LaserScan) -> None:
        self.stamps["lidar"] = None
        self.lidar_points = []
        self.lidar_front_m = self.lidar_left_m = self.lidar_right_m = None
        if not (math.isfinite(message.angle_min)
                and math.isfinite(message.angle_increment)
                and message.angle_increment != 0
                and math.isfinite(message.range_min)
                and math.isfinite(message.range_max)
                and 0 <= message.range_min < message.range_max):
            return
        self.lidar_front_m = self.sector_minimum(
            message, -LIDAR_FRONT_HALF_DEG, LIDAR_FRONT_HALF_DEG
        )
        self.lidar_left_m = self.sector_minimum(
            message, LIDAR_SIDE_MIN_DEG, LIDAR_SIDE_MAX_DEG
        )
        self.lidar_right_m = self.sector_minimum(
            message, -LIDAR_SIDE_MAX_DEG, -LIDAR_SIDE_MIN_DEG
        )
        for index, distance in enumerate(message.ranges):
            if (math.isfinite(distance) and distance > 0
                    and message.range_min <= distance <= message.range_max):
                angle = wrap_angle(message.angle_min + index * message.angle_increment)
                self.lidar_points.append((float(angle), float(distance)))
        self.lidar_points.sort()
        if all(value is not None for value in (
                self.lidar_front_m, self.lidar_left_m, self.lidar_right_m)):
            self.stamps["lidar"] = time.monotonic()

    def read_stm32(self) -> None:
        waiting = min(self.status_serial.in_waiting, 4096)
        if waiting:
            self.rx_buffer.extend(self.status_serial.read(waiting))
        if len(self.rx_buffer) > 8192:
            self.rx_buffer.clear()
            if not self.open_loop:
                self.stamps["stm32"] = None
            return
        while b"\n" in self.rx_buffer:
            raw, _, remainder = self.rx_buffer.partition(b"\n")
            self.rx_buffer = bytearray(remainder)
            fields = raw.rstrip(b"\r").decode("ascii", errors="replace").split(",")
            if not fields or fields[0] != "$STATUS":
                continue
            # Keep the existing 11-column STM32 protocol, but deliberately
            # ignore ADC/SHARP columns 2 and 3. Missing wheel feedback or ESTOP
            # cannot be substituted with a ready/zero default.
            try:
                if len(fields) != 11:
                    raise ValueError("Incomplete telemetry")
                left_rpm, right_rpm = float(fields[6]), float(fields[7])
                emergency = int(fields[10])
                if (not all(math.isfinite(value) and abs(value) <= 1000
                            for value in (left_rpm, right_rpm))
                        or emergency not in (0, 1)):
                    raise ValueError("Invalid telemetry")
            except ValueError:
                if not self.open_loop:
                    self.stamps["stm32"] = None
                continue
            self.left_wheel_rpm = left_rpm
            self.right_wheel_rpm = right_rpm
            # The firmware command watchdog latches emergency after 500 ms
            # without commands. The launcher explicitly permits clearing that
            # state with a zero-RPM command during startup only. Once driving
            # starts, any emergency remains latched for this invocation.
            if self.state == self.WAIT_SENSORS and (
                self.clear_startup_emergency or self.open_loop
            ):
                self.stm32_emergency = bool(emergency)
            else:
                self.stm32_emergency = self.stm32_emergency or bool(emergency)
            self.telemetry_seen = True
            if not self.open_loop:
                self.stamps["stm32"] = time.monotonic()

    def fresh(self, name: str, now: float) -> bool:
        stamp = self.stamps[name]
        return stamp is not None and 0.0 <= now - stamp <= SENSOR_TIMEOUT_S

    def all_sensors_fresh(self, now: float) -> bool:
        return all(self.fresh(name, now) for name in self.stamps)

    def update_odometry(self, now: float) -> None:
        dt = now - self.last_odometry_s
        self.last_odometry_s = now
        if self.open_loop:
            self.odometry.update(*self.last_command, dt)
        elif self.fresh("stm32", now):
            self.odometry.update(self.left_wheel_rpm, self.right_wheel_rpm, dt)

    def update_fused_objects(self) -> None:
        self.fused_objects = fuse_confirmed_tracks(
            self.tracker.confirmed_visible(), self.lidar_points, self.fusion_config
        )
        self.priority_target = select_priority_target(
            self.fused_objects, self.fusion_config
        )

    def left_clear(self) -> bool:
        return self.lidar_left_m is not None and self.lidar_left_m >= LIDAR_SIDE_CLEAR_M

    def right_clear(self) -> bool:
        return self.lidar_right_m is not None and self.lidar_right_m >= LIDAR_SIDE_CLEAR_M

    def side_blocked(self, state: str) -> bool:
        distance = self.lidar_left_m if state == self.TURN_LEFT else self.lidar_right_m
        return distance is None or distance <= LIDAR_SIDE_STOP_M

    def choose_turn(self, target: FusedObject) -> str | None:
        left, right = self.left_clear(), self.right_clear()
        preferred = self.TURN_RIGHT if target.bearing_rad > 0.0 else self.TURN_LEFT
        if abs(target.bearing_rad) < math.radians(3.0):
            preferred = (self.TURN_LEFT
                         if float(self.lidar_left_m or 0) >= float(self.lidar_right_m or 0)
                         else self.TURN_RIGHT)
        if preferred == self.TURN_LEFT and left:
            return self.TURN_LEFT
        if preferred == self.TURN_RIGHT and right:
            return self.TURN_RIGHT
        if left:
            return self.TURN_LEFT
        if right:
            return self.TURN_RIGHT
        return None

    def target_by_id(self, track_id: int | None) -> FusedObject | None:
        if track_id is None:
            return None
        return next(
            (item for item in self.fused_objects if item.track_id == track_id), None
        )

    def object_world_position(self, target: FusedObject) -> tuple[float, float]:
        return local_point_to_world(
            origin_x_m=self.odometry.x_m,
            origin_y_m=self.odometry.y_m,
            origin_yaw_rad=self.odometry.yaw_rad,
            forward_m=target.distance_m * math.cos(target.bearing_rad),
            lateral_m=target.distance_m * math.sin(target.bearing_rad),
        )

    def is_planned_obstacle(self, target: FusedObject) -> bool:
        if self.planned_obstacle_x_m is None or self.planned_obstacle_y_m is None:
            return target.track_id == self.target_track_id
        object_x, object_y = self.object_world_position(target)
        return bool(
            math.hypot(
                object_x - self.planned_obstacle_x_m,
                object_y - self.planned_obstacle_y_m,
            )
            <= TARGET_REIDENTIFICATION_RADIUS_M
        )

    def unplanned_priority_target(self) -> FusedObject | None:
        candidates = [
            item
            for item in self.fused_objects
            if item.intersects_corridor
            and item.forward_distance_m <= AVOIDANCE_TRIGGER_DISTANCE_M
            and not self.is_planned_obstacle(item)
        ]
        return min(candidates, key=lambda item: item.distance_m, default=None)

    def make_follower(self, path: CubicBezierPath) -> BezierPathFollower:
        return BezierPathFollower(
            path,
            lookahead_m=ENTRY_PATH_LOOKAHEAD_M,
            completion_radius_m=PATH_COMPLETION_RADIUS_M,
        )

    def start_avoidance(self, target: FusedObject, state: str, now: float) -> None:
        self.maneuver_started = True
        self.target_track_id = target.track_id
        self.pending_target_track_id = None
        self.planned_obstacle_x_m, self.planned_obstacle_y_m = (
            self.object_world_position(target)
        )

        lateral_offset = passing_lateral_offset_m(
            target,
            turn_left=state == self.TURN_LEFT,
            corridor_half_width_m=self.fusion_config.corridor_half_width_m,
        )
        entry_forward = max(
            MINIMUM_ENTRY_FORWARD_M,
            target.forward_distance_m - ENTRY_FRONT_CLEARANCE_M,
        )
        goal_x, goal_y = local_point_to_world(
            origin_x_m=self.odometry.x_m,
            origin_y_m=self.odometry.y_m,
            origin_yaw_rad=self.odometry.yaw_rad,
            forward_m=entry_forward,
            lateral_m=lateral_offset,
        )
        path = CubicBezierPath.from_poses(
            start_x_m=self.odometry.x_m,
            start_y_m=self.odometry.y_m,
            start_yaw_rad=self.odometry.yaw_rad,
            goal_x_m=goal_x,
            goal_y_m=goal_y,
            goal_yaw_rad=0.0,
        )
        self.entry_follower = self.make_follower(path)
        self.return_follower = None
        self.bypass_target_y_m = goal_y
        self.paused_state = None
        self.transition(state, now)

    def start_return_path(self, now: float) -> None:
        forward = max(
            RETURN_MINIMUM_FORWARD_M,
            abs(self.odometry.y_m) * RETURN_FORWARD_PER_LATERAL,
        )
        path = CubicBezierPath.from_poses(
            start_x_m=self.odometry.x_m,
            start_y_m=self.odometry.y_m,
            start_yaw_rad=self.odometry.yaw_rad,
            goal_x_m=self.odometry.x_m + forward,
            goal_y_m=0.0,
            goal_yaw_rad=0.0,
        )
        self.return_follower = self.make_follower(path)
        self.transition(self.RETURN_PATH, now)

    def finish_avoidance(self, now: float) -> None:
        self.cooldown_until_s = now + AVOIDANCE_COOLDOWN_S
        self.target_track_id = None
        self.pending_target_track_id = None
        self.planned_obstacle_x_m = None
        self.planned_obstacle_y_m = None
        self.entry_follower = None
        self.return_follower = None
        self.paused_state = None
        if self.maneuver_started:
            self.post_avoidance_started_s = now
            self.transition(self.POST_AVOIDANCE, now)
        else:
            # Losing a detection before a path was executed is not avoidance.
            self.transition(self.DRIVE, now)

    def path_recovered(self) -> bool:
        return bool(
            abs(self.odometry.y_m) <= PATH_OFFSET_TOLERANCE_M
            and abs(wrap_angle(self.odometry.yaw_rad))
            <= math.radians(HEADING_TOLERANCE_DEG)
        )

    def transition(self, state: str, now: float) -> None:
        self.state = state
        self.state_started_s = now
        self.clear_count = 0
        print(f"\nSTATE: {state}", flush=True)

    def update_state(self, now: float) -> None:
        if self.finished or self.state == self.WAIT_SENSORS:
            return
        if self.state == self.POST_AVOIDANCE:
            if self.unplanned_priority_target() is not None:
                self.abort("New obstacle during final forward segment", now)
            elif (self.post_avoidance_started_s is not None
                  and now - self.post_avoidance_started_s >= POST_AVOIDANCE_FORWARD_S):
                self.transition(self.COMPLETE, now)
            return
        if self.state == self.WAIT_CLEAR and now - self.state_started_s >= WAIT_CLEAR_TIMEOUT_S:
            self.abort("No safe LiDAR clearance within timeout", now)
            return
        if self.state == self.RETURN_PATH and now - self.state_started_s >= MAXIMUM_RETURN_PATH_S:
            self.abort("Return path exceeded timeout", now)
            return
        if self.state == self.DRIVE:
            if (
                self.priority_target is not None
                and now >= self.cooldown_until_s
            ):
                self.pending_target_track_id = self.priority_target.track_id
                self.transition(self.STOPPING, now)
            return

        # While passing or returning, ignore the frozen obstacle even if YOLO
        # gives it a new track id nearby. A spatially different obstacle still
        # preempts the path and starts a new stop-plan-follow cycle.
        if self.state in (self.BYPASS, self.RETURN_PATH):
            new_target = self.unplanned_priority_target()
            if new_target is not None:
                self.pending_target_track_id = new_target.track_id
                self.entry_follower = None
                self.return_follower = None
                self.paused_state = None
                self.transition(self.STOPPING, now)
                return

        target = self.target_by_id(self.pending_target_track_id)

        if self.state == self.STOPPING:
            if target is not None:
                self.clear_count = 0
            else:
                self.clear_count += 1
                if self.clear_count >= REQUIRED_CLEAR_SAMPLES:
                    self.pending_target_track_id = None
                    if self.path_recovered():
                        self.finish_avoidance(now)
                    else:
                        self.start_return_path(now)
                    return
            if target is not None and now - self.state_started_s >= STOP_BEFORE_TURN_S:
                turn = self.choose_turn(target)
                if turn is None:
                    self.transition(self.WAIT_CLEAR, now)
                else:
                    self.start_avoidance(target, turn, now)
            return

        if self.state == self.WAIT_CLEAR:
            if self.paused_state is not None:
                resume_state = self.paused_state
                if not self.side_blocked(resume_state):
                    self.paused_state = None
                    self.transition(resume_state, now)
                return
            if target is None:
                self.clear_count += 1
                if self.clear_count >= REQUIRED_CLEAR_SAMPLES:
                    self.pending_target_track_id = None
                    if self.path_recovered():
                        self.finish_avoidance(now)
                    else:
                        self.start_return_path(now)
                return
            self.clear_count = 0
            turn = self.choose_turn(target)
            if turn is not None:
                self.start_avoidance(target, turn, now)
            return

        if self.state in (self.TURN_LEFT, self.TURN_RIGHT):
            if self.side_blocked(self.state):
                self.paused_state = self.state
                self.transition(self.WAIT_CLEAR, now)
                return
            if self.entry_follower is None:
                self.abort("Entry path missing", now)
                return
            if self.entry_follower.completed(
                pose_x_m=self.odometry.x_m,
                pose_y_m=self.odometry.y_m,
                pose_yaw_rad=self.odometry.yaw_rad,
            ):
                self.bypass_start_distance_m = self.odometry.distance_travelled_m
                self.transition(self.BYPASS, now)
                return
            if now - self.state_started_s >= MAXIMUM_ENTRY_PATH_S:
                self.abort("Entry path exceeded timeout", now)
            return

        if self.state == self.BYPASS:
            travelled = self.odometry.distance_travelled_m - self.bypass_start_distance_m
            if travelled >= BYPASS_DISTANCE_M:
                self.start_return_path(now)
            return

        if self.state == self.RETURN_PATH:
            if self.return_follower is None:
                self.abort("Return path missing", now)
                return
            if self.return_follower.completed(
                pose_x_m=self.odometry.x_m,
                pose_y_m=self.odometry.y_m,
                pose_yaw_rad=self.odometry.yaw_rad,
            ):
                self.finish_avoidance(now)

    def motor_command(self, now: float) -> tuple[int, int]:
        if not self.all_sensors_fresh(now):
            return 0, 0
        if self.stm32_emergency:
            return 0, 0
        if self.lidar_front_m is None or self.lidar_front_m <= LIDAR_HARD_STOP_M:
            return 0, 0
        if self.state == self.POST_AVOIDANCE:
            if (self.post_avoidance_started_s is None
                    or now - self.post_avoidance_started_s >= POST_AVOIDANCE_FORWARD_S):
                return 0, 0
            return CRUISE_RPM, CRUISE_RPM
        if self.state == self.DRIVE:
            return CRUISE_RPM, CRUISE_RPM
        if self.state in (self.STOPPING, self.WAIT_CLEAR):
            return 0, 0
        if self.state in (self.TURN_LEFT, self.TURN_RIGHT):
            if self.side_blocked(self.state):
                return 0, 0
            if self.entry_follower is None:
                return 0, 0
            heading_error = self.entry_follower.heading_error_rad(
                pose_x_m=self.odometry.x_m,
                pose_y_m=self.odometry.y_m,
                pose_yaw_rad=self.odometry.yaw_rad,
            )
            return differential_rpm_command(
                AVOIDANCE_FORWARD_RPM,
                heading_error,
                gain_rpm_per_rad=AVOIDANCE_STEERING_GAIN_RPM_PER_RAD,
                maximum_delta_rpm=MAXIMUM_AVOIDANCE_DELTA_RPM,
                maximum_rpm=MAXIMUM_MOTOR_RPM,
            )
        if self.state == self.BYPASS:
            heading_error = line_heading_error_rad(
                pose_y_m=self.odometry.y_m,
                target_y_m=self.bypass_target_y_m,
                pose_yaw_rad=self.odometry.yaw_rad,
                lookahead_m=RETURN_LOOKAHEAD_M,
            )
            return differential_rpm_command(
                BYPASS_FORWARD_RPM,
                heading_error,
                gain_rpm_per_rad=PATH_STEERING_GAIN_RPM_PER_RAD,
                maximum_delta_rpm=MAXIMUM_PATH_DELTA_RPM,
                maximum_rpm=MAXIMUM_MOTOR_RPM,
            )
        if self.state == self.RETURN_PATH:
            if self.return_follower is None:
                heading_error = return_heading_error_rad(
                    pose_y_m=self.odometry.y_m,
                    pose_yaw_rad=self.odometry.yaw_rad,
                    lookahead_m=RETURN_LOOKAHEAD_M,
                )
            else:
                heading_error = self.return_follower.heading_error_rad(
                    pose_x_m=self.odometry.x_m,
                    pose_y_m=self.odometry.y_m,
                    pose_yaw_rad=self.odometry.yaw_rad,
                )
            return differential_rpm_command(
                RETURN_FORWARD_RPM,
                heading_error,
                gain_rpm_per_rad=PATH_STEERING_GAIN_RPM_PER_RAD,
                maximum_delta_rpm=MAXIMUM_PATH_DELTA_RPM,
                maximum_rpm=MAXIMUM_MOTOR_RPM,
            )
        return 0, 0

    @staticmethod
    def fmt(value: float | int | None, precision: int = 1) -> str:
        return "None" if value is None else f"{value:.{precision}f}"

    def check_health(self, now: float) -> None:
        if self.finished:
            return
        if self.stm32_emergency and not (
            self.state == self.WAIT_SENSORS
            and (self.clear_startup_emergency or self.open_loop)
        ):
            self.abort("STM32 emergency input is active", now)
            return
        if self.state == self.WAIT_SENSORS:
            if self.all_sensors_fresh(now):
                if self.open_loop and now - self.created_s < OPEN_LOOP_ARM_S:
                    # Keep sending zero commands long enough to clear the
                    # STM32 watchdog before allowing any motion command.
                    return
                if self.stm32_emergency and self.clear_startup_emergency:
                    # tick() will send $CMD,0,0,0; wait for a following
                    # telemetry frame to confirm that the watchdog cleared.
                    return
                self.ready_started_s = now
                self.last_odometry_s = now
                self.last_progress_s = now
                self.last_progress_distance_m = self.odometry.distance_travelled_m
                if self.preflight_only:
                    self.transition(self.COMPLETE, now)
                else:
                    self.transition(self.DRIVE, now)
            elif now - self.created_s >= self.startup_timeout_s:
                missing = [name for name in self.stamps if not self.fresh(name, now)]
                self.abort("Startup timeout: " + ", ".join(missing), now)
            return
        if not self.all_sensors_fresh(now):
            missing = [name for name in self.stamps if not self.fresh(name, now)]
            self.abort("Sensor data lost/invalid: " + ", ".join(missing), now)
            return
        if self.lidar_front_m is None or self.lidar_front_m <= LIDAR_HARD_STOP_M:
            self.abort("LiDAR front hard stop", now)
            return
        if self.ready_started_s is not None and now - self.ready_started_s >= self.max_runtime_s:
            self.abort("Maximum run duration exceeded", now)
            return
        distance = self.odometry.distance_travelled_m
        if (self.last_command == (0, 0)
                or distance - self.last_progress_distance_m >= 0.02):
            self.last_progress_s = now
            self.last_progress_distance_m = distance
        elif now - self.last_progress_s >= self.no_progress_timeout_s:
            self.abort("No wheel-feedback progress while commanding motion", now)

    def send_command(self, left: int, right: int) -> None:
        # E=0 clears the firmware's emergency latch. Do not send it before
        # reading a valid emergency field. Its existing command watchdog
        # stops old commands while startup waits for telemetry.
        if not self.telemetry_seen and not self.open_loop:
            if not self.finished and not self.closed:
                return
            left, right = 0, 0
            self.stm32_emergency = True
        if self.state == self.WAIT_SENSORS and (
            self.clear_startup_emergency or self.open_loop
        ):
            # Clear only the firmware watchdog latch, while commanding zero.
            left, right, emergency = 0, 0, 0
        else:
            emergency = int(self.stm32_emergency)
        self.serial.write(f"$CMD,{left},{right},{emergency}\r\n".encode("ascii"))
        self.last_command = (left, right)

    def tick(self) -> None:
        if self.finished:
            return
        self.read_stm32()
        # Sample after UART I/O: callback/telemetry timestamps must not be in
        # the future relative to the health check.
        now = time.monotonic()
        self.update_odometry(now)
        self.check_health(now)
        if not self.finished and self.state != self.WAIT_SENSORS:
            self.update_fused_objects()
            self.update_state(now)
        left, right = self.motor_command(now)
        self.send_command(left, right)
        report = (
            self.state, left, right,
            round(self.lidar_front_m or -1, 2),
            round(self.lidar_left_m or -1, 2),
            round(self.lidar_right_m or -1, 2),
            tuple(self.labels),
            round(self.odometry.y_m, 2),
            round(math.degrees(self.odometry.yaw_rad), 1),
            tuple(name for name in self.stamps if not self.fresh(name, now)),
        )
        if report != self.last_report:
            print(
                f"STATE={self.state} MOTOR=({left},{right}) "
                f"LIDAR(F/L/R)=({self.fmt(self.lidar_front_m, 2)}/"
                f"{self.fmt(self.lidar_left_m, 2)}/{self.fmt(self.lidar_right_m, 2)})m "
                f"YOLO={','.join(self.labels) if self.labels else 'none'} "
                f"POSE=(x={self.odometry.x_m:.2f}, y={self.odometry.y_m:.2f}, "
                f"yaw={math.degrees(self.odometry.yaw_rad):.1f}deg) "
                f"WAITING={','.join(report[-1]) or 'none'}",
                flush=True,
            )
            self.last_report = report

    def shutdown(self) -> None:
        if self.closed or self.serial is None:
            return
        self.closed = True
        try:
            for _ in range(3):
                try:
                    self.send_command(0, 0)
                except Exception as error:
                    print(f"Stop command failed: {error}", flush=True)
                    break
                time.sleep(0.02)
        finally:
            self.serial.close()
            if self.status_serial is not self.serial:
                self.status_serial.close()


# Keep the familiar import name for tooling using the original test node.
IntegratedAvoidance = YoloLidarAvoidance


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=PORT)
    parser.add_argument(
        "--status-port",
        default=None,
        help="Optional separate UART receiving STM32 $STATUS telemetry",
    )
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--startup-timeout", type=float, default=STARTUP_TIMEOUT_S)
    parser.add_argument("--max-runtime", type=float, default=MAXIMUM_RUNTIME_S)
    parser.add_argument("--no-progress-timeout", type=float, default=NO_PROGRESS_TIMEOUT_S)
    parser.add_argument("--preflight-only", action="store_true",
                        help="Check fresh sensors/telemetry while sending only zero motor commands")
    parser.add_argument(
        "--clear-startup-emergency",
        action="store_true",
        help="Clear the STM32 command-watchdog latch using zero RPM during startup",
    )
    parser.add_argument(
        "--open-loop",
        action="store_true",
        help="Use commanded RPM for odometry when STM32 status wiring is unavailable",
    )
    args, ros_args = parser.parse_known_args(arguments)
    for name in ("startup_timeout", "max_runtime", "no_progress_timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.baudrate <= 0:
        parser.error("--baudrate must be positive")
    return args, ros_args


def main(arguments=None) -> int:
    args, ros_args = parse_arguments(arguments)
    rclpy.init(args=ros_args)
    node = None
    interrupted = False

    def stop_signal(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    previous_sigint = signal.signal(signal.SIGINT, stop_signal)
    previous_sigterm = signal.signal(signal.SIGTERM, stop_signal)
    try:
        node = YoloLidarAvoidance(
            port=args.port, status_port=args.status_port, baudrate=args.baudrate,
            startup_timeout_s=args.startup_timeout,
            max_runtime_s=args.max_runtime,
            no_progress_timeout_s=args.no_progress_timeout,
            preflight_only=args.preflight_only,
            clear_startup_emergency=args.clear_startup_emergency,
            open_loop=args.open_loop,
        )
        print("YOLO + LiDAR one-shot avoidance; stop with Ctrl+C.", flush=True)
        while rclpy.ok() and not interrupted and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.01)
            if interrupted or not rclpy.ok():
                break
            node.tick()
            if node.state == node.POST_AVOIDANCE and node.post_avoidance_started_s is not None:
                remaining = (node.post_avoidance_started_s + POST_AVOIDANCE_FORWARD_S
                             - time.monotonic())
                time.sleep(max(0.0, min(CONTROL_PERIOD_S, remaining)))
            else:
                time.sleep(CONTROL_PERIOD_S)
        if not node.finished:
            node.abort("Interrupted or ROS shutdown", time.monotonic())
        return 0 if node.state == node.COMPLETE else 1
    except Exception as error:
        print(f"Controller failed: {type(error).__name__}: {error}", flush=True)
        if node is not None:
            node.abort(str(error), time.monotonic())
        return 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        print("Controller exited; motor stop commands sent where UART was available.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
