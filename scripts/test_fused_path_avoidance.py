#!/usr/bin/env python3
"""User-version YOLO/LiDAR fused avoidance using existing AMR hardware links.

Existing interfaces are intentionally unchanged:

* YOLO detections: ``/yolo/detections`` JSON ``xyxy`` boxes
* LiDAR: ``/scan``
* STM32: ``$STATUS,...`` input and ``$CMD,L,R,E`` output on ``/dev/ttyTHS1``
* Side clearance: the existing dual HC-SR04 Jetson GPIO wiring

SHARP remains an independent last-resort near-field brake. Normal avoidance
starts only for a repeatedly observed YOLO object whose associated LiDAR range
overlaps the robot's forward swept corridor.
"""

from __future__ import annotations

import json
import math
import time

import Jetson.GPIO as GPIO
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
LEFT_TRIG, LEFT_ECHO = 33, 31
RIGHT_TRIG, RIGHT_ECHO = 7, 15

SHARP_MIN_CM, SHARP_MAX_CM = 10, 80
SHARP_EMERGENCY_CM = 20

SIDE_CLEAR_CM, SIDE_STOP_CM = 35, 15
ULTRASONIC_MAX_CM = 300
ULTRASONIC_INTERVAL_S = 0.06
ULTRASONIC_ECHO_TIMEOUT_S = 0.03

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
SOUND_SPEED_CM_S = 34300.0


class IntegratedAvoidance(Node):
    DRIVE = "직진"
    STOPPING = "회피 전 정지"
    WAIT_CLEAR = "공간 대기"
    TURN_LEFT = "좌측 회피"
    TURN_RIGHT = "우측 회피"
    BYPASS = "장애물 통과"
    RETURN_PATH = "원경로 복귀"

    def __init__(self) -> None:
        super().__init__("integrated_avoidance_test")
        self.serial = serial.Serial(PORT, BAUDRATE, timeout=0.01)
        self.serial.reset_input_buffer()
        self.rx_buffer = bytearray()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LEFT_TRIG, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(LEFT_ECHO, GPIO.IN)
        GPIO.setup(RIGHT_TRIG, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(RIGHT_ECHO, GPIO.IN)
        time.sleep(0.1)

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
            "stm32": None,
            "lidar": None,
            "left_us": None,
            "right_us": None,
        }
        self.detections: list[dict] = []
        self.labels: list[str] = []
        self.lidar_points: list[tuple[float, float]] = []
        self.fused_objects: list[FusedObject] = []
        self.priority_target: FusedObject | None = None

        self.sharp_cm: int | None = None
        self.sharp_adc: int | None = None
        self.left_wheel_rpm = 0.0
        self.right_wheel_rpm = 0.0
        self.stm32_emergency = False
        self.left_us_cm: float | None = None
        self.right_us_cm: float | None = None
        self.lidar_front_m: float | None = None
        self.lidar_left_m: float | None = None
        self.lidar_right_m: float | None = None

        self.state = self.DRIVE
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
        self.next_ultrasonic_s = time.monotonic()
        self.measure_left_next = True
        self.last_report: tuple | None = None

        self.create_subscription(String, "/yolo/detections", self.on_yolo, 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

    def on_yolo(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, list):
                raise ValueError
            self.detections = [item for item in payload if isinstance(item, dict)]
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
        low, high = math.radians(low_deg), math.radians(high_deg)
        values = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if (
                low <= angle <= high
                and math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                values.append(float(distance))
        if values:
            return min(values)
        return float(message.range_max) if math.isfinite(message.range_max) else None

    def on_scan(self, message: LaserScan) -> None:
        self.lidar_front_m = self.sector_minimum(
            message, -LIDAR_FRONT_HALF_DEG, LIDAR_FRONT_HALF_DEG
        )
        self.lidar_left_m = self.sector_minimum(
            message, LIDAR_SIDE_MIN_DEG, LIDAR_SIDE_MAX_DEG
        )
        self.lidar_right_m = self.sector_minimum(
            message, -LIDAR_SIDE_MAX_DEG, -LIDAR_SIDE_MIN_DEG
        )
        self.lidar_points = []
        for index, distance in enumerate(message.ranges):
            if (
                math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                angle = message.angle_min + index * message.angle_increment
                self.lidar_points.append((float(angle), float(distance)))
        if all(
            value is not None
            for value in (
                self.lidar_front_m,
                self.lidar_left_m,
                self.lidar_right_m,
            )
        ):
            self.stamps["lidar"] = time.monotonic()

    def read_stm32(self) -> None:
        waiting = self.serial.in_waiting
        if waiting:
            self.rx_buffer.extend(self.serial.read(waiting))
        while b"\n" in self.rx_buffer:
            raw, _, remainder = self.rx_buffer.partition(b"\n")
            self.rx_buffer = bytearray(remainder)
            fields = raw.rstrip(b"\r").decode("ascii", errors="replace").split(",")
            if len(fields) < 4 or fields[0] != "$STATUS":
                continue
            try:
                adc, distance = int(fields[2]), int(fields[3])
                left_rpm = int(fields[6]) if len(fields) >= 8 else 0
                right_rpm = int(fields[7]) if len(fields) >= 8 else 0
                emergency = bool(int(fields[10])) if len(fields) >= 11 else False
            except ValueError:
                continue
            if SHARP_MIN_CM <= distance <= SHARP_MAX_CM:
                self.sharp_adc = adc
                self.sharp_cm = distance
                self.left_wheel_rpm = float(left_rpm)
                self.right_wheel_rpm = float(right_rpm)
                self.stm32_emergency = emergency
                self.stamps["stm32"] = time.monotonic()

    @staticmethod
    def measure_ultrasonic(trigger: int, echo: int) -> float | None:
        GPIO.output(trigger, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(trigger, GPIO.HIGH)
        time.sleep(0.000010)
        GPIO.output(trigger, GPIO.LOW)
        started = time.monotonic()
        while GPIO.input(echo) == GPIO.LOW:
            if time.monotonic() - started > ULTRASONIC_ECHO_TIMEOUT_S:
                return None
        pulse_started = time.monotonic()
        while GPIO.input(echo) == GPIO.HIGH:
            if time.monotonic() - pulse_started > ULTRASONIC_ECHO_TIMEOUT_S:
                return None
        distance = (time.monotonic() - pulse_started) * SOUND_SPEED_CM_S / 2.0
        return distance if 2.0 <= distance <= ULTRASONIC_MAX_CM else None

    def update_ultrasonic(self) -> None:
        now = time.monotonic()
        if now < self.next_ultrasonic_s:
            return
        if self.measure_left_next:
            self.left_us_cm = self.measure_ultrasonic(LEFT_TRIG, LEFT_ECHO)
            self.stamps["left_us"] = now if self.left_us_cm is not None else None
        else:
            self.right_us_cm = self.measure_ultrasonic(RIGHT_TRIG, RIGHT_ECHO)
            self.stamps["right_us"] = now if self.right_us_cm is not None else None
        self.measure_left_next = not self.measure_left_next
        self.next_ultrasonic_s = time.monotonic() + ULTRASONIC_INTERVAL_S

    def fresh(self, name: str, now: float) -> bool:
        stamp = self.stamps[name]
        return stamp is not None and now - stamp <= SENSOR_TIMEOUT_S

    def all_sensors_fresh(self, now: float) -> bool:
        return all(self.fresh(name, now) for name in self.stamps)

    def update_odometry(self, now: float) -> None:
        dt = now - self.last_odometry_s
        self.last_odometry_s = now
        if self.fresh("stm32", now):
            self.odometry.update(self.left_wheel_rpm, self.right_wheel_rpm, dt)

    def update_fused_objects(self) -> None:
        self.fused_objects = fuse_confirmed_tracks(
            self.tracker.confirmed_visible(), self.lidar_points, self.fusion_config
        )
        self.priority_target = select_priority_target(
            self.fused_objects, self.fusion_config
        )

    def left_clear(self) -> bool:
        return bool(
            self.left_us_cm is not None
            and self.left_us_cm >= SIDE_CLEAR_CM
            and self.lidar_left_m is not None
            and self.lidar_left_m >= LIDAR_SIDE_CLEAR_M
        )

    def right_clear(self) -> bool:
        return bool(
            self.right_us_cm is not None
            and self.right_us_cm >= SIDE_CLEAR_CM
            and self.lidar_right_m is not None
            and self.lidar_right_m >= LIDAR_SIDE_CLEAR_M
        )

    def side_blocked(self, state: str) -> bool:
        if state == self.TURN_LEFT:
            return bool(
                self.left_us_cm is None
                or self.left_us_cm <= SIDE_STOP_CM
                or self.lidar_left_m is None
                or self.lidar_left_m <= LIDAR_SIDE_STOP_M
            )
        return bool(
            self.right_us_cm is None
            or self.right_us_cm <= SIDE_STOP_CM
            or self.lidar_right_m is None
            or self.lidar_right_m <= LIDAR_SIDE_STOP_M
        )

    def choose_turn(self, target: FusedObject) -> str | None:
        left, right = self.left_clear(), self.right_clear()
        # Positive bearing is left. Prefer passing on the opposite side.
        preferred = self.TURN_RIGHT if target.bearing_rad > 0.0 else self.TURN_LEFT
        if abs(target.bearing_rad) < math.radians(3.0):
            left_clearance = min(
                float(self.lidar_left_m or 0.0),
                float(self.left_us_cm or 0.0) / 100.0,
            )
            right_clearance = min(
                float(self.lidar_right_m or 0.0),
                float(self.right_us_cm or 0.0) / 100.0,
            )
            preferred = (
                self.TURN_LEFT
                if left_clearance >= right_clearance
                else self.TURN_RIGHT
            )
        if preferred == self.TURN_LEFT and left:
            return self.TURN_LEFT
        if preferred == self.TURN_RIGHT and right:
            return self.TURN_RIGHT
        if left and not right:
            return self.TURN_LEFT
        if right and not left:
            return self.TURN_RIGHT
        if left and right:
            return preferred
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
        print(f"\n상태 전환: {state}")

    def update_state(self, now: float) -> None:
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
                self.transition(self.WAIT_CLEAR, now)
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
                self.entry_follower = None
                self.transition(self.WAIT_CLEAR, now)
            return

        if self.state == self.BYPASS:
            travelled = self.odometry.distance_travelled_m - self.bypass_start_distance_m
            if travelled >= BYPASS_DISTANCE_M:
                self.start_return_path(now)
            return

        if self.state == self.RETURN_PATH:
            if self.return_follower is None:
                self.start_return_path(now)
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
        # SHARP remains independent of YOLO/LiDAR fusion.
        if self.sharp_cm is not None and self.sharp_cm <= SHARP_EMERGENCY_CM:
            return 0, 0
        # Preserve the existing last-resort LiDAR hard stop, but do not use a
        # bare LiDAR return to start a normal avoidance maneuver.
        if self.lidar_front_m is not None and self.lidar_front_m <= LIDAR_HARD_STOP_M:
            return 0, 0
        if self.state == self.DRIVE:
            return CRUISE_RPM, CRUISE_RPM
        if self.state in (self.STOPPING, self.WAIT_CLEAR):
            return 0, 0
        if self.state in (self.TURN_LEFT, self.TURN_RIGHT):
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

    def tick(self) -> None:
        now = time.monotonic()
        self.read_stm32()
        self.update_ultrasonic()
        self.update_odometry(now)
        self.update_fused_objects()
        self.update_state(now)
        left, right = self.motor_command(now)
        self.serial.write(f"$CMD,{left},{right},0\r\n".encode("ascii"))
        self.serial.flush()

        target = self.priority_target
        target_report = (
            "없음"
            if target is None
            else (
                f"{target.label}#{target.track_id} {target.distance_m:.2f}m "
                f"Y={target.lateral_min_m:.2f}~{target.lateral_max_m:.2f}m"
            )
        )
        report = (
            self.state,
            left,
            right,
            self.sharp_cm,
            round(self.lidar_front_m or -1.0, 2),
            round(self.lidar_left_m or -1.0, 2),
            round(self.lidar_right_m or -1.0, 2),
            round(self.left_us_cm or -1.0, 1),
            round(self.right_us_cm or -1.0, 1),
            tuple(self.labels),
            None if target is None else target.track_id,
            round(self.odometry.y_m, 2),
            round(math.degrees(self.odometry.yaw_rad), 1),
        )
        if report != self.last_report:
            print(
                f"\n{self.state} MOTOR=({left},{right}) SHARP={self.sharp_cm}cm "
                f"LIDAR(F/L/R)=({self.fmt(self.lidar_front_m, 2)}/"
                f"{self.fmt(self.lidar_left_m, 2)}/"
                f"{self.fmt(self.lidar_right_m, 2)})m "
                f"US(L/R)=({self.fmt(self.left_us_cm)}/"
                f"{self.fmt(self.right_us_cm)})cm "
                f"YOLO={','.join(self.labels) if self.labels else '없음'} "
                f"TARGET={target_report} "
                f"POSE=(x={self.odometry.x_m:.2f}, y={self.odometry.y_m:.2f}, "
                f"yaw={math.degrees(self.odometry.yaw_rad):.1f}deg)"
            )
            self.last_report = report

    def shutdown(self) -> None:
        for _ in range(10):
            try:
                self.serial.write(b"$CMD,0,0,0\r\n")
                self.serial.flush()
            except Exception:
                pass
            time.sleep(0.05)
        self.serial.close()
        GPIO.output(LEFT_TRIG, GPIO.LOW)
        GPIO.output(RIGHT_TRIG, GPIO.LOW)
        GPIO.cleanup()


def main() -> None:
    rclpy.init()
    node = IntegratedAvoidance()
    print(
        "YOLO 추적 + LiDAR 객체거리 + SHARP 긴급제동 자동회피 시험 "
        "(종료: Ctrl+C)"
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.tick()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n시험 종료")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("모터 및 GPIO 정리 완료")


if __name__ == "__main__":
    main()
