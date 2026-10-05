"""Generate an illustrative GIF comparing the two avoidance state machines.

The virtual sensor geometry is deliberately simple.  State thresholds, wheel
RPM commands, fixed turn duration, bypass distance, and return-path steering
match the values in ``scripts/test_integrated_avoidance.py`` and
``scripts/test_fused_path_avoidance.py``.  This is a behavior explainer, not a
collision-proof physics or sensor validation environment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import matplotlib

matplotlib.use("Agg")

from matplotlib import animation, font_manager, patches, pyplot as plt

from jetson.amr_core.reactive_avoidance import (
    BezierPathFollower,
    CubicBezierPath,
    differential_rpm_command,
    line_heading_error_rad,
    local_point_to_world,
    return_heading_error_rad,
    wrap_angle,
)


DT_S = 0.05
DURATION_S = 35.0
FRAME_STEP = 5

WHEEL_DIAMETER_M = 0.20
WHEEL_BASE_M = 0.50
ROBOT_WIDTH_M = 0.50
ROBOT_LENGTH_M = 0.65
SAFETY_MARGIN_M = 0.15
CORRIDOR_HALF_WIDTH_M = ROBOT_WIDTH_M * 0.5 + SAFETY_MARGIN_M

OBSTACLE_X_M = 2.40
OBSTACLE_Y_M = -0.18
OBSTACLE_LENGTH_M = 0.55
OBSTACLE_WIDTH_M = 0.55
OBSTACLE_RADIUS_M = 0.5 * max(OBSTACLE_LENGTH_M, OBSTACLE_WIDTH_M)

IMAGE_WIDTH_PX = 640.0
CAMERA_FOV_DEG = 69.0
CAMERA_HALF_FOV_RAD = math.radians(CAMERA_FOV_DEG * 0.5)
CAMERA_FOCAL_PX = (IMAGE_WIDTH_PX * 0.5) / math.tan(CAMERA_HALF_FOV_RAD)

STATE_KO = {
    "DRIVE": "직진",
    "STOPPING": "회피 전 정지",
    "WAIT_CLEAR": "공간 대기",
    "TURN_LEFT": "좌측 회피",
    "TURN_RIGHT": "우측 회피",
    "BYPASS": "장애물 통과",
    "RETURN_PATH": "원경로 복귀",
}


@dataclass
class Pose:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    distance_m: float = 0.0


@dataclass(frozen=True)
class SensorFrame:
    yolo_visible: bool
    confidence: float
    bbox_center_px: float
    lidar_front_m: float
    lidar_cluster_m: float
    lidar_left_m: float
    lidar_right_m: float
    sharp_cm: int
    left_ultrasonic_cm: float
    right_ultrasonic_cm: float
    bearing_rad: float
    bearing_min_rad: float
    bearing_max_rad: float
    forward_distance_m: float
    lateral_min_m: float
    lateral_max_m: float
    intersects_corridor: bool


@dataclass(frozen=True)
class Snapshot:
    time_s: float
    pose: Pose
    state: str
    left_rpm: int
    right_rpm: int
    sensors: SensorFrame
    decision: str


def interval_overlaps(
    first_min: float, first_max: float, second_min: float, second_max: float
) -> bool:
    return first_max >= second_min and first_min <= second_max


def virtual_sensors(pose: Pose) -> SensorFrame:
    dx = OBSTACLE_X_M - pose.x_m
    dy = OBSTACLE_Y_M - pose.y_m
    cosine = math.cos(pose.yaw_rad)
    sine = math.sin(pose.yaw_rad)
    forward = cosine * dx + sine * dy
    lateral = -sine * dx + cosine * dy
    center_distance = max(0.01, math.hypot(forward, lateral))
    bearing = math.atan2(lateral, forward)
    half_angle = math.asin(min(0.95, OBSTACLE_RADIUS_M / center_distance))
    bearing_min = bearing - half_angle
    bearing_max = bearing + half_angle
    surface_distance = max(0.05, center_distance - OBSTACLE_RADIUS_M)

    yolo_visible = bool(
        forward > 0.0
        and center_distance <= 4.0
        and interval_overlaps(
            bearing_min,
            bearing_max,
            -CAMERA_HALF_FOV_RAD,
            CAMERA_HALF_FOV_RAD,
        )
    )
    bbox_center_px = IMAGE_WIDTH_PX * 0.5 - CAMERA_FOCAL_PX * math.tan(bearing)
    bbox_center_px = max(0.0, min(IMAGE_WIDTH_PX, bbox_center_px))
    confidence = 0.90 if yolo_visible else 0.0

    front_half = math.radians(15.0)
    left_min, left_max = math.radians(20.0), math.radians(75.0)
    right_min, right_max = -left_max, -left_min
    lidar_front = (
        surface_distance
        if forward > 0.0
        and interval_overlaps(bearing_min, bearing_max, -front_half, front_half)
        else 6.0
    )
    lidar_left = (
        surface_distance
        if forward > 0.0
        and interval_overlaps(bearing_min, bearing_max, left_min, left_max)
        else 6.0
    )
    lidar_right = (
        surface_distance
        if forward > 0.0
        and interval_overlaps(bearing_min, bearing_max, right_min, right_max)
        else 6.0
    )

    sharp_in_beam = forward > 0.0 and abs(bearing) <= math.radians(8.0)
    sharp_cm = (
        max(10, min(80, round(surface_distance * 100.0)))
        if sharp_in_beam
        else 80
    )
    lateral_min = lateral - OBSTACLE_WIDTH_M * 0.5
    lateral_max = lateral + OBSTACLE_WIDTH_M * 0.5
    intersects = bool(
        forward > 0.0
        and lateral_max >= -CORRIDOR_HALF_WIDTH_M
        and lateral_min <= CORRIDOR_HALF_WIDTH_M
    )

    # Both sides are intentionally open in this comparison.  The nearby
    # obstacle may appear in one LiDAR side sector, but the chosen left side
    # remains clear.
    return SensorFrame(
        yolo_visible=yolo_visible,
        confidence=confidence,
        bbox_center_px=bbox_center_px,
        lidar_front_m=lidar_front,
        lidar_cluster_m=surface_distance,
        lidar_left_m=lidar_left,
        lidar_right_m=lidar_right,
        sharp_cm=sharp_cm,
        left_ultrasonic_cm=180.0,
        right_ultrasonic_cm=180.0,
        bearing_rad=bearing,
        bearing_min_rad=bearing_min,
        bearing_max_rad=bearing_max,
        forward_distance_m=max(0.05, forward - OBSTACLE_LENGTH_M * 0.5),
        lateral_min_m=lateral_min,
        lateral_max_m=lateral_max,
        intersects_corridor=intersects,
    )


def integrate_pose(pose: Pose, left_rpm: float, right_rpm: float) -> None:
    circumference = math.pi * WHEEL_DIAMETER_M
    left_mps = left_rpm * circumference / 60.0
    right_mps = right_rpm * circumference / 60.0
    linear_mps = 0.5 * (left_mps + right_mps)
    angular_rps = (right_mps - left_mps) / WHEEL_BASE_M
    midpoint = pose.yaw_rad + 0.5 * angular_rps * DT_S
    pose.x_m += linear_mps * math.cos(midpoint) * DT_S
    pose.y_m += linear_mps * math.sin(midpoint) * DT_S
    pose.yaw_rad = wrap_angle(pose.yaw_rad + angular_rps * DT_S)
    pose.distance_m += abs(linear_mps) * DT_S


@dataclass
class IntegratedModel:
    pose: Pose = field(default_factory=Pose)
    state: str = "DRIVE"
    state_started_s: float = 0.0
    cooldown_until_s: float = 0.0
    close_count: int = 0
    clear_count: int = 0
    last_center_turn: str = "TURN_LEFT"

    def transition(self, new_state: str, now_s: float) -> None:
        self.state = new_state
        self.state_started_s = now_s
        self.clear_count = 0

    def choose_turn(self, sensors: SensorFrame) -> str:
        if sensors.bbox_center_px < IMAGE_WIDTH_PX * 0.45:
            return "TURN_RIGHT"
        if sensors.bbox_center_px > IMAGE_WIDTH_PX * 0.55:
            return "TURN_LEFT"
        selected = "TURN_RIGHT" if self.last_center_turn == "TURN_LEFT" else "TURN_LEFT"
        self.last_center_turn = selected
        return selected

    def step(self, now_s: float) -> Snapshot:
        sensors = virtual_sensors(self.pose)
        lidar_close = sensors.lidar_front_m <= 1.2
        sharp_close = sensors.sharp_cm <= 55
        close = lidar_close or (sensors.yolo_visible and sharp_close)
        clear = sensors.lidar_front_m >= 1.6 and sensors.sharp_cm >= 70

        if self.state == "DRIVE":
            if now_s < self.cooldown_until_s:
                self.close_count = 0
            elif close:
                self.close_count += 1
                if self.close_count >= 2:
                    self.close_count = 0
                    self.transition("STOPPING", now_s)
            else:
                self.close_count = 0
        elif self.state == "STOPPING":
            if clear:
                self.clear_count += 1
                if self.clear_count >= 5:
                    self.cooldown_until_s = now_s + 0.8
                    self.transition("DRIVE", now_s)
            elif now_s - self.state_started_s >= 0.3:
                self.transition(self.choose_turn(sensors), now_s)
        elif self.state == "WAIT_CLEAR":
            if clear:
                self.clear_count += 1
                if self.clear_count >= 5:
                    self.transition("DRIVE", now_s)
            else:
                self.transition(self.choose_turn(sensors), now_s)
        elif self.state in ("TURN_LEFT", "TURN_RIGHT"):
            chosen_lidar = (
                sensors.lidar_left_m
                if self.state == "TURN_LEFT"
                else sensors.lidar_right_m
            )
            if chosen_lidar <= 0.35:
                self.transition("WAIT_CLEAR", now_s)
            elif now_s - self.state_started_s >= 1.3:
                self.cooldown_until_s = now_s + 0.8
                self.transition("DRIVE", now_s)

        if sensors.sharp_cm <= 20 or sensors.lidar_front_m <= 0.25:
            left_rpm, right_rpm = 0, 0
        elif self.state == "DRIVE":
            left_rpm, right_rpm = 20, 20
        elif self.state == "TURN_LEFT":
            left_rpm, right_rpm = 3, 22
        elif self.state == "TURN_RIGHT":
            left_rpm, right_rpm = 22, 3
        else:
            left_rpm, right_rpm = 0, 0

        if self.state == "DRIVE" and close:
            decision = "근접 판정 누적: LiDAR≤1.2m 또는 YOLO+SHARP≤55cm"
        elif self.state == "STOPPING":
            decision = "0.3초 정지 후 회피 방향 선택"
        elif self.state.startswith("TURN"):
            decision = "고정 RPM으로 1.3초 회전"
        elif self.state == "DRIVE" and abs(self.pose.yaw_rad) > math.radians(5.0):
            decision = "고정 회전 종료 → 현재 방향으로 계속 직진"
        else:
            decision = "전방 센서 임계값 대기"

        snapshot = Snapshot(
            time_s=now_s,
            pose=Pose(**vars(self.pose)),
            state=self.state,
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            sensors=sensors,
            decision=decision,
        )
        integrate_pose(self.pose, left_rpm, right_rpm)
        return snapshot


@dataclass
class FusedPathModel:
    pose: Pose = field(default_factory=Pose)
    state: str = "DRIVE"
    state_started_s: float = 0.0
    cooldown_until_s: float = 0.0
    detection_hits: int = 0
    bypass_start_distance_m: float = 0.0
    bypass_target_y_m: float = 0.0
    entry_follower: BezierPathFollower | None = None
    return_follower: BezierPathFollower | None = None

    def transition(self, new_state: str, now_s: float) -> None:
        self.state = new_state
        self.state_started_s = now_s

    @staticmethod
    def side_clear(sensors: SensorFrame, left: bool) -> bool:
        lidar = sensors.lidar_left_m if left else sensors.lidar_right_m
        ultrasonic = (
            sensors.left_ultrasonic_cm if left else sensors.right_ultrasonic_cm
        )
        return lidar >= 0.8 and ultrasonic >= 35.0

    def choose_turn(self, sensors: SensorFrame) -> str | None:
        left_clear = self.side_clear(sensors, True)
        right_clear = self.side_clear(sensors, False)
        preferred = "TURN_RIGHT" if sensors.bearing_rad > 0.0 else "TURN_LEFT"
        if abs(sensors.bearing_rad) < math.radians(3.0):
            preferred = (
                "TURN_LEFT"
                if sensors.lidar_left_m >= sensors.lidar_right_m
                else "TURN_RIGHT"
            )
        if preferred == "TURN_LEFT" and left_clear:
            return "TURN_LEFT"
        if preferred == "TURN_RIGHT" and right_clear:
            return "TURN_RIGHT"
        if left_clear:
            return "TURN_LEFT"
        if right_clear:
            return "TURN_RIGHT"
        return None

    def start_entry_path(
        self, sensors: SensorFrame, turn: str, now_s: float
    ) -> None:
        turn_left = turn == "TURN_LEFT"
        lateral_offset = (
            sensors.lateral_max_m + CORRIDOR_HALF_WIDTH_M
            if turn_left
            else sensors.lateral_min_m - CORRIDOR_HALF_WIDTH_M
        )
        entry_forward = max(
            0.65,
            sensors.forward_distance_m
            - (ROBOT_LENGTH_M * 0.5 + SAFETY_MARGIN_M),
        )
        goal_x, goal_y = local_point_to_world(
            origin_x_m=self.pose.x_m,
            origin_y_m=self.pose.y_m,
            origin_yaw_rad=self.pose.yaw_rad,
            forward_m=entry_forward,
            lateral_m=lateral_offset,
        )
        path = CubicBezierPath.from_poses(
            start_x_m=self.pose.x_m,
            start_y_m=self.pose.y_m,
            start_yaw_rad=self.pose.yaw_rad,
            goal_x_m=goal_x,
            goal_y_m=goal_y,
            goal_yaw_rad=0.0,
        )
        self.entry_follower = BezierPathFollower(
            path, lookahead_m=0.25, completion_radius_m=0.14
        )
        self.return_follower = None
        self.bypass_target_y_m = goal_y
        self.transition(turn, now_s)

    def start_return_path(self, now_s: float) -> None:
        forward = max(1.0, abs(self.pose.y_m) * 2.0)
        path = CubicBezierPath.from_poses(
            start_x_m=self.pose.x_m,
            start_y_m=self.pose.y_m,
            start_yaw_rad=self.pose.yaw_rad,
            goal_x_m=self.pose.x_m + forward,
            goal_y_m=0.0,
            goal_yaw_rad=0.0,
        )
        self.return_follower = BezierPathFollower(
            path, lookahead_m=0.25, completion_radius_m=0.14
        )
        self.transition("RETURN_PATH", now_s)

    def step(self, now_s: float) -> Snapshot:
        sensors = virtual_sensors(self.pose)
        if sensors.yolo_visible:
            self.detection_hits += 1
        else:
            self.detection_hits = 0
        fused_object = sensors.yolo_visible and self.detection_hits >= 3
        priority_target = bool(
            fused_object
            and sensors.intersects_corridor
            and sensors.forward_distance_m <= 2.0
        )

        # Only DRIVE can start a maneuver for this single simulated object.
        # Once a path is frozen, seeing the same obstacle again cannot restart
        # the maneuver; real code still permits a spatially different target.
        if self.state == "DRIVE":
            if priority_target and now_s >= self.cooldown_until_s:
                self.transition("STOPPING", now_s)

        if self.state == "STOPPING":
            if priority_target and now_s - self.state_started_s >= 0.3:
                turn = self.choose_turn(sensors)
                if turn is None:
                    self.transition("WAIT_CLEAR", now_s)
                else:
                    self.start_entry_path(sensors, turn, now_s)
            elif not priority_target:
                self.transition("DRIVE", now_s)
        elif self.state == "WAIT_CLEAR":
            if priority_target:
                turn = self.choose_turn(sensors)
                if turn is not None:
                    self.start_entry_path(sensors, turn, now_s)
            else:
                self.transition("DRIVE", now_s)
        elif self.state in ("TURN_LEFT", "TURN_RIGHT"):
            chosen_lidar = (
                sensors.lidar_left_m
                if self.state == "TURN_LEFT"
                else sensors.lidar_right_m
            )
            if chosen_lidar <= 0.35:
                self.transition("WAIT_CLEAR", now_s)
            elif self.entry_follower is not None and self.entry_follower.completed(
                pose_x_m=self.pose.x_m,
                pose_y_m=self.pose.y_m,
                pose_yaw_rad=self.pose.yaw_rad,
            ):
                self.bypass_start_distance_m = self.pose.distance_m
                self.transition("BYPASS", now_s)
            elif now_s - self.state_started_s >= 20.0:
                self.transition("WAIT_CLEAR", now_s)
        elif self.state == "BYPASS":
            if self.pose.distance_m - self.bypass_start_distance_m >= 0.70:
                self.start_return_path(now_s)
        elif self.state == "RETURN_PATH":
            if self.return_follower is not None and self.return_follower.completed(
                pose_x_m=self.pose.x_m,
                pose_y_m=self.pose.y_m,
                pose_yaw_rad=self.pose.yaw_rad,
            ):
                self.cooldown_until_s = now_s + 0.5
                self.transition("DRIVE", now_s)

        if sensors.sharp_cm <= 20 or sensors.lidar_front_m <= 0.25:
            left_rpm, right_rpm = 0, 0
        elif self.state == "DRIVE":
            left_rpm, right_rpm = 20, 20
        elif self.state in ("STOPPING", "WAIT_CLEAR"):
            left_rpm, right_rpm = 0, 0
        elif self.state in ("TURN_LEFT", "TURN_RIGHT"):
            heading_error = (
                0.0
                if self.entry_follower is None
                else self.entry_follower.heading_error_rad(
                    pose_x_m=self.pose.x_m,
                    pose_y_m=self.pose.y_m,
                    pose_yaw_rad=self.pose.yaw_rad,
                )
            )
            left_rpm, right_rpm = differential_rpm_command(
                12,
                heading_error,
                gain_rpm_per_rad=18.0,
                maximum_delta_rpm=10.0,
                maximum_rpm=65,
            )
        elif self.state == "BYPASS":
            heading_error = line_heading_error_rad(
                pose_y_m=self.pose.y_m,
                target_y_m=self.bypass_target_y_m,
                pose_yaw_rad=self.pose.yaw_rad,
                lookahead_m=0.80,
            )
            left_rpm, right_rpm = differential_rpm_command(
                16,
                heading_error,
                gain_rpm_per_rad=16.0,
                maximum_delta_rpm=8.0,
                maximum_rpm=65,
            )
        elif self.state == "RETURN_PATH":
            heading_error = (
                return_heading_error_rad(
                    pose_y_m=self.pose.y_m,
                    pose_yaw_rad=self.pose.yaw_rad,
                    lookahead_m=0.80,
                )
                if self.return_follower is None
                else self.return_follower.heading_error_rad(
                    pose_x_m=self.pose.x_m,
                    pose_y_m=self.pose.y_m,
                    pose_yaw_rad=self.pose.yaw_rad,
                )
            )
            left_rpm, right_rpm = differential_rpm_command(
                14,
                heading_error,
                gain_rpm_per_rad=16.0,
                maximum_delta_rpm=8.0,
                maximum_rpm=65,
            )
        else:
            left_rpm, right_rpm = 0, 0

        if self.state == "DRIVE" and priority_target:
            decision = "YOLO 3프레임 + LiDAR 군집 + 회랑 겹침≤2.0m"
        elif self.state == "STOPPING":
            decision = "객체 위치 고정 → 0.3초 정지 후 S자 궤적 생성"
        elif self.state.startswith("TURN"):
            progress = (
                0.0
                if self.entry_follower is None
                else self.entry_follower.progress_ratio * 100.0
            )
            decision = f"고정된 부드러운 진입 궤적 추종 ({progress:.0f}%)"
        elif self.state == "BYPASS":
            travelled = self.pose.distance_m - self.bypass_start_distance_m
            decision = f"장애물 옆 고정 차선 평행 주행 ({travelled:.2f}/0.70m)"
        elif self.state == "RETURN_PATH":
            progress = (
                0.0
                if self.return_follower is None
                else self.return_follower.progress_ratio * 100.0
            )
            decision = f"고정된 복귀 S자 궤적 추종 ({progress:.0f}%)"
        else:
            decision = "YOLO-LiDAR 대응 대상과 주행 회랑 검사"

        snapshot = Snapshot(
            time_s=now_s,
            pose=Pose(**vars(self.pose)),
            state=self.state,
            left_rpm=left_rpm,
            right_rpm=right_rpm,
            sensors=sensors,
            decision=decision,
        )
        integrate_pose(self.pose, left_rpm, right_rpm)
        return snapshot


def simulate() -> tuple[list[Snapshot], list[Snapshot]]:
    integrated = IntegratedModel()
    fused = FusedPathModel()
    integrated_records: list[Snapshot] = []
    fused_records: list[Snapshot] = []
    steps = round(DURATION_S / DT_S) + 1
    for index in range(steps):
        now_s = index * DT_S
        integrated_records.append(integrated.step(now_s))
        fused_records.append(fused.step(now_s))
    return integrated_records, fused_records


def configure_fonts() -> None:
    korean_font = Path("C:/Windows/Fonts/malgun.ttf")
    if korean_font.exists():
        font_manager.fontManager.addfont(str(korean_font))
        plt.rcParams["font.family"] = font_manager.FontProperties(
            fname=str(korean_font)
        ).get_name()
    plt.rcParams["axes.unicode_minus"] = False


def robot_polygon(pose: Pose) -> list[tuple[float, float]]:
    half_length = ROBOT_LENGTH_M * 0.5
    half_width = ROBOT_WIDTH_M * 0.5
    local_corners = [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ]
    cosine = math.cos(pose.yaw_rad)
    sine = math.sin(pose.yaw_rad)
    return [
        (
            pose.x_m + cosine * x - sine * y,
            pose.y_m + sine * x + cosine * y,
        )
        for x, y in local_corners
    ]


def draw_sensor_overlay(ax: plt.Axes, snapshot: Snapshot, fused: bool) -> None:
    pose = snapshot.pose
    sensors = snapshot.sensors
    cone_range = min(1.25, max(0.35, sensors.lidar_cluster_m))
    cone_points = [(pose.x_m, pose.y_m)]
    for angle in (pose.yaw_rad + CAMERA_HALF_FOV_RAD, pose.yaw_rad - CAMERA_HALF_FOV_RAD):
        cone_points.append(
            (
                pose.x_m + cone_range * math.cos(angle),
                pose.y_m + cone_range * math.sin(angle),
            )
        )
    ax.add_patch(
        patches.Polygon(
            cone_points,
            closed=True,
            facecolor="#4CC9F0",
            edgecolor="none",
            alpha=0.10 if sensors.yolo_visible else 0.035,
            zorder=1,
        )
    )

    if sensors.yolo_visible and (fused or sensors.lidar_front_m < 6.0):
        ax.plot(
            [pose.x_m, OBSTACLE_X_M],
            [pose.y_m, OBSTACLE_Y_M],
            color="#F59E0B",
            linewidth=1.5,
            linestyle=(0, (2, 2)),
            alpha=0.8,
            zorder=3,
        )
    sharp_length = min(0.8, sensors.sharp_cm / 100.0)
    ax.plot(
        [pose.x_m, pose.x_m + sharp_length * math.cos(pose.yaw_rad)],
        [pose.y_m, pose.y_m + sharp_length * math.sin(pose.yaw_rad)],
        color="#D946EF",
        linewidth=2.0,
        alpha=0.55,
        zorder=3,
    )


def draw_panel(
    ax: plt.Axes,
    records: list[Snapshot],
    record_index: int,
    title: str,
    color: str,
    fused: bool,
) -> None:
    ax.clear()
    snapshot = records[record_index]
    pose = snapshot.pose
    sensors = snapshot.sensors

    ax.axhspan(
        -CORRIDOR_HALF_WIDTH_M,
        CORRIDOR_HALF_WIDTH_M,
        color="#CBD5E1",
        alpha=0.18,
        zorder=0,
        label="원 주행 회랑",
    )
    ax.axhline(0.0, color="#64748B", linestyle="--", linewidth=1.4, zorder=1)
    ax.add_patch(
        patches.Rectangle(
            (
                OBSTACLE_X_M - OBSTACLE_LENGTH_M * 0.5,
                OBSTACLE_Y_M - OBSTACLE_WIDTH_M * 0.5,
            ),
            OBSTACLE_LENGTH_M,
            OBSTACLE_WIDTH_M,
            facecolor="#EF4444",
            edgecolor="#991B1B",
            linewidth=1.8,
            alpha=0.88,
            zorder=4,
        )
    )
    ax.text(
        OBSTACLE_X_M,
        OBSTACLE_Y_M,
        "장애물",
        ha="center",
        va="center",
        color="white",
        fontsize=10,
        fontweight="bold",
        zorder=5,
    )

    history = records[: record_index + 1]
    ax.plot(
        [item.pose.x_m for item in history],
        [item.pose.y_m for item in history],
        color=color,
        linewidth=3.0,
        zorder=3,
    )
    draw_sensor_overlay(ax, snapshot, fused)
    ax.add_patch(
        patches.Polygon(
            robot_polygon(pose),
            closed=True,
            facecolor=color,
            edgecolor="#0F172A",
            linewidth=1.5,
            zorder=6,
        )
    )
    ax.arrow(
        pose.x_m,
        pose.y_m,
        0.42 * math.cos(pose.yaw_rad),
        0.42 * math.sin(pose.yaw_rad),
        width=0.015,
        head_width=0.10,
        head_length=0.10,
        length_includes_head=True,
        color="#0F172A",
        zorder=7,
    )

    yolo_text = (
        f"person {sensors.confidence:.2f}, x={sensors.bbox_center_px:.0f}px"
        if sensors.yolo_visible
        else "없음"
    )
    lidar_text = f"전방 {sensors.lidar_front_m:.2f}m"
    if fused:
        lidar_text += f" / 대응 군집 {sensors.lidar_cluster_m:.2f}m"
    info = (
        f"상태  {STATE_KO[snapshot.state]}\n"
        f"모터  L/R = {snapshot.left_rpm}/{snapshot.right_rpm} RPM\n"
        f"YOLO  {yolo_text}\n"
        f"LiDAR  {lidar_text}\n"
        f"SHARP  {sensors.sharp_cm}cm"
        + (" (센서 상한)" if sensors.sharp_cm == 80 else "")
        + f"\n판정  {snapshot.decision}"
    )
    ax.text(
        0.02,
        0.98,
        info,
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=9.2,
        linespacing=1.38,
        bbox={
            "boxstyle": "round,pad=0.55",
            "facecolor": "white",
            "edgecolor": color,
            "alpha": 0.94,
        },
        zorder=10,
    )
    if fused and 2.7 <= snapshot.time_s <= 16.0:
        ax.text(
            0.5,
            0.025,
            "개선 로직: 최초 객체 위치로 S자 궤적을 고정하여 같은 장애물 재감지에 흔들리지 않음",
            transform=ax.transAxes,
            ha="center",
            va="bottom",
            fontsize=8.6,
            color="#7C2D12",
            bbox={
                "boxstyle": "round,pad=0.35",
                "facecolor": "#FEF3C7",
                "edgecolor": "#F59E0B",
                "alpha": 0.95,
            },
            zorder=11,
        )

    ax.set_title(title, fontsize=15, fontweight="bold", color=color, pad=10)
    ax.set_xlim(-0.25, 6.70)
    ax.set_ylim(-1.30, 3.50)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("진행 거리 x (m)")
    ax.set_ylabel("원경로 기준 좌우 y (m)")
    ax.grid(True, color="#E2E8F0", linewidth=0.7)
    ax.set_facecolor("#F8FAFC")


def generate(output_path: Path) -> None:
    configure_fonts()
    integrated_records, fused_records = simulate()
    frame_indices = list(range(0, len(integrated_records), FRAME_STEP))
    if frame_indices[-1] != len(integrated_records) - 1:
        frame_indices.append(len(integrated_records) - 1)

    fig, axes = plt.subplots(1, 2, figsize=(14.0, 7.8), dpi=100)
    fig.patch.set_facecolor("white")
    fig.subplots_adjust(left=0.055, right=0.985, bottom=0.13, top=0.84, wspace=0.16)

    fig.suptitle(
        "동일한 가상 장애물·센서 입력에서 회피 방식 비교",
        fontsize=20,
        fontweight="bold",
        y=0.965,
    )
    subtitle = (
        "장애물: 정면에서 약간 오른쪽 · YOLO/LiDAR/SHARP 정상 · 좌우 공간 열림  |  "
        "청록=카메라 FOV, 주황 점선=LiDAR 대응, 자홍=SHARP 빔 · 개선판은 고정 S자 궤적 사용"
    )
    fig.text(0.5, 0.915, subtitle, ha="center", va="center", fontsize=11, color="#475569")
    time_text = fig.text(
        0.5,
        0.055,
        "",
        ha="center",
        va="center",
        fontsize=12,
        color="#0F172A",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "#F1F5F9", "edgecolor": "#CBD5E1"},
    )
    fig.text(
        0.5,
        0.018,
        "설명용 단순 시뮬레이션: 센서 노이즈·미끄러짐·통신 지연은 제외",
        ha="center",
        va="center",
        fontsize=9.5,
        color="#64748B",
    )

    def update(frame_number: int) -> list:
        record_index = frame_indices[frame_number]
        draw_panel(
            axes[0],
            integrated_records,
            record_index,
            "기존: integrated avoidance",
            "#2563EB",
            False,
        )
        draw_panel(
            axes[1],
            fused_records,
            record_index,
            "현재: fused path avoidance",
            "#16A34A",
            True,
        )
        time_text.set_text(f"가상 시간  {integrated_records[record_index].time_s:4.1f} s")
        return [time_text]

    movie = animation.FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        interval=100,
        blit=False,
        repeat=True,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    movie.save(output_path, writer=animation.PillowWriter(fps=10), dpi=90)
    plt.close(fig)


def main() -> None:
    output_path = Path(__file__).with_name("avoidance_comparison.gif")
    generate(output_path)
    print(output_path.resolve())


if __name__ == "__main__":
    main()
