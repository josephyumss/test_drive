"""Hardware-independent YOLO/LiDAR fusion for local reactive avoidance.

The module deliberately has no ROS, GPIO, serial, OpenCV, or third-party
dependencies.  The real-robot bench node supplies the existing YOLO boxes,
LaserScan samples, and STM32 wheel RPM feedback; this module only performs
tracking, geometry, target selection, and differential-drive odometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Iterable, Sequence


@dataclass(frozen=True)
class FusionConfig:
    image_width_px: float = 640.0
    camera_horizontal_fov_deg: float = 69.0
    camera_lidar_yaw_offset_deg: float = 0.0
    bbox_angular_padding_deg: float = 1.5
    lidar_cluster_distance_gap_m: float = 0.30
    lidar_cluster_angle_gap_deg: float = 2.0
    minimum_cluster_points: int = 2
    robot_width_m: float = 0.50
    safety_margin_m: float = 0.15
    avoidance_trigger_distance_m: float = 2.0

    def __post_init__(self) -> None:
        if self.image_width_px <= 0:
            raise ValueError("image_width_px must be positive")
        if not 1.0 < self.camera_horizontal_fov_deg < 179.0:
            raise ValueError("camera_horizontal_fov_deg must be in (1, 179)")
        if self.robot_width_m <= 0 or self.safety_margin_m < 0:
            raise ValueError("robot width must be positive and margin non-negative")
        if self.avoidance_trigger_distance_m <= 0:
            raise ValueError("avoidance trigger distance must be positive")
        if self.minimum_cluster_points <= 0:
            raise ValueError("minimum_cluster_points must be positive")

    @property
    def corridor_half_width_m(self) -> float:
        return self.robot_width_m * 0.5 + self.safety_margin_m


@dataclass
class TrackedDetection:
    track_id: int
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    consecutive_hits: int = 1
    missed_frames: int = 0


@dataclass(frozen=True)
class FusedObject:
    track_id: int
    label: str
    confidence: float
    confirmed_frames: int
    distance_m: float
    forward_distance_m: float
    bearing_rad: float
    bearing_min_rad: float
    bearing_max_rad: float
    lateral_min_m: float
    lateral_max_m: float
    intersects_corridor: bool


class DetectionTracker:
    """Small IoU tracker used only to reject one-frame YOLO detections."""

    def __init__(
        self,
        *,
        minimum_confidence: float = 0.45,
        required_frames: int = 3,
        iou_threshold: float = 0.30,
        maximum_missed_frames: int = 2,
    ) -> None:
        if not 0.0 <= minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if required_frames <= 0 or maximum_missed_frames < 0:
            raise ValueError("frame counts are invalid")
        if not 0.0 <= iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be in [0, 1]")
        self.minimum_confidence = minimum_confidence
        self.required_frames = required_frames
        self.iou_threshold = iou_threshold
        self.maximum_missed_frames = maximum_missed_frames
        self._next_id = 1
        self._tracks: list[TrackedDetection] = []

    @property
    def tracks(self) -> tuple[TrackedDetection, ...]:
        return tuple(self._tracks)

    def update(self, detections: Iterable[dict]) -> tuple[TrackedDetection, ...]:
        parsed = []
        for detection in detections:
            try:
                confidence = float(detection.get("confidence", 0.0))
                coordinates = detection.get("xyxy", [])
                if confidence < self.minimum_confidence or len(coordinates) != 4:
                    continue
                bbox = tuple(float(value) for value in coordinates)
                if not _valid_bbox(bbox):
                    continue
                parsed.append(
                    (
                        str(detection.get("class", "object")),
                        confidence,
                        bbox,
                    )
                )
            except (TypeError, ValueError):
                continue

        unmatched_detection_indexes = set(range(len(parsed)))
        for track in self._tracks:
            best_index = None
            best_iou = self.iou_threshold
            for index in unmatched_detection_indexes:
                label, _confidence, bbox = parsed[index]
                if label != track.label:
                    continue
                overlap = bbox_iou(track.bbox, bbox)
                if overlap >= best_iou:
                    best_index = index
                    best_iou = overlap
            if best_index is None:
                track.missed_frames += 1
                track.consecutive_hits = 0
                continue
            label, confidence, bbox = parsed[best_index]
            track.label = label
            track.confidence = confidence
            track.bbox = bbox
            track.consecutive_hits += 1
            track.missed_frames = 0
            unmatched_detection_indexes.remove(best_index)

        self._tracks = [
            track
            for track in self._tracks
            if track.missed_frames <= self.maximum_missed_frames
        ]
        for index in sorted(unmatched_detection_indexes):
            label, confidence, bbox = parsed[index]
            self._tracks.append(
                TrackedDetection(self._next_id, label, confidence, bbox)
            )
            self._next_id += 1
        return self.tracks

    def confirmed_visible(self) -> tuple[TrackedDetection, ...]:
        return tuple(
            track
            for track in self._tracks
            if track.missed_frames == 0
            and track.consecutive_hits >= self.required_frames
        )


class WheelOdometry:
    """Short-horizon pose estimate from existing STM32 wheel RPM feedback."""

    def __init__(self, *, wheel_diameter_m: float, wheel_base_m: float) -> None:
        if wheel_diameter_m <= 0 or wheel_base_m <= 0:
            raise ValueError("wheel geometry must be positive")
        self.wheel_diameter_m = wheel_diameter_m
        self.wheel_base_m = wheel_base_m
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.distance_travelled_m = 0.0

    def update(self, left_rpm: float, right_rpm: float, dt_s: float) -> None:
        if not 0.0 < dt_s <= 0.5:
            return
        circumference = math.pi * self.wheel_diameter_m
        left_mps = float(left_rpm) * circumference / 60.0
        right_mps = float(right_rpm) * circumference / 60.0
        linear = 0.5 * (left_mps + right_mps)
        angular = (right_mps - left_mps) / self.wheel_base_m
        midpoint = self.yaw_rad + 0.5 * angular * dt_s
        self.x_m += linear * math.cos(midpoint) * dt_s
        self.y_m += linear * math.sin(midpoint) * dt_s
        self.yaw_rad = wrap_angle(self.yaw_rad + angular * dt_s)
        self.distance_travelled_m += abs(linear) * dt_s


@dataclass(frozen=True)
class CubicBezierPath:
    """Fixed smooth path between two poses.

    The path is created once when avoidance starts.  Sensor jitter therefore
    cannot move the target on every control cycle.
    """

    p0: tuple[float, float]
    p1: tuple[float, float]
    p2: tuple[float, float]
    p3: tuple[float, float]

    @classmethod
    def from_poses(
        cls,
        *,
        start_x_m: float,
        start_y_m: float,
        start_yaw_rad: float,
        goal_x_m: float,
        goal_y_m: float,
        goal_yaw_rad: float,
        handle_ratio: float = 0.38,
        minimum_handle_m: float = 0.20,
    ) -> "CubicBezierPath":
        if handle_ratio <= 0.0 or minimum_handle_m <= 0.0:
            raise ValueError("Bezier handle values must be positive")
        chord = math.hypot(goal_x_m - start_x_m, goal_y_m - start_y_m)
        handle = max(minimum_handle_m, chord * handle_ratio)
        return cls(
            p0=(start_x_m, start_y_m),
            p1=(
                start_x_m + handle * math.cos(start_yaw_rad),
                start_y_m + handle * math.sin(start_yaw_rad),
            ),
            p2=(
                goal_x_m - handle * math.cos(goal_yaw_rad),
                goal_y_m - handle * math.sin(goal_yaw_rad),
            ),
            p3=(goal_x_m, goal_y_m),
        )

    def point(self, progress: float) -> tuple[float, float]:
        t = max(0.0, min(1.0, float(progress)))
        inverse = 1.0 - t
        weights = (
            inverse**3,
            3.0 * inverse * inverse * t,
            3.0 * inverse * t * t,
            t**3,
        )
        points = (self.p0, self.p1, self.p2, self.p3)
        return (
            sum(weight * point[0] for weight, point in zip(weights, points)),
            sum(weight * point[1] for weight, point in zip(weights, points)),
        )

    def tangent(self, progress: float) -> tuple[float, float]:
        t = max(0.0, min(1.0, float(progress)))
        inverse = 1.0 - t
        return (
            3.0 * inverse * inverse * (self.p1[0] - self.p0[0])
            + 6.0 * inverse * t * (self.p2[0] - self.p1[0])
            + 3.0 * t * t * (self.p3[0] - self.p2[0]),
            3.0 * inverse * inverse * (self.p1[1] - self.p0[1])
            + 6.0 * inverse * t * (self.p2[1] - self.p1[1])
            + 3.0 * t * t * (self.p3[1] - self.p2[1]),
        )


class BezierPathFollower:
    """Monotonic pure-pursuit follower for a fixed cubic Bezier path."""

    def __init__(
        self,
        path: CubicBezierPath,
        *,
        lookahead_m: float = 0.28,
        sample_count: int = 121,
        completion_radius_m: float = 0.14,
        completion_heading_tolerance_rad: float = math.radians(2.0),
    ) -> None:
        if (
            lookahead_m <= 0.0
            or completion_radius_m <= 0.0
            or completion_heading_tolerance_rad <= 0.0
        ):
            raise ValueError("path follower distances must be positive")
        if sample_count < 10:
            raise ValueError("sample_count must be at least 10")
        self.path = path
        self.lookahead_m = lookahead_m
        self.completion_radius_m = completion_radius_m
        self.completion_heading_tolerance_rad = completion_heading_tolerance_rad
        self._points = [
            path.point(index / (sample_count - 1)) for index in range(sample_count)
        ]
        self._distances = [0.0]
        for first, second in zip(self._points, self._points[1:]):
            self._distances.append(
                self._distances[-1]
                + math.hypot(second[0] - first[0], second[1] - first[1])
            )
        self._index = 0

    @property
    def progress_ratio(self) -> float:
        return self._index / (len(self._points) - 1)

    @property
    def remaining_distance_m(self) -> float:
        return max(0.0, self._distances[-1] - self._distances[self._index])

    def _update_progress(self, pose_x_m: float, pose_y_m: float) -> None:
        # Never move progress backwards; this prevents a noisy pose estimate
        # from making the steering target jump between earlier path sections.
        self._index = min(
            range(self._index, len(self._points)),
            key=lambda index: (
                (self._points[index][0] - pose_x_m) ** 2
                + (self._points[index][1] - pose_y_m) ** 2
            ),
        )

    def heading_error_rad(
        self, *, pose_x_m: float, pose_y_m: float, pose_yaw_rad: float
    ) -> float:
        self._update_progress(pose_x_m, pose_y_m)
        target_distance = self._distances[self._index] + self.lookahead_m
        target_index = self._index
        while (
            target_index < len(self._points) - 1
            and self._distances[target_index] < target_distance
        ):
            target_index += 1
        target_x, target_y = self._points[target_index]
        if target_index == len(self._points) - 1:
            tangent_x = self.path.p3[0] - self.path.p2[0]
            tangent_y = self.path.p3[1] - self.path.p2[1]
            tangent_length = max(1e-6, math.hypot(tangent_x, tangent_y))
            # Follow a virtual point beyond the endpoint so the robot becomes
            # parallel to the passing/original path instead of leaving the
            # curve with a large residual yaw angle.
            target_x += self.lookahead_m * tangent_x / tangent_length
            target_y += self.lookahead_m * tangent_y / tangent_length
        dx, dy = target_x - pose_x_m, target_y - pose_y_m
        if math.hypot(dx, dy) < 1e-6:
            tangent_x, tangent_y = self.path.tangent(
                target_index / (len(self._points) - 1)
            )
            desired_heading = math.atan2(tangent_y, tangent_x)
        else:
            desired_heading = math.atan2(dy, dx)
        return wrap_angle(desired_heading - pose_yaw_rad)

    def completed(
        self,
        *,
        pose_x_m: float,
        pose_y_m: float,
        pose_yaw_rad: float | None = None,
    ) -> bool:
        self._update_progress(pose_x_m, pose_y_m)
        tangent_x = self.path.p3[0] - self.path.p2[0]
        tangent_y = self.path.p3[1] - self.path.p2[1]
        tangent_length = max(1e-6, math.hypot(tangent_x, tangent_y))
        unit_x, unit_y = tangent_x / tangent_length, tangent_y / tangent_length
        relative_x = pose_x_m - self.path.p3[0]
        relative_y = pose_y_m - self.path.p3[1]
        along_track = relative_x * unit_x + relative_y * unit_y
        cross_track = abs(-relative_x * unit_y + relative_y * unit_x)
        heading_ok = True
        if pose_yaw_rad is not None:
            goal_heading = math.atan2(unit_y, unit_x)
            heading_ok = bool(
                abs(wrap_angle(goal_heading - pose_yaw_rad))
                <= self.completion_heading_tolerance_rad
            )
        return bool(
            along_track >= -self.completion_radius_m
            and cross_track <= self.completion_radius_m
            and heading_ok
        )


def bbox_iou(
    first: Sequence[float], second: Sequence[float]
) -> float:
    left = max(first[0], second[0])
    top = max(first[1], second[1])
    right = min(first[2], second[2])
    bottom = min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(
        0.0, second[3] - second[1]
    )
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def pixel_to_bearing_rad(pixel_x: float, config: FusionConfig) -> float:
    """Return robot-frame bearing: left is positive, right is negative."""

    center = config.image_width_px * 0.5
    half_fov = math.radians(config.camera_horizontal_fov_deg) * 0.5
    focal_x = center / math.tan(half_fov)
    camera_bearing = -math.atan((float(pixel_x) - center) / focal_x)
    return camera_bearing + math.radians(config.camera_lidar_yaw_offset_deg)


def fuse_confirmed_tracks(
    tracks: Iterable[TrackedDetection],
    lidar_points: Iterable[tuple[float, float]],
    config: FusionConfig,
) -> list[FusedObject]:
    """Associate confirmed YOLO boxes with coherent LiDAR point clusters."""

    points = sorted(
        (
            (float(angle), float(distance))
            for angle, distance in lidar_points
            if math.isfinite(angle)
            and math.isfinite(distance)
            and distance > 0.0
        ),
        key=lambda item: item[0],
    )
    result: list[FusedObject] = []
    angular_padding = math.radians(config.bbox_angular_padding_deg)
    for track in tracks:
        x1, _y1, x2, _y2 = track.bbox
        first_bearing = pixel_to_bearing_rad(x1, config)
        second_bearing = pixel_to_bearing_rad(x2, config)
        bearing_min = min(first_bearing, second_bearing) - angular_padding
        bearing_max = max(first_bearing, second_bearing) + angular_padding
        box_center_bearing = 0.5 * (first_bearing + second_bearing)
        sector_points = [
            point for point in points if bearing_min <= point[0] <= bearing_max
        ]
        clusters = _lidar_clusters(sector_points, config)
        if not clusters:
            continue
        cluster = min(
            clusters,
            key=lambda items: (
                abs(statistics.median(item[0] for item in items) - box_center_bearing),
                statistics.median(item[1] for item in items),
            ),
        )
        distance = float(statistics.median(item[1] for item in cluster))
        bearing = float(statistics.median(item[0] for item in cluster))
        forward = distance * math.cos(bearing)
        if forward <= 0.0:
            continue
        lateral_edges = (
            forward * math.tan(bearing_min),
            forward * math.tan(bearing_max),
        )
        lateral_min = min(lateral_edges)
        lateral_max = max(lateral_edges)
        corridor = config.corridor_half_width_m
        intersects = lateral_max >= -corridor and lateral_min <= corridor
        result.append(
            FusedObject(
                track_id=track.track_id,
                label=track.label,
                confidence=track.confidence,
                confirmed_frames=track.consecutive_hits,
                distance_m=distance,
                forward_distance_m=forward,
                bearing_rad=bearing,
                bearing_min_rad=bearing_min,
                bearing_max_rad=bearing_max,
                lateral_min_m=lateral_min,
                lateral_max_m=lateral_max,
                intersects_corridor=intersects,
            )
        )
    return result


def select_priority_target(
    objects: Iterable[FusedObject], config: FusionConfig
) -> FusedObject | None:
    """Choose the nearest object that actually intersects the drive corridor."""

    candidates = [
        item
        for item in objects
        if item.intersects_corridor
        and item.forward_distance_m <= config.avoidance_trigger_distance_m
    ]
    return min(candidates, key=lambda item: item.distance_m, default=None)


def clearance_bearing_rad(
    target: FusedObject, *, turn_left: bool, corridor_half_width_m: float
) -> float:
    """Bearing just outside the target, expanded for robot width and margin."""

    padding = math.atan2(
        corridor_half_width_m,
        max(0.05, target.forward_distance_m),
    )
    return (
        target.bearing_max_rad + padding
        if turn_left
        else target.bearing_min_rad - padding
    )


def passing_lateral_offset_m(
    target: FusedObject, *, turn_left: bool, corridor_half_width_m: float
) -> float:
    """Robot-centre offset that clears the frozen obstacle boundary."""

    if corridor_half_width_m <= 0.0:
        raise ValueError("corridor_half_width_m must be positive")
    return (
        target.lateral_max_m + corridor_half_width_m
        if turn_left
        else target.lateral_min_m - corridor_half_width_m
    )


def local_point_to_world(
    *,
    origin_x_m: float,
    origin_y_m: float,
    origin_yaw_rad: float,
    forward_m: float,
    lateral_m: float,
) -> tuple[float, float]:
    cosine = math.cos(origin_yaw_rad)
    sine = math.sin(origin_yaw_rad)
    return (
        origin_x_m + cosine * forward_m - sine * lateral_m,
        origin_y_m + sine * forward_m + cosine * lateral_m,
    )


def differential_rpm_command(
    base_rpm: float,
    heading_error_rad: float,
    *,
    gain_rpm_per_rad: float,
    maximum_delta_rpm: float,
    maximum_rpm: int = 65,
) -> tuple[int, int]:
    """Map positive-left heading error to differential wheel RPM."""

    delta = max(
        -maximum_delta_rpm,
        min(maximum_delta_rpm, gain_rpm_per_rad * heading_error_rad),
    )
    left = round(base_rpm - delta)
    right = round(base_rpm + delta)
    return (
        max(0, min(maximum_rpm, left)),
        max(0, min(maximum_rpm, right)),
    )


def return_heading_error_rad(
    *, pose_y_m: float, pose_yaw_rad: float, lookahead_m: float
) -> float:
    return line_heading_error_rad(
        pose_y_m=pose_y_m,
        target_y_m=0.0,
        pose_yaw_rad=pose_yaw_rad,
        lookahead_m=lookahead_m,
    )


def line_heading_error_rad(
    *,
    pose_y_m: float,
    target_y_m: float,
    pose_yaw_rad: float,
    lookahead_m: float,
) -> float:
    if lookahead_m <= 0:
        raise ValueError("lookahead_m must be positive")
    desired_heading = math.atan2(target_y_m - pose_y_m, lookahead_m)
    return wrap_angle(desired_heading - pose_yaw_rad)


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _valid_bbox(bbox: Sequence[float]) -> bool:
    return (
        len(bbox) == 4
        and all(math.isfinite(value) for value in bbox)
        and bbox[2] > bbox[0]
        and bbox[3] > bbox[1]
    )


def _lidar_clusters(
    points: list[tuple[float, float]], config: FusionConfig
) -> list[list[tuple[float, float]]]:
    if not points:
        return []
    maximum_angle_gap = math.radians(config.lidar_cluster_angle_gap_deg)
    clusters: list[list[tuple[float, float]]] = [[points[0]]]
    for point in points[1:]:
        previous = clusters[-1][-1]
        if (
            point[0] - previous[0] <= maximum_angle_gap
            and abs(point[1] - previous[1])
            <= config.lidar_cluster_distance_gap_m
        ):
            clusters[-1].append(point)
        else:
            clusters.append([point])
    return [
        cluster
        for cluster in clusters
        if len(cluster) >= config.minimum_cluster_points
    ]
