import math
import unittest

from jetson.amr_core.reactive_avoidance import (
    BezierPathFollower,
    CubicBezierPath,
    DetectionTracker,
    FusedObject,
    FusionConfig,
    WheelOdometry,
    clearance_bearing_rad,
    differential_rpm_command,
    fuse_confirmed_tracks,
    line_heading_error_rad,
    local_point_to_world,
    passing_lateral_offset_m,
    pixel_to_bearing_rad,
    return_heading_error_rad,
    select_priority_target,
)


def detection(label="person", confidence=0.9, bbox=(280, 80, 360, 360)):
    return {
        "class": label,
        "confidence": confidence,
        "xyxy": list(bbox),
    }


def fused_object(
    *,
    track_id=1,
    distance=1.0,
    forward=1.0,
    bearing=0.0,
    lateral_min=-0.1,
    lateral_max=0.1,
    intersects=True,
):
    return FusedObject(
        track_id=track_id,
        label="person",
        confidence=0.9,
        confirmed_frames=3,
        distance_m=distance,
        forward_distance_m=forward,
        bearing_rad=bearing,
        bearing_min_rad=bearing - 0.05,
        bearing_max_rad=bearing + 0.05,
        lateral_min_m=lateral_min,
        lateral_max_m=lateral_max,
        intersects_corridor=intersects,
    )


class DetectionTrackerTests(unittest.TestCase):
    def test_detection_requires_configured_consecutive_frames(self):
        tracker = DetectionTracker(required_frames=3, iou_threshold=0.3)
        tracker.update([detection()])
        self.assertEqual(tracker.confirmed_visible(), ())
        tracker.update([detection(bbox=(282, 82, 362, 362))])
        self.assertEqual(tracker.confirmed_visible(), ())
        tracker.update([detection(bbox=(284, 84, 364, 364))])
        confirmed = tracker.confirmed_visible()
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].consecutive_hits, 3)

    def test_missing_frame_removes_confirmation(self):
        tracker = DetectionTracker(required_frames=2, maximum_missed_frames=2)
        tracker.update([detection()])
        tracker.update([detection()])
        self.assertEqual(len(tracker.confirmed_visible()), 1)
        tracker.update([])
        self.assertEqual(tracker.confirmed_visible(), ())

    def test_low_confidence_detection_is_not_tracked(self):
        tracker = DetectionTracker(minimum_confidence=0.45, required_frames=1)
        tracker.update([detection(confidence=0.2)])
        self.assertEqual(tracker.confirmed_visible(), ())


class CameraLidarFusionTests(unittest.TestCase):
    def setUp(self):
        self.config = FusionConfig(
            image_width_px=640,
            camera_horizontal_fov_deg=60,
            robot_width_m=0.5,
            safety_margin_m=0.1,
            minimum_cluster_points=2,
            lidar_cluster_angle_gap_deg=3.0,
        )

    def test_pixel_bearing_uses_left_positive_robot_convention(self):
        self.assertGreater(pixel_to_bearing_rad(100, self.config), 0.0)
        self.assertAlmostEqual(pixel_to_bearing_rad(320, self.config), 0.0)
        self.assertLess(pixel_to_bearing_rad(540, self.config), 0.0)

    def test_yaw_offset_is_applied(self):
        config = FusionConfig(camera_lidar_yaw_offset_deg=5.0)
        self.assertAlmostEqual(
            pixel_to_bearing_rad(320, config), math.radians(5.0), places=6
        )

    def test_center_box_is_fused_with_lidar_and_intersects_corridor(self):
        tracker = DetectionTracker(required_frames=1)
        tracker.update([detection()])
        points = [
            (math.radians(-2), 1.01),
            (math.radians(0), 1.00),
            (math.radians(2), 0.99),
        ]
        objects = fuse_confirmed_tracks(
            tracker.confirmed_visible(), points, self.config
        )
        self.assertEqual(len(objects), 1)
        self.assertAlmostEqual(objects[0].distance_m, 1.0, places=2)
        self.assertTrue(objects[0].intersects_corridor)

    def test_object_outside_drive_corridor_is_not_selected(self):
        tracker = DetectionTracker(required_frames=1)
        tracker.update([detection(bbox=(0, 80, 70, 360))])
        points = [
            (math.radians(25), 1.5),
            (math.radians(27), 1.5),
            (math.radians(29), 1.5),
        ]
        objects = fuse_confirmed_tracks(
            tracker.confirmed_visible(), points, self.config
        )
        self.assertEqual(len(objects), 1)
        self.assertFalse(objects[0].intersects_corridor)
        self.assertIsNone(select_priority_target(objects, self.config))

    def test_nearest_intruding_object_has_priority_over_nearer_side_object(self):
        side = fused_object(
            track_id=1,
            distance=0.5,
            forward=0.45,
            lateral_min=0.7,
            lateral_max=0.9,
            intersects=False,
        )
        front_far = fused_object(track_id=2, distance=1.4, forward=1.4)
        front_near = fused_object(track_id=3, distance=0.9, forward=0.9)
        target = select_priority_target(
            [side, front_far, front_near], self.config
        )
        self.assertEqual(target.track_id, 3)

    def test_intruding_object_outside_trigger_distance_is_ignored(self):
        distant = fused_object(distance=2.5, forward=2.5)
        self.assertIsNone(select_priority_target([distant], self.config))


class SteeringAndOdometryTests(unittest.TestCase):
    def test_clearance_bearing_has_requested_direction(self):
        target = fused_object()
        corridor = 0.4
        self.assertGreater(
            clearance_bearing_rad(
                target, turn_left=True, corridor_half_width_m=corridor
            ),
            0.0,
        )
        self.assertLess(
            clearance_bearing_rad(
                target, turn_left=False, corridor_half_width_m=corridor
            ),
            0.0,
        )

    def test_positive_heading_error_turns_left(self):
        left, right = differential_rpm_command(
            12,
            math.radians(30),
            gain_rpm_per_rad=18,
            maximum_delta_rpm=10,
        )
        self.assertLess(left, right)

    def test_wheel_odometry_tracks_straight_and_turning_motion(self):
        odometry = WheelOdometry(wheel_diameter_m=0.2, wheel_base_m=0.5)
        odometry.update(20, 20, 0.5)
        self.assertGreater(odometry.x_m, 0.0)
        self.assertAlmostEqual(odometry.y_m, 0.0, places=6)
        self.assertAlmostEqual(odometry.yaw_rad, 0.0, places=6)
        odometry.update(5, 20, 0.5)
        self.assertGreater(odometry.yaw_rad, 0.0)
        self.assertGreater(odometry.y_m, 0.0)

    def test_return_controller_points_toward_original_path(self):
        # Positive y is left of the original path, so return must steer right.
        error = return_heading_error_rad(
            pose_y_m=0.5, pose_yaw_rad=0.0, lookahead_m=0.8
        )
        self.assertLess(error, 0.0)

    def test_passing_offset_clears_frozen_object_boundary(self):
        target = fused_object(lateral_min=-0.2, lateral_max=0.1)
        self.assertAlmostEqual(
            passing_lateral_offset_m(
                target, turn_left=True, corridor_half_width_m=0.4
            ),
            0.5,
        )
        self.assertAlmostEqual(
            passing_lateral_offset_m(
                target, turn_left=False, corridor_half_width_m=0.4
            ),
            -0.6,
        )

    def test_line_controller_holds_selected_passing_lane(self):
        self.assertGreater(
            line_heading_error_rad(
                pose_y_m=0.2,
                target_y_m=0.5,
                pose_yaw_rad=0.0,
                lookahead_m=0.8,
            ),
            0.0,
        )


class SmoothPathTests(unittest.TestCase):
    def setUp(self):
        self.path = CubicBezierPath.from_poses(
            start_x_m=0.0,
            start_y_m=0.0,
            start_yaw_rad=0.0,
            goal_x_m=1.5,
            goal_y_m=0.5,
            goal_yaw_rad=0.0,
        )

    def test_bezier_path_preserves_start_and_end_pose_directions(self):
        self.assertEqual(self.path.point(0.0), (0.0, 0.0))
        self.assertEqual(self.path.point(1.0), (1.5, 0.5))
        start_tangent = self.path.tangent(0.0)
        end_tangent = self.path.tangent(1.0)
        self.assertAlmostEqual(start_tangent[1], 0.0, places=6)
        self.assertAlmostEqual(end_tangent[1], 0.0, places=6)
        self.assertGreater(start_tangent[0], 0.0)
        self.assertGreater(end_tangent[0], 0.0)

    def test_path_follower_progress_never_moves_backwards(self):
        follower = BezierPathFollower(self.path)
        follower.heading_error_rad(
            pose_x_m=0.8, pose_y_m=0.25, pose_yaw_rad=0.0
        )
        progressed = follower.progress_ratio
        follower.heading_error_rad(
            pose_x_m=0.1, pose_y_m=0.0, pose_yaw_rad=0.0
        )
        self.assertGreaterEqual(follower.progress_ratio, progressed)

    def test_path_only_completes_near_endpoint_and_aligned(self):
        follower = BezierPathFollower(self.path)
        self.assertFalse(
            follower.completed(
                pose_x_m=1.5,
                pose_y_m=0.5,
                pose_yaw_rad=math.radians(15.0),
            )
        )
        self.assertTrue(
            follower.completed(
                pose_x_m=1.5,
                pose_y_m=0.5,
                pose_yaw_rad=0.0,
            )
        )

    def test_local_point_transform_uses_robot_heading(self):
        world = local_point_to_world(
            origin_x_m=1.0,
            origin_y_m=2.0,
            origin_yaw_rad=math.pi / 2.0,
            forward_m=1.0,
            lateral_m=0.5,
        )
        self.assertAlmostEqual(world[0], 0.5, places=6)
        self.assertAlmostEqual(world[1], 3.0, places=6)


class ImprovedAvoidanceScenarioTests(unittest.TestCase):
    def test_confirm_fuse_prioritize_and_generate_avoidance_command(self):
        config = FusionConfig(
            camera_horizontal_fov_deg=60,
            robot_width_m=0.5,
            safety_margin_m=0.15,
            minimum_cluster_points=2,
            lidar_cluster_angle_gap_deg=3.0,
        )
        tracker = DetectionTracker(required_frames=3)
        for _ in range(3):
            tracker.update([detection()])
        points = [
            (math.radians(-2), 1.1),
            (math.radians(0), 1.1),
            (math.radians(2), 1.1),
        ]
        objects = fuse_confirmed_tracks(
            tracker.confirmed_visible(), points, config
        )
        target = select_priority_target(objects, config)
        self.assertIsNotNone(target)
        desired = clearance_bearing_rad(
            target, turn_left=True, corridor_half_width_m=config.corridor_half_width_m
        )
        left, right = differential_rpm_command(
            12,
            desired,
            gain_rpm_per_rad=18,
            maximum_delta_rpm=10,
        )
        self.assertLess(left, right)


if __name__ == "__main__":
    unittest.main()
