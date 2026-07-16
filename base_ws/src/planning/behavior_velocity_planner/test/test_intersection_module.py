import unittest

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
)

from behavior_velocity_intersection_module import (
    IntersectionModuleManager,
    IntersectionParameters,
)
from behavior_velocity_planner_common import BehaviorVelocityMap, MapRoadLanelet, PlannerData
from behavior_velocity_planner_common.path_utils import to_path


def road_lanelet(lanelet_id, polygon, centerline, tags=None, previous=(), following=()):
    half = len(polygon) // 2
    return MapRoadLanelet(
        lanelet_id=lanelet_id,
        left_bound=tuple(polygon[:half]),
        right_bound=tuple(reversed(polygon[half:])),
        centerline=tuple(centerline),
        polygon=tuple(polygon),
        tags=dict(tags or {"type": "lanelet", "subtype": "road"}),
        previous_ids=tuple(previous),
        following_ids=tuple(following),
    )


def synthetic_map(turn_direction="straight"):
    before = road_lanelet(
        1,
        ((-5.0, 2.0, 0.0), (20.0, 2.0, 0.0), (20.0, -2.0, 0.0), (-5.0, -2.0, 0.0)),
        ((-5.0, 0.0, 0.0), (20.0, 0.0, 0.0)),
        previous=(),
        following=(10,),
    )
    target = road_lanelet(
        10,
        ((20.0, 2.0, 0.0), (40.0, 2.0, 0.0), (40.0, -2.0, 0.0), (20.0, -2.0, 0.0)),
        ((20.0, 0.0, 0.0), (40.0, 0.0, 0.0)),
        tags={
            "type": "lanelet",
            "subtype": "road",
            "turn_direction": turn_direction,
        },
        previous=(1,),
        following=(2,),
    )
    after = road_lanelet(
        2,
        ((40.0, 2.0, 0.0), (65.0, 2.0, 0.0), (65.0, -2.0, 0.0), (40.0, -2.0, 0.0)),
        ((40.0, 0.0, 0.0), (65.0, 0.0, 0.0)),
        previous=(10,),
    )
    conflict = road_lanelet(
        20,
        ((28.0, -20.0, 0.0), (28.0, 20.0, 0.0), (32.0, 20.0, 0.0), (32.0, -20.0, 0.0)),
        ((30.0, -20.0, 0.0), (30.0, 20.0, 0.0)),
    )
    return BehaviorVelocityMap(
        "synthetic_intersection",
        road_lanelets={
            item.lanelet_id: item for item in (before, target, after, conflict)
        },
    )


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in range(0, 61, 2):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        point.point.pose.orientation.w = 1.0
        point.point.longitudinal_velocity_mps = 10.0
        if x < 20:
            point.lane_ids = [1]
        elif x <= 40:
            point.lane_ids = [10]
        else:
            point.lane_ids = [2]
        path.points.append(point)
    return path


def make_odometry(x=0.0, speed=0.0):
    odometry = Odometry()
    odometry.pose.pose.position.x = float(x)
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = float(speed)
    return odometry


def make_vehicle(speed=2.0, start_y=-16.0, label=ObjectClassification.CAR):
    detected = PredictedObject()
    detected.object_id.uuid = list(range(16))
    detected.existence_probability = 1.0
    classification = ObjectClassification()
    classification.label = int(label)
    classification.probability = 1.0
    detected.classification = [classification]
    pose = detected.kinematics.initial_pose_with_covariance.pose
    pose.position.x = 30.0
    pose.position.y = float(start_y)
    pose.orientation.z = 0.70710678
    pose.orientation.w = 0.70710678
    detected.kinematics.initial_twist_with_covariance.twist.linear.x = float(speed)
    detected.shape.dimensions.x = 4.5
    detected.shape.dimensions.y = 1.8
    detected.shape.dimensions.z = 1.6
    prediction = PredictedPath()
    prediction.confidence = 1.0
    prediction.time_step = rospy.Duration.from_sec(0.5)
    for step in range(41):
        sample = Pose()
        sample.position.x = 30.0
        sample.position.y = float(start_y) + float(speed) * 0.5 * step
        sample.orientation = pose.orientation
        prediction.path.append(sample)
    detected.kinematics.predicted_paths = [prediction]
    message = PredictedObjects()
    message.header.frame_id = "map"
    message.objects = [detected]
    return message


def make_stationary(x, y):
    message = make_vehicle(speed=0.0, start_y=y)
    detected = message.objects[0]
    detected.kinematics.initial_pose_with_covariance.pose.position.x = float(x)
    for pose in detected.kinematics.predicted_paths[0].path:
        pose.position.x = float(x)
        pose.position.y = float(y)
    return message


def make_occupancy_grid(x=30.0, y=5.0, value=50):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = 1.0
    grid.info.width = 80
    grid.info.height = 60
    grid.info.origin.position.x = -10.0
    grid.info.origin.position.y = -30.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (grid.info.width * grid.info.height)
    column = int((x - grid.info.origin.position.x) / grid.info.resolution)
    row = int((y - grid.info.origin.position.y) / grid.info.resolution)
    # Autoware treats values between free_space_max and occupied_min as
    # unobserved/occluded cells. Occupied cells (>= 50) are not occlusion masks.
    grid.data[row * grid.info.width + column] = int(value)
    return grid


class IntersectionModuleTest(unittest.TestCase):
    def run_module(
        self,
        lane_map=None,
        objects=None,
        odometry=None,
        occupancy_grid=None,
        current_time=10.0,
        manager=None,
    ):
        input_path = make_path()
        output_path = to_path(input_path)
        manager = manager or IntersectionModuleManager(IntersectionParameters())
        result = manager.modify_path_velocity(
            input_path,
            output_path,
            PlannerData(
                lanelet_map=lane_map or synthetic_map(),
                odometry=odometry or make_odometry(),
                dynamic_objects=objects,
                occupancy_grid=occupancy_grid,
                current_time=current_time,
            ),
        )
        return result, output_path, manager

    def test_map_conflict_and_turn_direction_activate_module(self):
        lane_map = synthetic_map()

        self.assertEqual(
            [item.lanelet_id for item in lane_map.intersection_lanelet_candidates({10})],
            [10],
        )
        self.assertEqual(
            [item.lanelet_id for item in lane_map.conflicting_road_lanelets(10)],
            [20],
        )

    def test_clear_intersection_keeps_velocity(self):
        result, output, _ = self.run_module()

        self.assertFalse(result.modified)
        self.assertIn("10:Safe", result.status)
        self.assertTrue(all(point.longitudinal_velocity_mps == 10.0 for point in output.points))

    def test_temporal_collision_inserts_stop(self):
        result, output, _ = self.run_module(objects=make_vehicle(speed=2.0))

        self.assertTrue(result.modified)
        self.assertIn("NonOccludedCollisionStop", result.status)
        self.assertIn("collision stop", result.planning_factors[0].detail)
        self.assertEqual(result.stop_reasons[0].reason, "Intersection")
        self.assertTrue(any(point.longitudinal_velocity_mps == 0.0 for point in output.points))

    def test_same_geometry_with_early_object_is_timing_clear(self):
        result, _, _ = self.run_module(objects=make_vehicle(speed=8.0))

        self.assertFalse(result.modified)
        self.assertIn("10:Safe", result.status)

    def test_stuck_vehicle_on_ego_path_stops(self):
        result, _, _ = self.run_module(objects=make_stationary(35.0, 0.0))

        self.assertTrue(result.modified)
        self.assertIn("StuckStop", result.status)
        self.assertIn("stuck stop", result.planning_factors[0].detail)

    def test_yield_stuck_is_disabled_for_straight_and_enabled_for_left(self):
        detected = make_stationary(30.0, -3.0)
        straight, _, _ = self.run_module(objects=detected)
        left, _, _ = self.run_module(lane_map=synthetic_map("left"), objects=detected)

        self.assertFalse(straight.modified)
        self.assertIn("10:Safe", straight.status)
        self.assertTrue(left.modified)
        self.assertIn("YieldStuckStop", left.status)

    def test_pedestrian_is_not_collision_target(self):
        result, _, _ = self.run_module(
            objects=make_vehicle(speed=2.0, label=ObjectClassification.PEDESTRIAN)
        )

        self.assertFalse(result.modified)

    def test_ego_after_stop_line_returns_over_pass_judge_line(self):
        result, _, _ = self.run_module(
            objects=make_vehicle(speed=2.0), odometry=make_odometry(x=30.0)
        )

        self.assertFalse(result.modified)
        self.assertIn("OverPassJudgeLine", result.status)

    def test_occlusion_first_wait_and_peeking_states(self):
        parameters = IntersectionParameters(
            occlusion_detection_hold_time=0.0,
            temporal_stop_time_before_peeking=1.0,
        )
        manager = IntersectionModuleManager(parameters)
        first, _, manager = self.run_module(
            occupancy_grid=make_occupancy_grid(), manager=manager, current_time=10.0
        )
        stop_station = first.planning_factors[0].control_points[0].pose.position.x
        second, _, _ = self.run_module(
            occupancy_grid=make_occupancy_grid(),
            odometry=make_odometry(x=stop_station),
            manager=manager,
            current_time=12.0,
        )
        third, _, _ = self.run_module(
            occupancy_grid=make_occupancy_grid(),
            odometry=make_odometry(x=stop_station),
            manager=manager,
            current_time=13.5,
        )

        self.assertIn("FirstWaitBeforeOcclusion", first.status)
        self.assertIn("FirstWaitBeforeOcclusion", second.status)
        self.assertIn("PeekingTowardOcclusion", third.status)

    def test_occupied_cell_is_not_treated_as_unknown_occlusion(self):
        parameters = IntersectionParameters(occlusion_detection_hold_time=0.0)
        result, _, _ = self.run_module(
            occupancy_grid=make_occupancy_grid(value=100),
            manager=IntersectionModuleManager(parameters),
        )

        self.assertFalse(result.modified)
        self.assertIn("10:Safe", result.status)


if __name__ == "__main__":
    unittest.main()
