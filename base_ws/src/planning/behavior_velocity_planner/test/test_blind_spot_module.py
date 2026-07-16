import unittest

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
)

from behavior_velocity_blind_spot_module import (
    BlindSpotModuleManager,
    BlindSpotParameters,
)
from behavior_velocity_planner_common import BehaviorVelocityMap, MapRoadLanelet, PlannerData
from behavior_velocity_planner_common.path_utils import to_path


def road_lanelet(lanelet_id, x0, x1, tags=None, previous=(), following=()):
    return MapRoadLanelet(
        lanelet_id=lanelet_id,
        left_bound=((x0, 2.0, 0.0), (x1, 2.0, 0.0)),
        right_bound=((x0, -2.0, 0.0), (x1, -2.0, 0.0)),
        centerline=((x0, 0.0, 0.0), (x1, 0.0, 0.0)),
        polygon=(
            (x0, 2.0, 0.0),
            (x1, 2.0, 0.0),
            (x1, -2.0, 0.0),
            (x0, -2.0, 0.0),
        ),
        tags=dict(tags or {"type": "lanelet", "subtype": "road"}),
        previous_ids=tuple(previous),
        following_ids=tuple(following),
    )


def synthetic_map(turn_direction="left"):
    before = road_lanelet(1, 0.0, 20.0, following=(10,))
    target = road_lanelet(
        10,
        20.0,
        40.0,
        tags={
            "type": "lanelet",
            "subtype": "road",
            "turn_direction": turn_direction,
        },
        previous=(1,),
        following=(2,),
    )
    after = road_lanelet(2, 40.0, 60.0, previous=(10,))
    return BehaviorVelocityMap(
        "synthetic_blind_spot",
        road_lanelets={item.lanelet_id: item for item in (before, target, after)},
    )


def make_path():
    path = PathWithLaneId()
    path.header.frame_id = "map"
    for x in range(0, 61, 2):
        item = PathPointWithLaneId()
        item.point.pose.position.x = float(x)
        item.point.pose.orientation.w = 1.0
        item.point.longitudinal_velocity_mps = 10.0
        if x < 20:
            item.lane_ids = [1]
        elif x <= 40:
            item.lane_ids = [10]
        else:
            item.lane_ids = [2]
        path.points.append(item)
    return path


def make_odometry(x=0.0, speed=0.0):
    odometry = Odometry()
    odometry.pose.pose.position.x = float(x)
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = float(speed)
    return odometry


def make_object(
    turn_direction="left",
    label=ObjectClassification.BICYCLE,
    initial_lateral=None,
    time_step=0.5,
):
    sign = 1.0 if turn_direction == "left" else -1.0
    start_lateral = sign * 1.4 if initial_lateral is None else float(initial_lateral)
    detected = PredictedObject()
    detected.object_id.uuid = list(range(16))
    detected.existence_probability = 1.0
    classification = ObjectClassification()
    classification.label = int(label)
    classification.probability = 1.0
    detected.classification = [classification]
    detected.shape.dimensions.x = 1.8
    detected.shape.dimensions.y = 0.6
    detected.shape.dimensions.z = 1.6
    detected.kinematics.initial_pose_with_covariance.pose.position.x = 8.0
    detected.kinematics.initial_pose_with_covariance.pose.position.y = start_lateral
    detected.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
    detected.kinematics.initial_twist_with_covariance.twist.linear.x = 10.0

    prediction = PredictedPath()
    prediction.confidence = 1.0
    prediction.time_step = rospy.Duration.from_sec(float(time_step))
    for index in range(8):
        pose = Pose()
        pose.position.x = 8.0 + 5.0 * index
        pose.position.y = start_lateral - sign * 0.8 * index
        pose.orientation.w = 1.0
        prediction.path.append(pose)
    detected.kinematics.predicted_paths = [prediction]
    message = PredictedObjects()
    message.header.frame_id = "map"
    message.objects = [detected]
    return message


class BlindSpotModuleTest(unittest.TestCase):
    def run_module(
        self,
        turn_direction="left",
        objects=None,
        odometry=None,
        manager=None,
        current_time=10.0,
    ):
        input_path = make_path()
        output_path = to_path(input_path)
        manager = manager or BlindSpotModuleManager(BlindSpotParameters())
        result = manager.modify_path_velocity(
            input_path,
            output_path,
            PlannerData(
                lanelet_map=synthetic_map(turn_direction),
                odometry=odometry or make_odometry(),
                dynamic_objects=objects,
                current_time=float(current_time),
            ),
        )
        return result, output_path, manager

    def test_clear_turn_is_safe(self):
        result, output, _ = self.run_module()

        self.assertFalse(result.modified)
        self.assertIn("10:Safe", result.status)
        self.assertTrue(all(point.longitudinal_velocity_mps == 10.0 for point in output.points))

    def test_left_bicycle_ttc_conflict_stops(self):
        result, output, _ = self.run_module(objects=make_object())

        self.assertTrue(result.modified)
        self.assertIn("10:Unsafe", result.status)
        self.assertEqual(result.stop_reasons[0].reason, "BlindSpot")
        self.assertEqual(result.planning_factors[0].module, "blind_spot")
        self.assertEqual(result.planning_factors[0].safety_factors.factors[0].type, 2)
        self.assertTrue(any(point.longitudinal_velocity_mps == 0.0 for point in output.points))

    def test_right_turn_is_mirrored(self):
        result, _, _ = self.run_module(
            turn_direction="right", objects=make_object("right")
        )

        self.assertTrue(result.modified)
        self.assertIn("10:Unsafe", result.status)

    def test_fast_crossing_clears_before_ego_arrival(self):
        result, _, _ = self.run_module(objects=make_object(time_step=0.1))

        self.assertFalse(result.modified)
        self.assertIn("10:Safe", result.status)

    def test_car_is_ignored_by_default(self):
        result, _, _ = self.run_module(
            objects=make_object(label=ObjectClassification.CAR)
        )

        self.assertFalse(result.modified)
        self.assertIn("10:Safe", result.status)

    def test_object_outside_attention_area_is_ignored(self):
        result, _, _ = self.run_module(objects=make_object(initial_lateral=4.0))

        self.assertFalse(result.modified)
        self.assertIn("10:Safe", result.status)

    def test_stop_to_go_is_debounced(self):
        manager = BlindSpotModuleManager(BlindSpotParameters())
        unsafe, _, manager = self.run_module(
            objects=make_object(), manager=manager, current_time=10.0
        )
        held, _, manager = self.run_module(
            manager=manager, current_time=10.2
        )
        released, _, _ = self.run_module(
            manager=manager, current_time=10.8
        )

        self.assertTrue(unsafe.modified)
        self.assertTrue(held.modified)
        self.assertIn("collision debounce hold", held.planning_factors[0].detail)
        self.assertFalse(released.modified)
        self.assertIn("10:Safe", released.status)

    def test_unsafe_after_pass_judge_does_not_command_rapid_stop(self):
        result, _, _ = self.run_module(
            objects=make_object(), odometry=make_odometry(x=20.0, speed=8.0)
        )

        self.assertFalse(result.modified)
        self.assertIn("10:OverPassJudge", result.status)


if __name__ == "__main__":
    unittest.main()
