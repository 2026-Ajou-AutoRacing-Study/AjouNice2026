import unittest

from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId

from behavior_velocity_merge_from_private_module import (
    MergeFromPrivateModuleManager,
    MergeFromPrivateParameters,
)
from behavior_velocity_planner_common import BehaviorVelocityMap, MapRoadLanelet, PlannerData
from behavior_velocity_planner_common.path_utils import to_path


def lane(lane_id, x0, x1, location):
    return MapRoadLanelet(
        lane_id,
        ((x0, 2.0, 0.0), (x1, 2.0, 0.0)),
        ((x0, -2.0, 0.0), (x1, -2.0, 0.0)),
        ((x0, 0.0, 0.0), (x1, 0.0, 0.0)),
        ((x0, 2.0, 0.0), (x1, 2.0, 0.0), (x1, -2.0, 0.0), (x0, -2.0, 0.0)),
        {"type": "lanelet", "subtype": "road", "location": location},
    )


def input_path():
    message = PathWithLaneId()
    message.header.frame_id = "map"
    for x in range(0, 61, 2):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        point.point.pose.orientation.w = 1.0
        point.point.longitudinal_velocity_mps = 5.0
        point.lane_ids = [1 if x <= 30 else 2]
        message.points.append(point)
    return message


class MergeFromPrivateTest(unittest.TestCase):
    def test_private_to_public_requires_stop_and_releases_after_duration(self):
        map_data = BehaviorVelocityMap(
            "synthetic", road_lanelets={1: lane(1, 0.0, 30.0, "private"), 2: lane(2, 30.0, 60.0, "urban")}
        )
        manager = MergeFromPrivateModuleManager(
            MergeFromPrivateParameters(
                stopline_margin=3.0,
                stop_duration_sec=1.0,
                stop_distance_threshold=1.0,
                base_link_to_front=0.0,
            )
        )
        message = input_path()
        odometry = Odometry()
        odometry.pose.pose.position.x = 27.0
        odometry.pose.pose.orientation.w = 1.0
        first = manager.modify_path_velocity(
            message, to_path(message), PlannerData(lanelet_map=map_data, odometry=odometry, current_time=10.0)
        )
        second_output = to_path(message)
        second = manager.modify_path_velocity(
            message, second_output, PlannerData(lanelet_map=map_data, odometry=odometry, current_time=11.1)
        )
        self.assertTrue(first.modified)
        self.assertEqual(first.stop_reasons[0].reason, "MergeFromPrivateRoad")
        self.assertFalse(second.modified)
        self.assertIn("GO", second.status)

    def test_public_route_does_not_activate(self):
        map_data = BehaviorVelocityMap(
            "synthetic", road_lanelets={1: lane(1, 0.0, 30.0, "urban"), 2: lane(2, 30.0, 60.0, "urban")}
        )
        message = input_path()
        result = MergeFromPrivateModuleManager(MergeFromPrivateParameters()).modify_path_velocity(
            message, to_path(message), PlannerData(lanelet_map=map_data, current_time=1.0)
        )
        self.assertFalse(result.modified)


if __name__ == "__main__":
    unittest.main()
