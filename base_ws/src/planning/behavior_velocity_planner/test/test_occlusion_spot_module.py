import unittest

from nav_msgs.msg import OccupancyGrid, Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId

from behavior_velocity_occlusion_spot_module import (
    OcclusionSpotModuleManager,
    OcclusionSpotParameters,
)
from behavior_velocity_planner_common import PlannerData
from behavior_velocity_planner_common.path_utils import to_path


def path():
    message = PathWithLaneId()
    message.header.frame_id = "map"
    for x in range(0, 61):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        point.point.pose.orientation.w = 1.0
        point.point.longitudinal_velocity_mps = 10.0
        point.lane_ids = [1]
        message.points.append(point)
    return message


def grid(hidden=True):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = 1.0
    message.info.width = 80
    message.info.height = 20
    message.info.origin.position.y = -10.0
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (message.info.width * message.info.height)
    if hidden:
        for y in (11, 12):
            for x in (10, 11):
                message.data[y * message.info.width + x] = 50
    return message


class OcclusionSpotTest(unittest.TestCase):
    def run_case(self, hidden):
        message = path()
        output = to_path(message)
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear.x = 10.0
        parameters = OcclusionSpotParameters(
            detection_area_offset=10.0,
            detection_area_length=30.0,
            lateral_distance=1.0,
            max_lateral_distance=3.0,
            min_occlusion_spot_size=1.0,
            safety_ratio=0.5,
        )
        result = OcclusionSpotModuleManager(parameters).modify_path_velocity(
            message,
            output,
            PlannerData(odometry=odometry, occupancy_grid=grid(hidden), current_time=10.0),
        )
        return result, output

    def test_hidden_grid_slows_path(self):
        result, output = self.run_case(True)
        self.assertTrue(result.modified)
        self.assertFalse(result.planning_factors)
        self.assertTrue(result.markers)
        self.assertTrue(any(point.longitudinal_velocity_mps < 10.0 for point in output.points))

    def test_free_grid_keeps_profile(self):
        result, _ = self.run_case(False)
        self.assertFalse(result.modified)

    def test_stopped_ego_anchors_profile_and_restarts_slowly(self):
        message = path()
        output = to_path(message)
        odometry = Odometry()
        odometry.pose.pose.position.x = 5.0
        odometry.pose.pose.orientation.w = 1.0
        parameters = OcclusionSpotParameters(
            detection_area_offset=20.0,
            detection_area_length=30.0,
            lateral_distance=1.0,
            max_lateral_distance=3.0,
            min_occlusion_spot_size=1.0,
            base_link_to_front=3.0,
        )
        result = OcclusionSpotModuleManager(parameters).modify_path_velocity(
            message,
            output,
            PlannerData(odometry=odometry, occupancy_grid=grid(True), current_time=10.0),
        )
        self.assertTrue(result.modified)
        ego_points = [
            point
            for point in output.points
            if abs(point.pose.position.x - 5.0) < 1.0e-3
        ]
        self.assertEqual(ego_points[0].longitudinal_velocity_mps, 0.0)
        self.assertGreater(len(output.points), len(message.points))


if __name__ == "__main__":
    unittest.main()
