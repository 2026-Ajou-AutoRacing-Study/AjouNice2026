import unittest

import rospy
from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from tier4_v2x_msgs.msg import VirtualTrafficLightState, VirtualTrafficLightStateArray

from behavior_velocity_planner_common import (
    BehaviorVelocityMap,
    MapLineString,
    MapVirtualTrafficLight,
    PlannerData,
)
from behavior_velocity_planner_common.path_utils import to_path
from behavior_velocity_virtual_traffic_light_module import (
    VirtualTrafficLightModuleManager,
    VirtualTrafficLightParameters,
)


def line(line_id, x, tags=None):
    return MapLineString(line_id, ((x, -2.0, 0.0), (x, 2.0, 0.0)), tags or {})


def map_data():
    element = MapVirtualTrafficLight(
        200,
        line(100, 12.0, {"type": "fms"}),
        line(101, 5.0),
        line(102, 20.0),
        (line(103, 40.0),),
        (1,),
    )
    return BehaviorVelocityMap("synthetic", virtual_traffic_lights={200: element})


def path():
    message = PathWithLaneId()
    message.header.frame_id = "map"
    for x in range(0, 61, 2):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        point.point.pose.orientation.w = 1.0
        point.point.longitudinal_velocity_mps = 5.0
        point.lane_ids = [1]
        message.points.append(point)
    return message


def states(approval, finalized=False, stamp=10.0):
    state = VirtualTrafficLightState()
    state.stamp = rospy.Time.from_sec(stamp)
    state.type = "fms"
    state.id = "100"
    state.approval = approval
    state.is_finalized = finalized
    return VirtualTrafficLightStateArray(stamp=state.stamp, states=[state])


class VirtualTrafficLightTest(unittest.TestCase):
    def run_case(self, x, incoming=None):
        message = path()
        output = to_path(message)
        odometry = Odometry()
        odometry.pose.pose.position.x = float(x)
        odometry.pose.pose.orientation.w = 1.0
        manager = VirtualTrafficLightModuleManager(
            VirtualTrafficLightParameters(base_link_to_front=0.0)
        )
        result = manager.modify_path_velocity(
            message,
            output,
            PlannerData(
                lanelet_map=map_data(),
                odometry=odometry,
                virtual_traffic_light_states=incoming,
                current_time=10.0,
            ),
        )
        return result, output

    def test_no_approval_stops_at_stop_line_and_requests(self):
        result, _ = self.run_case(10.0, states(False))
        self.assertTrue(result.modified)
        self.assertEqual(result.stop_reasons[0].reason, "VirtualTrafficLight")
        self.assertEqual(result.infrastructure_commands[0].state, 1)

    def test_approval_before_stop_line_passes(self):
        result, _ = self.run_case(10.0, states(True))
        self.assertFalse(result.modified)
        self.assertIn("APPROVED", result.status)

    def test_after_stop_waits_for_finalization_at_end_line(self):
        result, _ = self.run_case(25.0, states(True, False))
        self.assertTrue(result.modified)
        self.assertIn("STOP_FOR_FINALIZATION", result.status)
        self.assertEqual(result.infrastructure_commands[0].state, 2)


if __name__ == "__main__":
    unittest.main()
