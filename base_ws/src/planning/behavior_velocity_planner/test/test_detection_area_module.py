import unittest

from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
)

from behavior_velocity_detection_area_module import (
    DetectionAreaModuleManager,
    DetectionAreaParameters,
)
from behavior_velocity_planner_common import (
    BehaviorVelocityMap,
    MapDetectionArea,
    MapLineString,
    PlannerData,
)
from behavior_velocity_planner_common.path_utils import to_path


def path():
    message = PathWithLaneId()
    message.header.frame_id = "map"
    for x in range(0, 41, 2):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        point.point.pose.orientation.w = 1.0
        point.point.longitudinal_velocity_mps = 5.0
        point.lane_ids = [1]
        message.points.append(point)
    return message


def lanelet_map():
    line = MapLineString(101, ((10.0, -2.0, 0.0), (10.0, 2.0, 0.0)), {})
    area = MapDetectionArea(
        201,
        (((14.0, -2.0, 0.0), (20.0, -2.0, 0.0), (20.0, 2.0, 0.0), (14.0, 2.0, 0.0)),),
        (100,),
        (1,),
        line,
    )
    return BehaviorVelocityMap("synthetic", detection_areas={201: area})


def object_message(x=None):
    message = PredictedObjects()
    if x is None:
        return message
    detected = PredictedObject()
    classification = ObjectClassification(label=ObjectClassification.CAR, probability=1.0)
    detected.classification = [classification]
    detected.kinematics.initial_pose_with_covariance.pose.position.x = float(x)
    detected.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
    message.objects = [detected]
    return message


class DetectionAreaTest(unittest.TestCase):
    def run_case(self, objects=None, manager=None, now=10.0):
        input_path = path()
        output = to_path(input_path)
        parameters = DetectionAreaParameters(
            pointcloud=False,
            target_labels={ObjectClassification.CAR},
            base_link_to_front=0.0,
            state_clear_time=1.0,
        )
        manager = manager or DetectionAreaModuleManager(parameters)
        odometry = Odometry()
        odometry.pose.pose.orientation.w = 1.0
        result = manager.modify_path_velocity(
            input_path,
            output,
            PlannerData(
                lanelet_map=lanelet_map(),
                odometry=odometry,
                dynamic_objects=objects or object_message(),
                current_time=now,
            ),
        )
        return result, output, manager

    def test_object_in_area_stops_at_ref_line(self):
        result, output, _ = self.run_case(object_message(16.0))
        self.assertTrue(result.modified)
        self.assertEqual(result.stop_reasons[0].reason, "DetectionArea")
        points = [point.pose.position.x for point in output.points]
        self.assertEqual(output.points[points.index(10.0)].longitudinal_velocity_mps, 0.0)

    def test_clear_area_passes(self):
        result, _, _ = self.run_case()
        self.assertFalse(result.modified)

    def test_detection_is_held_then_cleared(self):
        manager = DetectionAreaModuleManager(
            DetectionAreaParameters(
                pointcloud=False,
                target_labels={ObjectClassification.CAR},
                base_link_to_front=0.0,
                state_clear_time=1.0,
            )
        )
        first, _, manager = self.run_case(object_message(16.0), manager, 10.0)
        held, _, manager = self.run_case(object_message(), manager, 10.5)
        cleared, _, _ = self.run_case(object_message(), manager, 12.0)
        self.assertTrue(first.modified)
        self.assertTrue(held.modified)
        self.assertFalse(cleared.modified)


if __name__ == "__main__":
    unittest.main()
