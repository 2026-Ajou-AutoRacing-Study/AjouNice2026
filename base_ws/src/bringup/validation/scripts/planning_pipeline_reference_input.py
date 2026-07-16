#!/usr/bin/env python3

import json
import os
import uuid

import rospy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.msg import Scenario, VelocityLimit
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
)
from geometry_msgs.msg import AccelWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry


def read_json(path):
    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        return json.load(stream)


def assign_pose(target, value):
    position = value["position"]
    orientation = value["orientation"]
    target.position.x = float(position["x"])
    target.position.y = float(position["y"])
    target.position.z = float(position.get("z", 0.0))
    target.orientation.x = float(orientation.get("x", 0.0))
    target.orientation.y = float(orientation.get("y", 0.0))
    target.orientation.z = float(orientation.get("z", 0.0))
    target.orientation.w = float(orientation.get("w", 1.0))


def make_dynamic_objects(document, stamp):
    message = PredictedObjects()
    source = document.get("dynamic_objects") or {"frame_id": "map", "objects": []}
    message.header.stamp = stamp
    message.header.frame_id = str(source.get("frame_id", "map"))
    for value in source.get("objects", []):
        detected = PredictedObject()
        detected.object_id.uuid = [int(item) for item in value.get("object_id", [0] * 16)]
        detected.existence_probability = float(value.get("existence_probability", 1.0))
        for item in value.get("classification", []):
            classification = ObjectClassification()
            classification.label = int(item["label"])
            classification.probability = float(item["probability"])
            detected.classification.append(classification)
        assign_pose(detected.kinematics.initial_pose_with_covariance.pose, value["pose"])
        velocity = value.get("linear_velocity", {})
        twist = detected.kinematics.initial_twist_with_covariance.twist
        twist.linear.x = float(velocity.get("x", 0.0))
        twist.linear.y = float(velocity.get("y", 0.0))
        twist.linear.z = float(velocity.get("z", 0.0))
        shape = value.get("shape", {})
        detected.shape.type = int(shape.get("type", 0))
        dimensions = shape.get("dimensions", {})
        detected.shape.dimensions.x = float(dimensions.get("x", 1.0))
        detected.shape.dimensions.y = float(dimensions.get("y", 1.0))
        detected.shape.dimensions.z = float(dimensions.get("z", 1.0))
        for source_path in value.get("predicted_paths", []):
            predicted_path = PredictedPath()
            predicted_path.time_step = rospy.Duration.from_sec(
                float(source_path.get("time_step_sec", 0.5))
            )
            predicted_path.confidence = float(source_path.get("confidence", 1.0))
            for source_pose in source_path.get("poses", []):
                pose = Pose()
                assign_pose(pose, source_pose)
                predicted_path.path.append(pose)
            detected.kinematics.predicted_paths.append(predicted_path)
        message.objects.append(detected)
    return message


class PlanningPipelineReferenceInput:
    def __init__(self):
        self.reference_path = rospy.get_param("~reference_json")
        self.reference = read_json(self.reference_path)
        self.scenario = self.reference["scenario"]
        self.odometry_value = self.scenario["odometry"]
        self.route_value = self.scenario["route"]
        self.frame_id = str(self.odometry_value.get("frame_id", "map"))
        self.publish_rate = max(float(rospy.get_param("~publish_rate", 10.0)), 0.1)
        self.request_delay = max(float(rospy.get_param("~request_delay", 1.0)), 0.0)
        self.max_velocity = max(
            float(rospy.get_param("~max_velocity", 16.666666)), 0.0
        )
        self.route_service = rospy.get_param(
            "~route_service",
            "/planning/mission_planning/mission_planner/set_waypoint_route",
        )
        self.started_at = rospy.Time.now()
        self.request_sent = False

        self.odometry_pub = rospy.Publisher(
            "/localization/kinematic_state", Odometry, queue_size=1
        )
        self.acceleration_pub = rospy.Publisher(
            "/localization/acceleration",
            AccelWithCovarianceStamped,
            queue_size=1,
        )
        self.operation_mode_pub = rospy.Publisher(
            "/system/operation_mode/state",
            OperationModeState,
            queue_size=1,
            latch=True,
        )
        self.scenario_pub = rospy.Publisher(
            "/planning/scenario_planning/scenario", Scenario, queue_size=1, latch=True
        )
        self.velocity_limit_pub = rospy.Publisher(
            "/planning/scenario_planning/max_velocity",
            VelocityLimit,
            queue_size=1,
            latch=True,
        )
        self.objects_pub = rospy.Publisher(
            "/perception/object_recognition/objects",
            PredictedObjects,
            queue_size=1,
        )

        rospy.loginfo(
            "planning pipeline reference input: scenario=%s reference=%s",
            self.scenario.get("name", "unnamed"),
            self.reference_path,
        )

    def make_odometry(self, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.child_frame_id = str(
            self.odometry_value.get("child_frame_id", "base_link")
        )
        assign_pose(message.pose.pose, self.odometry_value["pose"])
        linear = self.odometry_value.get("linear_velocity", {})
        angular = self.odometry_value.get("angular_velocity", {})
        message.twist.twist.linear.x = float(linear.get("x", 0.0))
        message.twist.twist.linear.y = float(linear.get("y", 0.0))
        message.twist.twist.linear.z = float(linear.get("z", 0.0))
        message.twist.twist.angular.x = float(angular.get("x", 0.0))
        message.twist.twist.angular.y = float(angular.get("y", 0.0))
        message.twist.twist.angular.z = float(angular.get("z", 0.0))
        return message

    def publish_common_inputs(self, stamp):
        self.odometry_pub.publish(self.make_odometry(stamp))

        acceleration = AccelWithCovarianceStamped()
        acceleration.header.stamp = stamp
        acceleration.header.frame_id = self.frame_id
        self.acceleration_pub.publish(acceleration)

        operation_mode = OperationModeState()
        operation_mode.stamp = stamp
        operation_mode.mode = OperationModeState.AUTONOMOUS
        operation_mode.is_autoware_control_enabled = True
        operation_mode.is_stop_mode_available = True
        operation_mode.is_autonomous_mode_available = True
        operation_mode.is_local_mode_available = True
        operation_mode.is_remote_mode_available = True
        self.operation_mode_pub.publish(operation_mode)

        scenario = Scenario()
        scenario.current_scenario = Scenario.LANEDRIVING
        scenario.activating_scenarios = [Scenario.LANEDRIVING]
        self.scenario_pub.publish(scenario)

        velocity_limit = VelocityLimit()
        velocity_limit.stamp = stamp
        velocity_limit.max_velocity = self.max_velocity
        velocity_limit.use_constraints = False
        velocity_limit.sender = "planning_pipeline_reference_input"
        self.velocity_limit_pub.publish(velocity_limit)
        self.objects_pub.publish(make_dynamic_objects(self.reference, stamp))

    def maybe_request_route(self):
        if self.request_sent:
            return
        if (rospy.Time.now() - self.started_at).to_sec() < self.request_delay:
            return
        try:
            rospy.wait_for_service(self.route_service, timeout=1.0)
            request = SetWaypointRouteRequest()
            request.header.stamp = rospy.Time.now()
            request.header.frame_id = self.frame_id
            assign_pose(request.goal_pose, self.route_value["goal_pose"])
            request.uuid.uuid = list(uuid.uuid4().bytes)
            request.allow_modification = bool(
                self.route_value.get("allow_modification", False)
            )
            response = rospy.ServiceProxy(self.route_service, SetWaypointRoute)(request)
            if response.status.success:
                self.request_sent = True
                rospy.loginfo(
                    "planning pipeline route accepted: goal=(%.2f, %.2f)",
                    request.goal_pose.position.x,
                    request.goal_pose.position.y,
                )
            else:
                rospy.logwarn_throttle(
                    3.0, "planning pipeline route rejected: %s", response.status.message
                )
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(3.0, "waiting for Mission Planner service: %s", error)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.publish_common_inputs(stamp)
            self.maybe_request_route()
            rate.sleep()


def main():
    rospy.init_node("planning_pipeline_reference_input")
    PlanningPipelineReferenceInput().spin()


if __name__ == "__main__":
    main()
