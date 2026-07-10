#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from autoware_internal_planning_msgs.msg import Scenario
from autoware_planning_msgs.msg import LaneletPrimitive, LaneletRoute, LaneletSegment


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


class BppMinimalInputPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))

        self.lane_id = int(rospy.get_param("~lane_id", 1001))
        self.start_x = float(rospy.get_param("~start_x", 0.0))
        self.start_y = float(rospy.get_param("~start_y", 0.0))
        self.start_yaw = float(rospy.get_param("~start_yaw", 0.0))
        self.goal_x = float(rospy.get_param("~goal_x", 60.0))
        self.goal_y = float(rospy.get_param("~goal_y", 0.0))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", 0.0))

        route_topic = rospy.get_param("~route_topic", "/planning/mission_planning/route")
        odometry_topic = rospy.get_param("~odometry_topic", "/localization/kinematic_state")
        scenario_topic = rospy.get_param("~scenario_topic", "/planning/scenario_planning/scenario")

        self.route_pub = rospy.Publisher(route_topic, LaneletRoute, queue_size=1, latch=True)
        self.odom_pub = rospy.Publisher(odometry_topic, Odometry, queue_size=1)
        self.scenario_pub = rospy.Publisher(scenario_topic, Scenario, queue_size=1, latch=True)

        rospy.loginfo(
            "bpp_minimal_input_publisher: route lane_id=%d start=(%.2f, %.2f) goal=(%.2f, %.2f)",
            self.lane_id,
            self.start_x,
            self.start_y,
            self.goal_x,
            self.goal_y,
        )

    def make_route(self, stamp):
        route = LaneletRoute()
        route.header.stamp = stamp
        route.header.frame_id = self.frame_id
        route.start_pose.position.x = self.start_x
        route.start_pose.position.y = self.start_y
        route.start_pose.position.z = 0.0
        route.start_pose.orientation = quaternion_from_yaw(self.start_yaw)
        route.goal_pose.position.x = self.goal_x
        route.goal_pose.position.y = self.goal_y
        route.goal_pose.position.z = 0.0
        route.goal_pose.orientation = quaternion_from_yaw(self.goal_yaw)

        primitive = LaneletPrimitive()
        primitive.id = self.lane_id
        primitive.primitive_type = "lane"

        segment = LaneletSegment()
        segment.preferred_primitive = primitive
        segment.primitives = []
        route.segments = [segment]
        route.uuid.uuid = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
        route.allow_modification = True
        return route

    def make_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.start_x
        odom.pose.pose.position.y = self.start_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(self.start_yaw)
        return odom

    def make_scenario(self):
        scenario = Scenario()
        scenario.current_scenario = Scenario.LANEDRIVING
        scenario.activating_scenarios = [Scenario.LANEDRIVING]
        return scenario

    def spin(self):
        rate = rospy.Rate(max(self.publish_rate, 0.1))
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.route_pub.publish(self.make_route(stamp))
            self.odom_pub.publish(self.make_odometry(stamp))
            self.scenario_pub.publish(self.make_scenario())
            rate.sleep()


def main():
    rospy.init_node("bpp_minimal_input_publisher")
    BppMinimalInputPublisher().spin()


if __name__ == "__main__":
    main()
