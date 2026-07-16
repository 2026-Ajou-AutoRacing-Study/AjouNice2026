#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from autoware_internal_planning_msgs.msg import PathWithLaneId
from std_msgs.msg import Bool

from mvp_utils import project_pose_to_path


class OutOfLaneDetector:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.check_rate = float(rospy.get_param('~check_rate', 10.0))
        self.lateral_threshold = float(rospy.get_param('~lateral_threshold', 0.8))

        self.odom = None
        self.path = None

        self.sub_odom = rospy.Subscriber('/odometry', Odometry, self.cb_odom)
        input_path_topic = rospy.get_param('~input_path_topic', '~input/path')
        output_path_topic = rospy.get_param('~output_path_topic', '~output/path')
        self.sub_path = rospy.Subscriber(input_path_topic, PathWithLaneId, self.cb_path)

        self.pub_out = rospy.Publisher('~output/out_of_lane', Bool, queue_size=1)

        rospy.loginfo('OutOfLaneDetector initialized (lateral_threshold=%f)', self.lateral_threshold)

    def cb_odom(self, msg: Odometry):
        self.odom = msg
        self.check_and_publish()

    def cb_path(self, msg: PathWithLaneId):
        self.path = msg
        self.check_and_publish()

    def compute_min_distance_to_path(self, pose):
        if not self.path or not self.path.points:
            return float('inf')

        projection = project_pose_to_path(self.path, pose)
        return projection.distance if projection is not None else float('inf')

    def check_and_publish(self):
        if self.odom is None or self.path is None:
            return

        distance = self.compute_min_distance_to_path(self.odom.pose.pose)
        out = Bool()
        out.data = distance > self.lateral_threshold
        self.pub_out.publish(out)

    def run(self):
        rate = rospy.Rate(self.check_rate)
        while not rospy.is_shutdown():
            self.check_and_publish()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('out_of_lane_detector')
    node = OutOfLaneDetector()
    node.run()
