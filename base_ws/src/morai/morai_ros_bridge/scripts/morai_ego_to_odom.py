#!/usr/bin/env python3

import math

import rospy
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


class MoraiEgoToOdom:
    def __init__(self):
        # MORAI ego 상태를 받을 토픽 이름.
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")

        # 변환된 Odometry를 publish할 토픽 이름.
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odom")

        # Odometry의 기준 좌표계.
        # morai /tf 보면 map에서 base_link로 뻗어나가는 구조였으니까 똑같이 map 써주면 됨
        self.frame_id = rospy.get_param("~frame_id", "map")

        # 차량 body frame 이름.
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        # Odometry publisher.
        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=10
        )

        # MORAI ego status subscriber.
        self.ego_sub = rospy.Subscriber(
            self.ego_topic,
            EgoVehicleStatus,
            self.ego_callback,
            queue_size=10
        )

        rospy.loginfo("morai_ego_to_odom started")
        rospy.loginfo("subscribe: %s", self.ego_topic)
        rospy.loginfo("publish:   %s", self.odom_topic)

    def ego_callback(self, msg: EgoVehicleStatus):
        odom = Odometry()

        # MORAI header stamp를 그대로 사용.
        # rosbag play --clock 환경에서도 시간 동기 확인이 쉬움.
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # MORAI position을 Odometry pose.position에 복사.
        odom.pose.pose.position.x = msg.position.x
        odom.pose.pose.position.y = msg.position.y
        odom.pose.pose.position.z = msg.position.z

        # MORAI heading은 degree 단위로 들어오는 것으로 봄.
        # ROS quaternion은 radian 기반 roll/pitch/yaw에서 생성.
        yaw_rad = math.radians(msg.heading)

        q = quaternion_from_euler(0.0, 0.0, yaw_rad)
        odom.pose.pose.orientation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )

        # MORAI velocity를 Odometry twist.linear에 복사.
        odom.twist.twist.linear.x = msg.velocity.x
        odom.twist.twist.linear.y = msg.velocity.y
        odom.twist.twist.linear.z = msg.velocity.z

        # MORAI angular_velocity를 Odometry twist.angular에 복사.
        odom.twist.twist.angular.x = msg.angular_velocity.x
        odom.twist.twist.angular.y = msg.angular_velocity.y
        odom.twist.twist.angular.z = msg.angular_velocity.z

        self.odom_pub.publish(odom)


def main():
    rospy.init_node("morai_ego_to_odom")
    MoraiEgoToOdom()
    rospy.spin()


if __name__ == "__main__":
    main()