#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


def make_dummy_points(num_points):
    points = []

    for i in range(num_points):
        # 원형 point cloud를 만들기 위한 각도.
        angle = 2.0 * math.pi * i / num_points

        # 가짜 LiDAR 점들을 반지름 5m 원 위에 배치한다.
        radius = 5.0

        # LiDAR 좌표계 기준 x, y 위치.
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        # z=0 평면 위에 점들을 놓는다.
        z = 0.0

        # intensity는 LiDAR 반사 강도 같은 값.
        # 여기서는 전부 같은 dummy 값으로 둔다.
        intensity = 100.0

        # PointCloud2의 한 point를 [x, y, z, intensity] 구조로 추가한다.
        points.append([x, y, z, intensity])

    return points


def main():
    # 이 프로세스를 ROS node로 등록한다.
    rospy.init_node("dummy_lidar_pub")

    # publish할 PointCloud2 토픽 이름.
    topic_name = rospy.get_param("~topic", "/tutorial/lidar/points")

    # 이 point cloud가 어느 좌표계 기준인지 나타내는 frame_id.
    # static_tf.launch에서 base_link -> front_lidar TF와 연결된다.
    frame_id = rospy.get_param("~frame_id", "front_lidar")

    # LiDAR publish 주기.
    # 실제 LiDAR는 보통 10Hz 근처인 경우가 많다.
    rate_hz = rospy.get_param("~rate", 10.0)

    # 한 프레임에 넣을 dummy point 개수.
    num_points = rospy.get_param("~num_points", 360)

    # /tutorial/lidar/points 토픽에 sensor_msgs/PointCloud2 타입을 publish할 publisher 생성.
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=5)

    # while loop를 rate_hz 주기로 돌리기 위한 ROS Rate 객체.
    rate = rospy.Rate(rate_hz)

    # PointCloud2 내부의 point 하나가 어떤 필드들로 구성되는지 정의한다.
    # 여기서는 x, y, z, intensity 네 개의 float32 필드를 가진다.
    fields = [
        # "x"는 point 하나의 0 byte offset부터 시작하는 float32 값.
        PointField("x", 0, PointField.FLOAT32, 1),

        # "y"는 4 byte offset부터 시작한다. float32 하나가 4 bytes이기 때문.
        PointField("y", 4, PointField.FLOAT32, 1),

        # "z"는 8 byte offset부터 시작한다.
        PointField("z", 8, PointField.FLOAT32, 1),

        # "intensity"는 12 byte offset부터 시작한다.
        PointField("intensity", 12, PointField.FLOAT32, 1),
    ]

    rospy.loginfo("dummy_lidar_pub started")
    rospy.loginfo("publishing topic: %s", topic_name)
    rospy.loginfo("frame_id: %s", frame_id)
    rospy.loginfo("rate: %.1f Hz", rate_hz)
    rospy.loginfo("num_points: %d", num_points)

    while not rospy.is_shutdown():
        # PointCloud2에도 header가 필요하다.
        # stamp는 이 point cloud frame의 시간, frame_id는 좌표계 이름.
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # 가짜 point 목록 생성.
        points = make_dummy_points(num_points)

        # points 리스트를 sensor_msgs/PointCloud2 메시지로 변환한다.
        # 직접 binary layout을 만들지 않고 ROS helper를 사용한다.
        cloud_msg = point_cloud2.create_cloud(header, fields, points)

        # 완성된 PointCloud2 메시지를 publish한다.
        pub.publish(cloud_msg)

        # 지정한 rate_hz에 맞게 루프를 쉰다.
        rate.sleep()


if __name__ == "__main__":
    main()