#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


def main():
    # 이 프로세스를 ROS node로 등록한다.
    # rosnode list에서 /dummy_imu_pub 이름으로 보이게 된다.
    rospy.init_node("dummy_imu_pub")

    # launch 파일에서 넘긴 private parameter "~topic"을 읽는다.
    # 없으면 기본값 "/tutorial/imu"를 사용한다.
    topic_name = rospy.get_param("~topic", "/tutorial/imu")

    # 이 IMU 데이터가 어떤 좌표계 기준인지 나타낼 frame_id.
    # 나중에 TF tree에서 base_link -> imu 관계와 연결된다.
    frame_id = rospy.get_param("~frame_id", "imu")

    # IMU publish 주기.
    # 실제 IMU는 보통 LiDAR보다 훨씬 높은 주기로 나온다.
    rate_hz = rospy.get_param("~rate", 50.0)

    # /tutorial/imu 토픽에 sensor_msgs/Imu 타입 메시지를 publish할 publisher 생성.
    pub = rospy.Publisher(topic_name, Imu, queue_size=10)

    # while loop를 rate_hz 주기로 돌리기 위한 ROS Rate 객체.
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("dummy_imu_pub started")
    rospy.loginfo("publishing topic: %s", topic_name)
    rospy.loginfo("frame_id: %s", frame_id)
    rospy.loginfo("rate: %.1f Hz", rate_hz)

    while not rospy.is_shutdown():
        # sensor_msgs/Imu 메시지 객체 생성.
        msg = Imu()

        # header.stamp는 이 센서 데이터가 생성/측정된 ROS 시간.
        # 센서 동기화에서 매우 중요하다.
        msg.header.stamp = rospy.Time.now()

        # header.frame_id는 이 IMU 값들이 어느 좌표계 기준인지 나타낸다.
        msg.header.frame_id = frame_id

        # orientation은 IMU의 자세.
        # x=0,y=0,z=0,w=1은 회전이 없는 단위 quaternion.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # angular_velocity는 각속도(rad/s).
        # z축 각속도는 차량 기준 yaw rate와 관련된다.
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.01

        # linear_acceleration은 선형 가속도(m/s^2).
        # 여기서는 정지 상태의 중력가속도처럼 z축에 9.81을 넣는다.
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        # 완성된 IMU 메시지를 topic_name으로 publish한다.
        pub.publish(msg)

        # 지정한 rate_hz에 맞게 루프를 쉰다.
        rate.sleep()


if __name__ == "__main__":
    main()