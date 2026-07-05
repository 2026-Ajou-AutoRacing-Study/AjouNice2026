#!/usr/bin/env python3

import math

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


FRAME_ID = "map"


def yaw_to_quaternion(yaw):
    qx, qy, qz, qw = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return qx, qy, qz, qw


def make_pose(x, y, z, yaw, stamp):
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = FRAME_ID
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def generate_straight(num_points, ds):
    points = []
    for i in range(num_points):
        s = i * ds
        points.append((s, 0.0, 0.0, 0.0))
    return points


def generate_circle(num_points, radius):
    points = []
    if radius <= 0.0:
        rospy.logwarn("~radius는 양수여야 합니다. 20.0 m를 사용합니다.")
        radius = 20.0

    for i in range(num_points):
        theta = 2.0 * math.pi * float(i) / float(max(num_points, 1))
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        yaw = theta + math.pi / 2.0
        points.append((x, y, 0.0, yaw))
    return points


def generate_s_curve(num_points, ds, amplitude, wavelength):
    points = []
    if wavelength <= 0.0:
        rospy.logwarn("~wavelength는 양수여야 합니다. 40.0 m를 사용합니다.")
        wavelength = 40.0

    for i in range(num_points):
        s = i * ds
        phase = 2.0 * math.pi * s / wavelength
        x = s
        y = amplitude * math.sin(phase)
        dy_dx = amplitude * (2.0 * math.pi / wavelength) * math.cos(phase)
        yaw = math.atan2(dy_dx, 1.0)
        points.append((x, y, 0.0, yaw))
    return points


def build_path(points):
    stamp = rospy.Time.now()
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = FRAME_ID
    path.poses = [make_pose(x, y, z, yaw, stamp) for x, y, z, yaw in points]
    return path


def build_marker(points):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = FRAME_ID
    marker.ns = "reference_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.25
    marker.color.r = 0.1
    marker.color.g = 0.8
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.points = [Point(x=x, y=y, z=z) for x, y, z, _ in points]
    return marker


def generate_points(mode, num_points, ds, amplitude, wavelength, radius):
    if num_points < 2:
        rospy.logwarn("~num_points는 최소 2 이상이어야 합니다. 2를 사용합니다.")
        num_points = 2

    if mode == "straight":
        return generate_straight(num_points, ds)
    if mode == "circle":
        return generate_circle(num_points, radius)
    if mode == "s_curve":
        return generate_s_curve(num_points, ds, amplitude, wavelength)

    rospy.logwarn("지원하지 않는 ~mode '%s'입니다. 's_curve'로 대체합니다.", mode)
    return generate_s_curve(num_points, ds, amplitude, wavelength)


def main():
    rospy.init_node("virtual_trajectory_generator")

    mode = rospy.get_param("~mode", "s_curve")
    target_speed_mps = rospy.get_param("~target_speed_mps", 5.0)
    num_points = rospy.get_param("~num_points", 200)
    ds = rospy.get_param("~ds", 0.5)
    amplitude = rospy.get_param("~amplitude", 5.0)
    wavelength = rospy.get_param("~wavelength", 40.0)
    radius = rospy.get_param("~radius", 20.0)

    path_pub = rospy.Publisher("/reference_path", Path, queue_size=1, latch=True)
    speed_pub = rospy.Publisher("/target_speed", Float32, queue_size=1, latch=True)
    marker_pub = rospy.Publisher("/reference_path_marker", Marker, queue_size=1)

    points = generate_points(mode, int(num_points), float(ds), float(amplitude), float(wavelength), float(radius))
    path_msg = build_path(points)
    speed_msg = Float32(data=float(target_speed_mps))

    rate = rospy.Rate(10)
    rospy.loginfo("가상 궤적 발행 중: mode=%s, points=%d, target_speed=%.2f m/s",
                  mode, len(points), speed_msg.data)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        path_msg.header.stamp = now
        for pose in path_msg.poses:
            pose.header.stamp = now

        marker_msg = build_marker(points)
        marker_msg.header.stamp = now

        path_pub.publish(path_msg)
        speed_pub.publish(speed_msg)
        marker_pub.publish(marker_msg)
        rate.sleep()


if __name__ == "__main__":
    main()
