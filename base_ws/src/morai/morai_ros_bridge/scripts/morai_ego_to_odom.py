#!/usr/bin/env python3

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def wgs84_to_utm(latitude_deg, longitude_deg, zone):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    central_meridian = math.radians((zone - 1) * 6 - 180 + 3)

    e2 = WGS84_F * (2.0 - WGS84_F)
    ep2 = e2 / (1.0 - e2)
    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    tan_lat = math.tan(latitude)

    n = WGS84_A / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ep2 * cos_lat * cos_lat
    a = cos_lat * (longitude - central_meridian)

    m = WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * latitude
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * latitude)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * latitude)
        - (35 * e2**3 / 3072) * math.sin(6 * latitude)
    )

    easting = UTM_K0 * n * (
        a
        + (1 - t + c) * a**3 / 6
        + (5 - 18 * t + t**2 + 72 * c - 58 * ep2) * a**5 / 120
    ) + 500000.0
    northing = UTM_K0 * (
        m
        + n * tan_lat * (
            a**2 / 2
            + (5 - t + 9 * c + 4 * c**2) * a**4 / 24
            + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * a**6 / 720
        )
    )
    if latitude_deg < 0.0:
        northing += 10000000.0
    return easting, northing


class MoraiEgoToOdom:
    def __init__(self):
        # MORAI ego 상태를 받을 토픽 이름.
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")

        # 변환된 Odometry를 publish할 토픽 이름.
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odom")

        # MORAI EgoVehicleStatus.position은 MORAI local map 좌표이므로 morai_map에 둔다.
        self.frame_id = rospy.get_param("~frame_id", "morai_map")

        # 차량 body frame 이름.
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_tf = get_bool_param("~publish_tf", False)
        self.last_tf_stamp = None
        self.last_ego_stamp = None
        self.prev_ego_heading_deg = None
        self.prev_ego_stamp = None

        self.enable_gps_fallback = get_bool_param("~enable_gps_fallback", False)
        self.gps_topic = rospy.get_param("~gps_topic", "/sensor/gps")
        self.gps_ego_timeout_sec = float(rospy.get_param("~gps_ego_timeout_sec", 0.5))
        self.gps_utm_zone = int(rospy.get_param("~gps_utm_zone", 52))
        self.gps_use_message_offsets = get_bool_param("~gps_use_message_offsets", True)
        self.gps_fallback_east_offset = float(rospy.get_param("~gps_fallback_east_offset", 302595.0))
        self.gps_fallback_north_offset = float(rospy.get_param("~gps_fallback_north_offset", 4124145.0))
        self.gps_morai_origin_altitude = float(rospy.get_param("~gps_morai_origin_altitude", 0.0))
        self.gps_sensor_z_offset = float(rospy.get_param("~gps_sensor_z_offset", 1.23))
        self.gps_min_heading_distance = float(rospy.get_param("~gps_min_heading_distance", 0.05))
        self.last_gps_xy = None
        self.last_gps_yaw = 0.0

        # Odometry publisher.
        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        # MORAI ego status subscriber.
        self.ego_sub = rospy.Subscriber(
            self.ego_topic,
            EgoVehicleStatus,
            self.ego_callback,
            queue_size=10
        )
        self.gps_sub = None
        if self.enable_gps_fallback:
            self.gps_sub = rospy.Subscriber(
                self.gps_topic,
                GPSMessage,
                self.gps_callback,
                queue_size=50,
            )

        rospy.loginfo("morai_ego_to_odom started")
        rospy.loginfo("subscribe: %s", self.ego_topic)
        if self.enable_gps_fallback:
            rospy.loginfo("gps fallback: %s", self.gps_topic)
        rospy.loginfo("publish:   %s", self.odom_topic)
        rospy.loginfo(
            "frames:    %s -> %s%s",
            self.frame_id,
            self.child_frame_id,
            " with TF" if self.publish_tf else "",
        )

    def normalize_deg_diff(self, diff):
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0
        return diff

    def estimate_yaw_rate(self, heading_deg, stamp):
        yaw_rate = 0.0
        if self.prev_ego_heading_deg is not None and self.prev_ego_stamp is not None:
            dt = (stamp - self.prev_ego_stamp).to_sec()
            if dt > 1e-6:
                heading_rate_deg_s = self.normalize_deg_diff(
                    heading_deg - self.prev_ego_heading_deg
                ) / dt
                yaw_rate = math.radians(heading_rate_deg_s)
        self.prev_ego_heading_deg = heading_deg
        self.prev_ego_stamp = stamp
        return yaw_rate

    def ego_callback(self, msg: EgoVehicleStatus):
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        self.last_ego_stamp = stamp
        odom = Odometry()

        # MORAI header stamp를 그대로 사용.
        # rosbag play --clock 환경에서도 시간 동기 확인이 쉬움.
        odom.header.stamp = stamp
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

        angular_velocity = getattr(msg, "angular_velocity", None)
        if angular_velocity is not None:
            # Older local bags used an expanded EgoVehicleStatus with angular_velocity.
            odom.twist.twist.angular.x = angular_velocity.x
            odom.twist.twist.angular.y = angular_velocity.y
            odom.twist.twist.angular.z = angular_velocity.z
        else:
            # Competition EgoVehicleStatus exposes heading but not angular_velocity.
            odom.twist.twist.angular.z = self.estimate_yaw_rate(msg.heading, stamp)

        self.publish_odom(odom)

    def gps_callback(self, msg: GPSMessage):
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        if self.last_ego_stamp is not None:
            dt = abs((stamp - self.last_ego_stamp).to_sec())
            if dt <= self.gps_ego_timeout_sec:
                return
        if msg.status <= 0:
            rospy.logwarn_throttle(2.0, "Skipping GPS fallback with invalid status=%d", msg.status)
            return
        if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
            rospy.logwarn_throttle(2.0, "Skipping GPS fallback with non-finite lat/lon/alt")
            return

        east_offset = self.choose_offset(msg.eastOffset, self.gps_fallback_east_offset)
        north_offset = self.choose_offset(msg.northOffset, self.gps_fallback_north_offset)
        easting, northing = wgs84_to_utm(msg.latitude, msg.longitude, self.gps_utm_zone)
        x = easting - east_offset
        y = northing - north_offset
        z = msg.altitude - self.gps_morai_origin_altitude - self.gps_sensor_z_offset

        if self.last_gps_xy is not None:
            dx = x - self.last_gps_xy[0]
            dy = y - self.last_gps_xy[1]
            if math.hypot(dx, dy) >= self.gps_min_heading_distance:
                self.last_gps_yaw = math.atan2(dy, dx)
        self.last_gps_xy = (x, y)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, self.last_gps_yaw)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.publish_odom(odom)

    def choose_offset(self, message_offset, fallback_offset):
        if self.gps_use_message_offsets and math.isfinite(message_offset) and abs(message_offset) > 1.0:
            return message_offset
        return fallback_offset

    def publish_odom(self, odom):
        if self.tf_broadcaster is not None and odom.header.stamp != self.last_tf_stamp:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
            self.last_tf_stamp = odom.header.stamp

        self.odom_pub.publish(odom)


def main():
    rospy.init_node("morai_ego_to_odom")
    MoraiEgoToOdom()
    rospy.spin()


if __name__ == "__main__":
    main()
