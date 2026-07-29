#!/usr/bin/env python3

import json
import math
import os

import rospy
import tf.transformations as tft
import tf2_ros
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry


DEFAULT_NOISE_NS = "/localization_manager/providers/gnss_imu/noise"
WGS84_A = 6378137.0
WGS84_INV_F = 298.257223563
UTM_SCALE = 0.9996


def normalized_frame(frame):
    return frame.lstrip("/")


def wgs84_to_utm(latitude_deg, longitude_deg, zone, northern_hemisphere):
    f = 1.0 / WGS84_INV_F
    e2 = f * (2.0 - f)
    ep2 = e2 / (1.0 - e2)

    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    central_meridian = math.radians((zone - 1) * 6 - 180 + 3)

    sin_lat = math.sin(latitude)
    cos_lat = math.cos(latitude)
    tan_lat = math.tan(latitude)

    n = WGS84_A / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ep2 * cos_lat * cos_lat
    a = cos_lat * (longitude - central_meridian)

    m = WGS84_A * (
        (
            1.0
            - e2 / 4.0
            - 3.0 * e2**2 / 64.0
            - 5.0 * e2**3 / 256.0
        )
        * latitude
        - (
            3.0 * e2 / 8.0
            + 3.0 * e2**2 / 32.0
            + 45.0 * e2**3 / 1024.0
        )
        * math.sin(2.0 * latitude)
        + (15.0 * e2**2 / 256.0 + 45.0 * e2**3 / 1024.0)
        * math.sin(4.0 * latitude)
        - 35.0 * e2**3 / 3072.0 * math.sin(6.0 * latitude)
    )

    easting = (
        UTM_SCALE
        * n
        * (
            a
            + (1.0 - t + c) * a**3 / 6.0
            + (5.0 - 18.0 * t + t**2 + 72.0 * c - 58.0 * ep2)
            * a**5
            / 120.0
        )
        + 500000.0
    )
    northing = UTM_SCALE * (
        m
        + n
        * tan_lat
        * (
            a**2 / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c**2) * a**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c - 330.0 * ep2)
            * a**6
            / 720.0
        )
    )
    if not northern_hemisphere:
        northing += 10000000.0
    return easting, northing


def apply_transform(point, transform):
    quaternion = transform.transform.rotation
    matrix = tft.quaternion_matrix(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )
    source = [point[0], point[1], point[2], 1.0]
    target = matrix.dot(source)
    target[0] += transform.transform.translation.x
    target[1] += transform.transform.translation.y
    target[2] += transform.transform.translation.z
    return target[0], target[1], target[2]


class MoraiGpsToLocalOdometry:
    """Convert MORAI geodetic GPS into a map-frame position measurement."""

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/sensor/gps")
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/gnss/odom"
        )
        self.map_frame = normalized_frame(
            rospy.get_param("~map_frame", "map")
        )
        self.morai_map_frame = normalized_frame(
            rospy.get_param("~morai_map_frame", "morai_map")
        )
        self.base_frame = normalized_frame(
            rospy.get_param("~base_frame", "base_link")
        )
        self.utm_zone = int(rospy.get_param("~utm_zone", 52))
        self.northern_hemisphere = bool(
            rospy.get_param("~northern_hemisphere", True)
        )
        self.use_message_offsets = bool(
            rospy.get_param("~use_message_offsets", True)
        )
        self.fallback_east_offset = float(
            rospy.get_param("~fallback_east_offset", 302595.0)
        )
        self.fallback_north_offset = float(
            rospy.get_param("~fallback_north_offset", 4124145.0)
        )
        self.morai_origin_altitude = float(
            rospy.get_param("~morai_origin_altitude", 0.0)
        )
        self.require_status_ok = bool(
            rospy.get_param("~require_status_ok", True)
        )
        self.reject_duplicate_stamps = bool(
            rospy.get_param("~reject_duplicate_stamps", True)
        )
        self.tf_timeout = float(rospy.get_param("~tf_timeout_sec", 0.1))
        self.noise_ns = rospy.get_param(
            "~noise_namespace", DEFAULT_NOISE_NS
        )

        horizontal_stddev = self.positive_noise_param(
            "gnss/horizontal_stddev_m"
        )
        vertical_stddev = self.positive_noise_param(
            "gnss/vertical_stddev_m"
        )
        self.horizontal_variance = horizontal_stddev**2
        self.vertical_variance = vertical_stddev**2
        self.gps_z_in_base = self.read_gps_height()
        self.last_stamp = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=100
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, GPSMessage, self.gps_callback, queue_size=100
        )

        rospy.loginfo(
            "morai_gps_to_local_odometry: %s -> %s; UTM %d%s, "
            "fallback offset=(%.3f, %.3f), sigma_xy=%.3fm",
            self.input_topic,
            self.output_topic,
            self.utm_zone,
            "N" if self.northern_hemisphere else "S",
            self.fallback_east_offset,
            self.fallback_north_offset,
            horizontal_stddev,
        )

    def positive_noise_param(self, relative_name):
        value = float(rospy.get_param(self.noise_ns + "/" + relative_name))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException(
                "{} must be a positive finite standard deviation".format(
                    self.noise_ns + "/" + relative_name
                )
            )
        return value

    def read_gps_height(self):
        sensor_config = os.path.abspath(
            os.path.expanduser(rospy.get_param("~sensor_config"))
        )
        with open(sensor_config, "r") as source:
            data = json.load(source)
        gps_list = data.get("GPSList", [])
        if len(gps_list) != 1:
            raise rospy.ROSInitException(
                "Expected exactly one GPS in {}, found {}".format(
                    sensor_config, len(gps_list)
                )
            )
        position = gps_list[0].get("pos", {})
        gps_x = float(position["x"])
        gps_y = float(position["y"])
        gps_z = float(position["z"])
        if abs(gps_x) > 1.0e-6 or abs(gps_y) > 1.0e-6:
            raise rospy.ROSInitException(
                "GNSS x/y lever arm is non-zero; heading-aware base_link "
                "correction is required before using this converter"
            )
        return gps_z

    def choose_offset(self, message_offset, fallback):
        if (
            self.use_message_offsets
            and math.isfinite(message_offset)
            and abs(message_offset) > 1.0
        ):
            return message_offset
        return fallback

    def gps_callback(self, message):
        if message.header.stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Rejecting GPS with zero timestamp")
            return
        if (
            self.reject_duplicate_stamps
            and self.last_stamp is not None
            and message.header.stamp <= self.last_stamp
        ):
            rospy.logwarn_once(
                "Rejecting duplicate/out-of-order MORAI GPS; "
                "subsequent duplicates are suppressed"
            )
            return
        if self.require_status_ok and message.status <= 0:
            rospy.logwarn_throttle(
                2.0, "Rejecting MORAI GPS with status=%d", message.status
            )
            return
        values = (message.latitude, message.longitude, message.altitude)
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(2.0, "Rejecting non-finite MORAI GPS")
            return
        if not (-90.0 <= message.latitude <= 90.0) or not (
            -180.0 <= message.longitude <= 180.0
        ):
            rospy.logwarn_throttle(2.0, "Rejecting invalid MORAI latitude/longitude")
            return

        easting, northing = wgs84_to_utm(
            message.latitude,
            message.longitude,
            self.utm_zone,
            self.northern_hemisphere,
        )
        morai_point = (
            easting
            - self.choose_offset(
                message.eastOffset, self.fallback_east_offset
            ),
            northing
            - self.choose_offset(
                message.northOffset, self.fallback_north_offset
            ),
            message.altitude
            - self.morai_origin_altitude
            - self.gps_z_in_base,
        )

        try:
            map_from_morai = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.morai_map_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF %s -> %s: %s",
                self.morai_map_frame,
                self.map_frame,
                str(error),
            )
            return

        map_point = apply_transform(morai_point, map_from_morai)
        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = map_point[0]
        output.pose.pose.position.y = map_point[1]
        output.pose.pose.position.z = map_point[2]
        output.pose.pose.orientation.w = 1.0

        output.pose.covariance = [0.0] * 36
        output.pose.covariance[0] = self.horizontal_variance
        output.pose.covariance[7] = self.horizontal_variance
        output.pose.covariance[14] = self.vertical_variance
        output.pose.covariance[21] = 1.0e6
        output.pose.covariance[28] = 1.0e6
        output.pose.covariance[35] = 1.0e6
        output.twist.covariance = [0.0] * 36

        self.last_stamp = message.header.stamp
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_gps_to_local_odometry")
    MoraiGpsToLocalOdometry()
    rospy.spin()


if __name__ == "__main__":
    main()
