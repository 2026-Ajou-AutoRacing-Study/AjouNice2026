#!/usr/bin/env python3

import copy
import json
import math
import os
import threading

import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import TwistWithCovarianceStamped
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


DEFAULT_NOISE_NS = "/localization_manager/providers/gnss_imu/noise"
WGS84_A = 6378137.0
WGS84_INV_F = 298.257223563
UTM_SCALE = 0.9996


def normalized_frame(frame):
    return frame.lstrip("/")


def is_zero_geodetic_position(latitude, longitude, epsilon):
    """Return whether MORAI is using its all-zero GNSS outage sentinel."""
    return abs(latitude) <= epsilon and abs(longitude) <= epsilon


def quantized_cell(latitude, longitude, step_deg):
    """Return the MORAI latitude/longitude lattice cell indices."""
    return (int(round(latitude / step_deg)), int(round(longitude / step_deg)))


def should_publish_quantized_fix(mode, cell, last_published_cell):
    return mode == "off" or cell != last_published_cell


def resolve_motion_compensation_sec(delay_ms, legacy_sec):
    """Select one unit without adding the same compensation twice.

    An empty ms argument preserves old launch/campaign files. Positive values
    advance the GNSS position using motion; they never delay a callback or
    rewrite a receipt-time header. Both interfaces share the existing 100ms cap.
    """
    if isinstance(legacy_sec, bool):
        raise ValueError("motion_compensation_sec must be a number, not bool")
    seconds = float(legacy_sec)
    if not math.isfinite(seconds) or not 0.0 <= seconds <= 0.1:
        raise ValueError("motion_compensation_sec must be finite and in [0, 0.1] seconds")
    if delay_ms == "":
        return seconds
    if isinstance(delay_ms, bool):
        raise ValueError("gnss_delay_ms must be a number, not bool")
    milliseconds = float(delay_ms)
    if not math.isfinite(milliseconds) or not 0.0 <= milliseconds <= 100.0:
        raise ValueError("gnss_delay_ms must be finite and in [0, 100] milliseconds")
    if seconds != 0.0:
        raise ValueError("select gnss_delay_ms OR nonzero motion_compensation_sec, not both")
    return milliseconds / 1000.0


def motion_compensation_delta(
    compensation_sec, gps_stamp_sec, motion, orientation, max_age_sec, base_frame
):
    """Return an optional map-plane delta, never a replacement timestamp.

    Only the canonical GT motion adapter pair is accepted: body-frame vx and
    an IMU message whose header names its base_link sensor frame, but whose
    normalized quaternion already describes base_link orientation in map.
    Applying another heading offset or map-from-morai rotation is incorrect.
    No oracle, raw Ego position, or unprovided Competition vy is used here.
    """
    if compensation_sec == 0.0:
        return (0.0, 0.0), "disabled"
    if motion is None or orientation is None:
        return (0.0, 0.0), "missing_motion"
    try:
        if (
            normalized_frame(motion.header.frame_id) != base_frame
            or normalized_frame(orientation.header.frame_id) != base_frame
        ):
            return (0.0, 0.0), "wrong_motion_frame"
        if motion.header.stamp != orientation.header.stamp:
            return (0.0, 0.0), "unpaired_motion_stamps"
        source_stamp_sec = float(motion.header.stamp.to_sec())
        age_sec = float(gps_stamp_sec) - source_stamp_sec
        if not math.isfinite(source_stamp_sec) or source_stamp_sec <= 0.0:
            return (0.0, 0.0), "invalid_motion_stamp"
        if not math.isfinite(age_sec) or age_sec < 0.0 or age_sec > max_age_sec:
            return (0.0, 0.0), "stale_or_future_motion"
        vx = float(motion.twist.twist.linear.x)
        q = orientation.orientation
        quaternion = tuple(float(v) for v in (q.x, q.y, q.z, q.w))
        if not math.isfinite(vx) or not all(math.isfinite(v) for v in quaternion):
            return (0.0, 0.0), "nonfinite_motion"
        if orientation.orientation_covariance[0] < 0.0:
            return (0.0, 0.0), "unavailable_orientation"
        norm = math.hypot(*quaternion)
        if not math.isfinite(norm) or abs(norm - 1.0) > 1.0e-3:
            return (0.0, 0.0), "unnormalized_orientation"
        x, y, z, w = (v / norm for v in quaternion)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y*y + z*z))
        distance = compensation_sec * vx
        return (distance * math.cos(yaw), distance * math.sin(yaw)), "applied"
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        return (0.0, 0.0), "malformed_motion"


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
        self.reject_zero_position = bool(
            rospy.get_param("~reject_zero_position", True)
        )
        self.zero_position_epsilon_deg = float(
            rospy.get_param("~zero_position_epsilon_deg", 1.0e-9)
        )
        if (
            not math.isfinite(self.zero_position_epsilon_deg)
            or self.zero_position_epsilon_deg < 0.0
        ):
            raise rospy.ROSInitException(
                "~zero_position_epsilon_deg must be finite and non-negative"
            )
        self.reject_duplicate_stamps = bool(
            rospy.get_param("~reject_duplicate_stamps", True)
        )
        self.quantization_input_mode = rospy.get_param(
            "~quantization_input_mode", "off"
        )
        if self.quantization_input_mode not in ("off", "cell_transition"):
            raise rospy.ROSInitException(
                "~quantization_input_mode must be off or cell_transition"
            )
        self.quantization_step_deg = float(
            rospy.get_param("~quantization_step_deg", 1.0 / 600000.0)
        )
        if not math.isfinite(self.quantization_step_deg) or self.quantization_step_deg <= 0:
            raise rospy.ROSInitException(
                "~quantization_step_deg must be positive and finite"
            )
        self.tf_timeout = float(rospy.get_param("~tf_timeout_sec", 0.1))
        self.noise_ns = rospy.get_param(
            "~noise_namespace", DEFAULT_NOISE_NS
        )
        try:
            self.motion_compensation_sec = resolve_motion_compensation_sec(
                rospy.get_param("~gnss_delay_ms", ""),
                rospy.get_param("~motion_compensation_sec", 0.0),
            )
        except (TypeError, ValueError, OverflowError) as error:
            raise rospy.ROSInitException(str(error)) from error
        self.max_motion_age_sec = float(
            rospy.get_param("~max_motion_age_sec", 0.1)
        )
        if not math.isfinite(self.max_motion_age_sec) or self.max_motion_age_sec <= 0.0:
            raise rospy.ROSInitException("~max_motion_age_sec must be positive and finite")
        self.motion_topic = rospy.get_param(
            "~motion_topic", "/localization/gt/twist"
        )
        self.orientation_topic = rospy.get_param(
            "~orientation_topic", "/localization/gt/orientation"
        )
        if not all(
            isinstance(v, str) and v.strip()
            for v in (self.motion_topic, self.orientation_topic)
        ):
            raise rospy.ROSInitException("Motion/orientation topics must be non-empty strings")
        self.motion_lock = threading.Lock()
        self.latest_motion = None
        self.latest_orientation = None

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
        self.last_published_cell = None
        self.outage_start_stamp = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=100
        )
        # Default zero remains the original converter: no auxiliary subscriptions
        # or additional input requirements. Opt-in failure retains the raw fix.
        self.motion_subscriber = None
        self.orientation_subscriber = None
        if self.motion_compensation_sec > 0.0:
            self.motion_subscriber = rospy.Subscriber(
                self.motion_topic, TwistWithCovarianceStamped,
                self.motion_callback, queue_size=100,
            )
            self.orientation_subscriber = rospy.Subscriber(
                self.orientation_topic, Imu,
                self.orientation_callback, queue_size=100,
            )
        self.subscriber = rospy.Subscriber(
            self.input_topic, GPSMessage, self.gps_callback, queue_size=100
        )

        rospy.loginfo(
            "morai_gps_to_local_odometry: %s -> %s; UTM %d%s, "
            "fallback offset=(%.3f, %.3f), sigma_xy=%.3fm, quantization=%s, "
            "motion_compensation=%.3fs",
            self.input_topic,
            self.output_topic,
            self.utm_zone,
            "N" if self.northern_hemisphere else "S",
            self.fallback_east_offset,
            self.fallback_north_offset,
            horizontal_stddev,
            self.quantization_input_mode,
            self.motion_compensation_sec,
        )
        rospy.loginfo(
            "GNSS advance %.1fms; receipt stamps unchanged; no added wait",
            self.motion_compensation_sec * 1000.0,
        )

    def motion_callback(self, message):
        with self.motion_lock:
            self.latest_motion = message

    def orientation_callback(self, message):
        with self.motion_lock:
            self.latest_orientation = message

    def compensate_map_point(self, point, stamp):
        if self.motion_compensation_sec == 0.0:
            return point
        with self.motion_lock:
            motion, orientation = self.latest_motion, self.latest_orientation
        delta, reason = motion_compensation_delta(
            self.motion_compensation_sec, stamp.to_sec(), motion, orientation,
            self.max_motion_age_sec, self.base_frame,
        )
        if reason != "applied":
            rospy.logwarn_throttle(
                2.0, "GNSS motion compensation fallback (%s): raw fix retained", reason
            )
            return point
        return point[0] + delta[0], point[1] + delta[1], point[2]

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
        if self.reject_zero_position and is_zero_geodetic_position(
            message.latitude,
            message.longitude,
            self.zero_position_epsilon_deg,
        ):
            if self.outage_start_stamp is None:
                self.outage_start_stamp = message.header.stamp
                self.last_published_cell = None
                rospy.logwarn(
                    "GNSS outage started: rejecting zero latitude/longitude "
                    "from /sensor/gps"
                )
            return
        if not (-90.0 <= message.latitude <= 90.0) or not (
            -180.0 <= message.longitude <= 180.0
        ):
            rospy.logwarn_throttle(2.0, "Rejecting invalid MORAI latitude/longitude")
            return
        if self.outage_start_stamp is not None:
            outage_duration = (
                message.header.stamp - self.outage_start_stamp
            ).to_sec()
            rospy.loginfo(
                "GNSS outage ended after %.3f sec; resuming position updates",
                max(0.0, outage_duration),
            )
            self.outage_start_stamp = None

        cell = quantized_cell(
            message.latitude, message.longitude, self.quantization_step_deg
        )
        if not should_publish_quantized_fix(
            self.quantization_input_mode, cell, self.last_published_cell
        ):
            # Repeated lattice output is not a new independent position fix.
            # Keep the observed timestamp monotonic, but preserve the prior
            # published position until the cell changes. This is experimental.
            self.last_stamp = message.header.stamp
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
        map_point = self.compensate_map_point(map_point, message.header.stamp)
        output = Odometry()
        output.header = copy.deepcopy(message.header)
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
        self.last_published_cell = cell
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_gps_to_local_odometry")
    MoraiGpsToLocalOdometry()
    rospy.spin()


if __name__ == "__main__":
    main()
