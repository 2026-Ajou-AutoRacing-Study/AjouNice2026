#!/usr/bin/env python3

"""Opt-in stationary GNSS re-anchor; never an automatic teleport oracle.

GNSS innovation alone cannot distinguish a checkpoint move from outage drift.
Production remains disabled/monitor-only. An active trial requires fresh valid
GNSS, corrected heading and stationary allowed vx. A published reset must be
acknowledged; timeout prevents automatic retries until explicit node restart.
"""

import copy
import json
import math
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def finite_number(value, name, positive=False, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite number".format(name))
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0.0) or (
        nonnegative and value < 0.0
    ):
        raise ValueError("{} is outside its finite valid range".format(name))
    return value


def normalized_frame(value):
    if not isinstance(value, str) or not value.lstrip("/").strip():
        raise ValueError("frame must be a non-empty string")
    return value.lstrip("/")


def normalized_quaternion(value):
    values = (value.x, value.y, value.z, value.w)
    if not all(math.isfinite(component) for component in values):
        raise ValueError("orientation must be finite")
    norm = math.sqrt(sum(component * component for component in values))
    if norm < 1.0e-9 or abs(norm - 1.0) > 0.01:
        raise ValueError("orientation must be a near-unit, non-zero quaternion")
    result = copy.deepcopy(value)
    result.x, result.y, result.z, result.w = (
        component / norm for component in values
    )
    return result


def quaternion_yaw(value):
    q = normalized_quaternion(value)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def finite_covariance(values, size):
    width = 6 if size == 36 else 3
    return len(values) == size and all(math.isfinite(x) for x in values) and all(
        values[i * (width + 1)] >= 0.0 for i in range(width)
    )


def finite_pose(position, orientation):
    if not all(math.isfinite(value) for value in (position.x, position.y, position.z)):
        return False
    try:
        normalized_quaternion(orientation)
    except ValueError:
        return False
    return True


class RelocationDetector:
    """Consecutive valid innovations; detection and committed action are separate."""

    def __init__(self, threshold_m=5.0, consecutive_samples=3, cooldown_sec=5.0,
                 max_sample_gap_sec=0.5, reacquisition_grace_sec=0.0,
                 auto_commit=True):
        self.threshold_m = finite_number(threshold_m, "threshold_m", positive=True)
        if type(consecutive_samples) is not int or consecutive_samples < 1:
            raise ValueError("consecutive_samples must be a positive integer")
        if type(auto_commit) is not bool:
            raise ValueError("auto_commit must be a bool")
        self.consecutive_samples = consecutive_samples
        self.cooldown_sec = finite_number(cooldown_sec, "cooldown_sec", nonnegative=True)
        self.max_sample_gap_sec = finite_number(max_sample_gap_sec, "max_sample_gap_sec", positive=True)
        self.reacquisition_grace_sec = finite_number(reacquisition_grace_sec, "reacquisition_grace_sec", nonnegative=True)
        self.auto_commit = auto_commit
        self.count = 0
        self.last_trigger_sec = None
        self.last_observation_sec = None
        self.recovery_until_sec = None
        self.awaiting_reacquisition = False
        self.last_reason = "startup"

    def clear_candidate(self, reason):
        self.count = 0
        self.last_reason = reason

    def note_gap(self):
        self.clear_candidate("gnss_gap")
        self.awaiting_reacquisition = self.last_observation_sec is not None

    def commit_trigger(self, stamp_sec):
        stamp_sec = finite_number(stamp_sec, "trigger timestamp", nonnegative=True)
        self.last_trigger_sec = stamp_sec
        self.count = 0

    def observe(self, innovation_m, stamp_sec):
        if not isinstance(stamp_sec, (int, float)) or isinstance(stamp_sec, bool) or (
            not math.isfinite(stamp_sec) or stamp_sec < 0.0
        ):
            self.clear_candidate("invalid_timestamp")
            return False
        if self.last_observation_sec is not None:
            gap_sec = stamp_sec - self.last_observation_sec
            if gap_sec <= 0.0:
                self.clear_candidate("non_monotonic_timestamp")
                return False
            if gap_sec > self.max_sample_gap_sec:
                self.note_gap()
        self.last_observation_sec = stamp_sec
        if self.awaiting_reacquisition:
            self.recovery_until_sec = stamp_sec + self.reacquisition_grace_sec
            self.awaiting_reacquisition = False
        if self.recovery_until_sec is not None and stamp_sec < self.recovery_until_sec:
            self.clear_candidate("reacquisition_grace")
            return False
        if not math.isfinite(innovation_m) or innovation_m < 0.0:
            self.clear_candidate("invalid_innovation")
            return False
        if innovation_m < self.threshold_m:
            self.clear_candidate("below_threshold")
            return False
        if self.last_trigger_sec is not None and stamp_sec - self.last_trigger_sec < self.cooldown_sec:
            self.clear_candidate("cooldown")
            return False
        self.count += 1
        self.last_reason = "candidate"
        if self.count < self.consecutive_samples:
            return False
        self.count = 0
        self.last_reason = "persistent_candidate"
        if self.auto_commit:
            self.commit_trigger(stamp_sec)
        return True


class StationaryGate:
    """A hold on corrected allowed vx; never use withheld Competition vy."""

    def __init__(self, speed_max_mps=0.05, hold_sec=1.0, max_gap_sec=0.1):
        self.speed_max_mps = finite_number(speed_max_mps, "stationary_speed_max_mps", nonnegative=True)
        self.hold_sec = finite_number(hold_sec, "stationary_hold_sec", nonnegative=True)
        self.max_gap_sec = finite_number(max_gap_sec, "max_motion_age_sec", positive=True)
        self.last_stamp = None
        self.start_stamp = None

    def clear(self):
        self.start_stamp = None

    def observe(self, vx_mps, stamp_sec):
        if not math.isfinite(vx_mps) or not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            self.clear()
            return False
        if self.last_stamp is not None:
            dt = stamp_sec - self.last_stamp
            if dt <= 0.0:
                self.clear()
                return False
            if dt > self.max_gap_sec:
                self.clear()
        self.last_stamp = stamp_sec
        if abs(vx_mps) > self.speed_max_mps:
            self.clear()
            return False
        if self.start_stamp is None:
            self.start_stamp = stamp_sec
        return self.ready(stamp_sec)

    def ready(self, stamp_sec):
        return self.start_stamp is not None and self.last_stamp is not None and (
            abs(stamp_sec - self.last_stamp) <= self.max_gap_sec
            and stamp_sec - self.start_stamp >= self.hold_sec
        )


class ResetAcknowledgement:
    """One pending action; timeout prohibits retries until an explicit restart."""

    def __init__(self, timeout_sec=2.0, position_tolerance_m=0.2,
                 yaw_tolerance_rad=math.radians(5.0), consecutive_samples=2):
        self.timeout_sec = finite_number(timeout_sec, "reset_ack_timeout_sec", positive=True)
        self.position_tolerance_m = finite_number(position_tolerance_m, "reset_ack_position_tolerance_m", positive=True)
        self.yaw_tolerance_rad = finite_number(yaw_tolerance_rad, "reset_ack_yaw_tolerance_rad", positive=True)
        if type(consecutive_samples) is not int or consecutive_samples < 1:
            raise ValueError("reset_ack_consecutive_samples must be a positive integer")
        self.consecutive_samples = consecutive_samples
        self.pending = None
        self.last_state_stamp = None
        self.count = 0
        self.blocked = False

    def begin(self, stamp_sec, x, y, yaw):
        values = [finite_number(value, "reset target") for value in (stamp_sec, x, y, yaw)]
        if stamp_sec <= 0.0 or self.pending is not None or self.blocked:
            raise ValueError("reset unavailable while pending or blocked")
        self.pending = tuple(values)
        self.last_state_stamp = None
        self.count = 0

    def tick(self, stamp_sec):
        if self.pending is not None and stamp_sec - self.pending[0] >= self.timeout_sec:
            self.pending = None
            self.blocked = True
            self.count = 0
            return "timeout_blocked"
        return "pending" if self.pending is not None else ("blocked" if self.blocked else "idle")

    def observe(self, stamp_sec, x, y, yaw):
        if not all(math.isfinite(value) for value in (stamp_sec, x, y, yaw)):
            return self.tick(stamp_sec) if math.isfinite(stamp_sec) else "pending"
        status = self.tick(stamp_sec)
        if self.pending is None:
            return status
        if stamp_sec <= self.pending[0] or (
            self.last_state_stamp is not None and stamp_sec <= self.last_state_stamp
        ):
            return "pending"
        self.last_state_stamp = stamp_sec
        _, tx, ty, tyaw = self.pending
        yaw_error = math.atan2(math.sin(yaw - tyaw), math.cos(yaw - tyaw))
        if math.hypot(x - tx, y - ty) > self.position_tolerance_m or abs(yaw_error) > self.yaw_tolerance_rad:
            self.count = 0
            return "pending"
        self.count += 1
        if self.count < self.consecutive_samples:
            return "pending"
        self.pending = None
        self.count = 0
        return "acknowledged"


class GnssStationaryGate:
    """A bounded valid-GNSS position window supplements the allowed vx hold.

    The span tolerates the measured ~0.24m quantization-cell diagonal. It is
    not a proof of zero lateral slip below that resolution and is never used
    as a fused velocity measurement.
    """

    def __init__(self, hold_sec=1.0, max_span_m=0.3, max_gap_sec=0.5):
        self.hold_sec = finite_number(hold_sec, "gnss_stationary_hold_sec", nonnegative=True)
        self.max_span_m = finite_number(max_span_m, "gnss_stationary_max_span_m", positive=True)
        self.max_gap_sec = finite_number(max_gap_sec, "GNSS stationary max gap", positive=True)
        self.samples = deque()
        self.start_stamp = None
        self.last_stamp = None
        self.span_m = None

    def clear(self):
        self.samples.clear()
        self.start_stamp = None
        self.last_stamp = None
        self.span_m = None

    def observe(self, x, y, stamp_sec):
        if not all(math.isfinite(value) for value in (x, y, stamp_sec)) or stamp_sec <= 0.0:
            self.clear()
            return False
        if self.last_stamp is not None:
            dt = stamp_sec - self.last_stamp
            if dt <= 0.0:
                self.clear()
                return False
            if dt > self.max_gap_sec:
                self.clear()
        self.last_stamp = stamp_sec
        if self.start_stamp is None:
            self.start_stamp = stamp_sec
        self.samples.append((stamp_sec, x, y))
        while len(self.samples) > 1 and self.samples[0][0] < stamp_sec - self.hold_sec:
            self.samples.popleft()
        xs = [sample[1] for sample in self.samples]
        ys = [sample[2] for sample in self.samples]
        self.span_m = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if self.span_m > self.max_span_m:
            self.samples.clear()
            self.samples.append((stamp_sec, x, y))
            self.start_stamp = stamp_sec
            self.span_m = 0.0
            return False
        return self.ready(stamp_sec)

    def ready(self, stamp_sec):
        return self.start_stamp is not None and self.last_stamp is not None and (
            abs(stamp_sec - self.last_stamp) <= self.max_gap_sec
            and stamp_sec - self.start_stamp >= self.hold_sec
        )


class LocalizationRelocationGuard:
    def __init__(self):
        self.lock = threading.Lock()
        try:
            self.monitor_only = rospy.get_param("~monitor_only", True)
            if type(self.monitor_only) is not bool:
                raise ValueError("monitor_only must be a bool")
            self.map_frame = normalized_frame(rospy.get_param("~map_frame", "map"))
            self.base_frame = normalized_frame(rospy.get_param("~base_frame", "base_link"))
            self.max_state_age_sec = finite_number(rospy.get_param("~max_state_age_sec", 0.1), "max_state_age_sec", positive=True)
            self.max_orientation_age_sec = finite_number(rospy.get_param("~max_orientation_age_sec", 0.1), "max_orientation_age_sec", positive=True)
            # Only reset eligibility is gated; the core sensor/localization
            # timestamp policy and input streams are never modified here.
            self.max_input_age_sec = finite_number(rospy.get_param("~max_input_age_sec", 0.5), "max_input_age_sec", positive=True)
            self.future_tolerance_sec = finite_number(rospy.get_param("~future_tolerance_sec", 0.05), "future_tolerance_sec", nonnegative=True)
            self.detector = RelocationDetector(
                threshold_m=rospy.get_param("~innovation_threshold_m", 5.0),
                consecutive_samples=rospy.get_param("~consecutive_samples", 3),
                cooldown_sec=rospy.get_param("~cooldown_sec", 5.0),
                max_sample_gap_sec=rospy.get_param("~max_sample_gap_sec", 0.5),
                reacquisition_grace_sec=rospy.get_param("~reacquisition_grace_sec", 1.0),
                auto_commit=False,
            )
            self.stationary = StationaryGate(
                rospy.get_param("~stationary_speed_max_mps", 0.05),
                rospy.get_param("~stationary_hold_sec", 1.0),
                rospy.get_param("~max_motion_age_sec", 0.1),
            )
            self.gnss_stationary = GnssStationaryGate(
                rospy.get_param("~gnss_stationary_hold_sec", 1.0),
                rospy.get_param("~gnss_stationary_max_span_m", 0.3),
                self.detector.max_sample_gap_sec,
            )
            self.acknowledgement = ResetAcknowledgement(
                rospy.get_param("~reset_ack_timeout_sec", 2.0),
                rospy.get_param("~reset_ack_position_tolerance_m", 0.2),
                math.radians(finite_number(rospy.get_param("~reset_ack_yaw_tolerance_deg", 5.0), "reset_ack_yaw_tolerance_deg", positive=True)),
                rospy.get_param("~reset_ack_consecutive_samples", 2),
            )
            position_stddev = finite_number(rospy.get_param("~reset_position_stddev_m", 0.1), "reset_position_stddev_m", positive=True)
            yaw_stddev = math.radians(finite_number(rospy.get_param("~reset_yaw_stddev_deg", 1.0), "reset_yaw_stddev_deg", positive=True))
            position_variance = finite_number(position_stddev * position_stddev, "reset position variance", positive=True)
            yaw_variance = finite_number(yaw_stddev * yaw_stddev, "reset yaw variance", positive=True)
        except ValueError as error:
            raise rospy.ROSInitException(str(error))
        self.covariance = [0.0] * 36
        self.covariance[0] = position_variance
        self.covariance[7] = position_variance
        for index in (14, 21, 28):
            self.covariance[index] = 1.0e6
        self.covariance[35] = yaw_variance
        self.latest_state = None
        self.latest_state_is_valid = False
        self.latest_orientation = None
        self.latest_motion = None
        self.last_gnss_stamp = None
        self.last_status = None
        self.last_status_stamp = None
        self.reset_publisher = rospy.Publisher(rospy.get_param("~reset_topic", "/localization/initialpose"), PoseWithCovarianceStamped, queue_size=1, latch=False)
        self.status_publisher = rospy.Publisher(rospy.get_param("~status_topic", "/localization/relocation/status"), String, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber(rospy.get_param("~state_topic", "/localization/kinematic_state"), Odometry, self.state_callback, queue_size=100),
            rospy.Subscriber(rospy.get_param("~orientation_topic", "/localization/gt/orientation"), Imu, self.orientation_callback, queue_size=100),
            rospy.Subscriber(rospy.get_param("~motion_topic", "/localization/gt/twist"), TwistWithCovarianceStamped, self.motion_callback, queue_size=100),
            rospy.Subscriber(rospy.get_param("~gnss_topic", "/localization/gnss/odom"), Odometry, self.gnss_callback, queue_size=100),
        ]
        self.watchdog = rospy.Timer(rospy.Duration(0.1), self.watchdog_callback)
        rospy.loginfo("localization_relocation_guard monitor_only=%s threshold=%.2fm; active reset requires stationary corrected motion, recovery grace and Global acknowledgement", self.monitor_only, self.detector.threshold_m)

    @staticmethod
    def stamp_sec(message):
        return message.header.stamp.to_sec()

    def valid_header_time(self, message):
        stamp = self.stamp_sec(message)
        now = rospy.Time.now().to_sec()
        return math.isfinite(stamp) and stamp > 0.0 and math.isfinite(now) and now > 0.0 and (
            -self.future_tolerance_sec <= now - stamp <= self.max_input_age_sec
        )

    def valid_odometry(self, message):
        stamp = self.stamp_sec(message)
        return math.isfinite(stamp) and stamp > 0.0 and self.valid_header_time(message) and (
            message.header.frame_id.lstrip("/") == self.map_frame
            and message.child_frame_id.lstrip("/") == self.base_frame
            and finite_pose(message.pose.pose.position, message.pose.pose.orientation)
            and finite_covariance(message.pose.covariance, 36)
            and finite_covariance(message.twist.covariance, 36)
            and all(math.isfinite(value) for value in (
                message.twist.twist.linear.x, message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            ))
        )

    def status_locked(self, stamp_sec, state, reason, innovation=None, force=False,
                      stamp_source="gnss_header"):
        key = (state, reason)
        if not force and key == self.last_status and self.last_status_stamp is not None and stamp_sec - self.last_status_stamp < 0.1:
            return
        payload = {"schema_version": 1, "stamp_sec": stamp_sec, "state": state,
                   "stamp_source": stamp_source,
                   "reason": reason, "monitor_only": self.monitor_only,
                   "candidate_count": self.detector.count,
                   "innovation_m": innovation if innovation is not None and math.isfinite(innovation) else None,
                   "stationary": self.stationary.ready(stamp_sec),
                   "gnss_stationary": self.gnss_stationary.ready(stamp_sec),
                   "gnss_span_m": self.gnss_stationary.span_m,
                   "reset_pending": self.acknowledgement.pending is not None,
                   "reset_blocked": self.acknowledgement.blocked}
        self.status_publisher.publish(String(data=json.dumps(payload, allow_nan=False, sort_keys=True)))
        self.last_status, self.last_status_stamp = key, stamp_sec

    def state_callback(self, message):
        if not self.valid_odometry(message):
            with self.lock:
                self.latest_state_is_valid = False
                self.acknowledgement.count = 0
            return
        stamp = self.stamp_sec(message)
        with self.lock:
            if self.latest_state is not None and stamp <= self.stamp_sec(self.latest_state):
                return
            self.latest_state = message
            self.latest_state_is_valid = True
            p = message.pose.pose.position
            status = self.acknowledgement.observe(stamp, p.x, p.y, quaternion_yaw(message.pose.pose.orientation))
            if status in ("acknowledged", "timeout_blocked"):
                self.status_locked(stamp, status, "global_state_confirmation", force=True,
                                   stamp_source="global_state_header")
                if status == "timeout_blocked":
                    rospy.logerr("Relocation reset was not acknowledged; further resets are blocked until restart")

    def orientation_callback(self, message):
        stamp = self.stamp_sec(message)
        if not math.isfinite(stamp) or stamp <= 0.0 or not self.valid_header_time(message) or message.header.frame_id.lstrip("/") != self.base_frame or not finite_covariance(message.orientation_covariance, 9):
            with self.lock:
                self.latest_orientation = None
            return
        try:
            normalized_quaternion(message.orientation)
        except ValueError:
            with self.lock:
                self.latest_orientation = None
            return
        with self.lock:
            if self.latest_orientation is None or stamp > self.stamp_sec(self.latest_orientation):
                self.latest_orientation = message

    def motion_callback(self, message):
        stamp = self.stamp_sec(message)
        valid = math.isfinite(stamp) and stamp > 0.0 and self.valid_header_time(message) and message.header.frame_id.lstrip("/") == self.base_frame and finite_covariance(message.twist.covariance, 36) and math.isfinite(message.twist.twist.linear.x)
        with self.lock:
            if not valid:
                self.stationary.clear()
                return
            if self.latest_motion is not None and stamp <= self.stamp_sec(self.latest_motion):
                self.stationary.clear()
                return
            self.latest_motion = message
            self.stationary.observe(message.twist.twist.linear.x, stamp)

    def watchdog_callback(self, _event):
        stamp = rospy.Time.now().to_sec()
        if not math.isfinite(stamp) or stamp <= 0.0:
            return
        with self.lock:
            status = self.acknowledgement.tick(stamp)
            if status == "timeout_blocked":
                self.status_locked(stamp, status, "reset_confirmation_timeout", force=True,
                                   stamp_source="ros_now_watchdog")
                rospy.logerr("Relocation reset acknowledgement timed out; automatic retries blocked")
            if self.last_gnss_stamp is not None and stamp - self.last_gnss_stamp > self.detector.max_sample_gap_sec:
                self.detector.note_gap()
                self.gnss_stationary.clear()
                self.status_locked(stamp, "gnss_outage", "no_valid_gnss", force=False,
                                   stamp_source="ros_now_watchdog")

    def gnss_callback(self, message):
        stamp = self.stamp_sec(message)
        with self.lock:
            if not self.valid_odometry(message):
                self.detector.clear_candidate("invalid_gnss")
                self.gnss_stationary.clear()
                return
            if self.last_gnss_stamp is not None and stamp <= self.last_gnss_stamp:
                self.detector.clear_candidate("non_monotonic_gnss")
                self.gnss_stationary.clear()
                self.status_locked(stamp, "rejected", "duplicate_or_backward_gnss")
                return
            self.last_gnss_stamp = stamp
            self.gnss_stationary.observe(message.pose.pose.position.x,
                                         message.pose.pose.position.y, stamp)
            state, orientation = self.latest_state, self.latest_orientation
            if state is None or not self.latest_state_is_valid or abs(stamp - self.stamp_sec(state)) > self.max_state_age_sec:
                self.detector.observe(math.nan, stamp)
                self.status_locked(stamp, "waiting", "co_timed_state_unavailable")
                return
            dt = stamp - self.stamp_sec(state)
            yaw = quaternion_yaw(state.pose.pose.orientation)
            vx, vy = state.twist.twist.linear.x, state.twist.twist.linear.y
            sx = state.pose.pose.position.x + (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
            sy = state.pose.pose.position.y + (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
            innovation = math.hypot(message.pose.pose.position.x - sx, message.pose.pose.position.y - sy)
            ack_status = self.acknowledgement.tick(stamp)
            if self.acknowledgement.pending is not None or self.acknowledgement.blocked:
                self.detector.observe(math.nan, stamp)
                self.status_locked(stamp, ack_status, "reset_confirmation_pending_or_blocked", innovation)
                return
            if not self.monitor_only:
                reason = None
                if orientation is None or abs(stamp - self.stamp_sec(orientation)) > self.max_orientation_age_sec:
                    reason = "fresh_corrected_heading_unavailable"
                elif not self.stationary.ready(stamp):
                    reason = "stationary_motion_hold_not_met"
                elif not self.gnss_stationary.ready(stamp):
                    reason = "stationary_gnss_span_hold_not_met"
                elif self.reset_publisher.get_num_connections() <= 0:
                    reason = "reset_subscriber_unavailable"
                if reason is not None:
                    self.detector.observe(math.nan, stamp)
                    self.status_locked(stamp, "active_blocked", reason, innovation)
                    return
            if not self.detector.observe(innovation, stamp):
                self.status_locked(stamp, "monitoring", self.detector.last_reason, innovation)
                return
            if self.monitor_only:
                self.detector.commit_trigger(stamp)
                self.status_locked(stamp, "monitor_candidate", "persistent_large_innovation_not_teleport_proof", innovation, force=True)
                rospy.logwarn("MONITOR ONLY: persistent GNSS/Global innovation %.3fm; no reset", innovation)
                return
            reset = PoseWithCovarianceStamped()
            reset.header.stamp = message.header.stamp
            reset.header.frame_id = self.map_frame
            reset.pose.pose.position = copy.deepcopy(message.pose.pose.position)
            reset.pose.pose.orientation = normalized_quaternion(orientation.orientation)
            reset.pose.covariance = list(self.covariance)
            try:
                self.reset_publisher.publish(reset)
            except rospy.ROSException as error:
                # A serialization/connection failure is not a reason for an
                # endless publication loop. No cooldown was consumed because
                # nothing was published; an explicit restart is required.
                self.acknowledgement.blocked = True
                self.status_locked(stamp, "active_blocked", "reset_publication_failed", innovation, force=True)
                rospy.logerr("Relocation reset not published: %s", error)
                return
            self.detector.commit_trigger(stamp)
            self.acknowledgement.begin(stamp, reset.pose.pose.position.x, reset.pose.pose.position.y, quaternion_yaw(reset.pose.pose.orientation))
            self.gnss_stationary.clear()
            self.status_locked(stamp, "reset_pending", "stationary_reanchor_published", innovation, force=True)
            rospy.logwarn("Published stationary-only Global re-anchor, innovation %.3fm; awaiting acknowledgement", innovation)


def main():
    rospy.init_node("localization_relocation_guard")
    LocalizationRelocationGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
