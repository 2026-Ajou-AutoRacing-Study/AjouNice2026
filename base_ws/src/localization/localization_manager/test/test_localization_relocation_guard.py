#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "localization_relocation_guard.py"
SPEC = importlib.util.spec_from_file_location("localization_relocation_guard", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RelocationDetectorTest(unittest.TestCase):
    def test_single_outlier_does_not_trigger(self):
        detector = MODULE.RelocationDetector(5.0, 3, 5.0)
        self.assertFalse(detector.observe(20.0, 1.0))
        self.assertFalse(detector.observe(0.2, 1.1))
        self.assertEqual(detector.count, 0)

    def test_persistent_relocation_triggers_once(self):
        detector = MODULE.RelocationDetector(5.0, 3, 5.0)
        self.assertFalse(detector.observe(20.0, 1.0))
        self.assertFalse(detector.observe(20.0, 1.1))
        self.assertTrue(detector.observe(20.0, 1.2))
        self.assertFalse(detector.observe(20.0, 1.3))

    def test_cooldown_blocks_repeated_trigger(self):
        detector = MODULE.RelocationDetector(5.0, 2, 5.0)
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertTrue(detector.observe(6.0, 1.1))
        for stamp in (1.2, 1.3, 5.9, 6.0):
            self.assertFalse(detector.observe(6.0, stamp))
        self.assertFalse(detector.observe(6.0, 6.2))
        self.assertTrue(detector.observe(6.0, 6.3))

    def test_invalid_sample_clears_candidate(self):
        detector = MODULE.RelocationDetector(5.0, 2, 0.0)
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertFalse(detector.observe(math.nan, 1.1))
        self.assertFalse(detector.observe(6.0, 1.2))
        self.assertTrue(detector.observe(6.0, 1.3))

    def test_gnss_gap_clears_pre_outage_candidate(self):
        detector = MODULE.RelocationDetector(
            5.0, 3, 5.0, max_sample_gap_sec=0.5
        )
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertEqual(detector.count, 1)
        # The first post-outage sample starts a new consecutive sequence.
        self.assertFalse(detector.observe(6.0, 9.0))
        self.assertEqual(detector.count, 1)
        self.assertFalse(detector.observe(6.0, 9.1))
        self.assertTrue(detector.observe(6.0, 9.2))

    def test_non_monotonic_stamp_clears_candidate(self):
        detector = MODULE.RelocationDetector(
            5.0, 3, 5.0, max_sample_gap_sec=0.5
        )
        self.assertFalse(detector.observe(6.0, 2.0))
        self.assertFalse(detector.observe(6.0, 1.9))
        self.assertEqual(detector.count, 0)
        self.assertEqual(detector.last_observation_sec, 2.0)

    def test_duplicate_stamp_is_never_new_evidence(self):
        detector = MODULE.RelocationDetector(consecutive_samples=2)
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertEqual(detector.count, 0)
        self.assertIsNone(detector.last_trigger_sec)

    def test_reacquisition_grace_cannot_reuse_pre_gap_count(self):
        detector = MODULE.RelocationDetector(reacquisition_grace_sec=1.0)
        self.assertFalse(detector.observe(6.0, 1.0))
        self.assertFalse(detector.observe(6.0, 1.1))
        self.assertFalse(detector.observe(6.0, 9.0))
        for stamp in (9.2, 9.4, 9.6, 9.8):
            self.assertFalse(detector.observe(6.0, stamp))
            self.assertEqual(detector.count, 0)
        self.assertFalse(detector.observe(6.0, 10.0))
        self.assertFalse(detector.observe(6.0, 10.1))
        self.assertTrue(detector.observe(6.0, 10.2))

    def test_unpublished_candidate_does_not_consume_cooldown(self):
        detector = MODULE.RelocationDetector(consecutive_samples=1, auto_commit=False)
        self.assertTrue(detector.observe(6.0, 1.0))
        self.assertIsNone(detector.last_trigger_sec)
        self.assertTrue(detector.observe(6.0, 1.1))
        detector.commit_trigger(1.1)
        self.assertFalse(detector.observe(6.0, 1.2))

    def test_parameter_validation(self):
        with self.assertRaises(ValueError):
            MODULE.RelocationDetector(0.0, 3, 5.0)
        with self.assertRaises(ValueError):
            MODULE.RelocationDetector(5.0, 0, 5.0)
        with self.assertRaises(ValueError):
            MODULE.RelocationDetector(5.0, 3, 5.0, max_sample_gap_sec=0.0)
        for count in (True, 2.5, "3"):
            with self.assertRaises(ValueError):
                MODULE.RelocationDetector(consecutive_samples=count)
        for value in (math.nan, math.inf, -1.0):
            with self.assertRaises(ValueError):
                MODULE.RelocationDetector(reacquisition_grace_sec=value)


class StationaryAndAcknowledgementTest(unittest.TestCase):
    def test_stationary_hold_freshness_and_motion(self):
        gate = MODULE.StationaryGate(0.05, 0.2, 0.1)
        self.assertFalse(gate.observe(0.0, 1.0))
        self.assertFalse(gate.observe(0.0, 1.05))
        self.assertFalse(gate.observe(0.0, 1.1))
        self.assertFalse(gate.observe(0.0, 1.15))
        self.assertTrue(gate.observe(0.0, 1.21))
        self.assertFalse(gate.ready(1.4))
        self.assertFalse(gate.observe(1.0, 1.25))
        self.assertIsNone(gate.start_stamp)

    def test_missed_or_duplicate_motion_breaks_hold(self):
        gate = MODULE.StationaryGate(0.05, 0.05, 0.1)
        self.assertFalse(gate.observe(0.0, 1.0))
        self.assertTrue(gate.observe(0.0, 1.06))
        self.assertFalse(gate.observe(0.0, 1.06))
        self.assertFalse(gate.observe(0.0, 1.3))
        self.assertFalse(gate.observe(math.nan, 1.4))

    def test_ack_needs_two_new_poses_and_wrapped_yaw(self):
        ack = MODULE.ResetAcknowledgement()
        ack.begin(1.0, 20.0, 0.0, math.pi - .01)
        self.assertEqual(ack.observe(1.0, 20.0, 0.0, -math.pi + .01), "pending")
        self.assertEqual(ack.observe(1.01, 20.0, 0.0, -math.pi + .01), "pending")
        self.assertEqual(ack.observe(1.01, 20.0, 0.0, -math.pi + .01), "pending")
        self.assertEqual(ack.observe(1.02, 20.0, 0.0, -math.pi + .01), "acknowledged")
        self.assertIsNone(ack.pending)

    def test_bad_pose_breaks_ack_count_and_timeout_blocks_retry(self):
        ack = MODULE.ResetAcknowledgement(timeout_sec=.2)
        ack.begin(1.0, 20.0, 0.0, 0.0)
        self.assertEqual(ack.observe(1.01, 20.0, 0.0, 0.0), "pending")
        self.assertEqual(ack.observe(1.02, 19.0, 0.0, 0.0), "pending")
        self.assertEqual(ack.count, 0)
        self.assertEqual(ack.tick(1.21), "timeout_blocked")
        self.assertTrue(ack.blocked)
        with self.assertRaises(ValueError):
            ack.begin(2.0, 20.0, 0.0, 0.0)

    def test_pending_cannot_be_replaced(self):
        ack = MODULE.ResetAcknowledgement()
        ack.begin(1.0, 20.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            ack.begin(1.1, 30.0, 0.0, 0.0)


class FakePublisher:
    def __init__(self):
        self.messages = []
        self.connections = 1
        self.fail = False

    def publish(self, message):
        if self.fail:
            raise rospy.ROSException("test publication failure")
        self.messages.append(message)

    def get_num_connections(self):
        return self.connections


class GuardNodeContractTest(unittest.TestCase):
    def setUp(self):
        self.params = {"~monitor_only": False, "~stationary_hold_sec": 0.0,
                       "~gnss_stationary_hold_sec": 0.0}
        self.publishers = {}
        self.clock_sec = 1.0
        def make_publisher(topic, *_args, **_kwargs):
            self.publishers[topic] = FakePublisher()
            return self.publishers[topic]
        self.patches = [mock.patch.object(MODULE.rospy, "get_param", side_effect=lambda key, default=None: self.params.get(key, default)),
                        mock.patch.object(MODULE.rospy, "Publisher", side_effect=make_publisher),
                        mock.patch.object(MODULE.rospy, "Subscriber"),
                        mock.patch.object(MODULE.rospy, "Timer"),
                        mock.patch.object(MODULE.rospy.Time, "now", side_effect=lambda: rospy.Time.from_sec(self.clock_sec)),
                        mock.patch.object(MODULE.rospy, "loginfo"),
                        mock.patch.object(MODULE.rospy, "logwarn"),
                        mock.patch.object(MODULE.rospy, "logerr")]
        for patch in self.patches:
            patch.start()
            self.addCleanup(patch.stop)
        self.node = MODULE.LocalizationRelocationGuard()
        self.output = self.publishers["/localization/initialpose"]

    @staticmethod
    def odometry(stamp, x=0.0, frame="map"):
        result = Odometry()
        result.header.stamp = rospy.Time.from_sec(stamp)
        result.header.frame_id = frame
        result.child_frame_id = "base_link"
        result.pose.pose.position.x = x
        result.pose.pose.orientation.w = 1.0
        return result

    def sample(self, stamp, x=20.0, vx=0.0, gnss=None):
        self.clock_sec = stamp
        state = self.odometry(stamp)
        self.node.state_callback(state)
        orientation = Imu()
        orientation.header.stamp = state.header.stamp
        orientation.header.frame_id = "base_link"
        orientation.orientation.w = 1.0
        self.node.orientation_callback(orientation)
        motion = TwistWithCovarianceStamped()
        motion.header.stamp = state.header.stamp
        motion.header.frame_id = "base_link"
        motion.twist.twist.linear.x = vx
        self.node.motion_callback(motion)
        self.node.gnss_callback(gnss or self.odometry(stamp, x))

    def test_active_reset_and_global_acknowledgement(self):
        for stamp in (1.0, 1.1, 1.2):
            self.sample(stamp)
        self.assertEqual(len(self.output.messages), 1)
        self.assertIsNotNone(self.node.acknowledgement.pending)
        self.sample(1.3)
        self.assertEqual(len(self.output.messages), 1)
        self.node.state_callback(self.odometry(1.31, 20.0))
        self.node.state_callback(self.odometry(1.32, 20.0))
        self.assertIsNone(self.node.acknowledgement.pending)

    def test_moving_vehicle_can_never_reset(self):
        for stamp in (1.0, 1.1, 1.2, 1.3):
            self.sample(stamp, vx=2.0)
        self.assertEqual(len(self.output.messages), 0)
        self.assertIsNone(self.node.detector.last_trigger_sec)

    def test_wrong_frames_invalid_quaternion_and_covariance_rejected(self):
        bad = self.odometry(1.0, 20.0, "odom")
        self.sample(1.0, gnss=bad)
        bad = self.odometry(1.1, 20.0)
        bad.pose.pose.orientation.w = 0.0
        self.sample(1.1, gnss=bad)
        bad = self.odometry(1.2, 20.0)
        bad.pose.covariance[0] = math.nan
        self.sample(1.2, gnss=bad)
        self.assertEqual(len(self.output.messages), 0)
        self.assertIsNone(self.node.detector.last_trigger_sec)

    def test_duplicate_gnss_never_triggers(self):
        for _ in range(5):
            self.sample(1.0)
        self.assertEqual(len(self.output.messages), 0)

    def test_no_subscriber_or_publication_failure_does_not_commit(self):
        self.output.connections = 0
        for stamp in (1.0, 1.1, 1.2):
            self.sample(stamp)
        self.assertIsNone(self.node.detector.last_trigger_sec)
        self.output.connections = 1
        self.output.fail = True
        for stamp in (1.3, 1.4, 1.5):
            self.sample(stamp)
        self.assertIsNone(self.node.detector.last_trigger_sec)
        self.assertIsNone(self.node.acknowledgement.pending)
        self.assertTrue(self.node.acknowledgement.blocked)

    def test_monitor_candidate_does_not_require_motion(self):
        self.node.monitor_only = True
        for stamp in (1.0, 1.1, 1.2):
            self.clock_sec = stamp
            self.node.state_callback(self.odometry(stamp))
            self.node.gnss_callback(self.odometry(stamp, 20.0))
        self.assertEqual(len(self.output.messages), 0)
        self.assertAlmostEqual(self.node.detector.last_trigger_sec, 1.2, places=8)

    def test_parameters_are_strict_finite_and_boolean(self):
        for key, value in [("~monitor_only", "false"), ("~max_state_age_sec", math.nan),
                           ("~reset_position_stddev_m", math.inf),
                           ("~reset_position_stddev_m", 1.0e308),
                           ("~reset_yaw_stddev_deg", 1.0e-250),
                           ("~consecutive_samples", 2.5)]:
            with self.subTest(key=key, value=value):
                self.params[key] = value
                with self.assertRaises(rospy.ROSInitException):
                    MODULE.LocalizationRelocationGuard()
                del self.params[key]

    def test_normalizes_near_unit_reset_orientation(self):
        q = Quaternion(w=.999)
        normalized = MODULE.normalized_quaternion(q)
        self.assertEqual(normalized.w, 1.0)
        self.assertEqual(q.w, .999)

    def test_stale_heading_or_motion_clears_candidate_without_cooldown(self):
        for missing in ("heading", "motion"):
            with self.subTest(missing=missing):
                self.node = MODULE.LocalizationRelocationGuard()
                self.output = self.publishers["/localization/initialpose"]
                self.sample(1.0)
                self.sample(1.05)
                self.clock_sec = 1.3
                self.node.state_callback(self.odometry(1.3))
                if missing == "heading":
                    motion = TwistWithCovarianceStamped()
                    motion.header.stamp = rospy.Time.from_sec(1.3)
                    motion.header.frame_id = "base_link"
                    self.node.motion_callback(motion)
                else:
                    heading = Imu()
                    heading.header.stamp = rospy.Time.from_sec(1.3)
                    heading.header.frame_id = "base_link"
                    heading.orientation.w = 1.0
                    self.node.orientation_callback(heading)
                self.node.gnss_callback(self.odometry(1.3, 20.0))
                self.assertEqual(len(self.output.messages), 0)
                self.assertEqual(self.node.detector.count, 0)
                self.assertIsNone(self.node.detector.last_trigger_sec)

    def test_watchdog_breaks_count_and_applies_reacquisition_grace(self):
        self.node.monitor_only = True
        self.sample(1.0)
        self.sample(1.1)
        self.assertEqual(self.node.detector.count, 2)
        with mock.patch.object(MODULE.rospy.Time, "now", return_value=rospy.Time.from_sec(2.0)):
            self.node.watchdog_callback(None)
        self.assertEqual(self.node.detector.count, 0)
        for stamp in (8.0, 8.2, 8.4, 8.6, 8.8):
            self.sample(stamp)
            self.assertEqual(self.node.detector.count, 0)
        for stamp in (9.01, 9.1, 9.2):
            self.sample(stamp)
        self.assertAlmostEqual(self.node.detector.last_trigger_sec, 9.2, places=8)

    def test_monitor_innovation_compares_cotime_vehicle_position(self):
        self.node.monitor_only = True
        self.node.detector.threshold_m = .1
        for start in (1.0, 1.2, 1.4):
            self.clock_sec = start + .05
            state = self.odometry(start)
            state.twist.twist.linear.x = 10.0
            self.node.state_callback(state)
            self.node.gnss_callback(self.odometry(start + .05, .5))
        self.assertIsNone(self.node.detector.last_trigger_sec)
        self.assertEqual(self.node.detector.count, 0)

    def test_bad_global_state_clears_ack_evidence(self):
        for stamp in (1.0, 1.05, 1.1):
            self.sample(stamp)
        self.assertEqual(len(self.output.messages), 1)
        self.clock_sec = 1.2
        self.node.state_callback(self.odometry(1.2, 20.0))
        bad = self.odometry(1.21, 20.0)
        bad.pose.pose.orientation.w = 0.0
        self.node.state_callback(bad)
        self.assertEqual(self.node.acknowledgement.count, 0)
        self.assertFalse(self.node.latest_state_is_valid)
        self.clock_sec = 1.23
        self.node.state_callback(self.odometry(1.22, 20.0))
        self.assertIsNotNone(self.node.acknowledgement.pending)
        self.node.state_callback(self.odometry(1.23, 20.0))
        self.assertIsNone(self.node.acknowledgement.pending)

    def test_default_full_second_stationary_hold_precedes_candidate_count(self):
        del self.params["~stationary_hold_sec"]
        self.node = MODULE.LocalizationRelocationGuard()
        self.output = self.publishers["/localization/initialpose"]
        for i in range(50):
            self.sample(1.0 + i * .02)
        self.assertEqual(len(self.output.messages), 0)
        for stamp in (2.01, 2.03, 2.05):
            self.sample(stamp)
        self.assertEqual(len(self.output.messages), 1)

    def test_valid_gnss_position_hold_is_required_even_with_zero_vx(self):
        del self.params["~gnss_stationary_hold_sec"]
        self.node = MODULE.LocalizationRelocationGuard()
        self.output = self.publishers["/localization/initialpose"]
        for i in range(50):
            self.sample(1.0 + i * .02, x=20.0)
        self.assertEqual(len(self.output.messages), 0)
        for stamp in (2.01, 2.03, 2.05):
            self.sample(stamp, x=20.0)
        self.assertEqual(len(self.output.messages), 1)

    def test_lateral_gnss_motion_blocks_reset_even_when_vx_is_zero(self):
        self.params["~gnss_stationary_hold_sec"] = .2
        self.node = MODULE.LocalizationRelocationGuard()
        self.output = self.publishers["/localization/initialpose"]
        for i in range(20):
            gnss = self.odometry(1.0 + i * .05, 20.0)
            gnss.pose.pose.position.y = i * .1
            self.sample(1.0 + i * .05, gnss=gnss)
        self.assertEqual(len(self.output.messages), 0)


    def test_one_future_state_cannot_pin_monotonic_state_cache(self):
        self.clock_sec = 1000.0
        self.node.state_callback(self.odometry(4600.0))
        self.assertIsNone(self.node.latest_state)
        self.node.state_callback(self.odometry(1000.0))
        self.assertAlmostEqual(self.node.stamp_sec(self.node.latest_state), 1000.0)

    def test_cotimed_future_or_stale_input_cannot_reset(self):
        for difference in (3600.0, -2.0):
            with self.subTest(difference=difference):
                self.node = MODULE.LocalizationRelocationGuard()
                self.output = self.publishers["/localization/initialpose"]
                self.clock_sec = 1000.0
                with mock.patch.object(MODULE.rospy.Time, "now", return_value=rospy.Time.from_sec(1000.0)):
                    for i in range(60):
                        self.sample(1000.0 + difference + i * .02)
                self.assertEqual(len(self.output.messages), 0)
                self.assertIsNone(self.node.last_gnss_stamp)
                self.assertIsNone(self.node.latest_motion)
                self.assertIsNone(self.node.latest_orientation)

    def test_old_backlog_eligibility_does_not_modify_input_stamp(self):
        message = self.odometry(1.0, 20.0)
        original_stamp = message.header.stamp.to_nsec()
        self.clock_sec = 3.0
        self.node.gnss_callback(message)
        self.assertEqual(message.header.stamp.to_nsec(), original_stamp)
        self.assertIsNone(self.node.last_gnss_stamp)

    def test_guard_absolute_time_parameter_validation(self):
        for key, value in [("~max_input_age_sec", 0.0),
                           ("~max_input_age_sec", math.nan),
                           ("~future_tolerance_sec", -0.1),
                           ("~future_tolerance_sec", math.inf)]:
            with self.subTest(key=key, value=value):
                self.params[key] = value
                with self.assertRaises(rospy.ROSInitException):
                    MODULE.LocalizationRelocationGuard()
                del self.params[key]


class GnssStationaryGateTest(unittest.TestCase):
    def test_position_jump_or_gap_restarts_full_hold(self):
        gate = MODULE.GnssStationaryGate(.2, .3, .1)
        self.assertFalse(gate.observe(0.0, 0.0, 1.0))
        self.assertFalse(gate.observe(.1, 0.0, 1.05))
        self.assertFalse(gate.observe(.4, 0.0, 1.1))
        for stamp in (1.15, 1.2, 1.25):
            self.assertFalse(gate.observe(.4, 0.0, stamp))
        self.assertTrue(gate.observe(.4, 0.0, 1.31))
        self.assertFalse(gate.observe(.4, 0.0, 1.5))

    def test_quantization_diagonal_is_tolerated_but_bad_data_clears(self):
        gate = MODULE.GnssStationaryGate(.2, .3, .1)
        for i in range(7):
            gate.observe((i % 2)*.148, (i % 3 == 0)*.185, 1.0+i*.05)
        self.assertTrue(gate.ready(1.3))
        self.assertFalse(gate.observe(math.nan, 0.0, 1.35))
        self.assertFalse(gate.ready(1.35))


if __name__ == "__main__":
    unittest.main()
