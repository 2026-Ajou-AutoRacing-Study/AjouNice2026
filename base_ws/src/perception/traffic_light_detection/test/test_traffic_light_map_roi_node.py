#!/usr/bin/env python3
"""Fake callback tests: all TF queries stay in memory, no ROS master needed."""
import copy
import importlib.util
import json
import sys
import threading
import math
import types
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import Mock, patch

from test_traffic_light_candidate_node import Stamp, Duration, Message
from traffic_light_map_selector import load_camera, DEFAULT_SELECTOR_CONFIG

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]


class TransformError(Exception):
    pass


def xyz(values):
    return types.SimpleNamespace(**dict(zip("xyz", values)))


def transform(position, quaternion):
    return types.SimpleNamespace(transform=types.SimpleNamespace(translation=xyz(position),
        rotation=types.SimpleNamespace(**dict(zip("xyzw", quaternion)))))


def load_helper():
    modules = {}
    for name, attributes in {
        "rospy": {"Time": Stamp, "Duration": Duration, "logwarn_throttle": Mock()},
        "tf2_ros": {"TransformException": TransformError},
        "nav_msgs.msg": {"Odometry": Message},
        "sensor_msgs.msg": {"CompressedImage": Message},
        "std_msgs.msg": {"String": Message, "UInt64": Message},
    }.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    spec = importlib.util.spec_from_file_location("traffic_light_map_roi_fake", PACKAGE / "scripts/traffic_light_map_roi_node.py")
    result = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(result)
    return result


class MapRoiNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.module = load_helper()

    def node(self):
        node = object.__new__(self.module.TrafficLightMapRoiNode)
        node.geometry = {"geometry_frame": "morai_map", "heads": [{"id": "C1256W000016", "type": "car",
            "group_id": 201290, "point": [20., 0., 2.], "heading": 180., "width": 1.42, "height": .355}]}
        node.camera = load_camera(ROOT / "configs/sensors/Sensor_competition_local.json")
        node.map_frame, node.base_frame = "map", "base_link"
        node.route_stale, node.pose_delta, node.pose_age = 2., .15, .5
        node.max_position_sigma = DEFAULT_SELECTOR_CONFIG.max_position_sigma_m
        node.max_yaw_sigma = DEFAULT_SELECTOR_CONFIG.max_yaw_sigma_degrees
        node.max_vertical_sigma = 1.5
        node.selector_config = DEFAULT_SELECTOR_CONFIG
        node.projection_options = {}
        node._lock = threading.Lock()
        node._group_id, node._group_receipt = 201290, Stamp(9.9)
        pose = types.SimpleNamespace(header=types.SimpleNamespace(stamp=Stamp(9.99), frame_id="map"),
            child_frame_id="base_link", pose=types.SimpleNamespace(covariance=[0.]*36,
                pose=types.SimpleNamespace(position=xyz((0, 0, 0)),
                    orientation=types.SimpleNamespace(x=0, y=0, z=0, w=1))),
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=xyz((0, 0, 0)), angular=xyz((0, 0, 0)))))
        node._poses = deque([(pose, Stamp(9.99))])
        def lookup(target, source, stamp, timeout):
            if (target, source) == ("map", "base_link"):
                return transform((0, 0, 0), (0, 0, 0, 1))
            if (target, source) == ("base_link", "camera_front"):
                return transform(node.camera.position, node.camera.quaternion)
            if (target, source) == ("camera_front", "morai_map"):
                return transform((0, 0, 0), (0, 0, 0, 1))
            raise AssertionError("unexpected TF query")
        node.buffer = types.SimpleNamespace(lookup_transform=Mock(side_effect=lookup))
        node.pub = Mock()
        return node

    def image(self):
        return types.SimpleNamespace(header=types.SimpleNamespace(stamp=Stamp(9.99), frame_id="camera_front"))

    def call(self, node, image=None):
        node.image_callback(image or self.image())
        return json.loads(node.pub.publish.call_args[0][0].data)

    def test_ready_packet_uses_image_time_for_every_dynamic_TF(self):
        node = self.node()
        packet = self.call(node)
        self.assertEqual(packet["status"], "ready")
        self.assertEqual(packet["source_stamp_ns"], 9990000000)
        self.assertEqual(packet["additional_heading_correction_degrees"], 0.)
        self.assertFalse(packet["visibility_ground_truth"])
        for call in node.buffer.lookup_transform.call_args_list:
            target, source, stamp, _ = call[0]
            self.assertEqual(stamp.to_nsec(), 0 if (target, source) == ("base_link", "camera_front") else 9990000000)

    def test_missing_and_old_D1_routes_do_not_query_TF_or_abort(self):
        for group in (0, 201397):
            node = self.node()
            node._group_id = group
            packet = self.call(node)
            self.assertEqual(packet["status"], "missing")
            self.assertFalse(packet["rois"])
            node.buffer.lookup_transform.assert_not_called()

    def test_stale_route_is_missing(self):
        node = self.node()
        node._group_receipt = Stamp(7.)
        self.assertEqual(self.call(node)["reason"], "route_missing_or_stale")

    def test_missing_or_wrong_camera_stamp_frame_is_missing(self):
        for stamp, frame, reason in ((Stamp(0), "camera_front", "image_stamp_missing"),
                                     (Stamp(9.99), "camera_right", "image_camera_frame_mismatch")):
            node = self.node()
            message = types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp, frame_id=frame))
            self.assertEqual(self.call(node, message)["reason"], reason)

    def test_missing_stale_and_wrong_frame_localization_are_missing(self):
        node = self.node()
        node._poses.clear()
        self.assertEqual(self.call(node)["reason"], "localization_missing")
        node = self.node()
        node._poses[0][0].header.stamp = Stamp(8.)
        self.assertEqual(self.call(node)["reason"], "localization_stale_or_not_image_aligned")
        node = self.node()
        node._poses[0][0].header.frame_id = "odom"
        self.assertEqual(self.call(node)["reason"], "localization_frame_mismatch")

    def test_missing_TF_does_not_try_latest_or_stop_process(self):
        node = self.node()
        node.buffer.lookup_transform.side_effect = TransformError("image-time TF unavailable")
        packet = self.call(node)
        self.assertEqual(packet["status"], "missing")
        self.assertIn("unavailable", packet["reason"])
        self.assertEqual(node.buffer.lookup_transform.call_count, 1)

    def test_TF_and_localization_different_pose_is_missing(self):
        node = self.node()
        node._poses[0][0].pose.pose.position.x = 5.
        self.assertEqual(self.call(node)["reason"], "localization_tf_pose_inconsistent")

    def test_invalid_quaternion_and_covariance_are_missing(self):
        node = self.node()
        node._poses[0][0].pose.pose.orientation.w = 0.
        self.assertEqual(self.call(node)["reason"], "invalid_localization_quaternion")
        node = self.node()
        node._poses[0][0].pose.covariance[0] = 9.
        self.assertEqual(self.call(node)["reason"], "localization_uncertainty_exceeds_bound")

    def test_actual_fixed_camera_TF_mismatch_is_not_silently_recalibrated(self):
        node = self.node()
        original = node.buffer.lookup_transform.side_effect
        def mismatch(target, source, stamp, timeout):
            return transform((1.9, 0, 2.2), node.camera.quaternion) if source == "camera_front" else original(target, source, stamp, timeout)
        node.buffer.lookup_transform.side_effect = mismatch
        self.assertEqual(self.call(node)["reason"], "camera_tf_calibration_mismatch")

    def test_nonfinite_camera_extrinsic_is_missing(self):
        node = self.node()
        original = node.buffer.lookup_transform.side_effect
        def invalid(target, source, stamp, timeout):
            return transform((float("nan"), 0, 1.2), node.camera.quaternion) if source == "camera_front" else original(target, source, stamp, timeout)
        node.buffer.lookup_transform.side_effect = invalid
        self.assertEqual(self.call(node)["reason"], "nonfinite_camera_tf")

    def test_packet_reports_independent_XY_and_GNSS_z_uncertainty(self):
        node = self.node()
        covariance = node._poses[0][0].pose.covariance
        covariance[0], covariance[7], covariance[14], covariance[35] = .0007363848, .0004497517, 1., .0000522796
        packet = self.call(node)
        self.assertEqual(packet["status"], "ready")
        self.assertLess(packet["position_sigma_xy_m"], .03)
        self.assertEqual(packet["position_sigma_z_m"], 1.)
        self.assertEqual(packet["pose_covariance_diagonal_xyz_yaw"][2], 1.)

    def test_motion_evidence_includes_aligned_and_latest_pose_not_ego_input(self):
        node=self.node()
        aligned=node._poses[0][0];aligned.twist.twist.linear.x=3.
        latest=copy.deepcopy(aligned);latest.header.stamp=Stamp(10.)
        latest.twist.twist.linear.x=0.
        node._poses.append((latest,Stamp(10.)))
        packet=self.call(node)
        self.assertEqual(packet['status'],'ready')
        evidence=packet['motion_evidence']
        self.assertTrue(evidence['valid'])
        self.assertEqual(evidence['aligned_speed_mps'],3.)
        self.assertEqual(evidence['latest_speed_mps'],0.)
        self.assertEqual(evidence['aligned_pose_stamp_ns'],9_990_000_000)
        self.assertEqual(evidence['latest_pose_stamp_ns'],10_000_000_000)

    def test_invalid_latest_motion_disables_evidence_without_losing_map_projection(self):
        node=self.node()
        latest=copy.deepcopy(node._poses[0][0]);latest.header.stamp=Stamp(10.)
        latest.twist.twist.linear.x=float('nan')
        node._poses.append((latest,Stamp(10.)))
        packet=self.call(node)
        self.assertEqual(packet['status'],'ready')
        self.assertFalse(packet['motion_evidence']['valid'])

    def test_uncertainty_candidate_preserves_real_sigma_and_normal_missing(self):
        node=self.node();node.visual_continuation_sec=.3
        node._poses[0][0].pose.covariance[14]=1.
        node._poses[0][0].pose.covariance[35]=math.radians(4.5)**2
        packet=self.call(node)
        self.assertEqual(packet['status'],'missing')
        self.assertEqual(packet['reason'],'localization_uncertainty_exceeds_bound')
        self.assertEqual(packet['rois'],[])
        evidence=packet['nominal_continuation']
        self.assertAlmostEqual(evidence['actual_yaw_sigma_degrees'],4.5)
        self.assertEqual(evidence['actual_z_sigma_m'],1.)
        self.assertFalse(evidence['normal_eligibility'])
        self.assertTrue(evidence['rois'])
        self.assertTrue(all(row['eligible'] is False for row in evidence['rois']))

    def test_uncertainty_with_tf_failure_has_no_nominal_continuation_evidence(self):
        node=self.node();node.visual_continuation_sec=.3
        node._poses[0][0].pose.covariance[35]=math.radians(4.5)**2
        node.buffer.lookup_transform.side_effect=TransformError('image-time TF unavailable')
        packet=self.call(node)
        self.assertEqual(packet['status'],'missing')
        self.assertNotIn('nominal_continuation',packet)

    def test_per_head_padding_rejection_gets_nominal_evidence_but_normal_caps_unchanged(self):
        node=self.node();node.visual_continuation_sec=.3
        node.geometry['heads'][0]['point'][0]=10.
        node._poses[0][0].pose.covariance[35]=math.radians(3.9)**2
        packet=self.call(node)
        self.assertEqual(packet['status'],'ready')
        self.assertEqual(packet['reason'],'no_visible_route_head')
        self.assertTrue(any(r['reason']=='projection_uncertainty_exceeds_bound' for r in packet['rejected_heads']))
        self.assertEqual(packet['selector_config']['max_padding_px'],120.)
        self.assertFalse(packet['nominal_continuation']['rois'][0]['eligible'])


if __name__ == "__main__":
    unittest.main()
