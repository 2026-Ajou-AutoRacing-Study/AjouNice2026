"""No master/model: explicit origin math and actual bridge publication."""

import copy
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import rospy

sys.path.insert(0, str(Path(__file__).parent))
from test_frame_timing import bridge, module, Publisher, wire
from bevfusion_ros.box_center_origin import (
    BOTTOM_ORIGIN, GRAVITY_ORIGIN, LEGACY_OUTPUT, response_for_ros,
    validate_output_origin,
)


def sample_response(origin=BOTTOM_ORIGIN):
    return dict(operation='inference', frame_id='lidar1', box_center_origin=origin,
        detections=[dict(center=[8., -2., -1.5], size=[4.6, 1.9, 2.], yaw=.2,
            class_name='vehicle', label=0, score=.9, velocity=[0., 0.])])


class OriginTests(unittest.TestCase):
    def test_default_preserves_same_response_and_untagged_legacy(self):
        for response in (sample_response(), {'detections': []}):
            self.assertIs(response_for_ros(response), response)

    def test_geometric_center_once_without_input_mutation_or_other_fields_change(self):
        response = sample_response()
        before = copy.deepcopy(response)
        converted = response_for_ros(response, GRAVITY_ORIGIN)
        self.assertEqual(response, before)
        self.assertEqual(converted['detections'][0]['center'], [8., -2., -.5])
        self.assertEqual(converted['box_center_origin'], GRAVITY_ORIGIN)
        for key, value in before['detections'][0].items():
            if key != 'center':
                self.assertEqual(converted['detections'][0][key], value)
        self.assertIs(response_for_ros(converted, GRAVITY_ORIGIN), converted)

    def test_missing_unknown_origin_and_invalid_mode_fail_closed(self):
        for origin in (None, '', 'guess_from_size', 'bottom', False):
            response = sample_response(origin)
            with self.assertRaises(ValueError):
                response_for_ros(response, GRAVITY_ORIGIN)
        for value in (None, '', True, 'gravity', BOTTOM_ORIGIN):
            with self.assertRaises(ValueError):
                validate_output_origin(value)

    def test_numeric_center_is_normalized_and_overflow_rejected(self):
        response = sample_response()
        response['detections'][0]['center'] = ['8', '-2', '-1.5']
        converted = response_for_ros(response, GRAVITY_ORIGIN)
        self.assertEqual(converted['detections'][0]['center'], [8., -2., -.5])
        response['detections'][0]['center'] = [8., -2., 1.7e308]
        response['detections'][0]['size'][2] = 1.7e308
        with self.assertRaises(ValueError):
            response_for_ros(response, GRAVITY_ORIGIN)

    def test_bad_center_height_and_shape_rejected_only_in_trial(self):
        for key, value in (('center', [0., 1.]), ('size', [1., 2.]),
                           ('center', [0., 1., math.nan]),
                           ('size', [1., 2., 0.]), ('size', [1., 2., math.inf])):
            response = sample_response()
            response['detections'][0][key] = value
            self.assertIs(response_for_ros(response), response)
            with self.assertRaises(ValueError):
                response_for_ros(response, GRAVITY_ORIGIN)
        empty = response_for_ros(dict(box_center_origin=BOTTOM_ORIGIN, detections=[]), GRAVITY_ORIGIN)
        self.assertEqual(empty['detections'], [])

    def test_actual_bridge_objects_and_cube_share_corrected_z_and_legacy_wire(self):
        outputs = {}
        raw = sample_response()
        for mode in (LEGACY_OUTPUT, GRAVITY_ORIGIN):
            obj = bridge(False)
            obj.box_center_output_origin = mode
            obj.transport.request = lambda metadata, blobs: (copy.deepcopy(raw), {})
            prepared = ({'sequence': 1}, {}, rospy.Time.from_sec(100.), 1.)
            with patch.object(module.rospy.Time, 'now', return_value=rospy.Time.from_sec(100.1)):
                obj._request_and_publish(prepared)
            actual = obj.objects_pub.messages[0].objects[0]
            cube = next(m for m in obj.marker_pub.messages[0].markers if m.type == m.CUBE)
            expected = -1.5 if mode == LEGACY_OUTPUT else -.5
            self.assertEqual(actual.kinematics.pose_with_covariance.pose.position.z, expected)
            self.assertEqual(cube.pose.position.z, expected)
            self.assertEqual(actual.shape.dimensions.z, 2.)
            outputs[mode] = obj
        # Direct pre-origin publication is the default's exact wire oracle.
        old = bridge(False)
        old._publish_detections(raw, rospy.Time.from_sec(100.))
        self.assertEqual(wire(old.objects_pub.messages[0]), wire(outputs[LEGACY_OUTPUT].objects_pub.messages[0]))
        self.assertEqual(wire(old.marker_pub.messages[0]), wire(outputs[LEGACY_OUTPUT].marker_pub.messages[0]))
        self.assertEqual(raw, sample_response())

    def test_actual_bridge_rejects_old_untagged_server_before_box_publication(self):
        obj = bridge(False)
        obj.box_center_output_origin = GRAVITY_ORIGIN
        with self.assertRaises(ValueError):
            obj._request_and_publish(({'sequence': 1}, {}, rospy.Time.from_sec(100.), 1.))
        self.assertEqual(obj.objects_pub.messages, [])
        self.assertEqual(obj.marker_pub.messages, [])

    def test_trial_launch_forwarding_and_defaults_are_explicit(self):
        package = Path(__file__).resolve().parents[1]
        for name in ('base3_online.launch', 'base5_trial.launch',
                     'base5_e24_calibrated_trial.launch', 'base5_e9_calibrated_trial.launch'):
            tree = ET.parse(str(package/'launch'/name))
            argument = tree.find("./arg[@name='box_center_output_origin']")
            self.assertEqual(argument.attrib['default'], LEGACY_OUTPUT)
            if name != 'base3_online.launch':
                forwarded = tree.find(".//include/arg[@name='box_center_output_origin']")
                self.assertEqual(forwarded.attrib['value'], '$(arg box_center_output_origin)')


if __name__ == '__main__':
    unittest.main()
