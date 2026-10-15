"""Pure checks for the development harness. No ROS node or vehicle actuation."""
import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'validator_intervention', Path(__file__).with_name('run_validator_intervention.py'))
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


class InterventionTests(unittest.TestCase):
    def config(self):
        return {'localization_safety': {'enabled': True},
                'stack': {'pose_source': 'localization', 'detector': 'od_gt',
                          'traffic_light': False}, 'cases': [NS()]}

    def call(self, cfg, extra=()):
        args = ['trial', 'private.yaml', '--mode', 'checkpoint', *extra]
        with patch.object(sys, 'argv', args), patch.object(M.r, 'load_campaign', return_value=cfg), \
                patch.object(M.r, 'main', return_value=0) as run:
            result = M.main()
            return result, run.call_args

    def test_without_execute_is_only_check(self):
        result, call = self.call(self.config())
        self.assertEqual(result, 0)
        self.assertEqual(call.args, (['private.yaml', '--check'],))

    def test_rejects_wrong_provider_detector_or_unprotected_trial(self):
        for group, key, value in [('stack', 'pose_source', 'maptf_morai'),
                                  ('stack', 'detector', 'bevfusion'),
                                  ('stack', 'traffic_light', True),
                                  ('localization_safety', 'enabled', False)]:
            cfg = self.config()
            cfg[group][key] = value
            with self.subTest(key=key), self.assertRaises(SystemExit):
                self.call(cfg, ['--execute'])

    def test_one_case_only(self):
        cfg = self.config()
        cfg['cases'].append(NS())
        with self.assertRaises(SystemExit):
            self.call(cfg)

    def test_nonfinite_times_and_unknown_checkpoint_rejected(self):
        for extra in [['--hold-sec', 'nan'], ['--recovery-timeout', 'inf'],
                      ['--trigger-sec', '-1'], ['--checkpoint', '0']]:
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                self.call(self.config(), extra)

    def test_mgeo_link_requires_direction_and_distance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / 'kcity_map/mgeo_map/link_set.json'
            target.parent.mkdir(parents=True)
            target.write_text(json.dumps([
                {'idx': 'reverse', 'points': [[10, 0, 0], [0, 0, 0]]},
                {'idx': 'forward', 'points': [[0, 1, 0], [10, 1, 0]]}]))
            with patch.object(M, 'ROOT', root):
                distance, link, ratio = M.near_link((5, 0, 0), 0)
                self.assertEqual(link, 'forward')
                self.assertAlmostEqual(distance, 1)
                self.assertAlmostEqual(ratio, .5)
                with self.assertRaises(ValueError):
                    M.near_link((5, 20, 0), 0)

    def test_independent_heading_offset_applied_exactly_once(self):
        q = NS(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4))
        state = NS(pose=NS(pose=NS(position=NS(x=112., y=213.), orientation=q)))
        transform = NS(transform=NS(rotation=NS(x=0., y=0., z=0., w=1.),
                                   translation=NS(x=100., y=200.)))
        monitor = NS(ego=NS(position=NS(x=12., y=13.), heading=91.347),
                     odom=state, control_odom=state,
                     trial_tf_buffer=NS(lookup_transform=lambda *a: transform))
        with patch.dict(sys.modules, {'rospy': NS(Time=lambda *a: 0, Duration=lambda *a: 0),
                                      'tf2_ros': NS()}):
            result = M.position_errors(monitor)
        self.assertAlmostEqual(result['global_xy_error_m'], 0)
        self.assertAlmostEqual(result['smooth_xy_error_m'], 0)
        self.assertAlmostEqual(result['global_yaw_error_deg'], 0)
        self.assertAlmostEqual(result['smooth_yaw_error_deg'], 0)

    def test_known_prior_requires_fresh_canonical_heading(self):
        message = NS(header=NS(frame_id='base_link', stamp=NS(to_sec=lambda: 10.)),
                     orientation=NS(x=0., y=0., z=0., w=1.), orientation_covariance=[0.] * 9)
        self.assertIs(M.validate_corrected_heading(message, 10.05), message.orientation)
        for now in (9.9, 10.2, float('nan')):
            with self.subTest(now=now), self.assertRaises(M.rt.SafetyFailure):
                M.validate_corrected_heading(message, now)
        message.header.frame_id = 'map'
        with self.assertRaises(M.rt.SafetyFailure):
            M.validate_corrected_heading(message, 10.05)

    def test_known_prior_rejects_invalid_quaternion_and_covariance(self):
        message = NS(header=NS(frame_id='base_link', stamp=NS(to_sec=lambda: 10.)),
                     orientation=NS(x=0., y=0., z=0., w=0.), orientation_covariance=[0.] * 9)
        with self.assertRaises(M.rt.SafetyFailure):
            M.validate_corrected_heading(message, 10.05)
        message.orientation.w = 1.
        message.orientation_covariance[4] = float('nan')
        with self.assertRaises(M.rt.SafetyFailure):
            M.validate_corrected_heading(message, 10.05)


if __name__ == '__main__':
    unittest.main()
