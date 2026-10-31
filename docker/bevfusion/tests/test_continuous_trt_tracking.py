"""CPU checks for coordinate/identity diagnostics, not a tracker accuracy claim."""
from copy import deepcopy
import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
TOOLS = ROOT / 'perception/detection3d/bevfusion/tools'
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location('_continuous_tracker_tests',
    TOOLS / 'analyze_base5_continuous_tensorrt_tracking.py')
T = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(T)


def track(ident=1, label=0, x=0., associated=True):
    return dict(id=ident, label=label, class_name=T.CLASSES[label],
        confirmed=True, associated=associated, center=[x, 0., 1.],
        yaw=0., velocity=[1., 0.], yaw_rate=0., size=[2., 1., 2.],
        score=.9, age_frames=3, last_measurement_stamp=1., birth_stamp=.5)


class ContinuousTrackerTests(unittest.TestCase):
    def test_bottom_to_center_then_sensor_and_prepared_ego(self):
        detection = dict(center=[1., 2., 3.], size=[2., 1., 4.], yaw=.25,
            score=.8, label=0)
        original = deepcopy(detection)
        extrinsic = np.eye(4)
        extrinsic[:3, 3] = [10., 20., 30.]
        result = T.world_detection(detection,
            dict(x=100., y=200., z=300., yaw=math.pi/2), extrinsic)
        np.testing.assert_allclose(result['center'], [78., 211., 335.])
        self.assertAlmostEqual(result['yaw'], .25 + math.pi/2)
        self.assertEqual(detection, original)

    def test_core_parser_class_mapping_and_empty_frame(self):
        value = ('FRAME 1 1\n'
            'TRACK 5 3 1 0 10 20 3 .2 1 2 .1 1 .5 2 .9 4 .9 .5\n'
            'FRAME 1.1 0\n')
        rows = T.parse_core(value, [1., 1.1])
        self.assertEqual(rows[0]['tracks'][0]['label'], 1)
        self.assertFalse(rows[0]['tracks'][0]['associated'])
        self.assertEqual(rows[1]['tracks'], [])
        with self.assertRaises(ValueError):
            T.parse_core(value, [1.1, 1.])
        with self.assertRaises(ValueError):
            T.parse_core('FRAME 1 1\n', [1.])

    def test_identity_is_mapping_not_numeric_id_equality(self):
        first = [dict(stamp=1., tracks=[track(1)]),
                 dict(stamp=1.1, tracks=[track(1, x=.1)])]
        same = [dict(stamp=1., tracks=[track(40)]),
                dict(stamp=1.1, tracks=[track(40, x=.1)])]
        result = T.compare(first, same, .5)
        self.assertEqual(result['matched_observations'], 2)
        self.assertEqual(result['id_mapping_changes'], [])
        same[1]['tracks'][0]['id'] = 41
        changed = T.compare(first, same, .5)
        self.assertEqual(len(changed['id_mapping_changes']), 1)
        self.assertEqual(changed['distributions']['position_m']['maximum'], 0.)

    def test_class_and_gate_do_not_hide_missing_or_extra(self):
        old = [dict(stamp=1., tracks=[track(1), track(2, label=1, x=3.)])]
        new = [dict(stamp=1., tracks=[track(5, x=.6), track(6, label=2, x=3.)])]
        strict = T.compare(old, new, .5)
        self.assertEqual(strict['matched_observations'], 0)
        relaxed = T.compare(old, new, 2.)
        self.assertEqual(relaxed['matched_observations'], 1)
        self.assertEqual(sum(r['missing'] for r in relaxed['confirmed_set_changes']), 1)
        self.assertEqual(sum(r['extra'] for r in relaxed['confirmed_set_changes']), 1)

    def test_coasting_and_overlap_are_explicit_not_oracle_duplicates(self):
        rows = [dict(stamp=1.2, tracks=[track(1, associated=False), track(2, x=.1)])]
        result = T.within_run(rows)
        self.assertEqual(result['coasting_observations'], 1)
        self.assertAlmostEqual(result['coasting_age_sec']['mean'], .2)
        self.assertEqual(len(result['near_overlapping_confirmed_pairs']), 1)
        self.assertIn('not oracle', result['duplicate_note'])

    def test_yaw_and_footprint_changes_survive_zero_center_motion(self):
        first = track()
        rotated = deepcopy(first)
        rotated['yaw'] = math.pi/2
        result = T.compare([dict(stamp=1., tracks=[first])],
                           [dict(stamp=1., tracks=[rotated])], .5)
        self.assertEqual(result['distributions']['position_m']['maximum'], 0.)
        self.assertAlmostEqual(result['distributions']['yaw_deg']['maximum'], 90.)
        self.assertGreater(result['distributions']['footprint_corner_m']['maximum'], .5)
        self.assertLess(result['distributions']['rotated_bev_iou']['minimum'], .5)


if __name__ == '__main__':
    unittest.main()
