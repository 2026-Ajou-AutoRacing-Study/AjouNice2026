import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from diagnose_independent_direction_event import (associated_measurement,
    encoded_frames, fit_diagnostic, parse_observers)


class IndependentDirectionDiagnosisTests(unittest.TestCase):
    def test_insufficient_three_observations_are_not_trusted(self):
        rows = [dict(stamp=100.+i*.075, center=[i*.15, 0.]) for i in range(3)]
        fit = fit_diagnostic(rows)
        self.assertFalse(fit['sufficient_count'])
        self.assertFalse(fit['sufficient_span'])
        self.assertAlmostEqual(fit['velocity'][0], 2.)

    def test_mature_epoch_scale_fit_and_window(self):
        rows = [dict(stamp=1791968829.+i*.1, center=[i*.2, -i*.3]) for i in range(10)]
        fit = fit_diagnostic(rows)
        self.assertEqual(fit['count'], 5)
        self.assertTrue(fit['sufficient_count'])
        self.assertTrue(fit['sufficient_span'])
        self.assertAlmostEqual(fit['speed_mps'], math.sqrt(13.), places=5)
        self.assertLess(fit['rms_residual_m'], 1.e-6)

    def test_observer_stdout_preserves_actual_stamp_and_id(self):
        guards, histories = parse_observers('GUARD 100.125 3 0 0 0 -1\nHISTORY 100.125 3 0 100.0 1.25 2.5\n')
        self.assertFalse(guards[(100.125, 3)]['guarded'])
        self.assertEqual(histories[(100.125, 3)][0]['stamp'], 100.)

    def test_only_exact_history_center_identifies_association(self):
        obj = dict(class_name='vehicle', center=[1., 2., 3.], yaw=0., size=[4., 2., 1.5], score=.8)
        frame = dict(stamp=100., world_objects=[obj])
        result = associated_measurement([dict(stamp=100., center=[1., 2.])], {100.:frame})
        self.assertEqual(result['input_index'], 0)
        with self.assertRaises(ValueError):
            associated_measurement([dict(stamp=100., center=[1.00001, 2.])], {100.:frame})

    def test_ambiguous_exact_centers_fail_not_nearest_guess(self):
        obj = dict(class_name='vehicle', center=[1., 2., 3.], yaw=0., size=[4., 2., 1.5], score=.8)
        with self.assertRaises(ValueError):
            associated_measurement([dict(stamp=100., center=[1., 2.])], {100.:dict(world_objects=[obj, obj])})

    def test_encoding_keeps_full_binary_round_trip_precision(self):
        stamp = 1791968829.4064145
        obj = dict(class_name='vehicle', center=[1., 2., 3.], yaw=0., size=[4., 2., 1.5], score=.8)
        text = encoded_frames([dict(stamp=stamp, world_objects=[obj])])
        self.assertEqual(float(text.split()[0]), stamp)
        self.assertEqual(text.splitlines()[0].split()[1], '1')


if __name__ == '__main__':
    unittest.main()
