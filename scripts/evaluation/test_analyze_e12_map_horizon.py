"""Synthetic metric/coordinate tests, no ROS master, simulator or GPU."""
from copy import deepcopy
import math
import unittest
import numpy as np

from analyze_e12_map_horizon import (current_assignment, evaluate_case, future_reference,
    map_point, paired_horizon_effects, path_errors, prepare_case, prefix_extension_index)


def track(identity=1, velocity=(1., 0.), yaw=0.):
    return dict(id=identity, tracker_label=1, label=0, confirmed=True, associated=True,
        center=[0., 0., 0.], velocity=list(velocity), yaw=yaw, yaw_rate=0.)


def gt_series(end=7.):
    times = [i/10. for i in range(int(end*10)+1)]
    return times, [[t, 0., 0.] for t in times]


class MapHorizonTest(unittest.TestCase):
    def test_rigid_map_tf_does_not_repeat_heading_bias(self):
        tf = dict(parent='map', child='morai_map', rpy=[0., 0., math.pi/2], xyz=[10., 20., 1.])
        case = dict(frames=[dict(stamp=0., token='x', scoring_eligible=True, tracks=[track()],
                    gt=[dict(key='actor', label=0, center=[0., 0., 0.])])],
                    gt_trajectories=[dict(key='actor', label=0, series=[[0., [0., 0., 0.]], [1., [1., 0., 0.]]])])
        frames, trajectories = prepare_case(case, tf)
        obj = frames[0]['objects'][0]
        self.assertEqual(obj['center'], [10., 20., 1.])
        self.assertAlmostEqual(obj['yaw'], math.pi/2)
        self.assertEqual(obj['body_velocity'], [1., 0.])
        self.assertAlmostEqual(obj['world_velocity'][1], 1.)
        self.assertEqual(trajectories['actor'][1][0], [10., 20., 1.])

    def test_core_yaw_hysteresis_preserves_world_velocity(self):
        tf = dict(parent='map', child='morai_map', rpy=[0., 0., 0.], xyz=[0., 0., 0.])
        case = dict(frames=[dict(stamp=0., token='x', scoring_eligible=True,
            tracks=[track(velocity=(-2., 0.))], gt=[])], gt_trajectories=[])
        obj = prepare_case(case, tf)[0][0]['objects'][0]
        self.assertAlmostEqual(abs(obj['yaw']), math.pi)
        self.assertAlmostEqual(obj['body_velocity'][0], 2.)
        self.assertEqual(obj['world_velocity'], [-2., 0.])

    def test_current_assignment_never_uses_future_or_duplicates_gt(self):
        objects = [dict(center=[0., 0., 0.]), dict(center=[.1, 0., 0.]), dict(center=[10., 0., 0.])]
        truth = [dict(center=[0., 0., 0.])]
        self.assertEqual(current_assignment(objects, truth), [(0, 0, 0.)])

    def test_no_future_extrapolation_or_large_gap_fill(self):
        self.assertIsNone(future_reference(gt_series(2.), 0., 3)[0])
        times, points = gt_series()
        keep = [i for i, t in enumerate(times) if not .5 < t < 1.]
        reference, reason = future_reference(([times[i] for i in keep], [points[i] for i in keep]), 0., 3)
        self.assertIsNone(reference)
        self.assertEqual(reason, 'interpolation_gap_or_missing_samples')

    def test_future_gt_does_not_bridge_a_15hz_excluded_frame_with_small_time_gap(self):
        times = [i/15. for i in range(17) if i != 5]
        points = [[t, 0., 0.] for t in times]
        indices = [i for i in range(17) if i != 5]
        reference, reason = future_reference((times, points, indices), 0., 1)
        self.assertIsNone(reference)
        self.assertEqual(reason, 'nonadjacent_or_ineligible_original_gt_indices')

    def test_ade_excludes_current_and_minK_does_not_change_top1(self):
        reference = np.asarray(gt_series()[1][1:11])[:, :2]
        paths = [dict(xy=(reference+[3., 0.]).tolist()), dict(xy=reference.tolist())]
        result = path_errors(paths, reference, 1)
        self.assertEqual(result['top1_ADE_m'], 3.)
        self.assertEqual(result['top1_FDE_m'], 3.)
        self.assertEqual(result['minK_ADE_m'], 0.)
        self.assertEqual(result['oracle_ADE_path_index'], 1)

    def test_short_prediction_is_rejected_not_silently_scored(self):
        with self.assertRaisesRegex(ValueError, 'terminated'):
            path_errors([dict(xy=[[0., 0.]])], np.zeros((10, 2)), 1)

    def test_long_horizon_is_recomputed_and_early_route_changes_are_reported(self):
        called = []
        frame = dict(stamp=0., token='x', scoring_eligible=True,
            objects=[dict(id='t', center=[0., 0., 0.])], gt=[dict(key='a', center=[0., 0., 0.])])

        def predictor(frame, output_horizon):
            self.assertNotIn('future', frame)
            called.append(output_horizon)
            xy = [[i/10.+(0. if output_horizon == 3 else 1.), 0.] for i in range(1, output_horizon*10+1)]
            return [dict(mapped=output_horizon == 3, reason='mapped' if output_horizon == 3 else 'route_ends',
                paths=[dict(xy=xy, confidence=1., lanelet_ids=[output_horizon])])], dict(ms=1., mapped=1, fallback=0, paths=1)

        report, rows, _, prefixes = evaluate_case([frame], {'a': gt_series()}, predictor, 'independent')
        self.assertEqual(called, [3, 5, 6])
        self.assertEqual(report['counts']['current_2m_matched_observations'], 1)
        self.assertEqual(report['metrics']['3']['5']['top1_ADE_m']['n'], 0)
        paired = paired_horizon_effects(rows)['3s_to_6s']['1']
        self.assertAlmostEqual(paired['delta_ADE_m']['mean'], 1.)
        self.assertEqual(paired['mapped_to_fallback'], 1)
        self.assertEqual(paired['top1_lanelet_route_changed'], 1)
        self.assertEqual(report['direct_prefix']['3s_to_6s']['changed_prefix_gt1e9'], 1)
        self.assertAlmostEqual(prefixes[0]['prefix_mean_xy_difference_m'], 1.)
        self.assertAlmostEqual(report['step_groups']['6']['5_6s']['mean'], 1.)

    def test_ineligible_frames_run_prediction_but_never_enter_accuracy(self):
        frame = dict(stamp=0., token='x', scoring_eligible=False, objects=[], gt=[])
        called = []

        def predict(frame, horizon):
            called.append(horizon)
            return [], dict(ms=0., mapped=0, fallback=0, paths=0)

        report, rows, _, _ = evaluate_case([frame], {}, predict, 'c')
        self.assertEqual(len(called), 3)
        self.assertEqual(report['counts']['excluded_scoring_frames'], 1)
        self.assertEqual(rows, [])

    def test_prefix_extension_selects_real_compatible_path_without_gt(self):
        xy = [[i/10., 0.] for i in range(1, 61)]
        short = dict(mapped=True, paths=[dict(xy=xy[:30], lanelet_ids=[1, 2])])
        long = dict(mapped=True, paths=[dict(xy=xy, lanelet_ids=[1, 3, 4]),
            dict(xy=xy, lanelet_ids=[1, 2, 5])])
        self.assertEqual(prefix_extension_index(short, long), 1)
        long['paths'][1]['xy'] = [[x+1., y] for x, y in xy]
        self.assertIsNone(prefix_extension_index(short, long))
        long['mapped'] = False
        self.assertIsNone(prefix_extension_index(short, long))

    def test_future_gt_excludes_ineligible_source_but_keeps_tracker_history(self):
        tf = dict(parent='map', child='morai_map', rpy=[0., 0., 0.], xyz=[0., 0., 0.])
        frames = [dict(stamp=i/10., token=str(i), scoring_eligible=i != 5,
            tracks=[track()], gt=[]) for i in range(11)]
        case = dict(frames=frames, gt_trajectories=[dict(key='a', label=0,
            series=[[i/10., [i/10., 0., 0.]] for i in range(11)])])
        history, trajectories = prepare_case(case, tf)
        self.assertEqual(len(history), 11)
        self.assertNotIn(.5, trajectories['a'][0])
        reference, reason = future_reference(trajectories['a'], 0., 1)
        self.assertIsNone(reference)
        self.assertEqual(reason, 'nonadjacent_or_ineligible_original_gt_indices')


if __name__ == '__main__':
    unittest.main()
