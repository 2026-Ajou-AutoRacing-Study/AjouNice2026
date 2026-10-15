import os
from pathlib import Path
import unittest

import numpy as np

from build_base5_low_score_probe import probe_source, replace_once
from evaluate_base5_low_score_probe import replay, local_box, POLICIES, confidence_diagnostics, score_case
from evaluate_base5_far_retention import full_prepare
from test_base5_far_retention import row


class LowScoreDiagnosticsTests(unittest.TestCase):
    def test_strict_patch_refuses_missing_or_repeated_anchor(self):
        for text in ('none', 'a a'):
            with self.assertRaises(ValueError):
                replace_once(text, 'a', 'b')

    def test_world_to_lidar_preserves_bottom_reference(self):
        t = dict(center=[10., 23., 2.], size=[4., 2., 1.], yaw=np.pi / 2)
        ext = np.eye(4); ext[0, 3] = 1.
        np.testing.assert_allclose(local_box(t, [10., 20., 0., np.pi / 2], ext),
                                   [2., 0., 2., 4., 2., 1., 0.], atol=1e-12)

    def test_confidence_before_and_after_first_high_are_different(self):
        a = row(60., 60.1); a['pred_scores'] = [.2]
        b = row(60., 60.1); b.update(token='b', stamp=1.1, native_index=1, pred_scores=[.6])
        c = row(60., 60.1); c.update(token='c', stamp=1.2, native_index=2, pred_scores=[.2])
        d = confidence_diagnostics(full_prepare([a, b, c]))['by_band']['50_80m']
        self.assertEqual(d['weak_before_first_high'], 1)
        self.assertEqual(d['weak_after_first_high'], 1)
        self.assertEqual(d['vehicle_match_ge040'], 1)

    def test_excluded_frame_breaks_causal_confidence_history(self):
        a = row(60., 60.1); a['pred_scores'] = [.6]
        b = row(60., 60.1); b.update(token='b', stamp=1.2, native_index=2, pred_scores=[.2])
        result = confidence_diagnostics(full_prepare([a, b]))
        self.assertEqual(result['segment_count'], 2)
        self.assertEqual(result['by_band']['50_80m']['weak_before_first_high'], 1)

    def test_roi_filtered_tracks_keep_identity_for_quality(self):
        r = row(60., 60.1); prepared = full_prepare([r])
        def track(i, x):
            return dict(id=i, core_class=1, confirmed=True, associated=True, center=[x, 0., 0.],
                        size=[4., 2., 1.], yaw=0., velocity=[0., 0.], score=.8, last_measurement_time=1.)
        frames = [dict(stamp=1., tracks=[track(1, 120.), track(2, 60.1)])]
        result, observations = score_case(prepared, frames, np.array([[1., 0., 0., 0., 0.]]), np.eye(4), {}, .1)
        self.assertEqual(next(iter(observations.values()))['id'], 2)


@unittest.skipUnless(os.environ.get('BASE5_PROBE_BUILD'), 'Set BASE5_PROBE_BUILD for isolated compiled-core tests')
class ActualCoreTests(unittest.TestCase):
    def run_frames(self, observations, policy='weak020_xy', binary='probe'):
        rows, sources = [], {}
        for i, detections in enumerate(observations):
            token = str(i); t = 1. + i * .1
            rows.append(dict(token=token, pred_boxes=[d[0] for d in detections],
                             pred_labels=[d[2] if len(d) > 2 else 0 for d in detections], pred_scores=[d[1] for d in detections]))
            sources[token] = dict(timestamp=t)
        ego = np.array([[.9, 0., 0., 0., 0.], [10., 0., 0., 0., 0.]])
        # Exact Ego stamps avoid testing interpolation support in this fixture.
        ego = np.array([[s['timestamp'], 0., 0., 0., 0.] for s in sources.values()])
        return replay(Path(os.environ['BASE5_PROBE_BUILD']) / binary, rows, sources, ego, np.eye(4), POLICIES[policy])

    def detections(self, n, low_after=3, wrong_shape=False):
        return [[([50. + i * .4, 0., 0., 40. if wrong_shape and i >= low_after else 4., 2., 1., 0.],
                  .6 if i < low_after else .3)] for i in range(n)]

    def test_low_only_never_births(self):
        r = self.run_frames(self.detections(8, low_after=0))
        self.assertTrue(all(not f['tracks'] for f in r['frames']))
        self.assertEqual(r['weak_updates'], [])

    def test_existing_confirmed_vehicle_gets_weak_update(self):
        r = self.run_frames(self.detections(4))
        self.assertEqual(len(r['weak_updates']), 1)
        self.assertTrue(r['frames'][-1]['tracks'][0]['associated'])

    def test_tentative_track_cannot_be_confirmed_by_weak_measurements(self):
        r = self.run_frames(self.detections(8, low_after=2))
        self.assertFalse(any(t['confirmed'] for f in r['frames'] for t in f['tracks']))
        self.assertEqual(r['weak_updates'], [])

    def test_weak_updates_cannot_extend_track_indefinitely(self):
        r = self.run_frames(self.detections(30))
        self.assertFalse(r['frames'][-1]['tracks'])
        self.assertTrue(all(u['stamp'] <= 1.7 + 1e-9 for u in r['weak_updates']))

    def test_shape_gate_rejects_wrong_object(self):
        r = self.run_frames(self.detections(4, wrong_shape=True))
        self.assertEqual(r['weak_updates'], [])

    def test_high_association_has_priority_and_low_cannot_birth_duplicate(self):
        ds = self.detections(4, low_after=4)
        ds[-1].append(([51.3, 0., 0., 4., 2., 1., 0.], .3))
        result = self.run_frames(ds)
        self.assertEqual(result['weak_updates'], [])
        self.assertEqual(len(result['frames'][-1]['tracks']), 1)

    def test_xy_only_preserves_shape_height_and_z(self):
        ds = self.detections(4)
        ds[-1] = [([51.2, 0., 1., 4.5, 2.5, 2., .3], .3)]
        result = self.run_frames(ds)
        self.assertEqual(len(result['weak_updates']), 1)
        track = result['frames'][-1]['tracks'][0]
        self.assertEqual(track['size'], [4., 2., 1.])
        self.assertEqual(track['center'][2], 0.)

    def test_pi_heading_ambiguity_does_not_reject_same_vehicle(self):
        ds = self.detections(4)
        ds[-1][0][0][-1] = np.pi
        self.assertEqual(len(self.run_frames(ds)['weak_updates']), 1)

    def test_large_innovation_is_not_used_for_recovery(self):
        ds = self.detections(4)
        ds[-1][0][0][0] = 80.
        self.assertEqual(self.run_frames(ds)['weak_updates'], [])

    def test_unstable_vehicle_class_is_not_weakly_reinforced(self):
        ds = self.detections(6)
        ds[3] = [(ds[3][0][0], .6, 2)]
        ds[4] = [(ds[4][0][0], .6, 2)]
        stable = self.run_frames(ds, 'weak010_class080_xy')
        self.assertEqual(stable['weak_updates'], [])
        loose = self.run_frames(ds, 'weak010_xy')
        self.assertTrue(loose['weak_updates'])

    def test_stationary_vehicle_keeps_baseline_without_weak_extension(self):
        ds = self.detections(4)
        for frame in ds:
            frame[0][0][0] = 50.
        result = self.run_frames(ds, 'weak010_moving_class090_xy')
        self.assertEqual(result['weak_updates'], [])
        self.assertTrue(result['frames'][-1]['tracks'])

    def test_disabled_copy_and_repeat_are_exact(self):
        ds = self.detections(6, low_after=6)
        a = self.run_frames(ds, 'baseline', 'original')
        b = self.run_frames(ds, 'baseline', 'probe')
        self.assertEqual(a, b)
        self.assertEqual(a, self.run_frames(ds, 'baseline', 'original'))


if __name__ == '__main__':
    unittest.main()
