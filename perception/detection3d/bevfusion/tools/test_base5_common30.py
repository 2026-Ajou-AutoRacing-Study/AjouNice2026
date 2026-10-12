import copy
import unittest
from types import SimpleNamespace
import numpy as np
from compare_base5_common30 import (scope, fixed_matches, gt_identity, evaluate,
                                   adapt_intensity, validate_payload, validate_runtime_switches,
                                   steady_median_ms, CLASSES)


def row():
    boxes = [[0,0,0,1,1,1,0], [29,29,0,1,1,1,0], [31,0,0,1,1,1,0]]
    return dict(token='a', gt_boxes=boxes, gt_labels=[2]*3,
                gt_points=[0,6,20], gt_assets=['a','b','c'],
                pred_boxes=boxes, pred_labels=[2]*3, pred_scores=[.8,.7,.6])


class Common30Tests(unittest.TestCase):
    def test_small_pilot_timing_has_no_empty_warmup_nan(self):
        self.assertEqual(steady_median_ms([.01] * 8), 10.)
        self.assertEqual(steady_median_ms([1.] * 10 + [.02] * 2), 20.)
        with self.assertRaises(ValueError):steady_median_ms([])

    def test_runtime_switches_are_explicit_and_mutually_checked(self):
        validate_runtime_switches(SimpleNamespace())
        validate_runtime_switches(SimpleNamespace(compile_parts=['camera','neck'],parallel_camera=True))
        for attrs in (dict(compile_parts=['sparse']),dict(eager_camera_numerics=True),
                dict(compile_reference_layout=True),dict(compile_parts=['camera'],camera_cuda_graph_probe=True)):
            with self.assertRaises(ValueError):validate_runtime_switches(SimpleNamespace(**attrs))
    def test_prediction_export_contract(self):
        payload = dict(rows=[row()], frames=1, classes=CLASSES,
                       gt_identity=gt_identity([row()]))
        validate_payload(payload)
        for key, value in [('classes', ('obstacle', 'vehicle', 'pedestrian')),
                           ('frames', 2), ('gt_identity', 'changed')]:
            bad = dict(payload, **{key: value})
            with self.assertRaises(ValueError):
                validate_payload(bad)
        bad = copy.deepcopy(payload)
        bad['rows'][0]['pred_scores'][0] = np.nan
        with self.assertRaises(ValueError):
            validate_payload(bad)
        duplicate = dict(payload, rows=[row(), row()], frames=2,
                         gt_identity=gt_identity([row(), row()]))
        with self.assertRaises(ValueError):
            validate_payload(duplicate)

    def test_intensity_adapter_preserves_xyz_and_source(self):
        points = np.array([[1., 2., 3., .5, 7.]], dtype=np.float32)
        original = points.copy()
        scaled = adapt_intensity(points, 255.)
        np.testing.assert_array_equal(points, original)
        np.testing.assert_array_equal(scaled[:, :3], original[:, :3])
        np.testing.assert_array_equal(scaled[:, 4:], original[:, 4:])
        self.assertEqual(scaled[0, 3], 127.5)
        self.assertIs(adapt_intensity(points, 1.), points)

    def test_intensity_adapter_rejects_unqualified_inputs(self):
        for value in (2., -1., np.nan, np.inf):
            with np.errstate(invalid='ignore'), self.assertRaises(ValueError):
                adapt_intensity(np.array([[0., 0., 0., value]]), 255.)
        with self.assertRaises(ValueError):
            adapt_intensity(np.zeros((1, 3)), 255.)
        with self.assertRaises(ValueError):
            adapt_intensity(np.zeros((1, 4)), 2.)

    def test_raw_ros_to_base5_roundtrip(self):
        points = np.array([[1., 2., 3., 135.], [4., 5., 6., 255.]], np.float32)
        normalized = adapt_intensity(points, 1./255.)
        np.testing.assert_array_equal(normalized[:, :3], points[:, :3])
        np.testing.assert_allclose(normalized[:, 3], [135./255., 1.])
        np.testing.assert_allclose(adapt_intensity(normalized, 255.), points, atol=2e-5)
        with self.assertRaises(ValueError):
            adapt_intensity(np.array([[0., 0., 0., 256.]]), 1./255.)

    def test_radius_is_not_square(self):
        self.assertEqual(len(scope(row())['gt_boxes']), 1)
        self.assertEqual(len(scope(row(), False)['gt_boxes']), 2)

    def test_sparse_gt_is_not_dropped(self):
        s = scope(row())
        self.assertEqual(s['gt_points'].tolist(), [0])
        self.assertEqual(fixed_matches(s, 2, .1)[0], {0})

    def test_one_to_one(self):
        s = scope(row())
        s['pred_boxes'] = np.repeat(s['pred_boxes'], 2, axis=0)
        s['pred_labels'] = np.array([2,2])
        s['pred_scores'] = np.array([.2,.8])
        taken, count = fixed_matches(s, 2, .1)
        self.assertEqual((taken, count), ({0}, 2))

    def test_score_and_strict_distance(self):
        s = scope(row())
        self.assertEqual(fixed_matches(s, 2, .9), (set(), 0))
        s['pred_boxes'][0,0] = 1.
        self.assertEqual(fixed_matches(s, 2, .1)[0], set())

    def test_identity_includes_gt_not_predictions(self):
        a = [row()]; b = copy.deepcopy(a)
        b[0]['pred_scores'] = [0,0,0]
        self.assertEqual(gt_identity(a), gt_identity(b))
        b[0]['gt_points'][0] = 5
        self.assertNotEqual(gt_identity(a), gt_identity(b))

    def test_empty_scene_preserved(self):
        r = {k:([] if k != 'token' else 'empty') for k in row()}
        self.assertEqual(scope(r)['gt_boxes'].shape, (0,7))
        self.assertEqual(fixed_matches(scope(r), 2, .1), (set(), 0))

    def test_full_metric_and_support_share_gt(self):
        result = evaluate([row()])
        self.assertEqual(result['metrics']['center/obstacle_GT'], 1)
        self.assertAlmostEqual(result['metrics']['center/obstacle_mean_AP'], 1.)
        support = result['support_recall']['obstacle/score0.1_1m']['strata']
        self.assertEqual(support['lt5points']['tp'], 1)


if __name__ == '__main__':
    unittest.main()
