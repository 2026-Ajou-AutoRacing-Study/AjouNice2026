import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from base5_distance_diagnostics import CLASSES, diagnostic_metrics


def sample(xy, labels=None, prediction_xy=None):
    labels = np.asarray(labels if labels is not None else [0] * len(xy), dtype=np.int64)
    boxes = np.asarray([[x, y, 0, 2, 2, 2, 0] for x, y in xy], dtype=float).reshape(-1, 7)
    predictions = boxes.copy() if prediction_xy is None else np.asarray(
        [[x, y, 0, 2, 2, 2, 0] for x, y in prediction_xy], dtype=float).reshape(-1, 7)
    return dict(gt_boxes=boxes, gt_labels=labels, pred_boxes=predictions,
                pred_labels=labels.copy() if prediction_xy is None else np.zeros(len(predictions), dtype=np.int64),
                pred_scores=np.ones(len(predictions)) * .9)


class DistanceTests(unittest.TestCase):
    def test_resume_metadata_updates_preserve_run_id_and_training_config(self):
        from types import SimpleNamespace
        from base5_distance_diagnostics import enable_resume_metadata_updates
        backends = [dict(type='LocalVisBackend'), dict(type='MoraiWandbVisBackend', init_kwargs=dict(id='same-run', resume='allow'))]
        cfg = SimpleNamespace(visualizer=SimpleNamespace(vis_backends=backends), vis_backends=backends,
                              model={'unchanged': True}, optimizer={'unchanged': True})
        enable_resume_metadata_updates(cfg)
        self.assertEqual(backends[1]['init_kwargs'], dict(id='same-run', resume='allow', allow_val_change=True))
        self.assertEqual(backends[0], dict(type='LocalVisBackend'))
        self.assertEqual(cfg.model, {'unchanged': True})
        self.assertEqual(cfg.optimizer, {'unchanged': True})

    def test_exact_half_open_edges_and_corner_count(self):
        result = diagnostic_metrics([sample([(49.9, 0), (50, 0), (79.9, 0), (80, 0), (99.9, 0), (100, 0), (99, 29)])])
        self.assertEqual(result['range/30_50m/vehicle_GT'], 1)
        self.assertEqual(result['range/50_80m/vehicle_GT'], 2)
        self.assertEqual(result['range/80_100m/vehicle_GT'], 2)
        self.assertEqual(result['range_support/radial_ge100m/vehicle_GT'], 2)
        self.assertNotIn('range/100_infm/vehicle_mean_AP', result)

    def test_empty_class_is_unavailable_not_zero_ap(self):
        result = diagnostic_metrics([sample([(60, 0)])])
        self.assertEqual(result['range/50_80m/pedestrian_GT'], 0)
        self.assertEqual(result['range/50_80m/pedestrian_available'], 0)
        self.assertNotIn('range/50_80m/pedestrian_mean_AP', result)
        self.assertNotIn('range/80_100m/vehicle_mean_AP', result)

    def test_valid_gt_with_no_prediction_reports_zero_ap(self):
        result = diagnostic_metrics([sample([(60, 0)], prediction_xy=[])])
        self.assertEqual(result['range/50_80m/vehicle_mean_AP'], 0)
        self.assertEqual(result['range/50_80m/vehicle_available'], 1)

    def test_perfect_predictions_and_three_classes(self):
        result = diagnostic_metrics([sample([(60, 0), (65, 0), (70, 0)], [0, 1, 2])])
        for name in CLASSES:
            self.assertAlmostEqual(result[f'range/50_80m/{name}_mean_AP'], 1.)

    def test_front_and_rear_partition_counts(self):
        result = diagnostic_metrics([sample([(60, 0), (-60, 0), (-85, 0)])])
        self.assertEqual(result['range_direction/50_80m/front/vehicle_GT'], 1)
        self.assertEqual(result['range_direction/50_80m/rear/vehicle_GT'], 1)
        self.assertEqual(result['range_direction/80_100m/rear/vehicle_GT'], 1)
        self.assertNotIn('range_direction/80_100m/front/vehicle_mean_AP', result)

    def test_small_sample_flag(self):
        result = diagnostic_metrics([sample([(60, 0)] * 20)])
        self.assertEqual(result['range/50_80m/vehicle_small_sample'], 0)
        self.assertEqual(result['range/80_100m/vehicle_small_sample'], 0)

    def test_legacy_bins_are_not_recomputed_or_replaced(self):
        result = diagnostic_metrics([sample([(5, 0), (60, 0)])])
        self.assertNotIn('range/0_10m/vehicle_mean_AP', result)
        self.assertNotIn('center_mAP', result)


class IntegrationTests(unittest.TestCase):
    def test_extension_preserves_every_legacy_scalar_and_roi(self):
        import torch
        from base5_distance_diagnostics import install
        from perception.detection3d.bevfusion.plugin.morai_metric import MoraiBase5MergedCenterMetric, _Boxes
        metric = MoraiBase5MergedCenterMetric(point_cloud_range=[-100, -30, -3, 100, 30, 2])
        metric.dataset_meta = dict(classes=CLASSES)
        s = sample([(5, 0), (60, 0), (-85, 0), (99, 29), (105, 0)], [0, 1, 2, 0, 0])
        row = dict(info=dict(gt_boxes=s['gt_boxes'], gt_names=np.array(CLASSES)[s['gt_labels']],
                             valid_flag=np.ones(5, dtype=bool), num_lidar_pts=np.ones(5)),
                   prediction=dict(boxes_3d=_Boxes(s['pred_boxes']),
                                   scores_3d=torch.as_tensor(s['pred_scores']),
                                   labels_3d=torch.as_tensor(s['pred_labels'])),
                   original_semantic_class=np.array(['Vehicle', 'Pedestrian', 'Obstacle', 'Vehicle', 'Vehicle']),
                   assets=np.array(['test'] * 5))
        before = metric.compute_metrics([row])
        install()
        after = metric.compute_metrics([row])
        for key, value in before.items():
            self.assertEqual(after[key], value, key)
        self.assertEqual(after['center/vehicle_GT'], 2.)
        self.assertEqual(after['range_support/radial_ge100m/vehicle_GT'], 1.)
        self.assertEqual(after['range/50_80m/pedestrian_GT'], 1.)
        self.assertEqual(after['range_direction/80_100m/rear/vehicle_GT'], 0.)
        from mmdet3d.registry import HOOKS
        hook = HOOKS.get('Base5DistanceDiagnosticReceiptHook')()
        checkpoint = dict(meta=dict(epoch=10), optimizer={'preserve': True})
        hook.before_save_checkpoint(None, checkpoint)
        self.assertEqual(checkpoint['meta']['epoch'], 10)
        self.assertEqual(checkpoint['optimizer'], {'preserve': True})
        self.assertEqual(checkpoint['meta']['base5_distance_diagnostics']['version'], 'base5_distance_diagnostics_v1')


if __name__ == '__main__':
    unittest.main()
