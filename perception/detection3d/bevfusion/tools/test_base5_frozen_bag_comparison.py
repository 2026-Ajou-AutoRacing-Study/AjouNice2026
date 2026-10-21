"""CPU regressions for same-input two-checkpoint development comparisons."""
import copy
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import pickle
import tempfile
import unittest

from summarize_base5_frozen_bag_comparison import (
    THRESHOLDS, ap_summary, compare_contract, summarize, summarize_rows, verify_authored_support,
)
from compare_base5_common30 import gt_identity
from evaluate_base5_far_retention import full_prepare, metric_module, range_operating


def contract():
    return dict(gt_identity='gt', config_sha256='cfg', info_sha256='info',
                inference_precision='AMP', point_intensity_scale=1. / 255.,
                script_sha256='script', gpu='same GPU', torch_version='same torch',
                input_pipeline=[dict(type='same')], strict_state_load=True,
                inference_optimization=dict(compile_parts=[], camera_cuda_graph=False,
                                            parallel_camera=True))


class FrozenBagComparisonTests(unittest.TestCase):
    def test_current_production_scores_not_legacy_ped015(self):
        self.assertEqual(THRESHOLDS, (.4, .1, .1))

    def test_same_runtime_different_checkpoint_is_allowed(self):
        first, second = contract(), contract()
        first['checkpoint_sha256'], second['checkpoint_sha256'] = 'old', 'new'
        compare_contract(first, second)

    def test_mixed_precision_config_intensity_and_gt_are_rejected(self):
        for key, value in [('config_sha256', 'other'), ('gt_identity', 'other'),
                           ('inference_precision', 'FP32'), ('point_intensity_scale', 1.)]:
            changed = contract()
            changed[key] = value
            with self.assertRaises(ValueError):
                compare_contract(contract(), changed)

    def test_compile_is_outside_contract(self):
        candidate = contract()
        candidate['inference_optimization']['compile_parts'] = ['camera']
        with self.assertRaises(ValueError):
            compare_contract(candidate, candidate)

    def test_missing_class_is_null_not_zero_performance(self):
        row = dict(gt_boxes=[[20., 0., 0., 2., 2., 1., 0.]], gt_labels=[2],
                   gt_points=[0], gt_assets=['barrel'], pred_boxes=[],
                   pred_labels=[], pred_scores=[])
        prepared = full_prepare([row])
        result = ap_summary(prepared, metric_module())
        self.assertIsNone(result['classes']['vehicle']['ap'])
        self.assertEqual(result['classes']['obstacle']['gt'], 1)
        self.assertEqual(result['classes']['obstacle']['ap'], 0.)

    def test_authored_support_and_assets_are_immutable(self):
        source = dict(token='a', instances=[dict(num_lidar_pts=0, asset='box')])
        payload = dict(rows=[dict(token='a', gt_points=[0], gt_assets=['box'])])
        verify_authored_support([source], payload)
        changed = copy.deepcopy(payload)
        changed['rows'][0]['gt_points'] = [5]
        with self.assertRaises(ValueError):
            verify_authored_support([source], changed)

    def test_band_boundary_does_not_create_fixed_score_false_positive(self):
        row = dict(token='a', case='test', stamp=1., native_index=0,
                   gt_boxes=[[49.9, 0., 0., 4., 2., 1., 0.]], gt_labels=[0],
                   gt_points=[0], gt_assets=['car'], gt_track_keys=['one'],
                   pred_boxes=[[50.1, 0., 0., 4., 2., 1., 0.]],
                   pred_labels=[0], pred_scores=[.9])
        result = range_operating(full_prepare([row]), 0, .4, .1)
        self.assertEqual(result['30_50m']['matched_gt_counts']['2.0'], 1)
        self.assertEqual(result['50_80m']['unmatched_predictions_by_center_gate']['2.0'], 0)

    def test_pooled_miss_time_is_null_but_case_specific_cadence_is_preserved(self):
        row = dict(token='a', case='test', stamp=1., native_index=0,
                   gt_boxes=[[20., 0., 0., 2., 2., 1., 0.]], gt_labels=[2],
                   gt_points=[0], gt_assets=['box'], gt_track_keys=['one'],
                   pred_boxes=[], pred_labels=[], pred_scores=[])
        prepared = full_prepare([row])
        pooled = summarize_rows(prepared, metric_module())
        single = summarize_rows(prepared, metric_module(), period=.084)
        for values in pooled['operating'].values():
            for band in values['by_range'].values():
                self.assertIsNone(band['miss_runs_gate2m'])
        self.assertEqual(single['operating']['obstacle']['by_range']['0_30m']
                         ['miss_runs_gate2m']['max_seconds'], .084)
        self.assertEqual(pooled['operating']['obstacle']['by_range']['0_30m']['gt'],
                         single['operating']['obstacle']['by_range']['0_30m']['gt'])

    def test_complete_pair_uses_only_authored_scoring_frames_and_never_deletes_zero_support(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            instance = dict(bbox_3d=[20., 0., .5, 2., 2., 1., 0.],
                            bbox_label_3d=2, num_lidar_pts=0, asset='box', track_key='one')
            records = [dict(token=str(i), scene_id='test', timestamp=1. + i * .1,
                            raw_lidar_index=i, scoring_eligible=i == 0,
                            instances=[copy.deepcopy(instance)]) for i in range(2)]
            (root / 'infos_val.pkl').write_bytes(pickle.dumps(dict(data_list=records)))
            (root / 'manifest.json').write_text(json.dumps(dict(
                frames=2, scoring_frames=1, gt_reference='test catalog')))
            rows = [dict(token=str(i), gt_boxes=[[20., 0., 0., 2., 2., 1., 0.]],
                         gt_labels=[2], gt_points=[0], gt_assets=['box'],
                         pred_boxes=[], pred_labels=[], pred_scores=[]) for i in range(2)]
            predictions = []
            for name in ('old', 'new'):
                payload = dict(contract(), name=name, checkpoint_sha256=name,
                               classes=['vehicle', 'pedestrian', 'obstacle'], frames=2,
                               rows=rows, gt_identity=gt_identity(rows))
                payload['info_sha256'] = hashlib.sha256((root / 'infos_val.pkl').read_bytes()).hexdigest()
                path = root / (name + '.json.gz')
                with gzip.open(path, 'wt') as handle:
                    json.dump(payload, handle)
                predictions.append(path)
            output = root / 'comparison.json'
            report = summarize(argparse.Namespace(inputs=root, predictions=predictions,
                                                   output=output, max_frames=2))
            self.assertTrue(report['complete'])
            self.assertEqual(report['scoring_frames'], 1)
            for model in report['models'].values():
                self.assertEqual(model['pooled']['semantic_roi']['classes']['obstacle']['gt'], 1)
                self.assertEqual(model['pooled']['operating']['obstacle']['semantic_roi']['gt'], 1)
                self.assertEqual(model['excluded_by_case'], {'test': 1})
            with self.assertRaises(ValueError):
                summarize(argparse.Namespace(inputs=root, predictions=predictions,
                                              output=output, max_frames=2))


if __name__ == '__main__':
    unittest.main()
