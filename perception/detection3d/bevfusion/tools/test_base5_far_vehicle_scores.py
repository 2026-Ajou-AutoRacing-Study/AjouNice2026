import json
import unittest

import numpy as np

from analyze_base5_far_vehicle_scores import json_safe, pooled_ranges, prepare_cases, score_diagnostics
from compare_base5_common30 import gt_identity
from evaluate_base5_far_retention import full_prepare, range_operating
from test_base5_far_retention import row


class FarVehicleScoreTests(unittest.TestCase):
    def test_band_crossing_matches_are_not_false_positives(self):
        metrics = range_operating(full_prepare([row()]), 0, .4, .1)
        result = pooled_ranges({'a': dict(by_range=metrics), 'b': dict(by_range=metrics)})
        self.assertEqual(result['30_50m']['matched_gt_counts']['2.0'], 2)
        self.assertEqual(result['50_80m']['unmatched_predictions_by_center_gate']['2.0'], 0)
        self.assertEqual(result['50_80m']['precision_by_center_gate']['2.0'], 1.)
        self.assertIsNone(result['50_80m']['recall_by_center_gate']['2.0'])

    def test_low_scores_add_matches_without_relabelling(self):
        sample = row(60., 60.2)
        sample['pred_scores'] = [.3]
        prepared = full_prepare([sample])
        self.assertEqual(score_diagnostics(prepared, .4)['additional_matched_gt_observations_below_reference'], {})
        self.assertEqual(score_diagnostics(prepared, .2)['additional_matched_gt_observations_below_reference'],
                         {'50_80m': 1})

    def test_duplicates_are_separate_from_unknown_background(self):
        sample = row(60., 60.2)
        sample.update(pred_boxes=[sample['pred_boxes'][0], [61., 0, 0, 4, 2, 1, 0], [90., 0, 0, 4, 2, 1, 0]],
                      pred_labels=[0, 0, 0], pred_scores=[.8, .3, .2], pred_track_ids=[1, 2, 3])
        diagnostics = score_diagnostics(full_prepare([sample]), .1)
        self.assertEqual(diagnostics['unmatched_kinds_gate2m'],
                         {'same_gt_1_to_2m': 1, 'no_catalog_actor_within4m': 1})

    def test_only_vehicle_candidates_change(self):
        sample = row(60., 60.2)
        sample.update(pred_boxes=[[60.2, 0, 0, 4, 2, 1, 0]], pred_labels=[2], pred_scores=[.3])
        self.assertEqual(score_diagnostics(full_prepare([sample]), .1)['unmatched_kinds_gate2m'], {})

    def test_full_roi_eligibility_and_geometry_are_guarded(self):
        sample = row()
        payload = dict(classes=['vehicle', 'pedestrian', 'obstacle'], rows=[sample], frames=1,
                       gt_identity=gt_identity([sample]))
        instance = dict(bbox_3d=[49.9, 0, .5, 4, 2, 1, 0], bbox_label_3d=0, track_key='gt1')
        source = dict(token='a', scene_id='case', timestamp=1., raw_lidar_index=0,
                      scoring_eligible=False, instances=[instance])
        cases, excluded = prepare_cases([source], payload)
        self.assertEqual(cases, {})
        self.assertEqual(excluded, {'case': 1})
        source['instances'][0]['bbox_3d'][0] += 1
        with self.assertRaises(ValueError):
            prepare_cases([source], payload)

    def test_numpy_counts_are_serialized_without_nan(self):
        self.assertEqual(json_safe({'count': np.int64(2), 'hit': np.bool_(True)}),
                         {'count': 2, 'hit': True})
        json.dumps(json_safe(pooled_ranges({'a': dict(by_range=range_operating(
            full_prepare([row()]), 0, .4, .1))})), allow_nan=False)


if __name__ == '__main__':
    unittest.main()
