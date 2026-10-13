#!/usr/bin/env python3
import copy
import unittest

from experiment_base5_nearest_compile import final_difference, query_difference, scoring_rows


def item(key=(0, 1, 2), score=.5, x=0, yaw=0):
    return dict(key=list(key), score=score, box=[x, 0, -1, 4, 2, 1, yaw])


class QueryAuditTests(unittest.TestCase):
    def test_identical_cell_query_zero_difference(self):
        rows = [dict(queries=[item()])]
        result = query_difference(rows,rows)
        self.assertEqual(result['distributions']['raw/vehicle/center_m']['max'],0)
        self.assertEqual(result['threshold_crossings'],{})

    def test_key_order_not_sorted_score_order(self):
        a = [dict(queries=[item(),item((2,3,4),.2,x=5)])]
        b = [dict(queries=list(reversed(a[0]['queries'])))]
        self.assertEqual(query_difference(a,b)['distributions']['raw/vehicle/center_m']['max'],0)

    def test_threshold_crossing_visible(self):
        a,b = [dict(queries=[item()])],[dict(queries=[item(score=.39,x=.2)])]
        result = query_difference(a,b)
        self.assertEqual(result['threshold_crossings']['vehicle/threshold_crossing'],1)
        self.assertAlmostEqual(result['distributions']['operating_either/vehicle/center_m']['mean'],.2)

    def test_changed_cells_not_force_matched(self):
        result = query_difference([dict(queries=[item()])],[dict(queries=[item((0,9,9))])])
        self.assertEqual(result['query_set_changes'],{'vehicle/query_removed':1,'vehicle/query_added':1})
        self.assertEqual(result['distributions'],{})

    def test_duplicate_cell_rejected(self):
        with self.assertRaises(ValueError):
            query_difference([dict(queries=[item(),item()])],[dict(queries=[item()])])

    def test_no_source_mutation(self):
        a = [dict(queries=[item()])]
        saved = copy.deepcopy(a)
        query_difference(a,a)
        self.assertEqual(a,saved)


class FinalDetectorTests(unittest.TestCase):
    def row(self,score=.5,x=0,label=0):
        return dict(token='same',pred_boxes=[[x,0,-1,4,2,1,0]],pred_labels=[label],pred_scores=[score])

    def test_final_gate_crossing_counts_disappearance(self):
        result = final_difference([self.row()],[self.row(score=.39)])
        self.assertEqual(result['counts']['vehicle/baseline_only'],1)
        self.assertNotIn('vehicle/matched',result['counts'])

    def test_final_geometry_all_class_match(self):
        result = final_difference([self.row(label=2)],[self.row(x=.05,label=2)])
        self.assertEqual(result['counts']['obstacle/matched'],1)
        self.assertAlmostEqual(result['distributions']['obstacle/center_m']['p95'],.05)

    def test_unmatched_not_force_large_shift(self):
        result = final_difference([self.row()],[self.row(x=3)])
        self.assertEqual(result['counts']['vehicle/baseline_only'],1)
        self.assertEqual(result['counts']['vehicle/candidate_only'],1)
        self.assertEqual(result['distributions'],{})


class MetadataCompatibilityTests(unittest.TestCase):
    def test_legacy_scalar_metadata_removed_on_copy_only(self):
        source=[dict(token='same',gt_center_convention='bottom',gt_boxes=[[1,2,3,4,5,6,7]])]
        before=copy.deepcopy(source)
        result=scoring_rows(source)
        self.assertNotIn('gt_center_convention',result[0])
        self.assertEqual(result[0]['box_center_convention'],'bottom')
        self.assertEqual(source,before)
        self.assertIs(result[0]['gt_boxes'],source[0]['gt_boxes'])

    def test_unknown_metadata_type_is_not_silently_deleted(self):
        with self.assertRaises(ValueError):
            scoring_rows([dict(gt_center_convention=['per-observation'])])


if __name__=='__main__':
    unittest.main()
