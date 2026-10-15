#!/usr/bin/env python3
import copy
import unittest

import numpy as np
from analyze_base5_proposal_peaks import distribution, paired, prepared, summarize


def row(gt=((0, 0),), pred=((0, 0),), labels=None):
    labels = labels or [2] * len(gt)
    box = lambda xy: [*xy, -1, .2, .2, 1, 0]
    return dict(token='same', source='native', case='native_val',
                gt_boxes=[box(xy) for xy in gt], gt_labels=labels, gt_points=[20] * len(gt),
                gt_assets=['asset' + str(i) for i in range(len(gt))], parent_uids=list(range(len(gt))),
                pred_boxes=[box(xy) for xy in pred], pred_labels=[2] * len(pred),
                pred_scores=[.5 - i * .01 for i in range(len(pred))], query_scores=[],
                point_sha256='p', image_sha256={'camera': 'i'})


class AuditTests(unittest.TestCase):
    def test_roi_parent_alignment(self):
        r = row(gt=((200, 0), (0, 0)), pred=((0, 0),))
        result = prepared(r)
        self.assertEqual(result['parent_uids'].tolist(), [1])
        self.assertEqual(result['gt_assets'].tolist(), ['asset1'])

    def test_single_gt_extra_is_diagnostic(self):
        values = summarize([row(pred=((0, 0), (.01, .01)))], False)['classes']['obstacle']
        self.assertEqual((values['gt'], values['tp1m'], values['unmatched'], values['proximity_extras']), (1, 1, 1, 1))

    def test_distinct_close_not_duplicate(self):
        r = row(gt=((0, 0), (.8, 0)), pred=((0, 0), (.8, 0), (.4, 0)))
        values = summarize([r], False)['classes']['obstacle']
        self.assertEqual((values['tp1m'], values['proximity_extras'], values['gt_ambiguous_extra']), (2, 0, 1))
        result = paired([r], [r])
        self.assertEqual(result['distinct_close_pairs'][0]['baseline_hits'], 2)
        self.assertEqual(result['distinct_close_pairs'][0]['candidate_hits'], 2)

    def test_shared_gt_not_modified(self):
        r = row()
        original = copy.deepcopy(r)
        summarize([r], False)
        summarize([r], True)
        paired([r], [r])
        self.assertEqual(r, original)

    def test_score_gate_by_class(self):
        r = row(labels=[0])
        r['pred_labels'] = [0]
        r['pred_scores'] = [.3]
        v = summarize([r], False)['classes']['vehicle']
        self.assertEqual((v['tp1m'], v['predictions']), (0, 0))

    def test_different_raw_rejected(self):
        a, b = row(), row()
        b['point_sha256'] = 'new'
        with self.assertRaises(ValueError):
            paired([a], [b])

    def test_pair_hit_change(self):
        a, b = row(), row()
        b['pred_scores'] = [.05]
        self.assertEqual(paired([a], [b])['classes']['obstacle']['lost'], 1)

    def test_far_dev_not_false_positive_claim(self):
        r = row(gt=((60, 0),), pred=((60, 0),))
        r['source'] = 'dev'
        v = summarize([r], False)['classes']['obstacle']
        self.assertEqual((v['gt'], v['predictions']), (0, 0))

    def test_finite_metric_distribution(self):
        d = distribution([0, 1, 2, 3])
        self.assertEqual((d['n'], d['mean'], d['p50'], d['max']), (4, 1.5, 1.5, 3))
        self.assertIsNone(distribution([])['p95'])


if __name__ == '__main__':
    unittest.main()
