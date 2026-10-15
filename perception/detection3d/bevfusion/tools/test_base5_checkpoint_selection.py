import copy
import gzip
import json
from pathlib import Path
import tempfile
import unittest

from compare_base5_checkpoint_selection import attach_gt, aggregate, verify_repeat
from compare_base5_common30 import CLASSES, gt_identity
from evaluate_base5_far_retention import full_prepare, range_operating
from calibrate_base5_thresholds import score_curve


def fixture():
    row = dict(token='one', gt_boxes=[], gt_labels=[], gt_points=[], gt_assets=[],
        pred_boxes=[[50.,0.,0.,4.,2.,1.,0.]], pred_labels=[0], pred_scores=[.8])
    payload = dict(classes=CLASSES, frames=1, rows=[row], gt_identity=gt_identity([row]))
    sources = {'one': dict(instances=[dict(bbox_3d=[50.,0.,.5,4.,2.,1.,0.],
        bbox_label_3d=0, num_lidar_pts=0, asset='car')])}
    return payload, sources


class CheckpointSelectionTests(unittest.TestCase):
    def replay_fixture(self, root, shifted=False, repeated_change=False):
        receipt={'cases':[{'case':'case', 'input_frames':1}]}
        (root/'receipt.json').write_text(json.dumps(receipt))
        data={}
        for mode in ('off','on','off_repeat'):
            for stage in ('raw','policy'):
                data[f'case/{mode}/{stage}']=[{'stamp':2. if shifted and mode=='on' else 1.,
                    'objects':[{'id':2 if repeated_change and mode=='off_repeat' else 1}]}]
        with gzip.open(root/'messages.json.gz','wt') as f:
            json.dump(data,f)

    def test_reference_repeat_and_complete_candidate_stamps(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            self.replay_fixture(root)
            self.assertTrue(verify_repeat(root)['exact_reference_repeat'])

    def test_shifted_candidate_stamp_is_not_complete_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            self.replay_fixture(root,shifted=True)
            with self.assertRaises(ValueError):
                verify_repeat(root)

    def test_independent_repeat_object_change_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            self.replay_fixture(root,repeated_change=True)
            with self.assertRaises(ValueError):
                verify_repeat(root)

    def test_attach_preserves_predictions_and_source(self):
        payload, sources = fixture()
        original = copy.deepcopy(payload)
        result = attach_gt(payload, sources)
        self.assertEqual(payload, original)
        self.assertEqual(result['rows'][0]['pred_boxes'], original['rows'][0]['pred_boxes'])
        self.assertEqual(result['rows'][0]['gt_boxes'][0][2], 0.)
        self.assertEqual(result['rows'][0]['gt_points'], [0])
        self.assertNotEqual(result['gt_identity'], original['gt_identity'])

    def test_attach_requires_exact_complete_tokens(self):
        payload, sources = fixture()
        with self.assertRaises(ValueError):
            attach_gt(payload, {'another': sources['one']})

    def test_aggregate_counts_no_gt_and_range_boundary(self):
        payload, sources = fixture()
        r = attach_gt(payload, sources)['rows'][0]
        r.update(stamp=1., native_index=0, gt_track_keys=['car'])
        prepared = full_prepare([r])
        classes = {c: dict(overall=score_curve(prepared,i,.4,True,.1),
            by_range=range_operating(prepared,i,.4,.1)) for i,c in enumerate(CLASSES)}
        result = aggregate([classes,classes])
        self.assertEqual(result['vehicle']['gt'], 2)
        self.assertEqual(result['vehicle']['recall2m'], 1.)
        self.assertEqual(result['vehicle']['by_range']['50_80m']['matched_gt_counts']['2.0'], 2)
        self.assertIsNone(result['pedestrian']['recall2m'])
        self.assertIsNone(result['vehicle']['by_range']['0_30m']['recall']['2.0'])


if __name__ == '__main__':
    unittest.main()
