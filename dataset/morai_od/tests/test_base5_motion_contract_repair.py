import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_motion_contract_repair as repair


def meta(*motions):
    return dict(runtime_actors=[dict(motion=m) for m in motions])


class MotionRepairTests(unittest.TestCase):
    def test_static_secondary_cannot_replace_walking_secondary(self):
        self.assertFalse(repair.motion_matches(meta('standing','walking'),meta('standing','standing')))
        self.assertTrue(repair.motion_matches(meta('standing','walking'),meta('standing','walking')))

    def test_running_walking_and_primary_are_not_interchangeable(self):
        self.assertFalse(repair.motion_matches(meta('walking','standing'),meta('standing','walking')))
        self.assertFalse(repair.motion_matches(meta('standing','running'),meta('standing','walking')))
        self.assertFalse(repair.motion_matches(meta('standing','walking','walking'),meta('standing','walking','standing')))

    def test_source_static_policy_is_not_changed(self):
        self.assertTrue(repair.motion_matches(meta('standing'),meta('standing')))
        self.assertTrue(repair.motion_matches(meta('static'),meta('static')))

    def test_added_secondary_movers_are_valid_random_diversity(self):
        self.assertTrue(repair.motion_matches(meta('running','standing'),meta('running','running')))
        self.assertTrue(repair.motion_matches(meta('walking','standing'),meta('walking','walking')))

    def test_generation_filter_is_applied_and_restored(self):
        original=repair.ac.fa.gen.build
        source=meta('standing','walking')
        with patch.object(repair.ac.fa,'read',return_value=source),patch.object(repair.ac.fa.gen,'build',return_value=({},meta('standing','standing'))):
            def generate(*args,**kwargs):return repair.ac.fa.gen.build({}, {}, {})
            with patch.object(repair.ac,'prepare',side_effect=generate):
                with self.assertRaisesRegex(ValueError,'signature changed'):
                    repair.prepare(dict(metadata='source'),{},None,None,[],1,Path('unused'),lambda:None)
        self.assertIs(repair.ac.fa.gen.build,original)

    def test_partition_preserves_good_rows_and_bad_raw(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'source.json';repair.ac.fa.atomic_json(source,meta('standing','walking'))
            rows=[];lookup={}
            for ident,motions in [('good',('standing','walking')),('bad',('standing','standing'))]:
                capture=root/ident;capture.mkdir();repair.ac.fa.atomic_json(capture/'metadata.json',meta(*motions))
                rows.append(dict(source_cell_id=ident,capture_root=str(capture),status='accepted_candidate'))
                lookup[ident]=dict(metadata=str(source))
            original=copy.deepcopy(rows);kept,excluded=repair.partition(rows,lookup)
            self.assertEqual(kept,[rows[0]]);self.assertEqual(excluded[0]['original_selected'],rows[1])
            self.assertEqual(rows,original);self.assertTrue((root/'bad/metadata.json').exists())

    def test_inner_quality_replacement_is_checked_again(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'source.json';repair.ac.fa.atomic_json(source,meta('standing','walking'))
            capture=root/'raw';capture.mkdir();repair.ac.fa.atomic_json(capture/'metadata.json',meta('standing','standing'))
            row=dict(status='accepted_candidate',capture_root=str(capture))
            with patch.object(repair.ac,'verify',return_value=row):
                result=repair.verify({},dict(metadata=str(source)),{})
            self.assertEqual(result['status'],'review');self.assertEqual(result['observed_selected'],row)

    def test_runtime_review_not_rewritten(self):
        row=dict(status='review',reasons=['ground'])
        with patch.object(repair.ac,'verify',return_value=row):self.assertEqual(repair.verify({}, {}, {}),row)


if __name__=='__main__':unittest.main()
