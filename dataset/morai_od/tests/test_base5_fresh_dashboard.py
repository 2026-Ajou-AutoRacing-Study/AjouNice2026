import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_fresh_dashboard as d


class FollowingTests(unittest.TestCase):
    def test_header_reads_only_bounded_prefix_of_large_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'state.json'
            path.write_text(json.dumps({'contract':{'source_sha256':'sha','baseline':'root'},'large_baseline':'x'*100000}))
            self.assertEqual(d.lineage_header(path),{'source_sha256':'sha','baseline':'root'})
            path.write_text('{"unrelated":{"source_sha256":"sha","baseline":"root"}}')
            self.assertEqual(d.lineage_header(path),{})
    def setup_paths(self,root):
        parent=root/'capture';parent.mkdir();child=root/'capture_completed';child.mkdir()
        (child/'state.json').write_text(json.dumps({'contract':{'source_sha256':'sha','baseline':str(parent.resolve())}}))
        (child/'status.json').write_text(json.dumps({'flexible_completion':True}))
        (child/'selected_ledger.jsonl').write_text('')
        return parent,child

    def test_follows_only_matching_parent_and_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            parent,child=self.setup_paths(Path(tmp))
            self.assertEqual(d.completion_root(parent,'sha'),child)
            self.assertEqual(d.completion_root(parent,'wrong'),parent)
            (child/'state.json').write_text(json.dumps({'contract':{'source_sha256':'sha','baseline':'unrelated'}}))
            self.assertEqual(d.completion_root(parent,'sha'),parent)

    def test_missing_selection_never_switches(self):
        with tempfile.TemporaryDirectory() as tmp:
            parent,child=self.setup_paths(Path(tmp));(child/'selected_ledger.jsonl').unlink()
            self.assertEqual(d.completion_root(parent,'sha'),parent)

    def test_suite_never_adds_parent_to_completion(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);parent,child=self.setup_paths(root)
            (parent/'status.json').write_text(json.dumps({'stage':'complete_with_deficit','accepted_candidate':63,'requested':64,'finished':64,'review':1,'elapsed_sec':100}))
            (child/'status.json').write_text(json.dumps({'flexible_completion':True,'stage':'complete','accepted_candidate':64,'requested':64,'finished':64,'review':0,'elapsed_sec':110}))
            suite=root/'suite.json';suite.write_text(json.dumps({'environments':[{'environment':'sunny11','capture_output':str(parent),'manifest_sha256':'sha','expected_frames':64}]}))
            dashboard=d.FollowingDashboard(parent,suite=suite);dashboard.parent.manifest_sha256='sha'
            body=dashboard.payload();self.assertEqual(body['suite_accepted'],64)
            self.assertTrue(body['canonical_selection_followed']);self.assertEqual(body['review'],0)


if __name__=='__main__':unittest.main()
