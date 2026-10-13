import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import report_base5_final_coverage as coverage


class FinalCoverageTests(unittest.TestCase):
    def fixture(self,root):
        actor=dict(data_id=11,unique_id=8500,name='A',actual_ego_role='front')
        meta=root/'meta.json';meta.write_text(json.dumps(dict(runtime_actors=[actor],spec=dict(family='highway'))))
        payload=root/'scene.json';payload.write_text('{}')
        manifest=root/'manifest.json';manifest.write_text(json.dumps(dict(config=dict(target_scenes=1,
            training_roi_candidate=[-100,-30,-3,100,30,2],coverage_policy=dict(minimum_primary_scenes_per_asset=1)),
            scenes=[dict(id='cell',scenario=str(payload),scenario_sha256=coverage.gen.geo.sha256(payload),metadata=str(meta))])))
        pools={k:[] for k in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')}
        pools['vehicle']=[dict(data_id=11,name='A')]
        return manifest,dict(pools=pools),actor

    def capture(self,root,actor,positive=True,calibration=False):
        cap=root/'capture';cap.mkdir()
        (cap/'metadata.json').write_text(json.dumps(dict(runtime_actors=[actor],calibration_only=calibration)))
        (cap/'audit.json').write_text(json.dumps(dict(review_reasons=[],components=[dict(parent_uid=8500,
            positive_lidar1_camera0=positive,center_lidar_m=[5,0,0])])) )
        return dict(id='attempt',source_cell_id='cell',status='accepted_candidate',capture_root=str(cap))

    def test_planned_not_claimed_visible(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);m,ctx,_=self.fixture(root)
            with patch.object(coverage.gen,'context',return_value=ctx):result=coverage.report(m)
            self.assertEqual(result['status'],'PLANNED_PASS')
            self.assertEqual(result['effective_coverage_status'],'NOT_CAPTURED')

    def test_duplicate_attempt_does_not_inflate_coverage(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);m,ctx,a=self.fixture(root);row=self.capture(root,a)
            ledger=root/'ledger';ledger.write_text((json.dumps(row)+'\n')*2)
            with patch.object(coverage.gen,'context',return_value=ctx):result=coverage.report(m,ledger)
            self.assertEqual(result['accepted_source_scenes'],1)
            self.assertEqual(result['assets'][0]['visible_positive_primary'],1)
            self.assertEqual(result['effective_coverage_status'],'PASS')

    def test_invisible_primary_not_effective_gt(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);m,ctx,a=self.fixture(root);row=self.capture(root,a,positive=False)
            ledger=root/'ledger';ledger.write_text(json.dumps(row)+'\n')
            with patch.object(coverage.gen,'context',return_value=ctx):result=coverage.report(m,ledger)
            self.assertEqual(result['effective_coverage_status'],'FAIL')
            self.assertEqual(result['effective_primary_deficits'],{'A':1})

    def test_calibration_not_accepted_training_scene(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);m,ctx,a=self.fixture(root);row=self.capture(root,a,calibration=True)
            ledger=root/'ledger';ledger.write_text(json.dumps(row)+'\n')
            with patch.object(coverage.gen,'context',return_value=ctx):result=coverage.report(m,ledger)
            self.assertEqual(result['accepted_source_scenes'],0)
            self.assertEqual(result['effective_coverage_status'],'PARTIAL')

    def test_payload_modification_detected(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);m,ctx,_=self.fixture(root);(root/'scene.json').write_text('{"changed":true}')
            with patch.object(coverage.gen,'context',return_value=ctx):result=coverage.report(m)
            self.assertEqual(result['status'],'FAIL')

    def test_wrong_replacement_asset_does_not_fill_primary_quota(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);m,ctx,a=self.fixture(root)
            a=dict(a,data_id=12);row=self.capture(root,a)
            ledger=root/'ledger';ledger.write_text(json.dumps(row)+'\n')
            with patch.object(coverage.gen,'context',return_value=ctx):result=coverage.report(m,ledger)
            self.assertEqual(result['status'],'FAIL')
            self.assertEqual(result['assets'][0]['visible_positive_primary'],0)


if __name__=='__main__':unittest.main()
