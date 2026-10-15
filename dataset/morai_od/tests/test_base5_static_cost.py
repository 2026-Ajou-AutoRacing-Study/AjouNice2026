import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json
from report_base5_static_cost import report


class StaticCostTests(unittest.TestCase):
    def fixture(self,root):
        atomic_json(root/'status.json',dict(stage='complete_with_deficit',elapsed_sec=30))
        atomic_json(root/'quota.json',dict(target=2,accepted=1,deficit=1))
        manifest=root/'manifest.json'
        atomic_json(manifest,dict(config=dict(families=dict(highway=1500)),
            provenance=dict(static_population_family_counts=dict(highway=1500)),
            scenes=[dict(family='highway'),dict(family='highway')]))
        (root/'selected_ledger.jsonl').write_text(json.dumps(dict(id='a',source_cell_id='a',family='highway'))+'\n')
        child=root/'child';child.mkdir();atomic_json(child/'session_contract.json',{})
        atomic_json(child/'shutdown_safety.json',dict(park_verified=True))
        captures=[]
        for name,status in (('a','accepted_candidate'),('b','review')):
            capture=root/name;capture.mkdir();atomic_json(capture/'metadata.json',dict(runtime_actors=[]))
            captures.append(dict(id=name,family='highway',status=status,duration_sec=10,bytes=1,
                capture_root=str(capture),audit=dict(ground_warnings=[dict(reason='actor_ground_plane_unverified')])))
        (child/'ledger.jsonl').write_text('\n'.join(json.dumps(x) for x in [captures[0],captures[0],captures[1]])+'\n')
        return manifest

    def test_rejected_cost_included_and_resume_rows_deduplicated(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);result=report(root,self.fixture(root))
            self.assertEqual(result['actual_captures'],2)
            self.assertEqual(result['seconds_per_accepted'],30)
            self.assertEqual(result['family_seconds_per_accepted']['highway'],30)
            self.assertEqual(result['ground_warning_counts']['actor_ground_plane_unverified'],2)
            self.assertAlmostEqual(result['extrapolated_static4171_hours_family_weighted'],12.5)

    def test_moving_or_calibration_is_not_static_cost(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);manifest=self.fixture(root)
            atomic_json(root/'a/metadata.json',dict(runtime_actors=[],calibration_only=True))
            with self.assertRaises(ValueError):report(root,manifest)


if __name__=='__main__':unittest.main()
