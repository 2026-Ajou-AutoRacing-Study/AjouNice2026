import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from report_base5_setup_budget_eta import report


class BudgetEtaTest(unittest.TestCase):
    def test_only_natural_cells_pay_extra_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);meta=root/'meta.json';meta.write_text(json.dumps({'runtime_actors':[
                {'motion_path':{'policy':'natural-target-v1'}}]}))
            manifest=root/'manifest.json';manifest.write_text(json.dumps({'scenes':[
                {'metadata':str(meta)}]*10000}))
            cost=root/'cost.json';cost.write_text(json.dumps({'all_quota_and_shutdown_verified':True,
                'extrapolated_10k_hours_family_weighted':18.7}))
            result=report(manifest,cost,30.,(.01,))
            self.assertAlmostEqual(result['sensitivity'][0]['extra_hours'],100*30/3600)
            self.assertEqual(result['natural_motion_cells'],10000)
            meta.write_text(json.dumps({'runtime_actors':[]}))
            self.assertEqual(report(manifest,cost)['sensitivity'][-1]['extra_hours'],0)
            with self.assertRaises(ValueError):report(manifest,cost,float('nan'))
            with self.assertRaises(ValueError):report(manifest,cost,rates=(1.1,))
            cost.write_text('{}')
            with self.assertRaises(ValueError):report(manifest,cost)
