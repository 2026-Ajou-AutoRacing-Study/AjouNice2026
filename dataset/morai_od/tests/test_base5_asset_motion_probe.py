import sys
from pathlib import Path
import unittest
import json
from unittest.mock import MagicMock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from prepare_base5_asset_motion_probe import CASES,appearance_height_calibration
from report_base5_asset_motion_probe import condition_valid,component_size_changes


class AssetMotionProbeTest(unittest.TestCase):
    def test_unique_representative_assets(self):
        self.assertEqual(len(CASES),13)
        self.assertEqual(len({name for name,_ in CASES}),len(CASES))
        self.assertTrue(all(speed>0 for _,speed in CASES))

    def test_static_requires_observed_stationarity(self):
        self.assertTrue(condition_valid('static',0,0,.4))
        self.assertFalse(condition_valid('static',None,0,.4))
        self.assertFalse(condition_valid('static',.03,0,.4))

    def test_moving_requires_motion_and_target(self):
        self.assertTrue(condition_valid('moving',1.1,.3,.4))
        self.assertFalse(condition_valid('moving',0,0,.4))
        self.assertFalse(condition_valid('moving',1.1,.5,.4))
        self.assertFalse(condition_valid('moving',float('nan'),0,.4))

    def test_height_match_is_measured_not_gt_rewrite(self):
        p=MagicMock();p.resolve.return_value=Path('/example/report.json')
        rows=[dict(asset=name,condition_valid=True,mode=mode,object_position_before={'z':z})
              for name,_ in CASES for mode,z in [('static',28.),('moving',28.65)] for _ in range(2)]
        p.read_text.return_value=json.dumps({'rows':rows})
        c=appearance_height_calibration(p)
        self.assertAlmostEqual(c['CargoBox']['shift'],-.65)
        self.assertEqual(c['CargoBox']['static_z'],28.)
        self.assertEqual(rows[0]['object_position_before']['z'],28.)

    def test_incomplete_height_calibration_rejected(self):
        p=MagicMock();p.read_text.return_value=json.dumps({'rows':[]})
        with self.assertRaises(ValueError):appearance_height_calibration(p)

    def test_component_deltas_do_not_merge_person_and_stroller(self):
        rows=[dict(mode=mode,condition_valid=True,components=[
            dict(component_id='person',size_m=[length,.5,1.7]),
            dict(component_id='stroller',size_m=[1.,.6,1.])])
            for mode,length in [('static',.3),('moving',.7)]]
        changes=component_size_changes(rows)
        self.assertAlmostEqual(changes['person']['maximum_axis_median_difference_m'],.4)
        self.assertEqual(changes['stroller']['maximum_axis_median_difference_m'],0.)


if __name__=='__main__':unittest.main()
