import copy
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_intersection_placement as placement
import base5_intersection_ground as ground
from base5_capture_safety import atomic_json,sha256
import generate_base5_independent as gen


class RegionalPlacementTest(unittest.TestCase):
    def test_region_opt_in_does_not_change_old_contract(self):
        self.assertIsNone(placement.regional_motion(dict(name='Woman2'),'walking',{},{}))
        cfg=dict(capture=dict(dynamic_region=placement.SCOPE))
        asset=dict(name='Woman2',source_list='pedestrianList',semantic_class='pedestrian')
        for row in (dict(mission=False),dict(mission=True,avoidance_phase='alongside')):
            self.assertEqual(gen.population_appearance(asset,'walking',row,0,0,cfg,{})[0],'standing')
        row=dict(mission=True,dynamic_observation_phase='crossing')
        self.assertEqual(gen.population_appearance(asset,'walking',row,0,0,cfg,{})[0],'walking')
        self.assertEqual(gen.population_appearance(dict(asset,name='NCAP_EPTc'),'walking',row,0,0,cfg,{})[0],'standing')

    def test_lookup_applies_input_only_and_unsupported_span_stays_strict(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);report=root/'report.json';atomic_json(report,{})
            profile=dict(asset='Woman2',data_id=5,center=dict(x=0.,y=0.),axis=[1.,0.],maximum_path_offset_m=1.,
                knots=[dict(s=-10.,l=0.,z=28.),dict(s=10.,l=0.,z=28.2)])
            lookup=root/'lookup.json';atomic_json(lookup,dict(policy=ground.POLICY,status='LIVE_VALIDATED',sources=[],
                profiles={'cw/Woman2':profile},live_evidence=dict(path=str(report),sha256=sha256(report))))
            cfg=dict(capture=dict(intersection_ground_lookup=dict(path=str(lookup),sha256=sha256(lookup))))
            path=dict(points=[dict(x=-8.,y=0.,z=99.),dict(x=3.,y=0.,z=99.)],target_position=dict(x=0.,y=0.,z=99.))
            before=copy.deepcopy(path);asset=dict(name='Woman2',data_id=5)
            result,decision=placement.apply_path(path,asset,'cw',cfg,{})
            self.assertEqual(path,before);self.assertEqual(decision,'editor_path_initialization_v116')
            self.assertAlmostEqual(result['target_position']['z'],28.1)
            self.assertEqual(result['ground_path_evidence']['policy'],ground.POLICY)
            path['points'][0]['x']=-11.
            result,decision=placement.apply_path(path,asset,'cw',cfg,{})
            self.assertIs(result,path);self.assertNotIn('ground_path_evidence',result)
            self.assertEqual(decision,'outside_measured_path_keep_strict_preparation')
            cfg['capture']['intersection_ground_lookup']['require_covered_moving_paths']=True
            with self.assertRaisesRegex(ValueError,'regenerate same quota'):
                placement.apply_path(path,asset,'cw',cfg,{})
            with self.assertRaisesRegex(ValueError,'lacks live manual profile'):
                placement.apply_path(path,dict(name='Unknown',data_id=99),'cw',cfg,{})
            with self.assertRaisesRegex(ValueError,'lacks covered intersection'):
                placement.apply_path(path,asset,None,cfg,{})

    def test_mixed_manual_and_unprepared_movers_cannot_skip_calibration(self):
        good=dict(motion='walking',motion_path=dict(ground_path_evidence=dict(policy=ground.POLICY)))
        self.assertTrue(placement.ready(dict(runtime_actors=[good,dict(motion='standing')])))
        self.assertFalse(placement.ready(dict(runtime_actors=[good,dict(motion='walking')])))
        self.assertFalse(placement.ready(dict(runtime_actors=[])))


if __name__=='__main__':unittest.main()
