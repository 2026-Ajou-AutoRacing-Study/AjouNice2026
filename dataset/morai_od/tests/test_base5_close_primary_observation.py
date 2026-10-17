import copy
import json
import math
from pathlib import Path
import random
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_avoidance_geometry as avoidance
import base5_close_primary_observation as close
import base5_unit_geometry_contract as unit
import generate_base5_independent as gen


def fixture():
    reference = dict(path='/explicit/unit.json', sha256='a'*64)
    config = {unit.KEY: reference, close.KEY: close.configuration(reference)}
    asset = dict(name='small', data_id=7, source_list='objectList')
    native = dict(assets=dict(small=dict(data_id=7, source_list='objectList',
        native_size_lwh_m=[.4, .4, .85])))
    row = dict(family='obstacle', mission=True, motion='static', role='right',
        avoidance_phase='alongside', avoidance_side='left', avoidance_gap_bounds_m=[.2,.3])
    meta = dict(spec=row, **{unit.KEY:reference}, runtime_actors=[dict(asset, motion='static',
        geometry_reference_known=True)])
    stub = SimpleNamespace(geometry=lambda a,c:([.5,.6,1.3],[.01,-.02,0.],True),
        rectangle=gen.rectangle, geo=SimpleNamespace(continuous_route_pose_at_arc=lambda r,a,t:
            dict(x=t, y=20., z=1., yaw=30., route_arc_m=t)))
    base = dict(x=10., y=20., z=1., yaw=30., route_arc_m=100.)
    return reference, config, asset, native, meta, stub, base


class ClosePrimaryTests(unittest.TestCase):
    def setUp(self):
        self.ref,self.config,self.asset,self.native,self.meta,self.gen,self.base=fixture()
        self.loader=patch.object(unit,'load',return_value=self.native)
        self.loader.start();self.addCleanup(self.loader.stop)

    def observe(self,row,seed=12345,config=None):
        rng=random.Random(seed)
        result=avoidance.observation(self.gen,{},[],[0.,1000.],self.base,self.asset,row,rng,config=config)
        return result,rng

    def test_untagged_legacy_phase_coordinates_and_rng_are_exact(self):
        # Numeric fixtures were evaluated from the unmodified HEAD implementation.
        expected=dict(approach=(-52.463716429935374,0.),entry=(-9.335538724239196,.848302330309604),
            alongside=(1.4838105214550874,1.5689520999806745),clearing=(7.572120652087298,1.1335372865907072))
        for phase,(lon,lat) in expected.items():
            row=dict(self.meta['spec'],avoidance_phase=phase)
            (ego,target,proof),rng=self.observe(row)
            self.assertEqual(proof['longitudinal_center_offset_m'],lon)
            self.assertEqual(proof['lateral_center_offset_m'],lat)
            self.assertEqual(rng.random(),.8551383418471491)
            # A configured deck's unselected rows remain bit-for-bit legacy,
            # including unused policy validation and all RNG state.
            result2,rng2=self.observe(row,config=self.config)
            self.assertEqual((ego,target,proof),result2)
            self.assertEqual(rng2.random(),.8551383418471491)
        row=self.meta['spec'];(ego,_,_),_=self.observe(row)
        self.assertEqual([ego['x'],ego['y']],[9.342490115968173,21.445078510020195])

    def test_factory_is_pure_and_rows_pin_native_identity(self):
        original=copy.deepcopy(self.meta)
        row=close.tagged_row(self.meta,self.config,[1.,1.25])
        self.assertEqual(self.meta,original)
        self.assertNotIn(close.KEY,original['spec'])
        self.assertEqual(close.validate(row,self.config,self.asset,True),row[close.KEY])
        self.assertEqual({k:v for k,v in row.items() if k!=close.KEY},original['spec'])

    def test_tagged_optin_preserves_side_and_exact_unchanged_obb_gap(self):
        for side,role in (('left','right'),('right','left')):
            self.meta['spec'].update(avoidance_side=side,role=role)
            row=close.tagged_row(self.meta,self.config)
            for seed in range(40):
                (ego,target,proof),_=self.observe(row,seed,self.config)
                size,off,_=self.gen.geometry(self.asset,{})
                a=math.radians(target['yaw']);c,s=math.cos(a),math.sin(a)
                center=[target['x']+c*off[0]-s*off[1],target['y']+s*off[0]+c*off[1]]
                yaw=math.radians(ego['yaw']);c,s=math.cos(yaw),math.sin(yaw)
                dx,dy=center[0]-ego['x'],center[1]-ego['y']
                x,y=c*dx+s*dy,-s*dx+c*dy
                self.assertTrue(1.<=x<=1.5)
                self.assertLess(abs(x),abs(y))
                self.assertEqual(y>0,role=='left')
                ep=[ego['x']+1.35*c,ego['y']+1.35*s]
                gap=gen.rectangle(center,size,target['yaw']).distance(gen.rectangle(ep,[4.8,2.],ego['yaw']))
                self.assertAlmostEqual(gap,.30,places=7)
                self.assertFalse(proof[close.KEY]['placement_envelope_changed'])
                self.assertFalse(proof[close.KEY]['native_visibility_certified'])

    def test_contract_mismatch_is_fatal_without_legacy_fallback(self):
        row=close.tagged_row(self.meta,self.config)
        for mutation in ('no_config','wrong_policy','wrong_unit','wrong_name','wrong_id','unknown_geometry','bad_band'):
            r,c,a,k=copy.deepcopy(row),copy.deepcopy(self.config),copy.deepcopy(self.asset),True
            if mutation=='no_config':c=None
            elif mutation=='wrong_policy':c[close.KEY]['maximum_native_height_m']=2.
            elif mutation=='wrong_unit':c[unit.KEY]['sha256']='b'*64
            elif mutation=='wrong_name':a['name']='other'
            elif mutation=='wrong_id':a['data_id']=8
            elif mutation=='unknown_geometry':k=False
            else:r[close.KEY]['primary_ego_x_bounds_m']=[.5,2.]
            with self.subTest(mutation=mutation),self.assertRaises(close.ClosePrimaryContractError):
                close.validate(r,c,a,k)

    def test_narrow_proxy_crossing_side_front_is_bounded_proposal_rejection(self):
        self.native['assets']['small']['native_size_lwh_m']=[.1,.1,.5]
        self.gen.geometry=lambda a,c:([.1,.1,.5],[0.,0.,0.],True)
        row=close.tagged_row(self.meta,self.config)
        with self.assertRaisesRegex(ValueError,'crossed frozen side/front boundary'):
            self.observe(row,3,self.config)
        # No role rewrite or silently reduced X band.
        self.assertEqual(row['role'],'right')
        self.assertEqual(row[close.KEY]['primary_ego_x_bounds_m'],[1.,1.5])

    def test_no_arbitrary_population_or_clearance_changes(self):
        for mutation in ('high','large','nonphysical','moving','wider_gap','other_phase','scaled','compound','wrong_side'):
            m,n=copy.deepcopy(self.meta),copy.deepcopy(self.native)
            a=m['runtime_actors'][0];r=m['spec']
            if mutation=='high':n['assets']['small']['native_size_lwh_m'][2]=1.01
            elif mutation=='large':n['assets']['small']['native_size_lwh_m'][0]=1.51
            elif mutation=='nonphysical':a['source_list']='pedestrianList'
            elif mutation=='moving':r['motion']='walking'
            elif mutation=='wider_gap':r['avoidance_gap_bounds_m']=[.3,.5]
            elif mutation=='other_phase':r['avoidance_phase']='entry'
            elif mutation=='scaled':a['asset_scale']=dict(scale_xyz=[1.25]*3)
            elif mutation=='compound':a['component_count']=2
            else:r['role']='left'
            self.assertFalse(close.eligible(m,self.config[close.KEY],n),mutation)

    def test_prepared_entry_revalidates_policy_proof_side_band_gap_and_unit(self):
        row=close.tagged_row(self.meta,self.config)
        self.meta['spec']=row
        self.meta['runtime_actors'][0].update(longitudinal_m=1.2,lateral_m=-1.6,
            planned_surface_clearance_m=.30,actual_ego_role='right')
        self.meta['avoidance_observation']={close.KEY:dict(row[close.KEY],
            placement_envelope_changed=False,native_gt_modified=False)}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metadata.json';entry=dict(metadata=str(path))
            def check(value,config=self.config):
                path.write_text(json.dumps(value))
                return close.validate_entry(entry,dict(config=config))
            self.assertTrue(check(self.meta)['enabled'])
            for mutation in ('row_deleted','proof_deleted','native_ref','x','side','gap','proof_gt','config'):
                meta=copy.deepcopy(self.meta);config=copy.deepcopy(self.config)
                if mutation=='row_deleted':meta['spec'].pop(close.KEY)
                elif mutation=='proof_deleted':meta.pop('avoidance_observation')
                elif mutation=='native_ref':meta[unit.KEY]['sha256']='b'*64
                elif mutation=='x':meta['runtime_actors'][0]['longitudinal_m']=.2
                elif mutation=='side':meta['runtime_actors'][0]['lateral_m']=1.6
                elif mutation=='gap':meta['runtime_actors'][0]['planned_surface_clearance_m']=.2
                elif mutation=='proof_gt':meta['avoidance_observation'][close.KEY]['native_gt_modified']=True
                else:config.pop(close.KEY)
                with self.subTest(mutation=mutation),self.assertRaises(close.ClosePrimaryContractError):
                    check(meta,config)

    def test_untagged_prepared_legacy_entry_does_not_load_new_policy(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metadata.json';path.write_text(json.dumps(self.meta))
            self.assertEqual(close.validate_entry(dict(metadata=str(path)),dict(config={})),dict(enabled=False))
            unit.load.assert_not_called()

    def test_camera_proposal_cannot_certify_native_and_requires_whole_positive_proxy(self):
        rotation=np.asarray([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]])
        transform=np.eye(4);transform[:3,:3]=rotation
        camera=dict(ego_to_sensor=transform.tolist(),resolution_px=[640,480],
            intrinsic=[[100.,0.,320.],[0.,100.,240.],[0.,0.,1.]])
        m=dict(ego=dict(yaw=0.,z=0.),runtime_actors=[dict(estimated_size=[.5,.5,.5],
            map_ground_initializer=dict(usable=True,height_m=0.),placement_yaw_deg=0.,
            longitudinal_m=2.,lateral_m=0.)])
        snapshot=copy.deepcopy(m)
        result=close.projection_proposal(m,dict(cameras=dict(front=camera)))
        self.assertTrue(result['eligible']);self.assertFalse(result['native_visibility_certified'])
        self.assertFalse(result['native_annotation_synthesized']);self.assertEqual(m,snapshot)
        m['runtime_actors'][0]['longitudinal_m']=.1
        self.assertFalse(close.projection_proposal(m,dict(cameras=dict(front=camera)))['eligible'])
        m['runtime_actors'][0]['map_ground_initializer']['usable']=False
        self.assertFalse(close.projection_proposal(m,dict(cameras=dict(front=camera)))['eligible'])


if __name__=='__main__':unittest.main()
