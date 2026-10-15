import copy
from collections import Counter
from pathlib import Path
import sys
import unittest
import tempfile

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
import base5_roundabout_mission as mission


class RoundaboutMissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=gen.geo.load_json(gen.CONFIG)
        cls.config=copy.deepcopy(cls.original)
        cls.config['mission_roundabout_observation']=gen.geo.load_json(gen.geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_roundabout_mission_v118.json')
        cls.ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        cls.rows=[r for r in gen.specs(cls.config) if r.get('roundabout_phase')]

    def build(self,phase='circulation'):
        row=next(r for r in self.rows if r['roundabout_phase']==phase)
        for retry in range(512):
            try:return gen.build(row,self.config,self.ctx,retry)
            except ValueError:continue
        self.fail('Unfillable roundabout test cell')

    def test_exact_phase_and_density_quota(self):
        self.assertEqual(len(self.rows),250)
        self.assertEqual(Counter(r['roundabout_phase'] for r in self.rows),self.config['mission_roundabout_observation']['phase_counts'])
        self.assertEqual(Counter(r.get('roundabout_observation_state') for r in self.rows if r['roundabout_phase']=='approach'),dict(approaching=20,waiting=30))
        self.assertTrue(all(r['actors']>=2 for r in self.rows if r['roundabout_phase'] in ('entry','circulation')))
        for phase in self.config['mission_roundabout_observation']['phase_counts']:
            expected=gen.exact_deck(list(self.config['mission_roundabout_observation']['density_weights'].items()),
                                    sum(r['roundabout_phase']==phase for r in self.rows),__import__('random').Random(1))
            self.assertEqual(Counter(r['roundabout_density'] for r in self.rows if r['roundabout_phase']==phase),Counter(expected))

    def test_opt_in_changes_only_250_mission_rows(self):
        old=gen.specs(self.original);new=gen.specs(self.config)
        self.assertEqual(sum(a!=b for a,b in zip(old,new)),250)
        for a,b in zip(old,new):
            if not (a['family']=='roundabout' and a['mission']):self.assertEqual(a,b)
        self.assertEqual(new,gen.specs(self.config))

    def test_sample_exact_start_and_despawn(self):
        path=mission.corridor(gen,self.ctx)
        for field,point in (('start',path['route'][0]),('end',path['route'][-1])):
            ref=path[field]
            self.assertEqual(point['point_idx'],ref['pointIdx'])
            self.assertEqual([point[k] for k in ('x','y','z')],self.ctx['links'][ref['linkIdx']]['points'][ref['pointIdx']])
        self.assertNotIn('A2256W000848',path['path'])

    def test_every_phase_has_valid_geometry(self):
        for phase in self.config['mission_roundabout_observation']['phase_counts']:
            scene,meta=self.build(phase)
            self.assertTrue(mission.validate(gen,self.ctx,meta,scene,self.config))
            self.assertFalse(meta['roundabout_observation']['collider_verified'])
            self.assertFalse(scene['spawnPointList'])

    def test_circulation_ego_is_between_front_and_rear(self):
        scene,meta=self.build()
        signs=[a['longitudinal_m'] for a in meta['runtime_actors'][:2]]
        self.assertLess(signs[0]*signs[1],0)
        self.assertTrue(all(a['motion']=='static' for a in meta['runtime_actors']))

    def test_reject_actual_pose_shift_not_only_metadata(self):
        scene,meta=self.build();scene['vehicleList'][0]['initPosition']['pos']['x']+=.1
        with self.assertRaisesRegex(ValueError,'Serialized NPC'):mission.validate(gen,self.ctx,meta,scene,self.config)

    def test_reject_outside_official_corridor(self):
        scene,meta=self.build();meta['runtime_actors'][0]['sample_roundabout_placement']['reference_arc_m']=1000.
        with self.assertRaisesRegex(ValueError,'despawn'):mission.validate(gen,self.ctx,meta,scene,self.config)

    def test_reject_unknown_policy_and_wrong_quota(self):
        config=copy.deepcopy(self.config);config['mission_roundabout_observation']['policy']='unknown'
        with self.assertRaises(ValueError):gen.specs(config)
        config=copy.deepcopy(self.config);config['mission_roundabout_observation']['phase_counts']['entry']+=1
        with self.assertRaises(ValueError):gen.specs(config)

    def test_visible_pair_and_front_back_required_only_on_new_policy(self):
        meta=dict(runtime_actors=[dict(unique_id=1),dict(unique_id=2)],
                  roundabout_observation=dict(policy=mission.POLICY,phase='circulation',density='tight'))
        visible=[dict(parent_uid=1,positive_lidar1_camera0=True,center_ego_m=[5.,0.,0.]),
                 dict(parent_uid=2,positive_lidar1_camera0=True,center_ego_m=[-5.,0.,0.])]
        gaps=[dict(parent_uid=1,annotation_surface_gap_m=1.),dict(parent_uid=2,annotation_surface_gap_m=1.)]
        reasons,proof=mission.visibility_review(meta,visible,gaps)
        self.assertFalse(reasons);self.assertTrue(proof['visible_front_back'])
        reasons,_=mission.visibility_review(meta,visible[:1])
        self.assertIn('roundabout_visible_pair_deficit',reasons)
        visible[1]['center_ego_m'][0]=8.
        reasons,_=mission.visibility_review(meta,visible)
        self.assertIn('roundabout_visible_front_back_deficit',reasons)
        meta['roundabout_observation']['phase']='entry'
        self.assertFalse(mission.visibility_review(meta,visible)[0])
        meta.pop('roundabout_observation')
        self.assertEqual(mission.visibility_review(meta,[]),([],None))

    def test_actual_gap_cannot_satisfy_tight_quota_after_npc_moves(self):
        meta=dict(runtime_actors=[dict(unique_id=1),dict(unique_id=2)],
                  roundabout_observation=dict(policy=mission.POLICY,phase='circulation',density='tight'))
        components=[dict(parent_uid=1,positive_lidar1_camera0=True,center_ego_m=[5.,0.,0.]),
                    dict(parent_uid=2,positive_lidar1_camera0=True,center_ego_m=[-10.,0.,0.])]
        gaps=[dict(parent_uid=2,annotation_surface_gap_m=7.),dict(parent_uid=1,annotation_surface_gap_m=1.)]
        reasons,_=mission.visibility_review(meta,components,gaps)
        self.assertIn('roundabout_actual_density_gap_mismatch',reasons)

    def test_reject_approach_subquota_mismatch(self):
        config=copy.deepcopy(self.config)
        config['mission_roundabout_observation']['approach_states']['waiting']+=1
        with self.assertRaisesRegex(ValueError,'approach state quota'):gen.specs(config)

    def test_replacement_keeps_route_phase_density_primary_and_split(self):
        from base5_capture_safety import atomic_json,sha256
        from run_base5_quality_loop import replacement
        scene,meta=self.build()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);sp=root/'scene.json';mp=root/'metadata.json';manifest=root/'manifest.json'
            atomic_json(sp,scene);atomic_json(mp,meta)
            atomic_json(manifest,dict(config=self.config,scenes=[dict(id='roundabout-test',
                scenario=str(sp),metadata=str(mp),scenario_sha256=sha256(sp))]))
            path=replacement(manifest,root/'replacement')
            entry=gen.geo.load_json(path)['scenes'][0];new=gen.geo.load_json(Path(entry['metadata']))
            for key in ('roundabout_phase','roundabout_density','roundabout_primary_sign',
                        'roundabout_phase_ordinal','roundabout_phase_total'):
                self.assertEqual(new['spec'][key],meta['spec'][key])
            self.assertEqual(new['split'],meta['split'])
            self.assertEqual(new['runtime_actors'][0]['data_id'],meta['runtime_actors'][0]['data_id'])
            self.assertTrue(mission.validate(gen,self.ctx,new,gen.geo.load_json(Path(entry['scenario'])),self.config))


if __name__=='__main__':unittest.main()
