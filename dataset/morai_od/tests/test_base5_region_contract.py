import copy
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
import base5_population_budget as population
import base5_region_contract as region


class RegionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        cls.config={'regional_policy':{'policy':region.POLICY}}

    def test_quota_total_and_mission_absorption(self):
        self.assertEqual(sum(sum(p) for p in region.QUOTAS.values()),5000)
        self.assertEqual(region.QUOTAS['roundabout'],(0,350))
        rows=[dict(spec=dict(family=f,mission=flag)) for f,p in region.QUOTAS.items()
              for flag,n in zip((False,True),p) for _ in range(n)]
        region.validate_quotas(rows)
        rows.pop()
        with self.assertRaises(ValueError):region.validate_quotas(rows)

    def test_all_vehicle_modes_never_sample_nonvehicle(self):
        pools=self.ctx['pools'];p=pools['vehicle'][0]
        for mode in ('generic_single','generic_regular','generic_dense','mission_situation'):
            for seed in range(20):
                r=dict(population_mode=mode,vehicle_only_population=True,actors=3,
                       population_budget_bounds=[16,22] if mode=='generic_dense' else [10,14],population_max_actors=8)
                result=population.choose(p,'vehicle',pools,lambda a:gen.geometry(a,self.ctx),r,random.Random(seed))
                self.assertTrue(all(a in pools['vehicle'] for a in result))

    def test_vehicle_only_rejects_wrong_primary(self):
        with self.assertRaises(ValueError):
            population.choose(self.ctx['pools']['person'][0],'person',self.ctx['pools'],lambda a:[1,1,1],
                              {'vehicle_only_population':True},random.Random(1))

    def test_obstacle_compensation_preserves_primary_and_budget(self):
        pools=self.ctx['pools'];primary=pools['person'][0]
        for seed in range(30):
            r=dict(population_mode='generic_regular',actors=1,population_max_actors=8,
                population_budget_bounds=[10,14],required_secondary_pool='obstacle')
            result=population.choose(primary,'person',pools,lambda a:gen.geometry(a,self.ctx),r,random.Random(seed))
            self.assertEqual(result[0],primary)
            self.assertIn(result[1],pools['obstacle'])
            self.assertTrue(10<=sum(population.cost(gen.geometry(a,self.ctx)[0]) for a in result)<=14)

    def test_actual_position_not_family_label_controls_nonvehicle_restriction(self):
        p=self.ctx['links']['A2256W000410']['points'][20]
        actor=dict(population_pool='person',geometric_center_estimate=p[:2])
        meta=dict(spec=dict(family='obstacle',mission=True),ego={'x':0,'y':0},runtime_actors=[actor])
        with self.assertRaisesRegex(ValueError,'Nonvehicle actor'):region.validate(gen,self.ctx,meta,self.config)
        actor['population_pool']='vehicle'
        self.assertIsNotNone(region.validate(gen,self.ctx,meta,self.config))

    def test_shadow_exception_is_independent_and_legacy_is_noop(self):
        from base5_shadow_observation import bounds
        lo,hi,_=bounds(gen,self.ctx)
        p=gen.geo.continuous_route_pose_at_arc(self.ctx['route'],self.ctx['arc'],(lo+hi)/2)
        meta=dict(spec=dict(family='shadow',mission=False,shadow_phase='interior'),ego=p,
                  runtime_actors=[dict(population_pool='animal',geometric_center_estimate=[p['x'],p['y']])])
        self.assertEqual(region.validate(gen,self.ctx,meta,self.config)['actor_regions'],['shadow'])
        meta['spec']=dict(family='pedestrian_related',mission=False)
        with self.assertRaisesRegex(ValueError,'Generic Ego'):region.validate(gen,self.ctx,meta,self.config)
        self.assertIsNone(region.validate(gen,self.ctx,meta,{}))

    def test_reassignment_clears_stale_roundabout_state(self):
        config={'mission_roundabout_observation':dict(policy=gen.roundabout_mission.POLICY,
            phase_counts=dict(approach=0,entry=0,circulation=1,exit=0),approach_states={},
            actor_count_weights={p:{'2':1} for p in ('approach','entry','circulation','exit')},
            density_weights={'normal':1}), 'seed':1}
        row=dict(family='roundabout',mission=True,roundabout_observation_state='waiting')
        gen.roundabout_mission.assign(gen,[row],config)
        self.assertNotIn('roundabout_observation_state',row)

    def test_foreground_proxy_rejects_complete_not_partial_or_background(self):
        primary=gen.rectangle([40,0],[4,2,2],0)
        near=gen.rectangle([15,0],[4,2,2],0)
        partial=gen.rectangle([15,1.4],[4,2,2],0)
        far=gen.rectangle([70,0],[4,10,2],0)
        self.assertGreaterEqual(region.foreground_angular_coverage((0,0),primary,near),.95)
        self.assertLess(region.foreground_angular_coverage((0,0),primary,partial),.95)
        self.assertEqual(region.foreground_angular_coverage((0,0),primary,far),0)

    def test_foreground_proxy_handles_rear_angle_wrap(self):
        primary=gen.rectangle([-40,0],[4,2,2],0)
        near=gen.rectangle([-15,0],[4,2,2],0)
        self.assertGreaterEqual(region.foreground_angular_coverage((0,0),primary,near),.95)


if __name__=='__main__':unittest.main()
