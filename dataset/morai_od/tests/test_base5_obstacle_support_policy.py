import copy
import random
import sys
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_obstacle_support_policy as policy
import base5_population_budget as population


class SupportPolicyTests(unittest.TestCase):
    def row(self,**kw):
        return dict(family='obstacle',mission=False,population_mode='generic_regular',
                    seed=123,**kw)

    def actors(self):
        return [dict(data_id=1,population_pool='obstacle_box',source_list='objectList',motion='static',planned_center_distance_m=5),
                dict(data_id=2,population_pool='obstacle_barrier',source_list='objectList',motion='static',planned_center_distance_m=25)]

    def test_off_means_no_change(self):
        self.assertIsNone(policy.sample(self.row(),1,{},[],None,0))

    def test_attach_is_pure_and_preserves_actor_ids(self):
        row=self.row();before=copy.deepcopy(row)
        new=policy.attach(row,self.actors(),'near70')
        self.assertEqual(row,before)
        self.assertEqual(new['frozen_population_data_ids'],[1,2])
        self.assertEqual(new['secondary_obstacle_support']['ordinals'],[1])

    def test_already_near_obstacle_is_not_redistributed(self):
        actors=self.actors();actors[1]['planned_center_distance_m']=8
        self.assertEqual(policy.attach(self.row(),actors,'near70'),self.row())

    def test_protected_near_survives_recovery(self):
        actors=self.actors();actors.append(dict(actors[1],data_id=3,planned_center_distance_m=8))
        row=policy.attach(self.row(),actors,'near70')
        self.assertEqual(row['secondary_obstacle_support']['protected_near_ordinals'],[2])
        policy.verify_protected_near(row,2,8)
        with self.assertRaises(ValueError):policy.verify_protected_near(row,2,35)

    def test_reserved_mission_moving_and_traffic_are_untouched(self):
        for change in ({'mission':True},{'family':'highway'},{'family':'roundabout'},
                       {'family':'background'},{'shadow_phase':'internal'}):
            row=self.row();row.update(change)
            self.assertEqual(policy.attach(row,self.actors(),'near70'),row)
        actors=self.actors();actors[1]['motion']='walking'
        self.assertNotIn('secondary_obstacle_support',policy.attach(self.row(),actors,'near70'))

    def test_retry_keeps_band_and_role(self):
        row=policy.attach(self.row(),self.actors(),'near70')
        asset=dict(source_list='objectList',semantic_class='obstacle',data_id=2)
        first=policy.sample(row,1,asset,[.5,.5,1],'rear',0)
        for n in range(30):
            proof=policy.sample(row,1,asset,[.5,.5,1],'rear',n)
            self.assertEqual(proof['band'],first['band']);self.assertEqual(proof['role'],'rear')
            policy.verify_actual(proof,proof['target_center_distance_m'])
        with self.assertRaises(ValueError):policy.verify_actual(first,100.)

    def test_primary_and_nonobstacle_injection_refused(self):
        row=policy.attach(self.row(),self.actors(),'near70')
        row['secondary_obstacle_support']['ordinals']=[0]
        with self.assertRaises(ValueError):policy.sample(row,0,{},[],None,0)
        row=policy.attach(self.row(),self.actors(),'near70')
        with self.assertRaises(ValueError):policy.sample(row,1,dict(source_list='vehicleList'),[],None,0)

    def test_freeze_reuses_assets_but_consumes_same_rng(self):
        a=dict(data_id=1,size=[.5,.5]);b=dict(data_id=2,size=[5.,2.])
        pools={k:[a,b] for k in ('vehicle','obstacle','person')}
        row=dict(population_mode='generic_regular',population_budget_bounds=[10,14],population_max_actors=8)
        r1=random.Random(4);r2=random.Random(4)
        original=population.choose(b,'vehicle',pools,lambda a:(a['size'],),row,r1)
        row['frozen_population_data_ids']=[x['data_id'] for x in original]
        frozen=population.choose(b,'vehicle',pools,lambda a:(a['size'],),row,r2)
        self.assertEqual(original,frozen);self.assertEqual(r1.getstate(),r2.getstate())
        row['frozen_population_data_ids'][0]=1
        with self.assertRaises(ValueError):population.choose(b,'vehicle',pools,lambda a:(a['size'],),row,random.Random(4))


if __name__=='__main__':unittest.main()
