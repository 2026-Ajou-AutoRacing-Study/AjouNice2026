import random
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_obstacle_rich_floor as rich
import base5_obstacle_support_policy as policy
import base5_population_budget as population


class RichFloorTests(unittest.TestCase):
    def row(self):
        return dict(family='obstacle',mission=False,population_mode='generic_regular',
                    actors=3,population_max_actors=8,population_budget_bounds=[10,14],seed=1)

    def actors(self):
        return [dict(data_id=i+1,source_list='objectList',motion='static',
                     population_pool='obstacle_box',planned_center_distance_m=d)
                for i,d in enumerate((5,12,25,35))]

    def test_only_far_is_moved_and_mid_is_preserved(self):
        row=rich.attach(self.row(),self.actors())
        p=row['secondary_obstacle_support']
        self.assertEqual(p['ordinals'],[3]);self.assertEqual(p['protected_mid_ordinals'],[2])
        self.assertEqual(p['protected_near_ordinals'],[1])
        rich.verify_mid(row,2,25)
        with self.assertRaises(ValueError):rich.verify_mid(row,2,15)

    def test_append_keeps_primary_all_previous_actors_and_cap(self):
        r=self.row()
        r['seed']=next(i for i in range(50) if rich.extra_selected(dict(r,seed=i),4))
        extra=dict(data_id=9,source_list='objectList',semantic_class='obstacle')
        new=rich.attach(r,self.actors(),extra,2)
        self.assertEqual(new['frozen_population_data_ids'],[1,2,3,4,9])
        self.assertEqual(new['population_budget_bounds'],[12,16])
        self.assertEqual(new['population_rng_reference']['population_budget_bounds'],[10,14])
        self.assertFalse(rich.extra_selected(r,8))
        proof=policy.sample(new,4,extra,[.5,.5,.5],'front',0)
        self.assertTrue(proof['added_actor']);self.assertEqual(proof['band'],'near')

    def test_budget_reference_does_not_shift_existing_rng(self):
        a=dict(data_id=1,size=[5,2]);b=dict(data_id=2,size=[.5,.5])
        pools={k:[a,b] for k in ('vehicle','obstacle','person')}
        row=self.row();rng=random.Random(3)
        original=population.choose(a,'obstacle',pools,lambda a:(a['size'],),row,rng)
        pop_state=rng.getstate()
        new=dict(row,frozen_population_data_ids=[x['data_id'] for x in original]+[2],
                 population_rng_reference=dict(population_budget_bounds=row['population_budget_bounds']),
                 population_budget_bounds=[12,16])
        other=random.Random(3)
        result=population.choose(a,'obstacle',pools,lambda a:(a['size'],),new,other)
        self.assertEqual(result[:-1],original);self.assertEqual(other.getstate(),pop_state)

    def test_mission_and_actual_movers_cannot_gain_an_actor(self):
        r=self.row();r['mission']=True
        self.assertEqual(rich.attach(r,self.actors()),r)
        r=self.row();a=self.actors();a[-1]['motion']='walking'
        self.assertEqual(rich.attach(r,a),r)

    def test_low_small_extras_have_shorter_fixed_distance_bands(self):
        r=self.row()
        r['seed']=next(i for i in range(50) if rich.extra_selected(dict(r,seed=i),4))
        extra=dict(data_id=9,source_list='objectList',semantic_class='obstacle')
        new=rich.attach(r,self.actors(),extra,2)
        for height,bounds in ((.2,[6.,10.]),(.5,[6.,14.]),(1.,[6.,18.])):
            for attempt in (0,1,17,512):
                proof=policy.sample(new,4,extra,[.5,.5,height],'front',attempt)
                self.assertEqual(proof['distance_bounds_m'],bounds)
                policy.verify_actual(proof,proof['target_center_distance_m'])

    def test_protected_near_mid_sample_only_after_original_attempt(self):
        new=rich.attach(self.row(),self.actors())
        new['secondary_obstacle_support']['original_generation_attempt']=3
        asset=dict(data_id=2,source_list='objectList',semantic_class='obstacle')
        for ordinal in (1,2):
            self.assertIsNone(policy.sample(new,ordinal,asset,[.5,.5,1.],'front',3))
            for attempt in range(4,40):
                proof=policy.sample(new,ordinal,asset,[.5,.5,1.],'front',attempt)
                self.assertTrue(proof['protected_resample'])
                self.assertEqual(proof['band'],'near' if ordinal==1 else 'mid')
                policy.verify_actual(proof,proof['target_center_distance_m'])

    def test_input_specs_are_not_mutated(self):
        import copy
        original=self.row();snapshot=copy.deepcopy(original)
        rich.attach(original,self.actors())
        self.assertEqual(original,snapshot)


if __name__=='__main__':unittest.main()
