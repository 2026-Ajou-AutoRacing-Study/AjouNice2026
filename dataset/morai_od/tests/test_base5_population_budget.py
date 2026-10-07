import random
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_population_budget as population
import generate_base5_independent as gen


class PopulationTests(unittest.TestCase):
    def test_cost_shape(self):
        self.assertEqual(population.cost([.4,.4]),2)
        self.assertEqual(population.cost([4.,.2]),6)
        self.assertEqual(population.cost([12.,2.5]),8)
        with self.assertRaises(ValueError):population.cost([float('nan'),1])

    def test_background_stays_empty(self):
        self.assertEqual(population.choose({},'vehicle',{},None,dict(actors=0),None),[])

    def test_visible_parent_not_component_count(self):
        actors=[dict(unique_id=1),dict(unique_id=2)]
        components=[dict(parent_uid=1,center_lidar_m=[2,0,0],positive_lidar1_camera0=True,lidar_points=0)]*3
        components.append(dict(parent_uid=2,center_lidar_m=[120,0,0],positive_lidar1_camera0=True,lidar_points=3))
        summary=population.visibility_summary(actors,components)
        self.assertEqual(summary['visible_positive_actors'],1)
        self.assertEqual(summary['positive_components'],3)
        self.assertEqual(summary['no_positive_uids'],[2])
        self.assertEqual(population.visibility_review(dict(population_budget=dict(required_visible_actors=2)),summary),
            ['population_visible_actor_deficit'])
        self.assertEqual(population.visibility_review({},summary),[])
        self.assertEqual(population.visibility_review(dict(calibration_only=True,
            population_budget=dict(required_visible_actors=2)),summary),[])

    def test_budget_and_primary(self):
        small={'name':'small','size':[.5,.5]}
        large={'name':'large','size':[5.,2.]}
        pools={k:[small,large] for k in ('vehicle','obstacle','person')}
        for seed in range(50):
            row=dict(population_mode='generic_dense',population_budget_bounds=[16,22],population_max_actors=8)
            actors=population.choose(large,'vehicle',pools,lambda a:(a['size'],),row,random.Random(seed))
            self.assertIs(actors[0],large)
            self.assertTrue(2<=len(actors)<=8)
            self.assertTrue(16<=sum(population.cost(a['size']) for a in actors)<=22)

    def test_small_objects_can_reach_eight(self):
        a={'size':[.5,.5]};pools={k:[a] for k in ('vehicle','obstacle','person')}
        row=dict(population_mode='generic_dense',population_budget_bounds=[16,22],population_max_actors=8)
        result=population.choose(a,'obstacle',pools,lambda a:(a['size'],),row,random.Random(1))
        self.assertEqual(len(result),8)

    def test_impossible_budget_does_not_overfill(self):
        a={'size':[12,3]};pools={k:[a] for k in ('vehicle','obstacle','person')}
        row=dict(population_mode='generic_regular',population_budget_bounds=[10,14],population_max_actors=8)
        with self.assertRaises(ValueError):
            population.choose(a,'vehicle',pools,lambda a:(a['size'],),row,random.Random(1))

    def test_mission_no_foreign_pool(self):
        a={'name':'a'}; b={'name':'b'}
        result=population.choose(a,'vehicle',{'vehicle':[a],'person':[b]},None,
            dict(mission=True,actors=3,population_mode='mission_situation'),random.Random(2))
        self.assertEqual(result,[a,a,a])

    def test_exact_population_deck(self):
        rows=gen.specs(gen.geo.load_json(gen.CONFIG))
        generic=[r for r in rows if not r['mission'] and r['family']!='background']
        from collections import Counter
        self.assertEqual(Counter(r['population_mode'] for r in generic),
            dict(generic_single=900,generic_regular=1500,generic_dense=600))
        self.assertEqual(sum(r['family']=='background' for r in rows),250)
        self.assertEqual(sum(r['mission'] for r in rows),1750)
        self.assertEqual(rows,gen.specs(gen.geo.load_json(gen.CONFIG)))
        missions=[r for r in rows if r['mission']]
        self.assertTrue(all(r['population_mode']=='mission_situation' and 'population_budget_bounds' not in r for r in missions))
        self.assertEqual(Counter(r['actors'] for r in missions if r['family']=='obstacle'),{1:450,2:120,3:30})


if __name__=='__main__':unittest.main()
