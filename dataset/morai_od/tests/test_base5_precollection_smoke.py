import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_precollection_smoke as smoke


class SmokeSelectionTest(unittest.TestCase):
    def fixture(self):
        names = ('CargoBox', 'concrete-barrier', 'BabyStroller1', 'Ped_Pet1',
                 'Man1', 'Woman1', 'Dog1', 'Dog2')
        motions = ('static', 'static', 'static', 'standing', 'walking', 'running', 'walking', 'running')
        metadata = {}
        for i,(name,motion) in enumerate(zip(names,motions)):
            metadata[str(i)] = dict(spec=dict(family='obstacle', mission=False, motion=motion),
                runtime_actors=[dict(name=name, motion=motion, planned_center_distance_m=22 if i==1 else 10)])
        for i in (6,7):
            metadata[str(i)]['spec'].update(family='shadow', shadow_phase='interior')
        deck = dict(scenes=[dict(id=k) for k in metadata])
        registry = dict(entries=[dict(source_id=str(i), mode='exact_source') for i in (6,7)])
        return deck, metadata, registry

    def test_all_required_strata_are_selected_without_mutation(self):
        deck, meta, registry = self.fixture()
        before = copy.deepcopy((deck, meta, registry))
        result = smoke.select(deck, meta, registry)
        self.assertEqual(len(result), 8)
        self.assertEqual(len({e['id'] for _, e in result}), 8)
        self.assertEqual({label for label,_ in result}, {'low_obstacle_near','wide_obstacle_mid',
            'stroller_components','pet_components','intersection_walking','intersection_running',
            'shadow_reuse_Dog1','shadow_reuse_Dog2'})
        self.assertEqual((deck,meta,registry), before)

    def test_selection_is_source_order_independent(self):
        deck, meta, registry = self.fixture()
        expected = smoke.select(deck,meta,registry)
        deck['scenes'].reverse()
        self.assertEqual(smoke.select(deck,meta,registry), expected)

    def test_missing_stratum_is_not_silently_replaced(self):
        deck, meta, registry = self.fixture()
        meta['1']['runtime_actors'][0]['planned_center_distance_m'] = 10
        with self.assertRaisesRegex(ValueError, 'wide_obstacle_mid'):
            smoke.select(deck,meta,registry)

    def test_same_quota_replacement_cannot_claim_exact_reuse(self):
        deck,meta,registry = self.fixture()
        registry['entries'][0]['mode'] = 'same_quota_replacement'
        with self.assertRaisesRegex(ValueError, 'exact-source'):
            smoke.select(deck,meta,registry)

    def test_mission_is_not_substituted_for_generic_obstacle(self):
        deck,meta,registry = self.fixture()
        meta['0']['spec']['mission'] = True
        with self.assertRaisesRegex(ValueError, 'low_obstacle_near'):
            smoke.select(deck,meta,registry)


if __name__ == '__main__':
    unittest.main()
