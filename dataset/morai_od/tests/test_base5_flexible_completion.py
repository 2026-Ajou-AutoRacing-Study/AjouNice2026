import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_base5_flexible_completion as flex


def meta(primary=1, motions=('static',), family='obstacle', mission=True, split='train'):
    return dict(spec=dict(family=family, mission=mission, distance_bounds=[2, 5], role='left'),
        split=split, runtime_actors=[dict(data_id=primary if i==0 else 3, name=str(primary), motion=m)
                                   for i, m in enumerate(motions)])


class FlexibleCompletionTests(unittest.TestCase):
    pools = {1: 'obstacle', 2: 'obstacle', 3: 'person', 4: 'person', 5: 'animal'}

    def test_different_asset_distance_role_and_count_allowed(self):
        source=meta();actual=meta(2, ('static', 'standing'))
        actual['spec'].update(distance_bounds=[15, 30],role='front')
        self.assertTrue(flex.compatible(source, actual, self.pools))
        delta=flex.candidate_delta(source, actual)
        self.assertEqual(delta['replacement_actor_count'],2)
        self.assertEqual(delta['replacement_distance_bounds'],[15,30])

    def test_family_and_mission_not_interchangeable(self):
        self.assertFalse(flex.compatible(meta(), meta(family='highway'), self.pools))
        self.assertFalse(flex.compatible(meta(), meta(mission=False), self.pools))

    def test_split_and_semantic_pool_preserved(self):
        self.assertFalse(flex.compatible(meta(), meta(split='val'), self.pools))
        self.assertFalse(flex.compatible(meta(), meta(3), self.pools))

    def test_dynamic_primary_can_replace_static_primary_with_moving_secondary(self):
        source=meta(3, ('standing','walking'), family='pedestrian_related')
        self.assertTrue(flex.compatible(source, meta(4, ('walking',), family='pedestrian_related'), self.pools))

    def test_moving_to_standing_and_wrong_gait_rejected(self):
        source=meta(3, ('walking',), family='pedestrian_related')
        self.assertFalse(flex.compatible(source, meta(4, ('standing',), family='pedestrian_related'), self.pools))
        self.assertFalse(flex.compatible(source, meta(4, ('running',), family='pedestrian_related'), self.pools))

    def test_static_scene_not_changed_to_dynamic(self):
        self.assertFalse(flex.compatible(meta(3, ('standing',)), meta(4, ('walking',)), self.pools))

    def test_added_secondary_movers_allowed_but_lost_modes_rejected(self):
        source=meta(3, ('walking','running'))
        self.assertTrue(flex.compatible(source,meta(4,('walking','walking','running')),self.pools))
        self.assertFalse(flex.compatible(source,meta(4,('walking','walking')),self.pools))

    def test_empty_background_not_used_to_fill_positive(self):
        actual=meta();actual['runtime_actors']=[]
        self.assertFalse(flex.compatible(meta(),actual,self.pools))

    def test_original_rows_preserved_and_mapping_explicit(self):
        baseline=[dict(source_cell_id='z',id='z',capture_root='old')]
        state=dict(baseline_selected=baseline,attempts=[dict(id='a',manifest='new',delta={'changed':True},
            result=dict(status='accepted_candidate',capture_root='fresh',id='a_fx01'))])
        before=copy.deepcopy(state);rows,used=flex.selected_rows(state)
        self.assertEqual(state,before)
        self.assertEqual(rows[1],baseline[0])
        self.assertEqual(rows[0]['source_cell_id'],'a')
        self.assertEqual(rows[0]['id'],'a_fx01')
        self.assertEqual(used,{'a','z'})
        self.assertEqual(rows[0]['flexible_completion']['delta'],{'changed':True})

    def test_duplicate_slot_rejected_and_review_not_selected(self):
        state=dict(baseline_selected=[dict(source_cell_id='a')],attempts=[dict(id='a',
            result=dict(status='accepted_candidate'))])
        with self.assertRaisesRegex(ValueError,'twice'):flex.selected_rows(state)
        state['attempts'][0]['result']['status']='review'
        self.assertEqual(len(flex.selected_rows(state)[0]),1)

    def test_post_quality_actual_motion_checked(self):
        source=meta(3, ('walking',))
        row=dict(status='accepted_candidate',capture_root='/unused')
        with patch.object(flex.ac,'verify',return_value=row),patch.object(flex.ac.fa,'read',return_value=meta(4,('standing',))):
            result=flex.verify(dict(entry={}),{},source,self.pools)
        self.assertEqual(result['status'],'review')
        self.assertEqual(result['observed_selected'],row)

    def test_native_review_not_rewritten(self):
        row=dict(status='review',reasons=['missing_runtime_actor'])
        with patch.object(flex.ac,'verify',return_value=row):
            self.assertIs(flex.verify(dict(entry={}),{},meta(),self.pools),row)


if __name__ == '__main__':
    unittest.main()
