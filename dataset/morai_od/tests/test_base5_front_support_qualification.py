import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_obstacle_front_support as front
import base5_obstacle_rich_floor as rich
import prepare_base5_front_support_qualification as qualification


def fixture():
    entries, metadata, items = [], {}, []
    for asset in range(100, 119):
        for rank in range(2):
            ident = 'asset_%s_%s' % (asset, rank)
            actors = [dict(data_id=1, motion='static'),
                      dict(data_id=asset, motion='static', source_list='objectList',
                           population_pool='obstacle_box', estimated_size=[1., 1., 1.],
                           planned_center_distance_m=8., actual_ego_role='front')]
            spec = dict(family='obstacle', mission=False, population_mode='generic_regular',
                        actors=2, seed=asset * 2 + rank, frozen_population_data_ids=[1, asset],
                        secondary_obstacle_support=dict(policy=rich.POLICY, ordinals=[1],
                            new_actor_ordinals=[1], primary_preserved=True,
                            mission_and_moving_excluded=True))
            items.append((ident, spec, actors))
    plans = front.assign_plans(items)
    for ident, _, actors in items:
        plan = plans[ident][front.PLAN_KEY]
        if plan['selected']:
            actors[1]['secondary_obstacle_support'] = dict(front_support=copy.deepcopy(plan))
        metadata[ident] = dict(spec=plans[ident], runtime_actors=actors, split='train')
    repaired = ['npc_%02d' % i for i in range(24)]
    controls = ['control_failure', 'control_fixed']
    animated = ['moving_%d' % i for i in range(8)]
    for ident in repaired + controls + animated:
        metadata[ident] = dict(spec=dict(seed=1), split='train',
                              runtime_actors=[dict(data_id=1, motion='walking' if ident in animated else 'static')])
        if ident in repaired:
            metadata[ident]['parked_npc_support_source'] = dict(geometry_changed=True)
    for i in range(5000 - len(metadata)):
        metadata['background_%04d' % i] = dict(spec=dict(seed=i), split='train', runtime_actors=[])
    for ident in metadata:
        entries.append(dict(id=ident, scenario='/scenario/' + ident,
                            scenario_sha256='scenario:' + ident, metadata='/metadata/' + ident, split='train'))
    deck = dict(count=5000, scenes=entries, config=dict(frozen_specs_path='/frozen', frozen_specs_sha256='frozen',
        obstacle_front_support_policy=dict(policy=front.POLICY, source_sha256=front.implementation_sha256(),
                                          source_manifest_sha256='v150')))
    previous = dict(source_manifest_sha256='v150', all_changed_slots=repaired,
        static=[dict(id=ident, sha256='scenario:' + ident) for ident in repaired + controls],
        animated=[dict(id=ident, sha256='scenario:' + ident) for ident in animated])
    quota = dict(target=64, pending=0, deficit=1, cells=[dict(id=controls[0], reasons=['old_native_review'])])
    return deck, metadata, previous, quota


class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.deck, self.metadata, self.previous, self.quota = fixture()
        self.specs = [copy.deepcopy(self.metadata[e['id']]['spec']) for e in self.deck['scenes']]

    def reader(self, path):
        value = str(path)
        if value == '/frozen':
            return copy.deepcopy(self.specs)
        return copy.deepcopy(self.metadata[Path(value).name])

    @staticmethod
    def digest(path):
        value = str(path)
        if value == '/frozen':
            return 'frozen'
        return 'scenario:' + Path(value).name

    def validate(self):
        with patch.object(qualification, 'sha256', side_effect=self.digest), \
                patch.object(qualification, 'read', side_effect=self.reader):
            return qualification.validate_source(self.deck)

    def select(self):
        with patch.object(qualification, 'sha256', side_effect=self.digest), \
                patch.object(qualification, 'read', side_effect=self.reader):
            return qualification.select(self.deck, self.previous, self.quota)

    def test_full_source_and_deterministic_64_plus_eight(self):
        before = copy.deepcopy((self.deck, self.metadata, self.previous, self.quota))
        a = self.select()
        b = self.select()
        self.assertEqual(a, b)
        self.assertEqual((self.deck, self.metadata, self.previous, self.quota), before)
        _, _, static, moving, controls, roles = a
        self.assertEqual((len(static), len(moving), len(set(static + moving))), (64, 8, 72))
        self.assertEqual(controls[0], 'control_failure')
        self.assertEqual(roles['control_failure']['kind'], 'historical_deficit_negative_regression')
        counts = qualification.Counter(v['kind'] for v in roles.values())
        self.assertEqual(counts, dict(front_selected=19, front_legacy=19,
            parked_npc_repaired_regression=24, historical_deficit_negative_regression=1,
            fixed_static_regression_control=1))

    def test_missing_historical_control_uses_fixed_controls(self):
        self.quota['cells'][0]['id'] = 'not_in_candidate'
        *_, controls, roles = self.select()
        self.assertEqual(set(controls), {'control_failure', 'control_fixed'})
        self.assertTrue(all(roles[ident]['kind'] == 'fixed_static_regression_control' for ident in controls))

    def test_helper_and_payload_hash_changes_are_rejected(self):
        self.deck['config']['obstacle_front_support_policy']['source_sha256'] = 'changed'
        with self.assertRaisesRegex(ValueError, 'helper/policy'):
            self.validate()
        self.deck['config']['obstacle_front_support_policy']['source_sha256'] = front.implementation_sha256()
        self.deck['scenes'][0]['scenario_sha256'] = 'changed'
        with self.assertRaisesRegex(ValueError, 'scenario checksum'):
            self.validate()

    def test_frozen_spec_and_duplicate_slot_changes_are_rejected(self):
        self.specs[0]['seed'] += 1
        with self.assertRaisesRegex(ValueError, 'frozen spec/split'):
            self.validate()
        self.specs[0]['seed'] -= 1
        self.deck['scenes'][1]['id'] = self.deck['scenes'][0]['id']
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.validate()

    def test_selected_actual_band_and_motion_are_checked(self):
        ident = next(i for i, m in self.metadata.items() if m['spec'].get(front.PLAN_KEY, {}).get('selected'))
        self.metadata[ident]['runtime_actors'][1]['planned_center_distance_m'] = 30.
        with self.assertRaisesRegex(ValueError, 'actual pose'):
            self.validate()
        self.metadata[ident]['runtime_actors'][1]['planned_center_distance_m'] = 8.
        self.metadata[ident]['runtime_actors'][1]['motion'] = 'walking'
        with self.assertRaisesRegex(ValueError, 'static population'):
            self.validate()

    def test_missing_asset_stratum_is_rejected(self):
        ident = next(i for i, m in self.metadata.items() if m['spec'].get(front.PLAN_KEY, {}).get('selected'))
        self.metadata[ident]['spec'].pop(front.PLAN_KEY)
        position = next(i for i, e in enumerate(self.deck['scenes']) if e['id'] == ident)
        self.specs[position] = copy.deepcopy(self.metadata[ident]['spec'])
        with self.assertRaisesRegex(ValueError, '19 assets'):
            self.validate()

    def test_original_npc_and_moving_payloads_cannot_change(self):
        self.previous['static'][0]['sha256'] = 'changed'
        with self.assertRaisesRegex(ValueError, 'NPC repaired payload'):
            self.select()
        self.previous['static'][0]['sha256'] = 'scenario:' + self.previous['static'][0]['id']
        self.previous['animated'][0]['sha256'] = 'changed'
        with self.assertRaisesRegex(ValueError, 'moving payload'):
            self.select()

    def test_other_source_or_nonterminal_history_is_rejected(self):
        self.previous['source_manifest_sha256'] = 'other'
        with self.assertRaisesRegex(ValueError, 'another v150'):
            self.select()
        self.previous['source_manifest_sha256'] = 'v150'
        self.quota['pending'] = 1
        with self.assertRaisesRegex(ValueError, 'Terminal historical'):
            self.select()

    def test_existing_output_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(FileExistsError):
                qualification.prepare(Path('/manifest'), Path('/previous'), Path('/quota'), Path(folder))

    def test_new_output_records_pending_native_and_all_input_hashes(self):
        selected = self.select()
        with tempfile.TemporaryDirectory() as temporary:
            paths = [Path('/manifest'), Path('/previous'), Path('/quota')]
            values = dict(zip(map(str, paths), (self.deck, self.previous, self.quota)))
            output = Path(temporary) / 'registration'
            with patch.object(qualification, 'select', return_value=selected), \
                    patch.object(qualification, 'sha256', side_effect=self.digest), \
                    patch.object(qualification, 'read', side_effect=lambda p: copy.deepcopy(values[str(p)])):
                qualification.prepare(*paths, output)
            registration = json.loads((output / 'registration.json').read_text())
            self.assertEqual(len(registration['static']), 64)
            self.assertEqual(len(registration['animated']), 8)
            self.assertFalse(registration['live_qualified'])
            self.assertFalse(registration['bulk_started'])
            self.assertEqual(len(registration['input_sha256']), 3)
            for name, count in (('static64', 64), ('animated8', 8)):
                subset = json.loads((output / name / 'manifest.json').read_text())
                self.assertEqual(subset['count'], count)
                self.assertIn('PENDING_NATIVE', subset['status'])


if __name__ == '__main__':
    unittest.main()
