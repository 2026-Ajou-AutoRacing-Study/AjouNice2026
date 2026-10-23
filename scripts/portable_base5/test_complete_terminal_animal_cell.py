"""The last missing animal may move nearer, never change class or native safety."""
from copy import deepcopy
from contextlib import ExitStack, nullcontext
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import complete_terminal_animal_cell as last


def scene(distance=15., family='obstacle', mission=True, split='train', motion='static', animal=1):
    return dict(spec=dict(family=family, mission=mission, role='front', distance_bounds=[0, 100]),
        split=split, planned_primary_distance_m=distance,
        runtime_actors=[dict(name='animal-'+str(animal), motion=motion, data_id=animal)])


class TerminalAnimalCellTest(unittest.TestCase):
    def setUp(self):
        self.source = scene(distance=43.88)
        self.pools = {1:'animal', 2:'animal', 3:'vehicle'}

    def test_distance_only_exception_keeps_animal_and_structural_scope(self):
        self.assertTrue(last.same_family_readable_animal(self.source, scene(animal=2), self.pools))
        for kwargs in (dict(distance=4.99), dict(distance=30.01), dict(family='shadow'),
                       dict(mission=False), dict(split='val'), dict(motion='walking'), dict(animal=3)):
            self.assertFalse(last.same_family_readable_animal(self.source, scene(**kwargs), self.pools))
        self.assertEqual(last.RELAXED_FIELDS, ['purpose_range_bucket'])

    def test_unit_scale_and_nonempty_actor_required(self):
        scaled = scene()
        scaled['runtime_actors'][0]['asset_scale'] = {'scale':[1, 1, 1]}
        self.assertFalse(last.same_family_readable_animal(self.source, scaled, self.pools))
        empty = scene(); empty['runtime_actors'] = []
        self.assertFalse(last.same_family_readable_animal(self.source, empty, self.pools))

    def test_donor_order_is_deterministic_not_specific_asset_favoritism(self):
        metas = [scene(distance=25.), scene(distance=16., animal=2), scene(distance=15.)]
        metas[2]['runtime_actors'].append(dict(name='extra', motion='static', data_id=1))
        donors = [(dict(id=str(i), scenario_sha256=str(i)), meta) for i, meta in enumerate(metas)]
        ordered = last.readable_donors(self.source, donors, self.pools, 'slot', 4, set())
        self.assertEqual([entry['id'] for entry, _ in ordered], ['1', '0', '2'])
        self.assertEqual(last.readable_donors(self.source, donors, self.pools, 'slot', 4, {'0', '1', '2'}), [])

    def test_consumed_attempts_cannot_be_reset_or_implicitly_imported(self):
        for args in (SimpleNamespace(import_native_completion=[], attempts=3),
                     SimpleNamespace(import_native_completion=['unsafe'], attempts=5)):
            with self.assertRaisesRegex(RuntimeError, 'historical attempts'):
                last.execute(args)

    def test_parser_defaults_only_two_additional_finite_attempts(self):
        args = last.parser().parse_args(['--profile','profile','--parent','parent','--output','output'])
        self.assertEqual(args.attempts, 5)
        self.assertEqual(args.time_budget_sec, 600)
        self.assertFalse(args.run)

    def installation_fixture(self):
        parent, exact_parent = Path('/tmp/animal-parent'), Path('/tmp/exact-parent')
        history = [dict(id='missing', ordinal=i, unavailable='bounded CPU search',
                        manifest=None, result=None) for i in (1, 2, 3)]
        old = dict(contract=dict(parent=str(exact_parent), policy=last.flex.POLICY,
            adapter_sha256='hash', source_sha256='manifest', parent_pins={}),
            stage='complete_with_deficit', attempts=history, current=None)
        status = dict(stage='complete_with_deficit', error=None, failed=0, current=None,
                      accepted_candidate=4999, review=1)
        quota = dict(accepted=4999, deficit=1, pending=0, cells=[dict(id='missing')])
        original = scene(distance=43.88)
        values = {parent/'state.json':old, parent/'status.json':status,
                  parent/'quota.json':quota, Path('/tmp/original-meta'):original}
        selected = [dict(source_cell_id=str(i)) for i in range(4999)]
        root = (dict(manifest_sha256='manifest'), Path('/tmp/baseline'), exact_parent,
                Path('/tmp/new-output'), dict(scenes=[dict(id='missing', metadata='/tmp/original-meta')]),
                [], ['missing'], {})
        generator = SimpleNamespace(geo=SimpleNamespace(DEFAULT_RUNTIME_ASSET_DIR='assets',
            DEFAULT_MGEO='map'), context=lambda *unused:dict(pools=dict(animal=[dict(data_id=1)])))
        module = SimpleNamespace(batch=SimpleNamespace(campaign=SimpleNamespace(
            loop=SimpleNamespace(gen=generator))))
        args = SimpleNamespace(parent=parent, profile=Path('/tmp/profile'), output=Path('/tmp/new-output'))
        return args, old, values, selected, root, module

    def fixture_patches(self, values, selected, root, module):
        stack = ExitStack()
        stack.enter_context(patch.object(last.flex, 'read', side_effect=lambda p:values[Path(p)]))
        stack.enter_context(patch.object(last.flex, 'summary', return_value=root))
        stack.enter_context(patch.object(last.flex, 'validate_resume'))
        stack.enter_context(patch.object(last.flex, 'merge', return_value=(selected, ['missing'])))
        stack.enter_context(patch.object(last.flex.common, 'physical', side_effect=Path))
        stack.enter_context(patch.object(last.flex.common, 'sha', return_value='hash'))
        stack.enter_context(patch.object(last.flex.common, 'modules', return_value=(module, None)))
        stack.enter_context(patch.object(last.flex.common, 'route_compatibility', side_effect=lambda doc:nullcontext()))
        stack.enter_context(patch.object(last.flex.exact, 'prior_payloads', return_value={'old-payload'}))
        return stack

    def test_literal_prefix_and_three_cpu_ordinals_are_preserved(self):
        args, old, values, selected, root, module = self.installation_fixture()
        with self.fixture_patches(values, selected, root, module):
            prior_compatible = last.flex.compatible
            with last.installed(args):
                receipt = last.flex.import_completion([])
                self.assertEqual([row['ordinal'] for row in receipt['history']], [1, 2, 3])
                self.assertEqual(receipt['user_approved_extra_attempt_ordinals'], [4, 5])
                self.assertEqual(last.flex.summary()[5], selected)
                self.assertEqual(last.flex.summary()[6], ['missing'])
                self.assertEqual(receipt['preserved_primary_semantic_pool'], 'animal')
            self.assertIs(last.flex.compatible, prior_compatible)

    def test_native_failure_or_unsafe_parent_cannot_use_cpu_exception(self):
        for mutation in ('native', 'halted', 'duplicate_history'):
            args, old, values, selected, root, module = self.installation_fixture()
            if mutation == 'native':
                old['attempts'][0]['unavailable'] = None
                old['attempts'][0]['result'] = dict(selected=None)
            elif mutation == 'halted':
                old['stage'] = 'halted'
            else:
                old['attempts'][2]['ordinal'] = 2
            with self.fixture_patches(values, selected, root, module):
                with self.assertRaises(RuntimeError):
                    with last.installed(args):
                        self.fail('Unsafe exception must not install')


if __name__ == '__main__':
    unittest.main()
