import copy
from contextlib import ExitStack
import json
from pathlib import Path
import sys
import tempfile
import unittest
from argparse import Namespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_obstacle_front_support as front
import base5_obstacle_rich_floor as rich
import validate_base5_front_recovery_live as probe
from test_base5_front_completion_adapter import examples


def sources():
    small = examples()
    items = []
    for index, meta in enumerate(examples()):
        row, actors = meta['spec'], meta['runtime_actors']
        del row[front.PLAN_KEY]
        row['frozen_population_data_ids'][-1] = 99
        actors[-1].update(data_id=99, name='LargeBox', estimated_size=[4., 1., .5])
        actors[-1].pop('secondary_obstacle_support')
        items.append(('large' + str(index), row, actors))
    plans = front.assign_plans(items)
    large = []
    for slot, row, actors in items:
        row = plans[slot]
        legacy = rich.sample(row, 2, actors[-1], [4., 1., .5], 'rear', 2)
        proof = front.apply(row, 2, actors[-1], [4., 1., .5], 2, legacy)
        actors[-1].update(planned_center_distance_m=proof['target_center_distance_m'],
                         actual_ego_role=proof['role'], secondary_obstacle_support=proof)
        large.append(dict(spec=row, runtime_actors=actors, retry=2, split='train',
                          scene_group_id='fresh:' + slot))
    return {('small' + str(i) if i < 4 else 'large' + str(i - 4)): meta
            for i, meta in enumerate(small + large)}


class ProbeTests(unittest.TestCase):
    def fixture(self, root):
        metadata, entries = sources(), {}
        for ident, meta in metadata.items():
            sp, mp = root / 'source' / (ident + '.json'), root / 'source' / (ident + '_metadata.json')
            probe.atomic_json(sp, dict(original=ident))
            probe.atomic_json(mp, meta)
            entries[ident] = dict(id=ident, scenario=str(sp), scenario_sha256=probe.sha256(sp),
                metadata=str(mp), split=meta['split'], family='obstacle', mission=False,
                role=meta['spec']['role'], distance_bounds=meta['spec']['distance_bounds'])
        config = dict(environment_tag='sunny13', fresh_geometry_policy=dict(policy='test'),
                      obstacle_front_support_policy=dict(policy=front.POLICY))
        deck = dict(config=config, count=len(entries), scenes=list(entries.values()))
        manifest = root / 'source' / 'manifest.json'
        probe.atomic_json(manifest, deck)
        args = Namespace(manifest=manifest, output=root / 'probes', resume=False,
                         run=False, prepare_only=True, time_budget_sec=1200)
        snapshot = (deck, entries, metadata, 'source_inventory',
                    dict(planned=8, selected=4, legacy=4, assets=2))
        return args, snapshot

    def mocks(self, stack, snapshot, outcome='accepted_candidate', park=True):
        deck, entries, metadata, _, _ = snapshot
        stack.enter_context(patch.object(probe, 'source_snapshot', return_value=snapshot))
        actors = [dict(data_id=data_id) for data_id in (1, 2, 10, 99)]
        ctx = dict(pools={pool: actors if pool == 'obstacle' else [] for pool in probe.completion.POOLS})
        stack.enter_context(patch.object(probe.completion.spatial.gen, 'context', return_value=ctx))
        stack.enter_context(patch.object(probe.completion.adaptive, 'prewarm', side_effect=lambda ctx, config: ctx))
        stack.enter_context(patch.object(probe.completion.adaptive.fa, 'normalized_sensor_calibration', return_value={}))
        stack.enter_context(patch.object(probe.completion.adaptive, 'proposal_score', return_value=dict(score=1.)))
        stack.enter_context(patch.object(probe.completion.adaptive.fa, 'novelty', return_value=1.))
        by_key = {(m['spec']['seed'], m['spec']['frozen_population_data_ids'][-1]): m for m in metadata.values()}
        def build(row, config, context, retry):
            actual = copy.deepcopy(by_key[(row['seed'], row['frozen_population_data_ids'][-1])])
            actual['retry'] = retry
            return dict(seed=row['seed'], extra=row['frozen_population_data_ids'][-1], retry=retry), actual
        stack.enter_context(patch.object(probe.completion.spatial, 'build', side_effect=build))

        def child(attempt, output, heartbeat):
            capture = Path(attempt['capture'])
            raw = capture / 'captures' / 'raw'
            probe.atomic_json(raw / 'metadata.json', probe.read(attempt['entry']['metadata']))
            probe.atomic_json(capture / 'shutdown_safety.json', dict(park_verified=park))
            probe.atomic_json(capture / 'session_contract.json', dict(test=True))
            row = dict(status=outcome, capture_root=str(raw))
            (capture / 'ledger.jsonl').write_text(json.dumps(row) + '\n')
            return 0 if outcome == 'accepted_candidate' else 1

        def verify(attempt, deck, source, pools):
            return dict(status=outcome, capture_root=str(Path(attempt['capture']) / 'captures' / 'raw'))
        launched = stack.enter_context(patch.object(probe.completion, 'run_child', side_effect=child))
        verified = stack.enter_context(patch.object(probe.completion, 'verify', side_effect=verify))
        return launched, verified

    def test_strata_are_deterministic_distinct_and_cover_all_four(self):
        with tempfile.TemporaryDirectory() as folder:
            _, snap = self.fixture(Path(folder))
            deck, entries, metadata, _, _ = snap
            before = copy.deepcopy(metadata)
            a = probe.select_strata(entries, metadata)
            b = probe.select_strata(entries, dict(reversed(list(metadata.items()))))
            self.assertEqual(a, b)
            self.assertEqual(len({r['source_id'] for r in a}), 4)
            self.assertEqual({(r['selected'], r['large_footprint']) for r in a},
                             {(True, True), (True, False), (False, True), (False, False)})
            self.assertEqual(metadata, before)

    def test_missing_stratum_never_reduces_probe_contract(self):
        with tempfile.TemporaryDirectory() as folder:
            _, snap = self.fixture(Path(folder))
            deck, entries, metadata, _, _ = snap
            metadata = {i: m for i, m in metadata.items() if not m['spec'][front.PLAN_KEY]['large_footprint']}
            with self.assertRaises(ValueError): probe.select_strata(entries, metadata)

    def test_default_prepares_four_official_children_without_ros(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, verified = self.mocks(stack, snap)
            self.assertEqual(probe.execute(args), 0)
            launched.assert_not_called()
            verified.assert_not_called()
            state = probe.read(args.output / 'state.json')
            proof = probe.read(args.output / 'proof.json')
            self.assertEqual(state['stage'], 'prepared')
            self.assertEqual(proof['prepared_count'], 4)
            self.assertFalse(proof['passed'])
            self.assertTrue(proof['planned_probe'])
            self.assertFalse(proof['fabricated_parent_deficit'])
            self.assertFalse(proof['training_data'])
            for attempt in state['attempts']:
                self.assertEqual(attempt['id'], attempt['source_id'])
                actual = probe.read(attempt['entry']['metadata'])
                self.assertEqual(actual['spec'], snap[2][attempt['source_id']]['spec'])
                self.assertNotEqual(attempt['entry']['scenario_sha256'], snap[1][attempt['source_id']]['scenario_sha256'])

    def test_live_four_then_completed_resume_never_recaptures_or_rewrites_state(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, verified = self.mocks(stack, snap)
            args.run = True
            self.assertEqual(probe.execute(args), 0)
            self.assertEqual(launched.call_count, 4)
            before = (args.output / 'state.json').read_bytes()
            self.assertFalse(probe.read(args.output / 'proof.json')['passed'])
            args.resume = True
            self.assertEqual(probe.execute(args), 0)
            self.assertEqual(launched.call_count, 4)
            self.assertEqual(verified.call_count, 8)
            self.assertEqual((args.output / 'state.json').read_bytes(), before)
            proof = probe.read(args.output / 'proof.json')
            self.assertTrue(proof['passed'])
            self.assertEqual(proof['accepted_count'], 4)
            self.assertEqual(proof['additional_completed_resume_raw'], 0)
            self.assertEqual(len(proof['ledger_sha256map']), 4)
            self.assertEqual(proof['state_sha256'], probe.sha256(args.output / 'state.json'))

    def test_quality_review_is_terminal_without_a_fifth_capture(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, _ = self.mocks(stack, snap, outcome='review')
            args.run = True
            self.assertEqual(probe.execute(args), 0)
            self.assertEqual(launched.call_count, 4)
            self.assertEqual(probe.read(args.output / 'state.json')['stage'], 'complete_with_review')
            args.resume = True
            self.assertEqual(probe.execute(args), 0)
            self.assertEqual(launched.call_count, 4)
            self.assertFalse(probe.read(args.output / 'proof.json')['passed'])

    def test_unsafe_park_halts_and_resume_cannot_reload(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, verified = self.mocks(stack, snap, park=False)
            args.run = True
            self.assertEqual(probe.execute(args), 1)
            self.assertEqual(launched.call_count, 1)
            verified.assert_not_called()
            self.assertEqual(probe.read(args.output / 'state.json')['stage'], 'halted')
            args.resume = True
            self.assertEqual(probe.execute(args), 1)
            self.assertEqual(launched.call_count, 1)

    def test_partial_existing_child_is_verification_only_never_reloaded(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, verified = self.mocks(stack, snap)
            self.assertEqual(probe.execute(args), 0)
            state = probe.read(args.output / 'state.json')
            Path(state['attempts'][0]['capture']).mkdir()
            args.run, args.resume = True, True
            self.assertEqual(probe.execute(args), 1)
            launched.assert_not_called()
            verified.assert_not_called()

    def test_interrupted_parent_recovers_complete_child_without_reloading_it(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, _ = self.mocks(stack, snap)
            self.assertEqual(probe.execute(args), 0)
            state = probe.read(args.output / 'state.json')
            launched.side_effect(state['attempts'][0], args.output, lambda root: None)
            args.run, args.resume = True, True
            self.assertEqual(probe.execute(args), 0)
            self.assertEqual(launched.call_count, 3)

    def test_changed_contract_and_terminal_raw_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, _ = self.mocks(stack, snap)
            args.run = True
            self.assertEqual(probe.execute(args), 0)
            state = probe.read(args.output / 'state.json')
            raw = Path(state['attempts'][0]['capture']) / 'captures' / 'raw' / 'metadata.json'
            raw.write_text('tampered')
            args.resume = True
            self.assertEqual(probe.execute(args), 1)
            self.assertEqual(launched.call_count, 4)

    def test_runtime_change_is_rejected_before_any_capture(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, _ = self.mocks(stack, snap)
            self.assertEqual(probe.execute(args), 0)
            changed = dict(probe.runtime_hashes(), injected='changed')
            args.run, args.resume = True, True
            with patch.object(probe, 'runtime_hashes', return_value=changed):
                self.assertEqual(probe.execute(args), 1)
            launched.assert_not_called()

    def test_journal_cannot_add_a_fifth_native_probe(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            launched, _ = self.mocks(stack, snap)
            self.assertEqual(probe.execute(args), 0)
            state = probe.read(args.output / 'state.json')
            state['attempts'].append(copy.deepcopy(state['attempts'][0]))
            probe.atomic_json(args.output / 'state.json', state)
            args.run, args.resume = True, True
            self.assertEqual(probe.execute(args), 1)
            launched.assert_not_called()

    def test_journal_cannot_change_registered_source_or_load_path(self):
        for key, value in (('source_id', 'foreign'), ('ordinal', 2), ('capture', '/tmp/foreign'),
                           ('moving', True)):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
                args, snap = self.fixture(Path(folder))
                launched, _ = self.mocks(stack, snap)
                self.assertEqual(probe.execute(args), 0)
                state = probe.read(args.output / 'state.json')
                state['attempts'][0][key] = value
                probe.atomic_json(args.output / 'state.json', state)
                args.run, args.resume = True, True
                self.assertEqual(probe.execute(args), 1)
                launched.assert_not_called()

    def test_explicit_resume_and_time_bounds_are_required(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, snap = self.fixture(Path(folder))
            self.mocks(stack, snap)
            args.time_budget_sec = 100000
            with self.assertRaises(ValueError): probe.execute(args)
            args.time_budget_sec = 1200
            args.output.mkdir()
            with self.assertRaises(FileExistsError): probe.execute(args)


if __name__ == '__main__':
    unittest.main()
