import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_static_extent_recovery as recovery
from base5_capture_safety import sha256


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True)+'\n')


class ExtentRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.ident = 'sunny13_sunny11_b5v2_04060'
        self.before = dict(spec=dict(seed=1, family='obstacle', mission=True,
            avoidance_phase='approach', static_mission_extent=dict(stratum='before')),
            split='train', retry=4, runtime_actors=[dict(data_id=50300003, name='Dog2',
                source_list='pedestrianList', unique_id=8500, motion='static')],
            static_mission_extent=dict(stratum='before', primary_center_not_ego_used_for_coverage=True))

    def build(self, row, config, ctx, retry):
        meta = copy.deepcopy(self.before)
        meta.update(spec=row, retry=retry)
        return dict(retry=retry), meta

    def fixture(self):
        root = self.root/'capture'/self.ident
        audit = dict(raw_complete=True, accepted_candidate=False,
            review_reasons=['primary_no_sensor_evidence'], missing_uids=[], runtime_missing_uids=[],
            capture_safety=dict(collision=dict(observed=True, no_collision_verified=True, non_ego_uids=[]),
                                support=dict(stationary_support_observed=True)))
        write(root/'audit.json', audit)
        write(root/'metadata.json', self.before)
        write(root/'scenario.json', dict(scene=1))
        entry = dict(id=self.ident, scenario_sha256=sha256(root/'scenario.json'))
        row = dict(id=self.ident, status='review', scenario_sha256=entry['scenario_sha256'],
                   capture_root=str(root), audit=audit)
        ledger = self.root/'ledger.jsonl'
        ledger.write_text(json.dumps(row)+'\n')
        write(self.root/'shutdown_safety.json', dict(park_verified=True))
        return ledger, row, {self.ident: entry}, {self.ident: self.before}

    def test_only_first_valid_new_geometry_is_selected_without_visibility_ranking(self):
        calls = []
        def build(row, config, ctx, retry):
            calls.append(retry)
            if retry < 7:
                raise ValueError('geometry invalid')
            return self.build(row, config, ctx, retry)
        scene, meta, proof = recovery.first_candidate(self.before, {}, {}, set(), build=build)
        self.assertEqual(calls, [5, 6, 7])
        self.assertEqual(scene, dict(retry=7))
        self.assertEqual(meta['spec'], self.before['spec'])
        self.assertEqual(proof['selected_retry'], 7)
        self.assertEqual(proof['rejected'], {'geometry invalid': 2})

    def test_duplicate_payload_cannot_be_retried_as_new_capture(self):
        checksum = hashlib.sha256((json.dumps(dict(retry=5), ensure_ascii=False, indent=2)+'\n').encode()).hexdigest()
        _, _, proof = recovery.first_candidate(self.before, {}, {}, {checksum}, build=self.build)
        self.assertEqual(proof['selected_retry'], 6)
        self.assertEqual(proof['rejected'], {'duplicate_source_or_candidate_payload': 1})

    def test_nonordinary_contract_errors_are_not_hidden_as_geometry_misses(self):
        def build(*args):
            raise RuntimeError('source changed')
        with self.assertRaisesRegex(RuntimeError, 'source changed'):
            recovery.first_candidate(self.before, {}, {}, set(), build=build)
        def changed(row, config, ctx, retry):
            scene, meta = self.build(row, config, ctx, retry)
            meta['spec']['seed'] += 1
            return scene, meta
        with self.assertRaises(recovery.RecoveryContractError):
            recovery.first_candidate(self.before, {}, {}, set(), build=changed)

    def test_proposal_budget_is_finite_and_source_retry_never_reused(self):
        calls = []
        def invalid(row, config, ctx, retry):
            calls.append(retry)
            raise ValueError('infeasible')
        with self.assertRaisesRegex(recovery.RecoveryContractError, 'exhausted'):
            recovery.first_candidate(self.before, {}, {}, set(), maximum=3, build=invalid)
        self.assertEqual(calls, [5, 6, 7])
        for value in (0, 65, True):
            with self.assertRaises(recovery.RecoveryContractError):
                recovery.first_candidate(self.before, {}, {}, set(), maximum=value)

    def test_explicit_three_generation_chain_cannot_reset_budget(self):
        self.assertEqual(recovery.next_attempt({}, 1), 1)
        for requested in (0, 2, 4, True):
            with self.assertRaises(recovery.RecoveryContractError):
                recovery.next_attempt({}, requested)
        prior = dict(provenance=dict(static_extent_visibility_recovery=dict(attempt=1)))
        self.assertEqual(recovery.next_attempt(prior, 2), 2)
        with self.assertRaises(recovery.RecoveryContractError):
            recovery.next_attempt(prior, 1)

    def test_exact_visibility_review_and_failure_source_are_pinned(self):
        ledger, row, entries, metadata = self.fixture()
        failures, pins = recovery.review_rows(ledger, entries, metadata)
        self.assertEqual(set(failures), {self.ident})
        self.assertIn(str((Path(row['capture_root'])/'audit.json').resolve()), pins)
        self.assertIn(str((self.root/'shutdown_safety.json').resolve()), pins)

    def test_additional_review_reason_failed_status_or_missing_park_rejected(self):
        for kind in ('reason', 'failed', 'park'):
            ledger, row, entries, metadata = self.fixture()
            if kind == 'reason':
                row['audit']['review_reasons'].append('collision')
                write(Path(row['capture_root'])/'audit.json', row['audit'])
            elif kind == 'failed':
                row['status'] = 'failed'
            else:
                write(self.root/'shutdown_safety.json', dict(park_verified=False))
            ledger.write_text(json.dumps(row)+'\n')
            with self.subTest(kind=kind), self.assertRaises(recovery.RecoveryContractError):
                recovery.review_rows(ledger, entries, metadata)

    def test_collision_or_stationary_uncertainty_never_becomes_ordinary_deficit(self):
        for field, value in (('observed', False), ('no_collision_verified', False), ('non_ego_uids', [9])):
            ledger, row, entries, metadata = self.fixture()
            row['audit']['capture_safety']['collision'][field] = value
            write(Path(row['capture_root'])/'audit.json', row['audit'])
            ledger.write_text(json.dumps(row)+'\n')
            with self.subTest(field=field), self.assertRaises(recovery.RecoveryContractError):
                recovery.review_rows(ledger, entries, metadata)

    def test_other_asset_slot_and_mismatched_scenario_or_raw_spec_rejected(self):
        ledger, row, entries, metadata = self.fixture()
        row['scenario_sha256'] = 'foreign'
        ledger.write_text(json.dumps(row)+'\n')
        with self.assertRaises(recovery.RecoveryContractError):
            recovery.review_rows(ledger, entries, metadata)
        ledger, row, entries, metadata = self.fixture()
        captured = copy.deepcopy(self.before)
        captured['spec']['seed'] = 999
        write(Path(row['capture_root'])/'metadata.json', captured)
        with self.assertRaises(recovery.RecoveryContractError):
            recovery.review_rows(ledger, entries, metadata)


if __name__ == '__main__':
    unittest.main()
