"""원본 변경·ROS·시뮬레이터 접촉 없는 공통 safe-PAUSE 인계 회귀 검사."""
import copy
import fcntl
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import unified_paused_import as handoff


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def ledger(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))


class UnifiedPausedImportTests(unittest.TestCase):
    def fixture(self, directory):
        base = Path(directory)
        old, new, scripts = base/'old', base/'new', base/'runtime'
        old.mkdir(); scripts.mkdir()
        driver = write(scripts/'run_base5_batch_campaign.py', {'driver': 'original'})
        write(scripts/'base5_campaign_replenishment.py', {'replenishment': 'original'})
        write(scripts/'base5_campaign_import.py', {'importer': 'original'})
        entries, accepted, native = [], [], []
        for index in range(4):
            ident = 'scene_%02d' % index
            scenario = write(base/'source'/f'{ident}.json', {'index': index})
            metadata = write(base/'source'/f'{ident}.meta.json', {'calibration_only': False})
            entries.append(dict(id=ident, scenario=str(scenario), scenario_sha256=handoff.sha(scenario),
                                metadata=str(metadata)))
            capture = old/'chunk_0000/capture/captures'/ident
            write(capture/'scenario.json', {'index': index})
            write(capture/'metadata.json', {'calibration_only': False})
            write(capture/'audit.json', {'review_reasons': []})
            (capture/'sensor.bin').write_bytes(b'original native input')
            row = dict(id=ident, status='accepted_candidate', audit=dict(review_reasons=[]),
                       capture_root=str(capture), scenario_sha256=handoff.sha(capture/'scenario.json'),
                       between_scene_park=dict(park_verified=True))
            native.append(row)
            accepted.append(dict(copy.deepcopy(row), source_cell_id=ident))
        source = write(base/'source/manifest.json', dict(count=4, scenes=entries))
        root = old/'chunk_0000/capture'
        write(old/'chunk_0000/manifest.json', dict(count=4, scenes=entries))
        write(root/'session_contract.json', {'native': True, 'runner_options': {'shared_ros_batch': True}})
        write(root/'status.json', {'stage': 'complete'})
        write(root/'shutdown_safety.json', dict(park_verified=True, gear=1,
            brake_held_during_requests=True, request_fields=dict(option=2, gear=1)))
        native[1]['status'] = 'review'
        native[2]['status'] = 'review'
        ledger(root/'ledger.jsonl', native[:3])
        write(old/'PAUSE', {'reason': 'operator_graceful_pause'})
        write(old/'.batch.lock', {'lock': True})
        selected = accepted[:2]
        ledger(old/'selected_ledger.jsonl', selected)
        deficits = [dict(id='scene_02', index=2, reasons=[dict(reason='quality_exhausted')])]
        write(old/'quota.json', dict(target=4, accepted=2, deficit=1, pending=1, cells=deficits))
        write(old/'status.json', dict(stage='paused', current=None, error=None, failed=0,
            requested=4, finished=3, checkpoint_finished=3, accepted_candidate=2,
            review=1, elapsed_sec=123.456))
        for index in (1, 2):
            recovery = old/'chunk_0000/recovery'/('scene_%02d' % index)
            write(recovery/'status.json', dict(stage='complete' if index == 1 else 'complete_with_deficit'))
            ledger(recovery/'selected_ledger.jsonl', [accepted[index]] if index == 1 else [])
            write(recovery/'quota.json', dict(accepted=int(index == 1), deficit=int(index == 2),
                pending=0, cells=deficits[0]['reasons'] if index == 2 else []))
            for ordinal in range(2 if index == 2 else 1):
                write(recovery/f'scene_{index:02d}/round_{ordinal}/report.json',
                      dict(stage='complete', attempts=[dict(kind='original', retry=ordinal)]))
        legacy = dict(manifest=handoff.sha(source), indices=list(range(4)), rounds=2, code={})
        contract = dict(copy.deepcopy(legacy), batch_driver_sha256=handoff.sha(driver), chunk_size=4,
                        bulk=True, replenishment_driver_sha256=handoff.sha(scripts/'base5_campaign_replenishment.py'))
        write(old/'batch_contract.json', contract)
        batch = SimpleNamespace(__file__=str(driver), campaign=SimpleNamespace(contract=lambda *args:copy.deepcopy(legacy)),
            inspect_batch=lambda *args:copy.deepcopy(native[:3]), verify_recovery=lambda *args:0,
            is_collision_failure=lambda row:False, is_stationary_failure=lambda row:False)
        filename = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts/base5_campaign_import.py'
        spec = importlib.util.spec_from_file_location('_base5_original_import_for_paused_tests', filename)
        importer = importlib.util.module_from_spec(spec); spec.loader.exec_module(importer)
        return SimpleNamespace(base=base, source=source, old=old, new=new, scripts=scripts,
            batch=batch, importer=importer, selected=selected, deficits=deficits)

    def export(self, f, **kwargs):
        return handoff.export(f.source, f.old, f.new, batch=f.batch, importer=f.importer, **kwargs)

    def test_plan_is_readonly(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); before = handoff.historical.tree_snapshot(f.old)
            plan = handoff.plan(f.source, f.old, f.new)
            self.assertEqual((plan['selected_count'], plan['deficit_count'], plan['pending']), (2, 1, 1))
            self.assertFalse(plan['files_written'] or f.new.exists())
            self.assertEqual(before, handoff.historical.tree_snapshot(f.old))

    def test_exact_selected_deficit_elapsed_rounds_and_original_raw_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); before = handoff.historical.tree_snapshot(f.old)
            result = self.export(f)
            receipt = handoff.read(f.new/'imported_prefix.json')
            self.assertEqual(f.importer.verify(f.new/'imported_prefix.json', f.source, list(range(4))),
                             (f.selected, f.deficits, 123.456))
            self.assertEqual(len(receipt['recovery_history']), 2)
            self.assertEqual(len(receipt['recovery_history'][1]['reports']), 2)
            self.assertTrue(result['recovery_round_bounds_preserved'])
            self.assertTrue((f.old/'PAUSE').exists())
            self.assertEqual(before, handoff.historical.tree_snapshot(f.old))

    def test_all_hooks_are_readonly_and_original_callbacks_run(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); calls = []
            result = self.export(f, verify_context=lambda *args:calls.append(('context', args)),
                verify_regular=lambda *args:calls.append(('regular', args)),
                verify_recovery_extra=lambda *args:calls.append(('recovery', args)))
            self.assertEqual([kind for kind, _ in calls], ['context', 'regular', 'recovery', 'recovery'])
            self.assertTrue(result['original_import_verifier_passed'])

    def test_static_profile_cannot_silently_lose_extra_verifiers(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            write(f.old/'static_attitude_batch_contract.json', {'profile': 'original'})
            with self.assertRaisesRegex(RuntimeError, 'extra profile'):
                self.export(f)
            self.assertFalse(f.new.exists())

    def test_changed_raw_after_export_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); self.export(f)
            (Path(f.selected[0]['capture_root'])/'sensor.bin').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'evidence changed'):
                f.importer.verify(f.new/'imported_prefix.json', f.source, list(range(4)))

    def test_unsafe_parent_and_quota_variants_rejected(self):
        variants = [('stage', 'halted'), ('current', 'active'), ('failed', 1), ('error', 'Park failed'),
                    ('accepted_candidate', 3), ('elapsed_sec', float('nan')), ('checkpoint_finished', 2)]
        for key, value in variants:
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                f = self.fixture(directory); status = handoff.read(f.old/'status.json'); status[key] = value
                write(f.old/'status.json', status)
                with self.assertRaises(RuntimeError):self.export(f)
                self.assertFalse(f.new.exists())

    def test_stop_replenishment_symlink_and_duplicate_refused(self):
        for kind in ('stop', 'replenishment', 'symlink', 'duplicate'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                f = self.fixture(directory)
                if kind == 'stop':write(f.old/'STOP', {'stop': True})
                elif kind == 'replenishment':write(f.old/'replenishment/state.json', {'attempts': []})
                elif kind == 'symlink':(f.old/'unrelated_symlink').symlink_to(f.source)
                else:ledger(f.old/'selected_ledger.jsonl', [f.selected[0], f.selected[0]])
                with self.assertRaises(RuntimeError):self.export(f)
                self.assertFalse(f.new.exists())

    def test_active_native_lock_refused_without_creating_files(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            with (f.old/'.batch.lock').open('r') as stream:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with self.assertRaisesRegex(RuntimeError, 'still active'):
                    self.export(f)
            self.assertFalse(f.new.exists())

    def test_torn_native_ledger_is_not_repaired(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); path = f.old/'chunk_0000/capture/ledger.jsonl'
            path.write_bytes(path.read_bytes().rstrip(b'\n')); before = path.read_bytes()
            with self.assertRaisesRegex(RuntimeError, 'Torn'):
                self.export(f)
            self.assertEqual(path.read_bytes(), before); self.assertFalse(f.new.exists())

    def test_native_park_and_boundary_are_required(self):
        for kind in ('park', 'brake', 'boundary', 'incomplete'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                f = self.fixture(directory); root = f.old/'chunk_0000/capture'
                if kind == 'boundary':
                    rows = handoff.strict_rows(root/'ledger.jsonl'); rows[0]['between_scene_park'] = {}
                    ledger(root/'ledger.jsonl', rows)
                elif kind == 'incomplete':write(root/'status.json', {'stage': 'running'})
                else:
                    proof = handoff.read(root/'shutdown_safety.json')
                    proof['park_verified' if kind == 'park' else 'brake_held_during_requests'] = False
                    write(root/'shutdown_safety.json', proof)
                with self.assertRaisesRegex(RuntimeError, 'Native'):
                    self.export(f)
                self.assertFalse(f.new.exists())

    def test_isolated_native_uses_final_park_not_nonexistent_shared_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); root = f.old/'chunk_0000/capture'
            write(root/'session_contract.json', {'runner_options': {'shared_ros_batch': False}})
            rows = handoff.strict_rows(root/'ledger.jsonl')
            for row in rows:row.pop('between_scene_park', None)
            ledger(root/'ledger.jsonl', rows)
            # 원형 inspect_batch의 shared 경계 검사는 callback이 별도로 한다.
            self.assertEqual(len(handoff._native_safety(f.old, lambda path:None)), 1)

    def test_public_exit_verifier_is_readonly_and_rejects_active_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); before = handoff.historical.tree_snapshot(f.old)
            result = handoff.verify_exit(f.old)
            self.assertTrue(result['paused']); self.assertFalse(result['files_written'])
            self.assertEqual(before, handoff.historical.tree_snapshot(f.old))
            status = handoff.read(f.old/'status.json'); status['current'] = 'still_running'
            write(f.old/'status.json', status)
            with self.assertRaisesRegex(RuntimeError, 'parent paused'):
                handoff.verify_exit(f.old)

    def test_bounded_chunk_exit_can_explicitly_omit_marker_but_export_cannot(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory); (f.old/'PAUSE').unlink()
            before = handoff.historical.tree_snapshot(f.old)
            with self.assertRaisesRegex(RuntimeError, 'Normal PAUSE'):
                handoff.verify_exit(f.old)
            result = handoff.verify_exit(f.old, require_marker=False)
            self.assertTrue(result['paused'])
            self.assertFalse(result['pause_marker_required'] or result['pause_marker_present'])
            self.assertEqual(before, handoff.historical.tree_snapshot(f.old))
            with self.assertRaisesRegex(RuntimeError, 'Normal PAUSE'):
                self.export(f)
            write(f.old/'STOP', {'stop': True})
            with self.assertRaisesRegex(RuntimeError, 'Normal PAUSE'):
                handoff.verify_exit(f.old, require_marker=False)

    def test_unknown_or_incomplete_recovery_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            write(f.old/'chunk_0000/recovery/scene_02/status.json', {'stage': 'running'})
            with self.assertRaisesRegex(RuntimeError, 'Incomplete'):
                self.export(f)
            self.assertFalse(f.new.exists())

    def test_selected_or_deficit_evidence_cannot_drift(self):
        for kind in ('selected', 'deficit'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                f = self.fixture(directory)
                if kind == 'selected':
                    rows = copy.deepcopy(f.selected); rows[0]['extra_relabel'] = True
                    ledger(f.old/'selected_ledger.jsonl', rows)
                else:
                    quota = handoff.read(f.old/'quota.json'); quota['cells'][0]['reasons'] = ['new round cap']
                    write(f.old/'quota.json', quota)
                with self.assertRaisesRegex(RuntimeError, 'evidence|bounds'):
                    self.export(f)
                self.assertFalse(f.new.exists())

    def test_callback_cannot_repair_old_output_or_start_child(self):
        import subprocess
        for operation in ('write', 'child'):
            with self.subTest(operation=operation), tempfile.TemporaryDirectory() as directory:
                f = self.fixture(directory); before = handoff.historical.tree_snapshot(f.old)
                def callback(*args):
                    if operation == 'write':(f.old/'status.json').write_text('{}')
                    else:subprocess.Popen(['true'])
                with self.assertRaises(RuntimeError):self.export(f, verify_context=callback)
                self.assertFalse(f.new.exists()); self.assertEqual(before, handoff.historical.tree_snapshot(f.old))

    def test_nested_schema1_lineage_is_verified_and_ancestor_pins_flattened(self):
        with tempfile.TemporaryDirectory() as directory:
            f = self.fixture(directory)
            ancestor = f.base/'previous'; ancestor.mkdir()
            capture = ancestor/'capture'; original = Path(f.selected[0]['capture_root'])
            for filename in original.iterdir():
                target = capture/filename.name; target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(filename.read_bytes())
            inherited = copy.deepcopy(f.selected[0]); inherited['capture_root'] = str(capture)
            f.selected[0] = inherited; ledger(f.old/'selected_ledger.jsonl', f.selected)
            native = f.batch.inspect_batch(None, None); native[0]['status'] = 'review'
            f.batch.inspect_batch = lambda *args:copy.deepcopy(native)
            nested = dict(schema=1, original_output=str(ancestor), manifest_sha256=handoff.sha(f.source),
                selected=[inherited], deficits=[], elapsed_sec=20.0,
                hashes={str(path):handoff.sha(path) for path in capture.iterdir()}, sensor_stats={})
            write(f.old/'imported_prefix.json', nested)
            contract = handoff.read(f.old/'batch_contract.json')
            contract.update(imported_prefix_sha256=handoff.sha(f.old/'imported_prefix.json'),
                import_driver_sha256=handoff.sha(f.scripts/'base5_campaign_import.py'))
            write(f.old/'batch_contract.json', contract)
            result = self.export(f)
            receipt = handoff.read(f.new/'imported_prefix.json')
            self.assertEqual(len(result['import_lineage']), 1)
            self.assertTrue(set(nested['hashes']) <= set(receipt['hashes']))
            (capture/'sensor.bin').write_bytes(b'ancestor corruption')
            with self.assertRaisesRegex(ValueError, 'evidence changed'):
                f.importer.verify(f.new/'imported_prefix.json', f.source, list(range(4)))


if __name__ == '__main__':
    unittest.main()
