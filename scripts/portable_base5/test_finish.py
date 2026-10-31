"""Small local fixtures only. Never contact SSH, Drive, ROS, MORAI or a GPU."""
import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import pickle
import shutil
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('portable_finish', Path(__file__).with_name('finish.py'))
f = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(f)


class FinishTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def put(self, path, value):
        path = self.root / path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else (json.dumps(value) + '\n').encode())
        return path

    def fixture(self, count=1):
        source = self.put('config/source.json', dict(scenes=[]))
        adopted = self.put('config/adopted.json', dict(scenes=[]))
        sensor = self.put('config/sensor.json', dict(sensors='frozen'))
        rows = []
        for index in range(count):
            raw = self.root / f'capture/raw_{index:03d}'
            scenario = self.put(f'capture/raw_{index:03d}/scenario.json', dict(id=index))
            self.put(f'capture/raw_{index:03d}/original_Intensity.png', b'ORIGINAL PNG')
            rows.append(dict(id=f'cell_{index}', source_cell_id=f'cell_{index}',
                status='accepted_candidate', capture_root=str(raw), scenario_sha256=f.digest(scenario)))
        selected = self.put('capture/selected_ledger.jsonl',
                            ''.join(json.dumps(row) + '\n' for row in rows).encode())
        self.put('capture/status.json', dict(stage='complete'))
        self.put('capture/quota.json', dict(deficit=0, cells=[]))
        self.put('capture/fresh_runtime_contract.json', dict(frozen=True))
        self.put('capture/rejected_native/audit.json', dict(accepted=False))
        return f.parser().parse_args(['finish', '--source', str(source), '--adopted', str(adopted),
            '--selected', str(selected), '--sensor', str(sensor), '--output', str(self.root / 'finished'),
            '--expected-scenes', str(count), '--min-free-gib', '0'])

    def budget(self):
        return f.Budget([self.root], self.root, 100 * f.GIB, 0)

    def package(self, root):
        root.mkdir(parents=True)
        payloads = {'data/cell/points.bin': b'POINTS'}
        payloads.update({f'data/cell/CAMERA_{i}/frame.jpg': b'JPEG' for i in range(1, 5)})
        record = dict(token='cell', scene_group_id='group', lidar_points=dict(lidar_path='data/cell/points.bin'),
            images={f'CAMERA_{i}': dict(img_path=f'data/cell/CAMERA_{i}/frame.jpg') for i in range(1, 5)},
            instances=[dict(original_semantic_class='obstacle', bbox_label_3d=2)])
        for split in ('train', 'val', 'test'):
            payloads[f'infos/512x1056/morai_base5_infos_{split}.pkl'] = pickle.dumps(dict(
                metainfo=dict(classes=('vehicle', 'pedestrian', 'obstacle')),
                data_list=[record] if split == 'train' else []))
        inventory = {}
        for name, body in payloads.items():
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(body)
            inventory[name] = dict(bytes=len(body), sha256=f.digest(path))
        (root / 'manifest.json').write_text(json.dumps(dict(version=1, scenes=1,
            profiles=['512x1056'], files=inventory, provenance={},
            image_policy=dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'),
            training_label_policy='animal_to_pedestrian_v1')))

    def fake_cpu(self, command, budget, log):
        self.assertNotIn('--run', command)
        self.assertNotIn('--gpus', command)
        target = Path(command[command.index('--output') + 1])
        target.mkdir()
        Path(log).write_bytes(b'fixture CPU result\n')
        if command[1].endswith('finalize_base5_layout_split.py'):
            for name in ('manifest.json', 'split_overlay.json'):
                (target / name).write_text('{}\n')
            selected = target / 'derived_selected_ledger.jsonl'
            selected.write_text('{}\n')
            result = dict(manifest=str(target / 'manifest.json'),
                          split_overlay=str(target / 'split_overlay.json'),
                          derived_selected_ledger=str(selected))
            (target / 'result.json').write_text(json.dumps(result))
        else:
            self.assertIn('--split-overlay', command)
            self.package(target / 'package')
            (target / 'report.json').write_text(json.dumps(dict(cohorts=dict(completed=dict(
                frames=1, training3_package=str(target / 'package'))))))

    def test_default_finish_is_read_only_and_no_subprocess(self):
        args = self.fixture()
        before = f.inventory([self.root], metadata=True)
        with patch.object(f.subprocess, 'Popen', side_effect=AssertionError('No process in dry mode')):
            result = f.finish(args)
        self.assertFalse(result['execute'])
        self.assertEqual(before, f.inventory([self.root], metadata=True))
        self.assertFalse(args.output.exists())

    def test_no_artificial_64_frame_limit_and_reject_duplicate_or_review(self):
        args = self.fixture(65)
        rows = f.selected_rows(args.selected, 65)
        self.assertEqual(len(rows), 65)
        changed = copy.deepcopy(rows); changed[0]['status'] = 'review'
        args.selected.write_text(''.join(json.dumps(r) + '\n' for r in changed))
        with self.assertRaisesRegex(ValueError, 'Reviewed'):
            f.selected_rows(args.selected, 65)
        args.selected.write_text(json.dumps(rows[0]) + '\n' + json.dumps(rows[0]) + '\n')
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            f.selected_rows(args.selected, 2)

    def test_unterminated_source_and_changed_scenario_fail_closed(self):
        args = self.fixture()
        args.selected.write_bytes(args.selected.read_bytes().rstrip(b'\n'))
        with self.assertRaisesRegex(ValueError, 'newline'):
            f.selected_rows(args.selected, 1)
        args.selected.write_bytes(args.selected.read_bytes() + b'\n')
        (self.root / 'capture/raw_000/scenario.json').write_text('CHANGED')
        with self.assertRaisesRegex(ValueError, 'scenario changed'):
            f.selected_rows(args.selected, 1)

    def test_native_authority_requires_zero_deficit_and_exact_reconstruction(self):
        args = self.fixture()
        rows = f.selected_rows(args.selected, 1)
        module = types.ModuleType('run_base5_fresh_completion')
        module.verify_parent = lambda source, parent: (rows, [], 12.5)
        with patch.dict(sys.modules, {'run_base5_fresh_completion': module}):
            result = f.verify_authoritative(args.source, args.selected, rows)
            self.assertEqual(result['frames'], 1)
            self.assertEqual(result['elapsed_sec'], 12.5)
            module.verify_parent = lambda source, parent: ([], [], 12.5)
            with self.assertRaisesRegex(ValueError, 'reconstruction'):
                f.verify_authoritative(args.source, args.selected, rows)
            (args.selected.parent / 'quota.json').write_text(json.dumps(dict(deficit=1, cells=['a'])))
            with self.assertRaisesRegex(ValueError, 'Unfilled quota'):
                f.verify_authoritative(args.source, args.selected, rows)
            (args.selected.parent / 'fresh_runtime_contract.json').unlink()
            with self.assertRaisesRegex(ValueError, 'authoritative completed fresh'):
                f.verify_authoritative(args.source, args.selected, rows)

    def test_budget_counts_hardlinks_once_and_fails_before_reserved_archive(self):
        a = self.put('disk/a', b'X' * 8192)
        os.link(a, a.with_name('b'))
        self.assertEqual(f.allocated_bytes([a.parent]), a.stat().st_blocks * 512)
        b = f.Budget([a.parent], self.root, 8192, 0)
        with self.assertRaisesRegex(ValueError, 'usage'):
            b.check(reserve=1)
        with patch.object(f.shutil, 'disk_usage', return_value=argparse.Namespace(free=0)):
            with self.assertRaisesRegex(ValueError, 'safety reserve'):
                f.Budget([a.parent], self.root, f.GIB, 1).check()

    def test_stage_resume_validates_outputs_and_preserves_failed_partial(self):
        out = self.root / 'steps'; out.mkdir()
        def failed(attempt):
            (attempt / 'partial').write_bytes(b'PRESERVE')
            raise ValueError('interrupted')
        with self.assertRaisesRegex(ValueError, 'interrupted'):
            f.step(out, 'CPU', 'contract', failed)
        def successful(attempt):
            p = attempt / 'result'; p.write_bytes(b'OK')
            return dict(path=str(p)), [attempt]
        result = f.step(out, 'CPU', 'contract', successful)
        self.assertIn('attempt001', result['path'])
        self.assertEqual((out / 'CPU.attempt000/partial').read_bytes(), b'PRESERVE')
        self.assertEqual(f.step(out, 'CPU', 'contract', failed), result)
        Path(result['path']).with_name('unexpected').write_bytes(b'EXTRA')
        with self.assertRaisesRegex(ValueError, 'inventory changed'):
            f.step(out, 'CPU', 'contract', failed)

    @unittest.skipUnless(shutil.which('zstd'), 'zstd unavailable; do not install automatically')
    def test_single_archive_verifies_hardlinked_payloads_without_parts(self):
        p = self.put('a', b'fixture native bytes'); twin = self.root / 'b'; os.link(p, twin)
        target = self.root / 'single.tar.zst'
        result = f.archive_single({'dataset/a': p, 'dataset/b': twin}, target, self.budget(), 1, 1)
        self.assertEqual(result['sha256'], f.digest(target))
        f.verify_archive(target, result['members'])
        self.assertEqual(list(self.root.glob('*.part*')), [])
        self.assertFalse(self.root.joinpath('single.tar').exists())
        bad = copy.deepcopy(result['members']); bad['dataset/a']['sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'verification failed'):
            f.verify_archive(target, bad)

    def test_symlink_payload_and_unsafe_paths_are_rejected(self):
        source = self.put('original', b'original')
        link = self.root / 'link'; link.symlink_to(source)
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            f.package_members(link, {})
        for name in ('../escape', '/absolute'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                f.safe_name(name)

    @unittest.skipUnless(shutil.which('zstd'), 'zstd unavailable')
    def test_finish_fixture_pipeline_resume_and_raw_preservation(self):
        args = self.fixture(); args.run = True
        before = f.inventory([self.root / 'capture'], metadata=True)
        with patch.object(f, 'verify_authoritative', return_value=dict(frames=1, deficit=0)), \
             patch.object(f, 'run_cpu', side_effect=self.fake_cpu):
            result = f.finish(args)
            self.assertEqual(result['stage'], 'complete')
            again = f.finish(args)
            self.assertEqual(again, result)
        self.assertEqual(before, f.inventory([self.root / 'capture'], metadata=True))
        self.assertEqual(len(list(args.output.rglob('base5_finished.tar.zst'))), 1)
        self.assertEqual(list(args.output.rglob('*.part[0-9]*')), [])
        args.threads += 1
        with self.assertRaisesRegex(ValueError, 'contract changed'):
            f.finish(args)

    def pull_args(self, run=False):
        values = ['pull', '--host', 'fixture-host', '--remote-dir', '/work/04_archive.attempt000',
                  '--output', str(self.root / 'received'), '--min-free-gib', '0']
        return f.parser().parse_args(values + (['--run'] if run else []))

    def test_pull_is_plan_only_without_run_and_rejects_shell_syntax(self):
        args = self.pull_args()
        with patch.object(f.subprocess, 'Popen', side_effect=AssertionError('No SSH in dry mode')):
            result = f.pull(args)
        self.assertFalse(args.output.exists())
        self.assertFalse(result['execute'])
        self.assertTrue(all('--checksum' in command and '--remove-source-files' not in command
                            for command in result['commands']))
        args.host = 'host; touch /bad'
        with self.assertRaises(ValueError):
            f.pull(args)

    def test_mock_pull_checks_received_sha_resumes_and_never_deletes_remote(self):
        args = self.pull_args(True); payload = b'LOCAL FIXTURE ARCHIVE'; checksum = hashlib.sha256(payload).hexdigest()
        calls = []
        def transfer(command, budget, log):
            calls.append(command); Path(log).write_text('fixture no network\n')
            target = Path(command[-1])
            target.write_bytes((checksum + '  base5_finished.tar.zst\n').encode()
                               if target.name == 'SHA256SUMS' else payload)
        with patch.object(f, 'run_cpu', side_effect=transfer):
            self.assertTrue(f.pull(args)['verified'])
            self.assertTrue(f.pull(args)['verified'])
        self.assertEqual(len(calls), 2)
        self.assertTrue(all('--remove-source-files' not in c and '--delete' not in c for c in calls))
        args.host = 'other-host'
        with self.assertRaisesRegex(ValueError, 'different remote'):
            f.pull(args)

    def test_mock_pull_corrupt_archive_is_not_marked_received(self):
        args = self.pull_args(True)
        def corrupt(command, budget, log):
            Path(log).write_text('fixture')
            target = Path(command[-1])
            target.write_bytes((('0' * 64) + '  base5_finished.tar.zst\n').encode()
                               if target.name == 'SHA256SUMS' else b'bad')
        with patch.object(f, 'run_cpu', side_effect=corrupt), \
             self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
            f.pull(args)
        self.assertFalse((args.output / 'RECEIVED.json').exists())

    def test_unowned_existing_pull_output_and_source_overlap_are_rejected(self):
        args = self.pull_args(True); args.output.mkdir()
        protected = args.output / 'personal.txt'; protected.write_bytes(b'PRESERVE')
        with self.assertRaisesRegex(ValueError, 'dedicated directory'):
            f.pull(args)
        self.assertEqual(protected.read_bytes(), b'PRESERVE')
        args = self.fixture(); args.output = args.selected.parent / 'bad'
        with self.assertRaisesRegex(ValueError, 'must not overlap'):
            f.finish(args)

    def test_dependency_closure_contains_ast_bodies_but_no_weights_or_installed_packages(self):
        paths = f.dependency_paths()
        self.assertTrue(all(path.suffix == '.py' and 'site-packages' not in str(path) for path in paths))
        for name in ('benchmark_base5_jpeg_cache.py', 'morai_jpeg_cache.py',
                     'prepare_base5_jpeg_portable.py', 'online_preprocessor.py'):
            self.assertTrue(any(path.name == name for path in paths))


if __name__ == '__main__':
    unittest.main()
