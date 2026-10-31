"""CPU-only tests. Never import Torch/MMEngine or contact a remote host."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


P = Path(__file__).with_name('base5_checkpoint_retention_recovery.py')
spec = importlib.util.spec_from_file_location('retention_recovery_under_test', P)
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class RetentionRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.work = self.root / 'work'
        self.work.mkdir()
        self.old = self.work / 'epoch_3.pth'
        self.resume = self.work / 'epoch_5.pth'
        self.old.write_bytes(b'old preserved checkpoint bytes')
        self.resume.write_bytes(b'full resume checkpoint bytes')
        self.entry = self.root / 'original.py'
        self.entry.write_text('raise AssertionError("must not execute during install")\n')
        self.contract = 'a' * 64
        (self.work / 'run_manifest.json').write_text(json.dumps(dict(contract_sha256=self.contract)))
        (self.work / 'last_checkpoint').write_text(str(self.resume) + '\n')
        self.authority_path = self.root / 'authority.json'
        self.authority = dict(
            schema=recovery.SCHEMA, authorized=True,
            contract_sha256=self.contract, work_dir=str(self.work),
            resume_checkpoint=dict(path=str(self.resume), sha256=sha(self.resume),
                                   epoch=5, iteration=32670),
            preserved_checkpoint=dict(path=str(self.old), sha256=sha(self.old), epoch=3),
            original_entrypoint=dict(path=str(self.entry), sha256=sha(self.entry)),
            receipt_root=str(self.root / 'receipts'))
        self.write_authority()
        self.calls = []
        calls = self.calls

        class FakeBackend:
            def remove(self, path):
                calls.append(os.fspath(path))
                raise OSError(116, 'original backend failure')

        self.backend = FakeBackend

    def write_authority(self):
        self.authority_path.write_text(json.dumps(self.authority))

    def install(self):
        guard = recovery.install_guard(self.authority_path, backend_class=self.backend)
        self.addCleanup(guard.uninstall)
        return guard

    def test_install_does_not_call_remove_or_run_original_and_pins_receipt(self):
        original = self.backend.remove
        before = {p: p.read_bytes() for p in (self.old, self.resume, self.entry,
                  self.authority_path, self.work / 'last_checkpoint', self.work / 'run_manifest.json')}
        with patch.dict(os.environ, {'WANDB_CONSOLE': 'off'}):
            guard = self.install()
        self.assertIsNot(self.backend.remove, original)
        self.assertEqual(self.calls, [])
        for p, data in before.items():
            self.assertEqual(p.read_bytes(), data)
        r = json.loads((guard.receipt_dir / 'ENTRY_VALIDATED.json').read_text())
        self.assertEqual(r['authority_sha256'], sha(self.authority_path))
        self.assertEqual(r['pid'], os.getpid())
        self.assertEqual(r['pid_start_ticks'], recovery._pid_start_ticks())
        self.assertEqual(r['wandb_console'], 'off')
        self.assertEqual(r['contract_sha256'], self.contract)
        self.assertFalse(r['checkpoint_metadata_modified'])

    def test_exact_old_path_preserved_once_and_receipt_fsynced_before_return(self):
        guard = self.install()
        before = self.old.read_bytes()
        with patch.object(recovery.os, 'fsync', wraps=os.fsync) as sync:
            self.assertIsNone(self.backend().remove(self.old))
        self.assertGreaterEqual(sync.call_count, 2)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.old.read_bytes(), before)
        self.assertEqual(guard.skip_count, 1)
        r = json.loads((guard.receipt_dir / 'PRESERVED_OLD_CHECKPOINT.json').read_text())
        self.assertFalse(r['unlink_attempted'])
        self.assertFalse(r['original_remove_called'])
        self.assertEqual(r['preserved_checkpoint']['sha256'], sha(self.old))

    def test_other_path_calls_original_and_preserves_oserror(self):
        self.install()
        for path in (self.work / 'epoch_4.pth', self.resume, self.work / 'best_epoch_3.pth'):
            with self.assertRaises(OSError) as caught:
                self.backend().remove(path)
            self.assertEqual(caught.exception.errno, 116)
        self.assertEqual(len(self.calls), 3)

    def test_duplicate_preservation_fails_closed_without_unlink(self):
        guard = self.install()
        self.backend().remove(str(self.old))
        with self.assertRaises(recovery.RetentionRecoveryError):
            self.backend().remove(self.old)
        self.assertEqual(guard.skip_count, 1)
        self.assertEqual(self.calls, [])

    def test_skip_receipt_failure_prevents_success_and_no_unlink(self):
        guard = self.install()
        with patch.object(recovery, '_write_receipt', side_effect=OSError(28, 'disk full')):
            with self.assertRaises(OSError):
                self.backend().remove(self.old)
        self.assertEqual(guard.skip_count, 0)
        self.assertEqual(self.calls, [])
        with self.assertRaises(recovery.RetentionRecoveryError):
            self.backend().remove(self.old)

    def test_install_receipt_failure_does_not_patch_backend(self):
        original = self.backend.remove
        with patch.object(recovery, '_write_receipt', side_effect=OSError(28, 'disk full')):
            with self.assertRaises(OSError):
                self.install()
        self.assertIs(self.backend.remove, original)

    def test_new_receipt_never_overwrites_existing_evidence(self):
        path = self.root / 'evidence.json'
        path.write_bytes(b'existing evidence')
        with self.assertRaises(FileExistsError):
            recovery._write_receipt(path, {'new': True})
        self.assertEqual(path.read_bytes(), b'existing evidence')

    def test_checkpoint_change_before_skip_fails_closed(self):
        guard = self.install()
        self.old.write_bytes(b'changed bytes')
        with self.assertRaises(recovery.RetentionRecoveryError):
            self.backend().remove(self.old)
        self.assertEqual(guard.skip_count, 0)
        self.assertEqual(self.calls, [])

    def test_symlink_old_checkpoint_rejected_without_patch(self):
        other = self.root / 'other.pth'
        other.write_bytes(self.old.read_bytes())
        self.old.unlink()
        self.old.symlink_to(other)
        original = self.backend.remove
        with self.assertRaises(recovery.RetentionRecoveryError):
            self.install()
        self.assertIs(self.backend.remove, original)

    def test_pin_or_authority_errors_reject_before_backend_patch(self):
        mutations = [
            lambda a: a.update(authorized=False),
            lambda a: a.update(schema='other'),
            lambda a: a.update(contract_sha256='b' * 64),
            lambda a: a['resume_checkpoint'].update(sha256='b' * 64),
            lambda a: a['preserved_checkpoint'].update(sha256='b' * 64),
            lambda a: a['original_entrypoint'].update(sha256='b' * 64),
            lambda a: a['resume_checkpoint'].update(iteration=0),
            lambda a: a['preserved_checkpoint'].update(path=str(self.resume)),
            lambda a: a.update(guard_sha256='b' * 64),
        ]
        baseline = copy.deepcopy(self.authority)
        original = self.backend.remove
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.authority = copy.deepcopy(baseline)
                mutate(self.authority)
                self.write_authority()
                with self.assertRaises(recovery.RetentionRecoveryError):
                    self.install()
                self.assertIs(self.backend.remove, original)

    def test_wrong_last_pointer_and_wrong_cli_entry_rejected(self):
        (self.work / 'last_checkpoint').write_text(str(self.old))
        with self.assertRaises(recovery.RetentionRecoveryError):
            self.install()
        (self.work / 'last_checkpoint').write_text(str(self.resume))
        with self.assertRaises(recovery.RetentionRecoveryError):
            recovery.install_guard(self.authority_path, original_entrypoint=self.root / 'wrong.py',
                                   backend_class=self.backend)

    def test_install_twice_never_stacks_guards(self):
        self.install()
        with self.assertRaises(recovery.RetentionRecoveryError):
            self.install()

    def test_cli_requires_console_off_before_install(self):
        with patch.dict(os.environ, {'WANDB_CONSOLE': 'auto'}), \
                patch.object(recovery, 'install_guard') as install:
            with self.assertRaises(recovery.RetentionRecoveryError):
                recovery.main(['--authority', str(self.authority_path), '--', str(self.entry)])
        install.assert_not_called()

    def test_cli_preserves_original_argv_and_direct_script_import_path(self):
        self.entry.write_text('# inert test entry\n')
        self.authority['original_entrypoint']['sha256'] = sha(self.entry)
        self.write_authority()
        real_install = recovery.install_guard
        original_argv, original_path = sys.argv, list(sys.path)
        observed = []

        def fake_run(path, *, run_name):
            observed.append((path, run_name, list(sys.argv), sys.path[0]))

        def fake_install(*args, **kwargs):
            return real_install(*args, **kwargs, backend_class=self.backend)

        with patch.dict(os.environ, {'WANDB_CONSOLE': 'off'}), \
                patch.object(recovery, 'install_guard', side_effect=fake_install), \
                patch.object(recovery.runpy, 'run_path', side_effect=fake_run):
            recovery.main(['--authority', str(self.authority_path), '--', str(self.entry),
                           '--config', '/original/config.json', '--remote-worker', '--resume'])
        self.assertEqual(observed, [(str(self.entry), '__main__',
            [str(self.entry), '--config', '/original/config.json', '--remote-worker', '--resume'],
            str(self.entry.parent))])
        self.assertIs(sys.argv, original_argv)
        self.assertEqual(sys.path, original_path)
        self.assertFalse(getattr(self.backend.remove, '_base5_retention_guard', False))

    def test_original_entry_failure_propagates_and_restores_python_context(self):
        real_install = recovery.install_guard
        original_argv, original_path = sys.argv, list(sys.path)

        def fake_install(*args, **kwargs):
            return real_install(*args, **kwargs, backend_class=self.backend)

        with patch.dict(os.environ, {'WANDB_CONSOLE': 'off'}), \
                patch.object(recovery, 'install_guard', side_effect=fake_install), \
                patch.object(recovery.runpy, 'run_path', side_effect=OSError(116, 'actual save failure')):
            with self.assertRaises(OSError) as caught:
                recovery.main(['--authority', str(self.authority_path), '--', str(self.entry)])
        self.assertEqual(caught.exception.errno, 116)
        self.assertIs(sys.argv, original_argv)
        self.assertEqual(sys.path, original_path)


if __name__ == '__main__':
    unittest.main()
