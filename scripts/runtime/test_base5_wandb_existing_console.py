import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base5_wandb_existing_console as e
from base5_wandb_console_mirror import canonical


class API:
    def __init__(self):
        self.rows = ['old stale first\n', 'old stale tail\n']
        self.posts = []
        self.allowed = True
        self.fail_after = None

    def guard(self):
        if not self.allowed:
            raise e.MirrorError('writer active')

    def output(self):
        return list(self.rows)

    def append(self, offset, lines):
        self.guard()
        self.posts.append((offset, list(lines)))
        rows = canonical(lines)
        if self.fail_after is not None:
            rows = rows[:self.fail_after]
        self.rows[offset:offset + len(rows)] = rows
        if self.fail_after is not None:
            self.fail_after = None
            raise TimeoutError('response lost')

    def heartbeat(self):
        return None

    def complete(self):
        return None


class ExistingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.prelude = self.root / 'prelude.log'
        self.new = self.root / 'new.log'
        self.prelude.write_text('first\nsecond\nthird\nfourth\n--- host boundary ---\n')
        self.new.write_text('new first\npartial')
        self.status = self.root / 'status.json'
        self.status.write_text(json.dumps({'state': 'running'}))
        self.a = {'authorized': True, 'entity': 'team', 'project': 'project',
                  'run': 'existing', 'contract_sha256': 'a' * 64,
                  'training_status': str(self.status), 'guard_entry_receipt': str(self.root / 'entry.json'),
                  'prelude': {'path': str(self.prelude), 'sha256': hashlib.sha256(self.prelude.read_bytes()).hexdigest()},
                  'newlog': str(self.new), 'state_dir': str(self.root / 'state'), 'poll_interval_seconds': 20}
        self.a['config_binding'] = {'storage_authority_sha256': 'b' * 64}
        self.api = API()

    def mirror(self):
        return e.ExistingMirror(self.a, 'authority-sha', self.api, 'ram-secret-never-upload',
                                lambda: '2026-10-23T00:00:00Z')

    def test_restore_then_append_without_extra_boundary_and_partial(self):
        mirror = self.mirror()
        mirror.poll()
        self.assertEqual(self.api.rows, self.prelude.read_text().splitlines(True) + ['new first\n'])
        self.assertEqual([p[0] for p in self.api.posts], [0, 5])
        self.assertEqual(json.loads((self.root / 'state/original_console_snapshot.json').read_text()),
                         ['old stale first\n', 'old stale tail\n'])
        with self.new.open('a') as f:
            f.write(' done\n')
        self.mirror().poll()
        self.assertEqual(self.api.posts[-1][0], 6)
        self.assertEqual(self.api.rows[-1], 'partial done\n')

    def test_restore_response_lost_after_full_commit_is_idempotent(self):
        self.api.fail_after = 999
        with self.assertRaises(TimeoutError):
            self.mirror().poll()
        self.mirror().poll()
        self.assertEqual([p[0] for p in self.api.posts], [0, 5])

    def test_partial_restore_retry_accepts_only_known_snapshot_tail(self):
        self.api.fail_after = 1
        with self.assertRaises(TimeoutError):
            self.mirror().poll()
        self.assertEqual(self.api.rows, ['first\n', 'old stale tail\n'])
        self.mirror().poll()
        self.assertEqual([p[0] for p in self.api.posts], [0, 0, 5])

    def test_unknown_writer_after_partial_restore_refused(self):
        self.api.fail_after = 1
        with self.assertRaises(TimeoutError):
            self.mirror().poll()
        self.api.rows[1] = 'unknown concurrent writer\n'
        with self.assertRaises(e.MirrorError):
            self.mirror().poll()
        self.assertEqual(len(self.api.posts), 1)

    def test_guard_and_secrets_fail_before_any_write(self):
        self.api.allowed = False
        with self.assertRaises(e.MirrorError):
            self.mirror().poll()
        self.api.allowed = True
        with self.new.open('a') as f:
            f.write('\nWANDB_API_KEY=super-secret-123456789\n')
        with self.assertRaises(e.MirrorError):
            self.mirror().poll()
        self.assertEqual(self.api.posts, [])

    def test_terminal_completion_is_local_only(self):
        self.new.write_text('complete\n')
        mirror = self.mirror()
        mirror.poll()
        self.status.write_text(json.dumps({'state': 'finished'}))
        mirror.poll()
        self.assertFalse(mirror.s['complete'])
        mirror.poll()
        self.assertTrue(mirror.s['complete'])
        self.assertEqual(len(self.api.posts), 2)

    def test_protocol_files_only_no_metadata_or_lifecycle(self):
        guards = []
        api = e.ExistingAPI(self.a, 'ram-key', lambda: guards.append(True))
        calls = []
        api.request = lambda path, payload=None, **kwargs: calls.append((path, payload)) or b'{}'
        api.append(17, ['2026-10-23T00:00:00Z message\n'])
        api.heartbeat()
        api.complete()
        self.assertEqual(len(calls), 1)
        self.assertEqual(set(calls[0][1]), {'files'})
        self.assertEqual(set(calls[0][1]['files']), {'output.log'})
        self.assertIn('/existing/file_stream', calls[0][0])
        self.assertEqual(len(guards), 1)
        with self.assertRaises(e.MirrorError):
            api.ensure_run()

    def test_authority_explicit_and_run_contract_read_only(self):
        p = self.root / 'authority.json'
        p.write_text(json.dumps(self.a))
        a, sha = e.authority(p)
        self.assertEqual(sha, hashlib.sha256(p.read_bytes()).hexdigest())
        api = e.ExistingAPI(a, 'ram-key', lambda: None)
        api.graphql = lambda *a, **kw: {'project': {'run': {'name': 'existing',
            'config': json.dumps({'contract': {'value': 'a' * 64}}), 'state': 'running'}}}
        api.check_run()
        self.a['authorized'] = False
        p.write_text(json.dumps(self.a))
        with self.assertRaises(e.MirrorError):
            e.authority(p)

    def test_guard_pid_console_off_and_dead_unknown_hold(self):
        proc = self.root / 'proc/123'
        proc.mkdir(parents=True)
        proc.joinpath('stat').write_text('123 (python) ' + ' '.join(['S'] + ['0'] * 18 + ['999']))
        proc.joinpath('environ').write_bytes(b'WANDB_CONSOLE=off\0WANDB_API_KEY=must-not-print\0')
        Path(self.a['guard_entry_receipt']).write_text(json.dumps({'pid': 123, 'pid_start_ticks': 999,
            'authority_sha256': 'b' * 64, 'contract_sha256': 'a' * 64, 'wandb_console': 'off',
            'event': 'ENTRY_VALIDATED', 'complete': True}))
        original_path = Path
        def mapped(value):
            return self.root / 'proc' if value == '/proc' else original_path(value)
        guard = e.WriterGuard(self.a, 'bound')
        with patch.object(e, 'Path', mapped):
            guard()
            proc.joinpath('environ').write_bytes(b'WANDB_CONSOLE=wrap\0')
            with self.assertRaises(e.MirrorError):
                guard()
            proc.joinpath('stat').unlink()
            with self.assertRaises(e.MirrorError):
                guard()
            self.status.write_text(json.dumps({'state': 'failed'}))
            guard()

    def test_guard_distinct_console_and_storage_authority_hashes(self):
        p = self.root / 'authority.json'
        p.write_text(json.dumps(self.a))
        a, console_sha = e.authority(p)
        self.assertNotEqual(console_sha, 'b' * 64)
        Path(a['guard_entry_receipt']).write_text(json.dumps({'pid': 999999999, 'pid_start_ticks': 1,
            'authority_sha256': 'b' * 64, 'contract_sha256': 'a' * 64,
            'wandb_console': 'off', 'event': 'ENTRY_VALIDATED', 'complete': True}))
        self.status.write_text(json.dumps({'state': 'failed'}))
        e.WriterGuard(a, console_sha)()
        p.write_text(p.read_text() + '\n')
        with self.assertRaises(e.MirrorError):
            e.WriterGuard(a, console_sha)()


if __name__ == '__main__':
    unittest.main()
