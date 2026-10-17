"""CPU-only fixtures: never SSH, ROS, MORAI, collect, upload or delete RAW."""
import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import uuid
from unittest.mock import Mock, patch

import finish_collection as f


class FinishCollectionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.home = Path(self.temp.name)
        workspace = self.home / f.live.WORKSPACE
        mr = self.home / 'MoraiLauncher_Lin'
        workspace.mkdir(mode=0o700)
        mr.mkdir(mode=0o700)
        self.ctx = dict(workspace=workspace, mr=mr, session=workspace / 'work/ros-session-v2',
                        graph=dict(manifest=dict(source=str(f.LOGICAL_REPO /
                          'artifacts/evaluation/base5_unit_release_261017/'
                          'close_primary_fullsource_v1/candidate_v158a/manifest.json'))))
        self.p = f.paths(self.ctx)

    def put(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        path.write_text(json.dumps(value) + '\n')
        path.chmod(0o600)
        return path

    def complete(self):
        status = dict(stage='complete', requested=5000, finished=5000,
                      accepted_candidate=5000, review=0, failed=0)
        quota = dict(target=5000, accepted=5000, deficit=0, pending=0, cells=[])
        self.put(self.p['collection'] / 'status.json', status)
        self.put(self.p['collection'] / 'quota.json', quota)
        self.put(self.p['selected'], dict(fixture=True))
        return status, quota

    def test_terminal_exact_zero_deficit_only(self):
        status, quota = self.complete()
        proof = f.terminal(self.p['collection'])
        self.assertEqual(set(proof), {'status_sha256', 'quota_sha256', 'selected_sha256'})
        for changes in (dict(stage='paused'), dict(accepted_candidate=4999),
                        dict(review=1), dict(failed=1), dict(finished=4999), dict(requested=4999)):
            with self.subTest(changes=changes):
                self.put(self.p['collection'] / 'status.json', dict(status, **changes))
                with self.assertRaisesRegex(ValueError, 'complete 5000'):
                    f.terminal(self.p['collection'])
        self.put(self.p['collection'] / 'status.json', status)
        for changes in (dict(deficit=1), dict(pending=1), dict(accepted=4999),
                        dict(target=4999), dict(cells=[dict(id='missing')])):
            with self.subTest(changes=changes):
                self.put(self.p['collection'] / 'quota.json', dict(quota, **changes))
                with self.assertRaisesRegex(ValueError, 'Unfilled quota'):
                    f.terminal(self.p['collection'])

    def test_default_plan_no_terminal_requirement_no_writes_or_subprocess(self):
        before = sorted(self.ctx['workspace'].rglob('*'))
        out = io.StringIO()
        with patch.object(f.live, 'load', return_value=self.ctx), \
                patch.object(f.live, 'ready'), patch.object(f.live, 'verify_qualification'), \
                patch.object(f.subprocess, 'run', side_effect=AssertionError('No process in plan')), \
                contextlib.redirect_stdout(out):
            self.assertEqual(f.main(['--workspace', str(self.ctx['workspace'])]), 0)
        doc = json.loads(out.getvalue())
        self.assertFalse(doc['files_written'])
        self.assertFalse(doc['automatic_invocation'])
        self.assertFalse(doc['full_5k_finish_live_qualified'])
        self.assertEqual(sorted(self.ctx['workspace'].rglob('*')), before)

    def test_save_preserves_conflicts_and_exact_retries(self):
        path = self.ctx['workspace'] / 'new.json'
        f.save_unchanged(path, dict(a=1))
        before = path.stat()
        f.save_unchanged(path, dict(a=1))
        self.assertEqual(path.stat().st_mtime_ns, before.st_mtime_ns)
        with self.assertRaisesRegex(ValueError, 'preserve'):
            f.save_unchanged(path, dict(a=2))
        self.assertEqual(json.loads(path.read_text()), dict(a=1))

    def test_meter_partitions_only_known_morai_aliases(self):
        workspace, mr = self.ctx['workspace'], self.ctx['mr']
        for name in ('ros-home', 'ros-home-v2'):
            home = workspace / 'work' / name
            home.mkdir(parents=True, mode=0o700)
            alias = home / 'MoraiLauncher_Lin'
            alias.symlink_to(mr, target_is_directory=True)
            self.assertEqual(alias.lstat().st_blocks, 0)
            self.put(home / '.ros/test.json', dict(log=True))
        self.put(workspace / 'capture_bundle/manifest.json', dict(frozen=True))
        self.put(workspace / 'MORAI_START.txt', dict(operator_text=True))
        roots = f.meter_roots(self.ctx)
        self.assertIn(mr, roots)
        self.assertIn(workspace / 'capture_bundle', roots)
        self.assertNotIn(workspace, roots)
        self.assertTrue(all(not root.is_symlink() for root in roots))
        self.assertEqual(sum(root.name == '.ros' for root in roots), 2)
        alias = workspace / 'work/ros-home-v2/MoraiLauncher_Lin'
        alias.unlink()
        alias.symlink_to(workspace, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'Unexpected'):
            f.meter_roots(self.ctx)

    def test_meter_partitions_real_ros_latest_and_reserves_allocated_blocks(self):
        for name in ('ros-session', 'ros-session-v2'):
            log = self.ctx['workspace'] / 'work' / name / 'log'
            target = log / str(uuid.uuid4())
            target.mkdir(parents=True, mode=0o700)
            self.put(target / 'roslaunch.log', dict(log=True))
            (log / 'latest').symlink_to(target, target_is_directory=True)
        exclusions = []
        roots = f.meter_roots(self.ctx, exclusions)
        self.assertEqual(len(exclusions), 2)
        for item in exclusions:
            alias = Path(item['path'])
            self.assertEqual(item['kind'], 'owned_ros_latest_uuid_pointer')
            self.assertEqual(item['allocated_bytes'], alias.lstat().st_blocks * 512)
            self.assertIn(Path(item['target']), roots)
            self.assertNotIn(alias.parent, roots)
        finisher = f.load_finisher()
        # The actual existing Budget can scan every partition successfully.
        expected = finisher.allocated_bytes(roots)
        self.assertGreater(expected, 0)

    def test_meter_rejects_escaping_non_uuid_or_linked_latest_target(self):
        log = self.ctx['workspace'] / 'work/ros-session-v2/log'
        log.mkdir(parents=True, mode=0o700)
        for label, target in [('escape', self.ctx['mr']), ('non_uuid', log / 'not_a_uuid')]:
            with self.subTest(label=label):
                target.mkdir(exist_ok=True)
                alias = log / 'latest'
                alias.symlink_to(target, target_is_directory=True)
                with self.assertRaisesRegex(ValueError, 'same log folder'):
                    f.meter_roots(self.ctx)
                alias.unlink()
        target = log / str(uuid.uuid4())
        target.symlink_to(self.ctx['mr'], target_is_directory=True)
        (log / 'latest').symlink_to(target, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'same log folder|Unexpected symlink'):
            f.meter_roots(self.ctx)

    def test_meter_rejects_unknown_symlink_on_partitioned_branch(self):
        unknown = self.ctx['workspace'] / 'outside'
        unknown.symlink_to('/tmp', target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'Unexpected'):
            f.meter_roots(self.ctx)

    def test_inner_requires_explicit_namespaced_contract(self):
        path = self.put(self.ctx['workspace'] / 'intent.json', dict(a=1))
        with patch.object(f.subprocess, 'run', side_effect=AssertionError('No process')):
            with self.assertRaisesRegex(ValueError, 'namespaced'):
                f.main(['--inner-intent', str(path), '--intent-sha256', f.live.digest(path), '--run'])

    def test_run_rejects_live_capture_lock_without_namespace(self):
        import fcntl
        self.ctx['session'].mkdir(parents=True, mode=0o700)
        lock = self.ctx['session'] / 'capture.lock'
        lock.touch(mode=0o600)
        finisher = f.load_finisher()
        with lock.open('a') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.object(f.live, 'load', return_value=self.ctx), \
                    patch.object(f.live, 'ready'), patch.object(f.live, 'verify_qualification'), \
                    patch.object(f.subprocess, 'run', side_effect=AssertionError('No child')), \
                    self.assertRaises(BlockingIOError):
                f.main(['--workspace', str(self.ctx['workspace']), '--run'])
        self.assertFalse(self.p['session'].exists())

    def test_inner_rejects_path_scope_change_before_source_or_native(self):
        document = dict(schema=f.SCHEMA, expected_scenes=5000,
                        manifest_sha256=f.live.DECK_SHA, release_sha256=f.live.RELEASE_SHA,
                        workspace=str(self.ctx['workspace']), morai=str(self.ctx['mr']),
                        paths={key: str(value) for key, value in self.p.items()})
        document['paths']['output'] = '/tmp/unauthorized'
        with self.assertRaisesRegex(ValueError, 'scope'):
            f.finish_inside(document, self.p['session'] / 'intent.json')

    def test_native_raw_scenario_adoption_and_existing_finisher_arguments(self):
        self.complete()
        self.p['session'].mkdir(parents=True, mode=0o700)
        source = dict(count=5000, scenes=[dict(id='one', scenario_sha256='original')])
        source_path = self.p['source']
        document = dict(schema=f.SCHEMA, expected_scenes=5000,
            manifest_sha256=f.live.DECK_SHA, release_sha256=f.live.RELEASE_SHA,
            workspace=str(self.ctx['workspace']), morai=str(self.ctx['mr']),
            paths={key: str(value) for key, value in self.p.items()},
            helper_sha256='helper', finisher_sha256='finisher', live_helper_sha256='live',
            budget_alias_exclusions=[],
            native_parent=f.terminal(self.p['collection']))
        rows = [dict(source_cell_id='one', scenario_sha256='native')]
        adoption = Mock(return_value=dict(id='one', scenario_sha256='native'))
        native = SimpleNamespace(verify_parent=Mock(return_value=(rows, [], 12.)), adopted_entry=adoption)
        campaign = SimpleNamespace(subset_deck=Mock(return_value=dict(count=5000, scenes=[{}] * 5000)))
        finisher = SimpleNamespace(ROOT=None, SCRIPTS=None,
            selected_rows=Mock(return_value=rows), finish=Mock(return_value=dict(stage='complete')),
            parser=Mock(return_value=SimpleNamespace(parse_args=lambda values: values)))
        real_digest = f.live.digest
        def digest(path):
            path = Path(path)
            if path == Path(f.__file__): return 'helper'
            if path == Path(f.__file__).with_name('finish.py'): return 'finisher'
            if path == Path(f.live.__file__): return 'live'
            if path == source_path: return f.live.DECK_SHA
            return real_digest(path)
        real_read = f.live.read
        def read(path):
            return source if Path(path) == source_path else real_read(path)
        with patch.object(f.live, 'digest', side_effect=digest), \
                patch.object(f.live, 'read', side_effect=read), \
                patch.object(f, 'load_finisher', return_value=finisher), \
                patch.dict('sys.modules', {'run_base5_fresh_completion': native,
                                          'run_base5_capture_campaign': campaign}):
            result = f.finish_inside(document, self.p['session'] / 'intent.json')
        self.assertEqual(result['stage'], 'complete')
        adoption.assert_called_once_with(rows[0], {'one': source['scenes'][0]})
        command = finisher.finish.call_args.args[0]
        self.assertIn('--run', command)
        self.assertIn('--max-usage-gib', command)
        self.assertIn('--min-free-gib', command)
        self.assertNotIn('--complete', command)
        self.assertNotIn('--delete', command)
        self.assertEqual(json.loads((self.p['session'] / 'adopted_manifest.json').read_text())['count'], 5000)


if __name__ == '__main__':
    unittest.main()
