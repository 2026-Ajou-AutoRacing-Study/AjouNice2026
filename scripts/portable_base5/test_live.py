"""CPU-only borrowed-host execution safety checks; never contacts a ROS master."""
import contextlib
import io
import json
import os
from pathlib import Path
import signal
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import live


class LiveTests(unittest.TestCase):
    def fixture(self, root):
        home = Path(root)
        workspace = home / live.WORKSPACE
        workspace.mkdir(mode=0o700)
        bundle = workspace / 'capture_bundle'
        bundle.mkdir(mode=0o700)
        mr = home / 'MoraiLauncher_Lin'
        mr.mkdir(mode=0o700)
        doc = dict(can_run=False, native_live_execution_enabled=False)
        graph = dict(release=dict(sha256=live.RELEASE_SHA,
                     source='/home/luv/AjouNice2026/release.json'),
                     manifest=dict(sha256=live.DECK_SHA,
                     source='/home/luv/AjouNice2026/manifest.json'), files={},
                     bindings=[dict(logical_root=value, bundle_path='payload' + value)
                               for value in ('/home/luv/AjouNice2026',
                                             '/home/luv/MoraiLauncher_Lin',
                                             '/home/luv/morai_cap')])
        for kind, name in [('Sensor', 'Sensor_competition_local.json'),
                           ('Network', 'ego_network_mr_base5_savesensordata_collision_local.json')]:
            for path in (bundle / 'morai/MoraiLauncher_Lin' / live.MR_SAVE / kind / name,
                         mr / live.MR_SAVE / kind / '25.S4.MolitComp03' / name):
                path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                path.write_text('{}')
                path.chmod(0o600)
        (bundle / 'portable_bundle.json').write_text(json.dumps(doc))
        (bundle / 'portable_graph.json').write_text(json.dumps(graph))
        return dict(workspace=workspace, bundle=bundle, doc=doc, graph=graph, mr=mr,
                    session=workspace / 'work/ros-session', home=workspace / 'work/ros-home')

    def prepared(self, root):
        ctx = self.fixture(root)
        live.prepare(ctx)
        return ctx

    def invoke(self, ctx, args):
        output = io.StringIO()
        with patch.object(live, 'load', return_value=ctx), contextlib.redirect_stdout(output):
            result = live.main(['--workspace', str(ctx['workspace']), *args])
        return result, output.getvalue()

    def test_owned_refuses_symlinks_in_path(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            target = root / 'target'
            target.mkdir(mode=0o700)
            alias = root / 'alias'
            alias.symlink_to(target, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                live.owned(alias)
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                live.no_symlinks(alias / 'missing')

    def test_owned_refuses_writeable_and_foreign_paths(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            root.chmod(0o777)
            with self.assertRaisesRegex(ValueError, 'writable'):
                live.owned(root)
            root.chmod(0o700)
            with patch.object(live.os, 'getuid', return_value=-1):
                with self.assertRaisesRegex(ValueError, 'Foreign'):
                    live.owned(root)

    def test_load_keeps_original_preparation_only_flags(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            manifest = ctx['bundle'] / 'portable_bundle.json'
            before = manifest.read_bytes()
            with patch.object(Path, 'home', return_value=Path(folder)), \
                    patch.object(live, 'MANIFEST_SHA', live.digest(manifest)):
                result = live.load(ctx['workspace'], verify=False)
            self.assertEqual(result['doc']['can_run'], False)
            self.assertEqual(result['session'], ctx['workspace'] / 'work/ros-session-v2')
            self.assertEqual(result['home'], ctx['workspace'] / 'work/ros-home-v2')
            self.assertEqual(before, manifest.read_bytes())
            self.assertFalse(ctx['session'].exists())

    def test_load_rejects_changed_bundle_and_enabled_flag(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            manifest = ctx['bundle'] / 'portable_bundle.json'
            with patch.object(Path, 'home', return_value=Path(folder)):
                with self.assertRaisesRegex(ValueError, 'manifest changed'):
                    live.load(ctx['workspace'], verify=False)
                manifest.write_text('{"can_run": true}')
                with patch.object(live, 'MANIFEST_SHA', live.digest(manifest)):
                    with self.assertRaisesRegex(ValueError, 'preparation-only'):
                        live.load(ctx['workspace'], verify=False)

    def test_load_rejects_changed_source_release_or_deck(self):
        for field in ('release', 'manifest'):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as folder:
                ctx = self.fixture(folder)
                ctx['graph'][field]['sha256'] = '0' * 64
                (ctx['bundle'] / 'portable_graph.json').write_text(json.dumps(ctx['graph']))
                with patch.object(Path, 'home', return_value=Path(folder)), \
                        patch.object(live, 'MANIFEST_SHA', live.digest(ctx['bundle'] / 'portable_bundle.json')):
                    with self.assertRaisesRegex(ValueError, 'release or source deck'):
                        live.load(ctx['workspace'], verify=False)

    def test_load_rejects_wrong_workspace_and_changed_ui_profile(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            with patch.object(Path, 'home', return_value=Path(folder) / 'other'):
                with self.assertRaisesRegex(ValueError, 'approved workspace'):
                    live.load(ctx['workspace'], verify=False)
            profile = ctx['mr'] / live.MR_SAVE / 'Sensor/25.S4.MolitComp03/Sensor_competition_local.json'
            profile.write_text('{"changed": true}')
            with patch.object(Path, 'home', return_value=Path(folder)), \
                    patch.object(live, 'MANIFEST_SHA', live.digest(ctx['bundle'] / 'portable_bundle.json')):
                with self.assertRaisesRegex(ValueError, 'UI profile changed'):
                    live.load(ctx['workspace'], verify=False)

    def test_prepare_is_idempotent_and_does_not_touch_txt_or_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            txt = ctx['workspace'] / 'MORAI_START.txt'
            txt.write_text('operator text\nDO NOT TOUCH BELOW\nprivate operator section\n')
            before = {path: path.read_bytes() for path in
                      (txt, ctx['bundle'] / 'portable_bundle.json', ctx['bundle'] / 'portable_graph.json')}
            first = live.prepare(ctx)
            self.assertEqual(first, live.prepare(ctx))
            live.ready(ctx)
            self.assertEqual((ctx['home'] / 'MoraiLauncher_Lin').resolve(), ctx['mr'])
            for path, content in before.items():
                self.assertEqual(path.read_bytes(), content)
            self.assertFalse(first['bulk_started'])
            self.assertEqual(first['maximum_smoke_captures'], 3)

    def test_prepare_preserves_conflicting_alias_and_contract(self):
        for conflict in ('alias', 'contract'):
            with self.subTest(conflict=conflict), tempfile.TemporaryDirectory() as folder:
                ctx = self.fixture(folder)
                ctx['home'].mkdir(parents=True, mode=0o700)
                if conflict == 'alias':
                    (ctx['home'] / 'MoraiLauncher_Lin').mkdir()
                else:
                    ctx['session'].mkdir(mode=0o700)
                    (ctx['session'] / 'host_contract.json').write_text('{}')
                with self.assertRaises(ValueError):
                    live.prepare(ctx)
                self.assertFalse((ctx['workspace'] / 'collections').exists())

    def test_prepare_refuses_parent_symlink_before_writing(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            outside = Path(folder) / 'outside'
            outside.mkdir(mode=0o700)
            (ctx['workspace'] / 'work').symlink_to(outside, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                live.prepare(ctx)
            self.assertEqual(list(outside.iterdir()), [])

    def test_ready_rejects_retargeted_alias(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            alias = ctx['home'] / 'MoraiLauncher_Lin'
            alias.unlink()
            alias.symlink_to(ctx['workspace'], target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'alias'):
                live.ready(ctx)

    def test_environment_isolated_loopback_and_private_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            with patch.dict(os.environ, {'SSH_AUTH_SOCK': '/secret/agent',
                                        'PYTHONPATH': '/untrusted', 'LD_PRELOAD': '/bad.so'}):
                env = live.environment(ctx)
            for field in ('HOME', 'ROS_HOME', 'ROS_LOG_DIR', 'TMPDIR', 'TMP', 'TEMP',
                          'XDG_CACHE_HOME', 'XDG_CONFIG_HOME', 'XDG_DATA_HOME'):
                Path(env[field]).relative_to(ctx['workspace'])
            for field in ('SSH_AUTH_SOCK', 'SSH_AGENT_PID', 'LD_PRELOAD', 'PYTHONHOME',
                          'CONDA_PREFIX', 'VIRTUAL_ENV', 'DISPLAY', 'XAUTHORITY'):
                self.assertNotIn(field, env)
            self.assertEqual(env['ROS_MASTER_URI'], 'http://127.0.0.1:11311')
            self.assertEqual(env['ROS_IP'], '127.0.0.1')
            self.assertEqual(env['ROS_HOSTNAME'], '127.0.0.1')
            self.assertNotIn('/untrusted', env['PYTHONPATH'])
            self.assertEqual(env['PYTHONDONTWRITEBYTECODE'], '1')

    def test_namespace_only_approved_two_rw_mounts_and_immutable_roots(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            command = live.namespace(ctx, ['/usr/bin/python3', '-B', '-c', 'pass'])
            pairs = [(command[i + 1], command[i + 2]) for i, value in enumerate(command)
                     if value == '--bind']
            self.assertEqual(pairs, [(str(ctx['workspace']), str(ctx['workspace'])),
                                     (str(ctx['mr']), str(ctx['mr']))])
            readonly = [(command[i + 1], command[i + 2]) for i, value in enumerate(command)
                        if value == '--ro-bind']
            self.assertIn(('/', '/'), readonly)
            self.assertIn((str(ctx['bundle']), str(ctx['bundle'])), readonly)
            for entry in ctx['graph']['bindings']:
                self.assertIn((str(ctx['bundle'] / entry['bundle_path']), entry['logical_root']), readonly)
            self.assertNotIn('--unshare-net', command)
            self.assertNotIn('--dev-bind', command)
            self.assertIn('--die-with-parent', command)
            self.assertIn('--new-session', command)
            self.assertEqual(command[-4:], ['/usr/bin/python3', '-B', '-c', 'pass'])

    def test_smoke_command_has_three_selected_slots_and_no_recovery_or_bulk(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            command = live.action_command(ctx, 'smoke')
            self.assertTrue(command[2].endswith('/run_base5_capture_optimized.py'))
            self.assertEqual(command[command.index('--indices') + 1], ','.join(map(str, live.SMOKE_INDICES)))
            self.assertEqual(command[command.index('--limit') + 1], '3')
            self.assertEqual(command[command.index('--manifest') + 1], ctx['graph']['manifest']['source'])
            self.assertNotIn('--approve-bulk-capture', command)
            self.assertNotIn('--complete', command)
            self.assertNotIn('--resume', command)

    def test_capture_is_explicit_original_release_without_automatic_completion(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            command = live.action_command(ctx, 'capture')
            self.assertEqual(command[command.index('--release') + 1], ctx['graph']['release']['source'])
            self.assertEqual(command[command.index('--output') + 1], str(ctx['workspace'] / 'collections/sunny13_5k'))
            self.assertIn('--run', command)
            self.assertIn('--confirmed-environment', command)
            self.assertNotIn('--complete', command)

    def test_network_commands_are_loopback_and_unknown_action_fails(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            self.assertEqual(live.action_command(ctx, 'ros'), ['/opt/ros/noetic/bin/roscore', '-p', '11311'])
            bridge = live.action_command(ctx, 'bridge')
            self.assertIn('_address:=127.0.0.1', bridge)
            self.assertIn('_port:=9090', bridge)
            with self.assertRaises(ValueError):
                live.action_command(ctx, 'unknown')

    def test_every_default_plan_is_readonly_without_process_or_network_probe(self):
        for action in ('prepare', 'ros', 'bridge', 'inspect', 'smoke', 'qualify', 'capture'):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as folder:
                ctx = self.fixture(folder)
                with patch.object(live, 'supervise') as process, patch.object(live, 'port_open') as probe:
                    result, output = self.invoke(ctx, [action])
                self.assertEqual(result, 0)
                report = json.loads(output)
                self.assertFalse(report['files_written'])
                self.assertFalse(report['capture_started'])
                process.assert_not_called()
                probe.assert_not_called()
                self.assertFalse(ctx['session'].exists())

    def test_check_action_never_starts_a_process_or_creates_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.fixture(folder)
            with patch.object(live, 'supervise') as process, patch.object(live, 'port_open', return_value=False):
                result, output = self.invoke(ctx, ['check'])
            self.assertEqual(result, 0)
            self.assertFalse(json.loads(output)['files_written'])
            self.assertFalse(ctx['session'].exists())
            process.assert_not_called()

    def test_ros_refuses_existing_master_before_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            with patch.object(live, 'port_open', return_value=True), patch.object(live, 'supervise') as process:
                with self.assertRaisesRegex(ValueError, 'Existing ROS master'):
                    self.invoke(ctx, ['ros', '--run'])
                process.assert_not_called()

    def test_bridge_refuses_existing_listener_before_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            with patch.object(live, 'port_open', return_value=True), \
                    patch.object(live, 'service_owned', return_value=True), \
                    patch.object(live, 'supervise') as process:
                with self.assertRaisesRegex(ValueError, 'Existing bridge'):
                    self.invoke(ctx, ['bridge', '--run'])
                process.assert_not_called()

    def test_capture_and_smoke_require_actual_environment_confirmation(self):
        for action in ('smoke', 'capture'):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as folder:
                ctx = self.prepared(folder)
                with patch.object(live, 'port_open', return_value=True), \
                        patch.object(live, 'service_owned', return_value=True), \
                        patch.object(live, 'supervise') as process:
                    with self.assertRaisesRegex(ValueError, 'Sunny13'):
                        self.invoke(ctx, [action, '--run'])
                    process.assert_not_called()
                self.assertFalse((ctx['session'] / 'smoke_attempt.json').exists())

    def test_bulk_requires_native_qualification_before_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            with patch.object(live, 'port_open', return_value=True), \
                    patch.object(live, 'service_owned', return_value=True), \
                    patch.object(live, 'supervise') as process:
                with self.assertRaises((OSError, ValueError)):
                    self.invoke(ctx, ['capture', '--run', '--confirmed-environment', 'sunny13'])
                process.assert_not_called()
            self.assertFalse((ctx['workspace'] / 'collections/sunny13_5k').exists())

    def test_write_new_never_overwrites_existing_or_follows_symlink(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / 'keep.json'
            path.write_text('operator content')
            with self.assertRaises(FileExistsError):
                live.write_new(path, {})
            self.assertEqual(path.read_text(), 'operator content')
            alias = root / 'alias.json'
            alias.symlink_to(path)
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                live.write_new(alias, {})
            self.assertEqual(path.read_text(), 'operator content')

    def test_qualification_rejects_incomplete_status_before_native_process(self):
        for status in ({}, dict(stage='complete', accepted_candidate=2, review=0, failed=0),
                       dict(stage='complete', accepted_candidate=3, review=1, failed=0),
                       dict(stage='complete', accepted_candidate=3, review=0, failed=1)):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as folder:
                ctx = self.prepared(folder)
                root = ctx['workspace'] / 'collections/smoke_v1'
                root.mkdir(mode=0o700)
                (root / 'status.json').write_text(json.dumps(status))
                with patch.object(live.subprocess, 'run') as process:
                    with self.assertRaises(ValueError):
                        live.qualify(ctx)
                    process.assert_not_called()
                self.assertFalse((ctx['session'] / 'qualification.json').exists())

    def test_qualification_rejects_mismatched_order_duplicate_or_reviewed_ledger(self):
        originals = [dict(id=ident, status='accepted_candidate') for ident in live.SMOKE_IDS]
        for rows in (originals[::-1], originals[:2], [originals[0]] * 3,
                     [dict(row, status='review') for row in originals]):
            with self.subTest(rows=rows), tempfile.TemporaryDirectory() as folder:
                ctx = self.prepared(folder)
                root = ctx['workspace'] / 'collections/smoke_v1'
                root.mkdir(mode=0o700)
                (root / 'status.json').write_text(json.dumps(dict(stage='complete', accepted_candidate=3,
                                                               review=0, failed=0)))
                (root / 'ledger.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
                with patch.object(live.subprocess, 'run') as process:
                    with self.assertRaises(ValueError):
                        live.qualify(ctx)
                    process.assert_not_called()

    def test_verify_qualification_rejects_modified_host_contract(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            record = dict(host_contract=dict(live.contract(ctx), executor_sha256='0' * 64), native_frames=3)
            (ctx['session'] / 'qualification.json').write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, 'host contract'):
                live.verify_qualification(ctx)

    def test_foreign_master_or_bridge_is_not_reused_by_other_roles(self):
        for role, ownership in [('bridge', False), ('inspect', False), ('smoke', False),
                                ('capture', False), ('inspect', True)]:
            with self.subTest(role=role, master_owned=ownership), tempfile.TemporaryDirectory() as folder:
                ctx = self.prepared(folder)
                def service_owned(context, name):
                    return ownership if name == 'ros' else False
                with patch.object(live, 'port_open', return_value=True), \
                        patch.object(live, 'service_owned', side_effect=service_owned), \
                        patch.object(live, 'supervise') as process:
                    with self.assertRaisesRegex(ValueError, 'not owned'):
                        self.invoke(ctx, [role, '--run', '--confirmed-environment', 'sunny13'])
                    process.assert_not_called()
                self.assertFalse((ctx['session'] / 'smoke_attempt.json').exists())

    def test_service_ownership_refuses_absent_symlink_and_unlocked_markers(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            self.assertFalse(live.service_owned(ctx, 'ros'))
            marker = ctx['session'] / 'ros.lock'
            outside = Path(folder) / 'outside.lock'
            outside.write_text('operator content')
            marker.symlink_to(outside)
            self.assertFalse(live.service_owned(ctx, 'ros'))
            marker.unlink()
            marker.write_text('')
            self.assertFalse(live.service_owned(ctx, 'ros'))
            self.assertEqual(outside.read_text(), 'operator content')

    def test_service_ownership_checks_executor_command_and_pid_generation(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            lock_path = ctx['session'] / 'ros.lock'
            lock_path.write_text('')
            record_path = ctx['session'] / 'ros_1.json'
            start = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19]
            record = dict(supervisor_pid=os.getpid(), action='ros',
                          executor_sha256=live.digest(live.__file__), pid_start_ticks=start)
            good_argv = b'\x00'.join([b'/usr/bin/python3', os.fsencode(str(Path(live.__file__).resolve())),
                                      b'--workspace', os.fsencode(ctx['workspace']), b'ros', b'--run', b''])
            original_read_bytes = Path.read_bytes
            def read_bytes(path):
                if str(path) == '/proc/' + str(os.getpid()) + '/cmdline':
                    return good_argv
                return original_read_bytes(path)
            with lock_path.open('a') as lock, patch.object(Path, 'read_bytes', read_bytes):
                live.fcntl.flock(lock, live.fcntl.LOCK_EX | live.fcntl.LOCK_NB)
                self.assertFalse(live.service_owned(ctx, 'ros'))
                record_path.write_text(json.dumps(record))
                self.assertTrue(live.service_owned(ctx, 'ros'))
                for change in (dict(pid_start_ticks='0'), dict(action='bridge'),
                               dict(executor_sha256='0' * 64)):
                    record_path.write_text(json.dumps(dict(record, **change)))
                    self.assertFalse(live.service_owned(ctx, 'ros'))

    def test_supervise_smoke_is_once_only_and_reservation_does_not_start_bulk(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            child = Mock(pid=123456)
            child.wait.return_value = 0
            child.poll.return_value = 0
            with patch.object(live.subprocess, 'Popen', return_value=child) as process:
                result = live.supervise(ctx, 'smoke', live.action_command(ctx, 'smoke'))
                self.assertEqual(result, 0)
                process.assert_called_once()
                budget = json.loads((ctx['session'] / 'smoke_attempt.json').read_text())
                self.assertEqual(budget['maximum_captures'], 3)
                self.assertEqual(budget['indices'], list(live.SMOKE_INDICES))
                self.assertFalse((ctx['workspace'] / 'collections/sunny13_5k').exists())
                with self.assertRaisesRegex(ValueError, 'already reserved'):
                    live.supervise(ctx, 'smoke', live.action_command(ctx, 'smoke'))
                process.assert_called_once()

    def test_smoke_busy_lock_fails_before_reserving_budget(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            with (ctx['session'] / 'smoke.lock').open('a') as lock:
                os.fchmod(lock.fileno(), 0o600)
                live.fcntl.flock(lock, live.fcntl.LOCK_EX | live.fcntl.LOCK_NB)
                with patch.object(live.subprocess, 'Popen') as process:
                    with self.assertRaises(BlockingIOError):
                        live.supervise(ctx, 'smoke', live.action_command(ctx, 'smoke'))
                    process.assert_not_called()
            self.assertFalse((ctx['session'] / 'smoke_attempt.json').exists())

    def test_smoke_bad_namespace_fails_before_reserving_budget(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            with patch.object(live, 'namespace', side_effect=ValueError('bad mount plan')), \
                    patch.object(live.subprocess, 'Popen') as process:
                with self.assertRaisesRegex(ValueError, 'bad mount'):
                    live.supervise(ctx, 'smoke', live.action_command(ctx, 'smoke'))
                process.assert_not_called()
            self.assertFalse((ctx['session'] / 'smoke_attempt.json').exists())

    def test_supervisor_rejects_symlinked_lock_before_child_or_external_write(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            outside = Path(folder) / 'outside.lock'
            outside.write_text('user data')
            (ctx['session'] / 'smoke.lock').symlink_to(outside)
            with patch.object(live.subprocess, 'Popen') as process:
                with self.assertRaises(ValueError):
                    live.supervise(ctx, 'smoke', live.action_command(ctx, 'smoke'))
                process.assert_not_called()
            self.assertEqual(outside.read_text(), 'user data')
            self.assertFalse((ctx['session'] / 'smoke_attempt.json').exists())

    def qualification_fixture(self, ctx):
        root = ctx['workspace'] / 'collections/smoke_v1'
        root.mkdir(mode=0o700)
        (root / 'status.json').write_text(json.dumps(dict(stage='complete', accepted_candidate=3,
                                                       review=0, failed=0)))
        rows = [dict(id=ident, status='accepted_candidate') for ident in live.SMOKE_IDS]
        (root / 'ledger.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
        (root / 'shutdown_safety.json').write_text('{"park_verified": true}')
        native = root / 'captures/native.bin'
        native.parent.mkdir(mode=0o700)
        native.write_bytes(b'fixture-only native bytes; not an actual sensor capture')
        return root, native

    def test_qualification_invokes_native_reconstruction_and_pins_every_payload_file(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            root, native = self.qualification_fixture(ctx)
            def validate_subprocess(command, **kwargs):
                code = command[-1]
                compile(code, '<native qualification plan>', 'exec')
                for marker in ('prepare_session', 'audit_capture', 'verified_shutdown',
                               'between_scene_park', 'stationary_support_observed', '*_Instance.png'):
                    self.assertIn(marker, code)
                self.assertTrue(kwargs['check'])
                self.assertLessEqual(kwargs['timeout'], 120)
                return Mock(returncode=0)
            with patch.object(live.subprocess, 'run', side_effect=validate_subprocess) as process:
                proof = live.qualify(ctx)
            process.assert_called_once()
            self.assertEqual(proof['native_frames'], 3)
            self.assertFalse(proof['bulk_started'])
            self.assertEqual(proof['native_files'][str(native.relative_to(root))], live.digest(native))
            live.verify_qualification(ctx)
            native.write_bytes(b'modified sensor payload')
            with self.assertRaisesRegex(ValueError, 'payload changed'):
                live.verify_qualification(ctx)

    def test_qualification_preserves_missing_native_proof_as_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            self.qualification_fixture(ctx)
            failure = live.subprocess.CalledProcessError(1, ['fixture-only'])
            with patch.object(live.subprocess, 'run', side_effect=failure):
                with self.assertRaises(live.subprocess.CalledProcessError):
                    live.qualify(ctx)
            self.assertFalse((ctx['session'] / 'qualification.json').exists())

    def test_qualification_rejects_evidence_symlink_even_after_native_probe(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            root, native = self.qualification_fixture(ctx)
            (root / 'outside-alias').symlink_to(native)
            with patch.object(live.subprocess, 'run', return_value=Mock(returncode=0)):
                with self.assertRaisesRegex(ValueError, 'Symlink'):
                    live.qualify(ctx)
            self.assertFalse((ctx['session'] / 'qualification.json').exists())

    def test_application_argv_exactness_ignores_bwrap_and_extra_arguments(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py', '--limit', '3']
        encoded = [os.fsencode(value) for value in command]
        self.assertTrue(live.application_argv(encoded + [b''], command))
        for argv in ([b'bwrap', b'--', *encoded], [*encoded, b'--extra'],
                     encoded[:-1], [b'/other/python3', *encoded[1:]]):
            self.assertFalse(live.application_argv(argv, command))
        ros = ['/opt/ros/noetic/bin/roscore', '-p', '11311']
        self.assertTrue(live.application_argv([b'/usr/bin/python3', *map(os.fsencode, ros)], ros))
        self.assertFalse(live.application_argv([b'/bin/sh', *map(os.fsencode, ros)], ros))
        self.assertFalse(live.application_argv([b'bwrap', *map(os.fsencode, ros)], ros))

    def proc_path_factory(self, rows):
        """A read-only /proc fixture; no process is actually spawned or signalled."""
        real_path = Path
        class ProcPath:
            def __init__(self, parts=()):
                self.parts = parts
            def __truediv__(self, component):
                return ProcPath(self.parts + tuple(str(component).split('/')))
            def row(self):
                pid = int(self.parts[0])
                if pid not in rows:
                    raise FileNotFoundError('fixture process has exited')
                return rows[pid]
            def stat(self):
                return SimpleNamespace(st_uid=self.row().get('uid', os.getuid()))
            def read_text(self):
                row = self.row()
                if self.parts[-1] == 'children':
                    return ' '.join(map(str, row.get('children', [])))
                if self.parts[-1] == 'stat':
                    fields = ['S'] + ['0'] * 19
                    fields[19] = str(row.get('start', 17))
                    return str(self.parts[0]) + ' (fixture process) ' + ' '.join(fields)
                raise AssertionError('Unexpected fixture proc read: ' + str(self.parts))
            def read_bytes(self):
                row = self.row()
                value = row.get('argv', [])
                if callable(value):
                    value = value()
                return b'\x00'.join([os.fsencode(item) for item in value] + [b''])
        def path(value):
            return ProcPath() if value == '/proc' else real_path(value)
        return path

    def test_inner_signal_targets_exact_owned_application_not_namespace_process(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        rows = {100: dict(children=[101, 999], argv=['bwrap', '--', *command]),
                101: dict(children=[102], argv=['bwrap', '--', *command]),
                102: dict(children=[], argv=command)}
        with patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                patch.object(live.os, 'pidfd_open', return_value=777, create=True) as open_pid, \
                patch.object(live.signal, 'pidfd_send_signal', create=True) as send, \
                patch.object(live.os, 'close') as close, patch.object(live.os, 'killpg') as broad:
            self.assertTrue(live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT))
        open_pid.assert_called_once_with(102)
        send.assert_called_once_with(777, signal.SIGINT)
        close.assert_called_once_with(777)
        broad.assert_not_called()

    def test_inner_signal_does_not_guess_missing_application(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        rows = {100: dict(children=[], argv=['bwrap', '--', *command])}
        with patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                patch.object(live.os, 'pidfd_open', create=True) as open_pid, \
                patch.object(live.signal, 'pidfd_send_signal', create=True) as send, \
                patch.object(live.os, 'killpg') as broad:
            self.assertFalse(live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT))
        open_pid.assert_not_called()
        send.assert_not_called()
        broad.assert_not_called()

    def test_inner_signal_refuses_foreign_or_ambiguous_descendants(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        for foreign in (False, True):
            rows = {100: dict(children=[101, 102], argv=['bwrap']),
                    101: dict(argv=command), 102: dict(argv=command)}
            if foreign:
                rows[101]['uid'] = -1
            with self.subTest(foreign=foreign), \
                    patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                    patch.object(live.os, 'pidfd_open', create=True) as open_pid, \
                    patch.object(live.signal, 'pidfd_send_signal', create=True) as send:
                with self.assertRaises(ValueError):
                    live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT)
                open_pid.assert_not_called()
                send.assert_not_called()

    def test_inner_signal_requires_pidfd_without_unsafe_fallback(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        rows = {100: dict(children=[101], argv=['bwrap']), 101: dict(argv=command)}
        original_hasattr = hasattr
        def guarded_hasattr(obj, name):
            if name in ('pidfd_open', 'pidfd_send_signal'):
                return False
            return original_hasattr(obj, name)
        with patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                patch('builtins.hasattr', side_effect=guarded_hasattr), \
                patch.object(live.os, 'killpg') as broad:
            with self.assertRaisesRegex(ValueError, 'pidfd'):
                live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT)
        broad.assert_not_called()

    def test_inner_signal_rechecks_identity_after_pidfd_and_closes_on_error(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        argv = Mock(side_effect=[command, ['/usr/bin/python3', '/foreign.py']])
        rows = {100: dict(children=[101], argv=['bwrap']), 101: dict(argv=argv)}
        with patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                patch.object(live.os, 'pidfd_open', return_value=777, create=True), \
                patch.object(live.signal, 'pidfd_send_signal', create=True) as send, \
                patch.object(live.os, 'close') as close:
            with self.assertRaisesRegex(ValueError, 'identity changed'):
                live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT)
        send.assert_not_called()
        close.assert_called_once_with(777)

    def test_inner_signal_process_exit_closes_pidfd_without_broad_retry(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        rows = {100: dict(children=[101], argv=['bwrap']), 101: dict(argv=command)}
        with patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                patch.object(live.os, 'pidfd_open', return_value=777, create=True), \
                patch.object(live.signal, 'pidfd_send_signal', side_effect=ProcessLookupError, create=True), \
                patch.object(live.os, 'close') as close, patch.object(live.os, 'killpg') as broad:
            self.assertFalse(live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT))
        close.assert_called_once_with(777)
        broad.assert_not_called()

    def test_inner_signal_rejects_pid_reuse_or_owner_change_after_open(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        for changed in (dict(start=18), dict(uid=-1)):
            rows = {100: dict(children=[101], argv=['bwrap']), 101: dict(argv=command, start=17)}
            def opened(pid):
                rows[pid].update(changed)
                return 777
            with self.subTest(change=changed), \
                    patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                    patch.object(live.os, 'pidfd_open', side_effect=opened, create=True), \
                    patch.object(live.signal, 'pidfd_send_signal', create=True) as send, \
                    patch.object(live.os, 'close') as close:
                with self.assertRaisesRegex(ValueError, 'identity changed'):
                    live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT)
                send.assert_not_called()
                close.assert_called_once_with(777)

    def test_inner_signal_exit_before_pidfd_open_is_not_an_unsafe_retry(self):
        command = ['/usr/bin/python3', '-B', '/trusted/capture.py']
        rows = {100: dict(children=[101], argv=['bwrap']), 101: dict(argv=command)}
        with patch.object(live, 'Path', side_effect=self.proc_path_factory(rows)), \
                patch.object(live.os, 'pidfd_open', side_effect=ProcessLookupError, create=True), \
                patch.object(live.signal, 'pidfd_send_signal', create=True) as send, \
                patch.object(live.os, 'close') as close, patch.object(live.os, 'killpg') as broad:
            self.assertFalse(live.signal_inner_app(SimpleNamespace(pid=100), command, signal.SIGINT))
        send.assert_not_called()
        close.assert_not_called()
        broad.assert_not_called()

    def test_pause_request_is_scoped_idempotent_and_does_not_remove_existing_marker(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            marker = live.request_pause(ctx)
            self.assertEqual(marker, ctx['workspace'] / 'collections/sunny13_5k/PAUSE')
            before = marker.read_bytes()
            self.assertEqual(live.request_pause(ctx), marker)
            self.assertEqual(marker.read_bytes(), before)
            self.assertIn('current bounded chunk', json.loads(before)['semantics'])

    def test_pause_request_rejects_symlink_to_external_file(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            root = ctx['workspace'] / 'collections/sunny13_5k'
            root.mkdir(mode=0o700)
            outside = Path(folder) / 'operator.txt'
            outside.write_text('preserve')
            (root / 'PAUSE').symlink_to(outside)
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                live.request_pause(ctx)
            self.assertEqual(outside.read_text(), 'preserve')

    def test_supervisor_signal_bulk_requests_pause_without_signalling_namespace(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            child = Mock(pid=123456)
            child.poll.return_value = None
            handlers = {}
            def register(signum, handler):
                previous = handlers.get(signum, signal.SIG_DFL)
                handlers[signum] = handler
                return previous
            def wait():
                handlers[signal.SIGHUP](signal.SIGHUP, None)
                child.poll.return_value = 0
                return 0
            child.wait.side_effect = wait
            with patch.object(live.subprocess, 'Popen', return_value=child), \
                    patch.object(live.signal, 'signal', side_effect=register), \
                    patch.object(live, 'signal_inner_app') as inner, \
                    patch.object(live.os, 'killpg') as broad, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(live.supervise(ctx, 'capture', live.action_command(ctx, 'capture')), 0)
            self.assertTrue((ctx['workspace'] / 'collections/sunny13_5k/PAUSE').is_file())
            inner.assert_not_called()
            broad.assert_not_called()

    def test_supervisor_signal_smoke_reaches_inner_app_not_bwrap(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx = self.prepared(folder)
            child = Mock(pid=123456)
            child.poll.return_value = None
            handlers = {}
            def register(signum, handler):
                previous = handlers.get(signum, signal.SIG_DFL)
                handlers[signum] = handler
                return previous
            def wait():
                handlers[signal.SIGHUP](signal.SIGHUP, None)
                child.poll.return_value = 0
                return 0
            child.wait.side_effect = wait
            command = live.action_command(ctx, 'smoke')
            with patch.object(live.subprocess, 'Popen', return_value=child), \
                    patch.object(live.signal, 'signal', side_effect=register), \
                    patch.object(live, 'signal_inner_app', return_value=True) as inner, \
                    patch.object(live.os, 'killpg') as broad:
                self.assertEqual(live.supervise(ctx, 'smoke', command), 0)
            inner.assert_called_once_with(child, command, signal.SIGINT)
            broad.assert_not_called()
            self.assertFalse((ctx['workspace'] / 'collections/sunny13_5k/PAUSE').exists())

    def test_port_probe_is_bounded_and_only_loopback(self):
        fake = Mock()
        fake.connect_ex.return_value = 0
        fake.__enter__ = Mock(return_value=fake)
        fake.__exit__ = Mock(return_value=False)
        with patch.object(live.socket, 'socket', return_value=fake):
            self.assertTrue(live.port_open(11311))
        fake.settimeout.assert_called_once_with(.4)
        fake.connect_ex.assert_called_once_with(('127.0.0.1', 11311))


if __name__ == '__main__':
    unittest.main()
