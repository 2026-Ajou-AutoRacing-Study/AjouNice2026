"""CPU-only safety regressions; no ROS imports, network or simulator activity."""
import contextlib
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import runtime


class RuntimeTests(unittest.TestCase):
    def fixture(self, root):
        repo = root / 'payload/home/luv/AjouNice2026'
        repo.mkdir(parents=True)
        release = repo / 'release.json'
        release.write_text('{"bulk_ready": true}')
        (root / 'overlay').mkdir()
        (root / 'python').mkdir()
        doc = dict(schema=runtime.SCHEMA, can_run=False,
                   native_live_execution_enabled=False,
                   files={release.relative_to(root).as_posix(): runtime.sha256(release)},
                   runtime=dict(repository=repo.relative_to(root).as_posix(),
                                release=release.relative_to(root).as_posix(),
                                ros_overlay='overlay', python_overlay='python'))
        (root / runtime.MANIFEST).write_text(json.dumps(doc))
        return doc

    def approval(self, root, *, actions=('activate', 'bridge', 'capture')):
        scenario, sensor = root / 'mr-scenarios', root / 'mr-sensors'
        scenario.mkdir(exist_ok=True)
        sensor.mkdir(exist_ok=True)
        record = dict(schema=runtime.APPROVAL_SCHEMA, approved=True,
                      bundle_sha256=runtime.sha256(root / runtime.MANIFEST),
                      actions=list(actions), environment='sunny13',
                      scenario_directory=str(scenario), sensor_directory=str(sensor))
        path = root / 'approval.json'
        path.write_text(json.dumps(record))
        return path, record

    def test_inventory_check_does_not_create_runtime_home(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            self.assertEqual(runtime.load_bundle(root), (root, doc))
            with contextlib.redirect_stdout(io.StringIO()), patch.object(runtime.subprocess, 'run') as child:
                self.assertEqual(runtime.main(['--bundle', str(root), 'check', '--inventory-only']), 0)
                child.assert_not_called()
            self.assertFalse((root / 'runtime-home').exists())

    def test_unsafe_paths_are_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            for value in ('', '.', '..', '../x', '/absolute', 'a/../b', 'a//b', 'a\\b', 'a/'):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    runtime.inside(root, value)
            (root / 'alias').symlink_to(root, target_is_directory=True)
            with self.assertRaises(ValueError):
                runtime.inside(root, 'alias/file')

    def test_changed_input_and_unpinned_release_refused(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            release = root / doc['runtime']['release']
            original = release.read_bytes()
            release.write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'missing or changed'):
                runtime.load_bundle(root)
            release.write_bytes(original)
            other = root / 'uncovered.json'
            other.write_text('{}')
            doc['runtime']['release'] = 'uncovered.json'
            (root / runtime.MANIFEST).write_text(json.dumps(doc))
            with self.assertRaisesRegex(ValueError, 'covered by inventory'):
                runtime.load_bundle(root)

    def test_environment_is_bundle_local_and_does_not_forward_agent(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            inherited = dict(HOME='/private', PYTHONPATH='/untrusted', PYTHONHOME='/bad',
                             SSH_AUTH_SOCK='/agent', SSH_AGENT_PID='123',
                             VIRTUAL_ENV='/venv', CONDA_PREFIX='/conda', LD_PRELOAD='/hook',
                             ROS_MASTER_URI='http://outside:11311')
            env = runtime.child_environment(root, doc, inherited)
            self.assertEqual(env['HOME'], str(root / 'runtime-home'))
            self.assertEqual(env['ROS_MASTER_URI'], 'http://127.0.0.1:11311')
            self.assertEqual(env['PYTHONDONTWRITEBYTECODE'], '1')
            self.assertNotIn('/untrusted', env['PYTHONPATH'])
            for key in ('PYTHONHOME', 'SSH_AUTH_SOCK', 'SSH_AGENT_PID', 'VIRTUAL_ENV', 'CONDA_PREFIX', 'LD_PRELOAD'):
                self.assertNotIn(key, env)
            self.assertEqual(inherited['HOME'], '/private')
            self.assertFalse((root / 'runtime-home').exists())

    def test_plan_never_executes_or_creates_home(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            self.fixture(root)
            for action in ('bridge', 'capture'):
                out = io.StringIO()
                with contextlib.redirect_stdout(out), patch.object(runtime.subprocess, 'call') as call:
                    self.assertEqual(runtime.main(['--bundle', str(root), action]), 0)
                    call.assert_not_called()
                report = json.loads(out.getvalue())
                self.assertFalse(report['can_run'])
                self.assertTrue(report['requires_original_namespace'])
                self.assertFalse(report['simulator_contacted'])
            self.assertFalse((root / 'runtime-home').exists())

    def test_activation_requires_approval_and_matching_environment(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            path, record = self.approval(root)
            with self.assertRaises(ValueError):
                runtime.activate(root, doc, None, 'sunny13')
            with self.assertRaises(ValueError):
                runtime.activate(root, doc, path, 'foggy13')
            record['approved'] = False
            path.write_text(json.dumps(record))
            with self.assertRaises(ValueError):
                runtime.activate(root, doc, path, 'sunny13')
            self.assertFalse((root / 'runtime-home').exists())

    def test_activation_is_idempotent_and_never_writes_in_mr(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            path, record = self.approval(root)
            first = runtime.activate(root, doc, path, 'sunny13')
            second = runtime.activate(root, doc, path, 'sunny13')
            self.assertEqual(first, second)
            runtime.require_activation(root, record)
            self.assertEqual(list(Path(record['scenario_directory']).iterdir()), [])
            self.assertEqual(list(Path(record['sensor_directory']).iterdir()), [])

    def test_activation_preserves_existing_directory(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            path, _ = self.approval(root)
            existing = root / 'runtime-home' / runtime.SCENARIO_RELATIVE
            existing.mkdir(parents=True)
            marker = existing / 'user.json'
            marker.write_text('preserve')
            with self.assertRaisesRegex(ValueError, 'overwritten'):
                runtime.activate(root, doc, path, 'sunny13')
            self.assertEqual(marker.read_text(), 'preserve')
            self.assertFalse((root / 'runtime-home' / runtime.SENSOR_RELATIVE).exists())

    def test_activation_rejects_different_link_and_parent_symlinks(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            path, _ = self.approval(root)
            other = root / 'other'
            other.mkdir()
            home = root / 'runtime-home'
            home.symlink_to(other, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'HOME'):
                runtime.activate(root, doc, path, 'sunny13')
            home.unlink()
            link = home / runtime.SCENARIO_RELATIVE
            link.parent.mkdir(parents=True)
            link.symlink_to(other, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'differs'):
                runtime.activate(root, doc, path, 'sunny13')

    def test_approval_cannot_be_reused_for_changed_bundle(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            path, _ = self.approval(root)
            doc['new_field'] = 'changed'
            (root / runtime.MANIFEST).write_text(json.dumps(doc))
            with self.assertRaisesRegex(ValueError, 'does not authorize'):
                runtime.activate(root, doc, path, 'sunny13')

    def test_live_disabled_even_with_valid_approval(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            path, _ = self.approval(root)
            runtime.activate(root, doc, path, 'sunny13')
            with patch.object(runtime.subprocess, 'call') as call:
                for action in ('bridge', 'capture'):
                    with self.subTest(action=action), self.assertRaises(ValueError):
                        runtime.live_run(root, doc, action, path, 'sunny13')
                call.assert_not_called()
            # A hand-edited enable flag is still not an implemented capture path.
            doc['native_live_execution_enabled'] = True
            doc['can_run'] = True
            with self.assertRaisesRegex(ValueError, 'not implemented/qualified'):
                runtime.live_run(root, doc, 'capture', path, 'sunny13')

    def test_dependency_check_does_not_start_ros_and_pins_all_wire_types(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            doc = self.fixture(root)
            names = ('CtrlCmd', 'EgoVehicleStatus', 'ObjectStatusList', 'SaveSensorData',
                     'ScenarioLoad', 'CollisionData', 'MoraiEventCmdSrv', 'MoraiEventCmdSrvRequest')
            expected = {name: 'a' * 32 for name in names}
            doc['runtime']['ros_md5'] = expected
            child = SimpleNamespace(stdout=json.dumps(dict(ros_md5=expected, simulator_contacted=False)))
            with patch.object(runtime.subprocess, 'run', return_value=child) as run:
                runtime.check_dependencies(root, doc)
                command = run.call_args[0][0]
                self.assertNotIn('init_node(', command[3])
                self.assertNotIn('Service(', command[3])
                self.assertIn('-B', command)
                self.assertEqual(run.call_args[1]['env']['PYTHONDONTWRITEBYTECODE'], '1')
                doc['runtime']['ros_md5'] = {'CtrlCmd': 'a' * 32}
                with self.assertRaisesRegex(ValueError, 'wire identity'):
                    runtime.check_dependencies(root, doc)


if __name__ == '__main__':
    unittest.main()
