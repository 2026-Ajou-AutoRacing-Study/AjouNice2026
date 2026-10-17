import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import zipfile

import build
import namespace
import runtime


class BuildTests(unittest.TestCase):
    def test_credential_names_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            for name in ('.env', '.env.production', 'rclone.conf', '.netrc',
                         'client_secret_a.json', 'id_ed25519', 'a.pem', 'a.key'):
                file = Path(root) / name
                file.write_text('not an actual credential')
                with self.subTest(name=name), self.assertRaises(ValueError):
                    build.safe_file(file)

    def test_copy_preserves_bytes_without_source_changes(self):
        with tempfile.TemporaryDirectory() as root:
            source, target = Path(root) / 'input.json', Path(root) / 'new/a.json'
            source.write_bytes(b'{"path":"/original/absolute/path"}\n')
            before = build.sha256(source)
            build.copy_file(source, target)
            self.assertEqual(before, build.sha256(target))
            self.assertEqual(before, build.sha256(source))
            with self.assertRaises(ValueError):
                build.copy_file(source, target)

    def test_clean_morai_excludes_account_savefile_lock_and_debug(self):
        with tempfile.TemporaryDirectory() as root:
            mr = Path(root)
            for name in build.CORE_FILES:
                file = mr / name
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_bytes(b'fixture')
            for name in build.CORE_DIRS:
                directory = mr / build.DATA / name
                directory.mkdir(parents=True, exist_ok=True)
                (directory / 'core.bin').write_bytes(b'core')
                (directory / 'unity.lock').write_bytes(b'lock')
                (directory / 'debug.debug').write_bytes(b'debug')
            account = mr / build.DATA / 'Version/V2AccountInfo.byte'
            account.write_bytes(b'not copied')
            capture = mr / build.DATA / 'SaveFile/SensorData/old.png'
            capture.parent.mkdir(parents=True)
            capture.write_bytes(b'old')
            names = [file.relative_to(mr).as_posix() for file in build.morai_files(mr)]
            self.assertNotIn(str(account.relative_to(mr)), names)
            self.assertNotIn(str(capture.relative_to(mr)), names)
            self.assertFalse(any(name.endswith(('.debug', 'unity.lock')) for name in names))

    def test_wheel_library_extraction_and_no_script_generation(self):
        with tempfile.TemporaryDirectory() as root:
            wheel = Path(root) / 'demo.whl'
            with zipfile.ZipFile(wheel, 'w') as out:
                out.writestr('demo/__init__.py', 'ok=True')
                out.writestr('demo.data/purelib/other.py', 'yes=True')
                out.writestr('demo.data/scripts/run', 'should not run')
            target = Path(root) / 'python'
            build.expand_wheels([wheel], target)
            self.assertTrue((target / 'demo/__init__.py').is_file())
            self.assertTrue((target / 'other.py').is_file())
            self.assertFalse((target / 'demo.data/scripts/run').exists())

    def test_wheel_path_traversal_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            wheel = Path(root) / 'evil.whl'
            with zipfile.ZipFile(wheel, 'w') as out:
                out.writestr('../outside', 'bad')
            with self.assertRaises(ValueError):
                build.expand_wheels([wheel], Path(root) / 'python')
            self.assertFalse((Path(root) / 'outside').exists())

    def test_namespace_has_only_readonly_bindings_and_fixed_probe(self):
        with tempfile.TemporaryDirectory() as root:
            bundle = Path(root)
            (bundle / 'bin').mkdir()
            (bundle / 'bin/bwrap').write_bytes(b'fixture')
            (bundle / 'portable_graph.json').write_text(json.dumps(dict(
                bindings=[dict(bundle_path='payload/home/luv/AjouNice2026',
                               logical_root='/home/luv/AjouNice2026')])))
            with mock.patch.object(namespace, 'verify_graph'):
                command = namespace.command_plan(bundle)
            self.assertNotIn('--bind', command)
            self.assertIn('--ro-bind', command)
            self.assertIn('--tmpfs', command)
            self.assertIn('capture_started', command[-1])
            self.assertNotIn('launch_base5_obstacle_support.py', command[-1])
            self.assertNotIn('SSH_AUTH_SOCK', namespace.clean_environment())

    def test_bridge_plan_uses_ros_private_parameter_not_unknown_cli_flag(self):
        with tempfile.TemporaryDirectory() as root:
            base = Path(root)
            (base / 'repo').mkdir()
            command = runtime.command_plan(base, {'runtime': {'repository': 'repo'}}, 'bridge')
            self.assertIn('_address:=127.0.0.1', command)
            self.assertNotIn('--address', command)


if __name__ == '__main__':
    unittest.main()
