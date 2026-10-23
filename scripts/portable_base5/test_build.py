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
    def morai_fixture(self, root):
        for name in build.CORE_FILES:
            file = root / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(b'fixture')
        for name in build.CORE_DIRS:
            directory = root / build.DATA / name
            directory.mkdir(parents=True, exist_ok=True)
            (directory / 'core.bin').write_bytes(b'core')
            (directory / 'unity.lock').write_bytes(b'lock')
            (directory / 'debug.debug').write_bytes(b'excluded debug')
            (directory / 'runtime.log').write_bytes(b'excluded log')
        for name in (*build.SIMULATOR_INSTALL_FILES, 'Simulator.x86_64', 'UnityPlayer.so',
                     'Simulator_Data/Managed/core.dll', 'Simulator_Data/Resources/core.bin',
                     'Simulator_Data/Plugins/core.so'):
            file = root / build.SIMULATOR_ROOT / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(('real native fixture: ' + name).encode())

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

    def test_clean_morai_preserves_native_install_files_but_excludes_private_and_junk(self):
        with tempfile.TemporaryDirectory() as root:
            mr = Path(root)
            self.morai_fixture(mr)
            account = mr / build.DATA / 'Version/V2AccountInfo.byte'
            account.write_bytes(b'not copied')
            options = account.with_name('optionList.byte')
            options.write_bytes(b'not copied')
            capture = mr / build.DATA / 'SaveFile/SensorData/old.png'
            capture.parent.mkdir(parents=True)
            capture.write_bytes(b'old')
            other_debug = mr / build.SIMULATOR_ROOT / 'Other_DoNotShip/private.txt'
            other_debug.parent.mkdir()
            other_debug.write_bytes(b'not a required native install file')
            names = [file.relative_to(mr).as_posix() for file in build.morai_files(mr)]
            self.assertNotIn(str(account.relative_to(mr)), names)
            self.assertNotIn(str(options.relative_to(mr)), names)
            self.assertNotIn(str(capture.relative_to(mr)), names)
            self.assertNotIn(str(other_debug.relative_to(mr)), names)
            self.assertFalse(any(name.endswith(('debug.debug', 'unity.lock', '.log')) for name in names))
            for native in build.SIMULATOR_INSTALL_FILES:
                self.assertIn((build.SIMULATOR_ROOT / native).as_posix(), names)

    def test_install_files_survive_copy_and_inventory_without_fake_markers(self):
        with tempfile.TemporaryDirectory() as root:
            mr, target = Path(root) / 'source', Path(root) / 'new_bundle/morai/MoraiLauncher_Lin'
            mr.mkdir()
            self.morai_fixture(mr)
            for source in build.morai_files(mr):
                build.copy_file(source, target / source.relative_to(mr))
            inventory = build.inventory(target)
            simulator = target / build.SIMULATOR_ROOT
            self.assertGreater(len(list(simulator.glob('*.*'))), 2)
            self.assertGreater(sum(path.is_dir() for path in simulator.iterdir()), 1)
            self.assertGreater(sum(path.is_dir() for path in (simulator / 'Simulator_Data').iterdir()), 2)
            for native in build.SIMULATOR_INSTALL_FILES:
                relative = build.SIMULATOR_ROOT / native
                self.assertEqual(build.sha256(mr / relative), inventory[relative.as_posix()]['sha256'])

    def test_missing_real_install_file_is_not_replaced_with_placeholder(self):
        with tempfile.TemporaryDirectory() as root:
            mr = Path(root)
            self.morai_fixture(mr)
            (mr / build.SIMULATOR_ROOT / build.SIMULATOR_INSTALL_FILES[0]).unlink()
            with self.assertRaisesRegex(ValueError, 'regular nonsymlink file'):
                build.morai_files(mr)

    def test_empty_directories_cannot_pass_file_only_archive_install_check(self):
        with tempfile.TemporaryDirectory() as root:
            mr = Path(root)
            self.morai_fixture(mr)
            (mr / build.SIMULATOR_ROOT / 'Simulator_Data/Plugins/core.so').unlink()
            with self.assertRaisesRegex(ValueError, 'install recognition'):
                build.morai_files(mr)

    def test_debug_exception_is_restricted_to_exact_native_paths(self):
        with tempfile.TemporaryDirectory() as root:
            for name in ('LinuxPlayer_s.debug', 'UnityPlayer_s.debug',
                         str(build.SIMULATOR_ROOT / 'unrelated.debug')):
                path = Path(root) / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b'not allowlisted')
                with self.subTest(name=name), self.assertRaises(ValueError):
                    build.safe_file(path)

    def test_capture_presets_are_identical_in_flat_and_native_version_namespaces(self):
        with tempfile.TemporaryDirectory() as root:
            repository, save = Path(root) / 'repo', Path(root) / 'SaveFile'
            for name in (build.SENSOR, build.NETWORK):
                source = repository / name
                source.parent.mkdir(parents=True, exist_ok=True)
                source.write_bytes(('fixture preset: ' + name).encode())
            build.copy_capture_presets(repository, save)
            for name, kind in ((build.SENSOR, 'Sensor'), (build.NETWORK, 'Network')):
                expected = (repository / name).read_bytes()
                self.assertEqual((save / kind / Path(name).name).read_bytes(), expected)
                self.assertEqual((save / kind / '25.S4.MolitComp03' / Path(name).name).read_bytes(), expected)
            self.assertEqual(len(build.inventory(save)), 4)
            # Never silently replace any existing user preset on a retry.
            with self.assertRaisesRegex(ValueError, 'existing destination'):
                build.copy_capture_presets(repository, save)

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
