"""Local fixture-only ROS overlay assembly regressions; no ROS is imported."""
import json
from pathlib import Path
import tempfile
import unittest

import dependencies


class OverlayTests(unittest.TestCase):
    def fixture(self, root):
        repo, ros, docs = root / 'repo', root / 'ros', root / 'docs'
        generated = repo / 'base_ws/install_aw149/lib/python3/dist-packages/morai_msgs'
        source = repo / 'base_ws/src/morai/morai_msgs'
        for folder in (generated / 'msg', generated / 'srv', source / 'msg', source / 'srv'):
            folder.mkdir(parents=True)
        (generated / '__init__.py').write_text('')
        for kind in ('msg', 'srv'):
            (generated / kind / '__init__.py').write_text('')
        (source / 'package.xml').write_text('<package><name>morai_msgs</name><version>0.0.0</version><license>TODO</license></package>')
        for name in dependencies.MESSAGE_NAMES:
            definition = 'int32 value\n'
            (source / 'msg' / (name + '.msg')).write_text(definition)
            (generated / 'msg' / ('_' + name + '.py')).write_text(
                'class ' + name + ':\n    _md5sum = "' + 'a' * 32 + '"\n'
                '    _type = "morai_msgs/' + name + '"\n    _full_text = ' + repr(definition) + '\n')
        (source / 'srv/MoraiEventCmdSrv.srv').write_text('int32 value\n---\nint32 value\n')
        service = []
        for letter, name in zip('bcd', ('MoraiEventCmdSrvRequest', 'MoraiEventCmdSrvResponse', 'MoraiEventCmdSrv')):
            service.append('class ' + name + ':\n    _md5sum = "' + letter * 32 + '"\n'
                           '    _full_text = "int32 value\\n"\n')
        (generated / 'srv/_MoraiEventCmdSrv.py').write_text('\n'.join(service))
        for name in dependencies.PACKAGES:
            python = ros / 'lib/python3/dist-packages' / name
            python.mkdir(parents=True)
            (python / '__init__.py').write_text('# BSD source header retained\n')
            (python / '__pycache__').mkdir()
            (python / '__pycache__/ignored.cpython-38.pyc').write_bytes(b'not portable')
            (python / 'ignored.pyc').write_bytes(b'not portable')
            share = ros / 'share' / name
            share.mkdir(parents=True)
            (share / 'package.xml').write_text('<package><name>' + name + '</name><version>1.0</version><license>BSD</license></package>')
            copyright = docs / ('ros-noetic-' + name.replace('_', '-')) / 'copyright'
            copyright.parent.mkdir(parents=True)
            copyright.write_text('fixture copyright notice\n')
        executable = ros / 'lib/rosbridge_server/rosbridge_websocket'
        executable.parent.mkdir(parents=True)
        executable.write_text('#!/usr/bin/env python3\n# preserved upstream license\n')
        executable.chmod(0o755)
        return repo, ros, docs, generated, source

    def build(self, root, fixture):
        repo, ros, docs, _, _ = fixture
        return dependencies.assemble(root / 'overlay', repository=repo, ros_prefix=ros, doc_root=docs)

    def test_copy_inventory_wire_md5_and_no_generated_import(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            fixture = self.fixture(root)
            original = {str(path): dependencies.sha256(path) for folder in fixture[:3]
                        for path in folder.rglob('*') if path.is_file()}
            receipt = self.build(root, fixture)
            self.assertEqual(len(receipt['ros_md5']), 8)
            self.assertEqual(receipt['ros_md5']['ScenarioLoad'], 'a' * 32)
            self.assertFalse(receipt['network_used'])
            self.assertFalse(receipt['ros_started'])
            self.assertFalse(receipt['compiled_payload'])
            self.assertTrue(receipt['licensing_warnings'])
            for relative, item in receipt['files'].items():
                target = root / 'overlay' / relative
                self.assertFalse(target.is_symlink())
                self.assertEqual(dependencies.sha256(target), item['sha256'])
                self.assertEqual(target.stat().st_size, item['bytes'])
                self.assertNotIn('__pycache__', relative)
                self.assertFalse(relative.endswith('.pyc'))
            self.assertTrue((root / 'overlay/licenses/rosbridge_server/copyright').is_file())
            self.assertEqual((root / 'overlay/lib/rosbridge_server/rosbridge_websocket').stat().st_mode & 0o777, 0o755)
            self.assertEqual(json.loads((root / 'overlay/overlay_receipt.json').read_text()), receipt)
            self.assertEqual(original, {str(path): dependencies.sha256(path) for folder in fixture[:3]
                                      for path in folder.rglob('*') if path.is_file()})

    def test_existing_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            fixture = self.fixture(root)
            output = root / 'overlay'
            output.mkdir()
            marker = output / 'user.txt'
            marker.write_text('preserve')
            with self.assertRaises(FileExistsError):
                self.build(root, fixture)
            self.assertEqual(marker.read_text(), 'preserve')

    def test_dependency_symlink_is_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            fixture = self.fixture(root)
            target = fixture[1] / 'lib/python3/dist-packages/rosbridge_library/leak.py'
            target.symlink_to(fixture[3] / '__init__.py')
            with self.assertRaisesRegex(ValueError, 'Symlink'):
                self.build(root, fixture)
            self.assertFalse((root / 'overlay').exists())

    def test_binary_and_credentials_are_rejected_before_output_creation(self):
        for name in ('compiled.so', '.env', 'client_secret.json'):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as t:
                root = Path(t)
                fixture = self.fixture(root)
                (fixture[1] / 'lib/python3/dist-packages/rosbridge_library' / name).write_text('untrusted')
                with self.assertRaises(ValueError):
                    self.build(root, fixture)
                self.assertFalse((root / 'overlay').exists())

    def test_generated_source_message_drift_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            fixture = self.fixture(root)
            (fixture[4] / 'msg/ScenarioLoad.msg').write_text('string changed\n')
            with self.assertRaisesRegex(ValueError, 'definitions differ'):
                self.build(root, fixture)
            self.assertFalse((root / 'overlay').exists())

    def test_generated_service_drift_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            fixture = self.fixture(root)
            (fixture[4] / 'srv/MoraiEventCmdSrv.srv').write_text('int32 value\n---\nstring wrong\n')
            with self.assertRaisesRegex(ValueError, 'service definitions differ'):
                self.build(root, fixture)

    def test_invalid_md5_and_ambiguous_class_are_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            path = root / 'generated.py'
            path.write_text('class A:\n    _md5sum = "not-md5"\n')
            with self.assertRaisesRegex(ValueError, 'wire MD5'):
                dependencies.generated_attributes(path, 'A')
            path.write_text('class A: pass\nclass A: pass\n')
            with self.assertRaisesRegex(ValueError, 'ambiguous'):
                dependencies.generated_attributes(path, 'A')

    def test_missing_copyright_is_explicit_and_not_invented(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            fixture = self.fixture(root)
            (fixture[2] / 'ros-noetic-rosauth/copyright').unlink()
            receipt = self.build(root, fixture)
            self.assertTrue(any('rosauth:' in text for text in receipt['licensing_warnings']))
            self.assertFalse((root / 'overlay/licenses/rosauth/copyright').exists())
            self.assertTrue((root / 'overlay/licenses/rosauth/package.xml').exists())

    def test_output_cannot_be_placed_inside_source(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            repo, ros, docs, generated, _ = self.fixture(root)
            with self.assertRaisesRegex(ValueError, 'overlaps'):
                dependencies.assemble(generated / 'new', repository=repo, ros_prefix=ros, doc_root=docs)
            self.assertFalse((generated / 'new').exists())

    def test_builder_wrapper_uses_explicit_install_and_source_roots(self):
        with tempfile.TemporaryDirectory() as t:
            root = Path(t)
            repo, ros, _, _, source = self.fixture(root)
            result = dependencies.assemble_overlay(root / 'overlay', repo / 'base_ws/install_aw149', ros, source)
            self.assertEqual(len(result['ros_md5']), 8)
            self.assertEqual(result['packages']['morai_msgs']['name'], 'morai_msgs')


if __name__ == '__main__':
    unittest.main()
