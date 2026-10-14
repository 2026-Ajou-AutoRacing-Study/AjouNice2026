"""Shared localization evidence must contain execution-contracted input bytes."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from planning_scenario_validator import config, sharing


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class LocalizationShareIntegrityTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.output = self.root / 'execution'
        self.folder = self.output / 'success' / 'case'
        self.folder.mkdir(parents=True)
        self.source = self.write('base_ws/src/localization/fixture.py', 'value = 1\n')
        self.installed = self.write('install/lib/localization/fixture.py', 'value = 1\n')
        self.controller = self.write('configs/controller.yaml', 'control_tau: 0.5\n')
        self.sensor = self.write('configs/sensor.json', '{"cameraList": []}\n')
        self.network = self.write('configs/ego_network.json', '{"egoNetworkData": []}\n')
        self.write('execution/stack_logs/commands.json', '[{"command": ["roslaunch", "fixture"]}]')
        self.write('execution/localization_runtime.json', '{"fingerprint": "fixture"}')
        self.write('execution/campaign_config.yaml', 'campaign_name: fixture\n')
        self.write('execution/success/case/runtime_receipt.json', '{"stage": "control_ready"}')
        self.write('execution/success/case/safety_receipt.json', '{"shutdown": {"status": "VERIFIED"}}')
        self.write('execution/success/case/recording_audit.json', json.dumps({
            'status': 'OK', 'complete_for_analysis': True, 'analysis_kind': 'localization'}))
        (self.folder / 'fixture.bag').write_bytes(b'bag fixture')
        self.cfg = {'name': 'fixture', 'output': self.output,
            'network': {'sensor_json': self.sensor, 'ego_network_json': self.network},
            'upload': {'enabled': False, 'profile': 'bag'},
            'recording': {'bag_enabled': True, 'profile_preset': 'localization_analysis'},
            'model_contract': {'preset': 'od_gt'},
            'localization_contract': {'fingerprint': 'fixture',
                'install_root': str(self.root / 'install'),
                'config_files': {'controller': {'path': str(self.controller), 'sha256': digest(self.controller)}},
                'source_files_sha256': {str(self.source.relative_to(self.root)): digest(self.source)},
                'installed_files_sha256': {str(self.installed.relative_to(self.root / 'install')): digest(self.installed)}}}

    def write(self, relative, text):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        return path

    def bundle(self):
        with patch.object(config, 'ROOT', self.root):
            return sharing.build_bundle(self.cfg, self.folder)

    def test_safe_copy_expected_hash_checks_original_bytes_before_sanitization(self):
        source = self.write('settings.yaml', 'password: do-not-share\nvalue: 0.5\n')
        expected = digest(source)
        target = self.root / 'payload/settings.yaml'
        self.assertTrue(sharing.safe_copy(source, target, expected_sha256=expected))
        self.assertNotIn('do-not-share', target.read_text())
        self.assertIn('REDACTED', target.read_text())
        self.assertEqual(digest(source), expected)
        self.assertNotEqual(digest(target), expected)

    def test_safe_copy_wrong_hash_refuses_changed_bytes_without_creating_payload_file(self):
        expected = digest(self.controller)
        self.controller.write_text('control_tau: 9.0\n')
        target = self.root / 'payload/incorrect_controller.yaml'
        self.assertFalse(sharing.safe_copy(self.controller, target, expected_sha256=expected))
        self.assertFalse(target.exists())

    def test_unchanged_execution_inputs_keep_shared_analysis_available(self):
        _, target, manifest = self.bundle()
        self.assertTrue(manifest['analysis_available_in_share'], manifest['warnings'])
        self.assertEqual(manifest['warnings'], [])
        self.assertTrue((target / 'configs/configs/controller.yaml').is_file())
        self.assertTrue((target / 'code/source/base_ws/src/localization/fixture.py').is_file())
        self.assertTrue((target / 'code/installed/lib/localization/fixture.py').is_file())

    def test_changed_controller_config_cannot_be_shared_as_execution_provenance(self):
        self.controller.write_text('control_tau: 9.0\n')
        _, target, manifest = self.bundle()
        self.assertFalse(manifest['analysis_available_in_share'])
        self.assertTrue(manifest['analysis_available_locally'])
        self.assertFalse((target / 'configs/configs/controller.yaml').exists())
        self.assertTrue(any('localization config hash/파일 불일치: controller' in warning
                            for warning in manifest['warnings']))
        self.assertTrue((target / 'fixture.bag').is_file())

    def test_changed_source_code_cannot_be_shared_as_execution_provenance(self):
        self.source.write_text('value = 999\n')
        _, target, manifest = self.bundle()
        self.assertFalse(manifest['analysis_available_in_share'])
        self.assertFalse((target / 'code/source/base_ws/src/localization/fixture.py').exists())
        self.assertTrue(any('source hash/파일 불일치:' in warning for warning in manifest['warnings']))
        self.assertTrue((target / 'code/installed/lib/localization/fixture.py').is_file())

    def test_changed_installed_code_cannot_be_shared_as_execution_provenance(self):
        self.installed.write_text('value = 999\n')
        _, target, manifest = self.bundle()
        self.assertFalse(manifest['analysis_available_in_share'])
        self.assertFalse((target / 'code/installed/lib/localization/fixture.py').exists())
        self.assertTrue(any('installed hash/파일 불일치:' in warning for warning in manifest['warnings']))
        self.assertTrue((target / 'code/source/base_ws/src/localization/fixture.py').is_file())


if __name__ == '__main__':
    unittest.main()
