"""Private LAN preparation changes addresses only; no ROS or live connections."""
import copy
import io
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from functools import partial
from pathlib import Path
from unittest.mock import patch

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[3]
sys.path.insert(0, str(PACKAGE / 'src'))

from planning_scenario_validator import lan_profiles


class LanProfileTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.workspace = self.root / lan_profiles.PACKAGE / 'config/missions/full_course/workspace'
        self.workspace.mkdir(parents=True)
        self.ignore = self.workspace / '.gitignore'
        self.ignore.write_bytes((PACKAGE / 'config/missions/full_course/workspace/.gitignore').read_bytes())
        self.ego_path = self.root / 'fixtures/ego.json'
        self.sensor_path = self.root / 'fixtures/sensor.json'
        self.ego_path.parent.mkdir()
        self.ego_path.write_bytes((PACKAGE / 'config/internal/generated/ego_network_validator_local.json').read_bytes())
        self.sensor_path.write_bytes((REPOSITORY / 'configs/sensors/Sensor_competition_local.json').read_bytes())
        self.ego = json.loads(self.ego_path.read_text())
        self.sensor = json.loads(self.sensor_path.read_text())
        self.source = self.workspace / 'private.yaml'
        self.text = '''# keep team notes, original bytes, and tuning
schema_version: 2
campaign_name: private_test
network:
  sensor_json: fixtures/sensor.json # keep sensor comment
  ego_network_json: fixtures/ego.json # keep ego comment
morai:
  scenario_load_host: 127.0.0.1 # keep route comment
  scenario_install_dir: /mounted/MORAI/Scenario/R_KR_PR_K-city_2025
stack:
  detector: bevfusion
  bevfusion_model: base5_e12
  pose_source: localization
  traffic_light: true
  traffic_light_runtime: system
default_case:
  goal: {x: 1.5, y: 2.5, z: 28.0, yaw_deg: 60.0}
  timeout_sec: 600
recording:
  bag_enabled: true
  video: {enabled: true, display: ':0.0'}
'''
        self.source.write_text(self.text)
        self.document = yaml.safe_load(self.text)
        self.originals = {p: p.read_bytes() for p in (self.source, self.ego_path, self.sensor_path)}

    def prepare(self, *, output=None, morai_ip='10.42.0.252', ros_ip='10.42.0.1'):
        return lan_profiles.prepare_lan_bundle(self.source, morai_ip, ros_ip,
                                               output=output, root=self.root)

    def assert_originals(self):
        for path, original in self.originals.items():
            self.assertEqual(path.read_bytes(), original, str(path))

    def scenario(self, ego=None):
        network = self.ego if ego is None else ego
        return next(entry for entry in network['egoNetworkData'][0]['listEgoNetworkInfo']
                    if entry['PUBSUB_TYPE'] == 783)

    def test_active_addresses_include_control_and_every_sensor_without_other_changes(self):
        before = copy.deepcopy((self.document, self.ego, self.sensor))
        document, ego, sensor, report = lan_profiles.patch_lan_addresses(
            self.document, self.ego, self.sensor, '10.42.0.252', '10.42.0.1')
        expected_document, expected_ego, expected_sensor = copy.deepcopy(before)
        expected_document['morai']['scenario_load_host'] = '10.42.0.252'
        control_rows = 0
        for entry in expected_ego['egoNetworkData'][0]['listEgoNetworkInfo']:
            if entry.get('netType') != 1:
                continue
            for key in ('config', 'EgoCtrlConfig'):
                row = entry.get(key, {})
                if row.get('commType') == 1 and isinstance(row.get('udpConfig'), dict):
                    row['udpConfig'].update(hostIP='10.42.0.252', destinationIP='10.42.0.1')
                    control_rows += key == 'EgoCtrlConfig'
        self.assertGreater(control_rows, 0)
        expected_counts = {'cameraList': 4, 'Lidar3DList': 1, 'GPSList': 1, 'IMUList': 1}
        for category, key in [('cameraList', 'cc'), ('Lidar3DList', 'lc'),
                              ('GPSList', 'gc'), ('IMUList', 'ic')]:
            self.assertEqual(len(expected_sensor[category]), expected_counts[category])
            for item in expected_sensor[category]:
                self.assertEqual(item[key]['commType'], 1)
                item[key]['udpConfig'].update(hostIP='10.42.0.252', destinationIP='10.42.0.1')
        self.assertEqual(document, expected_document)
        self.assertEqual(ego, expected_ego)
        self.assertEqual(sensor, expected_sensor)
        self.assertEqual((self.document, self.ego, self.sensor), before)
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['ros_destination_ips'], ['10.42.0.1'])
        self.assert_originals()

    def test_inactive_udp_non_udp_metadata_and_second_ego_are_unchanged(self):
        ego = copy.deepcopy(self.ego)
        sensor = copy.deepcopy(self.sensor)
        ego['egoNetworkData'].append(copy.deepcopy(ego['egoNetworkData'][0]))
        ego['notes'] = {'hostIP': 'not-active', 'destinationIP': 'not-active'}
        inactive_camera = copy.deepcopy(sensor['cameraList'][0])
        inactive_camera['cc']['commType'] = 0
        sensor['cameraList'].append(inactive_camera)
        before_ego, before_sensor = copy.deepcopy((ego, sensor))
        _, changed_ego, changed_sensor, _ = lan_profiles.patch_lan_addresses(
            self.document, ego, sensor, '10.42.0.252', '10.42.0.1')
        self.assertEqual(changed_ego['egoNetworkData'][1], before_ego['egoNetworkData'][1])
        self.assertEqual(changed_ego['notes'], before_ego['notes'])
        self.assertEqual(changed_sensor['cameraList'][-1], inactive_camera)
        for original, changed in zip(before_ego['egoNetworkData'][0]['listEgoNetworkInfo'],
                                     changed_ego['egoNetworkData'][0]['listEgoNetworkInfo']):
            for key in ('config', 'EgoCtrlConfig'):
                if original.get('netType') != 1 or original.get(key, {}).get('commType') != 1:
                    self.assertEqual(changed.get(key), original.get(key))
        self.assertEqual(ego, before_ego)
        self.assertEqual(sensor, before_sensor)

    def test_bundle_changes_only_three_yaml_values_and_preserves_comments_and_source_bytes(self):
        result = self.prepare()
        generated = yaml.safe_load(result['yaml'].read_text())
        expected = copy.deepcopy(self.document)
        expected['morai']['scenario_load_host'] = '10.42.0.252'
        expected['network']['sensor_json'] = str(result['sensor'].relative_to(self.root))
        expected['network']['ego_network_json'] = str(result['ego_network'].relative_to(self.root))
        self.assertEqual(generated, expected)
        text = result['yaml'].read_text()
        for comment in ('# keep team notes, original bytes, and tuning', '# keep sensor comment',
                        '# keep ego comment', '# keep route comment'):
            self.assertIn(comment, text)
        self.assert_originals()

    def test_default_output_is_sibling_and_under_gitignored_personal_workspace(self):
        result = self.prepare()
        self.assertEqual(result['yaml'], self.workspace / 'private_lan_dev.yaml')
        self.assertEqual(result['sensor'], self.workspace / 'private_lan_dev_network/sensor_lan_dev.json')
        self.assertEqual(result['ego_network'], self.workspace / 'private_lan_dev_network/ego_network_lan_dev.json')
        self.assertEqual(result['yaml'].parent, self.source.parent)
        self.assertEqual(self.ignore.read_bytes(),
                         (PACKAGE / 'config/missions/full_course/workspace/.gitignore').read_bytes())
        if shutil.which('git'):
            subprocess.run(['git', 'init', '-q', str(self.root)], check=True)
            files = [str(result[key].relative_to(self.root)) for key in ('yaml', 'sensor', 'ego_network')]
            ignored = subprocess.check_output(['git', '-C', str(self.root), 'check-ignore', '--no-index',
                                                *files], text=True).splitlines()
            self.assertEqual(set(ignored), set(files))

    def test_source_base_snapshot_outside_nested_and_symlink_escape_are_rejected(self):
        invalid = [self.root / lan_profiles.PACKAGE / 'config/missions/full_course/base/example.yaml',
                   self.root / 'planner_logs/validation/frozen/config_snapshot.yaml',
                   self.root / 'outside.yaml', self.workspace / 'nested/source.yaml',
                   self.workspace / 'config_snapshot.yaml', self.workspace / 'source_config.yaml']
        for path in invalid:
            with self.subTest(path=path), self.assertRaises(ValueError):
                lan_profiles.prepare_lan_bundle(path, '10.42.0.252', '10.42.0.1', root=self.root)
        outside = self.root / 'outside.yaml'
        outside.write_text(self.text)
        link = self.workspace / 'escape.yaml'
        link.symlink_to(outside)
        with self.assertRaises(ValueError):
            lan_profiles.prepare_lan_bundle(link, '10.42.0.252', '10.42.0.1', root=self.root)
        self.assert_originals()

    def test_output_base_snapshot_outside_other_mission_and_source_overwrite_are_rejected(self):
        invalid = [self.root / lan_profiles.PACKAGE / 'config/missions/full_course/base/output.yaml',
                   self.root / 'planner_logs/validation/frozen/config_snapshot.yaml',
                   self.root / 'outside.yaml', self.source,
                   self.workspace / 'config_snapshot.yaml', self.workspace / 'source_config.yaml',
                   self.root / lan_profiles.PACKAGE / 'config/missions/highway/workspace/output.yaml']
        for path in invalid:
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.prepare(output=path)
        self.assert_originals()
        self.assertFalse((self.workspace / 'private_lan_dev_network').exists())

    def test_invalid_non_unicast_and_same_ips_are_rejected_without_writes(self):
        for bad in ('bad', '::1', '127.0.0.1', '0.0.0.0', '224.0.0.1', '255.255.255.255'):
            for role in ('morai_ip', 'ros_ip'):
                with self.subTest(bad=bad, role=role), self.assertRaises(ValueError):
                    self.prepare(**{role: bad})
        with self.assertRaises(ValueError):
            self.prepare(morai_ip='10.42.0.1', ros_ip='10.42.0.1')
        self.assertFalse((self.workspace / 'private_lan_dev.yaml').exists())
        self.assertFalse((self.workspace / 'private_lan_dev_network').exists())
        self.assert_originals()

    def test_scenario_slot_must_be_unique_active_udp_9095_to_9096(self):
        mutations = [lambda e: self.scenario(e)['config']['udpConfig'].update(hostPort=7603),
                     lambda e: self.scenario(e)['config']['udpConfig'].update(destinationPort=7502),
                     lambda e: self.scenario(e).update(netType=2),
                     lambda e: self.scenario(e)['config'].update(commType=0),
                     lambda e: e['egoNetworkData'][0]['listEgoNetworkInfo'].append(copy.deepcopy(self.scenario(e)))]
        for index, mutation in enumerate(mutations):
            ego = copy.deepcopy(self.ego)
            mutation(ego)
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, '9095'):
                lan_profiles.patch_lan_addresses(self.document, ego, self.sensor,
                                                '10.42.0.252', '10.42.0.1')

    def test_no_active_sensor_is_rejected(self):
        empty = copy.deepcopy(self.sensor)
        for key in ('cameraList', 'Lidar3DList', 'GPSList', 'IMUList'):
            empty[key] = []
        with self.assertRaisesRegex(ValueError, 'Sensor'):
            lan_profiles.patch_lan_addresses(self.document, self.ego, empty,
                                            '10.42.0.252', '10.42.0.1')

    def test_idempotent_rerun_does_not_rewrite_generated_or_original_files(self):
        first = self.prepare()
        files = [first[key] for key in ('yaml', 'sensor', 'ego_network')]
        captured = {path: (path.read_bytes(), path.stat().st_mtime_ns) for path in files}
        second = self.prepare()
        self.assertEqual(second, first)
        for path, (content, stamp) in captured.items():
            self.assertEqual(path.read_bytes(), content)
            self.assertEqual(path.stat().st_mtime_ns, stamp)
        self.assert_originals()

    def test_differing_existing_destination_causes_no_partial_writes(self):
        first = self.prepare()
        files = {key: first[key] for key in ('yaml', 'sensor', 'ego_network')}
        for conflict in files:
            for path in files.values():
                path.unlink(missing_ok=True)
            files[conflict].parent.mkdir(parents=True, exist_ok=True)
            files[conflict].write_text('unrelated existing file\n')
            with self.subTest(conflict=conflict), self.assertRaisesRegex(ValueError, '덮어쓰지'):
                self.prepare()
            self.assertEqual(files[conflict].read_text(), 'unrelated existing file\n')
            for key, path in files.items():
                if key != conflict:
                    self.assertFalse(path.exists(), str(path))
        self.assert_originals()

    def test_network_directory_symlink_is_rejected_without_writing_its_target(self):
        outside = self.root / 'external_network'
        outside.mkdir()
        marker = outside / 'unrelated.txt'
        marker.write_text('preserve existing external content\n')
        directory = self.workspace / 'private_lan_dev_network'
        directory.symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, '실제 폴더'):
            self.prepare()
        self.assertEqual(list(outside.iterdir()), [marker])
        self.assertEqual(marker.read_text(), 'preserve existing external content\n')
        self.assertFalse((self.workspace / 'private_lan_dev.yaml').exists())
        self.assert_originals()

    def test_existing_identical_payload_symlink_is_rejected_before_any_other_write(self):
        first = self.prepare()
        paths = {key: first[key] for key in ('yaml', 'sensor', 'ego_network')}
        expected = {key: path.read_bytes() for key, path in paths.items()}
        for key in ('sensor', 'ego_network'):
            for path in paths.values():
                path.unlink(missing_ok=True)
            outside = self.root / ('external_' + key + '.json')
            outside.write_bytes(expected[key])
            paths[key].symlink_to(outside)
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, '덮어쓰지'):
                self.prepare()
            self.assertTrue(paths[key].is_symlink())
            self.assertEqual(outside.read_bytes(), expected[key])
            for other_key, path in paths.items():
                if other_key != key:
                    self.assertFalse(path.exists(), str(path))
        self.assert_originals()

    def test_explicit_sibling_output_name_is_respected(self):
        output = self.workspace / 'team_choice.yaml'
        result = self.prepare(output=output)
        self.assertEqual(result['yaml'], output)
        self.assertEqual(result['sensor'].parent.name, 'team_choice_network')
        self.assertEqual(result['ego_network'].parent.name, 'team_choice_network')
        self.assert_originals()

    def test_main_invalid_input_returns_two_without_live_actions_or_output(self):
        prepare = partial(lan_profiles.prepare_lan_bundle, root=self.root)
        captured = io.StringIO()
        with patch.object(lan_profiles, 'prepare_lan_bundle', prepare), redirect_stdout(captured):
            result = lan_profiles.main(['--yaml', str(self.source), '--morai-ip', 'bad',
                                        '--ros-ip', '10.42.0.1'])
        self.assertEqual(result, 2)
        self.assertIn('LAN 준비 실패', captured.getvalue())
        self.assertFalse((self.workspace / 'private_lan_dev.yaml').exists())
        self.assert_originals()

    def test_main_success_labels_development_only_and_keeps_previous_lan_files(self):
        previous = self.workspace / 'private_lan.yaml'
        previous.write_text('previous personal LAN file, do not migrate\n')
        prepare = partial(lan_profiles.prepare_lan_bundle, root=self.root)
        captured = io.StringIO()
        with patch.object(lan_profiles, 'prepare_lan_bundle', prepare), redirect_stdout(captured):
            result = lan_profiles.main(['--yaml', str(self.source), '--morai-ip', '10.42.0.252',
                                        '--ros-ip', '10.42.0.1'])
        self.assertEqual(result, 0)
        self.assertIn('개발용 임시 사본', captured.getvalue())
        self.assertIn('대회에서는 검증된 현장점검 실행 절차와 설정을 그대로', captured.getvalue())
        self.assertTrue((self.workspace / 'private_lan_dev.yaml').is_file())
        self.assertEqual(previous.read_text(), 'previous personal LAN file, do not migrate\n')
        self.assert_originals()


if __name__ == '__main__':
    unittest.main()
