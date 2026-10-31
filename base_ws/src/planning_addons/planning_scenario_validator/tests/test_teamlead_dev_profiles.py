"""Checked-in team-lead LAN development profiles are IP-only derived copies."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[3]
sys.path.insert(0, str(PACKAGE / 'src'))

from planning_scenario_validator.lan_profiles import patch_lan_addresses
from planning_scenario_validator.network_contract import (
    _active_network_udp, _active_sensor_udp, inspect_network_contract)


class TeamleadDevProfileTests(unittest.TestCase):
    def setUp(self):
        self.sensor_source = REPOSITORY / 'configs/sensors/Sensor_competition_local.json'
        self.ego_source = PACKAGE / 'config/internal/generated/ego_network_validator_local.json'
        folder = PACKAGE / 'config/internal/network_profiles/teamlead_lan_dev'
        self.sensor_path = folder / 'sensor_lan_dev.json'
        self.ego_path = folder / 'ego_network_lan_dev.json'
        self.original_bytes = {path: path.read_bytes() for path in
                               (self.sensor_source, self.ego_source, self.sensor_path, self.ego_path)}
        self.sensor_source_data = json.loads(self.original_bytes[self.sensor_source])
        self.ego_source_data = json.loads(self.original_bytes[self.ego_source])
        self.sensor = json.loads(self.original_bytes[self.sensor_path])
        self.ego = json.loads(self.original_bytes[self.ego_path])
        self.document = {'morai': {'scenario_load_host': '10.42.0.252',
                                   'scenario_install_dir': '/mounted/MORAI/Scenario'}}

    def assert_read_only(self):
        for path, content in self.original_bytes.items():
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                             hashlib.sha256(content).hexdigest(), str(path))

    def test_dev_files_are_exact_fresh_semantic_derivations_from_local_sources(self):
        before_sensor = copy.deepcopy(self.sensor_source_data)
        before_ego = copy.deepcopy(self.ego_source_data)
        _, expected_ego, expected_sensor, report = patch_lan_addresses(
            self.document, self.ego_source_data, self.sensor_source_data,
            '10.42.0.252', '10.42.0.1')
        self.assertEqual(self.sensor, expected_sensor)
        self.assertEqual(self.ego, expected_ego)
        self.assertEqual(report['errors'], [])
        self.assertEqual(self.sensor_source_data, before_sensor)
        self.assertEqual(self.ego_source_data, before_ego)
        self.assert_read_only()

    def test_all_active_udp_including_control_and_seven_sensors_have_expected_ips(self):
        network_rows = list(_active_network_udp(self.ego))
        self.assertTrue(any(key == 'EgoCtrlConfig' for _, key, _ in network_rows))
        sensors = list(_active_sensor_udp(self.sensor))
        self.assertEqual(len(sensors), 7)
        for category, size in [('cameraList', 4), ('Lidar3DList', 1), ('GPSList', 1), ('IMUList', 1)]:
            self.assertEqual(len(self.sensor[category]), size)
        for row in [udp for _, _, udp in network_rows] + sensors:
            self.assertEqual(row['hostIP'], '10.42.0.252')
            self.assertEqual(row['destinationIP'], '10.42.0.1')
        self.assert_read_only()

    def test_every_non_ip_field_and_all_inactive_settings_match_the_sources(self):
        # Reconstruct independently instead of masking every IP leaf, so an
        # accidental update of inactive blocks/metadata would also fail.
        expected_ego = copy.deepcopy(self.ego_source_data)
        expected_sensor = copy.deepcopy(self.sensor_source_data)
        for entry in expected_ego['egoNetworkData'][0]['listEgoNetworkInfo']:
            if entry.get('netType') != 1:
                continue
            for key in ('config', 'EgoCtrlConfig'):
                config = entry.get(key, {})
                if config.get('commType') == 1 and isinstance(config.get('udpConfig'), dict):
                    config['udpConfig'].update(hostIP='10.42.0.252', destinationIP='10.42.0.1')
        for category, key in [('cameraList', 'cc'), ('Lidar3DList', 'lc'),
                              ('GPSList', 'gc'), ('IMUList', 'ic')]:
            for item in expected_sensor[category]:
                config = item.get(key, item)
                if config.get('commType') == 1 and isinstance(config.get('udpConfig'), dict):
                    config['udpConfig'].update(hostIP='10.42.0.252', destinationIP='10.42.0.1')
        self.assertEqual(self.ego, expected_ego)
        self.assertEqual(self.sensor, expected_sensor)
        self.assert_read_only()

    def test_scenario_slot_is_unique_and_static_contract_is_consistent(self):
        entries = self.ego['egoNetworkData'][0]['listEgoNetworkInfo']
        scenarios = [entry for entry in entries if entry.get('PUBSUB_TYPE') == 783]
        self.assertEqual(len(scenarios), 1)
        self.assertEqual(scenarios[0]['netType'], 1)
        self.assertEqual(scenarios[0]['config']['commType'], 1)
        udp = scenarios[0]['config']['udpConfig']
        self.assertEqual(udp['hostPort'], 9095)
        self.assertEqual(udp['destinationPort'], 9096)
        report = inspect_network_contract(self.document, self.ego, self.sensor)
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['warnings'], [])
        self.assertEqual(report['ros_destination_ips'], ['10.42.0.1'])
        self.assert_read_only()

    @unittest.skipUnless(shutil.which('git'), 'git is required to inspect ignore contracts')
    def test_shared_dev_pair_is_trackable_but_private_workspace_remains_ignored(self):
        relative = [str(path.relative_to(REPOSITORY)) for path in (self.sensor_path, self.ego_path)]
        shared = subprocess.run(['git', '-C', str(REPOSITORY), 'check-ignore', '--no-index', *relative],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.assertEqual(shared.returncode, 1, shared.stdout + shared.stderr)
        self.assertEqual(shared.stdout, '')
        private = str((PACKAGE / 'config/missions/full_course/workspace/teamlead_lan_dev_test.yaml')
                      .relative_to(REPOSITORY))
        ignored = subprocess.run(['git', '-C', str(REPOSITORY), 'check-ignore', '--no-index', private],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.assertEqual(ignored.returncode, 0, ignored.stdout + ignored.stderr)
        self.assertEqual(ignored.stdout.strip(), private)
        self.assertTrue(all(path.name.endswith('_dev.json') for path in (self.sensor_path, self.ego_path)))
        self.assert_read_only()


if __name__ == '__main__':
    unittest.main()
