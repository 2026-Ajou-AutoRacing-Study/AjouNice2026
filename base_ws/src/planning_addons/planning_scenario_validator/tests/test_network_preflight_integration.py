"""Reject the uploaded LAN mismatch without modifying input or starting ROS."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml

from planning_scenario_validator import config, executions, runner


class NetworkPreflightIntegrationTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)
        path = config.ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/base/localization_baseline.yaml'
        self.document = yaml.safe_load(path.read_text())
        self.document['output_root'] = str(self.directory / 'results')
        self.document['morai']['scenario_load_host'] = '127.0.0.1'
        network = config.resolve_path(self.document['network']['ego_network_json'])
        data = json.loads(network.read_text())
        for entry in data['egoNetworkData'][0]['listEgoNetworkInfo']:
            if entry.get('PUBSUB_TYPE') == 783:
                entry['config']['udpConfig']['hostIP'] = '10.42.0.252'
        network = self.directory / 'ego.json'
        network.write_text(json.dumps(data))
        self.document['network']['ego_network_json'] = str(network)
        self.path = self.directory / 'source.yaml'
        self.path.write_text(yaml.safe_dump(self.document))

    def test_readonly_load_exposes_mismatch_and_keeps_original_bytes(self):
        original = self.path.read_bytes()
        cfg = config.load_campaign(self.path, check_install=False)
        self.assertIn('10.42.0.252', cfg['network_contract']['errors'][0])
        self.assertEqual(self.path.read_bytes(), original)

    def test_new_snapshot_rejects_mismatch_before_creating_results(self):
        with self.assertRaisesRegex(ValueError, 'ScenarioLoad'):
            executions.prepare_execution(self.path, check_install=False)
        self.assertFalse((self.directory / 'results').exists())

    def test_cli_check_returns_two_without_any_ros_or_gpu_launch(self):
        cfg = config.load_campaign(self.path, check_install=False)
        with patch.object(runner, 'load_campaign', return_value=cfg), \
             patch.object(runner, 'run') as drive, \
             patch.object(runner, 'resolve_contract') as model:
            self.assertEqual(runner.main([str(self.path), '--check']), 2)
        drive.assert_not_called()
        model.assert_not_called()

    def test_matching_host_allows_normal_snapshot_creation(self):
        self.document['morai']['scenario_load_host'] = '10.42.0.252'
        self.path.write_text(yaml.safe_dump(self.document))
        cfg = executions.prepare_execution(self.path, check_install=False)
        self.assertEqual(cfg['network_contract']['errors'], [])
