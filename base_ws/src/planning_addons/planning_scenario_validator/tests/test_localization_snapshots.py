"""Immutable localization input bytes and source/install-aware resume, no ROS."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from planning_scenario_validator import config, executions
from planning_scenario_validator.localization_contract import CONFIG_DEFAULTS


class LocalizationSnapshotTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name) / 'checkout'
        self.root.mkdir()
        root_patch = patch.object(config, 'ROOT', self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)
        for role, relative in CONFIG_DEFAULTS.items():
            self.write(relative, yaml.safe_dump({'role': role, 'frequency': 50}))
        for relative in (
            'base_ws/src/control/trajectory_follower/config/trajectory_follower_autoware.yaml',
            'base_ws/src/control/trajectory_follower/config/purepursuit_autoware.yaml',
            'base_ws/src/control/trajectory_follower/config/longi_pi_autoware.yaml',
            'configs/vehicle/ioniq5.yaml', 'configs/tf/morai_lanelet_static_tf.yaml',
            'base_ws/src/competition/competition_rule_monitor/config/kcity_v99_rules.yaml',
        ):
            self.write(relative, 'fixture: true\n')
        self.source_node = self.write('base_ws/src/localization/localization_manager/scripts/fixture.py', '# source fixture\n')
        self.installed_node = self.write('base_ws/install_aw149/lib/localization_manager/fixture.py', '# source fixture\n')
        self.write('base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch', '<launch/>\n')
        self.write('base_ws/install_aw149/share/bringup/launch/localization_gnss_imu_gt_ctra.launch', '<launch/>\n')
        self.scene = self.write('scenarios/baseline.json', json.dumps({
            'mapInfo': {'mapName': 'R_KR_PR_K-city_2025'},
            'egoVehicle': {'initPosition': {'pos': {'x': 0, 'y': 0}}}}))
        self.sensor = self.write('network/sensor.json', '{"cameraList":[],"Lidar3DList":[]}')
        self.network = self.write('network/ego.json', json.dumps({'egoNetworkData': [{
            'listEgoNetworkInfo': [{'PUBSUB_TYPE': 783, 'netType': 1,
                                  'config': {'commType': 1, 'udpConfig': {'hostPort': 9095}}}]}]}))
        self.install = self.root / 'morai_scenarios'
        self.install.mkdir()
        self.source = self.root / 'authoring.yaml'
        self.doc = {
            'schema_version': 2, 'campaign_name': 'localization_probe',
            'scenarios': {'directory': str(self.scene.parent), 'glob': '*.json'},
            'default_case': {'goal': {'x': 0, 'y': 0}, 'timeout_sec': 10},
            'stack': {'pose_source': 'localization', 'detector': 'od_gt', 'traffic_light': False},
            'recording': {'bag_enabled': False, 'video': {'enabled': False}},
            'network': {'sensor_json': str(self.sensor), 'ego_network_json': str(self.network)},
            'morai': {'scenario_install_dir': str(self.install)},
            'output_root': str(self.root / 'results'),
        }

    def write(self, relative, content):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    def prepare(self, *, check_install=True):
        self.source.write_text(yaml.safe_dump(self.doc))
        return executions.prepare_execution(self.source, check_install=check_install)

    def rewrite_manifest_hash(self, cfg, relative):
        manifest_path = cfg['output'] / 'execution.json'
        manifest = json.loads(manifest_path.read_text())
        manifest['files'][relative] = executions.sha256(cfg['output'] / relative)
        manifest_path.write_text(json.dumps(manifest))

    def test_all_six_fusion_configs_are_snapshotted_independent_of_od_gt(self):
        cfg = self.prepare()
        self.assertEqual(cfg['model_contract']['preset'], 'od_gt')
        self.assertIn('localization_contract', cfg)
        self.assertIn('localization_contract.json', cfg['execution']['files'])
        for role, relative in CONFIG_DEFAULTS.items():
            path = cfg['output'] / 'localization_configs' / (role + '.yaml')
            self.assertEqual(path.read_bytes(), (self.root / relative).read_bytes())
            self.assertEqual(cfg['stack']['localization_args'][role], str(path))
            self.assertEqual(cfg['localization_launch_args'][role], str(path))
            self.assertIn('localization_configs/' + role + '.yaml', cfg['execution']['files'])
        self.assertFalse(any(path.name.startswith('controller_') for path in (cfg['output'] / 'localization_configs').iterdir()))

    def test_authoring_config_edits_or_deletion_do_not_change_resumed_frozen_bytes(self):
        cfg = self.prepare()
        fingerprint = cfg['localization_contract']['fingerprint']
        role = 'measurement_noise_config'
        source = self.root / CONFIG_DEFAULTS[role]
        old_bytes = Path(cfg['localization_launch_args'][role]).read_bytes()
        source.write_text('sigma: 9\n')
        resumed = executions.load_execution(cfg['output'], check_install=True)
        self.assertEqual(resumed['localization_contract']['fingerprint'], fingerprint)
        self.assertEqual(Path(resumed['localization_launch_args'][role]).read_bytes(), old_bytes)
        source.unlink()
        resumed = executions.load_execution(cfg['output'], check_install=True)
        self.assertEqual(resumed['localization_contract']['fingerprint'], fingerprint)

    def test_original_yaml_edits_do_not_mutate_snapshot_candidate_or_heading(self):
        self.doc['stack']['localization_args'] = {'control_yaw_tau_sec': 0.2}
        cfg = self.prepare()
        original_snapshot = cfg['path'].read_bytes()
        original_authoring = (cfg['output'] / 'source_config.yaml').read_bytes()
        self.doc['stack']['localization_args']['control_yaw_tau_sec'] = 9
        self.source.write_text(yaml.safe_dump(self.doc))
        resumed = executions.load_execution(cfg['output'], check_install=True)
        self.assertEqual(resumed['localization_launch_args']['control_yaw_tau_sec'], 0.2)
        self.assertEqual(resumed['path'].read_bytes(), original_snapshot)
        self.assertEqual((cfg['output'] / 'source_config.yaml').read_bytes(), original_authoring)

    def test_source_installed_node_and_controller_config_changes_reject_resume(self):
        controller = self.root / 'base_ws/src/control/trajectory_follower/config/longi_pi_autoware.yaml'
        for path in (self.source_node, self.installed_node, controller):
            with self.subTest(path=path):
                cfg = self.prepare()
                original = path.read_bytes()
                try:
                    path.write_bytes(original + b'\n# changed runtime identity\n')
                    with self.assertRaisesRegex(ValueError, 'resume'):
                        executions.load_execution(cfg['output'], check_install=True)
                finally:
                    path.write_bytes(original)

    def test_changed_snapshot_candidate_rejected_even_if_manifest_hash_is_updated(self):
        cfg = self.prepare()
        snapshot = yaml.safe_load(cfg['path'].read_text())
        snapshot['stack']['localization_args']['control_yaw_tau_sec'] = 0.2
        cfg['path'].write_text(yaml.safe_dump(snapshot))
        self.rewrite_manifest_hash(cfg, 'config_snapshot.yaml')
        with self.assertRaisesRegex(ValueError, 'resume'):
            executions.load_execution(cfg['output'], check_install=True)

    def test_changed_frozen_config_rejected_even_if_manifest_hash_is_updated(self):
        cfg = self.prepare()
        relative = 'localization_configs/global_ekf_gt_config.yaml'
        (cfg['output'] / relative).write_text('candidate: changed\n')
        self.rewrite_manifest_hash(cfg, relative)
        with self.assertRaisesRegex(ValueError, 'resume'):
            executions.load_execution(cfg['output'], check_install=True)

    def test_changed_frozen_config_rejected_by_integrity_check_in_inspection_mode(self):
        cfg = self.prepare()
        (cfg['output'] / 'localization_configs/global_ekf_gt_config.yaml').write_text('candidate: changed\n')
        with self.assertRaisesRegex(ValueError, '사본'):
            executions.load_execution(cfg['output'], check_install=False)

    def test_gt_manager_config_is_frozen_and_default_uncorrected_heading_is_explicit(self):
        self.doc['stack']['pose_source'] = 'maptf_morai'
        cfg = self.prepare()
        args = cfg['stack']['gt_pose_args']
        self.assertFalse(args['use_heading_correction'])
        self.assertEqual(args['heading_yaw_offset_deg'], -1.347)
        self.assertEqual(set(path.name for path in (cfg['output'] / 'localization_configs').iterdir()),
                         {'localization_manager_config.yaml'})
        source = self.root / CONFIG_DEFAULTS['localization_manager_config']
        frozen = Path(args['localization_manager_config']).read_bytes()
        source.write_text('manager: changed\n')
        resumed = executions.load_execution(cfg['output'], check_install=True)
        self.assertEqual(Path(resumed['gt_pose_launch_args']['localization_manager_config']).read_bytes(), frozen)
        self.assertFalse(resumed['localization_contract']['launch_args']['use_heading_correction'])

    def test_corrected_gt_opt_in_is_distinct_from_legacy_gt_snapshot(self):
        self.doc['stack']['pose_source'] = 'maptf_morai'
        legacy = self.prepare()
        self.doc['stack']['gt_pose_args'] = {'use_heading_correction': True, 'heading_yaw_offset_deg': -1.347}
        corrected = self.prepare()
        self.assertTrue(corrected['gt_pose_launch_args']['use_heading_correction'])
        self.assertNotEqual(legacy['localization_contract']['fingerprint'], corrected['localization_contract']['fingerprint'])

    def test_readonly_inspection_does_not_certify_changed_runtime_source(self):
        cfg = self.prepare()
        original_fingerprint = cfg['localization_contract']['fingerprint']
        self.source_node.write_text('# later source change\n')
        inspected = executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual(inspected['localization_contract']['fingerprint'], original_fingerprint)
        with self.assertRaisesRegex(ValueError, 'resume'):
            executions.load_execution(cfg['output'], check_install=True)

    def test_old_snapshot_without_localization_contract_remains_readable(self):
        cfg = self.prepare()
        manifest_path = cfg['output'] / 'execution.json'
        manifest = json.loads(manifest_path.read_text())
        del manifest['files']['localization_contract.json']
        manifest_path.write_text(json.dumps(manifest))
        (cfg['output'] / 'localization_contract.json').unlink()
        legacy = executions.load_execution(cfg['output'], check_install=True)
        self.assertNotIn('localization_contract', legacy)


if __name__ == '__main__':
    unittest.main()
