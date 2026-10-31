"""Explicit MR candidate boundaries, without starting ROS or MORAI."""
import copy
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from planning_scenario_validator.config import ROOT, ConfigError, load_campaign, resolve_path
from planning_scenario_validator.localization_contract import (
    CONFIG_DEFAULTS, MR_PINNED_ARGS, effective_localization_args, effective_gt_pose_args,
    normalize_gt_pose_launch_args, normalize_localization_launch_args,
    normalize_localization_safety, resolve_localization_contract)


EXAMPLE = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/base/example.yaml'


class LocalizationContractTests(unittest.TestCase):
    def test_existing_yaml_preserves_localization_and_gt_defaults(self):
        cfg = load_campaign(EXAMPLE, check_install=False)
        self.assertEqual(cfg['localization_launch_args'], {})
        self.assertEqual(cfg['gt_pose_launch_args'], {})
        self.assertEqual(cfg['localization_safety'], {'enabled': False})
        args = effective_localization_args(cfg, ROOT)
        self.assertEqual({key: args[key] for key in MR_PINNED_ARGS}, MR_PINNED_ARGS)
        self.assertEqual(args['control_translation_tau_sec'], 0.5)
        self.assertEqual(args['control_yaw_tau_sec'], 0.5)
        self.assertFalse(args['use_quantized_gnss_control_state'])
        self.assertNotIn('gnss_delay_ms', args)

    def test_every_allowlisted_argument_is_a_real_bringup_launch_argument(self):
        launch = ET.parse(ROOT / 'base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch')
        names = {node.attrib['name'] for node in launch.findall('arg')}
        args = effective_localization_args(load_campaign(EXAMPLE, check_install=False), ROOT)
        self.assertLessEqual(set(args), names)

    def test_candidate_scalars_are_normalized_and_pinned_keys_rejected(self):
        explicit = {'control_translation_tau_sec': 0, 'control_yaw_tau_sec': 1,
                    'use_quantized_gnss_control_state': True,
                    'gnss_quantization_input_mode': 'cell_transition',
                    'relocation_guard_consecutive_samples': 4,
                    'enable_relocation_guard': True}
        result = normalize_localization_launch_args({'pose_source': 'localization', 'localization_args': explicit})
        self.assertEqual(result, explicit)
        for key in ('gt_source', 'sensor_config', 'use_sim_time', 'heading_yaw_offset_deg',
                    'use_heading_correction', 'publish_control_state', 'control_state_topic'):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, '고정된'):
                normalize_localization_launch_args({'pose_source': 'localization', 'localization_args': {key: 0}})

    def test_wrong_types_nonfinite_negative_values_and_bad_modes_rejected(self):
        invalid = [('control_yaw_tau_sec', '0.5'), ('control_translation_tau_sec', True),
                   ('control_yaw_tau_sec', float('inf')), ('control_yaw_tau_sec', -1),
                   ('use_quantized_gnss_control_state', 'true'),
                   ('control_state_reset_yaw_error_deg', 181),
                   ('relocation_guard_threshold_m', 0),
                   ('relocation_guard_consecutive_samples', 1.5),
                   ('relocation_guard_consecutive_samples', True),
                   ('gnss_quantization_input_mode', 'made_up'),
                   ('initial_position_source', 'competition'),
                   ('initial_position_source', 'manual')]
        for key, value in invalid:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                normalize_localization_launch_args({'pose_source': 'localization', 'localization_args': {key: value}})

    def test_arguments_on_wrong_provider_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'pose_source'):
            normalize_localization_launch_args({'pose_source': 'maptf_morai', 'localization_args': {'control_yaw_tau_sec': 0.2}})
        with self.assertRaisesRegex(ValueError, 'pose_source'):
            normalize_gt_pose_launch_args({'pose_source': 'localization', 'gt_pose_args': {'use_heading_correction': True}})

    def test_final_hardening_candidates_are_opt_in_and_bounded(self):
        cfg = load_campaign(EXAMPLE, check_install=False)
        defaults = effective_localization_args(cfg, ROOT)
        self.assertEqual(defaults['gnss_motion_compensation_sec'], 0.0)
        self.assertFalse(defaults['enable_relocation_guard'])
        self.assertTrue(defaults['relocation_guard_monitor_only'])
        explicit = {'gnss_motion_compensation_sec': 0.02,
                    'relocation_guard_reset_ack_consecutive_samples': 2,
                    'relocation_guard_reacquisition_grace_sec': 1.0}
        self.assertEqual(normalize_localization_launch_args({
            'pose_source': 'localization', 'localization_args': explicit}), explicit)
        invalid = [('gnss_motion_compensation_sec', 0.10001),
                   ('gnss_motion_compensation_sec', True),
                   ('relocation_guard_stationary_hold_sec', 0),
                   ('relocation_guard_max_motion_age_sec', 0),
                   ('relocation_guard_max_input_age_sec', 0),
                   ('relocation_guard_future_tolerance_sec', -0.01),
                   ('relocation_guard_gnss_stationary_max_span_m', 0),
                   ('relocation_guard_reset_ack_consecutive_samples', True),
                   ('relocation_guard_reset_ack_consecutive_samples', 2.5),
                   ('relocation_guard_reset_ack_yaw_tolerance_deg', 181)]
        for key, value in invalid:
            with self.subTest(key=key), self.assertRaises(ValueError):
                normalize_localization_launch_args({
                    'pose_source': 'localization', 'localization_args': {key: value}})

    def test_optional_gnss_delay_ms_accepts_bounded_numbers(self):
        for value in (0, 0.0, 12, 19.5, 100):
            with self.subTest(value=value):
                result = normalize_localization_launch_args({
                    'pose_source': 'localization',
                    'localization_args': {'gnss_delay_ms': value}})
                self.assertEqual(result, {'gnss_delay_ms': float(value)})
        for value in (-0.001, 100.001, True, False, '20', '', None,
                      float('inf'), float('-inf'), float('nan'), 10 ** 1000):
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_localization_launch_args({
                    'pose_source': 'localization',
                    'localization_args': {'gnss_delay_ms': value}})

    def test_gnss_delay_ms_rejects_nonzero_legacy_compensation(self):
        for delay_ms in (0, 20):
            for explicit in ({'gnss_delay_ms': delay_ms, 'gnss_motion_compensation_sec': 0.02},
                             {'gnss_motion_compensation_sec': 0.02, 'gnss_delay_ms': delay_ms}):
                with self.subTest(explicit=explicit), self.assertRaisesRegex(ValueError, '0이어야'):
                    normalize_localization_launch_args({
                        'pose_source': 'localization', 'localization_args': explicit})
        explicit = {'gnss_delay_ms': 20, 'gnss_motion_compensation_sec': 0.0}
        self.assertEqual(normalize_localization_launch_args({
            'pose_source': 'localization', 'localization_args': explicit}), explicit)

    def test_gnss_delay_ms_is_explicit_in_effective_args_and_real_launch(self):
        cfg = load_campaign(EXAMPLE, check_install=False)
        cfg['localization_launch_args'] = normalize_localization_launch_args({
            'pose_source': 'localization', 'localization_args': {'gnss_delay_ms': 20}})
        effective = effective_localization_args(cfg, ROOT)
        self.assertEqual(effective['gnss_delay_ms'], 20.0)
        self.assertEqual(effective['gnss_motion_compensation_sec'], 0.0)
        launch = ET.parse(ROOT / 'base_ws/src/bringup/launch/localization_gnss_imu_gt_ctra.launch')
        names = {node.attrib['name'] for node in launch.findall('arg')}
        self.assertLessEqual(set(effective), names)

    def test_load_campaign_preserves_explicit_gnss_delay_ms(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'delay.yaml'
            document = yaml.safe_load(EXAMPLE.read_text())
            document['stack']['localization_args'] = {'gnss_delay_ms': 20}
            path.write_text(yaml.safe_dump(document))
            cfg = load_campaign(path, check_install=False)
            self.assertEqual(cfg['localization_launch_args'], {'gnss_delay_ms': 20.0})
            document['stack']['localization_args']['gnss_motion_compensation_sec'] = 0.02
            path.write_text(yaml.safe_dump(document))
            with self.assertRaisesRegex(ConfigError, '0이어야'):
                load_campaign(path, check_install=False)

    def test_explicit_corrected_gt_and_legacy_gt_are_distinct(self):
        self.assertEqual(normalize_gt_pose_launch_args({'pose_source': 'maptf_morai'}), {})
        self.assertEqual(normalize_gt_pose_launch_args({'pose_source': 'maptf_morai', 'gt_pose_args': {
            'use_heading_correction': True, 'heading_yaw_offset_deg': -1.347}}),
            {'use_heading_correction': True, 'heading_yaw_offset_deg': -1.347})
        for bad in ({'use_heading_correction': 'true'}, {'heading_yaw_offset_deg': float('nan')},
                    {'gt_source': 'ego'}):
            with self.subTest(args=bad), self.assertRaises(ValueError):
                normalize_gt_pose_launch_args({'pose_source': 'maptf_morai', 'gt_pose_args': bad})

    def test_config_paths_must_exist_and_contain_yaml_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            good = root / 'good.yaml'
            good.write_text('frequency: 50\n')
            bad = root / 'bad.yaml'
            bad.write_text('[1, 2]\n')
            stack = {'pose_source': 'localization', 'localization_args': {'global_ekf_gt_config': str(good)}}
            result = normalize_localization_launch_args(stack, resolve_path)
            self.assertEqual(result['global_ekf_gt_config'], str(good))
            for path in (root / 'missing.yaml', bad, root / 'other.json'):
                stack['localization_args']['global_ekf_gt_config'] = str(path)
                with self.subTest(path=path), self.assertRaises(ValueError):
                    normalize_localization_launch_args(stack, resolve_path)

    def test_load_campaign_exposes_resolved_args_and_rejects_unknowns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'candidate.yaml'
            document = yaml.safe_load(EXAMPLE.read_text())
            document['stack']['localization_args'] = {
                'measurement_noise_config': CONFIG_DEFAULTS['measurement_noise_config'],
                'control_yaw_tau_sec': 0.2}
            path.write_text(yaml.safe_dump(document))
            cfg = load_campaign(path, check_install=False)
            self.assertEqual(cfg['localization_launch_args']['control_yaw_tau_sec'], 0.2)
            self.assertEqual(cfg['localization_launch_args']['measurement_noise_config'],
                             str(ROOT / CONFIG_DEFAULTS['measurement_noise_config']))
            document['stack']['localization_args']['yaw_magic'] = 3
            path.write_text(yaml.safe_dump(document))
            with self.assertRaisesRegex(ConfigError, 'yaw_magic'):
                load_campaign(path, check_install=False)

    def test_config_content_fingerprint_is_snapshot_path_independent(self):
        cfg = load_campaign(EXAMPLE, check_install=False)
        first = resolve_localization_contract(cfg, ROOT)
        with tempfile.TemporaryDirectory() as directory:
            snapshot = Path(directory) / 'frozen_noise.yaml'
            original = ROOT / CONFIG_DEFAULTS['measurement_noise_config']
            snapshot.write_bytes(original.read_bytes())
            cfg['localization_launch_args']['measurement_noise_config'] = str(snapshot)
            copied = resolve_localization_contract(cfg, ROOT)
            self.assertEqual(first['fingerprint'], copied['fingerprint'])
            snapshot.write_text(snapshot.read_text() + '\n# candidate modification\n')
            changed = resolve_localization_contract(cfg, ROOT)
            self.assertNotEqual(first['fingerprint'], changed['fingerprint'])

    def test_gt_manager_config_can_be_frozen_without_changing_legacy_semantics(self):
        cfg = load_campaign(EXAMPLE, check_install=False)
        cfg['stack']['pose_source'] = 'maptf_morai'
        before = resolve_localization_contract(cfg, ROOT)
        with tempfile.TemporaryDirectory() as directory:
            frozen = Path(directory) / 'manager.yaml'
            frozen.write_bytes((ROOT / CONFIG_DEFAULTS['localization_manager_config']).read_bytes())
            cfg['gt_pose_launch_args'] = normalize_gt_pose_launch_args({
                'pose_source': 'maptf_morai', 'gt_pose_args': {'localization_manager_config': str(frozen)}}, resolve_path)
            after = resolve_localization_contract(cfg, ROOT)
            self.assertEqual(before['fingerprint'], after['fingerprint'])
            self.assertFalse(effective_gt_pose_args(cfg, ROOT)['use_heading_correction'])

    def test_semantic_args_and_installed_artifacts_are_resume_identity(self):
        cfg = load_campaign(EXAMPLE, check_install=False)
        with tempfile.TemporaryDirectory() as directory:
            # Real installed files are not guaranteed in a fresh CI checkout.
            # Use an explicit artifact to test identity, not the developer PC.
            install = Path(directory) / 'installed'
            binary = install / 'lib/trajectory_follower/longi_pi_autoware'
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b'fixture-controller-build')
            first = resolve_localization_contract(cfg, ROOT, install_root=install)
            candidate = copy.deepcopy(cfg)
            candidate['localization_launch_args']['control_yaw_tau_sec'] = 0.2
            second = resolve_localization_contract(candidate, ROOT, install_root=install)
            self.assertNotEqual(first['fingerprint'], second['fingerprint'])
            self.assertIn('controller_pi', first['config_files'])
            self.assertIsNotNone(first['installed_files_sha256']['lib/trajectory_follower/longi_pi_autoware'])
            missing_install = resolve_localization_contract(cfg, ROOT, install_root=Path(directory) / 'missing')
            self.assertNotEqual(first['fingerprint'], missing_install['fingerprint'])
            self.assertTrue(missing_install['missing_installed_files'])

    def test_safety_is_off_unless_opted_in_with_validated_defaults(self):
        self.assertEqual(normalize_localization_safety({}), {'enabled': False})
        guard = normalize_localization_safety({'validation': {'localization_safety': {'enabled': True}}})
        self.assertEqual(guard['speed_max_mps'], 0.05)
        self.assertEqual(guard['startup_timeout_sec'], 20)
        self.assertTrue(guard['park_after_run'])
        self.assertTrue(guard['stop_on_collision'])

    def test_safety_rejects_unknown_wrong_type_nonpositive_and_nonfinite(self):
        bad_settings = [{'enabled': 'true'}, {'speed_max_mps': 0}, {'freshness_sec': -1},
                        {'stationary_hold_sec': float('nan')}, {'stop_timeout_sec': '10'},
                        {'startup_timeout_sec': True}, {'park_after_run': 1},
                        {'no_collision_gate': True}]
        for value in bad_settings:
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_localization_safety({'validation': {'localization_safety': value}})


if __name__ == '__main__':
    unittest.main()
