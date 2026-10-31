"""Count-based localization evidence contract; no ROS master or simulator needed."""
import json
from pathlib import Path
import tempfile
import unittest

import rosbag
import yaml
from std_msgs.msg import String

from planning_scenario_validator import executions, gui, recording_options as options
from planning_scenario_validator.config import ROOT, load_campaign
from planning_scenario_validator.recording_audit import (
    CORE, LOCALIZATION_REPLAY, LOCALIZATION_FUSION, audit_bag,
    localization_required_topics, profile_contract_warnings)
from planning_scenario_validator.topic_contract import model_topics


class LocalizationRecordingTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.doc = gui.load_document(gui.MISSIONS / 'gps_shadow/base/example.yaml')
        self.doc['stack'].update(pose_source='localization', detector='od_gt', traffic_light=False)
        self.doc['output_root'] = str(self.root / 'results')
        self.doc['recording']['video']['enabled'] = False
        options.apply_mode(self.doc, options.LOCALIZATION)
        self.path = self.root / 'campaign.yaml'

    def cfg(self):
        self.path.write_text(yaml.safe_dump(self.doc))
        return load_campaign(self.path, check_install=False)

    def bag(self, topics):
        # Real indexed messages, intentionally not a semantic/GT-accuracy fixture.
        path = self.root / 'fixture.bag'
        with rosbag.Bag(str(path), 'w') as bag:
            for topic in dict.fromkeys(topics):
                bag.write(topic, String(data='count fixture'))
        return path

    def complete_topics(self, cfg):
        topics = list(CORE) + localization_required_topics(cfg)
        if cfg['stack']['pose_source'] != 'localization':
            topics = [t for t in topics if t != '/localization/control_kinematic_state']
        if cfg['stack']['detector'] == 'bevfusion':
            model = model_topics(cfg['stack']['bevfusion_model'])
            topics += [model['status'], model['latency_ms']]
        return list(dict.fromkeys(topics))

    def test_preset_has_exact_inputs_without_raw_sensors_or_perception_timing(self):
        cfg = self.cfg()
        self.assertEqual(cfg['recording']['profile_preset'], 'localization_analysis')
        self.assertEqual(options.mode_label(cfg['recording']), options.LOCALIZATION)
        self.assertFalse(cfg['recording']['analysis_mode'])
        self.assertFalse(cfg['recording']['include_sensors'])
        self.assertFalse(cfg['recording']['video']['enabled'])
        self.assertFalse(set(options.SENSORS) & set(cfg['recording']['topics']))
        self.assertTrue(set(localization_required_topics(cfg)) <= set(cfg['recording']['topics']))
        self.assertFalse(options.diagnostics_enabled(cfg['recording']))
        self.assertEqual(cfg['recording']['topic_regex'], [])
        self.assertFalse(any(t.endswith('/frame_timing') for t in cfg['recording']['topics']))
        self.assertIn('/localization/initialpose', options.topic_catalog(ROOT))

    def test_gui_yaml_and_immutable_snapshot_roundtrip_preserve_localization_mode(self):
        values = gui.form_values(self.doc)
        self.assertEqual(values['recording_mode'], options.LOCALIZATION)
        result = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.assertEqual(result, self.doc)
        cfg = self.cfg()
        execution = executions.prepare_execution(cfg['path'], check_install=False)
        loaded = executions.load_execution(execution['output'], check_install=False)
        self.assertEqual(options.mode_label(loaded['recording']), options.LOCALIZATION)
        self.assertEqual(loaded['recording']['topics'], cfg['recording']['topics'])
        self.assertFalse(loaded['recording']['video']['enabled'])

    def test_localization_missing_internal_gt_or_global_ekf_warns(self):
        cfg = self.cfg()
        topics = self.complete_topics(cfg)
        for missing in ('/localization/imu', '/localization/gt/orientation',
                        '/localization/odom', '/localization/initialpose'):
            with self.subTest(missing=missing):
                report = audit_bag(self.bag([t for t in topics if t != missing]), cfg, driving_started=True)
                self.assertEqual(report['status'], 'WARN')
                self.assertIn(missing, report['missing_required'])
                self.assertEqual(report['topics'][missing]['message_count'], 0)
                self.assertFalse(report['complete_for_localization_assessment'])

    def test_gt_pose_comparison_has_no_false_warning_for_ekf_or_smooth_absence(self):
        self.doc['stack']['pose_source'] = 'maptf_morai'
        cfg = self.cfg()
        report = audit_bag(self.bag(self.complete_topics(cfg)), cfg, driving_started=True)
        self.assertEqual(report['status'], 'OK', report['warnings'])
        self.assertTrue(report['complete_for_localization_replay'])
        self.assertTrue(report['complete_for_localization_assessment'])
        self.assertTrue(report['complete_for_analysis'])
        self.assertEqual(report['analysis_kind'], 'localization')
        self.assertFalse(set(LOCALIZATION_FUSION) & set(report['missing_required']))
        self.assertTrue(set(LOCALIZATION_FUSION) <= set(report['missing_requested']))
        self.assertEqual(report['localization_recording']['conditionally_optional'], LOCALIZATION_FUSION)
        self.assertFalse(report['planner_processing_time']['requested'])

    def test_missing_raw_input_cannot_be_claimed_replayable_or_assessable(self):
        cfg = self.cfg()
        topics = [t for t in self.complete_topics(cfg) if t != '/sensor/imu']
        report = audit_bag(self.bag(topics), cfg, driving_started=True)
        self.assertEqual(report['status'], 'WARN')
        self.assertFalse(report['complete_for_localization_replay'])
        self.assertFalse(report['complete_for_localization_assessment'])
        self.assertIn('/sensor/imu', report['localization_recording']['missing_for_replay'])
        self.assertFalse(report['re_inference_possible'])

    def test_raw_startup_can_be_replayed_but_is_not_a_drive_assessment(self):
        cfg = self.cfg()
        report = audit_bag(self.bag(self.complete_topics(cfg)), cfg, driving_started=False)
        self.assertTrue(report['complete_for_localization_replay'])
        self.assertFalse(report['complete_for_localization_assessment'])
        self.assertFalse(report['complete_for_analysis'])

    def test_positive_counts_do_not_claim_outage_coverage_or_actual_standstill(self):
        cfg = self.cfg()
        report = audit_bag(self.bag(self.complete_topics(cfg)), cfg, driving_started=True)
        self.assertTrue(report['complete_for_localization_assessment'])
        self.assertTrue(report['localization_recording']['count_validation_only'])
        self.assertFalse(report['localization_recording']['outage_phase_coverage_verified'])
        self.assertIn('physical standstill', report['localization_recording']['note'])
        self.assertEqual(report['topics']['/sensor/gps']['message_count'], 1)

    def test_e24_e25_only_selected_model_telemetry_is_required(self):
        self.doc['stack']['detector'] = 'bevfusion'
        for preset in ('base5_e12', 'base5_e24', 'e25'):
            with self.subTest(preset=preset):
                self.doc['stack']['bevfusion_model'] = preset
                cfg = self.cfg()
                report = audit_bag(self.bag(self.complete_topics(cfg)), cfg, driving_started=True)
                self.assertEqual(report['status'], 'OK', report['warnings'])
                self.assertTrue(report['complete_for_localization_assessment'])
                other = model_topics('e25' if preset != 'e25' else 'base5_e12')['status']
                self.assertEqual(report['topics'][other]['message_count'], 0)
                self.assertNotIn(other, report['missing_required'])

    def test_frozen_old_profile_omission_warns_without_mutating_input(self):
        cfg = self.cfg()
        old_topics = [t for t in cfg['recording']['topics'] if t != '/competition/raw_twist']
        path = self.root / 'frozen_old_profile.json'
        path.write_text(json.dumps(old_topics))
        original = path.read_bytes()
        cfg['recording'].update(topic_profile=str(path), topics=old_topics)
        self.assertTrue(profile_contract_warnings(cfg))
        report = audit_bag(self.bag(old_topics), cfg, driving_started=True)
        self.assertIn('/competition/raw_twist', report['not_requested_required'])
        self.assertEqual(path.read_bytes(), original)

    def test_localization_share_helper_warns_when_bag_excluded_and_off_disables(self):
        cfg = self.cfg()
        self.assertTrue(options.sharing_enabled(cfg['recording']))
        self.assertTrue(options.sharing_warnings(cfg['recording'], {'profile': 'video'}))
        self.assertFalse(options.sharing_warnings(cfg['recording'], {'profile': 'bag'}))
        self.assertFalse(options.diagnostics_enabled(cfg['recording']))
        options.apply_mode(self.doc, options.OFF)
        self.assertFalse(options.sharing_enabled(self.doc['recording']))

    def test_preset_switch_preserves_existing_video_preference(self):
        self.doc['recording']['video']['enabled'] = True
        options.apply_mode(self.doc, options.LOCALIZATION)
        self.assertTrue(self.doc['recording']['video']['enabled'])
        self.assertFalse(self.doc['recording']['include_sensors'])
        self.assertEqual(options.mode_label(self.doc['recording']), options.LOCALIZATION)


if __name__ == '__main__':
    unittest.main()
