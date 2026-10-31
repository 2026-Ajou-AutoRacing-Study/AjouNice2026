"""GNSS timing selection through normal GUI save and temporary quick runs."""
import copy
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from planning_scenario_validator import gui
from planning_scenario_validator.authoring import quick_test_document, updated_yaml
from planning_scenario_validator.localization_contract import normalize_localization_launch_args


class GnssDelayGuiTests(unittest.TestCase):
    def setUp(self):
        self.doc = gui.load_document(gui.EXAMPLES / 'static_pair_stress_gt.yaml')
        self.doc['stack']['pose_source'] = 'localization'
        self.doc['stack'].pop('localization_args', None)

    def save(self, doc, value, latest=None, **fields):
        values = gui.form_values(doc)
        values.update(gnss_delay_ms=value, **fields)
        return gui.operator_document(doc, latest or doc, values, doc['scenarios'].get('include'))

    def test_missing_default_is_zero_and_no_edit_does_not_insert_keys(self):
        self.assertEqual(gui.form_values(self.doc)['gnss_delay_ms'], 0.)
        for value in ['0', '0.0', 0.]:
            self.assertEqual(self.save(self.doc, value), self.doc)

    def test_explicit_ms_roundtrip_and_numeric_equality(self):
        for value in [0, 15, 22.5, 100]:
            doc = copy.deepcopy(self.doc)
            doc['stack']['localization_args'] = {'gnss_delay_ms': value}
            self.assertEqual(gui.form_values(doc)['gnss_delay_ms'], value)
            self.assertEqual(self.save(doc, str(float(value))), doc)

    def test_seconds_display_and_unchanged_save_preserve_legacy(self):
        for seconds in [0., .015, .1]:
            doc = copy.deepcopy(self.doc)
            doc['stack']['localization_args'] = {'gnss_motion_compensation_sec': seconds}
            self.assertEqual(gui.form_values(doc)['gnss_delay_ms'], seconds * 1000.)
            self.assertEqual(self.save(doc, str(seconds * 1000.)), doc)
            self.assertEqual(gui.build_document(doc, gui.form_values(doc)), doc)

    def test_edit_converts_legacy_unit_only_and_preserves_advanced_values(self):
        doc = copy.deepcopy(self.doc)
        doc['stack']['localization_args'] = {'gnss_motion_compensation_sec': .015,
                                             'control_yaw_tau_sec': .4}
        result = self.save(doc, '20')
        self.assertEqual(result['stack']['localization_args'],
                         {'gnss_delay_ms': 20., 'control_yaw_tau_sec': .4})
        self.assertIn('gnss_motion_compensation_sec', doc['stack']['localization_args'])
        normalize_localization_launch_args(result['stack'])

    def test_new_value_and_zero_use_real_backend_numeric_argument(self):
        result = self.save(self.doc, '15')
        self.assertEqual(normalize_localization_launch_args(result['stack']), {'gnss_delay_ms': 15.})
        result = self.save(result, '0')
        self.assertEqual(normalize_localization_launch_args(result['stack']), {'gnss_delay_ms': 0.})

    def test_invalid_values_rejected_before_writing(self):
        for value in [-1, 100.1, 'nan', 'inf', '-inf', 'bad', '', True, None]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.save(self.doc, value)

    def test_distinct_vscode_advanced_edit_is_preserved(self):
        latest = copy.deepcopy(self.doc)
        latest['stack']['localization_args'] = {'control_yaw_tau_sec': .4}
        result = self.save(self.doc, '15', latest=latest)
        self.assertEqual(result['stack']['localization_args'],
                         {'control_yaw_tau_sec': .4, 'gnss_delay_ms': 15.})

    def test_vscode_ms_or_legacy_seconds_conflict_is_not_silently_lost(self):
        for before, after in [({'gnss_delay_ms': 15}, {'gnss_delay_ms': 20}),
                              ({'gnss_motion_compensation_sec': .015}, {'gnss_motion_compensation_sec': .02}),
                              ({'gnss_motion_compensation_sec': .015}, {'gnss_delay_ms': 20})]:
            doc = copy.deepcopy(self.doc);doc['stack']['localization_args'] = before
            latest = copy.deepcopy(doc);latest['stack']['localization_args'] = after
            with self.subTest(before=before, after=after), self.assertRaisesRegex(ValueError, '양쪽'):
                self.save(doc, '10', latest=latest)

    def test_gt_switch_removes_only_timing_keys_and_no_hidden_revival(self):
        doc = self.save(self.doc, '15')
        gt = self.save(doc, '15', pose_source='maptf_morai')
        self.assertNotIn('localization_args', gt['stack'])
        self.assertEqual(gui.form_values(gt)['gnss_delay_ms'], 0.)
        back = self.save(gt, '0', pose_source='localization')
        self.assertNotIn('localization_args', back['stack'])
        self.assertEqual(doc['stack']['localization_args']['gnss_delay_ms'], 15.)

    def test_gt_switch_refuses_erasing_unrelated_advanced_configuration(self):
        doc = self.save(self.doc, '15')
        doc['stack']['localization_args']['control_yaw_tau_sec'] = .4
        before = copy.deepcopy(doc)
        with self.assertRaisesRegex(ValueError, '고급 측위 설정을 보존'):
            self.save(doc, '15', pose_source='maptf_morai')
        self.assertEqual(doc, before)

    def test_gt_switch_conflicts_with_concurrent_new_advanced_setting(self):
        doc = self.save(self.doc, '15')
        latest = copy.deepcopy(doc)
        latest['stack']['localization_args']['control_yaw_tau_sec'] = .4
        with self.assertRaisesRegex(ValueError, '양쪽'):
            self.save(doc, '15', latest=latest, pose_source='maptf_morai')
        self.assertEqual(latest['stack']['localization_args']['control_yaw_tau_sec'], .4)

    def test_no_gui_edit_preserves_external_unit_change(self):
        doc = self.save(self.doc, '15')
        latest = copy.deepcopy(doc)
        latest['stack']['localization_args'] = {'gnss_motion_compensation_sec': .02}
        self.assertEqual(self.save(doc, '15', latest=latest), latest)

    def test_quick_provider_comparison_cleans_only_temporary_copy(self):
        doc = self.save(self.doc, '15')
        before = copy.deepcopy(doc)
        gt = quick_test_document(doc, ['one.json'], detector='od_gt', pose_source='maptf_morai', traffic_light=False)
        self.assertNotIn('localization_args', gt['stack'])
        loc = quick_test_document(doc, ['one.json'], detector='od_gt', pose_source='localization', traffic_light=False)
        self.assertEqual(loc['stack']['localization_args']['gnss_delay_ms'], 15.)
        self.assertEqual(doc, before)
        gt['stack']['gt_pose_args'] = {'heading_yaw_offset_deg': -1.347}
        converted = quick_test_document(gt, ['one.json'], detector='od_gt', pose_source='localization', traffic_light=False)
        self.assertNotIn('gt_pose_args', converted['stack'])

    def test_yaml_patch_retains_comments_and_fixed_fields(self):
        text = yaml.safe_dump(self.doc, sort_keys=False)
        text = '# user comment\n' + text
        result = self.save(self.doc, '15')
        edited = updated_yaml(text, result)
        self.assertIn('# user comment', edited)
        self.assertEqual(yaml.safe_load(edited), result)
        self.assertEqual(result['network'], self.doc['network'])
        self.assertEqual(result['default_case'], self.doc['default_case'])


@unittest.skipUnless(os.environ.get('DISPLAY'), 'No Tk display; use Xvfb for widget check')
class GnssDelayWidgetTests(unittest.TestCase):
    def test_ms_entry_save_and_gt_running_locks(self):
        with tempfile.TemporaryDirectory() as directory:
            doc = gui.load_document(gui.EXAMPLES / 'static_pair_stress_gt.yaml')
            doc['stack']['pose_source'] = 'localization'
            doc['stack'].pop('localization_args', None)
            doc['output_root'] = str(Path(directory) / 'results')
            path = Path(directory) / 'personal.yaml'
            path.write_text(yaml.safe_dump(doc))
            def ready(app):
                try:
                    app['actions']['open_yaml'](path)
                    widget = app['field_widgets']['gnss_delay_ms']
                    self.assertEqual(str(widget.cget('state')), 'normal')
                    app['fields']['gnss_delay_ms'].set('15')
                    app['actions']['save']()
                    self.assertEqual(yaml.safe_load(path.read_text())['stack']['localization_args']['gnss_delay_ms'], 15.)
                    app['actions']['set_running_ui'](True)
                    self.assertEqual(str(widget.cget('state')), 'disabled')
                    app['actions']['set_running_ui'](False)
                    self.assertEqual(str(widget.cget('state')), 'normal')
                    app['fields']['pose_source'].set('maptf_morai')
                    self.assertEqual(str(widget.cget('state')), 'disabled')
                    app['actions']['set_running_ui'](False)
                    self.assertEqual(str(widget.cget('state')), 'disabled')
                    app['actions']['save']()
                    self.assertNotIn('localization_args', yaml.safe_load(path.read_text())['stack'])
                finally:
                    app['root'].destroy()
            with patch.dict(os.environ, {'XDG_CONFIG_HOME': directory}):
                gui.main(on_ready=ready)


if __name__ == '__main__':unittest.main()

