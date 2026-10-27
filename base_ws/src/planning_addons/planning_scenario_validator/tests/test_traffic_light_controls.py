"""Opt-in traffic controls: pure config/runtime mocks and isolated Tk checks."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml

from planning_scenario_validator import batch, executions, gui, runner
from planning_scenario_validator.authoring import quick_test_document
from planning_scenario_validator.config import ConfigError, load_campaign, validate_campaign_document
from planning_scenario_validator.recording_options import TRAFFIC_LIGHT_DEBUG_TOPIC
from planning_scenario_validator.traffic_light import (launch_options, timed_policy_options,
    TIMED_POLICY_DEFAULTS, yellow_braking_options, YELLOW_BRAKING_DEFAULTS,
    YELLOW_POLICY_IMPLEMENTATION)


YELLOW = 'traffic_light_yellow_passthrough'
DEBUG = 'traffic_light_debug_view'


class TrafficControlsTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.document = gui.load_document(gui.MISSIONS/'highway/base/example.yaml')
        self.document.update(campaign_name='traffic_controls', output_root=str(self.root/'results'))
        scenes = self.root/'scenes'; scenes.mkdir()
        (scenes/'one.json').write_text(json.dumps({'mapInfo': {'mapName': 'R_KR_PR_K-city_2025'},
            'egoVehicle': {'initPosition': {'pos': {'x': 0, 'y': 0}}}}))
        self.document['scenarios'] = {'directory': str(scenes), 'glob': '*.json'}
        self.document['stack'].update(traffic_light=True, traffic_light_profile='legacy',
            traffic_light_runtime='system', detector='od_gt')
        self.document['recording']['video'] = {'enabled': False}
        profile = self.root/'topics.json'; profile.write_text('["/Ego_topic"]')
        self.document['recording']['topic_profile'] = str(profile)
        self.path = self.root/'source.yaml'

    def save(self, name='source.yaml'):
        path = self.root/name
        path.write_text(yaml.safe_dump(self.document))
        return path

    def load(self, name='source.yaml'):
        return load_campaign(self.save(name), check_install=False)

    def test_new_controls_default_off_and_noop_gui_save_does_not_migrate_yaml(self):
        expected = {'yellow_passthrough': False, 'debug_view': False, 'publish_debug_image': False}
        self.assertEqual(launch_options({}), expected)
        values = gui.form_values(self.document)
        self.assertFalse(values[YELLOW]); self.assertFalse(values[DEBUG])
        self.assertEqual(gui.build_document(self.document, values), self.document)
        self.assertEqual(gui.operator_document(self.document, self.document, values, None), self.document)
        cfg = self.load()
        self.assertEqual(cfg['traffic_light_options'], expected)
        self.assertNotIn(TRAFFIC_LIGHT_DEBUG_TOPIC, cfg['recording']['topics'])

    def test_boolean_validation_rejects_strings_numbers_and_null_even_when_traffic_off(self):
        self.document['stack']['traffic_light'] = False
        for key in (YELLOW, DEBUG):
            for invalid in ('false', 0, 1, None, [], {}):
                with self.subTest(key=key, invalid=invalid):
                    doc = copy.deepcopy(self.document); doc['stack'][key] = invalid
                    with self.assertRaisesRegex(ConfigError, key):
                        validate_campaign_document(doc)
                    with self.assertRaisesRegex(ValueError, key):
                        launch_options(doc['stack'])

    def test_gui_edits_both_controls_and_preserves_explicit_display_and_fixed_fields(self):
        self.document['recording']['video']['display'] = ':17.0'
        values = gui.form_values(self.document); values.update({YELLOW: True, DEBUG: True})
        for transform in ('operator', 'build'):
            with self.subTest(transform=transform), patch.dict(os.environ, {'DISPLAY': ':91.0'}):
                result = (gui.operator_document(self.document, self.document, values, None)
                          if transform == 'operator' else gui.build_document(self.document, values))
                self.assertTrue(result['stack'][YELLOW]); self.assertTrue(result['stack'][DEBUG])
                self.assertFalse(result['recording']['video']['enabled'])
                self.assertEqual(result['recording']['video']['display'], ':17.0')
                self.assertEqual(result['default_case'], self.document['default_case'])
                self.assertEqual(result['network'], self.document['network'])

    def test_concurrent_same_option_conflict_is_not_overwritten(self):
        # Explicit false differs from an absent field in the immutable authoring baseline.
        latest = copy.deepcopy(self.document); latest['stack'][YELLOW] = False
        values = gui.form_values(self.document); values[YELLOW] = True
        with self.assertRaisesRegex(ValueError, '양쪽'):
            gui.operator_document(self.document, latest, values, None)

    def test_debug_view_resolves_display_without_enabling_video_and_respects_bag_off(self):
        self.document['stack'][DEBUG] = True
        for enabled in (True, False):
            with self.subTest(bag_enabled=enabled):
                self.document['recording']['bag_enabled'] = enabled
                with patch.dict(os.environ, {'DISPLAY': ':92.0'}):
                    cfg = self.load()
                self.assertEqual(cfg['recording']['video'], {'enabled': False, 'display': ':92.0',
                    'bottom_bar_px': 'auto'})
                self.assertEqual(TRAFFIC_LIGHT_DEBUG_TOPIC in cfg['recording']['topics'], enabled)
                self.assertEqual(self.document['recording']['video'], {'enabled': False})

    def test_debug_topic_is_added_once_to_custom_selection_and_only_when_traffic_is_on(self):
        self.document['stack'][DEBUG] = True
        self.document['recording']['selected_topics'] = ['/Ego_topic', TRAFFIC_LIGHT_DEBUG_TOPIC]
        self.assertEqual(self.load()['recording']['topics'].count(TRAFFIC_LIGHT_DEBUG_TOPIC), 1)
        self.document['recording']['selected_topics'] = ['/Ego_topic']
        self.document['stack']['traffic_light'] = False
        self.assertEqual(self.load()['recording']['topics'], ['/Ego_topic'])

    def test_new_snapshot_freezes_off_defaults_and_hash_protects_opt_in(self):
        path = self.save(); before = path.read_bytes()
        cfg = executions.prepare_execution(path, check_install=False)
        frozen = cfg['document']['stack']
        self.assertIs(frozen[YELLOW], False); self.assertIs(frozen[DEBUG], False)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(executions.load_execution(cfg['output'], check_install=False)['traffic_light_options'],
                         cfg['traffic_light_options'])
        changed = yaml.safe_load(cfg['path'].read_text()); changed['stack'][YELLOW] = True
        cfg['path'].write_text(yaml.safe_dump(changed))
        with self.assertRaisesRegex(ValueError, '변경'):
            executions.load_execution(cfg['output'], check_install=False)

    def test_debug_snapshot_freezes_display_options_and_topic_with_video_off(self):
        self.document['stack'].update({YELLOW: True, DEBUG: True})
        path = self.save()
        with patch.dict(os.environ, {'DISPLAY': ':93.0'}):
            cfg = executions.prepare_execution(path, check_install=False)
        self.document['stack'].update({YELLOW: False, DEBUG: False}); self.save()
        with patch.dict(os.environ, {'DISPLAY': ':94.0'}):
            resumed = executions.load_execution(cfg['output'], check_install=False)
        self.assertTrue(resumed['traffic_light_options']['yellow_passthrough'])
        self.assertTrue(resumed['traffic_light_options']['debug_view'])
        self.assertFalse(resumed['recording']['video']['enabled'])
        self.assertEqual(resumed['recording']['video']['display'], ':93.0')
        self.assertIn(TRAFFIC_LIGHT_DEBUG_TOPIC, resumed['recording']['topics'])

    def test_old_snapshot_retains_previous_implicit_yellow_and_image_publisher_without_viewer(self):
        path = self.save('config_snapshot.yaml'); before = path.read_bytes()
        cfg = load_campaign(path, check_install=False)
        self.assertEqual(cfg['traffic_light_options'],
                         {'yellow_passthrough': True, 'debug_view': False, 'publish_debug_image': True})
        self.assertEqual(path.read_bytes(), before)
        self.document['stack'].update({YELLOW: False, DEBUG: False})
        cfg = self.load('config_snapshot.yaml')
        self.assertEqual(cfg['traffic_light_options'], launch_options(self.document['stack']))

    def test_quick_test_preserves_or_explicitly_overrides_both_options(self):
        self.document['stack'].update({YELLOW: True, DEBUG: True})
        parameters = dict(detector='od_gt', pose_source='localization', traffic_light=True)
        self.assertTrue(quick_test_document(self.document, ['one.json'], **parameters)['stack'][YELLOW])
        result = quick_test_document(self.document, ['one.json'], **parameters,
            traffic_light_yellow_passthrough=False, traffic_light_debug_view=False)
        self.assertFalse(result['stack'][YELLOW]); self.assertFalse(result['stack'][DEBUG])
        self.assertTrue(self.document['stack'][YELLOW])

    def test_batch_preserves_each_campaign_options_and_frozen_resume_contract(self):
        first = self.save('first.yaml')
        self.document['campaign_name'] = 'traffic_enabled'
        self.document['stack'].update({YELLOW: True, DEBUG: True})
        second = self.save('second.yaml')
        configs, rows = batch.plan([first, second], check_install=False)
        self.assertFalse(configs[0]['traffic_light_options']['yellow_passthrough'])
        self.assertTrue(configs[1]['traffic_light_options']['yellow_passthrough'])
        self.assertNotEqual(rows[0]['contract']['yaml'], rows[1]['contract']['yaml'])
        for path in (first, second):
            frozen = executions.prepare_execution(path, check_install=False)
            resumed = executions.load_execution(frozen['output'], check_install=False)
            self.assertEqual(frozen['traffic_light_options'], resumed['traffic_light_options'])
            self.assertEqual(batch.input_contract(frozen), batch.input_contract(resumed))

    def test_runner_forwards_typed_flags_and_owns_viewer_only_when_enabled(self):
        for enabled in (False, True):
            for debug in (False, True):
                with self.subTest(traffic=enabled, debug=debug):
                    self.document['stack'].update(traffic_light=enabled,
                        traffic_light_yellow_passthrough=True, traffic_light_debug_view=debug)
                    cfg = self.load(); processes = Mock()
                    with patch.object(runner, 'runtime_settings', return_value=('/mock/python', '0')), \
                         patch.object(runner, 'candidate_assets', return_value=None):
                        runner._start_case_stack(cfg, processes, Mock())
                    commands = {c.args[0]: c.args[1] for c in processes.start.call_args_list}
                    self.assertIn('enable_yellow_passthrough:=true', commands['planning'])
                    self.assertEqual('traffic_light_debug_view' in commands, enabled and debug)
                    if enabled:
                        self.assertIn('publish_debug_image:='+str(debug).lower(), commands['traffic_light'])
                    if enabled and debug:
                        self.assertIn('display:='+cfg['recording']['video']['display'], commands['traffic_light_debug_view'])
                        self.assertIn('image_topic:='+TRAFFIC_LIGHT_DEBUG_TOPIC, commands['traffic_light_debug_view'])
        self.assertLess(runner.CASE_STACK.index('traffic_light_debug_view'), runner.CASE_STACK.index('traffic_light'))

    def test_new_runner_defaults_explicitly_forward_false_instead_of_parent_defaults(self):
        cfg = self.load(); processes = Mock()
        with patch.object(runner, 'runtime_settings', return_value=('/mock/python', '0')), \
             patch.object(runner, 'candidate_assets', return_value=None):
            runner._start_case_stack(cfg, processes, Mock())
        commands = {c.args[0]: c.args[1] for c in processes.start.call_args_list}
        self.assertIn('enable_yellow_passthrough:=false', commands['planning'])
        self.assertIn('signal_compliance_required_green_observations:=1', commands['planning'])
        self.assertIn('publish_debug_image:=false', commands['traffic_light'])
        self.assertNotIn('traffic_light_debug_view', commands)

    def test_timed_policy_has_typed_explicit_options_and_old_snapshot_shape_is_preserved(self):
        old = launch_options({}, legacy_snapshot=True)
        self.assertEqual(old, dict(yellow_passthrough=True, debug_view=False, publish_debug_image=True))
        self.assertEqual(timed_policy_options({})['signal_compliance_required_green_observations'], 1)
        self.document['stack'].update(signal_compliance_enable_timed_yellow=True,
            signal_compliance_required_green_observations=2,
            signal_compliance_min_yellow_sec=2., signal_compliance_yellow_time_margin_sec=.25)
        cfg = self.load(); options = cfg['traffic_light_options']
        self.assertTrue(options['signal_compliance_enable_timed_yellow'])
        self.assertEqual(options['signal_compliance_required_green_observations'], 2)
        # Candidate flag does not silently enable the master yellow switch.
        self.assertFalse(options['yellow_passthrough'])
        processes = Mock()
        with patch.object(runner, 'runtime_settings', return_value=('/mock/python', '0')), \
             patch.object(runner, 'candidate_assets', return_value=None):
            runner._start_case_stack(cfg, processes, Mock())
        command = next(c.args[1] for c in processes.start.call_args_list if c.args[0] == 'planning')
        for argument in ('signal_compliance_enable_timed_yellow:=false',
                         'signal_compliance_required_green_observations:=2',
                         'signal_compliance_min_yellow_sec:=2.0',
                         'signal_compliance_yellow_time_margin_sec:=0.25',
                         'enable_yellow_passthrough:=false'):
            self.assertIn(argument, command)
        # Advanced YAML fields survive both GUI save paths, without GUI migration.
        values = gui.form_values(self.document)
        self.assertEqual(gui.build_document(self.document, values), self.document)
        self.assertEqual(gui.operator_document(self.document, self.document, values, None), self.document)
        quick = quick_test_document(self.document, ['one.json'], detector='od_gt',
            pose_source='localization', traffic_light=True,
            traffic_light_yellow_passthrough=True)
        for key in TIMED_POLICY_DEFAULTS:
            self.assertEqual(quick['stack'][key], self.document['stack'][key])

    def test_timed_policy_validation_rejects_invalid_types_and_bounds(self):
        bad = {
            'signal_compliance_required_green_observations': (0, 4, 2., True, '2', None),
            'signal_compliance_enable_timed_yellow': (0, 1, 'false', None),
            'signal_compliance_min_yellow_sec': (True, '2', float('inf'), float('nan')),
            'signal_compliance_yellow_time_margin_sec': (True, '.2', float('inf'), float('nan')),
        }
        for key, values in bad.items():
            for value in values:
                with self.subTest(key=key, value=value):
                    doc = copy.deepcopy(self.document); doc['stack'][key] = value
                    with self.assertRaisesRegex(ConfigError, key):
                        validate_campaign_document(doc)
        # Explicit zero margin is valid for a diagnostic comparison.
        self.assertEqual(launch_options({'signal_compliance_yellow_time_margin_sec': 0})[
            'signal_compliance_yellow_time_margin_sec'], 0.)

    def test_new_snapshot_freezes_timed_defaults_and_changes_remain_hash_protected(self):
        path = self.save(); before = path.read_bytes()
        cfg = executions.prepare_execution(path, check_install=False)
        self.assertEqual(path.read_bytes(), before)
        for key, expected in TIMED_POLICY_DEFAULTS.items():
            self.assertEqual(cfg['document']['stack'][key], expected)
        frozen = cfg['path'].read_bytes()
        resumed = executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual(resumed['traffic_light_options'], cfg['traffic_light_options'])
        self.assertEqual(cfg['path'].read_bytes(), frozen)
        changed = yaml.safe_load(frozen); changed['stack']['signal_compliance_min_yellow_sec'] = 1.
        cfg['path'].write_text(yaml.safe_dump(changed))
        with self.assertRaisesRegex(ValueError, '변경'):
            executions.load_execution(cfg['output'], check_install=False)

    def test_launches_expose_and_forward_all_timed_policy_parameters(self):
        import xml.etree.ElementTree as ET
        launch = Path(__file__).resolve().parents[2]/'ajounice_planning_integration/launch'
        planning = ET.parse(launch/'planning.launch').getroot()
        stack = ET.parse(launch/'morai_planning_stack.launch').getroot()
        for key, default in TIMED_POLICY_DEFAULTS.items():
            with self.subTest(key=key):
                for xml in (planning, stack):
                    args = {e.get('name'): e.get('default') for e in xml.findall('arg')}
                    self.assertEqual(args[key], str(default).lower())
                params = {e.get('name'): e.get('value') for e in planning.findall('param')}
                expected = ('false'
                            if key == 'signal_compliance_enable_timed_yellow' else '$(arg '+key+')')
                self.assertEqual(params['/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light/'+key], expected)
                included = [e for e in stack.findall('include') if e.get('file', '').endswith('/launch/planning.launch')]
                self.assertEqual(len(included), 1)
                forwarded = {e.get('name'): e.get('value') for e in included[0].findall('arg')}
                self.assertEqual(forwarded[key], '$(arg '+key+')')

    def test_yellow_braking_deceleration_defaults_is_positive_finite_and_not_gui_migrated(self):
        self.assertEqual(yellow_braking_options({}), YELLOW_BRAKING_DEFAULTS)
        for key in YELLOW_BRAKING_DEFAULTS:
            for invalid in (0, -8, True, '8', None, float('inf'), float('nan')):
                with self.subTest(key=key, invalid=invalid):
                    document = copy.deepcopy(self.document)
                    document['stack'][key] = invalid
                    with self.assertRaisesRegex(ConfigError, key):
                        validate_campaign_document(document)
        self.document['stack']['signal_compliance_yellow_deceleration_mps2'] = 7.5
        self.document['stack']['signal_compliance_yellow_jerk_mps3'] = 14.
        values = gui.form_values(self.document)
        self.assertEqual(gui.build_document(self.document, values), self.document)
        self.assertEqual(gui.operator_document(self.document, self.document, values, None), self.document)
        cfg = self.load(); processes = Mock()
        with patch.object(runner, 'runtime_settings', return_value=('/mock/python', '0')), \
             patch.object(runner, 'candidate_assets', return_value=None):
            runner._start_case_stack(cfg, processes, Mock())
        command = next(call.args[1] for call in processes.start.call_args_list if call.args[0] == 'planning')
        self.assertIn('signal_compliance_yellow_deceleration_mps2:=7.5', command)
        self.assertIn('signal_compliance_yellow_jerk_mps3:=14.0', command)
        self.assertIn('enable_yellow_passthrough:=false', command)

    def test_old_timed_values_are_accepted_but_cannot_enable_the_replaced_branch(self):
        self.document['stack'].update(signal_compliance_enable_timed_yellow=True,
            signal_compliance_min_yellow_sec=-1., signal_compliance_yellow_time_margin_sec=20.)
        cfg = self.load(); processes = Mock()
        self.assertTrue(cfg['traffic_light_options']['signal_compliance_enable_timed_yellow'])
        with patch.object(runner, 'runtime_settings', return_value=('/mock/python', '0')), \
             patch.object(runner, 'candidate_assets', return_value=None):
            runner._start_case_stack(cfg, processes, Mock())
        command = next(call.args[1] for call in processes.start.call_args_list if call.args[0] == 'planning')
        self.assertIn('signal_compliance_enable_timed_yellow:=false', command)
        self.assertIn('signal_compliance_yellow_deceleration_mps2:=6.0', command)
        self.assertIn('signal_compliance_yellow_jerk_mps3:=10.0', command)
        self.assertEqual(self.document['stack']['signal_compliance_min_yellow_sec'], -1.)

    def test_new_snapshot_freezes_yellow_deceleration_and_records_replaced_policy_provenance(self):
        path = self.save(); before = path.read_bytes()
        cfg = executions.prepare_execution(path, check_install=False)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(cfg['document']['stack']['signal_compliance_yellow_deceleration_mps2'], 6.)
        self.assertEqual(cfg['document']['stack']['signal_compliance_yellow_jerk_mps3'], 10.)
        self.assertEqual(cfg['execution']['traffic_light_yellow_policy'], dict(
            implementation=YELLOW_POLICY_IMPLEMENTATION, deceleration_mps2=6., jerk_mps3=10.,
            timed_candidate_active=False))
        resumed = executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual(resumed['execution']['traffic_light_yellow_policy'],
                         cfg['execution']['traffic_light_yellow_policy'])
        changed = yaml.safe_load(cfg['path'].read_text())
        changed['stack']['signal_compliance_yellow_deceleration_mps2'] = 7.5
        cfg['path'].write_text(yaml.safe_dump(changed))
        with self.assertRaisesRegex(ValueError, '변경'):
            executions.load_execution(cfg['output'], check_install=False)

    def test_new_snapshot_jerk_parameter_is_hash_protected(self):
        cfg = executions.prepare_execution(self.save(), check_install=False)
        changed = yaml.safe_load(cfg['path'].read_text())
        changed['stack']['signal_compliance_yellow_jerk_mps3'] = 12.
        cfg['path'].write_text(yaml.safe_dump(changed))
        with self.assertRaisesRegex(ValueError, '변경'):
            executions.load_execution(cfg['output'], check_install=False)

    def test_explicit_original_braking_values_are_not_silently_migrated(self):
        self.document['stack'].update(signal_compliance_yellow_deceleration_mps2=8.,
                                      signal_compliance_yellow_jerk_mps3=16.)
        cfg = executions.prepare_execution(self.save(), check_install=False)
        self.assertEqual(cfg['execution']['traffic_light_yellow_policy']['deceleration_mps2'],8.)
        self.assertEqual(cfg['execution']['traffic_light_yellow_policy']['jerk_mps3'],16.)


@unittest.skipUnless(os.environ.get('DISPLAY'), 'requires isolated Xvfb')
class TrafficControlGuiTests(unittest.TestCase):
    setUp = TrafficControlsTests.setUp
    save = TrafficControlsTests.save

    def test_same_row_order_default_states_and_auto_save_check_without_running_any_ros(self):
        path = self.save(); captured = {}
        process = Mock(); process.poll.return_value = 0; process.wait.return_value = 0
        def ready(app):
            try:
                app['root'].withdraw(); app['actions']['open_yaml'](path)
                widgets = app['field_widgets']
                row = widgets['traffic_light'].master
                captured['order'] = [widget.cget('text') for widget in row.pack_slaves()]
                self.assertFalse(app['fields'][YELLOW].get()); self.assertFalse(app['fields'][DEBUG].get())
                app['fields'][YELLOW].set(True); app['fields'][DEBUG].set(True)
                app['actions']['check']()
                captured['saved'] = yaml.safe_load(path.read_text())
            finally:
                app['root'].destroy()
        with patch.object(gui, 'USERS', self.root), \
             patch.object(gui, 'gui_settings_path', return_value=self.root/'gui_settings.json'), \
             patch('tkinter.messagebox.showerror') as errors, \
             patch.object(gui.subprocess, 'Popen', return_value=process) as popen, \
             patch.object(gui.threading, 'Thread'):
            gui.main(on_ready=ready, runner_executable='/tmp/validator-mock-no-exec')
        errors.assert_not_called(); popen.assert_called_once()
        self.assertEqual(captured['order'][:4],
                         ['신호등 인지', '황색 통과 허용', '신호등 디버그 화면', '화면 영상 녹화'])
        self.assertTrue(captured['saved']['stack'][YELLOW]); self.assertTrue(captured['saved']['stack'][DEBUG])

    def test_start_with_minimal_video_mapping_keeps_video_off(self):
        self.document['recording']['video'] = {}
        self.test_new_start_and_quick_start_forward_gui_options_without_running_child()

    def test_new_start_and_quick_start_forward_gui_options_without_running_child(self):
        for action in ('start', 'quick_start'):
            with self.subTest(action=action):
                path = self.save(); captures = {}
                process = Mock(); process.poll.return_value = 0; process.wait.return_value = 0
                def prepare(saved_path, **_):
                    cfg = load_campaign(saved_path, check_install=False)
                    captures['prepared'] = cfg
                    return cfg
                def ready(app):
                    try:
                        app['root'].withdraw(); app['actions']['open_yaml'](path)
                        fields = app['quick_fields'] if action == 'quick_start' else app['fields']
                        fields[YELLOW].set(True); fields[DEBUG].set(True)
                        if action == 'quick_start':
                            app['quick_table'].selection_set('one.json')
                        app['actions'][action]()
                    finally:
                        app['root'].destroy()
                with patch.object(gui, 'USERS', self.root), \
                     patch.object(gui, 'PACKAGE', self.root/'package'), \
                     patch.object(gui, 'gui_settings_path', return_value=self.root/'gui_settings.json'), \
                     patch('tkinter.messagebox.askokcancel', return_value=True), \
                     patch('tkinter.messagebox.showerror') as errors, \
                     patch.object(gui.subprocess, 'Popen', return_value=process) as popen, \
                     patch.object(gui.threading, 'Thread'), \
                     patch.object(gui.executions, 'prepare_execution', side_effect=prepare):
                    gui.main(on_ready=ready, runner_executable='/tmp/validator-mock-no-exec')
                errors.assert_not_called(); popen.assert_called_once()
                self.assertTrue(captures['prepared']['traffic_light_options']['yellow_passthrough'])
                self.assertTrue(captures['prepared']['traffic_light_options']['debug_view'])
                if action == 'quick_start':
                    # Quick overrides are not persisted into the original authoring YAML.
                    saved = yaml.safe_load(path.read_text())['stack']
                    self.assertNotIn(YELLOW, saved); self.assertNotIn(DEBUG, saved)


if __name__ == '__main__':
    unittest.main()
