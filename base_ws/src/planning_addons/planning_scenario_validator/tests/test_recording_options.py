"""Selectable/off bag, independent video, immutable resume and actual Tk controls."""
import copy
import csv
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock

import yaml

from planning_scenario_validator import gui, executions, batch, runner, recording_options as options
from planning_scenario_validator.config import load_campaign, resolve_path
from planning_scenario_validator.runtime import Recording


class RecordingOptionsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.doc = gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')
        self.doc['output_root'] = str(self.root / 'results')
        self.path = self.root / 'source.yaml'

    def cfg(self):
        self.path.write_text(yaml.safe_dump(self.doc, allow_unicode=True), encoding='utf-8')
        return load_campaign(self.path, check_install=False)

    def test_old_yaml_round_trip_and_bag_default_remain_identical(self):
        self.assertEqual(gui.build_document(self.doc, gui.form_values(self.doc)), self.doc)
        self.assertEqual(gui.operator_document(self.doc, self.doc, gui.form_values(self.doc),
                                              self.doc['scenarios'].get('include')), self.doc)
        self.assertNotIn('bag_enabled', self.doc['recording'])
        self.assertTrue(self.cfg()['recording']['topics'])

    def test_legacy_sensor_flag_displays_custom_and_noop_save_preserves_contract(self):
        self.doc['recording']['include_sensors'] = True
        values = gui.form_values(self.doc)
        self.assertEqual(values['recording_mode'], options.CUSTOM)
        result = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.assertEqual(result, self.doc)
        self.assertTrue(set(options.SENSORS) <= set(options.effective_topics(result['recording'], resolve_path)))

    def test_custom_selection_can_remove_legacy_sensors_without_silent_readdition(self):
        self.doc['recording']['include_sensors'] = True
        values = gui.form_values(self.doc)
        chosen = ['/Ego_topic', '/sensor/camera/front/compressed']
        values['selected_topics'] = json.dumps(chosen)
        result = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.assertFalse(result['recording']['include_sensors'])
        self.assertEqual(options.effective_topics(result['recording'], resolve_path), chosen)
        values = gui.form_values(self.doc)
        values['recording_mode'] = options.BASIC
        basic = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.assertFalse(set(options.SENSORS) & set(options.effective_topics(basic['recording'], resolve_path)))

    def test_off_needs_no_topic_profile_and_preserves_video(self):
        video = copy.deepcopy(self.doc['recording']['video'])
        values = gui.form_values(self.doc)
        values['recording_mode'] = options.OFF
        self.doc = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.assertEqual(self.doc['recording']['video'], video)
        self.doc['recording'].pop('topic_profile')
        cfg = self.cfg()
        self.assertFalse(cfg['recording']['bag_enabled'])
        self.assertEqual(cfg['recording']['topics'], [])
        snapshot = executions.prepare_execution(self.path, check_install=False)
        restored = executions.load_execution(snapshot['output'], check_install=False)
        self.assertFalse(restored['recording']['bag_enabled'])
        self.assertEqual(restored['recording']['topics'], [])
        self.assertIsNone(batch.input_contract(restored)['topic_profile'])

    def test_lightweight_select_does_not_force_video_on(self):
        self.doc['recording']['video']['enabled'] = False
        values = gui.form_values(self.doc)
        values['recording_mode'] = options.LIGHT
        self.doc = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.assertFalse(self.doc['recording']['video']['enabled'])
        self.assertFalse(self.doc['recording']['include_sensors'])
        snapshot = executions.prepare_execution(self.cfg()['path'], check_install=False)
        self.assertEqual(gui.form_values(snapshot['document'])['recording_mode'], options.LIGHT)
        self.assertEqual(len(snapshot['recording']['topics']), 13)

    def test_custom_exact_topics_roundtrip_snapshot_batch_and_hash_guard(self):
        chosen = ['/Ego_topic', '/planning/trajectory', '/sensor/camera/front/compressed']
        values = gui.form_values(self.doc)
        values.update(recording_mode=options.CUSTOM, selected_topics=json.dumps(chosen), include_sensors=False)
        self.doc = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.doc['recording']['topic_profile'] = '/nonexistent/profile.json'
        cfg = self.cfg()
        self.assertEqual(cfg['recording']['topics'], chosen)
        self.assertEqual(gui.build_document(self.doc, gui.form_values(self.doc)), self.doc)
        snapshot = executions.prepare_execution(self.path, check_install=False)
        restored = executions.load_execution(snapshot['output'], check_install=False)
        self.assertEqual(restored['recording']['topics'], chosen)
        self.assertEqual(options.mode_label(restored['recording']), options.CUSTOM)
        self.assertEqual(batch.input_contract(snapshot), batch.input_contract(restored))
        frozen = snapshot['path']
        changed = yaml.safe_load(frozen.read_text())
        changed['recording']['selected_topics'].append('/Object_topic')
        frozen.write_text(yaml.safe_dump(changed))
        with self.assertRaisesRegex(ValueError, '변경'):
            executions.load_execution(snapshot['output'], check_install=False)

    def test_switch_to_preset_removes_custom_list(self):
        self.doc['recording']['selected_topics'] = ['/Ego_topic']
        values = gui.form_values(self.doc)
        values.update(recording_mode=options.BASIC, selected_topics='')
        result = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        self.assertNotIn('selected_topics', result['recording'])
        self.assertEqual(options.mode_label(result['recording']), options.BASIC)

    def test_off_and_back_preserves_selection_until_preset_is_selected(self):
        self.doc['recording']['selected_topics'] = ['/Ego_topic']
        options.apply_mode(self.doc, options.OFF)
        self.assertEqual(options.effective_topics(self.doc['recording'], resolve_path), [])
        options.apply_mode(self.doc, options.CUSTOM)
        self.assertEqual(options.effective_topics(self.doc['recording'], resolve_path), ['/Ego_topic'])

    def test_invalid_enabled_empty_regex_and_selected_topic_types_refused(self):
        for value in ([], ['/Ego_topic', '--all'], ['/planning/.*'], '/Ego_topic', [None]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                options.effective_topics({'selected_topics': value}, resolve_path)
        with self.assertRaises(ValueError):
            options.effective_topics({'bag_enabled': 'false'}, resolve_path)

    def test_concurrent_yaml_selection_conflict_is_not_overwritten(self):
        self.doc['recording']['selected_topics'] = ['/Ego_topic']
        latest = copy.deepcopy(self.doc)
        latest['recording']['selected_topics'] = ['/Object_topic']
        values = gui.form_values(self.doc)
        values['selected_topics'] = json.dumps(['/CollisionData'])
        with self.assertRaisesRegex(ValueError, '양쪽'):
            gui.operator_document(self.doc, latest, values, self.doc['scenarios'].get('include'))

    def test_catalog_includes_preset_raw_sensor_and_custom_topics(self):
        catalog = options.topic_catalog(gui.ROOT, ['/custom/from_yaml'])
        self.assertIn('/custom/from_yaml', catalog)
        self.assertTrue(set(options.SENSORS) <= set(catalog))
        self.assertEqual(options.topic_group('/sensor/lidar1'), '원본 센서 (고용량)')

    def test_no_recording_spawns_nothing_and_stop_returns_none(self):
        recorder = Recording({'bag_enabled': False, 'topics': [], 'video': {'enabled': False}}, self.root, {})
        with patch('planning_scenario_validator.runtime.subprocess.Popen') as popen, \
             patch('planning_scenario_validator.runtime.time.sleep') as sleep:
            recorder.start('test')
            self.assertEqual(recorder.stop('test_F'), (None, None))
            popen.assert_not_called()
            sleep.assert_not_called()
        self.assertFalse((self.root / 'rosbag.log').exists())

    def test_video_only_starts_ffmpeg_not_rosbag_and_finalizes(self):
        recorder = Recording({'bag_enabled': False, 'topics': [], 'video': {'enabled': True, 'display': ':77'}}, self.root, {})
        proc = Mock()
        proc.poll.return_value = None
        with patch('planning_scenario_validator.runtime.subprocess.Popen', return_value=proc) as popen, \
             patch('planning_scenario_validator.runtime.time.sleep'), \
             patch('planning_scenario_validator.runtime.shutil.which', return_value='/usr/bin/ffmpeg'), \
             patch('planning_scenario_validator.runtime.video_size', return_value='800x600'), \
             patch('planning_scenario_validator.runtime.stop_group') as stop:
            recorder.start('video')
            recorder.video_path.touch()
            bag, video = recorder.stop('video_F')
            self.assertIsNone(bag)
            self.assertEqual(video, self.root / 'video_F.mp4')
            self.assertEqual(popen.call_count, 1)
            self.assertEqual(popen.call_args.args[0][0], 'ffmpeg')
            stop.assert_called_once_with(proc, timeout=15)

    def test_custom_recorder_launch_contains_only_requested_topics_no_regex(self):
        topics = ['/Ego_topic', '/Object_topic']
        recorder = Recording({'topics': topics}, self.root, {})
        proc = Mock()
        proc.poll.return_value = None
        with patch('planning_scenario_validator.runtime.subprocess.Popen', return_value=proc) as popen, \
             patch('planning_scenario_validator.runtime.time.sleep'), \
             patch('planning_scenario_validator.runtime.stop_group'):
            recorder.start('selected')
            Path(str(recorder.bag_prefix) + '.bag').touch()
            bag, video = recorder.stop('selected_P')
            self.assertEqual(popen.call_args.args[0][-2:], topics)
            self.assertNotIn('--regex', popen.call_args.args[0])
            self.assertIsNone(video)
            self.assertTrue(bag.is_file())

    def test_case_collision_without_recorders_still_writes_json_csv_and_failure_evidence(self):
        self.doc['stack']['detector'] = 'od_gt'
        self.doc['recording'].update(bag_enabled=False, video={'enabled': False})
        cfg = self.cfg()
        cfg['output'].mkdir(parents=True)
        state = runner._load_state(cfg, False)
        case = cfg['cases'][0]
        processes = Mock()
        processes.log_dir = self.root / 'stack_logs'
        processes.log_dir.mkdir()
        processes.env = dict(os.environ)
        processes.running = {'planning': Mock()}
        monitor = Mock()
        monitor.begin_case.side_effect = lambda path: Path(path).touch()
        monitor.run_until_done.return_value = ('COLLISION', .4)
        monitor.drive_evidence.return_value = {}
        monitor.collision_count = 1
        monitor.goal_min = float('inf')
        monitor.max_speed = 2.0
        with patch.object(runner, 'prepare_scenario', return_value=(case.source, (0, 0))), \
             patch.object(runner, 'BrakeHold'), patch.object(runner, '_load_scene'), \
             patch.object(runner, '_start_case_stack'), patch.object(runner, 'publish_brake'), \
             patch.object(runner, 'request_route', return_value='test-route'), \
             patch.object(runner.subprocess, 'run'), \
             patch.object(runner.subprocess, 'check_output', return_value='test-git-sha'), \
             patch('rospy.wait_for_message'):
            result = runner._run_case(cfg, processes, monitor, case, state)
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual(result['reason'], 'COLLISION')
        self.assertEqual(result['failure_analysis']['category'], 'COLLISION')
        self.assertEqual(result['files']['bag'], None)
        self.assertEqual(result['files']['video'], None)
        self.assertEqual(result['recording_audit']['status'], 'DISABLED')
        self.assertTrue(Path(state['results'][case.name]['result_path']).is_file())
        with (cfg['output'] / 'summary.csv').open() as stream:
            row = next(csv.DictReader(stream))
        self.assertEqual(row['status'], 'FAIL')
        self.assertEqual(row['bag_enabled'], 'False')
        self.assertEqual(row['video_enabled'], 'False')
        self.assertEqual(row['collision_count'], '1')
        self.assertEqual(list(cfg['output'].rglob('*.bag')), [])

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'Tk display required')
    def test_modal_group_search_bulk_preserves_hidden_choices(self):
        import tkinter as tk
        from tkinter import ttk
        root = tk.Tk()
        root.withdraw()
        failures = []

        def descendants(widget):
            for child in widget.winfo_children():
                yield child
                yield from descendants(child)

        def operate():
            try:
                window = next(w for w in root.winfo_children() if isinstance(w, tk.Toplevel))
                widgets = list(descendants(window))
                group = next(w for w in widgets if isinstance(w, ttk.Combobox))
                search = next(w for w in widgets if isinstance(w, ttk.Entry) and not isinstance(w, ttk.Combobox))
                buttons = {w.cget('text'): w for w in widgets if isinstance(w, ttk.Button)}
                window.setvar(group.cget('textvariable'), '원본 센서 (고용량)')
                buttons['표시된 토픽 모두 선택'].invoke()
                window.setvar(search.cget('textvariable'), 'front')
                buttons['표시된 토픽 모두 해제'].invoke()
                buttons['적용'].invoke()
            except BaseException as error:
                failures.append(error)
                root.destroy()

        root.after(50, operate)
        selected = options.choose_topics(root, ['/Ego_topic'] + options.SENSORS, ['/Ego_topic'])
        root.destroy()
        if failures:
            raise failures[0]
        self.assertEqual(set(selected), {'/Ego_topic', '/sensor/lidar1', '/sensor/camera/back/compressed'})

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'Tk display required')
    def test_real_gui_off_custom_save_reload_and_running_controls(self):
        self.cfg()
        failures = []

        def ready(app):
            try:
                app['actions']['open_yaml'](self.path)
                variables = app['fields']
                variables['recording_mode'].set(options.OFF)
                app['actions']['save']()
                app['actions']['open_yaml'](self.path)
                self.assertEqual(variables['recording_mode'].get(), options.OFF)
                self.assertNotIn('include_sensors', app['field_widgets'])
                variables['recording_mode'].set(options.CUSTOM)
                variables['selected_topics'].set(json.dumps(['/Ego_topic', '/Object_topic']))
                variables['include_sensors'].set(False)
                app['actions']['save']()
                app['actions']['open_yaml'](self.path)
                self.assertEqual(json.loads(variables['selected_topics'].get()), ['/Ego_topic', '/Object_topic'])
                self.assertEqual(variables['recording_mode'].get(), options.CUSTOM)
                app['actions']['set_running_ui'](True)
                self.assertEqual(str(app['field_widgets']['recording_mode']['state']), 'disabled')
                self.assertEqual(str(app['topics_button']['state']), 'disabled')
                app['actions']['set_running_ui'](False)
                self.assertEqual(str(app['field_widgets']['recording_mode']['state']), 'readonly')
            except BaseException as error:
                failures.append(error)
            finally:
                app['root'].destroy()

        with patch.object(gui, 'USERS', self.root), patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'gui_state')}), \
             patch('tkinter.messagebox.showerror') as errors:
            gui.main(on_ready=ready)
            errors.assert_not_called()
        if failures:
            raise failures[0]


if __name__ == '__main__':
    unittest.main()
