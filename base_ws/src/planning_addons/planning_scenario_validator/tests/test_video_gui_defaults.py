"""Enabling screen capture from minimal agent YAMLs resolves one stable display."""
import copy
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import yaml

from planning_scenario_validator import executions, gui, recording_options
from planning_scenario_validator.config import ROOT, load_campaign


EXAMPLE = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/base/localization_baseline.yaml'


class VideoGuiDefaultsTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.document = gui.load_document(EXAMPLE)
        self.document['output_root'] = str(self.root / 'results')
        self.document['recording']['video'] = {'enabled': False}
        self.path = self.root / 'personal.yaml'

    def save_input(self, document=None):
        self.path.write_text(yaml.safe_dump(document or self.document))
        return self.path

    def test_video_off_does_not_add_display_or_rewrite_an_existing_explicit_display(self):
        for video in ({'enabled': False}, {'enabled': False, 'display': ':11.0'}):
            with self.subTest(video=video):
                recording = {'video': copy.deepcopy(video)}
                recording_options.ensure_video_display(recording, environ={'DISPLAY': ':91.0'})
                self.assertEqual(recording['video'], video)

    def test_video_on_resolves_environment_display_or_fallback_only_when_missing(self):
        for video, environment, expected in [
                ({'enabled': True}, {'DISPLAY': ':91.0'}, ':91.0'),
                ({'enabled': True, 'display': ''}, {'DISPLAY': ' :92.0 '}, ':92.0'),
                ({'enabled': True, 'display': '   '}, {'DISPLAY': '   '}, ':0'),
                ({'enabled': True}, {}, ':0'),
                ({'enabled': True, 'display': ':17.0'}, {'DISPLAY': ':91.0'}, ':17.0')]:
            with self.subTest(video=video, environment=environment):
                recording = {'video': copy.deepcopy(video)}
                recording_options.ensure_video_display(recording, environ=environment)
                self.assertEqual(recording['video']['display'], expected)

    def test_operator_video_checkbox_adds_display_and_retains_unrelated_yaml_settings(self):
        original = copy.deepcopy(self.document)
        values = gui.form_values(self.document)
        values['video'] = True
        with patch.dict(os.environ, {'DISPLAY': ':91.0'}):
            result = gui.operator_document(self.document, self.document, values,
                self.document['scenarios'].get('include'))
        self.assertTrue(result['recording']['video']['enabled'])
        self.assertEqual(result['recording']['video']['display'], ':91.0')
        self.assertEqual(result['validation'], original['validation'])
        self.assertEqual(result['default_case'], original['default_case'])
        self.assertEqual(self.document, original)

    def test_legacy_build_document_video_checkbox_also_adds_display(self):
        values = gui.form_values(self.document)
        values['video'] = True
        with patch.dict(os.environ, {'DISPLAY': ':92.0'}):
            result = gui.build_document(self.document, values)
        self.assertTrue(result['recording']['video']['enabled'])
        self.assertEqual(result['recording']['video']['display'], ':92.0')

    def test_analysis_preset_forcing_video_on_also_resolves_display(self):
        for transform in ('operator', 'build'):
            with self.subTest(transform=transform):
                values = gui.form_values(self.document)
                values['recording_mode'] = recording_options.ANALYSIS
                self.assertFalse(values['video'])
                with patch.dict(os.environ, {'DISPLAY': ':93.0'}):
                    result = (gui.operator_document(self.document, self.document, values,
                        self.document['scenarios'].get('include')) if transform == 'operator'
                        else gui.build_document(self.document, values))
                self.assertTrue(result['recording']['video']['enabled'])
                self.assertEqual(result['recording']['video']['display'], ':93.0')
                self.assertTrue(result['recording']['include_sensors'])

    def test_concurrent_vscode_display_edit_is_retained_when_gui_enables_video(self):
        latest = copy.deepcopy(self.document)
        latest['recording']['video']['display'] = ':17.0'
        values = gui.form_values(self.document)
        values['video'] = True
        with patch.dict(os.environ, {'DISPLAY': ':93.0'}):
            result = gui.operator_document(self.document, latest, values,
                self.document['scenarios'].get('include'))
        self.assertEqual(result['recording']['video']['display'], ':17.0')
        self.assertTrue(result['recording']['video']['enabled'])

    def test_config_missing_display_is_runtime_normalized_without_rewriting_authoring_yaml(self):
        self.document['recording']['video']['enabled'] = True
        path = self.save_input()
        original = path.read_bytes()
        with patch.dict(os.environ, {'DISPLAY': ':94.0'}):
            cfg = load_campaign(path, check_install=False)
        self.assertEqual(cfg['recording']['video']['display'], ':94.0')
        self.assertEqual(cfg['document']['recording']['video']['display'], ':94.0')
        self.assertEqual(path.read_bytes(), original)

    def test_explicit_display_is_preserved_in_gui_save_and_config_loading(self):
        self.document['recording']['video']['display'] = ':17.0'
        values = gui.form_values(self.document)
        values['video'] = True
        with patch.dict(os.environ, {'DISPLAY': ':95.0'}):
            result = gui.operator_document(self.document, self.document, values,
                self.document['scenarios'].get('include'))
            self.save_input(result)
            cfg = load_campaign(self.path, check_install=False)
        self.assertEqual(cfg['recording']['video']['display'], ':17.0')

    def test_snapshot_freezes_inferred_display_across_resume_environment_changes(self):
        self.document['recording']['video']['enabled'] = True
        path = self.save_input()
        original = path.read_bytes()
        with patch.dict(os.environ, {'DISPLAY': ':96.0'}):
            cfg = executions.prepare_execution(path, check_install=False)
        snapshot = yaml.safe_load(cfg['path'].read_text())
        self.assertEqual(snapshot['recording']['video']['display'], ':96.0')
        with patch.dict(os.environ, {'DISPLAY': ':97.0'}):
            resumed = executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual(resumed['recording']['video']['display'], ':96.0')
        self.assertEqual(path.read_bytes(), original)


@unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
class VideoGuiAutoSaveTests(unittest.TestCase):
    setUp = VideoGuiDefaultsTests.setUp
    save_input = VideoGuiDefaultsTests.save_input

    def exercise_action(self, action):
        path = self.save_input()
        original = path.read_bytes()
        captures = {}
        process = Mock()
        process.poll.return_value = 0
        process.wait.return_value = 0
        def prepare(saved_path, **kwargs):
            captures['snapshot_input'] = Path(saved_path).read_bytes()
            return load_campaign(saved_path, check_install=False)
        def ready(app):
            try:
                app['root'].withdraw()
                app['actions']['open_yaml'](path)
                app['fields']['video'].set(True)
                self.assertEqual(path.read_bytes(), original)
                app['actions'][action]()
                captures['saved'] = yaml.safe_load(path.read_text())
                captures['process_started'] = app['active'].get('check_process' if action == 'check' else 'process')
            finally:
                app['root'].destroy()
        with patch.object(gui, 'USERS', self.root), \
             patch.object(gui, 'gui_settings_path', return_value=self.root / 'gui_settings.json'), \
             patch('tkinter.messagebox.askokcancel', return_value=True), \
             patch('tkinter.messagebox.showerror') as errors, \
             patch.object(gui.subprocess, 'Popen', return_value=process) as popen, \
             patch.object(gui.threading, 'Thread'), \
             patch.object(gui.executions, 'prepare_execution', side_effect=prepare):
            gui.main(on_ready=ready, runner_executable='/tmp/validator-test-no-exec')
        errors.assert_not_called()
        popen.assert_called_once()
        self.assertIs(captures['process_started'], process)
        self.assertTrue(captures['saved']['recording']['video']['enabled'])
        self.assertEqual(captures['saved']['recording']['video']['display'], os.environ['DISPLAY'].strip())
        if action == 'start':
            self.assertEqual(yaml.safe_load(captures['snapshot_input']), captures['saved'])
        else:
            self.assertIn('--dry-run', popen.call_args.args[0])

    def test_check_button_auto_saves_video_checkbox_without_separate_yaml_save_click(self):
        self.exercise_action('check')

    def test_start_button_auto_saves_video_checkbox_before_preparing_execution(self):
        self.exercise_action('start')


if __name__ == '__main__':
    unittest.main()
