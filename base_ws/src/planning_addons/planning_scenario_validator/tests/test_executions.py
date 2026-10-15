"""Frozen execution inputs, editable source YAMLs and CLI/GUI resume, without ROS."""
import copy
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from planning_scenario_validator import executions, gui, runner


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root / 'highway_bev.yaml'
        self.scenes = self.root / 'scenarios'
        self.scenes.mkdir()
        for name in ('one', 'two'):
            (self.scenes / (name + '.json')).write_text(json.dumps({
                'mapInfo': {'mapName': 'R_KR_PR_K-city_2025'},
                'egoVehicle': {'initPosition': {'pos': {'x': 0, 'y': 0}}}}))
        self.doc = copy.deepcopy(gui.load_document(gui.MISSIONS / 'highway/base/example.yaml'))
        self.doc.update(campaign_name='highway_bev', output_root=str(self.root / 'results'))
        self.doc['scenarios'] = {'directory': str(self.scenes), 'glob': '*.json'}
        for key in ('sensor_json', 'ego_network_json'):
            path = self.root / (key + '.json')
            path.write_bytes(gui.resolve_path(self.doc['network'][key]).read_bytes())
            self.doc['network'][key] = str(path)
        profile = self.root / 'topics.json'
        profile.write_text('["/Ego_topic"]')
        self.doc['recording']['topic_profile'] = str(profile)
        self.doc['recording']['video']['enabled'] = False
        self.doc['stack']['detector'] = 'od_gt'
        self.source.write_text(yaml.safe_dump(self.doc))

    def prepare(self):
        return executions.prepare_execution(self.source, check_install=False)

    def test_repeated_execution_keeps_yaml_and_creates_unique_directories(self):
        original = self.source.read_bytes()
        first, second = self.prepare(), self.prepare()
        self.assertNotEqual(first['output'], second['output'])
        self.assertRegex(first['name'], r'^highway_bev_\d{6}_\d{6}(?:_\d+)?$')
        self.assertEqual(self.source.read_bytes(), original)
        self.assertEqual(first['execution']['experiment_name'], 'highway_bev')
        self.assertEqual((first['output'] / 'source_config.yaml').read_bytes(), original)
        self.assertEqual(len(first['cases']), 2)

    def test_edit_or_delete_originals_does_not_change_resume_inputs(self):
        first = self.prepare()
        frozen = first['path'].read_bytes()
        self.doc['default_case']['goal']['x'] = 123
        gui.save_personal(self.doc, self.source)
        (self.scenes / 'one.json').unlink()
        Path(self.doc['network']['sensor_json']).unlink()
        Path(self.doc['recording']['topic_profile']).unlink()
        resumed = executions.load_execution(first['output'], check_install=False)
        self.assertEqual(resumed['path'].read_bytes(), frozen)
        self.assertEqual(len(resumed['cases']), 2)
        self.assertNotEqual(resumed['cases'][0].goal['x'], 123)

    def test_long_experiment_label_is_preserved_but_execution_id_fits_contract(self):
        self.doc['campaign_name'] = 'h' * 60
        self.source.write_text(yaml.safe_dump(self.doc))
        first, second = self.prepare(), self.prepare()
        self.assertLessEqual(len(first['name']), 60)
        self.assertLessEqual(len(second['name']), 60)
        self.assertEqual(first['execution']['experiment_name'], 'h' * 60)

    def test_snapshot_edit_rejected_before_resume_and_state_preserved(self):
        cfg = self.prepare()
        state = cfg['output'] / 'campaign_state.json'
        state.write_text('{"results": {}}')
        frozen = cfg['output'] / 'scenarios/one.json'
        frozen.write_text(frozen.read_text() + '\n')
        with self.assertRaisesRegex(ValueError, '사본'):
            executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual(state.read_text(), '{"results": {}}')

    def test_added_snapshot_scene_is_rejected(self):
        cfg = self.prepare()
        (cfg['output'] / 'scenarios/extra.json').write_bytes(cfg['cases'][0].source.read_bytes())
        with self.assertRaisesRegex(ValueError, '목록'):
            executions.load_execution(cfg['output'], check_install=False)

    def test_bad_manifest_and_legacy_state_raise_clear_errors(self):
        cfg = self.prepare()
        manifest = cfg['output'] / 'execution.json'
        manifest.write_text('[]')
        with self.assertRaisesRegex(ValueError, '형식'):
            executions.load_execution(cfg['output'], check_install=False)
        manifest.unlink()
        (cfg['output'] / 'campaign_state.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'execution.json'):
            executions.load_execution(cfg['output'], check_install=False)
        legacy = self.root / 'legacy'
        legacy.mkdir()
        (legacy / 'campaign_state.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'state 형식'):
            executions.load_execution(legacy, check_install=False)

    def test_dead_runner_history_is_interrupted_not_running(self):
        cfg = self.prepare()
        (cfg['output'] / 'campaign_state.json').write_text(json.dumps({
            'run_status': 'RUNNING', 'runner_pid': -1, 'current': {'phase': 'DRIVING'},
            'total': 2, 'results': {}}))
        self.assertEqual(executions.history(cfg['output'].parent)[0]['status'], 'INTERRUPTED')

    def test_snapshot_cannot_be_edited_or_used_as_new_authoring_yaml(self):
        cfg = self.prepare()
        original = cfg['path'].read_bytes()
        with self.assertRaisesRegex(ValueError, '사본'):
            gui.save_personal(self.doc, cfg['path'])
        with self.assertRaisesRegex(ValueError, '사본'):
            executions.prepare_execution(cfg['path'], check_install=False)
        self.assertEqual(cfg['path'].read_bytes(), original)

    def test_failed_copy_leaves_no_partial_execution(self):
        with patch.object(executions, 'atomic_json', side_effect=OSError('disk full')):
            with self.assertRaisesRegex(OSError, 'disk full'):
                self.prepare()
        self.assertEqual(list((self.root / 'results').iterdir()), [])

    def test_history_counts_and_damaged_records_remain_visible(self):
        first, second = self.prepare(), self.prepare()
        (first['output'] / 'campaign_state.json').write_text(json.dumps({
            'total': 2, 'results': {'one': {'status': 'PASS'}, 'two': {'status': 'FAIL'}}}))
        (second['output'] / 'execution.json').write_text('damaged')
        rows = executions.history(self.root / 'results')
        statuses = {row['directory']: row for row in rows}
        self.assertEqual(statuses[str(first['output'])]['status'], 'DONE')
        self.assertEqual(statuses[str(first['output'])]['counts']['FAIL'], 1)
        self.assertEqual(statuses[str(second['output'])]['status'], 'DAMAGED')

    def test_cli_new_and_resume_are_distinct_and_check_does_not_create_results(self):
        actual_load = runner.load_campaign
        actual_execution = runner.load_execution
        actual_prepare = runner.prepare_execution
        with patch.object(runner, 'load_campaign', side_effect=lambda path: actual_load(path, check_install=False)), \
             patch.object(runner, 'load_execution', side_effect=lambda path: actual_execution(path, check_install=False)), \
             patch.object(runner, 'prepare_execution', side_effect=lambda path: actual_prepare(path, check_install=False)), \
             patch.object(runner.signal, 'signal'), patch.object(runner, 'run', return_value=0) as run:
            self.assertEqual(runner.main([str(self.source), '--check']), 0)
            self.assertFalse((self.root / 'results').exists())
            self.assertEqual(runner.main([str(self.source)]), 0)
            first = run.call_args.args[0]
            self.assertEqual(runner.main([str(self.source)]), 0)
            second = run.call_args.args[0]
            self.assertNotEqual(first['output'], second['output'])
            self.source.unlink()
            self.assertEqual(runner.main(['--execution', str(first['output'])]), 0)
            self.assertEqual(run.call_args.args[0]['path'], first['path'])

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_gui_new_reuses_yaml_and_resume_uses_frozen_path(self):
        executable = self.root / 'fake-validator'
        executable.write_text('#!/usr/bin/python3\nprint("fake run")\n')
        executable.chmod(0o755)
        captured = []
        def ready(app):
            try:
                app['actions']['open_yaml'](self.source)
                with patch('tkinter.messagebox.askokcancel', return_value=True):
                    app['actions']['start']()
                first = app['active']['execution']
                self.assertIsNotNone(first)
                app['active']['process'].wait(timeout=3)
                app['active']['process'] = None
                self.doc['default_case']['goal']['x'] = 321
                self.source.write_text(yaml.safe_dump(self.doc))
                self.assertEqual(app['actions']['save'](), self.source)
                with patch('tkinter.messagebox.askokcancel', return_value=True):
                    app['actions']['start']()
                second = app['active']['execution']
                app['active']['process'].wait(timeout=3)
                app['active']['process'] = None
                self.assertNotEqual(first['output'], second['output'])
                self.assertEqual(second['cases'][0].goal['x'], 321)
                app['actions']['select_execution'](first['output'])
                with patch('tkinter.messagebox.askokcancel', return_value=True):
                    app['actions']['resume_execution'](first['output'])
                self.assertEqual(app['active']['process'].args[1], str(first['path']))
                app['active']['process'].wait(timeout=3)
                app['active']['process'] = None
                self.assertNotEqual(first['cases'][0].goal['x'], 321)
                self.assertEqual(len(app['history_table'].get_children()), 2)
                captured.append(True)
            finally:
                app['root'].destroy()
        with patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'gui')}), \
             patch.object(gui, 'USERS', self.root):
            gui.main(on_ready=ready, runner_executable=executable)
        self.assertEqual(captured, [True])
