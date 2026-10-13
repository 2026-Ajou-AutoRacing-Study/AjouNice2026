"""Repetitions, realized parameters and interruption/resume without a simulator."""
import copy
import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator import executions, gui, runner
from planning_scenario_validator.authoring import annotated_yaml, pasted_yaml_path, quick_test_document
from planning_scenario_validator.config import ConfigError, load_campaign, validate_campaign_document
from planning_scenario_validator.repetitions import repetition_settings, sampled_scene
from planning_scenario_validator.runtime import prepare_scenario


class RepetitionTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.doc = gui.load_document(gui.MISSIONS / 'highway/base/random_example.yaml')
        self.doc['scenarios']['directory'] = str(self.root / 'scenarios')
        (self.root / 'scenarios').mkdir()
        base = gui.MISSIONS / 'highway/base/scenarios'
        for name in self.doc['scenarios']['include']:
            (self.root / 'scenarios' / name).write_bytes((base / name).read_bytes())
        self.doc.update(output_root=str(self.root / 'results'), campaign_name='random_test')
        self.path = self.root / 'campaign.yaml'

    def config(self):
        self.path.write_text(annotated_yaml(self.doc), encoding='utf-8')
        return load_campaign(self.path, check_install=False)

    def test_round_robin_repetitions_are_unique_and_keep_original_sources(self):
        cfg = self.config()
        self.assertEqual(len(cfg['cases']), 20)
        self.assertEqual(len({c.source for c in cfg['cases']}), 2)
        self.assertEqual(len({c.name for c in cfg['cases']}), 20)
        self.assertEqual([c.repeat_index for c in cfg['cases'][:4]], [1, 1, 2, 2])
        self.assertTrue(cfg['cases'][0].name.endswith('_r001'))

    def test_sampled_parameters_preserve_placements_routes_and_originals(self):
        case = self.config()['cases'][0]
        scene = json.loads(case.source.read_text())
        original = copy.deepcopy(scene)
        sampled, audit = sampled_scene(scene, source_sha256=case.source_sha256,
                                       repeat_index=1, settings=case.randomization)
        again, _ = sampled_scene(scene, source_sha256=case.source_sha256,
                                repeat_index=1, settings=case.randomization)
        other, _ = sampled_scene(scene, source_sha256=case.source_sha256,
                                repeat_index=2, settings=case.randomization)
        self.assertEqual(sampled, again)
        self.assertNotEqual(sampled, other)
        self.assertEqual(scene, original)
        self.assertEqual(len(audit['changes']), 3)
        for before, after in zip(scene['spawnPointList'], sampled['spawnPointList']):
            speed = after['MinDesiredVelocity_Custom']
            self.assertTrue(50 <= speed <= 60)
            self.assertEqual(speed, after['MaxDesiredVelocity_Custom'])
            self.assertEqual(after['latBiasMode'], 2)
            self.assertTrue(-10 <= after['MinLatBias'] <= 10)
            self.assertEqual(after['MinLatBias'], after['MaxLatBias'])
            for key in ('pos', 'startLinkInfo', 'endLinkInfo', 'listVehicleLengthType', 'MinSpawnVelocity_Custom'):
                if key in before:
                    self.assertEqual(before[key], after[key])

    def test_spawn_and_placed_vehicle_offsets_use_percent_not_fractions(self):
        scene = {'egoVehicle': {'bias': 0}, 'vehicleList': [{'UNIQUEID': 1, 'bias': 0}],
                 'spawnPointList': [{'UNIQUEID': 2, 'MinLatBias': 0, 'MaxLatBias': 0}]}
        settings = {'enabled': True, 'path_offset_percent': [50, 50],
                    'vehicle_speed_kmh': [55, 55]}
        sampled, audit = sampled_scene(scene, source_sha256='abc', repeat_index=1,
                                       settings=settings)
        self.assertEqual(sampled['egoVehicle'], scene['egoVehicle'])
        self.assertEqual(sampled['vehicleList'][0]['bias'], 50)
        self.assertEqual(sampled['vehicleList'][0]['constantVelocity'], 55)
        self.assertEqual(sampled['spawnPointList'][0]['MinLatBias'], 50)
        self.assertEqual(sampled['spawnPointList'][0]['MaxLatBias'], 50)
        self.assertEqual([row['after']['latBiasMode'] for row in audit['changes']], [2, 2])

    def test_installed_json_matches_audit_and_preserves_source_hash(self):
        case = self.config()['cases'][0]
        install = self.root / 'install'
        install.mkdir()
        network = gui.resolve_path(self.doc['network']['ego_network_json'])
        before = case.source.read_bytes()
        path, _ = prepare_scenario(case, install, network)
        actual = json.loads(path.read_text())
        expected, audit = sampled_scene(json.loads(before), source_sha256=case.source_sha256,
                                       repeat_index=case.repeat_index, settings=case.randomization)
        self.assertEqual(actual['spawnPointList'], expected['spawnPointList'])
        self.assertEqual(case.source.read_bytes(), before)
        self.assertEqual(hashlib.sha256(before).hexdigest(), case.source_sha256)
        self.assertTrue(audit['enabled'])

    def test_snapshot_copies_unique_inputs_and_keeps_sampling_contract_on_resume(self):
        self.config()
        original = self.path.read_bytes()
        cfg = executions.prepare_execution(self.path, check_install=False)
        self.assertEqual(len(cfg['cases']), 20)
        self.assertEqual(sum(k.startswith('scenarios/') for k in cfg['execution']['files']), 2)
        self.assertEqual(executions.history(cfg['output'].parent)[0]['total'], 20)
        self.doc['scenarios']['repeat'] = 1
        self.path.write_text(yaml.safe_dump(self.doc))
        resumed = executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual(len(resumed['cases']), 20)
        self.assertEqual(cfg['cases'], resumed['cases'])
        self.assertEqual((cfg['output'] / 'source_config.yaml').read_bytes(), original)

    def test_invalid_repetitions_and_ranges_fail_before_launch(self):
        for value in (0, -1, True, '10', 1001, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                repetition_settings({'repeat': value})
        for field, value in (('vehicle_speed_kmh', [60, 50]), ('vehicle_speed_kmh', [50, float('nan')]),
                              ('path_offset_percent', [-101, 10]), ('seed', True), ('seed', -1),
                              ('enabled', 'true'), ('unknown', 1)):
            spec = {'enabled': True, 'vehicle_speed_kmh': [50, 60], field: value}
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                repetition_settings({'randomization': spec})

    def test_manual_abort_resumes_remaining_repetitions_not_completed_ones(self):
        self.doc['scenarios']['repeat'] = 2
        cfg = self.config()
        seen = []
        aborted = [False]

        def fake_case(config, processes, monitor, case, state):
            seen.append(case.name)
            abort = len(seen) == 2 and not aborted[0]
            if abort:
                aborted[0] = True
            status = 'ERROR' if abort else 'PASS'
            state['results'][case.name] = {'source_sha256': case.source_sha256, 'status': status,
                'reason': 'MANUAL_ABORT' if abort else 'SUCCESS', 'attempts': 1,
                'result_path': str(self.root / (case.name + '.json'))}
            return {'status': status, 'reason': state['results'][case.name]['reason']}

        with patch.dict(sys.modules, {'rospy': SimpleNamespace(init_node=Mock())}), \
             patch.object(runner, '_env', return_value={}), patch.object(runner, 'Processes'), \
             patch.object(runner, 'LiveMonitor'), patch.object(runner, '_start_master'), \
             patch.object(runner, '_start_persistent'), patch.object(runner, '_check_traffic_light'), \
             patch.object(runner, 'check_free_disk'), patch.object(runner.signal, 'signal'), \
             patch.object(runner, '_run_case', side_effect=fake_case):
            self.assertEqual(runner._run_campaign(cfg), 1)
            self.assertEqual(runner._run_campaign(cfg), 0)
        self.assertEqual(seen[0], cfg['cases'][0].name)
        self.assertEqual(seen[1], seen[2])
        self.assertEqual(len(seen), 5)

    def test_comment_renderer_and_quick_tests_preserve_fixed_contract(self):
        rendered = annotated_yaml(self.doc)
        self.assertEqual(yaml.safe_load(rendered), self.doc)
        self.assertIn('여기부터 수정 X', rendered)
        self.assertLess(rendered.index('repeat:'), rendered.index('여기부터 수정 X'))
        quick = quick_test_document(self.doc, self.doc['scenarios']['include'],
                                    detector='od_gt', pose_source='maptf_morai', traffic_light=False)
        validate_campaign_document(quick)
        self.assertEqual(quick['default_case'], self.doc['default_case'])
        self.assertEqual(quick['scenarios']['repeat'], 1)
        self.assertFalse(quick['scenarios']['randomization']['enabled'])
        self.assertFalse(quick['upload']['enabled'])
        self.assertEqual(self.doc['scenarios']['repeat'], 10)
        with self.assertRaises(ValueError):
            quick_test_document(self.doc, ['1.json', '2.json', '3.json'],
                                detector='od_gt', pose_source='maptf_morai', traffic_light=False)

    def test_vscode_path_paste_supports_quotes_spaces_tilde_and_relative_paths(self):
        for path in ('~/AjouNice2026/my.yaml', '/tmp/my file.yaml', 'configs/my.yaml'):
            self.assertEqual(pasted_yaml_path(' "{}" '.format(path)), path)
        for text in ('', '/tmp/a.json', 'one.yaml\ntwo.yaml'):
            with self.assertRaises(ValueError):
                pasted_yaml_path(text)

    def test_copy_selected_random_example_keeps_ranges_comments_and_reopens_same_yaml(self):
        missions = self.root / 'missions'
        base = missions / 'highway/base'
        (base / 'scenarios').mkdir(parents=True)
        for source in (self.root / 'scenarios').iterdir():
            (base / 'scenarios' / source.name).write_bytes(source.read_bytes())
        example = base / 'random_example.yaml'
        example.write_text(annotated_yaml(self.doc))
        with patch.object(gui, 'MISSIONS', missions), patch.object(gui, 'ROOT', self.root):
            path = gui.copy_base_to_workspace(example)
            doc = gui.load_document(path)
            self.assertEqual(doc['scenarios']['repeat'], 10)
            self.assertEqual(doc['scenarios']['randomization'], self.doc['scenarios']['randomization'])
            self.assertIn('여기부터 수정 X', path.read_text())
            original = path.read_bytes()
            self.assertEqual(gui.copy_base_to_workspace(path), path)
            self.assertEqual(path.read_bytes(), original)


@unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
class NewGuiTests(unittest.TestCase):
    def test_free_path_paste_cannot_change_readonly_yaml_selection(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'copied YAML.yaml'
            path.write_text(annotated_yaml(gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')))
            failures = []

            def ready(app):
                def exercise():
                    try:
                        root, entry = app['root'], app['source_entry']
                        original_path = app['active']['source_path']
                        original_label = app['yaml_value'].get()
                        self.assertEqual(str(entry.cget('state')), 'readonly')
                        entry.focus_force()
                        root.clipboard_clear()
                        root.clipboard_append('"{}"'.format(path))
                        entry.event_generate('<Control-a>')
                        entry.event_generate('<Control-v>')
                        root.update()
                        entry.event_generate('<Return>')
                        self.assertEqual(app['active']['source_path'], original_path)
                        self.assertEqual(app['yaml_value'].get(), original_label)
                    except BaseException as error:
                        failures.append(error)
                    finally:
                        app['root'].destroy()
                app['root'].after(30, exercise)

            with patch.dict(os.environ, {'XDG_CONFIG_HOME': temp}), \
                 patch('tkinter.messagebox.showerror') as errors:
                gui.main(on_ready=ready)
                errors.assert_not_called()
            if failures:
                raise failures[0]

    def test_fixed_mission_and_yaml_selection_keep_quick_selection(self):
        with tempfile.TemporaryDirectory() as temp:
            path = gui.MISSIONS / 'highway/base/random_example.yaml'
            original = path.read_bytes()

            def ready(app):
                try:
                    app['mission_value'].set(gui.MISSION_LABELS['highway'])
                    app['actions']['select_mission']()
                    self.assertTrue(all(gui.mission_directory(p).name == 'highway'
                                        for p in app['picker_choices'].values()))
                    app['yaml_value'].set('예제 / random_example.yaml')
                    app['actions']['select_yaml']()
                    self.assertEqual(app['active']['source_path'], path)
                    self.assertEqual(str(app['source_entry'].cget('state')), 'readonly')
                    self.assertEqual(str(app['mission_box'].cget('state')), 'readonly')
                    self.assertEqual(path.read_bytes(), original)
                    self.assertEqual(len(app['quick_table'].selection()), 2)
                    for key, widget in app['field_widgets'].items():
                        if key not in gui.EDITABLE_FIELDS:
                            self.assertIn(str(widget.cget('state')), ('readonly', 'disabled'))
                finally:
                    app['root'].destroy()

            with patch.dict(os.environ, {'XDG_CONFIG_HOME': temp}):
                gui.main(on_ready=ready)

    def test_quick_start_launches_two_frozen_inputs_without_changing_original(self):
        with tempfile.TemporaryDirectory() as temp:
            root_path = Path(temp)
            source = root_path / 'personal.yaml'
            doc = gui.load_document(gui.MISSIONS / 'highway/base/random_example.yaml')
            doc['output_root'] = str(root_path / 'results')
            source.write_text(annotated_yaml(doc))
            original = source.read_bytes()
            executable = root_path / 'fake_validator'
            executable.write_text('#!/usr/bin/python3\nprint("QUICK_STARTED", flush=True)\n')
            executable.chmod(0o755)
            failures = []

            def ready(app):
                def exercise():
                    try:
                        app['actions']['open_yaml'](source)
                        app['quick_fields']['detector'].set('od_gt')
                        app['actions']['quick_start']()
                        cfg = app['active']['execution']
                        self.assertEqual(len(cfg['cases']), 2)
                        self.assertEqual(cfg['stack']['detector'], 'od_gt')
                        self.assertTrue(all(c.repeat_total == 1 for c in cfg['cases']))
                        self.assertFalse(cfg['document']['scenarios']['randomization']['enabled'])
                        self.assertEqual(cfg['cases'][0].goal, doc['default_case']['goal'])
                        self.assertFalse(cfg['upload']['enabled'])
                        self.assertEqual(source.read_bytes(), original)
                        app['active']['process'].wait(timeout=2)
                        app['active']['process'] = None
                    except BaseException as error:
                        failures.append(error)
                    finally:
                        app['root'].destroy()
                app['root'].after(20, exercise)

            with patch.dict(os.environ, {'XDG_CONFIG_HOME': temp}), \
                 patch.object(gui, 'PACKAGE', root_path / 'package'), \
                 patch('tkinter.messagebox.askokcancel', return_value=True), \
                 patch('tkinter.messagebox.showerror') as errors:
                gui.main(on_ready=ready, runner_executable=executable)
                errors.assert_not_called()
            if failures:
                raise failures[0]


if __name__ == '__main__':
    unittest.main()
