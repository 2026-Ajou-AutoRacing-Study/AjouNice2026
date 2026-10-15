"""Minimal operator controls, comment-preserving save and fixed-field protection."""
import copy
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator import gui
from planning_scenario_validator.authoring import annotated_yaml, updated_yaml


class MinimalEditingTests(unittest.TestCase):
    def setUp(self):
        self.doc = gui.load_document(gui.MISSIONS / 'highway/base/random_example.yaml')

    def test_only_allowed_fields_merge_and_fixed_settings_ignore_form_injection(self):
        values = gui.form_values(self.doc)
        values.update(repeat='3', glob='high_baseline_*.json', detector='od_gt', pose_source='maptf_morai',
                      traffic_light=True, video=False, include_sensors=True, upload_enabled=True,
                      upload_profile=gui.UPLOAD_PROFILE_LABELS['video'], goal_x='999', sensor_json='bad')
        result = gui.operator_document(self.doc, self.doc, values, None)
        self.assertEqual(result['scenarios']['repeat'], 3)
        self.assertNotIn('include', result['scenarios'])
        self.assertEqual(result['default_case'], self.doc['default_case'])
        self.assertEqual(result['network'], self.doc['network'])
        self.assertEqual(result['scenarios']['randomization'], self.doc['scenarios']['randomization'])
        self.assertEqual(result['upload']['profile'], 'video')
        self.assertTrue(result['upload']['enabled'])
        self.assertFalse(result['recording']['video']['enabled'])

    def test_external_fixed_changes_merge_but_same_field_conflicts_fail(self):
        latest = copy.deepcopy(self.doc)
        latest['default_case']['goal']['x'] = 123
        values = gui.form_values(self.doc)
        values['detector'] = 'od_gt'
        result = gui.operator_document(self.doc, latest, values, self.doc['scenarios']['include'])
        self.assertEqual(result['default_case']['goal']['x'], 123)
        latest['scenarios']['repeat'] = 5
        values['repeat'] = '3'
        with self.assertRaisesRegex(ValueError, '양쪽'):
            gui.operator_document(self.doc, latest, values, self.doc['scenarios']['include'])

    def test_enabling_drive_fills_only_a_missing_destination(self):
        for remote in (None, ''):
            document = copy.deepcopy(self.doc)
            document['upload'].pop('rclone_remote', None)
            if remote is not None:
                document['upload']['rclone_remote'] = remote
            values = gui.form_values(document)
            unchanged = gui.operator_document(document, document, values,
                                              document['scenarios']['include'])
            self.assertEqual(unchanged, document)
            values['upload_enabled'] = True
            result = gui.operator_document(document, document, values,
                                           document['scenarios']['include'])
            self.assertEqual(result['upload']['rclone_remote'], gui.DEFAULT_UPLOAD_REMOTE)

    def test_enabling_drive_preserves_an_external_custom_destination(self):
        document = copy.deepcopy(self.doc)
        document['upload'].pop('rclone_remote')
        latest = copy.deepcopy(document)
        latest['upload']['rclone_remote'] = 'my_drive:validation'
        values = gui.form_values(document)
        values['upload_enabled'] = True
        result = gui.operator_document(document, latest, values,
                                       document['scenarios']['include'])
        self.assertEqual(result['upload']['rclone_remote'], 'my_drive:validation')

    def test_update_keeps_custom_comments_fixed_block_and_new_optional_fields(self):
        text = annotated_yaml(self.doc).replace('detector: bevfusion', 'detector: bevfusion # 내 인지 메모')
        text += '# 실험 메모도 유지\n'
        result = copy.deepcopy(self.doc)
        result['stack']['detector'] = 'od_gt'
        result['scenarios']['repeat'] = 4
        result['scenarios'].pop('include')
        output = updated_yaml(text, result)
        self.assertEqual(yaml.safe_load(output), result)
        self.assertIn('detector: od_gt # 내 인지 메모', output)
        self.assertIn('# 실험 메모도 유지', output)
        self.assertIn('여기부터 수정 X', output)
        self.assertEqual(updated_yaml(output, result), output)
        minimal = 'scenarios:\n  glob: "*.json" # 패턴\nstack: {detector: bevfusion}\n'
        target = yaml.safe_load(minimal)
        target['scenarios']['repeat'] = 2
        target['stack']['traffic_light'] = False
        target['upload'] = {'enabled': True, 'profile': 'bag'}
        patched = updated_yaml(minimal, target)
        self.assertEqual(yaml.safe_load(patched), target)
        self.assertIn('# 패턴', patched)

    def test_no_edit_does_not_rewrite_and_invalid_repeat_or_alias_is_safe(self):
        text = annotated_yaml(self.doc)
        self.assertEqual(updated_yaml(text, self.doc), text)
        for repeat in ('0', '1001', '1.5', 'no'):
            values = gui.form_values(self.doc)
            values['repeat'] = repeat
            with self.subTest(repeat=repeat), self.assertRaises(ValueError):
                gui.operator_document(self.doc, self.doc, values, self.doc['scenarios']['include'])
        with self.assertRaisesRegex(ValueError, '앵커'):
            updated_yaml('a: &a 1\nb: *a\n', {'a': 2, 'b': 1})


@unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
class MinimalGuiTests(unittest.TestCase):
    def test_save_button_updates_same_yaml_all_toggles_pattern_and_repeat(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'personal.yaml'
            doc = gui.load_document(gui.MISSIONS / 'highway/base/random_example.yaml')
            doc['output_root'] = str(Path(temp) / 'results')
            path.write_text('# 내 설명\n' + annotated_yaml(doc))
            failures = []

            def ready(app):
                try:
                    app['actions']['open_yaml'](path)
                    values = app['fields']
                    values['glob'].set('high_baseline_*.json')
                    self.assertIsNone(app['active']['scenario_include'])
                    values['repeat'].set('3')
                    values['detector'].set('od_gt')
                    values['pose_source'].set('maptf_morai')
                    for key in ('traffic_light', 'upload_enabled'):
                        values[key].set(True)
                    values['recording_mode'].set('직접 선택')
                    values['selected_topics'].set(json.dumps(['/Ego_topic'] + gui.bag_options.SENSORS))
                    values['video'].set(False)
                    values['upload_profile'].set(gui.UPLOAD_PROFILE_LABELS['bag'])
                    values['goal_x'].set('999')
                    app['root'].update_idletasks()
                    self.assertTrue(app['save_button'].winfo_ismapped())
                    self.assertEqual(app['save_button'].cget('text'), 'YAML 저장')
                    self.assertTrue(app['start_button'].winfo_ismapped())
                    self.assertEqual(app['start_button'].cget('text'), '▶ 새 실험 시작')
                    self.assertEqual(app['start_button'].cget('background'), '#e2eee6')
                    app['save_button'].invoke()
                    saved = gui.load_document(path)
                    self.assertTrue(path.read_text().startswith('# 내 설명'))
                    self.assertEqual(saved['default_case'], doc['default_case'])
                    self.assertEqual(saved['scenarios']['repeat'], 3)
                    self.assertNotIn('include', saved['scenarios'])
                    self.assertEqual(saved['stack']['detector'], 'od_gt')
                    self.assertEqual(saved['stack']['pose_source'], 'maptf_morai')
                    self.assertTrue(saved['stack']['traffic_light'])
                    self.assertFalse(saved['recording']['include_sensors'])
                    self.assertTrue(set(gui.bag_options.SENSORS) <= set(saved['recording']['selected_topics']))
                    self.assertFalse(saved['recording']['video']['enabled'])
                    self.assertTrue(saved['upload']['enabled'])
                    self.assertEqual(saved['upload']['profile'], 'bag')
                    self.assertEqual(app['active']['source_path'], path)
                    self.assertEqual(len(list(Path(temp).glob('*.yaml'))), 1)
                except BaseException as error:
                    failures.append(error)
                finally:
                    app['root'].destroy()

            with patch.dict(os.environ, {'XDG_CONFIG_HOME': temp}), \
                 patch('tkinter.messagebox.showerror') as errors:
                gui.main(on_ready=ready)
                errors.assert_not_called()
            if failures:
                raise failures[0]

    def test_vscode_save_requires_reload_to_update_display(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'personal.yaml'
            doc = gui.load_document(gui.MISSIONS / 'highway/base/random_example.yaml')
            doc['output_root'] = str(Path(temp) / 'results')
            path.write_text(annotated_yaml(doc))
            failures = []

            def ready(app):
                try:
                    app['actions']['open_yaml'](path)
                    doc['scenarios']['repeat'] = 3
                    doc['stack']['detector'] = 'od_gt'
                    path.write_text(annotated_yaml(doc))
                    app['root'].update_idletasks()
                    self.assertEqual(app['fields']['repeat'].get(), '10')
                    self.assertEqual(app['fields']['detector'].get(), 'bevfusion')
                    self.assertEqual(app['reload_button'].cget('text'), '불러오기/새로고침')
                    app['reload_button'].invoke()
                    self.assertEqual(app['fields']['repeat'].get(), '3')
                    self.assertEqual(app['fields']['detector'].get(), 'od_gt')
                except BaseException as error:
                    failures.append(error)
                finally:
                    app['root'].destroy()

            with patch.dict(os.environ, {'XDG_CONFIG_HOME': temp}), \
                 patch('tkinter.messagebox.showerror') as errors:
                gui.main(on_ready=ready)
                errors.assert_not_called()
            if failures:
                raise failures[0]

    def test_primary_actions_group_order_runtime_lock_and_monitor_restore(self):
        with tempfile.TemporaryDirectory() as temp:
            failures = []

            def ready(app):
                try:
                    root = app['root']
                    root.geometry('850x650')
                    root.update_idletasks()
                    buttons = app['primary_controls'].pack_slaves()
                    self.assertEqual([button.cget('text') for button in buttons],
                                     ['YAML 저장', '불러오기/새로고침', '▶ 새 실험 시작'])
                    self.assertLess(app['start_button'].winfo_rootx() + app['start_button'].winfo_width(),
                                    app['auxiliary_button'].winfo_rootx())
                    self.assertEqual([app['auxiliary_menu'].entrycget(i, 'label') for i in (0, 1, 3)],
                                     ['설정 검사', '실험 기록', '결과 폴더'])
                    app['actions']['set_running_ui'](True)
                    self.assertTrue(all(button.cget('state') == 'disabled' for button in buttons))
                    self.assertEqual(app['auxiliary_menu'].entrycget(0, 'state'), 'disabled')
                    self.assertEqual(str(app['stop_button'].cget('state')), 'normal')
                    app['actions']['set_running_ui'](False)
                    self.assertTrue(all(button.cget('state') == 'normal' for button in buttons))
                    app['actions']['compact'](True)
                    root.update_idletasks()
                    self.assertFalse(app['primary_controls'].winfo_ismapped())
                    self.assertTrue(app['compact_button'].winfo_ismapped())
                    app['actions']['compact'](False)
                    root.update_idletasks()
                    self.assertTrue(app['primary_controls'].winfo_ismapped())
                    self.assertEqual(app['primary_controls'].pack_slaves(), buttons)
                except BaseException as error:
                    failures.append(error)
                finally:
                    app['root'].destroy()

            with patch.dict(os.environ, {'XDG_CONFIG_HOME': temp}):
                gui.main(on_ready=ready)
            if failures:
                raise failures[0]

    def test_dialog_selection_is_saved_and_pattern_can_replace_it(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'personal.yaml'
            doc = gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')
            doc['output_root'] = str(Path(temp) / 'results')
            path.write_text(annotated_yaml(doc))
            failures = []

            def ready(app):
                try:
                    app['actions']['open_yaml'](path)
                    chosen = ['high_baseline_01.json']
                    with patch.object(gui, 'choose_scenarios', return_value=(True, chosen)):
                        app['actions']['select_scenarios']()
                    app['actions']['save']()
                    self.assertEqual(gui.load_document(path)['scenarios']['include'], chosen)
                    app['fields']['glob'].set('high_normal_*.json')
                    app['actions']['save']()
                    self.assertNotIn('include', gui.load_document(path)['scenarios'])
                except BaseException as error:
                    failures.append(error)
                finally:
                    app['root'].destroy()

            with patch.dict(os.environ, {'XDG_CONFIG_HOME': temp}):
                gui.main(on_ready=ready)
            if failures:
                raise failures[0]


if __name__ == '__main__':
    unittest.main()
