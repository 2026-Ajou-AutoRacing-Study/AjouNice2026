"""Filename selection and real Tk checklist; no simulator or vehicle control."""
import copy
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from planning_scenario_validator import executions, gui
from planning_scenario_validator.config import ConfigError, load_campaign, scenario_sources
from planning_scenario_validator.scenario_selection import Selection, choose_scenarios, matching_names


class SelectionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.scenes = self.root / 'scenarios'
        self.scenes.mkdir()
        self.names = ['high_baseline_00.json', 'high_normal_02.json', 'high_stress_03.json']
        for name in self.names:
            (self.scenes / name).write_text(json.dumps({
                'mapInfo': {'mapName': 'R_KR_PR_K-city_2025'},
                'egoVehicle': {'initPosition': {'pos': {'x': 0, 'y': 0}}}}))
        self.doc = copy.deepcopy(gui.load_document(gui.MISSIONS / 'highway/base/example.yaml'))
        self.doc.update(campaign_name='selection_test', output_root=str(self.root / 'results'))
        self.doc['scenarios'] = {'directory': str(self.scenes), 'glob': '*.json'}
        self.path = self.root / 'selection_test.yaml'

    def save(self):
        self.path.write_text(yaml.safe_dump(self.doc))
        return self.path

    def test_search_substring_wildcard_or_and_empty(self):
        self.assertEqual(matching_names(self.names, ''), self.names)
        self.assertEqual(matching_names(self.names, 'BASELINE'), self.names[:1])
        self.assertEqual(matching_names(self.names, 'high_??????_02.json'), self.names[1:2])
        self.assertEqual(matching_names(self.names, 'baseline, stress'), [self.names[0], self.names[2]])
        self.assertEqual(matching_names(self.names, ', ,'), self.names)
        self.assertEqual(matching_names(self.names, 'not_found'), [])

    def test_bulk_preserves_hidden_checks_and_empty_filter_is_safe(self):
        model = Selection(self.names, [])
        model.set_visible('baseline,normal', True)
        model.set_visible('stress', True)
        model.set_visible('normal', False)
        model.set_visible('nothing', False)
        self.assertEqual(model.result(), [self.names[0], self.names[2]])
        model.toggle([self.names[0], self.names[2]])
        with self.assertRaisesRegex(ValueError, '최소 한'):
            model.result()

    def test_many_scenes_selected_in_bulk(self):
        names = ['car_{:04d}.json'.format(i) for i in range(1000)]
        model = Selection(names, [])
        model.set_visible('car_0[12]*', True)
        self.assertEqual(len(model.result()), 200)
        model.set_visible('', True)
        self.assertEqual(len(model.result()), 1000)

    def test_missing_saved_names_remain_visible_until_unchecked(self):
        model = Selection(self.names, ['missing.json', self.names[0]])
        self.assertIn('missing.json', model.visible())
        with self.assertRaisesRegex(ValueError, '없어진'):
            model.result()
        model.set_visible('missing', False)
        self.assertEqual(model.result(), self.names[:1])

    def test_default_glob_and_explicit_include_override(self):
        cfg = load_campaign(self.save(), check_install=False)
        self.assertEqual(len(cfg['cases']), 3)
        self.doc['scenarios'].update(glob='*baseline*', include=[self.names[2], self.names[1]])
        cfg = load_campaign(self.save(), check_install=False)
        self.assertEqual([case.source.name for case in cfg['cases']], self.names[1:])
        (self.scenes / 'new.json').write_bytes((self.scenes / self.names[0]).read_bytes())
        self.assertEqual(len(load_campaign(self.path, check_install=False)['cases']), 2)
        self.doc['scenarios'].pop('include')
        self.doc['scenarios']['glob'] = '*.json'
        self.assertEqual(len(load_campaign(self.save(), check_install=False)['cases']), 4)

    def test_bad_includes_rejected_without_falling_back_to_all(self):
        for names in ([], None, 'one.json', ['../one.json'], ['/tmp/a.json'],
                      ['sub/a.json'], ['sub\\a.json'], ['a.yaml'], [1],
                      [self.names[0], self.names[0]], ['missing.json']):
            with self.subTest(names=names):
                self.doc['scenarios']['include'] = names
                with self.assertRaises(ConfigError):
                    load_campaign(self.save(), check_install=False)

    def test_selected_symlink_outside_folder_rejected(self):
        outside = self.root / 'outside.json'
        outside.write_text('{}')
        (self.scenes / 'escape.json').symlink_to(outside)
        with self.assertRaisesRegex(ConfigError, '폴더 밖'):
            scenario_sources({'directory': str(self.scenes), 'include': ['escape.json']})

    def test_yaml_form_roundtrip_preserves_selection(self):
        self.doc['scenarios']['include'] = self.names[:1]
        self.assertEqual(gui.build_document(self.doc, gui.form_values(self.doc)), self.doc)

    def test_snapshot_copies_only_selected_scenes_and_resume_is_independent(self):
        self.doc['scenarios']['include'] = self.names[:1]
        cfg = executions.prepare_execution(self.save(), check_install=False)
        self.assertEqual([p.name for p in (cfg['output'] / 'scenarios').iterdir()], self.names[:1])
        # Unselected JSON may even be invalid; it must not affect this execution.
        (self.scenes / self.names[1]).write_text('broken')
        (self.scenes / self.names[0]).unlink()
        self.doc['scenarios']['include'] = [self.names[2]]
        self.save()
        resumed = executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual([case.source.name for case in resumed['cases']], self.names[:1])

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'Tk requires X11/Xvfb')
    def test_real_dialog_bulk_apply_cancel_pattern_and_missing(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        captured = []

        def run(action, include=None, pattern='*.json'):
            def ready(app):
                def exercise():
                    try:
                        action(app)
                    except BaseException as error:
                        captured.append(error)
                        app['dialog'].destroy()
                root.after(10, exercise)
            return choose_scenarios(root, self.scenes, pattern, include, on_ready=ready)

        def select(app):
            self.assertFalse(app['model'].selected)
            app['model'].selected.clear()
            app['query'].set('baseline,stress')
            app['bulk'](True)
            app['query'].set('normal')
            self.assertEqual(len(app['model'].selected), 2)
            app['apply']()

        self.assertEqual(run(select), (True, [self.names[0], self.names[2]]))
        self.assertEqual(run(lambda app: app['dialog'].destroy(), self.names[:1]), (False, self.names[:1]))
        self.assertEqual(run(lambda app: app['all'](), self.names[:1]), (True, None))
        self.assertEqual(run(lambda app: app['all'](), self.names[:1], 'no_match*.json'), (True, None))
        def missing(app):
            self.assertIn('missing.json', app['tree'].get_children())
            app['query'].set('missing')
            app['bulk'](False)
            app['apply']()
        self.assertEqual(run(missing, [self.names[0], 'missing.json']), (True, self.names[:1]))
        # No matches in old glob should not prevent choosing another valid file.
        self.assertEqual(run(lambda app: (app['query'].set('baseline'), app['bulk'](True), app['apply']()),
                             pattern='no_match*.json'), (True, self.names[:1]))
        if captured:
            raise captured[0]

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'Tk requires X11/Xvfb')
    def test_check_refresh_preserves_scroll_highlight_and_keyboard_focus(self):
        import tkinter as tk
        for index in range(100):
            (self.scenes / 'car_{:03d}.json'.format(index)).write_text('{}')
        root = tk.Tk()
        self.addCleanup(root.destroy)
        captured = []

        def ready(app):
            def exercise():
                try:
                    tree = app['tree']
                    app['dialog'].update()
                    tree.yview_moveto(.5)
                    tree.selection_set('car_050.json')
                    tree.focus('car_050.json')
                    position = tree.yview()[0]
                    app['model'].toggle(tree.selection())
                    app['refresh']()
                    self.assertAlmostEqual(tree.yview()[0], position, places=2)
                    self.assertEqual(tree.selection(), ('car_050.json',))
                    self.assertEqual(tree.focus(), 'car_050.json')
                    app['query'].set('baseline')
                    self.assertEqual(tree.yview()[0], 0)
                except BaseException as error:
                    captured.append(error)
                finally:
                    app['dialog'].destroy()
            root.after(10, exercise)
        self.assertEqual(choose_scenarios(root, self.scenes, '*.json', None, on_ready=ready), (False, None))
        if captured:
            raise captured[0]

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'Tk requires X11/Xvfb')
    def test_main_gui_selection_load_save_clear_cancel_and_running_guard(self):
        captured = []
        self.save()

        def ready(app):
            def exercise():
                try:
                    app['actions']['open_yaml'](self.path)
                    with patch.object(gui, 'choose_scenarios', return_value=(True, self.names[:1])):
                        app['actions']['select_scenarios']()
                    self.assertNotIn('include', yaml.safe_load(self.path.read_text())['scenarios'])
                    app['actions']['save']()
                    self.assertEqual(yaml.safe_load(self.path.read_text())['scenarios']['include'], self.names[:1])
                    app['actions']['open_yaml'](self.path)
                    self.assertEqual(app['active']['scenario_include'], self.names[:1])
                    with patch.object(gui, 'choose_scenarios', return_value=(False, None)):
                        app['actions']['select_scenarios']()
                    self.assertEqual(app['active']['scenario_include'], self.names[:1])
                    with patch.object(gui, 'choose_scenarios', return_value=(True, None)):
                        app['actions']['select_scenarios']()
                    app['actions']['save']()
                    self.assertNotIn('include', yaml.safe_load(self.path.read_text())['scenarios'])
                    for key in ('process', 'check_process'):
                        from types import SimpleNamespace
                        app['active'][key] = SimpleNamespace(poll=lambda: None)
                        with patch.object(gui, 'choose_scenarios') as choose, \
                             patch('tkinter.messagebox.showerror') as error:
                            app['actions']['select_scenarios']()
                            choose.assert_not_called()
                            error.assert_called_once()
                        app['active'][key] = None
                except BaseException as error:
                    captured.append(error)
                finally:
                    app['root'].destroy()
            app['root'].after(10, exercise)

        with patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'settings')}), \
             patch.object(gui, 'USERS', self.root):
            gui.main(on_ready=ready)
        if captured:
            raise captured[0]


if __name__ == '__main__':
    unittest.main()
