"""Fixed mission picker: scoped authoring files, no path-entry escape hatch."""
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from planning_scenario_validator import gui
from planning_scenario_validator.authoring import annotated_yaml


class MissionPickerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.missions = self.root / 'missions'
        self.document = gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')
        for mission in ('highway', 'roundabout', 'not_a_mission'):
            base = self.missions / mission / 'base'
            base.mkdir(parents=True)
            (base / 'scenarios').mkdir()
            (base / 'example.yaml').write_text(annotated_yaml(self.document))
            (base.parent / 'workspace').mkdir()
        context = patch.object(gui, 'MISSIONS', self.missions)
        context.start()
        self.addCleanup(context.stop)

    def test_only_fixed_missions_and_own_base_workspace_yamls_are_listed(self):
        workspace = self.missions / 'highway/workspace'
        (workspace / 'mine.yml').write_text(annotated_yaml(self.document))
        (workspace / 'scene.case.yaml').write_text('timeout_sec: 10')
        (workspace / 'other.yaml').symlink_to(self.missions / 'roundabout/base/example.yaml')
        (workspace / 'shared.yaml').symlink_to(self.missions / 'highway/base/example.yaml')
        outside = self.root / 'outside.yaml'
        outside.write_text(annotated_yaml(self.document))
        (workspace / 'external.yaml').symlink_to(outside)
        snapshot = workspace / 'run'
        snapshot.mkdir()
        (snapshot / 'execution.json').write_text('{}')
        (snapshot / 'config.yaml').write_text(annotated_yaml(self.document))
        self.assertEqual(gui.available_missions(), ['highway', 'roundabout'])
        self.assertEqual(set(gui.mission_yaml_choices('highway')),
                         {'예제 / example.yaml', '내 작업 / mine.yml'})
        with self.assertRaises(ValueError):
            gui.mission_yaml_choices('not_a_mission')

    def test_default_and_new_file_refresh_prefer_reusable_starter(self):
        base = self.missions / 'highway/base/example.yaml'
        self.assertEqual(gui.default_mission_yaml('highway'), base)
        starter = self.missions / 'highway/workspace/highway_starter.yaml'
        starter.write_text(annotated_yaml(self.document))
        self.assertEqual(gui.default_mission_yaml('highway'), starter)
        self.assertIn('내 작업 / highway_starter.yaml', gui.mission_yaml_choices('highway'))
        starter.unlink()
        self.assertEqual(gui.default_mission_yaml('highway'), base)

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_mission_switch_copy_list_refresh_restore_and_running_lock(self):
        failures = []

        def ready(app):
            try:
                picker = app['mission_box'].master
                labels = [widget.cget('text') for widget in picker.winfo_children()
                          if widget.winfo_class() == 'TLabel']
                self.assertEqual(set(labels), {'담당 미션', '실험 YAML'})
                buttons = [widget.cget('text') for widget in picker.winfo_children()
                           if widget.winfo_class() == 'TButton']
                self.assertIn('베이스 → 내 작업 폴더 복사 (최초 1회 실행)', buttons)
                app['mission_value'].set('고주로')
                app['actions']['select_mission']()
                self.assertEqual(app['yaml_value'].get(), '예제 / example.yaml')
                app['actions']['copy_workspace']()
                path = self.missions / 'highway/workspace/highway_starter.yaml'
                self.assertEqual(app['active']['source_path'], path)
                self.assertEqual(app['yaml_value'].get(), '내 작업 / highway_starter.yaml')
                new = path.parent / 'new_conditions.yaml'
                new.write_text(annotated_yaml(self.document))
                app['reload_button'].invoke()
                self.assertIn('내 작업 / new_conditions.yaml', app['picker_choices'])
                app['yaml_value'].set('내 작업 / new_conditions.yaml')
                app['actions']['select_yaml']()
                self.assertEqual(app['active']['source_path'], new)
                app['mission_value'].set('회전교차로')
                app['actions']['select_mission']()
                self.assertTrue(all(gui.mission_directory(p).name == 'roundabout'
                                    for p in app['picker_choices'].values()))
                app['actions']['set_running_ui'](True)
                for key in ('source_entry', 'mission_box'):
                    self.assertEqual(str(app[key].cget('state')), 'disabled')
                app['actions']['set_running_ui'](False)
                for key in ('source_entry', 'mission_box'):
                    self.assertEqual(str(app[key].cget('state')), 'readonly')
            except BaseException as error:
                failures.append(error)
            finally:
                app['root'].destroy()

        with patch.object(gui, 'ROOT', self.root), \
             patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'state')}), \
             patch('tkinter.messagebox.showerror') as errors:
            gui.main(on_ready=ready)
            errors.assert_not_called()
        if failures:
            raise failures[0]


if __name__ == '__main__':
    unittest.main()
