"""Full-course source preservation and offline campaign contracts, not live proof."""
import copy
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
sys.path.insert(0, str(PACKAGE / 'scripts'))
from full_course_starter import ROUTE_SOURCE
from planning_scenario_validator import gui
from planning_scenario_validator.config import ROOT, load_campaign


class FullCourseTests(unittest.TestCase):
    def setUp(self):
        self.folder = PACKAGE / 'config/missions/full_course/base'
        self.sample = json.loads((ROOT / 'configs/scenario/2026_molit_comp_sample_scene.json').read_text())
        self.scenes = {path.stem: json.loads(path.read_text()) for path in (self.folder / 'scenarios').glob('*.json')}

    def test_baseline_preserves_sample_and_variants_only_change_identity(self):
        self.assertEqual(self.scenes['full_baseline_00_sample'], self.sample)
        for scene in self.scenes.values():
            normalized = copy.deepcopy(scene)
            for key in ('vehicleList', 'objectList', 'pedestrianList', 'spawnPointList'):
                self.assertEqual([x['UNIQUEID'] for x in scene[key]], [x['UNIQUEID'] for x in self.sample[key]])
                for actor, original in zip(normalized[key], self.sample[key]):
                    for field in ('DataID', 'spawnVehicleTypeID', 'scale'):
                        if field in original:
                            actor[field] = copy.deepcopy(original[field])
            self.assertEqual(normalized, self.sample)
        for stem in ('full_normal_02_obstacle_types', 'full_normal_03_mixed_types'):
            actor = self.scenes[stem]['objectList'][0]
            self.assertEqual(actor['DataID'], 40100005)
            self.assertEqual([actor['scale'][a] for a in ('x', 'y', 'z')], [1, 1, 1])
        self.assertNotEqual(self.scenes['full_normal_01_vehicle_types']['vehicleList'], self.sample['vehicleList'])

    def test_ordered_route_a_non_start_goal_and_full_stack(self):
        route = yaml.safe_load((ROOT / ROUTE_SOURCE).read_text())
        cfg = load_campaign(self.folder / 'example.yaml', check_install=False)
        common = cfg['document']['default_case']
        refs = route['route_a']['waypoint_refs']
        self.assertEqual(common['waypoints'], [{k: route['poses'][ref][k] for k in ('x', 'y', 'z', 'yaw_deg')} for ref in refs])
        goal = common['goal']
        start = self.sample['egoVehicle']['initPosition']['pos']
        translation = yaml.safe_load((ROOT / 'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']['xyz']
        self.assertGreater(math.hypot(goal['x'] - start['x'] - translation[0], goal['y'] - start['y'] - translation[1]), 20)
        self.assertEqual(common['timeout_sec'], 600)
        self.assertEqual(cfg['stack']['pose_source'], 'localization')
        self.assertEqual(cfg['stack']['detector'], 'bevfusion')
        self.assertTrue(cfg['stack']['traffic_light'])
        self.assertEqual(cfg['stack']['traffic_light_runtime'], 'virtualenv')
        self.assertEqual(len(cfg['cases']), 4)
        single = load_campaign(self.folder / 'sample_only.yaml', check_install=False)
        self.assertEqual(len(single['cases']), 1)
        self.assertEqual(single['cases'][0].source.stem, 'full_baseline_00_sample')
        self.assertEqual(single['document']['default_case'], common)
        self.assertIn('full_course', gui.available_missions())

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_gui_select_and_workspace_copy_keep_route_and_sensor_choices(self):
        failures = []
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            missions = root / 'missions'
            base = missions / 'full_course/base'
            base.mkdir(parents=True)
            import shutil
            shutil.copytree(self.folder, base, dirs_exist_ok=True)

            def ready(app):
                try:
                    self.assertEqual(app['mission_value'].get(), '전체 코스 주행')
                    app['yaml_value'].set('예제 / sample_only.yaml')
                    app['actions']['select_yaml']()
                    app['actions']['copy_workspace']()
                    path = app['active']['source_path']
                    self.assertEqual(path, missions / 'full_course/workspace/sample_only.yaml')
                    personal = yaml.safe_load(path.read_text())
                    original = yaml.safe_load((base / 'sample_only.yaml').read_text())
                    self.assertEqual(personal['default_case'], original['default_case'])
                    self.assertTrue(personal['stack']['traffic_light'])
                    self.assertFalse(personal['recording']['include_sensors'])
                    self.assertTrue((path.parent / 'scenarios/full_baseline_00_sample.json').is_file())
                except BaseException as exc:
                    failures.append(exc)
                finally:
                    app['root'].destroy()

            with patch.object(gui, 'ROOT', root), patch.object(gui, 'MISSIONS', missions), \
                 patch.dict(os.environ, {'XDG_CONFIG_HOME': str(root / 'state')}), \
                 patch('tkinter.messagebox.showerror') as errors:
                gui.main(on_ready=ready)
                errors.assert_not_called()
            if failures:
                raise failures[0]
