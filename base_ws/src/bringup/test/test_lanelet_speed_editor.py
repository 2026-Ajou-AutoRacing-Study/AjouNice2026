#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / 'validation/scripts'
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location('speed_editor', SCRIPTS / 'lanelet_speed_editor.py')
EDITOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EDITOR)
ROOT = Path(__file__).resolve().parents[4]


class LaneletSpeedEditorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.definition = yaml.safe_load((ROOT / 'base_ws/src/planning_addons/competition_route_requester/config/kcity_v87_competition_route.yaml').read_text())
        cls.road = EDITOR.RouteMap(ROOT, cls.definition)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / 'caps.yaml'
        self.model = EDITOR.SpeedSelections(self.road, self.path)

    def point(self, lane):
        line = self.road.lines[lane]
        # Two-point compacted centerlines must be sampled inside, not at their exit.
        lengths = [math.dist(a, b) for a, b in zip(line, line[1:])]
        target = sum(lengths) / 2
        for a, b, length in zip(line, line[1:], lengths):
            if target <= length:
                t = target / length
                return tuple(x + t * (y - x) for x, y in zip(a, b))
            target -= length
        raise AssertionError('empty centerline')

    def test_click_distinguishes_three_parallel_merge_lanes(self):
        for lane in (201439, 201441, 201446):
            self.assertEqual(self.model.hit(*self.point(lane)), lane)

    def test_toggle_saves_exact_id_and_reload_preserves_it(self):
        self.assertEqual(self.model.toggle(*self.point(201439)), 201439)
        data = yaml.safe_load(self.path.read_text())
        self.assertEqual(data['lanelet_speed_limits'], {'201439': 50.0})
        self.assertEqual(data['map_sha256'], self.road.fingerprint)
        reloaded = EDITOR.SpeedSelections(self.road, self.path)
        self.assertEqual(reloaded.selected, {201439: 50.0})
        reloaded.toggle(*self.point(201439))
        self.assertEqual(reloaded.selected, {})
        self.assertEqual(yaml.safe_load(self.path.read_text())['lanelet_speed_limits'], {})

    def test_adjacent_nonpreferred_lane_can_be_selected(self):
        self.assertNotIn(201441, self.model.route_ids)
        self.model.toggle(*self.point(201441))
        self.assertEqual(self.model.selected, {201441: 50.0})

    def test_off_road_and_nan_clicks_do_not_select_nearest_road(self):
        for point in ((1e6, 1e6, 28.4), (float('nan'), 400, 28.4)):
            with self.assertRaises(ValueError):
                self.model.toggle(*point)
        self.assertFalse(self.path.exists())
        self.assertEqual(self.model.selected, {})

    def test_undo_is_persistent_and_write_failure_keeps_state(self):
        self.model.toggle(*self.point(201439))
        with patch.object(self.model, 'save', side_effect=OSError('read only')):
            with self.assertRaises(OSError):
                self.model.toggle(*self.point(201441))
        self.assertEqual(self.model.selected, {201439: 50.0})
        self.model.undo()
        self.assertEqual(EDITOR.SpeedSelections(self.road, self.path).selected, {})

    def test_other_map_or_invalid_speed_is_rejected(self):
        self.model.toggle(*self.point(201439))
        data = yaml.safe_load(self.path.read_text())
        data['map_sha256'] = '0'*64
        self.path.write_text(yaml.safe_dump(data))
        with self.assertRaises(ValueError):
            EDITOR.SpeedSelections(self.road, self.path)
        for speed in (0, -1, 151, float('nan')):
            with self.assertRaises(ValueError):
                EDITOR.SpeedSelections(self.road, self.path, speed)

    def test_boundary_is_selectable_without_snapping_across_gap(self):
        square = [(0,0,0), (1,0,0), (1,1,0), (0,1,0)]
        self.assertTrue(EDITOR.contains(square, 1, 0.5))
        self.assertTrue(EDITOR.contains(square, 0.5, 0.5))
        self.assertFalse(EDITOR.contains(square, 1.1, 0.5))


if __name__ == '__main__':
    unittest.main()
