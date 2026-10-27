#!/usr/bin/env python3
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'validation/scripts/planning_route_editor.py'
SPEC = importlib.util.spec_from_file_location('planning_route_editor', SCRIPT)
EDITOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EDITOR)
ROOT = Path(__file__).resolve().parents[4]


class RouteEditorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        definition = yaml.safe_load((ROOT / 'base_ws/src/planning_addons/competition_route_requester/config/kcity_v96_competition_route.yaml').read_text())
        cls.road = EDITOR.RouteMap(ROOT, definition)

    def setUp(self):
        self.model = EDITOR.RouteDrafts(self.road)

    def test_original_routes_follow_all_preferred_lanelets(self):
        self.assertEqual(len(self.model.nodes), 19)
        self.assertEqual(len(self.model.routes['route_b']), 18)
        for key in ('route_a', 'route_b'):
            self.assertEqual(self.model.lines(key), self.road.route_lines(
                self.road.definition[key]['expected_path_lanelet_ids']))

    def test_local_utm_matches_audited_start_pose(self):
        point = self.road.lines[200889][0]
        start = self.road.definition['start']['map_pose']
        self.assertAlmostEqual(point[0], start['x'], places=5)
        self.assertAlmostEqual(point[1], start['y'], places=5)
        self.assertAlmostEqual(point[2], start['z'], places=5)

    def test_click_inserts_between_matching_route_anchors(self):
        point = self.road.lines[200348][10]
        index = self.model.insert(point[0], point[1])
        self.assertEqual(index, 2)
        self.assertEqual(self.model.nodes[index-1]['label'], 'wp_200178')
        self.assertEqual(self.model.nodes[index+1]['label'], 'wp_200346')
        self.assertTrue(self.model.lines())
        self.assertAlmostEqual(self.model.nodes[index]['z'], point[2], places=3)

    def test_far_or_nonfinite_click_leaves_draft_unchanged(self):
        original = copy.deepcopy(self.model.routes)
        for x, y in ((1e6, 1e6), (float('nan'), 100)):
            with self.assertRaises(ValueError):
                self.model.insert(x, y)
        self.assertEqual(self.model.routes, original)

    def test_drag_and_delete_undo_restore_exact_draft(self):
        original = copy.deepcopy(self.model.nodes)
        p = self.road.lines[200348][10]
        index = self.model.insert(p[0], p[1])
        added = copy.deepcopy(self.model.nodes)
        q = self.road.lines[200348][15]
        self.model.move(index, q[0], q[1])
        self.model.undo()
        self.assertEqual(self.model.nodes, added)
        self.model.delete(index)
        self.assertEqual(self.model.nodes, original)
        self.model.undo()
        self.assertEqual(self.model.nodes, added)
        self.model.undo()
        self.assertEqual(self.model.nodes, original)
        self.assertFalse(self.model.changed['route_a'])

    def test_start_fixed_and_start_goal_cannot_be_deleted(self):
        for index in (-1, 0, len(self.model.nodes)-1):
            with self.assertRaises(ValueError):
                self.model.delete(index)
        with self.assertRaises(ValueError):
            self.model.move(0, 0, 0)

    def test_route_b_edit_does_not_change_route_a(self):
        original_a = copy.deepcopy(self.model.nodes)
        self.model.selected = 'route_b'
        self.model.delete(1)
        self.assertEqual(self.model.routes['route_a'], original_a)
        self.model.reset()
        self.assertEqual(self.model.nodes, self.model.original['route_b'])

    def test_save_load_preserves_both_drafts_and_original_config(self):
        config_before = copy.deepcopy(self.road.definition)
        self.model.delete(1)
        self.model.selected = 'route_b'
        self.model.delete(2)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'nested/routes.yaml'
            self.model.save(path)
            other = EDITOR.RouteDrafts(self.road)
            other.load(path)
            self.assertEqual(other.routes, self.model.routes)
            self.assertTrue(all(other.changed.values()))
        self.assertEqual(self.road.definition, config_before)

    def test_wrong_map_is_rejected_without_replacing_draft(self):
        original = copy.deepcopy(self.model.routes)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'route.yaml'
            self.model.save(path)
            value = yaml.safe_load(path.read_text())
            value['map_sha256'] = 'different'
            path.write_text(yaml.safe_dump(value))
            with self.assertRaises(ValueError):
                self.model.load(path)
        self.assertEqual(self.model.routes, original)

    def test_empty_file_is_rejected_without_replacing_draft(self):
        original = copy.deepcopy(self.model.routes)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'empty.yaml'
            path.write_text('')
            with self.assertRaises(ValueError):
                self.model.load(path)
        self.assertEqual(self.model.routes, original)

    def test_same_lane_preview_stops_at_goal(self):
        line = self.road.lines[200348]
        a = self.road.snap(line[5][0], line[5][1], 0.1)
        b = self.road.snap(line[20][0], line[20][1], 0.1)
        lines = self.road.leg(a, b)
        self.assertAlmostEqual(lines[0][0][0], a['x'], places=5)
        self.assertAlmostEqual(lines[-1][-1][1], b['y'], places=5)
        with self.assertRaises(ValueError):
            self.road.leg(b, a)


if __name__ == '__main__':
    unittest.main()
