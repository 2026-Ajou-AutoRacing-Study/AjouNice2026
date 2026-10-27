"""Explicit v94 profile preserves the released v88 routing policy and road graph.

The team-Drive OSM is optional at checkout. Config/policy contracts always run;
asset tests skip only when that OSM has not been provisioned. No ROS is started.
"""
import copy
import hashlib
from itertools import zip_longest
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from competition_route_requester import policy
from competition_route_requester.geometry import Geometry


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[3]
CONFIG = PACKAGE / 'config'
MANIFEST = REPO / 'configs/map/kcity_v94_merged_asset.yaml'
V94_SHA = '3d11cd820b2f4d755960ae565dbc267e4cf5826307ef2df1149e10922c138cbf'
STOP_NODES = {92992, 92993, 92998, 92999, 93020, 93021, 93034, 93035}


def structure(element):
    """Compare OSM content independently of XML indentation."""
    return (element.tag, tuple(sorted(element.attrib.items())),
            tuple(structure(child) for child in element))


def without_coordinates(node):
    return (tuple(sorted((key, value) for key, value in node.attrib.items()
                         if key not in ('lat', 'lon'))),
            tuple(structure(child) for child in node
                  if not (child.tag == 'tag' and child.get('k') in ('local_x', 'local_y'))))


def map_elements(path):
    """Stream top-level elements so two 32 MB maps do not stay in memory."""
    depth = 0
    root = None
    for event, element in ET.iterparse(path, events=('start', 'end')):
        if event == 'start':
            depth += 1
            if root is None:
                root = element
                yield ('root', tuple(sorted(root.attrib.items())))
        else:
            if depth == 2:
                yield element
                root.clear()
            depth -= 1


class V94ProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_cfg = yaml.safe_load((CONFIG / 'competition_route_requester.param.yaml').read_text())
        cls.cfg = yaml.safe_load((CONFIG / 'competition_route_requester_v94.param.yaml').read_text())
        cls.routes = yaml.safe_load((CONFIG / 'kcity_v94_competition_route.yaml').read_text())
        cls.manifest = yaml.safe_load(MANIFEST.read_text())

    def require_asset(self):
        path = REPO / self.routes['map']['file']
        if not path.is_file():
            self.skipTest('Team-Drive v94 OSM has not been provisioned')
        return path

    def test_v94_map_contract_matches_the_merged_asset_manifest(self):
        self.assertEqual(self.routes['map'], {
            'version': self.manifest['map_version'],
            'file': self.manifest['map_file'],
            'projector_file': self.manifest['projector_file'],
            'sha256': self.manifest['map_sha256'],
        })
        self.assertEqual(self.routes['map']['version'], 94)
        self.assertEqual(self.routes['map']['sha256'], V94_SHA)
        self.assertEqual(set(self.manifest['stopline_patch']['changed_node_ids']), STOP_NODES)

    def test_public_and_explicit_requester_select_the_same_v94_contract(self):
        expected = dict(self.base_cfg,
            route_definition_file='kcity_v94_competition_route.yaml')
        self.assertEqual(self.cfg, expected)
        self.assertEqual(self.base_cfg['route_definition_file'], 'kcity_v94_competition_route.yaml')

    def test_all_five_operation_cases_keep_the_released_policy(self):
        for case in policy.CASES:
            policy.validate(dict(self.cfg, route_operation_case=case), self.routes)
            for key in ('route_a', 'route_b', 'fallback_route'):
                definition = self.routes[key]
                goal = self.routes['poses'][definition['goal_ref']]
                self.assertTrue(policy.route_matches(self.routes, key,
                    definition['uuid'], definition['expected_path_lanelet_ids'],
                    (goal['x'], goal['y']), self.cfg['goal_tolerance_m']))

    def test_provisioned_asset_sha_and_only_eight_stop_nodes_and_version_change(self):
        path = self.require_asset()
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), V94_SHA)
        base = REPO / 'kcity_map/lanelet_map/lanelet2_map_v88.osm'
        self.assertEqual(hashlib.sha256(base.read_bytes()).hexdigest(),
                         self.manifest['teammate_base']['sha256'])
        changed_nodes = set()
        metadata_count = 0
        for before, after in zip_longest(map_elements(base), map_elements(path)):
            self.assertIsNotNone(before, 'v94 added an OSM element')
            self.assertIsNotNone(after, 'v94 removed an OSM element')
            if isinstance(before, tuple) or isinstance(after, tuple):
                self.assertEqual(before, after)
                continue
            self.assertEqual(before.tag, after.tag)
            self.assertEqual(before.get('id'), after.get('id'))
            if before.tag == 'MetaInfo':
                metadata_count += 1
                self.assertEqual(before.get('map_version'), '88')
                self.assertEqual(after.get('map_version'), '94')
                expected = copy.deepcopy(before)
                expected.set('map_version', '94')
                self.assertEqual(structure(after), structure(expected))
            elif structure(before) != structure(after):
                self.assertEqual(before.tag, 'node', 'way/relation or road topology changed')
                changed_nodes.add(int(before.get('id')))
                self.assertEqual(without_coordinates(before), without_coordinates(after))
        self.assertEqual(metadata_count, 1)
        self.assertEqual(changed_nodes, STOP_NODES)

    def test_provisioned_profile_geometry_contains_all_route_lanelets(self):
        self.require_asset()
        geometry = Geometry(REPO, self.routes)
        required = set(self.routes['start']['accepted_start_lanelet_ids'])
        for key in ('route_a', 'route_b', 'fallback_route'):
            definition = self.routes[key]
            required.update(definition['expected_path_lanelet_ids'])
            required.update(definition.get('waypoint_path_lanelet_ids', []))
        for pose in self.routes['poses'].values():
            required.add(pose['lanelet_id'])
            if 'progress_lanelet_id' in pose:
                required.add(pose['progress_lanelet_id'])
        self.assertFalse(required - set(geometry.lanes), 'route lanelet missing from v94')


if __name__ == '__main__':
    unittest.main()
