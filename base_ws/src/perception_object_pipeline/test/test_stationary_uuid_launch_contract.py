"""Keep qualified identity recovery ON with an explicit legacy-ID rollback."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
NAME = 'stationary_unknown_uuid_continuity'

class StationaryUuidLaunchContract(unittest.TestCase):
    def parse(self, path):
        tree = ET.parse(path).getroot()
        self.assertEqual(tree.find("arg[@name='"+NAME+"']").get('default'), 'true')
        return tree

    def test_pipeline_defaults_on_and_forwards_override_to_policy(self):
        root = self.parse(PACKAGE/'launch/object_pipeline.launch')
        node = root.find("node[@name='tracked_object_motion_policy']")
        self.assertEqual(node.find("param[@name='"+NAME+"']").get('value'), '$(arg '+NAME+')')

    def test_tracking_wrapper_forwards_candidate_to_pipeline(self):
        root = self.parse(PACKAGE/'launch/tracking_prediction.launch')
        include = next(x for x in root.findall('include') if 'object_pipeline.launch' in x.get('file',''))
        self.assertEqual(include.find("arg[@name='"+NAME+"']").get('value'), '$(arg '+NAME+')')

    def test_integrated_stack_forwards_candidate_to_pipeline(self):
        root = self.parse(ROOT/'base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch')
        include = next(x for x in root.iter('include') if 'object_pipeline.launch' in x.get('file',''))
        self.assertEqual(include.find("arg[@name='"+NAME+"']").get('value'), '$(arg '+NAME+')')

    def test_operating_alias_inherits_default_and_allows_false_override(self):
        alias = next(line for line in (ROOT/'docs/common_alias.txt').read_text().splitlines()
                     if line.startswith('alias tracking='))
        self.assertIn('object_pipeline.launch', alias)
        self.assertNotIn(NAME + ':=', alias)
        # Entry points forward $(arg NAME), not hardcoded true: explicit false
        # reaches the policy. Direct node fallback remains off (node tests).
        for relative in ('launch/object_pipeline.launch', 'launch/tracking_prediction.launch'):
            tree = self.parse(PACKAGE/relative)
            self.assertTrue(any(item.get('value') == '$(arg '+NAME+')'
                                for item in tree.iter() if item.get('name') == NAME))

    def test_unqualified_birth_guard_and_legacy_radius_are_unchanged(self):
        paths = [(PACKAGE/'launch/object_pipeline.launch', 'duplicate_birth_suppression_distance_m'),
                 (PACKAGE/'launch/tracking_prediction.launch', 'tracker_duplicate_birth_suppression_distance_m')]
        for path, distance in paths:
            with self.subTest(path=path):
                tree = self.parse(path)
                self.assertEqual(tree.find("arg[@name='suppress_small_unknown_duplicate_birth']").get('default'), 'false')
                self.assertEqual(tree.find("arg[@name='"+distance+"']").get('default'), '0.75')
