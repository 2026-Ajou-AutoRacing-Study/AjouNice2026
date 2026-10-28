"""Keep the identity-only candidate explicit across both stack entrypoints."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
NAME = 'stationary_unknown_uuid_continuity'

class StationaryUuidLaunchContract(unittest.TestCase):
    def parse(self, path):
        tree = ET.parse(path).getroot()
        self.assertEqual(tree.find("arg[@name='"+NAME+"']").get('default'), 'false')
        return tree

    def test_pipeline_keeps_candidate_off_and_forwards_to_policy(self):
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
