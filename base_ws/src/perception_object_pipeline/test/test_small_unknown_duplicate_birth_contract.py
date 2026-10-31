"""The small-object birth candidate stays explicit through ROS entrypoints."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
NAME = 'suppress_small_unknown_duplicate_birth'


class SmallUnknownDuplicateBirthContract(unittest.TestCase):
    def parse(self, path):
        tree = ET.parse(path).getroot()
        self.assertEqual(tree.find("arg[@name='" + NAME + "']").get('default'), 'false')
        return tree

    def assert_forwarded(self, root, destination):
        include = next(item for item in root.iter('include')
                       if destination in item.get('file', ''))
        self.assertEqual(include.find("arg[@name='" + NAME + "']").get('value'),
                         '$(arg ' + NAME + ')')

    def test_pipeline_passes_opt_in_to_native_tracker(self):
        root = self.parse(PACKAGE/'launch/object_pipeline.launch')
        self.assert_forwarded(root, 'ekf_multi_class_object_tracking.launch')

    def test_tracking_wrapper_and_integrated_stack_pass_candidate(self):
        paths = [PACKAGE/'launch/tracking_prediction.launch',
                 ROOT/'base_ws/src/planning_addons/ajounice_planning_integration/launch/morai_planning_stack.launch']
        for path in paths:
            with self.subTest(path=path):
                self.assert_forwarded(self.parse(path), 'object_pipeline.launch')

    def test_native_launch_sets_private_configuration(self):
        native = ROOT/'base_ws/src/ekf-multi-object-tracking'
        if not (native/'launch/ekf_multi_class_object_tracking.launch').exists():
            self.skipTest('Native tracker submodule is not initialized')
        root = self.parse(native/'launch/ekf_multi_class_object_tracking.launch')
        parameter = root.find("param[@name='configure/" + NAME + "']")
        self.assertEqual(parameter.get('value'), '$(arg ' + NAME + ')')


if __name__ == '__main__':
    unittest.main()
