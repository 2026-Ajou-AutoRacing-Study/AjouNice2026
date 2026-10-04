"""Run launch-adapter imports, without starting ROS nodes or publishing data.

Source scripts/workspace/source_aw149.sh before running these deployment tests.
Pure geometry tests and py_compile cannot detect a message package mix-up.
"""
import ast
from pathlib import Path
import unittest


class TrialRuntimeImports(unittest.TestCase):
    def test_overtaking_main_ros_imports_resolve_in_installed_overlay(self):
        source = Path(__file__).with_name('overtaking_trial_policy.py')
        tree = ast.parse(source.read_text(), filename=str(source))
        main = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        imports = [node for node in main.body
                   if isinstance(node, (ast.Import, ast.ImportFrom))]
        self.assertTrue(imports)
        # Only imports run; main's node initialization/requests are never called.
        namespace = {}
        exec(compile(ast.Module(body=imports, type_ignores=[]), str(source), 'exec'),
             namespace)
        self.assertEqual(namespace['PathWithLaneId']._type,
                         'autoware_internal_planning_msgs/PathWithLaneId')


if __name__ == '__main__':
    unittest.main()
