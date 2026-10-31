"""CPU AST and guarded-MRO tests; real E1 target/loss/gradient gate is separate."""
import ast
import copy
from pathlib import Path
import unittest

from scripts.runtime import base5_skip_discarded_heatmap as candidate

ROOT = Path(__file__).resolve().parents[2]
PARENT = ROOT/'third_party/mmdetection3d/projects/BEVFusion/bevfusion/transfusion_head.py'


def parent_function():
    tree = ast.parse(PARENT.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == 'TransFusionHead')
    return next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                and node.name == 'get_targets_single')


class StructureTests(unittest.TestCase):
    def test_full_source_pins_match(self):
        self.assertEqual(candidate._sha(PARENT), candidate.PARENT_SHA256)
        self.assertEqual(candidate._sha(ROOT/'perception/detection3d/bevfusion/plugin/morai_rectangular_bev.py'),
                         candidate.RECTANGULAR_SHA256)

    def test_only_discarded_block_changes(self):
        original = parent_function(); before = ast.dump(original)
        patched = candidate.strip_discarded_block(original)
        self.assertEqual(ast.dump(original), before)
        start = next(i for i, node in enumerate(original.body) if isinstance(node, ast.Assign)
                     and isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'device')
        self.assertEqual([ast.dump(n) for n in original.body[:start]],
                         [ast.dump(n) for n in patched.body[:start]])
        self.assertEqual([ast.dump(n) for n in original.body[start+8:]],
                         [ast.dump(n) for n in patched.body[start+1:]])
        self.assertEqual(len(original.body)-len(patched.body), 7)
        self.assertIn('new_empty', ast.unparse(patched.body[start]))
        self.assertNotIn('gaussian_radius', ast.unparse(patched))
        self.assertNotIn('draw_heatmap_gaussian', ast.unparse(patched))

    def test_changed_source_rejected(self):
        with self.assertRaisesRegex(ValueError, 'SHA'):
            candidate.specialize_parent(PARENT.read_text()+'\n', {})

    def test_changed_structure_rejected(self):
        function = parent_function()
        for node in function.body:
            if (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                    and node.targets[0].id == 'voxel_size'):
                node.targets[0].id = 'unexpected_size'
        with self.assertRaisesRegex(ValueError, 'statements'):
            candidate.strip_discarded_block(function)

    def test_eighth_output_contract_required(self):
        function = parent_function(); function.body[-1].value.elts.pop()
        with self.assertRaisesRegex(ValueError, 'tuple'):
            candidate.strip_discarded_block(function)

    def test_compile_original_prefix_without_importing_framework(self):
        # Names resolve only on invocation; compilation itself needs no CUDA/MMCV.
        function = candidate.specialize_parent(PARENT.read_text(), {})
        self.assertTrue(callable(function))


class GuardTests(unittest.TestCase):
    def test_nonrectangular_head_remains_original(self):
        class Legacy: pass
        class Rectangular: pass
        calls = []
        def old(self, value): calls.append('old'); return value+1
        def new(self, value): calls.append('new'); return value+2
        guarded = candidate.guarded_parent(old, new, Rectangular)
        self.assertEqual(guarded(Legacy(), 3), 4)
        self.assertEqual(guarded(Rectangular(), 3), 5)
        self.assertEqual(calls, ['old', 'new'])

    def test_rectangular_replacement_and_seven_used_outputs_preserved(self):
        calls = []
        class Parent:
            def get_targets_single(self):
                calls.append('legacy_heatmap')
                return (0, 1, 2, 3, 4, 5, 6, 'discarded')
        class Rectangular(Parent):
            def get_targets_single(self):
                targets = super().get_targets_single()
                calls.append('correct_heatmap')
                return (*targets[:-1], 'correct_XY')
        class Legacy(Parent): pass
        baseline = Rectangular().get_targets_single()
        original = Parent.get_targets_single
        def skip(self): return (0, 1, 2, 3, 4, 5, 6, 'device_only')
        Parent.get_targets_single = candidate.guarded_parent(original, skip, Rectangular)
        calls.clear()
        self.assertEqual(Rectangular().get_targets_single(), baseline)
        self.assertEqual(calls, ['correct_heatmap'])
        self.assertEqual(Legacy().get_targets_single()[-1], 'discarded')

    def test_uninstall_before_install_is_safe(self):
        self.assertIsNone(candidate.uninstall())


if __name__ == '__main__': unittest.main()
