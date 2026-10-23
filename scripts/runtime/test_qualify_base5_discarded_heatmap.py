"""CPU fixture-copy/gradient comparison tests, not a GPU accuracy certificate."""
import unittest

import torch

from scripts.runtime import qualify_base5_discarded_heatmap as gate


class FixtureTests(unittest.TestCase):
    def test_preserve_flags_and_allow_original_inplace_sigmoid(self):
        source = [dict(dense=torch.tensor([-.3, .7], requires_grad=True)*2,
                       detached=torch.tensor([.1, .2]), label=torch.tensor([1, 0]))]
        frozen, flags = gate.freeze_tree(source)
        copied, leaves = gate.thaw_tree(frozen, flags)
        self.assertEqual(set(leaves), {'/0/dense'})
        copied[0]['dense'].sigmoid_().sum().backward()
        self.assertIsNotNone(leaves['/0/dense'].grad)
        torch.testing.assert_close(frozen[0]['dense'], source[0]['dense'])
        self.assertFalse(frozen[0]['dense'].requires_grad)
        self.assertIsNone(source[0]['dense'].grad)

    def test_two_cases_have_independent_leaf_storage(self):
        template, flags = gate.freeze_tree(dict(value=torch.tensor([2.], requires_grad=True)))
        left, left_leaves = gate.thaw_tree(template, flags)
        right, right_leaves = gate.thaw_tree(template, flags)
        left['value'].sum().backward(); right['value'].sum().backward()
        self.assertNotEqual(left_leaves['/value'].data_ptr(), right_leaves['/value'].data_ptr())
        self.assertTrue(gate.gradient_comparison(
            {'value': left_leaves['/value'].grad}, {'value': right_leaves['/value'].grad})['exact'])

    def test_gradient_difference_or_missing_not_hidden(self):
        comparison = gate.gradient_comparison({'a': torch.tensor([1.]), 'b': None},
                                              {'a': torch.tensor([1.1]), 'b': None})
        self.assertFalse(comparison['exact'])
        self.assertAlmostEqual(comparison['max_abs'], .1, places=5)
        self.assertFalse(gate.gradient_comparison({'a': torch.tensor([1.])}, {'a': None})['exact'])

    def test_all_target_tensors_and_scalars_compared(self):
        original = [torch.tensor([1., 2.]), 3, .4]
        copied = gate.cpu_targets(original)
        self.assertTrue(gate.targets_equal(original, copied))
        copied[0][0] = 9
        self.assertFalse(gate.targets_equal(original, copied))
        self.assertEqual(float(original[0][0]), 1.)


if __name__ == '__main__': unittest.main()
