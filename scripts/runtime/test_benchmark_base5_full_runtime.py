"""CPU checks for the bounded full-cohort profiler; no model or network."""
import copy
from pathlib import Path
import tempfile
import unittest

import numpy as np

from scripts.runtime import benchmark_base5_full_runtime as probe


def sample(token='a', x=2., score=.8, label=0, yaw=0.):
    return dict(token=token, gt_boxes=np.asarray([[2., 0., 0., 1., 1., 1., 0.]]),
        gt_labels=np.asarray([0]), pred_boxes=np.asarray([[x, 0., 0., 1., 1., 1., yaw]]),
        pred_labels=np.asarray([label]), pred_scores=np.asarray([score]))


class Tiny:
    def __init__(self, size): self.size = size
    def __len__(self): return self.size


class Concat:
    def __init__(self, sizes): self.datasets = [Tiny(size) for size in sizes]


class SubsetTests(unittest.TestCase):
    def test_each_cohort_and_no_duplicate(self):
        selected = probe.cohort_indices(Concat([750]*6+[38]*3), 64)
        self.assertEqual(len(selected), 64)
        self.assertEqual(len(set(selected)), 64)
        counts = np.histogram(selected, bins=[0, 750, 1500, 2250, 3000,
                                             3750, 4500, 4538, 4576, 4614])[0]
        self.assertTrue(np.all(counts >= 7))
        self.assertTrue(np.all(counts <= 8))

    def test_small_child_not_oversampled(self):
        self.assertEqual(probe.cohort_indices(Concat([1, 4]), 5), [0, 1, 2, 3, 4])

    def test_single_plain_dataset(self):
        self.assertEqual(probe.cohort_indices(Tiny(5), 3), [0, 2, 4])

    def test_too_large_rejected(self):
        with self.assertRaises(ValueError): probe.cohort_indices(Concat([2]), 3)


class ComparisonTests(unittest.TestCase):
    def test_identical_and_no_mutation(self):
        a = [sample()]; before = copy.deepcopy(a)
        report = probe.compare_predictions(a, a)
        self.assertEqual(report['exact_frames'], 1)
        self.assertEqual(report['matched'], 1)
        self.assertEqual(report['unmatched_reference'], 0)
        for key in before[0]:
            np.testing.assert_equal(a[0][key], before[0][key])

    def test_quantifies_deltas_and_wrapped_yaw(self):
        report = probe.compare_predictions([sample(yaw=np.pi-.1)],
                                           [sample(x=2.2, score=.7, yaw=-np.pi+.1)])
        self.assertAlmostEqual(report['deltas']['xy_m']['mean'], .2)
        self.assertAlmostEqual(report['deltas']['score']['mean'], .1)
        self.assertAlmostEqual(report['deltas']['yaw_rad']['mean'], .2)

    def test_wrong_class_remains_unmatched(self):
        report = probe.compare_predictions([sample()], [sample(label=1)])
        self.assertEqual(report['matched'], 0)
        self.assertEqual(report['unmatched_reference'], 1)
        self.assertEqual(report['unmatched_candidate'], 1)

    def test_strict_radius(self):
        report = probe.compare_predictions([sample()], [sample(x=3.)])
        self.assertEqual(report['matched'], 0)

    def test_token_or_gt_change_rejected(self):
        with self.assertRaisesRegex(ValueError, 'tokens'):
            probe.compare_predictions([sample()], [sample(token='b')])
        changed = sample(); changed['gt_boxes'][0, 0] += 1
        with self.assertRaisesRegex(ValueError, 'GT'):
            probe.compare_predictions([sample()], [changed])

    def test_empty_predictions_supported(self):
        row = sample()
        for key in ('pred_boxes', 'pred_labels', 'pred_scores'): row[key] = row[key][:0]
        report = probe.compare_predictions([row], [row])
        self.assertEqual(report['matched'], 0)
        self.assertEqual(report['exact_frames'], 1)
        self.assertEqual(report['deltas']['xy_m'], {})


class InterfaceTests(unittest.TestCase):
    def test_real_proc_fd_count(self):
        count = probe.open_fd_count()
        self.assertTrue(count is None or count >= 3)

    def test_defaults_preserve_fp32_and_compile_is_not_an_option(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); checkpoint = root/'epoch_1.pth'; checkpoint.touch()
            paths = dict(source=root, adapter=root/'adapter.py', data_index=root/'index.json',
                         initializer=root/'init.pth', checkpoint=checkpoint, output=root/'fresh')
            argv = ['--mode', 'val']
            for key, value in paths.items(): argv.extend(['--'+key.replace('_', '-'), str(value)])
            args = probe.parse_args(argv)
            self.assertFalse(args.val_amp)
            self.assertEqual(args.val_batches, [1, 2, 4])
            self.assertEqual(args.steps, 32)

    def test_sorted_resolved_config_not_used_for_reconstruction(self):
        source = Path(probe.__file__).read_text()
        self.assertIn('adapter.adapted_prepare(core_path.read_text(), vars(core))(setup)', source)
        self.assertNotIn('Config.fromfile', source)
        self.assertIn('cfg.default_hooks.checkpoint = None', source)
        self.assertIn('original_run_modified=False', source)


if __name__ == '__main__': unittest.main()
