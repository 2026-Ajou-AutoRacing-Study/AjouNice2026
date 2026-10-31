"""Lightweight recovery receipt/prefix checks; full4614 GPU gate is separate."""
from pathlib import Path
import unittest

from scripts.runtime import recover_base5_full_validation as recovery


class RecoveryTests(unittest.TestCase):
    def test_existing_prefix_preserved(self):
        self.assertEqual(recovery.metric_key('val', 'center/mAP'), 'val/center/mAP')
        self.assertEqual(recovery.metric_key(None, 'center/mAP'), 'center/mAP')

    def test_exact_e1_is_pinned(self):
        self.assertEqual(recovery.E1_SHA256, '471e1e9c5a2c2c3a92330dc858570109644286bc3ea15fd74f54382b0274ba36')
        self.assertEqual((recovery.E1_EPOCH, recovery.E1_ITER, recovery.FRAMES), (1, 6534, 4614))

    def test_full_loop_without_training_or_subset(self):
        source = Path(recovery.__file__).read_text()
        self.assertIn('runner.val()', source)
        self.assertNotIn('runner.train()', source)
        self.assertNotIn('Subset(', source)
        self.assertIn('complete=True, valid=True', source)
        self.assertIn("cfg.val_cfg['fp16'] = False", source)


if __name__ == '__main__': unittest.main()
