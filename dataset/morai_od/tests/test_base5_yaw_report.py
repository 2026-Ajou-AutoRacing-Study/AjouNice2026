import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from report_base5_yaw_probe import augmentation_retention


class AugmentationReportTest(unittest.TestCase):
    def test_near_center_remains_inside(self):
        r=augmentation_retention([[10,0,0]],trials=100)
        self.assertEqual(r['profiles']['current_RTS']['conditional_inside_retention'],1.)

    def test_already_outside_not_counted_as_new_drop(self):
        r=augmentation_retention([[10,0,0],[70,34,0]],trials=100)
        self.assertEqual(r['initial_inside'],1)
        self.assertEqual(r['profiles']['current_RTS']['conditional_inside_retention'],1.)

    def test_stress_boundary_detects_augmentation_drop(self):
        r=augmentation_retention([[99,0,0]],trials=1000)
        self.assertLess(r['profiles']['current_RTS']['conditional_inside_retention'],.8)
        self.assertGreater(r['profiles']['translation_only']['conditional_inside_retention'],.99)


if __name__=='__main__':unittest.main()
