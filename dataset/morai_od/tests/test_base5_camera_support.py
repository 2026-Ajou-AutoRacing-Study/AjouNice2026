import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from audit_base5_camera_support import projected_support


class CameraSupportTest(unittest.TestCase):
    def test_front_and_behind_are_distinct(self):
        intrinsic = [[100, 0, 320], [0, 100, 240], [0, 0, 1]]
        front = projected_support([0, 0, 10, 2, 2, 2, 0], np.eye(4), intrinsic, 640, 480)
        self.assertTrue(front['supported'])
        self.assertGreater(front['native_area_px'], 0)
        rear = projected_support([0, 0, -10, 2, 2, 2, 0], np.eye(4), intrinsic, 640, 480)
        self.assertFalse(rear['supported'])

    def test_outside_image_not_supported(self):
        result = projected_support([100, 0, 10, 2, 2, 2, 0], np.eye(4),
                                   [[100, 0, 320], [0, 100, 240], [0, 0, 1]], 640, 480)
        self.assertFalse(result['supported'])


if __name__ == '__main__':
    unittest.main()
