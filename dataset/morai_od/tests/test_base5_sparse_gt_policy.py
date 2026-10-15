import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_sparse_gt_policy import reviewed_camera_support


class SparsePolicyTest(unittest.TestCase):
    def test_review_distance_and_model_crop_required(self):
        camera = dict(lidar2cam=np.eye(4), cam2img=[[100, 0, 352], [0, 100, 128], [0, 0, 1]],
                      width=704, height=256)
        box = [0., 0., 5., 1., 1., 1., 0.]
        self.assertFalse(reviewed_camera_support('a', 1, box, {'front': camera}, set())[0])
        self.assertTrue(reviewed_camera_support('a', 1, box, {'front': camera}, {('a', 1)})[0])
        box[0] = 11.
        self.assertEqual(reviewed_camera_support('a', 1, box, {'front': camera}, {('a', 1)})[1], 'outside_nearfield')
        box[0], box[1] = 0., -20.
        self.assertFalse(reviewed_camera_support('a', 1, box, {'front': camera}, {('a', 1)}, maximum_distance_m=30.)[0])
