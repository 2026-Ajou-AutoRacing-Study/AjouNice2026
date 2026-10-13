#!/usr/bin/env python3
import copy
import unittest

import numpy as np

from prepare_base5_nearest_compile_continuous import calibration_comparison, chronological_indices


class ContinuousContractTests(unittest.TestCase):
    def calibration(self):
        camera=dict(frame_id='camera_front',width=1280,height=720,
            cam2img=np.eye(3),lidar2cam=np.eye(4),cam2lidar=np.eye(4),lidar2img=np.eye(4))
        return dict(cameras=[camera],base_from_lidar=np.eye(4),config_sha256='original')

    def test_geometry_equal_despite_byte_identity_difference(self):
        a=self.calibration();b=copy.deepcopy(a);b['config_sha256']='network_variant'
        result=calibration_comparison(a,b)
        self.assertTrue(result['numeric_geometry_equal'])
        self.assertFalse(result['file_bytes_equal'])

    def test_camera_matrix_difference_rejected(self):
        a=self.calibration();b=copy.deepcopy(a);b['cameras'][0]['lidar2cam'][0,3]=.01
        with self.assertRaises(ValueError):
            calibration_comparison(a,b)

    def test_resolution_difference_rejected(self):
        a=self.calibration();b=copy.deepcopy(a);b['cameras'][0]['width']=1056
        with self.assertRaises(ValueError):
            calibration_comparison(a,b)

    def test_indices_preserve_excluded_gaps_not_token_suffix(self):
        self.assertEqual(chronological_indices([10.,11.,12.,13.],[10.,13.]),[(0,0),(3,3)])

    def test_same_sort_dedup_contract_as_original_preparer(self):
        self.assertEqual(chronological_indices([11.,10.,11.,12.],[10.,12.]),[(0,1),(2,3)])

    def test_missing_raw_stamp_rejected(self):
        with self.assertRaises(ValueError):
            chronological_indices([10.,11.],[10.,12.])


if __name__=='__main__':
    unittest.main()
