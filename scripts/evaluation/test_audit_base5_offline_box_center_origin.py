#!/usr/bin/env python3
import unittest

import numpy as np

from audit_base5_offline_box_center_origin import convert_origin_trial, convert_record_origin_trial


class OriginContractTests(unittest.TestCase):
    def boxes(self):
        return np.asarray([[1.,2.,-1.9,4.,2.,1.8,.4,.7,-.8]])

    def test_declared_bottom_to_gravity_only_changes_z(self):
        original=self.boxes();saved=original.copy()
        candidate=convert_origin_trial(original,'lidar_bottom_v1','gravity_center_v1')
        self.assertAlmostEqual(candidate[0,2],-1.)
        np.testing.assert_array_equal(candidate[:,[0,1,3,4,5,6,7,8]],saved[:,[0,1,3,4,5,6,7,8]])
        np.testing.assert_array_equal(original,saved)

    def test_versioned_roundtrip(self):
        original=self.boxes()
        converted=convert_origin_trial(original,'lidar_bottom_v1','gravity_center_v1')
        restored=convert_origin_trial(converted,'gravity_center_v1','lidar_bottom_v1')
        np.testing.assert_allclose(restored,original,rtol=0,atol=1.e-14)

    def test_already_declared_gravity_is_not_shifted_again(self):
        original=self.boxes()
        np.testing.assert_array_equal(convert_origin_trial(original,'gravity_center_v1','gravity_center_v1'),original)

    def test_unknown_origin_rejected_not_guessed(self):
        with self.assertRaises(ValueError):
            convert_origin_trial(self.boxes(),'unknown','gravity_center_v1')

    def test_invalid_dimension_and_nan_rejected(self):
        for value in (-1.,0.,float('nan')):
            boxes=self.boxes();boxes[0,5]=value
            with self.assertRaises(ValueError):
                convert_origin_trial(boxes,'lidar_bottom_v1','gravity_center_v1')

    def test_empty_shape_preserved(self):
        value=convert_origin_trial(np.empty((0,7)),'lidar_bottom_v1','gravity_center_v1')
        self.assertEqual(value.shape,(0,7))

    def test_tagged_record_is_idempotent_not_double_shifted(self):
        source=dict(boxes=self.boxes(),box_center_origin='lidar_bottom_v1',score=[.9])
        once=convert_record_origin_trial(source,'gravity_center_v1')
        twice=convert_record_origin_trial(once,'gravity_center_v1')
        np.testing.assert_array_equal(once['boxes'],twice['boxes'])
        np.testing.assert_array_equal(source['boxes'],self.boxes())
        self.assertEqual(once['raw_box_center_origin'],'lidar_bottom_v1')
        self.assertEqual(twice['box_center_origin'],'gravity_center_v1')
        self.assertEqual(twice['score'],[.9])

    def test_untagged_wire_record_rejected(self):
        with self.assertRaises(ValueError):
            convert_record_origin_trial(dict(boxes=self.boxes()),'gravity_center_v1')


if __name__=='__main__':
    unittest.main()
