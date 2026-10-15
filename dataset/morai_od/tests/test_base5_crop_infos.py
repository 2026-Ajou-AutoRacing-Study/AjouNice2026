import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from build_base5_crop_infos import update_component


class CropInfosTest(unittest.TestCase):
    def component(self,points=0,ambiguous=False):
        return dict(lidar_points=points,camera_component_ambiguous=ambiguous,
                    center_lidar_m=[1,2,3],size_m=[2,1,1],rotation_rpy=[0,0,.3],
                    camera_visible_candidate=False,positive_lidar1_camera0=points>=1,ignore_reason=None)
    def test_zero_point_camera_restored_without_changing_geometry(self):
        c=self.component();n=update_component(c,dict(candidates=dict(bottom448=dict(camera_gate=True))),'bottom448')
        self.assertTrue(n['positive_lidar1_camera0']);self.assertFalse(c['camera_visible_candidate'])
        self.assertEqual(c['center_lidar_m'],n['center_lidar_m'])
    def test_ambiguous_shared_parent_does_not_create_camera_gt(self):
        n=update_component(self.component(ambiguous=True),dict(candidates=dict(bottom448=dict(camera_gate=True))),'bottom448')
        self.assertFalse(n['positive_lidar1_camera0'])
    def test_lidar_support_survives_no_camera(self):
        n=update_component(self.component(points=1),None,'bottom448')
        self.assertTrue(n['positive_lidar1_camera0'])


if __name__=='__main__':unittest.main()
