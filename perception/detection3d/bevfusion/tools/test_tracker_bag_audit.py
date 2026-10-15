import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

from audit_recorded_perception_pipeline import statistics,read,PREFIX


class AuditTests(unittest.TestCase):
    def test_empty_statistics_do_not_become_zero_error(self):
        self.assertIsNone(statistics([])['p95'])
        self.assertEqual(statistics([1.,2.,3.])['p50'],2.)

    def test_recorded_detection_and_prediction_have_different_kinematics_fields(self):
        pose=N(position=N(x=1.,y=2.));twist=N(linear=N(x=3.,y=4.))
        det=N(classification=[N(label=0,probability=1.)],existence_probability=.5,
            kinematics=N(pose_with_covariance=N(pose=pose),twist_with_covariance=N(twist=twist)))
        pred=N(classification=det.classification,existence_probability=.5,object_id=N(uuid=[0]*16),
            kinematics=N(initial_pose_with_covariance=N(pose=pose),initial_twist_with_covariance=N(twist=twist)))
        header=N(stamp=N(to_sec=lambda:1.),frame_id='map')
        rows=[(PREFIX+'detection/objects',N(header=header,objects=[det]),N(to_sec=lambda:1.2)),
              (PREFIX+'objects',N(header=header,objects=[pred]),N(to_sec=lambda:1.2))]
        class Bag:
            def __init__(self,*args):pass
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def read_messages(self,topics):return iter(rows)
        with patch.dict(sys.modules,rosbag=N(Bag=Bag)):
            frames,odom=read(Path('unused.bag'))
        self.assertEqual(odom,[])
        self.assertEqual(frames['detection'][0]['objects'][0]['speed'],5.)
        self.assertEqual(frames['prediction'][0]['objects'][0]['center'],[1.,2.])
        self.assertEqual(frames['prediction'][0]['objects'][0]['id'],'00'*16)


if __name__=='__main__':unittest.main()
