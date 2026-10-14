import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).parent))
import analyze_live_perception_candidates as audit


class LiveCandidateTests(unittest.TestCase):
    def test_assignment_preserves_class_and_maximum_cardinality(self):
        a=[[0,0],[.9,0]]; b=[[.1,0],[-.8,0]]
        self.assertEqual(len(audit.matches(a,b,[1,1],[1,1],1.)),2)
        self.assertEqual(audit.matches(a,b,[0,0],[1,1],1.),[])
        self.assertEqual(audit.matches([],b,[],[1,1]),[])

    def test_world_origin_is_shifted_once_before_sensor_translation(self):
        row=dict(lidar_translation=[1,0,2],ego=dict(x=10,y=20,z=30,yaw=0))
        box=[3,4,-2,5,6,2,.1];saved=copy.deepcopy(box)
        self.assertEqual(audit.world(box,row,False),[14,24,30,5,6,2,.1])
        self.assertEqual(audit.world(box,row,True),[14,24,31,5,6,2,.1])
        self.assertEqual(box,saved)

    def test_motion_diagnostic_uses_centers_and_refuses_large_gap(self):
        stamps=[1.,1.1,1.2]
        truth=[dict(key='p',center=[t-1,0,1],class_name='pedestrian') for t in stamps]
        output=[dict(stamp=t,tracks=[dict(confirmed=True,core_class=3,center=[t-1,0,1],velocity=[1,0])]) for t in stamps]
        result=audit.motion_summary(output,dict(gt=[(stamps,truth)]))
        self.assertAlmostEqual(result['distributions']['pedestrian/moving/velocity_vector_error_mps']['max'],0)
        result=audit.motion_summary(output,dict(gt=[([1.,1.1,1.5],truth)]))
        self.assertGreater(result['counts']['unbracketed_velocity'],0)


if __name__=='__main__': unittest.main()
