import copy
import unittest

from propose_traffic_light_housing_annotations import propose


class ProposalTest(unittest.TestCase):
    def setUp(self):
        self.ego={'position_xyz_m':[0.,0.,0.], 'heading_degrees':1.347}
        self.head={'idx':'test','type':'car','point':[20.,0.,3.],
                   'heading':180.,'width':1.42,'height':.355}

    def test_front_head_is_unapproved_proposal(self):
        original=copy.deepcopy(self.head)
        result=propose([self.head], self.ego)
        self.assertEqual(len(result),1)
        self.assertFalse(result[0]['box_qc_accepted'])
        self.assertTrue(result[0]['proposal_only'])
        self.assertEqual(self.head,original)

    def test_pedestrian_back_and_zero_dimensions_not_fabricated(self):
        for changes in ({'type':'pedestrian'},{'heading':0.},{'width':0.},{'height':0.}):
            self.assertEqual(propose([{**self.head,**changes}], self.ego),[])

    def test_behind_camera_and_extreme_size_not_accepted(self):
        self.assertEqual(propose([{**self.head,'point':[-20,0,3]}],self.ego),[])
        self.assertEqual(propose([self.head],self.ego,minimum_width=10000),[])

    def test_side_face_excluded(self):
        self.assertEqual(propose([{**self.head,'heading':95.}],self.ego),[])


if __name__=='__main__':
    unittest.main()
