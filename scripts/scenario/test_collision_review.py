import unittest
from collision_review import PendingCollisionReview


def npc(x=73., y=265.):
    return dict(xy=(x,y), speed=0., valid=True)


class Tests(unittest.TestCase):
    def test_gt_arrives_2_94ms_later_distant(self):
        p=PendingCollisionReview();p.report(21,(73.,265.),10.,{})
        self.assertEqual(p.resolve(10.,{},0,lambda e,o:9.),[])
        result=p.resolve(10.00294,{21:npc()},10.00294,lambda e,o:9.06)
        self.assertFalse(result[0]['review']);self.assertEqual(p.pending,{})

    def test_true_near_immediate(self):
        p=PendingCollisionReview();p.report(21,(73.,265.),10.,{})
        result=p.resolve(10.,{21:npc()},10.,lambda e,o:.1)
        self.assertTrue(result[0]['review']);self.assertEqual(result[0]['association_wait_sec'],0)

    def test_reused_id_other_position_cannot_clear(self):
        p=PendingCollisionReview();p.report(21,(73.,265.),10.,{})
        self.assertEqual(p.resolve(10.01,{21:npc(73.,-47)},10.,lambda e,o:9.),[])
        self.assertTrue(p.resolve(10.101,{},10.,lambda e,o:9.)[0]['review'])

    def test_timeout_even_if_late_gt_clear(self):
        p=PendingCollisionReview();p.report(21,(73.,265.),10.,{})
        result=p.resolve(10.101,{21:npc()},10.101,lambda e,o:9.)
        self.assertEqual(result[0]['reason'],'collision_gt_association_timeout')

    def test_repeated_packet_keeps_first_deadline(self):
        p=PendingCollisionReview();p.report(21,(73.,265.),10.,{})
        p.report(21,(73.,265.),10.09,{})
        self.assertEqual(p.pending[21]['deadline'],10.1)
        self.assertTrue(p.resolve(10.101,{},0,lambda e,o:9.)[0]['review'])

    def test_invalid_box_or_stale_gt_does_not_clear(self):
        for obj,gt_time in [(dict(npc(),valid=False),10.),(npc(),9.)]:
            p=PendingCollisionReview();p.report(21,(73.,265.),10.,{})
            self.assertEqual(p.resolve(10.,{21:obj},gt_time,lambda e,o:9.),[])


if __name__ == '__main__':
    unittest.main()
