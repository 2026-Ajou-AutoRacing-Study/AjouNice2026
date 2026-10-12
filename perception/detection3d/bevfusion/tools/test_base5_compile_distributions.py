import unittest

from summarize_base5_compile_distributions import detection_summary, tracker_summary, TRACK_CLASSES


class CompileDistributionTests(unittest.TestCase):
    def test_class_mapping_matches_cpp_enum(self):
        self.assertEqual(TRACK_CLASSES[0], 'obstacle')
        self.assertEqual(TRACK_CLASSES[1], 'vehicle')
        self.assertEqual(TRACK_CLASSES[3], 'pedestrian')

    def test_observations_and_track_balanced_means_are_distinct(self):
        def track(identity,x):return dict(id=identity,label=3,confirmed=True,associated=True,
            center=[x,0.],velocity=[0.,0.],yaw=0.)
        a={'bag':[dict(stamp=1.,tracks=[track(1,0),track(2,2)]),
                  dict(stamp=2.,tracks=[track(1,0)])]}
        b={'bag':[dict(stamp=1.,tracks=[track(1,.1),track(2,2.3)]),
                  dict(stamp=2.,tracks=[track(1,.1)])]}
        r,rows=tracker_summary(a,b,.5)
        d=r['by_class']['pedestrian']
        self.assertEqual(d['reference_observations'],3)
        self.assertEqual(d['reference_track_ids'],2)
        self.assertAlmostEqual(d['distributions']['position_m']['mean'],1/6)
        self.assertAlmostEqual(d['reference_track_balanced']['distribution_of_track_mean_position_m']['mean'],.2)
        self.assertEqual(r['by_class']['vehicle']['distributions']['position_m']['count'],0)
        self.assertEqual(len(rows),3)

    def test_unmatched_and_range_boundaries_are_separate_from_quantiles(self):
        def d(x):return dict(center=[x,0,0],size=[1,1,1],yaw=0,label=2,class_name='obstacle',score=.8)
        r,rows=detection_summary({'a':[d(2),d(40)]},{'a':[d(3),d(40)]},30.)
        block=r['by_class']['obstacle']
        self.assertEqual(block['removed'],1)
        self.assertEqual(block['added'],1)
        self.assertEqual(len(rows),0)
        self.assertIsNone(block['distributions']['center_xy_m']['mean'])

    def test_reused_slot_id_is_split_into_lifecycle_segments(self):
        def f(t,x):return dict(stamp=t,tracks=[] if x is None else [dict(id=1,label=0,
            confirmed=True,associated=True,center=[x,0.],velocity=[0.,0.],yaw=0.)])
        a={'bag':[f(1,0.),f(2,None),f(3,0.)]}
        b={'bag':[f(1,.1),f(2,None),f(3,.3)]}
        r,rows=tracker_summary(a,b,.5)
        self.assertEqual([v['reference_segment'] for v in rows],[1,2])
        self.assertEqual(r['by_class']['obstacle']['reference_track_balanced']['matched_track_segments'],2)


if __name__=='__main__':unittest.main()
