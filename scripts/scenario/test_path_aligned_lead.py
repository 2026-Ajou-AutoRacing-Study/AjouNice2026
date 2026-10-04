import math
import unittest

from overtaking_trial_policy import path_aligned_leads, RetryPassingPolicy
from test_retry_passing_policy import snapshot


def obj(name='lead', x=30., y=0., heading=0., predictions=None):
    return dict(uuid=name, pose=(x,y,heading), length=5., width=2., speed=11.,
                predictions=predictions or [])


class PathLeadTests(unittest.TestCase):
    def test_r49_001_gt30_precedes_gt26_on_published_path(self):
        # Rounded geometry from bag 39.2s; full-resolution bag replay is separate.
        path=[(194.1707,743.186),(194.1647,727.186),(194.4941,699.1891),
              (195.2243,686.5),(196.3692,671.2558),(196.5256,669.2619),
              (197.4186,657.2952),(198.422,635.3207),(198.94,552.),(199.,520.)]
        a=obj('GT26',198.94114501487712,552.043201585323,-1.55839130)
        a.update(length=6.557000160217285,width=2.5169999599456787)
        b=obj('GT30',198.29918582115093,669.7277626463106,-1.54142787)
        b.update(length=5.026000022888184,width=1.9190000295639038)
        leads=path_aligned_leads(path,(194.15994348671327,727.3971453503017,-math.pi/2),[a,b],1.89)
        self.assertEqual([o['uuid'] for o in leads],['GT30','GT26'])
        self.assertTrue(55 < leads[0]['gap'] < 60)
        self.assertEqual(leads[0]['path_match_time'],0.)

    def test_future_lane_arrival_and_adjacent_nonarrival(self):
        leads=path_aligned_leads([(0.,0.),(100.,0.)],(5.,0.,0.),
            [obj('adjacent',y=3.6),obj('joining',y=3.6,
                predictions=[(1.,(40.,3.,0.)),(2.,(50.,1.,0.))])],1.89)
        self.assertEqual([o['uuid'] for o in leads],['joining'])
        self.assertEqual(leads[0]['path_match_time'],2.)
        self.assertEqual(leads[0]['gap'],25.)  # Current gap, not future pose gap.

    def test_crossing_oncoming_behind_and_path_end_not_leaders(self):
        objects=[obj('crossing',heading=math.pi/2),obj('oncoming',heading=math.pi),
                 obj('behind',x=-1.),obj('past_end',x=101.)]
        self.assertEqual(path_aligned_leads([(0.,0.),(100.,0.)],(5.,0.,0.),objects,1.89),[])

    def test_nearest_path_not_nearest_lane_id(self):
        leads=path_aligned_leads([(0.,0.),(20.,0.),(60.,4.),(100.,4.)],(5.,0.,0.),
            [obj('future_path',x=70.,y=4.),obj('old_lane',x=60.,y=0.)],1.89)
        self.assertEqual([o['uuid'] for o in leads],['future_path'])

    def test_invalid_geometry_fails_closed(self):
        for points,ego,width in [([(0.,0.)],(0.,0.,0.),1.89),
                ([(0.,0.),(math.nan,0.)],(0.,0.,0.),1.89),
                ([(0.,0.),(100.,0.)],(5.,4.,0.),1.89),
                ([(0.,0.),(100.,0.)],(5.,0.,0.),0.)]:
            with self.assertRaises(ValueError):path_aligned_leads(points,ego,[],width)

    def test_selected_lead_does_not_grant_lane_permission_or_lease(self):
        s=snapshot();s['lead']=path_aligned_leads([(0.,0.),(100.,0.)],(5.,0.,0.),[obj()],1.89)[0]
        s.update(left_allowed=False,right_allowed=False)
        policy=RetryPassingPolicy();self.assertIsNone(policy.step(s))
        s['now']=2.;self.assertIsNone(policy.step(s));self.assertEqual(policy.attempt_count,0)


if __name__=='__main__':unittest.main()
