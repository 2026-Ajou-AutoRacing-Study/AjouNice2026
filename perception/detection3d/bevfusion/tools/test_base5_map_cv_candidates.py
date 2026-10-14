import math
import unittest
from types import SimpleNamespace as N

from experiment_base5_map_cv_candidates import history_fit, history_rank, gates, lateral_paths, prepare_network


class CandidateTests(unittest.TestCase):
    def test_insufficient_history_falls_back(self):
        self.assertIsNone(history_fit([(0.,0.,0.),(.1,1.,0.)],.1))

    def test_fit_is_causal_and_velocity_has_physical_units(self):
        history=[(i/10,i,0.) for i in range(6)]+[(9.,10000.,10000.)]
        fit=history_fit(history,.5)
        self.assertAlmostEqual(fit['velocity'][0],10.)
        self.assertAlmostEqual(fit['position'][0],5.)

    def test_large_gap_rejected(self):
        self.assertIsNone(history_fit([(0.,0.,0.),(.1,1.,0.),(.4,4.,0.),(.5,5.,0.)],.5))

    def test_noisy_history_rejected(self):
        self.assertIsNone(history_fit([(i/10,i,(-1)**i*2.) for i in range(6)],.5))

    def test_missing_history_does_not_change_paths(self):
        paths=(N(confidence=.5),N(confidence=.5))
        self.assertIs(history_rank(None,paths,None),paths)

    def test_acceptance_requires_each_case_and_tail(self):
        def metric(mean,p95):return dict(top1=dict(n=40,mean=mean,p95=p95))
        r={'case':dict(metrics={'baseline':{str(h):metric(1.,2.) for h in (1,2,3)},
                               'trial':{str(h):metric(.9,2.2) for h in (1,2,3)}})}
        self.assertFalse(gates(r,'trial')['pass_all'])

    def network(self):
        from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord
        network=LaneletNetwork([LaneletRecord(1,((0.,0.,0.),(100.,0.,0.)))])
        prepare_network(network)
        return network

    def test_history_does_not_change_sibling_branch_ratio(self):
        from perception_object_pipeline.map_based_prediction import PredictedTrajectory
        network=self.network()
        paths=(PredictedTrajectory(((0.,0.,0.,0.),),.7,(1,2)),
               PredictedTrajectory(((0.,1.,0.,0.),),.3,(1,3)))
        fit=history_fit([(i/10,i,1.) for i in range(6)],.5)
        out=history_rank(network,paths,fit,dict(center=[5.,1.,0.],yaw=0.))
        self.assertEqual({p.lanelet_ids for p in out},{(1,2),(1,3)})
        self.assertAlmostEqual(out[0].confidence/out[1].confidence,.7/.3)
        self.assertAlmostEqual(sum(p.confidence for p in out),1.)

    def test_lateral_motion_mirror_and_terminal_centerline(self):
        network=self.network(); horizon=tuple(i/10 for i in range(1,31))
        results=[]
        for sign in (1.,-1.):
            obj=dict(center=[10.,sign,0.],yaw=0.,body_velocity=[5.,0.])
            paths=network.predict(10.,sign,0.,0.,5.,horizon)
            results.append(lateral_paths(network,paths,obj,dict(velocity=[5.,sign*.5]))[0])
        self.assertGreater(results[0].poses[0][1],1.)
        self.assertAlmostEqual(results[0].poses[0][1],-results[1].poses[0][1])
        self.assertAlmostEqual(results[0].poses[-1][1],0.)
        self.assertAlmostEqual(results[0].poses[-1][0],25.)

    def test_lateral_missing_history_preserves_exact_baseline(self):
        paths=(N(confidence=1.),)
        self.assertIs(lateral_paths(None,paths,None,None),paths)

    def test_rare_regression_is_not_hidden_by_p95_of_all_frames(self):
        from review_base5_map_cv_candidate_risk import summarize
        rows=[]
        for i in range(100):
            rows.append(dict(stamp=i/10,id=1,gt_key=['case','actor'],horizon=3,
                errors={'baseline':dict(top1=1.,minK=1.),
                        'route_history':dict(top1=21. if i==50 else 1.,minK=1.)}))
        risk=summarize(dict(rows=rows))
        self.assertEqual(risk['deltas']['3']['all_observations']['p95'],0.)
        self.assertEqual(risk['deltas']['3']['changed_only']['p95'],20.)
        self.assertEqual(risk['deltas']['3']['extra_error_gt5m'],1)


if __name__=='__main__':unittest.main()
