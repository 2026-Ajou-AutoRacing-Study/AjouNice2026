import copy
import json
import math
import unittest

from analyze_base5_runtime_effects import compare, delivered, drift, match, distribution


def detection(x=0.,yaw=0.,score=.8,label=0,size=(4.,2.,1.5)):
    return dict(center=[x,0.,0.],size=list(size),yaw=yaw,score=score,label=label,
        class_name=('vehicle','pedestrian','obstacle')[label])


class RuntimeEffectsTests(unittest.TestCase):
    def test_full_distribution_and_empty_population(self):
        r=distribution([0.,1.,2.,3.,4.])
        self.assertEqual(r['mean'],2.)
        self.assertEqual(r['p50'],2.)
        self.assertAlmostEqual(r['p90'],3.6)
        self.assertAlmostEqual(r['p95'],3.8)
        self.assertIsNone(distribution([])['mean'])
        with self.assertRaises(ValueError):distribution([float('nan')])

    def test_score_filter_nms_and_class_separation(self):
        ds=[detection(score=.8),detection(score=.7),detection(x=8,score=.39),
            detection(score=.16,label=1)]
        self.assertEqual([d['score'] for d in delivered(ds,(.4,.15,.1))],[.8,.16])

    def test_symmetric_footprint_does_not_invent_a_boundary_shift(self):
        a=detection();b=detection(yaw=math.pi)
        self.assertAlmostEqual(drift(a,b)['footprint_corner_m'],0,places=12)
        self.assertAlmostEqual(drift(a,b)['rotated_bev_iou'],1,places=12)

    def test_geometry_and_confidence_regime_are_separate(self):
        row=drift(detection(score=.4999),detection(x=.002,score=.5001))
        self.assertAlmostEqual(row['center_xy_m'],.002)
        self.assertTrue(row['tracker_confidence_regime_flip'])

    def test_matching_is_gated_and_one_to_one(self):
        pairs,removed,added=match([detection(),detection(5)],[detection(.1),detection(20)])
        self.assertEqual(pairs,[(0,0)]);self.assertEqual(removed,[1]);self.assertEqual(added,[1])
        self.assertEqual(match([], [detection()]), ([],[],[0]))

    def test_threshold_crossing_is_counted_not_hidden_as_small_score_drift(self):
        old={'frame':[detection(score=.3999)]};new={'frame':[detection(score=.4001)]}
        result=compare(old,new,['frame'],(.4,.15,.1))['vehicle']
        self.assertEqual(result['added'],1);self.assertEqual(result['matched'],0)

    def test_results_are_json_serializable_and_do_not_modify_inputs(self):
        old={'frame':[detection()]};new={'frame':[detection(yaw=.03)]};before=copy.deepcopy(old)
        result=compare(old,new,['frame'],(.4,.15,.1))
        self.assertEqual(result['vehicle']['yaw_gate_exceedances'],1)
        json.dumps(result,allow_nan=False);self.assertEqual(old,before)

    def test_wrong_class_and_nonfinite_payloads_are_rejected(self):
        d=detection();d['class_name']='obstacle'
        with self.assertRaises(ValueError):delivered([d],(.4,.15,.1))
        d=detection(x=float('nan'))
        with self.assertRaises(ValueError):delivered([d],(.4,.15,.1))
        for d in (detection(score=-.1),detection(size=(0,2,1))):
            with self.assertRaises(ValueError):delivered([d],(.4,.15,.1))


if __name__=='__main__':unittest.main()
