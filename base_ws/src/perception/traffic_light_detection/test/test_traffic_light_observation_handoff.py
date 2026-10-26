import copy
import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1]/'scripts/traffic_light_observation_handoff.py'
SPEC = importlib.util.spec_from_file_location('handoff_under_test', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DetectorMissEligibilityTest(unittest.TestCase):
    def setUp(self):
        self.packet = dict(status='ready', group_id=201326, source_stamp_ns=10_000_000_000,
                           generated_stamp_ns=10_010_000_000,
                           rois=[dict(eligible=True, group_id=201326, head_id='D052')])
        self.selection = dict(status='missing', reason='no_unambiguous_route_detection', rejected=[])

    def decide(self, packet=None, selection=None, **kw):
        return MODULE.detection_miss_context(
            self.packet if packet is None else packet,
            self.selection if selection is None else selection,
            kw.get('head', 'D052'), kw.get('group', 201326),
            kw.get('stamp', 10_000_000_000), kw.get('now', 10_050_000_000))

    def test_current_same_housing_detector_miss(self):
        self.assertTrue(self.decide()['eligible'])

    def test_no_prior_housing_or_other_mission(self):
        self.assertFalse(self.decide(head=None)['eligible'])
        self.assertFalse(self.decide(group=201351)['eligible'])
        self.assertFalse(self.decide(head='other')['eligible'])

    def test_wrong_or_stale_image_and_future_packet(self):
        self.assertFalse(self.decide(stamp=10_000_000_001)['eligible'])
        self.assertFalse(self.decide(now=11_000_000_000)['eligible'])
        self.assertFalse(self.decide(now=9_000_000_000)['eligible'])

    def test_missing_localization_or_fov_is_not_detector_dropout(self):
        p = copy.deepcopy(self.packet); p['status'] = 'missing'
        self.assertFalse(self.decide(packet=p)['eligible'])
        p = copy.deepcopy(self.packet); p['rois'] = []
        self.assertFalse(self.decide(packet=p)['eligible'])
        p = copy.deepcopy(self.packet); p['rois'][0]['eligible'] = False
        self.assertFalse(self.decide(packet=p)['eligible'])

    def test_identity_ambiguity_never_holds_go(self):
        s = dict(self.selection, rejected=[dict(reason='other_head_identity_ambiguous')])
        self.assertFalse(self.decide(selection=s)['eligible'])

    def test_other_nonmatching_objects_do_not_manufacture_a_new_observation(self):
        s = dict(self.selection, rejected=[dict(reason='no_route_head_match')])
        result = self.decide(selection=s)
        self.assertTrue(result['eligible'])
        self.assertNotIn('lamps', result)
        self.assertNotIn('source_stamp_ns', result)

    def test_current_detected_unknown_cannot_enter_detector_miss_hold(self):
        s = dict(status='matched', reason='map_head_association', head_id='D052')
        self.assertFalse(self.decide(selection=s)['eligible'])

    def test_duplicate_or_malformed_projection_is_not_eligible(self):
        p = copy.deepcopy(self.packet); p['rois'] *= 2
        self.assertFalse(self.decide(packet=p)['eligible'])
        p = copy.deepcopy(self.packet); del p['generated_stamp_ns']
        self.assertFalse(self.decide(packet=p)['eligible'])


class SoftUnknownEligibilityTest(unittest.TestCase):
    def setUp(self):
        self.packet=dict(status='ready',group_id=201326,source_stamp_ns=10_000_000_000,
            generated_stamp_ns=10_010_000_000,
            rois=[dict(eligible=True,group_id=201326,head_id='D052')],
            motion_evidence=dict(schema_version=1,valid=True,aligned_speed_mps=2.,latest_speed_mps=2.,
                aligned_pose_stamp_ns=10_000_000_000,latest_pose_stamp_ns=10_020_000_000,
                latest_pose_receipt_stamp_ns=10_020_000_000))
        self.selection=dict(status='matched',head_id='D052')
        self.classifier=dict(known=False,reason='low_confidence_bits:left_arrow',
                             bits=dict(red=1,yellow=0,green=0,left_arrow=None))

    def decide(self,**kw):
        return MODULE.low_confidence_context(kw.get('packet',self.packet),
            kw.get('selection',self.selection),kw.get('classifier',self.classifier),
            kw.get('head','D052'),kw.get('group',201326),
            kw.get('source',10_000_000_000),kw.get('now',10_050_000_000))

    def test_fresh_same_head_and_both_moving_pose_samples_qualify(self):
        self.assertTrue(self.decide()['eligible'])
        self.assertTrue(self.decide()['confident_red'])

    def test_no_motion_and_speed_threshold_are_not_inferred_from_map(self):
        for speed in (0.,.99,float('nan'),float('inf'),True,'2',None):
            for field in ('aligned_speed_mps','latest_speed_mps'):
                with self.subTest(field=field,speed=speed):
                    p=copy.deepcopy(self.packet);p['motion_evidence'][field]=speed
                    self.assertFalse(self.decide(packet=p)['eligible'])
        p=copy.deepcopy(self.packet);del p['motion_evidence']
        self.assertFalse(self.decide(packet=p)['eligible'])
        p=copy.deepcopy(self.packet)
        p['motion_evidence'].update(aligned_speed_mps=1.,latest_speed_mps=1.)
        self.assertTrue(self.decide(packet=p)['eligible'])

    def test_lost_head_route_tf_or_exact_image_identity_cannot_hold(self):
        for changes in (dict(status='missing'),dict(group_id=0),
                        dict(source_stamp_ns=10_000_000_001),dict(rois=[])):
            p=copy.deepcopy(self.packet);p.update(changes)
            self.assertFalse(self.decide(packet=p)['eligible'])
        self.assertFalse(self.decide(head='other')['eligible'])
        self.assertFalse(self.decide(group=0)['eligible'])
        self.assertFalse(self.decide(selection=dict(status='missing',head_id='D052'))['eligible'])

    def test_real_lamps_dark_or_model_failure_are_not_soft_unknowns(self):
        for classifier in (dict(known=True,reason='no_lamps'),dict(known=False,reason='inference_error:x'),
                           dict(known=False,reason='invalid_probabilities:x')):
            self.assertFalse(self.decide(classifier=classifier)['eligible'])
        c=copy.deepcopy(self.classifier);c['bits']['yellow']=1
        self.assertFalse(self.decide(classifier=c)['eligible'])

    def test_future_stale_misaligned_or_reordered_motion_samples_reject(self):
        for field,value in (('aligned_pose_stamp_ns',9_800_000_000),
                            ('latest_pose_stamp_ns',10_100_000_000),
                            ('latest_pose_stamp_ns',9_999_999_999),
                            ('latest_pose_receipt_stamp_ns',9_700_000_000),
                            ('latest_pose_receipt_stamp_ns',10_100_000_000)):
            p=copy.deepcopy(self.packet);p['motion_evidence'][field]=value
            self.assertFalse(self.decide(packet=p)['eligible'])
        self.assertFalse(self.decide(now=10_600_000_000)['eligible'])
        self.assertFalse(self.decide(now=9_999_999_999)['eligible'])


if __name__ == '__main__':
    unittest.main()
