import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('stamped_vote_replay', Path(__file__).with_name(
    'replay_traffic_light_stamped_vote.py'))
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)
G = [['green','circle',None]]
R = [['red','circle',None]]
Y = [['yellow','circle',None]]


def frame(source, lamps=G, logged=True, identity=True, group=201365, head='C1256W000091'):
    fd = dict(group_id=group, head_id=head, normalized_tuples=copy.deepcopy(lamps),
        observed_tuples=copy.deepcopy(lamps), final_tuples=copy.deepcopy(lamps),
        raw_observation_confidence=.9 if lamps else 0., final_confidence=.9 if lamps else 0.,
        planner_compliance_observations=True,
        policy_reason='test:observed' if lamps else 'test:observation_missing',
        observation_timing=dict(guard_clock_sec=(source+30000000)/1e9,
            adopted_observation_source_stamp_ns=source if lamps else None, output_source_stamp_ns=source),
        stamped_vote=dict(enabled=logged, identity_valid=identity) if logged else dict(enabled=False))
    return m.make_frame(dict(source_stamp_ns=source, frame_decision=fd),
        dict(receipt_ros_ns=source+40000000, receipt_monotonic=1000+source/1e9, record_id=source))


def evidence(f):
    s,g,h=f['source_stamp_ns'],f['group_id'],f['head_id']
    return (dict(source_stamp_ns=s, group_id=g, head_id=h, status='matched', reason='map_head_association',
        mode='map_association', projection_source_stamp_ns=s, box_index=0,
        roi=dict(eligible=True, group_id=g, head_id=h)),
        dict(source_stamp_ns=s, group_id=g, head_id=h, detector_bbox_xyxy=[1.,2.,5.,6.], crop_xyxy=[0,0,8,8]))


class ReplayTests(unittest.TestCase):
    def test_logged_qualification_is_authoritative_not_retained_head_guess(self):
        f=frame(1000000000,identity=False)
        self.assertFalse(m.identity_for(f,None,None)['value'])
        self.assertEqual(m.identity_for(f,None,None)['basis'],'original_logged_stamped_vote_identity_valid')

    def test_legacy_requires_exact_current_crop_and_eligible_map_roi(self):
        f=frame(1000000000,logged=False);s,c=evidence(f)
        self.assertTrue(m.identity_for(f,s,c)['value'])
        self.assertTrue(m.identity_for(f,s,c)['final_vote_instant_route_and_packet_recheck_not_independently_logged'])
        c['source_stamp_ns']+=1
        self.assertIsNone(m.identity_for(f,s,c)['value'])
        self.assertIsNone(m.identity_for(f,s,None)['value'])

    def test_hard_selection_loss_false_but_missing_record_unprovable(self):
        f=frame(1000000000,logged=False)
        self.assertFalse(m.identity_for(f,dict(status='missing'),None)['value'])
        self.assertIsNone(m.identity_for(f,None,None)['value'])

    def test_replay_uses_raw_confidence_and_never_final_held_lamps_as_input(self):
        frames=[frame(1000000000),frame(1100000000),frame(1200000000,[])]
        frames[-1]['decision']['final_tuples']=R
        m.replay_frames(frames,{}, {})
        self.assertEqual(frames[-1]['original_output']['label'],'R')
        self.assertEqual(frames[-1]['replays']['vote3_strict']['label'],'G')
        self.assertEqual(frames[-1]['replays']['vote3_strict']['adopted_observation_source_stamp_ns'],1100000000)
        self.assertEqual(frames[-1]['replays']['vote3_strict']['confidence'],.9)

    def test_seven_bridges_three_unknown_only_after_sufficient_green_history(self):
        frames=[frame(1000000000+i*100000000,G if i<7 else []) for i in range(11)]
        m.replay_frames(frames,{}, {})
        self.assertEqual([f['replays']['vote7_passthrough']['label'] for f in frames[7:]],['G','G','G','UNKNOWN'])
        self.assertEqual(frames[8]['replays']['vote3_strict']['label'],'UNKNOWN')

    def test_original_B_like_warmup_does_not_bridge_and_can_create_split(self):
        inputs=[Y,G,[],[],[],G,G,G]
        frames=[frame(1000000000+i*100000000,x) for i,x in enumerate(inputs)]
        m.replay_frames(frames,{}, {})
        self.assertEqual([f['replays']['vote7_passthrough']['label'] for f in frames],
                         ['Y','G','UNKNOWN','UNKNOWN','UNKNOWN','G','UNKNOWN','G'])
        self.assertTrue(frames[4]['replays']['vote7_passthrough']['startup_passthrough_active'])
        self.assertEqual(frames[6]['replays']['vote7_passthrough']['reason'],'stamped_vote:unconfirmed')

    def test_hard_identity_loss_resets_even_full_seven_window(self):
        frames=[frame(1000000000+i*100000000) for i in range(7)]
        frames += [frame(1700000000,[],identity=False),frame(1800000000)]
        m.replay_frames(frames,{}, {})
        self.assertEqual(frames[-2]['replays']['vote7_passthrough']['reason'],'stamped_vote:identity_loss')
        self.assertEqual(frames[-1]['replays']['vote7_passthrough']['vote_source_stamps_ns'],[1800000000])
        self.assertEqual(frames[-1]['replays']['vote3_strict']['label'],'UNKNOWN')

    def test_unprovable_identity_is_unavailable_not_unknown_or_success(self):
        frames=[frame(1000000000,logged=False)]
        m.replay_frames(frames,{}, {})
        self.assertFalse(frames[0]['replays']['vote7_passthrough']['available'])
        self.assertEqual(m.label_for(frames[0],'vote7_passthrough'),'UNAVAILABLE')

    def test_legacy_corrected_phases_never_become_observation_votes(self):
        frames=[frame(1000000000)]
        frames[0]['planner_compliance_observations']=False
        m.replay_frames(frames,{}, {})
        self.assertFalse(frames[0]['replays']['vote7_passthrough']['available'])
        self.assertEqual(frames[0]['replays']['vote7_passthrough']['reason'],'input_not_logged_observed_only')

    def test_three_and_seven_delay_new_red_in_full_window_source_stamps(self):
        frames=[frame(1000000000+i*100000000,G if i<7 else R) for i in range(11)]
        m.replay_frames(frames,{}, {})
        events=m.transition_lags(frames,[])
        red=next(e for e in events if e['new_normalized_raw_phase']=='R')
        self.assertAlmostEqual(red['streams']['vote3_strict']['first_input_to_winner_original_source_sec'],.1)
        self.assertAlmostEqual(red['streams']['vote7_passthrough']['first_input_to_winner_original_source_sec'],.3)

    def test_old_strict_three_golden_lamps_and_original_winner_sources(self):
        inputs=[G,G,R,R,[],[],G,G]
        frames=[frame(1000000000+i*100000000,x) for i,x in enumerate(inputs)]
        m.replay_frames(frames,{}, {})
        outputs=[f['replays']['vote3_strict'] for f in frames]
        self.assertEqual([x['label'] for x in outputs],['UNKNOWN','G','G','R','R','UNKNOWN','UNKNOWN','G'])
        self.assertEqual([x['adopted_observation_source_stamp_ns'] for x in outputs],
                         [None,1100000000,1100000000,1300000000,1300000000,None,None,1700000000])
        self.assertEqual([x['confidence'] for x in outputs],[0.,.9,.9,.9,.9,0.,0.,.9])

    def test_mode_activity_distinguishes_warmup_full_unconfirmed_and_resets(self):
        inputs=[Y,G,[],[],[],G,G,G]
        frames=[frame(1000000000+i*100000000,x) for i,x in enumerate(inputs)]
        frames.append(frame(1800000000,[],identity=False))
        m.replay_frames(frames,{}, {})
        activity=m.case_activity(frames,'vote7_passthrough')
        self.assertEqual(activity['counts']['startup_passthrough_frames'],6)
        self.assertEqual(activity['counts']['window_full_frames'],2)
        self.assertEqual(activity['counts']['unconfirmed_full_window_frames'],1)
        self.assertEqual(activity['identity_loss_reset_count'],1)

    def test_six_hz_underfull_queue_does_not_reopen_cold_startup(self):
        frames=[frame(1000000000+i*166666667,G if i<7 else R) for i in range(8)]
        m.replay_frames(frames,{}, {})
        output=frames[-1]['replays']['vote7_passthrough']
        self.assertEqual(output['candidate_version'],4)
        self.assertEqual(output['accepted_count'],7)
        self.assertEqual(output['available_vote_count'],6)
        self.assertFalse(output['window_full'])
        self.assertTrue(output['startup_complete'])
        self.assertFalse(output['startup_passthrough_active'])
        self.assertEqual(output['label'],'G')
        activity=m.case_activity(frames,'vote7_passthrough')
        self.assertEqual(activity['counts']['startup_passthrough_frames'],6)
        self.assertEqual(activity['counts']['majority_evaluation_underfull_window_frames'],2)

    def test_source_reversal_clears_votes_not_completed_startup_ledger(self):
        frames=[frame(1000000000+i*100000000) for i in range(7)]
        reversal=frame(1550000000,R)
        reversal['guard_clock_ns']=1730000000
        reversal['guard_clock_sec_as_logged']=1.73
        frames += [reversal,frame(1800000000,R)]
        m.replay_frames(frames,{}, {})
        self.assertEqual(frames[-2]['replays']['vote7_passthrough']['reset_reason'],'source_reversal')
        output=frames[-1]['replays']['vote7_passthrough']
        self.assertTrue(output['startup_complete'])
        self.assertFalse(output['startup_passthrough_active'])
        self.assertEqual(output['label'],'UNKNOWN')
        self.assertEqual(m.case_activity(frames,'vote7_passthrough')['source_reset_count'],1)

    def test_clock_reversal_starts_new_cold_epoch(self):
        frames=[frame(1000000000+i*100000000) for i in range(7)]
        frames.append(frame(500000000,R))
        m.replay_frames(frames,{}, {})
        output=frames[-1]['replays']['vote7_passthrough']
        self.assertEqual(output['reset_reason'],'clock_reversal')
        self.assertFalse(output['startup_complete'])
        self.assertEqual(output['accepted_count'],1)
        self.assertTrue(output['startup_passthrough_active'])
        self.assertEqual(output['label'],'R')

    def test_yellow_winner_is_fourth_observation_not_onset_clock(self):
        frames=[frame(1000000000+i*100000000,G if i<7 else Y) for i in range(11)]
        m.replay_frames(frames,{}, {})
        event=next(e for e in m.transition_lags(frames,[]) if e['new_normalized_raw_phase']=='Y')
        self.assertFalse(event['yellow_onset_caution']['timed_yellow_ON_compatibility_verified'])
        winner=event['streams']['vote7_passthrough']
        self.assertEqual(winner['winner_original_source_stamp_ns'],2000000000)
        self.assertAlmostEqual(winner['first_input_to_winner_original_source_sec'],.3)
        self.assertAlmostEqual(winner['first_input_to_output_guard_sec'],.33)
        self.assertAlmostEqual(winner['first_input_to_original_observer_receipt_of_output_frame_sec'],.34)

    def test_no_closed_wrapper_no_replay_and_source_bytes_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'observer').mkdir();(root/'selection.json').write_text('{}')
            path=root/'observer/observations.jsonl';path.write_text('')
            original=path.read_bytes()
            with self.assertRaisesRegex(ValueError,'Completed wrapper'):
                m.analyze(root)
            self.assertEqual(path.read_bytes(),original)


if __name__=='__main__':
    unittest.main()
