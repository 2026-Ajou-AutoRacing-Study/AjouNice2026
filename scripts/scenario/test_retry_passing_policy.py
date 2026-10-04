import copy
import unittest
from overtaking_trial_policy import RetryPassingPolicy, atomic_response_outcome
from test_passing_policy import snapshot as basic_snapshot


def snapshot(now=0):
    s=basic_snapshot();s.update(now=now,ros_now=100+now,ownership_sequence=int(now*100)+1,
        ownership_evidence=[],rtc_auto_owners=[],mandatory_priority=False)
    s['authoritative_owners']={name:dict(active=False,active_uuids=[],uuid_complete=True,
        stamp=s['ros_now']) for name in RetryPassingPolicy.MODULES}
    return s


def tick(s,now):
    s=copy.deepcopy(s);s.update(now=now,ros_now=100+now,ownership_sequence=int(now*100)+1)
    for value in s['authoritative_owners'].values():value['stamp']=s['ros_now']
    return s


class RetryTests(unittest.TestCase):
    def arm(self):
        p=RetryPassingPolicy();s=snapshot();p.step(s);s=tick(s,1.1);p.step(s)
        self.assertEqual(p.state,'ARM');return p,s

    def failed(self):
        p,s=self.arm();s['rtc']['right'][0]['state']=4;p.step(s)
        self.assertEqual(p.state,'COOLDOWN');return p,s

    def settle(self,p,s,start=3.2):
        for t in (start,start+.2,start+.4):
            s=tick(s,t);self.assertIsNone(p.step(s))
        return s

    def test_failed_uuid_tombstone_new_uuid_new_intent_and_ack(self):
        p,s=self.failed();old_intent=p.intent_id
        s=self.settle(p,s);self.assertEqual(p.state,'IDLE')
        s['rtc']['right']=[dict(uuid='R',state=0,safe=True)]
        p.step(tick(s,4));p.step(tick(s,5.2));self.assertEqual(p.state,'IDLE')
        s['rtc']['right'][0]['uuid']='R2'
        p.step(tick(s,5.4));p.step(tick(s,6.6));self.assertEqual(p.state,'ARM')
        self.assertEqual(p.candidate,'R2');self.assertNotEqual(p.intent_id,old_intent)
        self.assertIsNone(p.step(tick(s,6.8)))
        s['intent_ack']=True;self.assertEqual(p.step(tick(s,7)),'right')
        self.assertEqual(p.attempt_count,2)

    def test_cache_does_not_prove_settled(self):
        p,s=self.failed();s=tick(s,3.2);p.step(s)
        for t in (3.4,3.6,4):
            s['now']=t;p.step(s)
        self.assertEqual(p.state,'COOLDOWN')

    def test_incomplete_active_unknown_stale_owners_block(self):
        for case in ('missing','active','incomplete','uuid','oldstamp','busy','unresolved','lease'):
            p,s=self.failed();s=tick(s,3.2);key=p.MODULES[0]
            if case=='missing':del s['authoritative_owners'][key]
            elif case=='active':s['authoritative_owners'][key]['active']=True
            elif case=='incomplete':s['authoritative_owners'][key]['uuid_complete']=False
            elif case=='uuid':s['authoritative_owners'][key]['active_uuids']=['X']
            elif case=='oldstamp':s['authoritative_owners'][key]['stamp']-=1
            elif case=='busy':s['busy']=True
            elif case=='unresolved':s['ownership_evidence']=[dict(status='stale_unresolved')]
            else:s['rtc_auto_owners']=[dict(uuid='X')]
            p.step(s);self.assertEqual(p.state,'COOLDOWN',case)
            self.assertIsNone(p.settle_stamp,case)

    def test_minimum_cooldown_even_with_fresh_settlement(self):
        p,s=self.failed();self.settle(p,s,1.2)
        self.assertEqual(p.state,'COOLDOWN');self.assertEqual(p.reason,'retry_cooldown')

    def test_gap_in_authority_restarts_settlement(self):
        p,s=self.failed();p.step(tick(s,3.2));p.step(tick(s,4))
        self.assertEqual(p.state,'COOLDOWN');p.step(tick(s,4.2));p.step(tick(s,4.4))
        self.assertEqual(p.state,'IDLE')

    def test_future_stamp_or_nonincreasing_sequence_fail_closed(self):
        p,s=self.failed();p.step(tick(s,3.2));bad=tick(s,3.4)
        bad['ownership_sequence']=1;p.step(bad)
        self.assertIsNone(p.settle_stamp);self.assertEqual(p.state,'COOLDOWN')
        future=tick(s,3.6)
        for owner in future['authoritative_owners'].values():owner['stamp']+=.001
        p.step(future);self.assertIsNone(p.settle_stamp)

    def test_pregrant_candidate_loss_retires_exact_uuid(self):
        p,s=self.arm();s['rtc']['right'][0]['uuid']='replacement';p.step(s)
        self.assertEqual(p.state,'COOLDOWN');self.assertEqual(p.retired,{'R'})
        self.settle(p,s);self.assertEqual(p.state,'IDLE')

    def test_invalid_input_resets_settlement(self):
        p,s=self.failed();p.step(tick(s,3.2));bad=tick(s,3.4);bad['valid']=False
        p.step(bad);p.step(tick(s,3.6));self.assertEqual(p.state,'COOLDOWN')

    def test_mandatory_reserve_or_negative_ack_has_priority(self):
        for fields in ({'remaining':140},{'remaining':400,'ack_reserve':450},
                       {'mandatory_priority':True}):
            p,s=self.failed();s.update(fields);self.settle(p,s)
            self.assertEqual(p.state,'COOLDOWN');self.assertEqual(p.reason,'retry_mandatory_route_priority')

    def test_rpc_rejection_is_recoverable_but_ambiguity_latches(self):
        p,s=self.arm();p.record_rpc_result(s,False);self.assertEqual(p.state,'COOLDOWN')
        for result in (None,'transport_error'):
            p,s=self.arm();p.record_rpc_result(s,result);self.settle(p,s)
            self.assertEqual(p.state,'HOLD');self.assertEqual(p.reason,'atomic_uuid_lease_ambiguous')

    def test_positive_grant_waiting_does_not_repeat_rpc(self):
        p,s=self.arm();s['intent_ack']=True
        self.assertEqual(p.step(s),'right');p.record_rpc_result(s,True)
        self.assertEqual(p.state,'OUTBOUND')
        s['rtc']['right'][0]['auto_mode']=True
        s['rtc_auto_owners']=[dict(uuid='R')]
        for t in (1.3,1.5,1.7,1.9):
            self.assertIsNone(p.step(tick(s,t)));self.assertEqual(p.state,'OUTBOUND')
        s['rtc']['right'][0]['state']=1
        self.assertIsNone(p.step(tick(s,2)));self.assertEqual(p.state,'OUTBOUND')
        s['rtc']['right'][0]['state']=3
        self.assertIsNone(p.step(tick(s,2.2)));self.assertEqual(p.state,'PASSING')

    def test_positive_grant_never_observed_running_latches_hold(self):
        p,s=self.arm();p.record_rpc_result(s,True)
        for t in (1.4,1.6,2,3.2):self.assertIsNone(p.step(tick(s,t)))
        self.assertEqual(p.state,'HOLD');self.assertEqual(p.reason,'outbound_completion_missing')

    def test_exact_negative_only_response_classification(self):
        self.assertIs(atomic_response_outcome('X',7,[dict(uuid='X',module=7,success=False)]),False)
        self.assertIs(atomic_response_outcome('X',7,[dict(uuid='X',module=7,success=True)]),True)
        for responses in ([],[dict(uuid='Y',module=7,success=False)],
                [dict(uuid='X',module=8,success=False)],[dict(uuid='X',module=7,success=0)]):
            self.assertIsNone(atomic_response_outcome('X',7,responses))

    def test_missing_running_completion_never_retries(self):
        p,s=self.arm();s['rtc']['right'][0]['state']=1;p.step(s)
        s['rtc']['right']=[];p.step(tick(s,2));p.step(tick(s,4.1))
        self.assertEqual(p.state,'HOLD');self.settle(p,s,6)
        self.assertEqual(p.reason,'outbound_completion_missing')

    def test_running_or_aborting_never_rpc_retry(self):
        for state in (1,2):
            p,s=self.arm();s['rtc']['right'][0]['state']=state
            s.update(busy=True,remaining=20,intent_ack=True)
            for t in (2,3,4):self.assertIsNone(p.step(tick(s,t)))
            self.assertEqual(p.state,'OUTBOUND')

    def test_other_bound_uuid_latches_and_never_retries(self):
        p,s=self.arm();s['rtc']['left']=[dict(uuid='other',state=0,safe=True,auto_mode=True)]
        p.step(s);self.settle(p,s);self.assertEqual(p.state,'HOLD')
        self.assertEqual(p.reason,'server_bound_other_uuid')

    def test_route_or_boot_change_latches(self):
        for key,value in (('intent_route',('different',1)),('intent_boot','newboot')):
            p,s=self.failed();s[key]=value;p.step(s);self.assertEqual(p.state,'HOLD')
        p,s=self.failed();s.update(valid=False,reason='ownership_epoch_changed_restart_policy_required')
        p.step(s);self.assertEqual(p.state,'HOLD')

    def test_completed_return_allows_next_attempt_not_forced_return(self):
        p,s=self.arm();s['rtc']['right'][0]['state']=3;p.step(s)
        self.assertEqual(p.state,'PASSING');s['fully_passed']=True;p.step(s)
        self.assertEqual(p.state,'RETURN_RELEASED');p.step(s)
        self.assertEqual(p.state,'COOLDOWN');self.assertEqual(p.outcome_reason,'normal_return_observed')
        self.settle(p,s);self.assertEqual(p.state,'IDLE');self.assertIn('R',p.retired)

    def test_no_intent_during_cooldown_and_latched_hold(self):
        p,s=self.failed();self.assertIsNone(p.intent(s,1,'map'))
        p.record_rpc_result(s,None);self.assertIsNone(p.intent(s,1,'map'))

    def test_retired_uuid_does_not_hide_other_concurrent_waiting(self):
        p,s=self.failed();self.settle(p,s)
        s['rtc']['right']=[dict(uuid=x,state=0,safe=True) for x in ('R','R2')]
        p.step(tick(s,5));p.step(tick(s,6.2));self.assertEqual(p.state,'IDLE')

    def test_auto_lease_appearing_after_rearm_blocks_new_attempt(self):
        p,s=self.failed();self.settle(p,s);s['rtc_auto_owners']=[dict(uuid='X')]
        p.step(tick(s,5));p.step(tick(s,6.2));self.assertEqual(p.state,'IDLE')
        self.assertEqual(p.reason,'existing_external_auto_lease')


if __name__=='__main__':unittest.main()
