#!/usr/bin/env python3
import unittest
from overtaking_trial_policy import Policy, verified_route_status, forward_lane_chain, ownership_busy, authoritative_inactive


def snapshot(now=0):
    return dict(valid=True, now=now, busy=False, remaining=300, left_allowed=True,
        right_allowed=True, desired_speed=30, ego_speed=20,
        lead=dict(uuid='lead', gap=35, speed=16), tracked_lead_present=True,
        fully_passed=False, rtc={'left':[dict(uuid='L', state=0, safe=True)],
                              'right':[dict(uuid='R', state=0, safe=True)]})


class Tests(unittest.TestCase):
    def test_authoritative_snapshot_route_and_freshness(self):
        snap=dict(valid=True,complete_snapshot=True,stamp=10.,route_uuid='route',route_stamp=5.,
                  route_generation=99,modules=[])
        proof=authoritative_inactive(snap,10.1,'route',5.,['left'])
        self.assertEqual(proof['left']['active'],False)
        self.assertIsNone(authoritative_inactive(snap,10.5,'route',5.,['left']))
        self.assertIsNone(authoritative_inactive(snap,10.1,'other',5.,['left']))
        self.assertIsNone(authoritative_inactive(snap,10.1,'route',6.,['left']))
        self.assertIsNone(authoritative_inactive(dict(snap,valid=False),10.1,'route',5.,['left']))
        snap['modules']=[dict(name='left',active=True,approved=True,state=1,waiting_approval=False,rtc_uuid_available=True,
                             rtc_uuids=[dict(interface='',uuid='new')])]
        self.assertTrue(authoritative_inactive(snap,10.1,'route',5.,['left'])['left']['active'])
        proof=authoritative_inactive(snap,10.1,'route',5.,['left'])
        self.assertFalse(ownership_busy({'left':[dict(uuid='old',state=2,stamp=2.)]},10.1,proof)[0])
        self.assertTrue(proof['left']['active'])  # New owner must still block new requests.

    def test_waiting_candidate_does_not_own_but_approved_wait_does(self):
        module=dict(name='left',active=False,approved=False,state=1,waiting_approval=True,
                    rtc_uuid_available=True,rtc_uuids=[dict(interface='',uuid='L')])
        snap=dict(valid=True,complete_snapshot=True,stamp=10.,route_uuid='route',route_stamp=5.,modules=[module])
        proof=authoritative_inactive(snap,10.1,'route',5.,['left'])
        self.assertFalse(proof['left']['active'])
        p=Policy();s=snapshot();s['busy']=any(v['active'] for v in proof.values())
        p.step(s);s['now']=2
        self.assertEqual(p.step(s),'left')
        # An approved owner cannot be released by a renewed approval wait.
        module.update(approved=True,active=True)
        proof=authoritative_inactive(snap,10.1,'route',5.,['left'])
        self.assertTrue(proof['left']['active'])
        p=Policy();s=snapshot();s['busy']=True
        p.step(s);s['now']=2;self.assertIsNone(p.step(s))
        module.update(approved=False,active=True)
        self.assertIsNone(authoritative_inactive(snap,10.1,'route',5.,['left']))
        module.update(waiting_approval=False)
        self.assertTrue(authoritative_inactive(snap,10.1,'route',5.,['left'])['left']['active'])

    def test_successor_chain_not_adjacent_or_ambiguous_fork(self):
        graph={1:[2],2:[3],3:[4,5],8:[9]}
        self.assertEqual(forward_lane_chain(1,graph),{1,2,3})
        self.assertNotIn(8,forward_lane_chain(1,graph))
        self.assertEqual(forward_lane_chain(1,{1:[2],2:[1]}),{1,2})

    def test_stale_rtc_needs_authoritative_inactive_evidence(self):
        status={'left':[dict(uuid='old',state=2,stamp=2.)]}
        self.assertTrue(ownership_busy(status,10.)[0])
        self.assertEqual(ownership_busy(status,10.)[1][0]['status'],'stale_unresolved')
        self.assertTrue(ownership_busy(status,10.,{'left':dict(active=False,stamp=9.)})[0])
        self.assertFalse(ownership_busy(status,10.,{'left':dict(active=False,stamp=9.9)})[0])
        status['left'][0]['stamp']=9.8
        self.assertTrue(ownership_busy(status,10.,{'left':dict(active=False,stamp=9.9)})[0])

    def test_wait_can_observe_terminal_without_running_sample(self):
        for terminal, expected in [(3, 'PASSING'), (4, 'HOLD')]:
            p=Policy();s=snapshot();p.step(s);s['now']=2;p.step(s)
            s['rtc']['left'][0]['state']=terminal
            self.assertIsNone(p.step(s));self.assertEqual(p.state,expected)

    def test_missing_running_owner_never_implies_success(self):
        p=Policy();s=snapshot();p.step(s);s['now']=2;p.step(s)
        s['rtc']['left'][0]['state']=1;p.step(s)
        s['rtc']['left']=[];s['now']=3
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'LEFT_RUNNING')
        s['now']=5.1
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'HOLD')
        self.assertEqual(p.reason,'running_owner_missing_unresolved')

    def test_verified_route_lifecycle(self):
        status = dict(phase='VERIFIED', verified_active='route_b', route_key='route_b',
                      uncertain=False, busy=False, generation=1, stamp=101.)
        self.assertTrue(verified_route_status(status, 100.))
        self.assertFalse(verified_route_status(status, 102.))
        for update in [dict(phase='REQUESTED'),dict(uncertain=True),dict(verified_active='route_a'),dict(busy=True)]:
            changed = dict(status);changed.update(update)
            self.assertFalse(verified_route_status(changed, 100.))

    def test_server_uuid_race_holds_without_new_request(self):
        p=Policy();s=snapshot();p.step(s);s['now']=2;p.step(s)
        s['rtc']['left']=[dict(uuid='raced',state=1,safe=True,auto_mode=True)]
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'HOLD')
        self.assertIn('different_uuid',p.reason)

    def test_equalized_speed_still_allows_pass(self):
        p=Policy();s=snapshot();s['ego_speed']=16
        self.assertIsNone(p.step(s));s['now']=1.1
        self.assertEqual(p.step(s),'left')

    def test_stale_busy_and_reserve_block(self):
        for update in [dict(valid=False), dict(busy=True), dict(remaining=149),dict(left_allowed=False)]:
            p=Policy();s=snapshot();s.update(update);p.step(s);s['now']=2
            self.assertIsNone(p.step(s))

    def test_ambiguous_or_unsafe(self):
        p=Policy();s=snapshot();p.step(s);s['now']=2;s['rtc']['left'][0]['safe']=False
        self.assertIsNone(p.step(s));s['rtc']['left']*=2
        self.assertIsNone(p.step(s))

    def test_uuid_replacement_not_authorized(self):
        p=Policy();s=snapshot();p.step(s);s['now']=2;p.step(s)
        s['rtc']['left'][0]['uuid']='changed'
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'HOLD')

    def test_pass_then_safe_return(self):
        p=Policy();s=snapshot();p.step(s);s['now']=2;self.assertEqual(p.step(s),'left')
        s['rtc']['left'][0]['state']=1;s['busy']=True
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'LEFT_RUNNING')
        s['rtc']['left'][0]['state']=3;s['busy']=False;p.step(s)
        self.assertEqual(p.state,'PASSING');self.assertIsNone(p.step(s))
        s['fully_passed']=True;self.assertEqual(p.step(s),'right')

    def test_missing_lead_and_bottleneck_no_return(self):
        p=Policy();p.state='PASSING';s=snapshot();s['fully_passed']=True
        s['tracked_lead_present']=False;self.assertIsNone(p.step(s))
        s['tracked_lead_present']=True;s['remaining']=79
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'HOLD')


if __name__ == '__main__':
    unittest.main()
