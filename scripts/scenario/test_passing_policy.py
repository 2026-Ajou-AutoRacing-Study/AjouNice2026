import unittest
from overtaking_trial_policy import PassingPolicy, atomic_response_success, candidate_service_name, require_candidate_service_type

def snapshot():
    return dict(valid=True,now=0,busy=False,remaining=500,lane=1,left_allowed=False,right_allowed=True,
        desired_speed=30,ego_speed=20,lead=dict(uuid='lead',gap=35,speed=16),
        tracked_lead_present=True,fully_passed=False,intent_ack=False,
        intent_route=('route',1.),intent_boot='boot',paired_chains={'right':[[1,2],[3,4]],'left':None},
        rtc={'left':[],'right':[dict(uuid='R',state=0,safe=True)]})

class Tests(unittest.TestCase):
    def test_dedicated_endpoint_only_and_exact_type(self):
        for direction in ('left','right'):
            endpoint=candidate_service_name(direction)
            self.assertEqual(endpoint,'/planning/enable_auto_mode_candidate/external_request_lane_change_'+direction)
            require_candidate_service_type(endpoint,'tier4_rtc_msgs/CooperateCommands')
            for wrong in (None,'tier4_rtc_msgs/AutoMode',''):
                with self.assertRaises(RuntimeError):require_candidate_service_type(endpoint,wrong)
        with self.assertRaises(RuntimeError):
            require_candidate_service_type('/planning/cooperate_commands/external_request_lane_change_left','tier4_rtc_msgs/CooperateCommands')
        with self.assertRaises(ValueError):candidate_service_name('straight')
    def test_atomic_response_must_bind_exact_uuid_and_module(self):
        self.assertTrue(atomic_response_success('X',15,[dict(uuid='X',module=15,success=True)]))
        for response in ([],[dict(uuid='Y',module=15,success=True)],
                         [dict(uuid='X',module=16,success=True)],[dict(uuid='X',module=15,success=False)]):
            self.assertFalse(atomic_response_success('X',15,response))
    def arm(self):
        p=PassingPolicy();s=snapshot();p.step(s);s['now']=2
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'ARM');return p,s
    def test_right_pass_requires_ack_before_service(self):
        p,s=self.arm();self.assertEqual(p.direction,'right')
        self.assertIsNone(p.step(s));s['intent_ack']=True
        self.assertEqual(p.step(s),'right')
    def test_left_safe_candidate_also_supported(self):
        p=PassingPolicy();s=snapshot();s['left_allowed']=True
        s['paired_chains']['left']=[[1,2],[5,6]]
        s['rtc']['left']=[dict(uuid='L',state=0,safe=True)]
        p.step(s);s['now']=2;p.step(s)
        self.assertEqual(p.direction,'left')
    def test_pass_finished_releases_normal_return_never_rpc(self):
        p,s=self.arm();s['rtc']['right'][0]['state']=1;p.step(s)
        self.assertEqual(p.state,'OUTBOUND');s['rtc']['right'][0]['state']=3;p.step(s)
        self.assertEqual(p.state,'PASSING');s['lane']=3
        self.assertIsNotNone(p.intent(s,10,'hash'))
        s['fully_passed']=True;self.assertIsNone(p.step(s))
        self.assertEqual(p.state,'RETURN_RELEASED');self.assertIsNone(p.intent(s,10,'hash'))
        s['lane']=1;p.step(s);self.assertEqual(p.state,'DONE')
    def test_mandatory_reserve_and_lost_lead_release_hold(self):
        for update in ({'remaining':140},{'tracked_lead_present':False},{'ack_reserve':600}):
            p,s=self.arm();p.state='PASSING';s.update(update)
            self.assertIsNone(p.step(s));self.assertEqual(p.state,'RETURN_RELEASED')
    def test_stale_inputs_stop_intent_and_rpc(self):
        p,s=self.arm();s['valid']=False
        self.assertIsNone(p.step(s));self.assertIsNone(p.intent(s,10,'hash'))
    def test_candidate_change_or_competing_owner_holds(self):
        for update in ('candidate','busy'):
            p,s=self.arm();s['intent_ack']=True
            if update=='candidate':s['rtc']['right'][0]['uuid']='different'
            else:s['busy']=True
            self.assertIsNone(p.step(s));self.assertEqual(p.state,'HOLD')
    def test_missing_completion_never_infers_success(self):
        p,s=self.arm();p.state='OUTBOUND';s['rtc']['right']=[]
        p.step(s);s['now']=5;p.step(s)
        self.assertEqual(p.state,'HOLD')
    def test_no_manual_return_request_even_if_rtc_safe(self):
        p,s=self.arm();p.state='RETURN_RELEASED';s['lane']=3;s['intent_ack']=True
        self.assertIsNone(p.step(s));self.assertEqual(p.state,'RETURN_RELEASED')

if __name__=='__main__':unittest.main()
