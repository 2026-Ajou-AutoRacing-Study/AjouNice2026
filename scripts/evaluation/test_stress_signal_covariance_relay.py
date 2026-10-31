"""Pure tests: no ROS, simulator, signal publisher or vehicle controls."""
import copy
import math
from types import SimpleNamespace as NS
import unittest

import stress_signal_covariance_relay as stress


def odometry(now=10., speed=0.):
    ns = int(round(now * 1e9))
    return NS(header=NS(stamp=NS(secs=ns//10**9, nsecs=ns%10**9), seq=123, frame_id='map'),
              child_frame_id='base_link',
              pose=NS(pose=NS(position=NS(x=7., y=8., z=9.),
                              orientation=NS(x=.1, y=.2, z=.3, w=.9)),
                      covariance=tuple(i/1000. for i in range(36))),
              twist=NS(twist=NS(linear=NS(x=speed, y=0., z=0.), angular=NS(x=.1, y=.2, z=.3)),
                       covariance=tuple(i/100. for i in range(36))))


def observation(color, now=10., group=201351):
    ns = int(round(now * 1e9))
    return dict(source_stamp_ns=ns, frame_decision=dict(group_id=group,
        selection_status='matched', classifier_known=True,
        observed_tuples=[[color, 'circle', None]], final_tuples=[[color, 'circle', None]],
        observation_timing=dict(output_is_observed=True,
            adopted_observation_source_stamp_ns=ns, output_source_stamp_ns=ns)))


def call(relay, now, speed=0.):
    return relay.relay(odometry(now, speed), now, int(round(now*1e9)))


def arm(relay):
    call(relay, 10., 0.)
    relay.observe_debug(observation('red'), 10., 10*10**9)


class CovarianceStressTest(unittest.TestCase):
    def test_deepcopy_preserves_everything_except_pose_covariance_35(self):
        original = odometry()
        snapshot = copy.deepcopy(original)
        result = stress.covariance_copy(original, math.radians(3.2)**2)
        self.assertIsNot(result, original)
        self.assertIsNot(result.pose, original.pose)
        self.assertEqual(result.header, original.header)
        self.assertEqual(result.child_frame_id, original.child_frame_id)
        self.assertEqual(result.pose.pose, original.pose.pose)
        self.assertEqual(result.twist, original.twist)
        self.assertEqual(list(result.pose.covariance[:35]), list(original.pose.covariance[:35]))
        self.assertEqual(result.pose.covariance[35], math.radians(3.2)**2)
        self.assertEqual(original, snapshot)
        self.assertEqual(stress.covariance_copy(original), original)

    def test_one_shot_after_actual_red_stop_then_current_green_departure(self):
        events = []
        relay = stress.StressRelay(10., emit=lambda event, **kw: events.append(dict(event=event, **kw)))
        arm(relay)
        relay.observe_debug(observation('green', 10.1), 10.1, 10100000000)
        self.assertEqual(call(relay, 10.1, .299).pose.covariance, odometry().pose.covariance)
        result = call(relay, 10.2, .3)
        self.assertEqual(result.pose.covariance[35], math.radians(3.2)**2)
        self.assertEqual(call(relay, 10.399, 1.).pose.covariance[35], relay.yaw_variance)
        self.assertEqual(call(relay, 10.4, 1.).pose.covariance, odometry().pose.covariance)
        relay.observe_debug(observation('red', 11.), 11., 11*10**9)
        call(relay, 11., 0.)
        relay.observe_debug(observation('green', 11.1), 11.1, 11100000000)
        self.assertEqual(call(relay, 11.1, 1.).pose.covariance, odometry().pose.covariance)
        self.assertEqual(relay.mutated_count, 2)
        triggers = [e for e in events if e['event'] == 'stress_trigger']
        self.assertEqual(len(triggers), 1)
        self.assertEqual(triggers[0]['trigger_stamp_ns'], 10200000000)
        self.assertEqual(triggers[0]['original_yaw_covariance'], .035)

    def test_no_red_no_stop_wrong_group_and_speed_outside_window_do_not_trigger(self):
        for scenario in ('no_red', 'moving_red', 'wrong_group', 'too_slow', 'too_fast'):
            with self.subTest(scenario=scenario):
                relay = stress.StressRelay(10.)
                call(relay, 10., .1 if scenario == 'moving_red' else 0.)
                if scenario != 'no_red':
                    relay.observe_debug(observation('red', group=201365 if scenario == 'wrong_group' else 201351),
                                        10., 10*10**9)
                relay.observe_debug(observation('green', 10.1), 10.1, 10100000000)
                speed = .299 if scenario == 'too_slow' else 2.001 if scenario == 'too_fast' else 1.
                self.assertEqual(call(relay, 10.1, speed), odometry(10.1, speed))
                self.assertIsNone(relay.trigger_monotonic)

    def test_held_synthetic_stale_future_unknown_or_malformed_green_is_not_permission(self):
        for scenario in ('held', 'synthetic', 'stale', 'future', 'unknown', 'wrong_final', 'malformed'):
            with self.subTest(scenario=scenario):
                relay = stress.StressRelay(10.)
                arm(relay)
                report = observation('green', 10.1)
                frame = report['frame_decision']
                if scenario == 'held': frame['observation_timing']['adopted_observation_source_stamp_ns'] -= 1
                if scenario == 'synthetic': frame['observation_timing']['output_is_observed'] = False
                if scenario == 'stale': report['source_stamp_ns'] = 9*10**9
                if scenario == 'future': report['source_stamp_ns'] = 11*10**9
                if scenario == 'unknown': frame['classifier_known'] = False
                if scenario == 'wrong_final': frame['final_tuples'] = [['red', 'circle', None]]
                if scenario == 'malformed': report = None
                relay.observe_debug(report, 10.1, 10100000000)
                self.assertEqual(call(relay, 10.1, 1.), odometry(10.1, 1.))
                self.assertIsNone(relay.trigger_monotonic)

    def test_stale_receipt_or_odometry_and_timeout_cannot_manufacture_trigger(self):
        relay = stress.StressRelay(10., max_seconds=1.)
        arm(relay)
        relay.observe_debug(observation('green', 10.1), 10.1, 10100000000)
        self.assertEqual(call(relay, 10.7, 1.), odometry(10.7, 1.))
        relay.observe_debug(observation('green', 10.8), 10.8, 10800000000)
        self.assertEqual(relay.relay(odometry(9., 1.), 10.8, 10800000000), odometry(9., 1.))
        self.assertIsNone(call(relay, 11., 1.))
        self.assertIsNone(relay.trigger_monotonic)
        self.assertEqual(relay.mutated_count, 0)

    def test_no_inputs_means_no_heartbeat_no_mutation_and_idempotent_shutdown(self):
        events = []
        relay = stress.StressRelay(10., max_seconds=1.,
            emit=lambda event, **kw: events.append(dict(event=event, **kw)))
        relay.observe_debug(observation('red'), 10., 10*10**9)
        relay.observe_debug(observation('green', 10.1), 10.1, 10100000000)
        self.assertFalse(relay.red_armed)
        self.assertEqual(relay.relayed_count, 0)
        self.assertEqual(relay.mutated_count, 0)
        self.assertIsNone(relay.odom)
        self.assertTrue(relay.expired(11.))
        relay.stop('max_seconds')
        relay.stop('owner_shutdown')
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['mutated_count'], 0)
        self.assertFalse(events[0]['triggered'])
        self.assertIsNone(call(relay, 11., 1.))

    def test_stale_stop_cannot_arm_and_new_group_disarms_untriggered_gate(self):
        relay = stress.StressRelay(10.)
        call(relay, 10., 0.)
        relay.observe_debug(observation('red', 10.6), 10.6, 10600000000)
        self.assertFalse(relay.red_armed)
        call(relay, 10.6, 0.)
        self.assertTrue(relay.red_armed)
        relay.observe_debug(observation('red', 10.7, group=201365), 10.7, 10700000000)
        self.assertFalse(relay.red_armed)
        relay.observe_debug(observation('green', 10.8), 10.8, 10800000000)
        self.assertEqual(call(relay, 10.8, 2.), odometry(10.8, 2.))

    def test_bounds_and_fixed_evaluation_destination(self):
        for kwargs in (dict(duration_sec=.099), dict(duration_sec=.301),
                       dict(yaw_stddev_deg=.99), dict(yaw_stddev_deg=5.01),
                       dict(max_seconds=180.001), dict(max_seconds=0.),
                       dict(duration_sec=math.nan), dict(yaw_stddev_deg=True)):
            with self.assertRaises(ValueError): stress.StressRelay(10., **kwargs)
        self.assertEqual(stress.OUTPUT_TOPIC, '/evaluation/traffic_light_policy/covariance_odometry')
        self.assertNotEqual(stress.OUTPUT_TOPIC, stress.INPUT_TOPIC)
        for duration in (.1, .3):
            stress.StressRelay(10., duration_sec=duration, yaw_stddev_deg=5.)


if __name__ == '__main__':
    unittest.main()
