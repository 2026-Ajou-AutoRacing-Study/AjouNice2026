"""Pure fake-message checks: no ROS, master, GPU, or vehicle required."""

import importlib.util
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as N
import unittest


SPEC = importlib.util.spec_from_file_location('field_topic_health',
    Path(__file__).with_name('field_topic_health.py'))
health_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = health_module
SPEC.loader.exec_module(health_module)
TopicHealth = health_module.TopicHealth


def header(stamp):
    return N(stamp=stamp)


def signal_packet(group=201326, lamps=None, source=100., status='matched'):
    return N(data=json.dumps(dict(source_stamp_ns=int(source * 1e9), frame_decision=dict(
        group_id=group, selection_status=status,
        final_tuples=lamps if lamps is not None else [['red', 'circle', None], ['green', 'arrow', True]],
        observation_distance_gate={}))))


class TopicHealthTests(unittest.TestCase):
    def health(self, component):
        return TopicHealth(component, clock=lambda: 0.)

    def route(self, health, group=201326):
        health.observe('route', N(data=group), 0.)
        health.observe('route', N(data=group), .01)

    def test_import_does_not_require_or_import_ros(self):
        self.assertNotIn('rospy', health_module.__dict__)

    def test_missing_wait_grace_then_missing_is_error(self):
        health = self.health('bevfusion')
        self.assertEqual(health.snapshot(2., 100.), ('waiting output', 'WARN'))
        self.assertEqual(health.snapshot(6., 100.), ('waiting output', 'ERROR'))

    def test_od_counts_use_strongest_class_empty_is_fresh(self):
        health = self.health('bevfusion')
        objects = [N(classification=[N(label=0, probability=.1), N(label=1, probability=.9)]),
                   N(classification=[N(label=7, probability=.8)]), N(classification=[])]
        for index in range(11):
            health.observe('od', N(header=header(100 + index / 10), objects=objects), index / 10)
        detail, level = health.snapshot(1., 101.02)
        self.assertIn('V1 P1 O1 10.0Hz 20ms', detail)
        self.assertEqual(level, 'INFO')
        health.observe('od', N(header=header(101.1), objects=[]), 1.1)
        self.assertIn('V0 P0 O0', health.snapshot(1.1, 101.1)[0])

    def test_recent_receipt_with_old_source_is_stale(self):
        health = self.health('tracking')
        health.observe('trk', N(header=header(90), objects=[]), 10.)
        self.assertEqual(health.snapshot(10., 100.), ('output stale 10.0s', 'ERROR'))

    def test_ros_clock_reset_does_not_create_false_staleness(self):
        health = self.health('tracking')
        health.observe('trk', N(header=header(100), objects=[]), 10.)
        self.assertEqual(health.snapshot(10., 90.), ('source clock differs', 'WARN'))

    def test_tl_latched_startup_metadata_not_fresh_frame(self):
        health = self.health('traffic')
        health.observe('tl', N(data=json.dumps(dict(source_stamp_ns=None, frame_decision=None))), 1.)
        self.assertEqual(health.snapshot(6., 100.), ('waiting output', 'ERROR'))

    def test_tl_mission_lamps_and_invalid_json(self):
        health = self.health('traffic')
        self.route(health)
        health.observe('tl', signal_packet(), 0.)
        health.observe('tl', signal_packet(source=100.1), .1)
        self.assertEqual(health.snapshot(.1, 100.27), ('D RC+GLA 10.0Hz 170ms', 'INFO'))
        health.observe('tl', N(data='{broken'), .2)
        self.assertEqual(health.parse_errors, 1)
        self.assertEqual(health.snapshot(2., 102.)[1], 'ERROR')

    def test_tl_selection_missing_does_not_claim_current_lamp(self):
        health = self.health('traffic')
        self.route(health)
        health.observe('selection', N(data='{"reason":"route_missing_or_stale"}'), 0.)
        health.observe('tl', signal_packet(lamps=[], status='missing'), 0.)
        self.assertIn('D route-none', health.snapshot(0., 100.)[0])

    def test_tl_no_upcoming_mission_is_not_detector_failure(self):
        health = self.health('traffic')
        self.route(health, 0)
        health.observe('selection', N(data='{"reason":"route_missing_or_stale"}'), 0.)
        health.observe('tl', signal_packet(group=0, lamps=[], status='missing'), 0.)
        detail, level = health.snapshot(.1, 100.1)
        self.assertIn('no mission', detail)
        self.assertEqual(level, 'INFO')

    def test_tl_arrow_boolean_is_direction_not_confidence(self):
        self.assertEqual(health_module._lamps([['green', 'arrow', True]]), 'GLA')
        self.assertEqual(health_module._lamps([['green', 'arrow', False]]), 'GRA')
        self.assertEqual(health_module._lamps([['green', 'arrow', .9]]), 'GA?')

    def test_actual_bag_far_gate_does_not_display_ignored_yellow(self):
        health = self.health('traffic')
        self.route(health, 201351)
        packet = dict(source_stamp_ns=100000000000, frame_decision=dict(
            group_id=201351, head_id='A_head', selection_status='matched',
            observed_tuples=[['yellow', 'circle', None]], final_tuples=[],
            observation_distance_gate=dict(enabled=True, suppressed=True, activated=False,
                just_opened=False, threshold_m=40., current_matched_camera_depth_m=65.347,
                reason='awaiting_near_current_match')))
        health.observe('tl', N(data=json.dumps(packet)), 0.)
        packet['source_stamp_ns'] += 100000000
        health.observe('tl', N(data=json.dumps(packet)), .1)
        self.assertEqual(health.snapshot(.1, 100.17), ('A far 10.0Hz 70ms', 'INFO'))

    def test_actual_bag_active_d_gives_green_circle_not_left_arrow(self):
        health = self.health('traffic')
        self.route(health)
        packet = dict(source_stamp_ns=100000000000, frame_decision=dict(
            group_id=201326, selection_status='matched',
            observed_tuples=[['green', 'circle', None]],
            final_tuples=[['green', 'circle', None]],
            observation_distance_gate=dict(enabled=True, suppressed=False, activated=True,
                just_opened=True, threshold_m=30., current_matched_camera_depth_m=28.737,
                reason='active_latched')))
        health.observe('tl', N(data=json.dumps(packet)), 0.)
        packet['source_stamp_ns'] += 100000000
        health.observe('tl', N(data=json.dumps(packet)), .1)
        self.assertEqual(health.snapshot(.1, 100.13), ('D GC 10.0Hz 30ms', 'INFO'))

    def test_no_mission_requires_live_independent_route_selector(self):
        health = self.health('traffic')
        health.observe('tl', signal_packet(group=0, lamps=[], status='missing'), 0.)
        self.assertEqual(health.snapshot(.1, 100.1), ('waiting route selector', 'WARN'))
        health.observe('route', N(data=0), 0.)
        self.assertEqual(health.snapshot(.1, 100.1), ('waiting route stream', 'WARN'))
        health.observe('route', N(data=0), .1)
        self.assertIn('no mission', health.snapshot(.1, 100.1)[0])
        health.observe('tl', signal_packet(group=0, lamps=[], status='missing', source=101.2), 1.2)
        self.assertEqual(health.snapshot(1.2, 101.2), ('route selector stale 1.1s', 'ERROR'))

    def test_selector_camera_handoff_is_visible_not_false_no_mission(self):
        health = self.health('traffic')
        self.route(health)
        health.observe('tl', signal_packet(group=0, lamps=[], status='missing'), 0.)
        self.assertEqual(health.snapshot(.1, 100.1), ('mission updating', 'WARN'))

    def test_static_latched_tf_zero_stamps_do_not_expire(self):
        health = self.health('sensing')
        transforms = [N(header=N(frame_id='base_link', stamp=0), child_frame_id=child)
                      for child in health_module.STATIC_CHILDREN]
        health.observe('tf', N(transforms=transforms), 0.)
        self.assertEqual(health.snapshot(3600., 100.), ('static 7/7', 'INFO'))

    def test_localization_shadow_is_valid_input_not_missing_gnss(self):
        health = self.health('localization')
        for index in range(51):
            receipt, source = index / 50., 100 + index / 50.
            for key in ('global', 'smooth', 'comp'):
                health.observe(key, N(header=header(source)), receipt)
            health.observe('gps', N(header=header(source), latitude=0, longitude=0), receipt)
        self.assertEqual(health.snapshot(1., 101.1), ('50Hz GPS shadow', 'WARN'))
        self.assertIn('stale', health.snapshot(1.7, 101.7)[0])

    def test_udp_lidar_anymsg_reads_header_only(self):
        health = self.health('udp')
        for index in range(6):
            stamp = 100 + index / 5.
            health.observe('comp', N(header=header(stamp)), index / 5.)
            health.observe('gps', N(header=header(stamp), latitude=37, longitude=127), index / 5.)
            health.observe('imu', N(header=header(stamp)), index / 5.)
            health.observe('lidar', N(_buff=struct.pack('<III', 1, int(stamp), int((stamp % 1) * 1e9))), index / 5.)
        detail, level = health.snapshot(1., 101.)
        self.assertEqual(detail, 'C5 G5 I5 L5.0')
        self.assertEqual(level, 'WARN')

    def test_udp_never_seen_lidar_is_visible_and_low_inputs_caution(self):
        health = self.health('udp')
        for index in range(7):
            stamp = 100 + index
            health.observe('comp', N(header=header(stamp)), index)
            health.observe('gps', N(header=header(stamp), latitude=37, longitude=127), index)
            health.observe('imu', N(header=header(stamp)), index)
        detail, level = health.snapshot(6., 106.)
        self.assertEqual(detail, 'C1 G1 I1 L-wait')
        self.assertEqual(level, 'WARN')

    def test_udp_shadow_never_downgrades_stale_lidar_error(self):
        health = self.health('udp')
        health.observe('lidar', N(_buff=struct.pack('<III', 1, 100, 0)), 0.)
        for key in ('comp', 'imu'):
            health.observe(key, N(header=header(102.)), 2.)
        health.observe('gps', N(header=header(102.), latitude=0, longitude=0), 2.)
        detail, level = health.snapshot(2., 102.)
        self.assertIn('stale', detail)
        self.assertIn('shadow', detail)
        self.assertEqual(level, 'ERROR')

    def test_localization_low_rate_is_caution_not_dead(self):
        health = self.health('localization')
        for index in range(3):
            receipt, source = index / 5., 100 + index / 5.
            for key in ('global', 'smooth', 'comp'):
                health.observe(key, N(header=header(source)), receipt)
            health.observe('gps', N(header=header(source), latitude=37, longitude=127), receipt)
        self.assertEqual(health.snapshot(.4, 100.4), ('5Hz GPS valid', 'WARN'))

    def test_bounded_samples_close_only_unregisters_owned_observers(self):
        health = self.health('tracking')
        for index in range(1000):
            health.observe('trk', N(header=header(100 + index / 50.), objects=[]), index / 50.)
        self.assertLessEqual(len(health.samples['trk'].times), 256)
        calls = []
        health._subscriptions.append(N(unregister=lambda: calls.append('unregister')))
        health.close()
        health.close()
        self.assertEqual(calls, ['unregister'])
        self.assertIsNone(health.snapshot(20., 120.))

    def path(self, stamp, count=100):
        frame = b'map'
        secs = int(stamp)
        return N(_buff=struct.pack('<IIII', 1, secs, round((stamp-secs)*1e9), len(frame))
                 + frame + struct.pack('<I', count))

    def test_planviz_waiting_inputs_and_map_is_latched(self):
        health = self.health('planviz')
        self.assertEqual(health.snapshot(1., 100.), ('waiting pose', 'WARN'))
        self.assertEqual(health.snapshot(6., 100.), ('waiting pose', 'ERROR'))
        health.observe('global', N(header=header(100.)), 0.)
        self.assertIn('map wait', health.snapshot(.1, 100.1)[0])
        health.observe('viz_map', N(_buff=struct.pack('<I', 3000)), 0.)
        self.assertIn('path wait', health.snapshot(.1, 100.1)[0])
        for index in range(11):
            health.observe('global', N(header=header(200+index/10)), 100+index/10)
            health.observe('viz_path', self.path(200+index/10), 100+index/10)
        self.assertEqual(health.snapshot(101.,201.05), ('map ready pose 10Hz path 10.0Hz', 'INFO'))

    def test_planviz_path_empty_stale_and_clock_mismatch_are_distinct(self):
        health = self.health('planviz')
        health.observe('viz_map', N(_buff=struct.pack('<I', 3)), 0.)
        health.observe('global', N(header=header(100.)), 0.)
        health.observe('viz_path', self.path(100.,0), 0.)
        self.assertEqual(health.snapshot(0.,100.), ('map ready path empty','WARN'))
        health.observe('global', N(header=header(102.)), 2.)
        self.assertEqual(health.snapshot(2.,102.), ('path stale 2.0s','ERROR'))
        health.observe('viz_path', self.path(110.),2.)
        self.assertEqual(health.snapshot(2.,102.), ('path clock differs','WARN'))

    def test_planviz_malformed_header_does_not_escape_or_create_fresh_status(self):
        health = self.health('planviz')
        for key, buffer in [('viz_map',b''),('viz_path',b'broken'),
                            ('viz_path',struct.pack('<IIII',1,100,0,9999999))]:
            health.observe(key, N(_buff=buffer),0.)
        self.assertEqual(health.parse_errors,3)
        self.assertNotIn('viz_map',health.samples)
        self.assertNotIn('viz_path',health.samples)

    def test_planviz_reads_path_count_without_deserializing_point_array(self):
        health = self.health('planviz')
        health.observe('viz_path', self.path(100.,1000000),0.)
        self.assertTrue(health.samples['viz_path'].valid)
        self.assertEqual(health.samples['viz_path'].detail,'1000000')


if __name__ == '__main__':
    unittest.main()
