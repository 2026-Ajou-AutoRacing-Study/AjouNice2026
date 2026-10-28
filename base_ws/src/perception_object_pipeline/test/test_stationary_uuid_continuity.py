#!/usr/bin/env python3
"""Causal identity-only continuity, including explicit ambiguous-newborn limit."""
import copy
import io
import math
import unittest

import rospy
from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects

from perception_object_pipeline.stationary_uuid_continuity import (
    ContinuityConfig, StationaryUuidContinuity, _observe, _footprint_iou)


def identity(number):
    return b'CONTTEST' + int(number).to_bytes(8, 'big')


def actor(number, x=0., y=0., z=1., speed=0., label=0, size=(1.2, 1.2, 1.2), yaw=0.):
    obj = TrackedObject()
    obj.object_id.uuid = list(identity(number))
    obj.classification = [ObjectClassification(label=label, probability=.8)]
    obj.existence_probability = .73
    pose = obj.kinematics.pose_with_covariance.pose
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
    obj.shape.type = 0
    obj.shape.dimensions.x, obj.shape.dimensions.y, obj.shape.dimensions.z = size
    obj.kinematics.twist_with_covariance.twist.linear.x = speed
    obj.kinematics.pose_with_covariance.covariance[0] = .27
    obj.kinematics.twist_with_covariance.covariance[7] = .38
    obj.kinematics.acceleration_with_covariance.accel.linear.y = .19
    return obj


def message(stamp, *objects, frame='map'):
    msg = TrackedObjects()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.header.frame_id = frame
    msg.objects = list(objects)
    return msg


def wire(msg):
    output = io.BytesIO()
    msg.serialize(output)
    return output.getvalue()


class StationaryUuidContinuityTest(unittest.TestCase):
    def setUp(self):
        self.helper = StationaryUuidContinuity(enabled=True)

    def support(self, *objects, start=1.):
        if not objects:
            objects = (actor(1),)
        for index in range(9):
            self.helper.apply(message(start + index * .1, *objects))

    def retired(self, *objects):
        self.support(*objects)
        result = self.helper.apply(message(1.9))
        self.assertEqual(result.objects, [])

    def test_default_off_has_exact_input_identity_and_no_history(self):
        helper = StationaryUuidContinuity()
        msg = message(1., actor(1))
        self.assertIs(helper.apply(msg), msg)
        self.assertEqual(helper.active, {})

    def test_newborn_inherits_unique_absent_supported_uuid(self):
        self.retired()
        source = message(2.2, actor(2, x=.1, yaw=.5))
        before = wire(source)
        output = self.helper.apply(source)
        self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))
        self.assertEqual(wire(source), before)
        restored = copy.deepcopy(output)
        restored.objects[0].object_id.uuid = list(identity(2))
        self.assertEqual(wire(restored), before)
        self.assertEqual(self.helper.last_events[0]['kind'], 'reuse')

    def test_logical_uuid_persists_through_class_and_position_changes(self):
        self.retired()
        self.helper.apply(message(2.2, actor(2, x=.1)))
        for stamp, x, label in ((2.3, .2, 1), (2.4, 1., 3), (2.5, 4., 0)):
            output = self.helper.apply(message(stamp, actor(2, x=x, label=label)))
            self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))
            self.assertEqual(output.objects[0].kinematics.pose_with_covariance.pose.position.x, x)

    def test_never_merges_current_concurrent_ids_or_deletes_objects(self):
        self.support()
        source = message(1.9, actor(1), actor(2, x=.1))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(len(self.helper.active), 2)

    def test_existing_track_cannot_alias_later_after_old_disappears(self):
        self.support()
        self.helper.apply(message(1.9, actor(1), actor(2, x=.1, speed=1.)))
        source = message(2., actor(2, x=.2, speed=1.))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_stationary_requirement_uses_raw_velocity_before_zero_policy(self):
        self.retired(actor(1, speed=2.))
        source = message(2.2, actor(2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_raw_position_motion_rejects_even_if_reported_speed_zero(self):
        for index in range(9):
            self.helper.apply(message(1. + index * .1, actor(1, x=index * .15)))
        self.helper.apply(message(1.9))
        source = message(2.2, actor(2, x=1.2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_prior_known_class_history_is_not_a_stationary_unknown(self):
        self.helper.apply(message(.9, actor(1, label=1)))
        self.retired()
        source = message(2.2, actor(2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_repeated_timestamp_is_not_support(self):
        for _ in range(40):
            self.helper.apply(message(1., actor(1)))
        self.helper.apply(message(1.9))
        source = message(2.2, actor(2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_insufficient_time_support_rejects(self):
        for index in range(5):
            self.helper.apply(message(1. + index * .05, actor(1)))
        self.helper.apply(message(1.3))
        source = message(1.4, actor(2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_expired_gap_never_extends_ttl_or_resurrects_output(self):
        self.retired()
        self.assertEqual(self.helper.apply(message(2.3)).objects, [])
        source = message(4., actor(2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.retired, {})

    def test_geometry_center_height_size_and_overlap_gates(self):
        cases = (actor(2, x=.51), actor(2, z=1.61),
                 actor(2, size=(2.4, 2.4, 2.4)))
        for newborn in cases:
            with self.subTest(newborn=newborn):
                self.setUp()
                self.retired()
                source = message(2.2, newborn)
                self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.setUp()
        self.retired(actor(1, size=(1.2, .2, 1.2)))
        source = message(2.2, actor(2, y=.3, size=(1.2, .2, 1.2)))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_nearby_current_object_blocks_even_when_only_one_matches_geometry(self):
        self.retired()
        source = message(2.2, actor(2), actor(3, x=.9, label=1))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_rejections[0]['reason'], 'nearby_current_object')

    def test_two_retired_neighbours_are_ambiguous(self):
        # They were not simultaneously visible (otherwise the lifetime
        # crowding latch already makes them ineligible sources).
        self.retired(actor(1))
        for index in range(9):
            self.helper.apply(message(2. + index * .1, actor(2, x=.6)))
        self.helper.apply(message(2.9))
        source = message(3., actor(3, x=.2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_rejections[0]['reason'], 'ambiguous_retired_objects')

    def test_crowding_history_survives_neighbour_retired_pool_expiry(self):
        # Deliberately retain the old loose geometry gates to isolate this
        # lifetime-history protection from the new default tighter geometry.
        self.helper = StationaryUuidContinuity(True, ContinuityConfig(
            maximum_center_distance_m=.5, minimum_footprint_iou=.2))
        for index in range(9):
            self.helper.apply(message(1. + index * .1, actor(1, x=.1, size=(.6, .6, .4)),
                                      actor(2, x=.5, size=(.6, .6, .4))))
        self.assertTrue(self.helper.active[identity(1)].nearby_other_uuid_seen)
        for index in range(30):
            self.helper.apply(message(1.9 + index * .1, actor(1, x=.1, size=(.6, .6, .4))))
        self.assertNotIn(identity(2), self.helper.retired)
        self.helper.apply(message(4.9))
        source = message(5.3, actor(3, x=.5, size=(.6, .6, .4)))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_distant_other_uuid_does_not_disable_unique_continuity(self):
        self.support(actor(1), actor(9, x=2.))
        self.assertFalse(self.helper.active[identity(1)].nearby_other_uuid_seen)
        self.helper.apply(message(1.9, actor(9, x=2.)))
        source = message(2.2, actor(2, x=.1), actor(9, x=2.))
        output = self.helper.apply(source)
        self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))
        self.assertEqual(len(output.objects), 2)

    def test_tighter_overlap_gate_rejects_biased_neighbour_counterexample(self):
        self.retired(actor(1, x=.1, size=(.6, .6, .4)))
        source = message(2.2, actor(2, x=.5, size=(.6, .6, .4)))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.config.maximum_center_distance_m, .35)
        self.assertEqual(self.helper.config.minimum_footprint_iou, .35)

    def test_crowding_flag_is_per_track_lifetime_not_per_raw_uuid_forever(self):
        self.support(actor(1), actor(2, x=.6))
        self.helper.apply(message(1.9))
        self.support(actor(1, x=20.), start=2.)
        self.assertFalse(self.helper.active[identity(1)].nearby_other_uuid_seen)
        self.helper.apply(message(2.9))
        output = self.helper.apply(message(3., actor(3, x=20.1)))
        self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))

    def test_invalid_nearby_shape_class_or_speed_still_latches_crowding(self):
        neighbours = [actor(2, x=.8, size=(float('nan'), 1.2, 1.2)),
                      actor(2, x=.8, speed=float('nan')), actor(2, x=.8)]
        neighbours[-1].classification = []
        for neighbour in neighbours:
            with self.subTest(neighbour=neighbour):
                self.setUp()
                self.support(actor(1))
                self.helper.apply(message(1.9, actor(1), neighbour))
                self.assertTrue(self.helper.active[identity(1)].nearby_other_uuid_seen)
                self.helper.apply(message(2., actor(1)))
                self.helper.apply(message(2.1))
                source = message(2.2, actor(3))
                self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_unlocated_other_raw_id_conservatively_disqualifies_source_history(self):
        self.support(actor(1))
        self.helper.apply(message(1.9, actor(1), actor(2, x=float('nan'))))
        self.assertTrue(self.helper.active[identity(1)].nearby_other_uuid_seen)
        self.helper.apply(message(2.))
        source = message(2.2, actor(3))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_invalid_but_located_far_neighbour_does_not_latch_nearby_history(self):
        self.support(actor(1))
        self.helper.apply(message(1.9, actor(1), actor(2, x=20., speed=float('nan'))))
        self.assertFalse(self.helper.active[identity(1)].nearby_other_uuid_seen)

    def test_two_newborns_cannot_reuse_one_old_uuid(self):
        self.retired()
        source = message(2.2, actor(2, x=-.1), actor(3, x=.1))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_slot_reuse_and_outgoing_collision_fail_open(self):
        self.retired()
        self.helper.apply(message(2.2, actor(2)))
        source = message(2.3, actor(1), actor(2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_events[-1]['reason'], 'outgoing_uuid_collision')
        self.assertEqual(self.helper.active, {})

    def test_raw_uuid_later_reused_far_away_does_not_inherit_mapping(self):
        self.retired()
        self.helper.apply(message(2.2, actor(2)))
        self.helper.apply(message(2.3))
        source = message(2.4, actor(2, x=20.))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_backward_time_invalidates_mapping(self):
        self.retired()
        self.helper.apply(message(2.2, actor(2)))
        source = message(.5, actor(2))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_events[0]['reason'], 'backward_time')

    def test_local_coordinates_or_invalid_geometry_do_not_reuse(self):
        self.retired()
        source = message(2.2, actor(2), frame='base_link')
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.active, {})
        self.retired()
        source = message(2.2, actor(2, x=float('nan')))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_mapping_does_not_disappear_on_same_timestamp_republication(self):
        self.retired()
        source = message(2.2, actor(2))
        first = self.helper.apply(source)
        self.assertEqual(wire(self.helper.apply(source)), wire(first))

    def test_changed_same_stamp_class_cannot_bypass_known_class_history(self):
        self.retired()
        self.helper.apply(message(2.2, actor(2)))
        for index in range(9):
            self.helper.apply(message(2.3 + index * .1, actor(2)))
        source = message(2.3 + 8 * .1, actor(2, label=1))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_events[0]['reason'], 'changed_payload_same_stamp')
        self.assertTrue(self.helper.active[identity(2)].known_class_seen)
        self.helper.apply(message(3.2))
        source = message(3.3, actor(3))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_changed_same_stamp_geometry_discards_mapping_and_old_support(self):
        for replacement in (actor(2, x=50.), actor(2, size=(float('nan'), 1.2, 1.2))):
            with self.subTest(replacement=replacement):
                self.setUp()
                self.retired()
                self.helper.apply(message(2.2, actor(2)))
                source = message(2.2, replacement)
                self.assertEqual(wire(self.helper.apply(source)), wire(source))
                self.assertEqual(self.helper.last_events[0]['reason'], 'changed_payload_same_stamp')
                self.assertEqual(self.helper.retired, {})

    def test_publisher_sequence_alone_is_not_changed_payload_or_new_support(self):
        self.retired()
        source = message(2.2, actor(2))
        first = self.helper.apply(source)
        source.header.seq = 123
        second = self.helper.apply(source)
        self.assertEqual(bytes(second.objects[0].object_id.uuid), bytes(first.objects[0].object_id.uuid))
        self.assertEqual(len(self.helper.active[identity(2)].samples), 1)
        self.assertEqual(self.helper.last_events, [])

    def test_monotonic_sequence_does_not_change_continuity(self):
        self.retired()
        source = message(2.2, actor(2))
        source.header.seq = 20
        self.helper.apply(source)
        for sequence in (21, 22, 100):
            source = message(2.2 + sequence * .01, actor(2))
            source.header.seq = sequence
            output = self.helper.apply(source)
            self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))
            self.assertEqual(self.helper.last_events, [])

    def test_sequence_regression_discards_old_mapping_even_with_forward_clock(self):
        self.retired()
        source = message(2.2, actor(2))
        source.header.seq = 20
        self.helper.apply(source)
        source = message(2.3, actor(2, x=50.))
        source.header.seq = 19
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_events[0]['reason'], 'publisher_sequence_regression')

    def test_uint32_sequence_wrap_is_not_a_publisher_restart(self):
        self.retired()
        source = message(2.2, actor(2))
        source.header.seq = 0xffffffff
        self.helper.apply(source)
        source = message(2.3, actor(2))
        output = self.helper.apply(source)
        self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))
        self.assertEqual(self.helper.last_events, [])

    def test_fixed_zero_sequence_cannot_identify_silent_publisher_restart(self):
        # Explicit observability limit: an always-zero seq provides no epoch
        # evidence while time and raw UUID continue, unlike native publisher.
        self.retired()
        self.helper.apply(message(2.2, actor(2)))
        output = self.helper.apply(message(2.3, actor(2, x=50.)))
        self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))

    def test_footprint_iou_is_numerically_stable_at_utm_coordinates(self):
        local = (_observe(actor(1, x=.1, y=.2, yaw=.7)),
                 _observe(actor(2, x=.25, y=.1, yaw=-.3)))
        expected = _footprint_iou(*local)
        for origin_x, origin_y in ((302000., 4120000.), (10000000., 30000000.)):
            translated = (_observe(actor(1, x=origin_x + .1, y=origin_y + .2, yaw=.7)),
                          _observe(actor(2, x=origin_x + .25, y=origin_y + .1, yaw=-.3)))
            self.assertAlmostEqual(_footprint_iou(*translated), expected, places=7)

    def test_large_forward_callback_gap_invalidates_active_mapping(self):
        self.retired()
        self.helper.apply(message(2.2, actor(2)))
        source = message(20., actor(2, x=50.))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_events[0]['reason'], 'forward_callback_gap')

    def test_invalid_nearby_geometry_cannot_be_ignored_for_new_handoff(self):
        self.retired()
        source = message(2.2, actor(2), actor(3, x=float('nan')))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))
        self.assertEqual(self.helper.last_rejections[0]['reason'], 'invalid_current_observation')

    def test_unknown_to_known_is_not_eligible_for_first_handoff(self):
        self.retired()
        source = message(2.2, actor(2, label=1))
        self.assertEqual(wire(self.helper.apply(source)), wire(source))

    def test_fresh_ambiguous_moving_newborn_is_not_provably_same_physical_object(self):
        # Explicit limitation: at its first confirmed position a different,
        # identically shaped UNKNOWN can be observationally indistinguishable
        # from reacquisition of the old static object. We do not use its noisy
        # newborn EKF speed as a false physical-identity oracle. Preserve that
        # speed/geometry exactly and expose the assumption in the replay report.
        self.retired()
        source = message(2.2, actor(2, speed=2.))
        output = self.helper.apply(source)
        self.assertEqual(bytes(output.objects[0].object_id.uuid), identity(1))
        self.assertEqual(output.objects[0].kinematics.twist_with_covariance.twist.linear.x, 2.)
        restored = copy.deepcopy(output)
        restored.objects[0].object_id.uuid = list(identity(2))
        self.assertEqual(wire(restored), wire(source))

    def test_invalid_config_rejected(self):
        for arguments in ({'minimum_samples': 1}, {'maximum_gap_sec': 0.},
                          {'maximum_raw_speed_mps': float('nan')},
                          {'support_window_sec': .1}, {'minimum_footprint_iou': 1.1}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                ContinuityConfig(**arguments)


if __name__ == '__main__':
    unittest.main()
