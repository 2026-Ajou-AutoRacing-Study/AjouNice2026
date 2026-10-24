"""Live-bag audit protocol tests; no ROS master or simulator needed."""
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from analyze_map_mixed_horizon_live import (
    compare_recorded, input_key, normalized_wire, path_lengths, roundabout_internal_ids,
)
from qualify_e12_mixed_horizon import wire
import test_qualify_e12_mixed_horizon as qualification_tests


class LiveBagAuditTest(unittest.TestCase):
    def output(self, poses=31):
        import rospy
        message = qualification_tests.MixedHorizonQualificationTest().output(poses)
        message.header.stamp = rospy.Time(10, 123)
        return message

    def test_publisher_seq_only_is_ignored(self):
        first, second = self.output(), self.output()
        second.header.seq = 100
        self.assertEqual(normalized_wire(first), normalized_wire(second))
        second.header.frame_id = 'wrong'
        self.assertNotEqual(normalized_wire(first), normalized_wire(second))

    def test_serialization_views_do_not_mutate_original_messages(self):
        message = self.output(61)
        before = wire(message)
        self.assertEqual(normalized_wire(message, prefix=True), normalized_wire(self.output()))
        normalized_wire(message)
        self.assertEqual(wire(message), before)
        self.assertEqual(message.header.seq, 42)
        self.assertEqual(len(message.objects[0].kinematics.predicted_paths[0].path), 61)

    def test_exact_stamp_ordered_uuid_key_rejects_nearby(self):
        first, second = self.output(), self.output()
        second.header.stamp.nsecs += 1
        self.assertNotEqual(input_key(first), input_key(second))
        second = deepcopy(first)
        second.objects[0].object_id.uuid[0] ^= 1
        self.assertNotEqual(input_key(first), input_key(second))

    def test_recorded_prefix_and_current_fields_preserved(self):
        baseline, mixed = self.output(), self.output(61)
        mixed.header.seq = 20
        computed = {input_key(baseline): [(normalized_wire(baseline), normalized_wire(mixed))]}
        result = compare_recorded([mixed], computed)
        self.assertEqual(result['counts']['prefix_exact'], 1)
        self.assertEqual(result['counts']['full_mixed_wire_exact'], 1)
        mixed.objects[0].kinematics.initial_twist_with_covariance.covariance[4] = .2
        result = compare_recorded([mixed], computed)
        self.assertEqual(result['counts']['prefix_mismatch'], 1)

    def test_duplicate_input_keys_are_not_cherry_picked(self):
        message = self.output()
        serialized = normalized_wire(message)
        computed = {input_key(message): [(serialized, serialized), (b'bad', b'bad')]}
        result = compare_recorded([message], computed)
        self.assertEqual(result['counts'], {'ambiguous_input_key': 1})
        self.assertEqual(compare_recorded([message], {})['counts'], {'unmatched_input_key': 1})

    def test_partial_extension_is_flagged(self):
        message = self.output(61)
        message.objects[0].kinematics.predicted_paths.append(
            deepcopy(self.output().objects[0].kinematics.predicted_paths[0]))
        counts, invalid = path_lengths(message)
        self.assertEqual(dict(counts), {61: 1, 31: 1})
        self.assertEqual(invalid, 1)

    def test_current_regulatory_roles_determine_ring(self):
        xml = '''<osm><relation id="1"><member type="relation" ref="8" role="entry"/>
          <member type="relation" ref="9" role="internal"/>
          <tag k="subtype" v="roundabout"/></relation>
          <relation id="2"><member type="relation" ref="10" role="internal"/>
          <tag k="subtype" v="other"/></relation></osm>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'map.osm'
            path.write_text(xml)
            self.assertEqual(roundabout_internal_ids(path), {9})


if __name__ == '__main__':
    unittest.main()
