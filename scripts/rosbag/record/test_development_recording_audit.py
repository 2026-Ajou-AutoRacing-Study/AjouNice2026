import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('record_audit', Path(__file__).with_name('development_recording_audit.py'))
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class AuditTests(unittest.TestCase):
    def test_capabilities_require_actual_nonzero_counts(self):
        topics = {t: {'messages': 10} for t in audit.GROUPS['raw_sensor_replay']['all']}
        topics['/sensor/lidar1/packets']['messages'] = 0
        result = audit.capabilities(topics)
        self.assertFalse(result['raw_sensor_replay']['recorded'])
        self.assertIn('/sensor/lidar1/packets', result['raw_sensor_replay']['missing_all_required'])
        self.assertFalse(result['detector_stage_timing']['recorded'])

    def test_canonical_or_trial_timing_is_sufficient(self):
        self.assertTrue(audit.capabilities({audit.TIMINGS[1]: {'messages': 1}})['detector_stage_timing']['recorded'])

    def test_no_fictitious_cruise_output_required(self):
        self.assertTrue(all(not n.endswith('/obstacle_cruise/trajectory')
                            for g in audit.GROUPS.values() for names in g.values() for n in names))

    def test_raw_header_and_both_rosbag_layouts(self):
        data = struct.pack('<IIII', 7, 12, 345, 3) + b'cam'
        self.assertEqual(audit.header(data), (7, 12000000345, 19))
        self.assertEqual(audit.raw_bytes(('type', data, 'md5', 0, None)), data)
        self.assertEqual(audit.raw_bytes(('type', (data, 'md5', 0), None)), data)

    def test_bad_headers_rejected_not_clamped(self):
        for data in (b'', struct.pack('<IIII', 1, 1, 1000000000, 0),
                     struct.pack('<IIII', 1, 1, 1, 100)):
            with self.subTest(data=data), self.assertRaises(ValueError):
                audit.header(data)

    def test_ros1_port_message_types_have_headers(self):
        for name in ('autoware_perception_msgs/DetectedObjects',
                     'autoware_perception_msgs/TrackedObjects',
                     'autoware_perception_msgs/PredictedObjects',
                     'autoware_planning_msgs/Trajectory'):
            self.assertIn(name, audit.HEADER_TYPES)

    def test_percentiles_and_empty_population(self):
        self.assertEqual(audit.summary([]), {'count': 0})
        self.assertEqual(audit.summary([1, 2, 3, 4])['p50'], 2.5)
        self.assertAlmostEqual(audit.summary([1, 2, 3, 4])['p95'], 3.85)


try:
    import rosbag
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import CompressedImage
    from autoware_perception_msgs.msg import DetectedObjects
except ImportError:
    rosbag = None


@unittest.skipIf(rosbag is None, 'ROS message serializers unavailable')
class SyntheticBagTests(unittest.TestCase):
    def test_closed_bag_audit_no_master(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'synthetic.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for seq, sec in ((1, 1.), (3, 1.1), (0, 1.5)):
                    msg = CompressedImage()
                    msg.header.seq = seq
                    msg.header.stamp = rospy.Time.from_sec(sec - .03)
                    msg.header.frame_id = 'front'
                    msg.format = 'jpeg'
                    msg.data = b'original-bytes'
                    bag.write(audit.CAMERAS[0], msg, rospy.Time.from_sec(sec))
                bag.write(audit.TIMINGS[0], String(data=json.dumps({'server_inference_ms': 60.,
                          'counters': {'camera_sync_reject': 2}})), rospy.Time.from_sec(1.6))
                bag.write(audit.TIMINGS[0], String(data='bad json'), rospy.Time.from_sec(1.7))
                det = DetectedObjects()
                det.header.stamp = rospy.Time.from_sec(1.6)
                bag.write(audit.DET, det, rospy.Time.from_sec(1.7))
            report = audit.audit(path, ['.*'], audit.CAMERAS, [])
            row = report['cadence'][audit.CAMERAS[0]]
            self.assertEqual(row['header_sequence_skips'], 1)
            self.assertEqual(row['header_sequence_nonincreasing'], 1)
            self.assertEqual(row['gaps_over_250ms'], 1)
            self.assertAlmostEqual(row['header_to_bag_receipt_ms']['mean'], 30., places=4)
            self.assertFalse(report['analysis_capabilities']['raw_sensor_replay']['recorded'])
            self.assertTrue(report['analysis_capabilities']['detector_stage_timing']['recorded'])
            self.assertEqual(report['detector_frame_timing']['invalid_json'], 1)
            self.assertEqual(report['detector_frame_timing']['metrics']['server_inference_ms']['p95'], 60.)
            self.assertAlmostEqual(report['cadence'][audit.DET]['header_to_bag_receipt_ms']['mean'], 100.)


if __name__ == '__main__':
    unittest.main()
