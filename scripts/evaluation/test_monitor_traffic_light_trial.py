import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('trial_monitor', Path(__file__).with_name('monitor_traffic_light_trial.py'))
monitor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(monitor)


class Stamp:
    __slots__ = ('secs', 'nsecs')
    def __init__(self, secs=4, nsecs=5): self.secs, self.nsecs = secs, nsecs


class Header:
    __slots__ = ('stamp', 'frame_id')
    def __init__(self): self.stamp, self.frame_id = Stamp(), 'camera'


class Message:
    __slots__ = ('header', 'values')
    _type = 'fake/Stamped'
    def __init__(self): self.header, self.values = Header(), [1., 2.]


class String:
    __slots__ = ('data',)
    _type = 'std_msgs/String'
    def __init__(self, data): self.data = data


class BuiltinTime:
    __slots__ = ('sec', 'nanosec')
    def __init__(self, sec=4, nanosec=5): self.sec, self.nanosec = sec, nanosec


class SignalArray:
    __slots__ = ('stamp', 'traffic_light_groups')
    _type = 'autoware_perception_msgs/TrafficLightGroupArray'
    def __init__(self, stamp=None):
        self.stamp = BuiltinTime() if stamp is None else stamp
        self.traffic_light_groups = []


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.poses = [{'group_id':'A','group_regulatory_id':201351,'x':0.,'y':0.,'yaw':179.}]
        self.ego = {'x':0.,'y':0.,'heading':-179.,'receipt_monotonic':10.}

    def test_yaw_wrap_and_match(self):
        self.assertEqual(monitor.yaw_distance(179., -179.), 2.)
        self.assertEqual(monitor.select_group(self.poses,self.ego,10.1)['group_id'],201351)

    def test_missing_invalid_nonfinite_fail_closed(self):
        self.assertEqual(monitor.select_group(self.poses,None,10.)['group_id'],0)
        for value in (float('nan'),float('inf'),True):
            self.assertEqual(monitor.select_group(self.poses,{**self.ego,'x':value},10.1)['group_id'],0)

    def test_receipt_age_strict_and_clock_regression(self):
        for now in (10.5,11.,9.):
            self.assertEqual(monitor.select_group(self.poses,self.ego,now)['group_id'],0)
        self.assertEqual(monitor.select_group(self.poses,self.ego,10.499)['group_id'],201351)

    def test_strict_distance_and_yaw_gates(self):
        for changes in ({'x':2.},{'heading':164.}):
            self.assertEqual(monitor.select_group(self.poses,{**self.ego,**changes},10.1)['group_id'],0)

    def test_ambiguous_missions_zero_same_id_duplicates_allowed(self):
        other={**self.poses[0],'group_id':'B','group_regulatory_id':201365}
        self.assertEqual(monitor.select_group(self.poses+[other],self.ego,10.1)['group_id'],0)
        self.assertEqual(monitor.select_group(self.poses*2,self.ego,10.1)['group_id'],201351)

    def test_explicit_plan_validation_hash_and_no_guess(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json'
            path.write_text(json.dumps({'poses':[{'group_id':'A','group_regulatory_id':201351,
                                                  'pose':{'x':0,'y':0,'yaw':179}}]}))
            poses,meta=monitor.load_plan(path)
            self.assertEqual(poses,self.poses);self.assertEqual(len(meta['sha256']),64)
            path.write_text(json.dumps({'poses':[{'group_id':'A','group_regulatory_id':0,'pose':{}}]}))
            with self.assertRaises(ValueError):monitor.load_plan(path)

    def test_header_and_raw_debug_text_association(self):
        row=monitor.observation_record('/topic',Message(),9000000000,12.)
        self.assertEqual(row['source_header_stamp_ns'],4000000005)
        self.assertEqual(row['message']['values'],[1.,2.])
        text='{"source_stamp_ns":123,"status":"unknown"}'
        row=monitor.observation_record('/debug',String(text),99,1.)
        self.assertEqual(row['message']['data'],text)
        self.assertEqual(row['image_association_stamp_ns'],123)
        self.assertTrue(row['not_node_compute_time'])
        nonfinite=monitor.observation_record('/debug',String('{"confidence":NaN}'),99,1.)
        self.assertEqual(nonfinite['parsed_debug']['confidence'],{'nonfinite':'nan'})
        self.assertEqual(nonfinite['message']['data'],'{"confidence":NaN}')

    def test_serialization_limits_and_binary_omission(self):
        self.assertEqual(monitor.to_json_value(b'JPEG')['bytes_omitted'],4)
        self.assertEqual(monitor.to_json_value(float('nan')),{'nonfinite':'nan'})
        with self.assertRaises(ValueError):monitor.to_json_value([0]*1025)
        with self.assertRaises(ValueError):monitor.to_json_value(object())

    def test_headerless_typed_array_uses_separate_top_level_stamp(self):
        row=monitor.observation_record('/traffic_light_trial/traffic_signals',SignalArray(),99,1.)
        self.assertIsNone(row['source_header_stamp_ns'])
        self.assertEqual(row['source_message_stamp_ns'],4000000005)
        self.assertEqual(row['image_association_stamp_ns'],4000000005)
        self.assertEqual(row['image_association_stamp_origin'],'message.stamp')
        self.assertEqual(row['message']['stamp'],{'sec':4,'nanosec':5})

    def test_top_level_time_validation_and_zero_sentinel(self):
        for sec,nanosec in ((-1,0),(1,-1),(1,1000000000),(True,0),(1,2.5)):
            row=monitor.observation_record('/signals',SignalArray(BuiltinTime(sec,nanosec)),99,1.)
            self.assertIsNone(row['source_message_stamp_ns'])
            self.assertIsNone(row['image_association_stamp_ns'])
        row=monitor.observation_record('/signals',SignalArray(BuiltinTime(0,0)),99,1.)
        self.assertEqual(row['source_message_stamp_ns'],0)
        self.assertIsNone(row['image_association_stamp_ns'])

    def test_exact_image_join_uses_source_not_receipt_clock(self):
        signal_row=monitor.observation_record('/signals',SignalArray(),99000000000,900.)
        debug_row=monitor.observation_record('/debug',String('{"source_stamp_ns":4000000005}'),1,3.)
        self.assertNotEqual(signal_row['receipt_ros_ns'],debug_row['receipt_ros_ns'])
        self.assertEqual(signal_row['image_association_stamp_ns'],debug_row['image_association_stamp_ns'])
        # A generic odometry/IMU header is preserved but never promoted to camera identity.
        generic=monitor.observation_record('/odom',Message(),99000000000,900.)
        self.assertEqual(generic['source_header_stamp_ns'],4000000005)
        self.assertIsNone(generic['image_association_stamp_ns'])
        contract=monitor.timestamp_contracts()
        self.assertFalse(contract['common_source_and_receipt_ros_clock_independently_verified'])
        self.assertIn('never',contract['receipt_monotonic_clock'])
        self.assertEqual(contract['image_association_priority'][0],'positive_valid_message.stamp')

    def test_compressed_header_only_and_truncation(self):
        raw=struct.pack('<IIII',7,10,12,3)+b'cam'+b'JPEG_NOT_PARSED'
        header,stamp=monitor.compressed_header(raw)
        self.assertEqual(header['frame_id'],'cam');self.assertEqual(stamp,10000000012)
        with self.assertRaises(ValueError):monitor.compressed_header(raw[:8])
        with self.assertRaises(ValueError):monitor.compressed_header(struct.pack('<IIII',1,1,1000000000,0))

    def test_fresh_jsonl_threadsafe_close_and_trial_namespace(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'data.jsonl';writer=monitor.JsonlWriter(path)
            writer.write({'event':'test'});writer.close();writer.close()
            self.assertEqual(json.loads(path.read_text()),{'event':'test'})
            with self.assertRaises(FileExistsError):monitor.JsonlWriter(path)
        self.assertEqual(monitor.validate_route_topic(monitor.ROUTE_TOPIC),monitor.ROUTE_TOPIC)
        for topic in ('/control_cmd','relative','/traffic_light_trial/../../control'):
            with self.assertRaises(ValueError):monitor.validate_route_topic(topic)


if __name__=='__main__':unittest.main()
