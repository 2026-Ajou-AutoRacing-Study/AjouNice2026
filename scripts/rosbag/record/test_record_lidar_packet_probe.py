import importlib.util
import pathlib
import types
import unittest
import tempfile

try:
    import rosbag
    import rospy
    import roslib.message
    from velodyne_msgs.msg import VelodynePacket, VelodyneScan
    ROS_AVAILABLE = roslib.message.get_message_class("morai_msgs/EgoVehicleStatus") is not None
except ImportError:
    ROS_AVAILABLE = False

PATH = pathlib.Path(__file__).with_name("record_lidar_packet_probe.py")
SPEC = importlib.util.spec_from_file_location("probe", PATH)
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class ProbeTests(unittest.TestCase):
    def contract(self):
        return dict(model="VLP16", rpm=900.0, timestamp_first_packet=True,
                    cut_angle=-0.01, time_offset=0.0, gps_time=False, pcap_time=False)

    def test_default_contract(self):
        probe.require_driver_contract(self.contract())

    def test_changed_contract_rejected(self):
        for key, value in [("rpm", 750), ("timestamp_first_packet", False),
                           ("cut_angle", 0), ("time_offset", 1),
                           ("gps_time", True), ("pcap_time", True), ("npackets", 50)]:
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                probe.require_driver_contract(dict(self.contract(), **{key: value}))

    def test_missing_and_single_message_not_ready(self):
        counts = {topic: 2 for topic in probe.REQUIRED}
        self.assertEqual(probe.missing_inputs(counts), [])
        counts["/sensor/lidar1/packets"] = 1
        self.assertEqual(probe.missing_inputs(counts), ["/sensor/lidar1/packets"])

    def test_packet_summary(self):
        stamp = lambda t: types.SimpleNamespace(to_sec=lambda: t)
        packets = [types.SimpleNamespace(data=bytes(1206), stamp=stamp(i * 0.001)) for i in range(51)]
        scan = types.SimpleNamespace(packets=packets, header=types.SimpleNamespace(stamp=stamp(0)))
        result = probe.packet_summary([scan, scan])
        self.assertEqual(result["scans"], 2)
        self.assertEqual(result["packet_counts"], [51])
        self.assertEqual(result["packet_bytes"], [1206])
        self.assertAlmostEqual(result["packet_span_sec_max"], 0.05)
        self.assertEqual(result["header_minus_first_packet_sec_max_abs"], 0)

    def test_empty_scan(self):
        result = probe.packet_summary([types.SimpleNamespace(packets=[])])
        self.assertEqual(result["packet_counts"], [0])
        self.assertIsNone(result["header_minus_first_packet_sec_max_abs"])

    def test_scenarios_exist_and_keyboard_park(self):
        import json
        for name in probe.CASES.values():
            data = json.loads((probe.ROOT / "configs/scenario/perception_benchmark" / name).read_text())
            ego = data["egoVehicle"]
            self.assertEqual(ego["currentControlMode"], 0)
            self.assertEqual(ego["initPosition"]["gear"], 1)


@unittest.skipUnless(ROS_AVAILABLE, "source ROS runtime for synthetic bag tests")
class SyntheticBagTests(unittest.TestCase):
    def write_bag(self, path, packet_count=51, omit=None):
        with rosbag.Bag(str(path), "w", compression="lz4") as bag:
            for index in range(2):
                now = rospy.Time.from_sec(100 + index)
                for topic, typename in probe.REQUIRED.items():
                    if topic == omit:
                        continue
                    message = roslib.message.get_message_class(typename)()
                    message.header.stamp = now
                    if topic == "/sensor/lidar1/packets":
                        message.packets = [VelodynePacket(stamp=now + rospy.Duration(i * 0.001),
                                                        data=bytes(1206)) for i in range(packet_count)]
                    bag.write(topic, message, now)

    def test_valid_bag(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / "test.bag"
            self.write_bag(path)
            audit = probe.audit_bag(path)
            self.assertTrue(audit["pass"])
            self.assertEqual(audit["packets"]["scans"], 2)

    def test_wrong_count_bag(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / "test.bag"
            self.write_bag(path, packet_count=50)
            self.assertFalse(probe.audit_bag(path)["pass"])

    def test_missing_camera_bag(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / "test.bag"
            self.write_bag(path, omit="/sensor/camera/back/compressed")
            audit = probe.audit_bag(path)
            self.assertFalse(audit["pass"])
            self.assertEqual(audit["missing_inputs"], ["/sensor/camera/back/compressed"])


if __name__ == "__main__":
    unittest.main()
