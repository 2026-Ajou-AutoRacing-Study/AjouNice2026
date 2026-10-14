import copy
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import prepare_source_footer_gt_sensitivity as sensitivity


def packet(stamp, footer):
    data = bytearray(1206)
    for offset in range(0, 1200, 100):
        struct.pack_into("<HH", data, offset, 0xeeff, (offset//100)*60)
    struct.pack_into("<I", data, 1200, footer)
    return NS(stamp=NS(to_sec=lambda: stamp), data=data)


def scan(size=50, mixed=False):
    packets = [packet(3600.10+index*.001, 200000 if not mixed or index == 0 else 210000) for index in range(size)]
    return NS(header=NS(stamp=NS(to_sec=lambda: packets[0].stamp.to_sec())), packets=packets)


class SourceFooterSensitivityTests(unittest.TestCase):
    def test_exactly_one_source_50_packet_frame_is_supported(self):
        anchor = sensitivity.scan_anchor(scan())
        self.assertTrue(anchor["eligible"])
        self.assertEqual(anchor["footer_distinct"], 1)
        self.assertAlmostEqual(anchor["evaluation_reference_stamp"], 3600.2)
        self.assertAlmostEqual(anchor["source_footer_minus_header_sec"], .1)

    def test_51_packet_and_mixed_source_50_are_not_assigned_a_fake_anchor(self):
        self.assertFalse(sensitivity.scan_anchor(scan(size=51))["eligible"])
        mixed = sensitivity.scan_anchor(scan(mixed=True))
        self.assertFalse(mixed["eligible"])
        self.assertEqual(mixed["footer_distinct"], 2)

    def test_invalid_block_contract_or_changed_first_packet_stamp_is_rejected(self):
        value = scan()
        value.packets[0].data[0] = 0
        with self.assertRaises(ValueError):
            sensitivity.scan_anchor(value)
        value = scan()
        value.header.stamp.to_sec = lambda: 3600.2
        with self.assertRaises(ValueError):
            sensitivity.scan_anchor(value)

    def test_nearest_hour_wrap_is_only_experimental_not_header_replacement(self):
        value = scan()
        value.packets = [packet(7199.99+index*.0001, 10000) for index in range(50)]
        value.header.stamp.to_sec = lambda: 7199.99
        anchor = sensitivity.scan_anchor(value)
        self.assertAlmostEqual(anchor["evaluation_reference_stamp"], 7200.01)
        self.assertAlmostEqual(value.header.stamp.to_sec(), 7199.99)

    def test_gt_only_mutation_preserves_every_model_input_and_header_pose(self):
        original = dict(token="frame", stamp=1., point_path="points.bin", images={"cam": "image.jpg"},
                        ego=dict(x=1., y=2., z=3., yaw=.2), gt_boxes=[[0., 0., 0., 4., 2., 2., 0.]])
        derived = copy.deepcopy(original)
        derived.update(gt_boxes=[[1., 0., 0., 4., 2., 2., 0.]], evaluation_reference_stamp=1.05,
                       evaluation_ego=dict(x=2., y=2., z=3., yaw=.2))
        sensitivity.unchanged_sensor_contract(original, derived)
        derived["stamp"] = 1.05
        with self.assertRaises(ValueError):
            sensitivity.unchanged_sensor_contract(original, derived)
        derived["stamp"] = 1.
        derived["ego"]["x"] = 2.
        with self.assertRaises(ValueError):
            sensitivity.unchanged_sensor_contract(original, derived)


if __name__ == "__main__":
    unittest.main()
