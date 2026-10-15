import pathlib
import struct
import sys
import types
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import lidar_packet_reassembly as packet
from decode_lidar_packet_candidates import arrays_exact
import numpy as np


class PacketTests(unittest.TestCase):
    def test_fixed_groups_use_every_packet_once_except_tail(self):
        for policy, count in (("fixed50", 50), ("fixed51", 51)):
            groups = packet.group_indices([0] * 155, policy)
            flattened = [i for start, end in groups for i in range(start, end)]
            self.assertEqual(flattened, list(range((155 // count) * count)))

    def test_cut_zero_crossing_includes_boundary_packet(self):
        self.assertEqual(packet.group_indices([35000, 35720, 440, 1160, 35720, 440], "cut0"),
                         [(0, 3), (3, 6)])

    def test_cut_middle_normal_and_wrap(self):
        self.assertTrue(packet.crosses_cut(17900, 18100, 18000))
        self.assertFalse(packet.crosses_cut(35900, 100, 18000))
        self.assertTrue(packet.crosses_cut(10000, 9000, 18000))

    def test_no_boundary_synthetic_duplication(self):
        azimuths = [(i * 720 + 123) % 36000 for i in range(151)]
        groups = packet.group_indices(azimuths, "cut0")
        self.assertTrue(all(end - start == 50 for start, end in groups[1:]))
        self.assertTrue(all(a[1] == b[0] for a, b in zip(groups, groups[1:])))

    def test_block_marker_and_extent(self):
        rows = []
        for i in range(51):
            data = bytearray(1206)
            for block in range(12):
                struct.pack_into("<HH", data, block * 100, 0xEEFF, (i * 720 + block * 60) % 36000)
            rows.append(packet.rotations(types.SimpleNamespace(data=data)))
        self.assertAlmostEqual(packet.coverage_deg(rows, 0, 50), 360.0)
        self.assertAlmostEqual(packet.coverage_deg(rows, 0, 51), 367.2)
        with self.assertRaises(ValueError):
            packet.rotations(types.SimpleNamespace(data=bytes(1206)))

    def test_exact_fields_nan_and_dtype(self):
        left = np.asarray([1., np.nan], dtype=np.float32)
        self.assertTrue(arrays_exact(left, left.copy()))
        self.assertFalse(arrays_exact(left, left.astype(np.float64)))
        self.assertFalse(arrays_exact(left, np.asarray([2., np.nan], dtype=np.float32)))

    def test_footer_nearest_hour_at_rollover(self):
        self.assertAlmostEqual(packet.footer_nearest_hour(3599.99, 20000), 3600.02)
        self.assertAlmostEqual(packet.footer_nearest_hour(3600.01, 3599980000), 3599.98)
        with self.assertRaises(ValueError):
            packet.footer_nearest_hour(3600, 3600000000)


if __name__ == "__main__":
    unittest.main()
