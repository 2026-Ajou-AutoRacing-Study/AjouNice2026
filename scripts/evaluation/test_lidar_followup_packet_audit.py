import pathlib
import struct
import sys
import types
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import lidar_followup_packet_audit as followup


class FollowupPacketTests(unittest.TestCase):
    def test_inclusive_ranges_and_zero_returns(self):
        data = bytearray(1206)
        for i, value in enumerate((0, 199, 200, 65000, 65001)):
            struct.pack_into('<H', data, 4+3*i, value)
        counts = followup.raw_return_counts(types.SimpleNamespace(data=data))
        self.assertEqual(counts[0], 2)
        self.assertEqual(int(counts.sum()), 2)

    def test_invalid_packet_length_rejected(self):
        with self.assertRaises(ValueError):
            followup.raw_return_counts(types.SimpleNamespace(data=b''))

    def test_50_complete_and_51_recurrent_sector(self):
        angles = [(123 + i*60) % 36000 for i in range(612)]
        self.assertEqual(np.count_nonzero(followup.repeated_direction_blocks(angles[:600])), 0)
        self.assertEqual(np.count_nonzero(followup.repeated_direction_blocks(angles)), 12)

    def test_centidegree_rounding_is_not_new_direction(self):
        angles = [(123 + i*60 + (i%3 == 0)) % 36000 for i in range(612)]
        self.assertEqual(np.count_nonzero(followup.repeated_direction_blocks(angles)), 12)

    def test_phase_sweep_aligned_phase_only(self):
        footers = (np.arange(260)+13)//50
        stamps = np.arange(260)*.001
        angles = (np.arange(260)*720)%36000
        rows = followup.phase_sweep(stamps, footers, angles)
        aligned = [r for r in rows if r['single_footer_fraction'] == 1.]
        self.assertEqual([r['offset_packets'] for r in aligned], [37])
        self.assertTrue(all(r['single_footer_fraction'] == 0. for r in rows if r not in aligned))

    def test_pose_interpolation_and_no_extrapolation(self):
        ego = dict(times=np.array([0., .1]), xyz=np.array([[0., 0., 0.], [2., 1., 0.]]),
                   yaw=np.array([0., 1.]), speed=np.array([0., 10.]))
        sample = followup.sample_ego(ego, .05)
        np.testing.assert_array_equal(sample['xyz'], [1., .5, 0.])
        self.assertEqual(sample['speed'], 5.)
        self.assertIsNone(followup.sample_ego(ego, -.001))
        self.assertIsNone(followup.sample_ego(ego, .101))
        self.assertIsNone(followup.sample_ego(ego, .05, maximum_gap=.01))

    def test_scenario_reload_pose_jump_is_not_interpolated(self):
        ego = dict(times=np.array([0., .1]), xyz=np.array([[0., 0., 0.], [300., 0., 0.]]),
                   yaw=np.array([0., 1.]), speed=np.array([0., 0.]), segments=np.array([0, 1]))
        self.assertIsNone(followup.sample_ego(ego, .05))
        self.assertEqual(followup.sample_ego(ego, 0.)['segment'], 0)
        self.assertEqual(followup.sample_ego(ego, .1)['segment'], 1)

    def test_steady_summary_excludes_unavailable_pose(self):
        row = dict(source_footer_count=1, host_packet_span_ms=10., latest_footer_minus_header_ms=9.,
                   first_to_last_footer_ms=0., raw_in_range_points=4, repeated_angular_sector_blocks=0,
                   repeated_angular_sector_return_fraction=0., non_dominant_source_return_fraction=0., source_pose_translation_m=None,
                   source_pose_yaw_change_deg=None, ego_speed_kmh=None, reference_yaw_rate_deg_s=None)
        summary = followup.summarize_rows([row])
        self.assertEqual(summary['frames'], 1)
        self.assertEqual(summary['source_pose_translation_m']['count'], 0)

    def test_fixed_groups_do_not_repeat_or_publish_tail(self):
        groups = followup.fixed_groups(167, 50, 13)
        self.assertEqual(groups, [(13, 63), (63, 113), (113, 163)])


if __name__ == '__main__':
    unittest.main()
