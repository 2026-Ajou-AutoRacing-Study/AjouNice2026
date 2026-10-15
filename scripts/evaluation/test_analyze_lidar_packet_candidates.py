import math
import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from analyze_lidar_packet_candidates import operating, paired_metrics, to_world, yaw_axis_delta
from infer_lidar_packet_candidates import nearest_received


def box(x=0., score=.8, cls='vehicle', yaw=0.):
    return dict(center=[x, 0., 0.], size=[2., 1., 1.], yaw=yaw, score=score, class_name=cls)


class PairedPacketTests(unittest.TestCase):
    def test_floor_nms_class_and_threshold(self):
        result = operating([box(score=.05), box(score=.39), box(score=.41),
                            box(score=.11, cls='obstacle')])
        self.assertEqual([(b['class_name'], b['score']) for b in result],
                         [('vehicle', .41), ('obstacle', .11)])

    def test_top100_before_bridge_nms(self):
        result = operating([box(score=.9)]*100+[box(10., score=.8)])
        self.assertEqual(len(result), 1)

    def test_axis_ambiguity(self):
        self.assertAlmostEqual(yaw_axis_delta(.2, .2+math.pi), 0.)
        self.assertAlmostEqual(yaw_axis_delta(0, math.pi/2), math.pi/2)

    def test_pairing_is_class_preserving_and_counts_missing(self):
        result = paired_metrics([box(), box(10., cls='pedestrian')],
                                [box(.2), box(10., cls='obstacle')])
        self.assertEqual(result['vehicle']['paired'], 1)
        self.assertAlmostEqual(result['vehicle']['center_m'][0], .2)
        self.assertEqual(result['pedestrian']['baseline_unmatched'], 1)
        self.assertEqual(result['obstacle']['candidate_unmatched'], 1)

    def test_world_declared_reference_pose(self):
        converted = to_world([box(2.)], dict(center=[10., 20., 30.], yaw=math.pi/2),
                             np.asarray([1., 0., 2.]))[0]
        self.assertTrue(np.allclose(converted['center'], [10., 23., 32.]))
        self.assertAlmostEqual(converted['yaw'], math.pi/2)

    def test_image_nearest_must_already_be_received(self):
        chosen = nearest_received([(1., 2., 'future'), (.98, 1.1, 'available')], 1., 1.2)
        self.assertEqual(chosen[-1], 'available')
        self.assertIsNone(nearest_received([(1., 2., 'future')], 1., 1.2))
        self.assertIsNone(nearest_received([(.8, .9, 'old')], 1., 1.2))


if __name__ == '__main__':
    unittest.main()
