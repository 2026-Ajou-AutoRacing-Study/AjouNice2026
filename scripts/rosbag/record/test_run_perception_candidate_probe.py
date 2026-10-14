import copy
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import run_perception_candidate_probe as probe


class ProbeTests(unittest.TestCase):
    def test_unique_bounded_install_stem(self):
        a = probe.install_stem(Path('/tmp/'+'very_long_'*30), 8)
        self.assertLessEqual(len(a.encode()), 30)
        self.assertNotEqual(a, probe.install_stem(Path('/tmp/other'), 8))
        self.assertNotEqual(a, probe.install_stem(Path('/tmp/'+'very_long_'*30), 7))

    def test_unity_coordinate_contract(self):
        self.assertEqual(probe.unity(dict(x=1., y=2., z=3.)), dict(x=-1., y=3., z=-2.))

    def test_two_points_identity_and_parallel_offset_preserve_heights(self):
        template = dict(initPos={}, standardPos={}, anim=dict(foo=7))
        before = copy.deepcopy(template)
        profile = dict(live_operational_qa_passed=True, maximum_path_offset_m=1.,
            knots=[dict(x=0., y=0., z=2.), dict(x=10., y=0., z=2.1)])
        actor, path, metadata = probe.pedestrian(template, 301, 'Man1', 50100001, profile, .4)
        self.assertEqual(template, before)
        self.assertEqual(actor['UNIQUEID'], 301)
        self.assertEqual(actor['anim'], before['anim'])
        self.assertEqual([p['pos']['y'] for p in path['waypointData']], [2., 2.1])
        self.assertEqual([p['pos']['z'] for p in path['waypointData']], [-.4, -.4])
        self.assertEqual(len(actor['waypointEventList']), 2)
        opposite = probe.pedestrian(template, 302, 'Man1', 50100001, profile, -.4, True)
        self.assertEqual(opposite[2]['points'][0]['x'], 10.)

    def test_no_height_extrapolation_or_unqualified_profile(self):
        profile = dict(live_operational_qa_passed=True, maximum_path_offset_m=1.,
            knots=[dict(x=0., y=0., z=2.), dict(x=10., y=0., z=2.1)])
        with self.assertRaises(ValueError):
            probe.pedestrian({}, 301, 'Man1', 50100001, profile, 1.1)
        profile['live_operational_qa_passed'] = False
        with self.assertRaises(ValueError):
            probe.pedestrian({}, 301, 'Man1', 50100001, profile)

    def test_driver_trials_do_not_change_rpm_or_stamp(self):
        self.assertEqual(probe.POLICIES['fixed51'], [])
        self.assertEqual(probe.POLICIES['fixed50'], ['lidar_npackets:=50'])
        self.assertEqual(probe.POLICIES['cut0'], ['lidar_cut_angle:=0'])


if __name__ == '__main__':
    unittest.main()
