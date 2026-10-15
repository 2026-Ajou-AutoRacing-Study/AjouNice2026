import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from prepare_base5_z_margin_probe import shift_input, measured_corrections, ASSETS


class ZMarginProbeTest(unittest.TestCase):
    def fixture(self):
        scene = dict(pedestrianList=[dict(UNIQUEID=1, pos=dict(x=2., y=3., z=28.),
            initPos=dict(y=28.), standardPos=dict(y=28.))],
            waypointDataList=[dict(pathName='test - 1', waypointData=[dict(pos=dict(x=2., y=28., z=3.))])])
        meta = dict(runtime_actors=[dict(unique_id=1, source_list='pedestrianList',
            position=dict(x=2., y=3., z=28.), motion_path=dict(points=[dict(z=28.)], target_position=dict(z=28.)))],
            motion_probe=dict(target_position=dict(z=28.), height_calibration=dict(static_z=28.)))
        return scene, meta

    def test_positive_margin_updates_all_height_fields_not_xy(self):
        scene, meta = self.fixture()
        shift_input(scene, meta, .02)
        self.assertEqual(scene['pedestrianList'][0]['pos']['x'], 2.)
        self.assertAlmostEqual(scene['pedestrianList'][0]['pos']['z'], 28.02)
        self.assertAlmostEqual(scene['pedestrianList'][0]['initPos']['y'], 28.02)
        self.assertAlmostEqual(scene['waypointDataList'][0]['waypointData'][0]['pos']['y'], 28.02)
        self.assertAlmostEqual(meta['motion_probe']['target_position']['z'], 28.02)
        self.assertNotIn('height_calibration', meta['motion_probe'])

    def test_bounded_negative_control(self):
        scene, meta = self.fixture()
        shift_input(scene, meta, -.3)
        self.assertAlmostEqual(meta['runtime_actors'][0]['position']['z'], 27.7)

    def test_large_positive_margin_refused(self):
        scene, meta = self.fixture()
        with self.assertRaises(ValueError):
            shift_input(scene, meta, .5)

    def test_trace_uses_uid_and_reports_only_small_interval_steps(self):
        from report_base5_z_margin_probe import summarize_trace
        samples=[dict(uid=1,t=1.,z=28.),dict(uid=2,t=1.05,z=99.),
            dict(uid=1,t=1.1,z=28.02),dict(uid=1,t=1.2,z=28.02)]
        result=summarize_trace(samples,1,0.,1.2)
        self.assertEqual(result['samples'],3)
        self.assertAlmostEqual(result['maximum_adjacent_z_step_m'],.02)
        self.assertEqual(result['transitions_over_10cm'],[])

    def test_ground_control_requires_safe_measured_session(self):
        with self.assertRaises(ValueError):
            measured_corrections(dict(shutdown=dict(park_verified=False),rows=[]))
        rows=[dict(asset=name,mode=mode,ground_gap_m=.32,collision_clear=True,
            delta_m=.02,capture_root='raw',source_sha256='fixture') for _,name in ASSETS for mode in ('static','moving')]
        corrections=measured_corrections(dict(shutdown=dict(park_verified=True),rows=rows))
        self.assertAlmostEqual(corrections[('Dog2','moving')]['shift_m'],-.33)

    def test_below_ground_is_not_repaired_by_arbitrary_upward_shift(self):
        rows=[dict(asset=name,mode=mode,ground_gap_m=-.1,collision_clear=True,
            delta_m=0.,capture_root='raw',source_sha256='fixture') for _,name in ASSETS for mode in ('static','moving')]
        corrections=measured_corrections(dict(shutdown=dict(park_verified=True),rows=rows))
        self.assertEqual(corrections[('Dog2','moving')]['shift_m'],0.)


if __name__ == '__main__':
    unittest.main()
