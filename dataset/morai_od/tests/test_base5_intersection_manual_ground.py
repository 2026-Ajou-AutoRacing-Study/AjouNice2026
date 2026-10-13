import copy
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import prepare_base5_intersection_manual_ground as manual


class ManualGroundTest(unittest.TestCase):
    def test_constant_z_and_normalized_axis(self):
        crossing = dict(axis=[-2., 0.], center=dict(x=10., y=20., z=3.))
        original = copy.deepcopy(crossing)
        points, axis = manual.constant_path(crossing, 4.)
        self.assertEqual(crossing, original)
        self.assertEqual(axis, (1., -0.))
        self.assertEqual([p['x'] for p in points], [-2., 4., 10., 16., 22.])
        self.assertTrue(all(p['z'] == 4. and p['y'] == 20. for p in points))
        for p in points:
            self.assertEqual(manual.gen.geo.waypoint_to_world(manual.gen.geo.world_to_morai_waypoint(p)), p)

    def test_invalid_axis(self):
        with self.assertRaises(ValueError):
            manual.constant_path(dict(axis=[0., 0.], center=dict(x=0., y=0.)), 3.)

    def test_park_sets_both_serialized_gears(self):
        ego = dict(initPosition=dict(gear=4, vehicleSaveData=dict(velocity=10., inputData=[0]*7)),
                   currentVelocity=10., customValue=10.)
        manual.park(ego)
        self.assertEqual(ego['initPosition']['gear'], 1)
        self.assertEqual(ego['initPosition']['vehicleSaveData']['inputData'][6], 1)
        self.assertEqual(ego['currentVelocity'], 0.)

    def test_moving_path_serialization(self):
        scene = dict(waypointDataList=[], pedestrianList=[])
        actor = dict(pos={}, rot={}, initPos={}, standardPos={})
        asset = dict(name='Dog2', source_list='pedestrianList', ontology_class='Animal')
        points = [dict(x=float(i), y=2., z=28.) for i in range(5)]
        manual.gen.set_pose(actor, dict(points[0], yaw=0.), 1, asset, scene, 'walking',
                            dict(policy='natural-target-v1', points=points, speed_field_value=4.32))
        self.assertEqual(len(scene['pedestrianList']), 1)
        self.assertEqual(len(actor['waypointEventList']), 5)
        self.assertEqual(actor['speed'], 4.32)
        self.assertTrue(actor['active'])
        self.assertEqual(actor['activeDistance'], 1000.)
        self.assertEqual([manual.gen.geo.waypoint_to_world(n['pos']) for n in
                          scene['waypointDataList'][0]['waypointData']], points)

    def test_install_preflight_preserves_existing_and_creates_nothing(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source = root / 'source.json'; source.write_text('new')
            old = root / 'existing.json'; old.write_text('user')
            new = root / 'new.json'
            with self.assertRaises(FileExistsError):
                manual.install_files([(source, new), (source, old)])
            self.assertFalse(new.exists())
            self.assertEqual(old.read_text(), 'user')

    def test_install_sidecar_and_scenario_exact_bytes(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source = root / 'source.json'; source.write_bytes(b'{"test": 1}\n')
            destinations = [root/'runtime/0_1.json', root/'runtime/EgoNetwork/0_1_MN.json']
            manual.install_files([(source, dst) for dst in destinations])
            self.assertTrue(all(dst.read_bytes() == source.read_bytes() for dst in destinations))


if __name__ == '__main__':
    unittest.main()
