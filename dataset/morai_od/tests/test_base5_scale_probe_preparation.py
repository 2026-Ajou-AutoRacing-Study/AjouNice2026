"""CPU probe registration and bound guards, using a synthetic safe map context."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_scale_probe as prep


class ScaleProbePreparationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value) + '\n')
        return path

    def fixture(self):
        source_dir = self.root / 'preserved'
        scene = self.write(source_dir / 'scene.json', dict(egoVehicle=dict(initPosition={}), objectList=[]))
        meta = self.write(source_dir / 'meta.json', dict(spec=dict(family='obstacle', motion='static'),
            ego=dict(x=0., y=0., z=0., yaw=0.),
            base=dict(yaw=0.), split='train', split_group='source-group',
            runtime_actors=[dict(position=dict(x=-15., y=0., z=0.),
                map_ground_initializer=dict(preferred_link='link'))]))
        source = self.write(self.root / 'source.json', dict(config=dict(capture=dict(
            static_ground_reference=dict(enabled=True)), placement_ground=dict(enabled=True)),
            scenes=[dict(id='sunny13_sunny11_b5v2_03942', scenario=str(scene),
                metadata=str(meta), scenario_sha256=prep.sha256(scene))]))
        assets = [dict(name=name, data_id=ident, source_list='objectList', semantic_class='obstacle',
                       template=dict(DataID=ident)) for name, (ident, _) in prep.WHITELIST.items()]
        context = dict(links={}, pools=dict(obstacle=assets))
        return source, context

    def mocks(self, context):
        def initial_pose(pose, *args):
            return dict(pose, z=.1), dict(policy='test-scale1-reference'), dict(preferred_link='link')
        def set_pose(actor, pose, uid, asset, scene, motion):
            actor.update(UNIQUEID=uid, pos={k:pose[k] for k in 'xyz'}, yaw=pose['yaw'])
            scene['objectList'].append(actor)
        return [patch.object(prep.gen, 'context', return_value=context),
            patch.object(prep, 'prewarm', side_effect=lambda ctx, cfg: ctx),
            patch.object(prep, 'configure', side_effect=lambda ctx, cfg: ctx),
            patch.object(prep, 'MapSurface', return_value=object()),
            patch.object(prep, 'ego_road_support', return_value=dict(road_supported=True, height_m=0.)),
            patch.object(prep, 'initial_pose', side_effect=initial_pose),
            patch.object(prep, 'saved_height_candidate', return_value=dict(usable=True,
                pivot_height_candidate_m=.05, policy='test-saved-height')),
            patch.object(prep.gen, 'geometry', return_value=([.8, 1.2, .5], [0., 0., .2], True)),
            patch.object(prep.gen.geo, 'clear_actor_lists', side_effect=lambda s: s.update(objectList=[])),
            patch.object(prep.gen.geo, 'set_ego_pose', side_effect=lambda ego, pose, **kw: ego.update(pose)),
            patch.object(prep.gen, 'set_pose', side_effect=set_pose)]

    def run_prepare(self, assets, **kwargs):
        source, context = self.fixture()
        old = {p: p.read_bytes() for p in self.root.rglob('*.json')}
        patches = self.mocks(context)
        for item in patches:
            item.start()
            self.addCleanup(item.stop)
        output = self.root / 'new'
        manifest = prep.prepare(source, output, assets, **kwargs)
        self.assertEqual(old, {p: p.read_bytes() for p in old})
        return json.loads(manifest.read_text())

    def test_axis12_has_explicit_yaw_scale_pair_metadata_and_preserves_source(self):
        deck = self.run_prepare(['CargoBox'], axis=True)
        self.assertEqual(deck['count'], 12)
        self.assertFalse(deck['scale_probe_registration']['production_quota_claim'])
        self.assertFalse(deck['scale_probe_registration']['native_geometry_qualified'])
        self.assertNotIn('static_ground_reference', deck['config']['capture'])
        scenes = [json.loads(Path(e['scenario']).read_text()) for e in deck['scenes']]
        metas = [json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']]
        self.assertTrue(all(m['ego']['yaw'] == 180. for m in metas))
        self.assertEqual({m['scale_probe']['requested_yaw_deg'] for m in metas}, {0., 90.})
        self.assertEqual(sum(m['scale_probe']['condition'] == 'control' for m in metas), 2)
        self.assertEqual(sum(m['scale_probe']['requested_scale'] == [2., 3., 2.] for m in metas), 2)
        self.assertEqual({tuple(s['objectList'][0]['pos'][k] for k in ('x', 'y')) for s in scenes}, {(-18., 0.)})
        self.assertTrue(all(m['diagnostic_provenance']['source_metadata_sha256'] for m in metas))
        self.assertTrue(all(m['diagnostic_provenance']['source_files'] for m in metas))

    def test_barriers_use_length_only_and_drums_use_isotropic_candidates(self):
        deck = self.run_prepare(['Steel_Barricade', 'RedBarrel'], yaws=[0.])
        self.assertEqual(deck['count'], 6)
        metas = [json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']]
        barrier = [m['scale_probe']['requested_scale'] for m in metas if m['scale_probe']['asset'] == 'Steel_Barricade']
        drum = [m['scale_probe']['requested_scale'] for m in metas if m['scale_probe']['asset'] == 'RedBarrel']
        self.assertEqual(barrier, [[1., 1., 1.], [1.2, 1., 1.], [1.5, 1., 1.]])
        self.assertEqual(drum, [[1., 1., 1.], [1.2, 1.2, 1.2], [1.5, 1.5, 1.5]])

    def test_requested_six_metre_distance_keeps_fixed_xy_and_records_map_initializer(self):
        deck = self.run_prepare(['CargoBox'], distance_m=6., yaws=[0.])
        metas = [json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']]
        self.assertTrue(all(m['scale_probe']['requested_pivot_distance_m'] == 6. for m in metas))
        self.assertTrue(all(m['scale_probe']['probe_pivot_xy_m'] == [-6., 0.] for m in metas))
        self.assertTrue(all(m['runtime_actors'][0]['map_ground_initializer']['preferred_link'] == 'link' for m in metas))
        self.assertTrue(all(m['runtime_actors'][0]['planned_surface_clearance_m'] >= .3 for m in metas))

    def test_unit_reference_changes_initial_geometry_preserving_saved_height_proxy(self):
        reference = self.write(self.root / 'unit_reference.json', dict(assets=dict(CargoBox=dict(
            bbox_size_m=[.6, .8, .4], pivot_to_bbox_center_local_m=[.01, .02, 9.], data_id=40100001))))
        deck = self.run_prepare(['CargoBox'], unit_reference=reference, yaws=[0.], factors=[1.5])
        metas = [json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']]
        self.assertEqual(metas[0]['runtime_actors'][0]['estimated_size'], [.6, .8, .4])
        np.testing.assert_allclose(metas[1]['runtime_actors'][0]['estimated_size'], [.9, 1.2, .6])
        proof = metas[0]['runtime_actors'][0]['probe_unit_geometry_override']
        np.testing.assert_allclose(proof['applied_unit_pivot_proxy_m'], [.01, .02, .15])
        self.assertEqual(proof['saved_pivot_height_proxy_m'], .05)
        self.assertTrue(proof['pose_height_policy_preserved'])
        self.assertFalse(proof['production_qualification'])
        self.assertEqual(deck['scale_probe_registration']['unit_reference']['sha256'], prep.sha256(reference))
        self.assertEqual(metas[0]['scale_probe']['unit_reference']['sha256'], prep.sha256(reference))
        self.assertAlmostEqual(metas[1]['scale_probe']['pivot_height_delta_m'], .025)

    def test_bad_unit_reference_identity_or_geometry_fails_before_output(self):
        for name, record in [('OBJ_G80', dict(bbox_size_m=[1., 1., 1.], pivot_to_bbox_center_local_m=[0., 0., 0.])),
            ('CargoBox', dict(bbox_size_m=[-1., 1., 1.], pivot_to_bbox_center_local_m=[0., 0., 0.])),
            ('CargoBox', dict(bbox_size_m=[1., 1., 1.], pivot_to_bbox_center_local_m=[0., 0., float('nan')])),
            ('CargoBox', dict(bbox_size_m=[1., 1., 1.], pivot_to_bbox_center_local_m=[0., 0., 0.], data_id=123))]:
            reference = self.write(self.root / 'bad_unit.json', dict(assets={name:record}))
            with self.subTest(name=name, record=record), self.assertRaises(ValueError):
                prep.load_unit_reference(reference)

    def test_invalid_inputs_fail_before_reading_or_writing_source(self):
        cases = [dict(assets=['OBJ_G80']), dict(assets=['Bollard01']),
            dict(assets=['CargoBox', 'CargoBox']), dict(assets=[]),
            dict(assets=['WoodBox'], axis=True), dict(assets=['CargoBox'], factors=[float('inf')]),
            dict(assets=['CargoBox'], factors=[float('nan')]), dict(assets=['CargoBox'], factors=[-1.]),
            dict(assets=['CargoBox'], factors=[1.]), dict(assets=['CargoBox'], factors=[True]),
            dict(assets=['CargoBox'], yaws=[float('nan')]), dict(assets=['CargoBox'], yaws=[0., 360.]),
            dict(assets=list(prep.WHITELIST)), dict(assets=['CargoBox'], factors=[1.251, 1.254]),
            dict(assets=['CargoBox'], distance_m=float('nan')), dict(assets=['CargoBox'], distance_m=5.9),
            dict(assets=['CargoBox'], distance_m=25.1)]
        for args in cases:
            with self.subTest(args=args), self.assertRaises(ValueError):
                prep.prepare(self.root / 'absent.json', self.root / 'unwritten', **args)
        self.assertFalse((self.root / 'unwritten').exists())

    def test_catalog_name_id_semantic_container_are_not_interchangeable(self):
        source, context = self.fixture()
        context['pools']['obstacle'][0]['semantic_class'] = 'vehicle'
        for item in self.mocks(context):
            item.start()
            self.addCleanup(item.stop)
        with self.assertRaisesRegex(ValueError, 'identity/class/container'):
            prep.prepare(source, self.root / 'bad', ['CargoBox'])
        self.assertFalse((self.root / 'bad').exists())


if __name__ == '__main__':
    unittest.main()
