import copy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_motion_ground as ground
import generate_base5_independent as gen


class MotionGroundTest(unittest.TestCase):
    def test_zero_offset_terrain_preserves_exact_ego_viewpoint(self):
        import prepare_base5_empty_ground_probe as probe
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);ego=dict(x=3.,y=.7,z=28.,yaw=13.)
            meta=dict(ego=ego,base=dict(x=3.,y=0.,z=28.,route_arc_m=3.),runtime_actors=[],spec={})
            ground.atomic_json(root/'meta.json',meta);ground.atomic_json(root/'scene.json',dict(egoVehicle={}))
            ground.atomic_json(root/'manifest.json',dict(scenes=[dict(id='a',metadata=str(root/'meta.json'),
                scenario=str(root/'scene.json'),scenario_sha256='hash')]))
            with patch.object(probe.gen,'context',return_value=dict(route=[],arc=[])), \
                 patch.object(probe.gen.geo,'continuous_route_pose_at_arc',return_value=dict(x=3.,y=0.)), \
                 patch.object(probe.gen.geo,'clear_actor_lists'),patch.object(probe.gen.geo,'set_ego_pose') as set_pose:
                out=probe.prepare(root/'manifest.json',[0],root/'out',backward_route_m=0.)
            self.assertEqual(set_pose.call_args.args[1],ego)
            result=gen.geo.load_json(Path(gen.geo.load_json(out)['scenes'][0]['metadata']))
            self.assertEqual(result['ground_probe_source']['placement_policy'],'source-ego-exact')

    def test_world_ground_cloud_excludes_ego_mesh_and_zero_returns(self):
        from base5_ground_probe import world_cloud
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'LIDAR_5').mkdir()
            points=np.array([[0,0,0,1],[1,0,-1,1],[5,0,-1,1],[0,2,-1,1]],dtype=np.float32)
            points.tofile(root/'LIDAR_5/test_Intensity.bin')
            output=world_cloud(root,dict(lidar_to_ego=np.eye(4)),
                dict(ego_before=dict(x=0.,y=0.,z=0.,heading=1.347)))
            np.testing.assert_allclose(output,points[2:,:3])

    def test_near_empty_probe_observes_motion_target_not_departure(self):
        import prepare_base5_empty_ground_probe as probe
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);target=dict(x=8.,y=2.,z=28.)
            actor=dict(unique_id=1,position=dict(x=-5.,y=2.,z=28.),motion_path=dict(target_position=target))
            meta=dict(runtime_actors=[actor],spec={},ground_support_check=dict(required_uids=[1]))
            ground.atomic_json(root/'meta.json',meta);ground.atomic_json(root/'scene.json',dict(egoVehicle={}))
            ground.atomic_json(root/'manifest.json',dict(scenes=[dict(id='a',metadata=str(root/'meta.json'),
                scenario=str(root/'scene.json'),scenario_sha256=ground.sha256(root/'scene.json'))]))
            with patch.object(probe.gen,'context',return_value=dict(links={})), \
                 patch.object(probe,'target_probe_pose',return_value=dict(x=2.,y=2.,z=28.,yaw=0.)) as pose, \
                 patch.object(probe.gen.geo,'clear_actor_lists'),patch.object(probe.gen.geo,'set_ego_pose'):
                manifest=probe.prepare(root/'manifest.json',[0],root/'out',target_uid=1,target_motion=True)
            self.assertEqual(pose.call_args.args[1]['position'],target)
            row=gen.geo.load_json(manifest)['scenes'][0];output=gen.geo.load_json(Path(row['metadata']))
            self.assertTrue(output['calibration_only']);self.assertEqual(output['runtime_actors'],[])
            self.assertEqual(output['ground_probe_source']['targets'],[actor])
            self.assertEqual(gen.geo.load_json(root/'meta.json'),meta)

    def test_additional_motion_terrain_is_bounded(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            ground.atomic_json(root/'scene.json',{})
            ground.atomic_json(root/'meta.json',{})
            ground.atomic_json(root/'manifest.json',dict(scenes=[dict(id='a',metadata=str(root/'meta.json'),scenario=str(root/'scene.json'))]))
            with self.assertRaisesRegex(ValueError,'nine sessions'):
                ground.prepare(root/'manifest.json',root/'t',root/'r',root/'out',additional_terrain=[root/'t']*9)

    def test_near_probes_support_population_of_eight_without_waiving_ground(self):
        actors=[dict(unique_id=i,motion='walking') for i in range(8)]
        self.assertEqual(ground.near_probe_actors(dict(runtime_actors=actors)),actors)
        with self.assertRaisesRegex(ValueError,'eight actors'):
            ground.near_probe_actors(dict(runtime_actors=actors+[dict(unique_id=9,motion='walking')]))
        with self.assertRaisesRegex(ValueError,'Duplicate'):
            ground.near_probe_actors(dict(runtime_actors=actors[:1]*2))

    def test_mixed_motion_near_probes_include_static_secondaries(self):
        actors=[dict(unique_id=1,motion='walking'),dict(unique_id=2,motion='static')]
        self.assertEqual(ground.near_probe_actors(dict(runtime_actors=actors)),actors)

    def test_mixed_preparation_measures_static_secondary_and_changes_input_only(self):
        for static_z in (.5,1.):
            with self.subTest(static_z=static_z),tempfile.TemporaryDirectory() as d:
                root=Path(d);target=dict(x=8.,y=0.,z=.5)
                path=dict(target_position=target,points=[dict(x=6.,y=0.,z=.5),dict(x=10.,y=0.,z=.5)])
                records=[dict(unique_id=1,name='Woman2',source_list='pedestrianList',motion='walking',
                    position=copy.deepcopy(path['points'][0]),motion_path=path),
                    dict(unique_id=2,name='CargoBox',source_list='objectList',motion='static',
                        position=dict(x=12.,y=0.,z=static_z))]
                scene=dict(pedestrianList=[dict(UNIQUEID=1,rot=gen.geo.formatted_rotation(0.))],
                    objectList=[dict(UNIQUEID=2,rot=gen.geo.formatted_rotation(0.))],
                    waypointDataList=[dict(pathName='PedPath - 1')])
                ground.atomic_json(root/'scene.json',scene)
                ground.atomic_json(root/'meta.json',dict(runtime_actors=records,spec={}))
                row=dict(id='a',metadata=str(root/'meta.json'),scenario=str(root/'scene.json'),
                    scenario_sha256=ground.sha256(root/'scene.json'))
                ground.atomic_json(root/'manifest.json',dict(scenes=[row]))
                for name in ('terrain','reference'):
                    session=root/name;capture=session/'capture';(capture/'LIDAR_5').mkdir(parents=True)
                    ground.atomic_json(session/'shutdown_safety.json',dict(park_verified=True))
                    (session/'ledger.jsonl').write_text(__import__('json').dumps(dict(status='review',capture_root=str(capture)))+'\n')
                    ground.atomic_json(capture/'audit.json',dict(review_reasons=['actor_ground_plane_unverified']))
                    metadata=dict(calibration_only=True,spec={},lidar_to_ego=np.eye(4).tolist())
                    if name=='terrain':metadata.update(runtime_actors=[],ground_probe_source=dict(scenario_sha256=row['scenario_sha256']))
                    ground.atomic_json(capture/'metadata.json',metadata)
                    ground.atomic_json(capture/'telemetry.json',dict(ego_before=dict(x=0.,y=0.,z=0.,heading=1.347)))
                    (capture/'LIDAR_5/test_Intensity.bin').write_bytes(b'raw')
                    (capture/'LIDAR_5/test_Instance.txt').write_text('raw annotation')
                x,y=np.meshgrid(np.linspace(0,20,61),np.linspace(-5,5,41));cloud=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
                boxes=[dict(unique_id=uid,center=np.array([x,0.,z]),size=np.ones(3),rotation_rpy=np.zeros(3))
                    for uid,x,z in ((1,8.,.5),(2,12.,static_z))]
                samples=[dict(x=6.,y=0.,z=0.),dict(x=10.,y=0.,z=0.)]
                with patch.object(ground,'world_cloud',return_value=cloud),patch.object(ground,'parse_instances',return_value=boxes), \
                     patch.object(ground,'supported_motion_direction',return_value=(samples,0.,0)):
                    out=ground.prepare(root/'manifest.json',root/'terrain',root/'reference',root/'out')
                output=gen.geo.load_json(Path(gen.geo.load_json(out)['scenes'][0]['metadata']))
                self.assertEqual([r['uid'] for r in output['ground_probe_references']],[1,2])
                self.assertEqual(output['ground_support_check']['required_uids'],[1,2])
                self.assertAlmostEqual(output['runtime_actors'][1]['position']['z'],.5 if static_z==.5 else .47)
                self.assertEqual(gen.geo.load_json(root/'meta.json')['runtime_actors'][1]['position']['z'],static_z)
                self.assertEqual((root/'reference/capture/LIDAR_5/test_Instance.txt').read_text(),'raw annotation')

    def test_collision_skip_requires_matching_hashed_peer_evidence(self):
        from base5_capture_safety import validate_collision_failure
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'collision.json'
            ground.atomic_json(p,dict(scenario_sha256='scene',non_ego_uids=[8500]))
            row=dict(status='failed',failure_kind='physical_collision',scenario_sha256='scene',
                failure_evidence=str(p),failure_evidence_sha256=ground.sha256(p))
            validate_collision_failure(row)
            row['status']='accepted_candidate'
            with self.assertRaises(ValueError):validate_collision_failure(row)
            row['status']='failed';p.write_text('{}')
            with self.assertRaises(ValueError):validate_collision_failure(row)

    def test_scene_collision_epoch_keeps_post_match_contacts(self):
        from base5_capture_safety import scene_collision_epoch
        def sample(t,x,peer=False):
            actors=[dict(unique_id=0,position=dict(x=x,y=0),heading=1.347)]
            if peer:actors.append(dict(unique_id=8500))
            return dict(t=t,message=dict(collision_object=actors,collision_objecta=[]))
        rows=[sample(1.,4.,True),sample(2.,0.),sample(3.,2.,True)]
        kept,epoch=scene_collision_epoch(rows,1.,dict(x=0,y=0,yaw=0))
        self.assertEqual(epoch['excluded_transition_messages'],1)
        self.assertEqual(kept,rows[1:]) # collision-induced drift is still a contact
        with self.assertRaises(RuntimeError):scene_collision_epoch(rows,1.,dict(x=20,y=0,yaw=0))

    def test_sloped_terrain_is_sampled_not_constant_z(self):
        x,y=np.meshgrid(np.linspace(-10,10,61),np.linspace(-5,5,41))
        cloud=np.c_[x.ravel(),y.ravel(),(28+.02*x-.01*y).ravel()]
        rows=ground.terrain_samples(cloud,dict(x=-3,y=0),dict(x=3,y=0))
        self.assertEqual(len(rows),13)
        self.assertAlmostEqual(rows[-1]['z']-rows[0]['z'],.12,places=6)

    def test_direction_changes_only_when_measured_path_requires_it(self):
        def samples(cloud,start,end):
            if abs(start['y'])>.01 or abs(end['y'])>.01:raise ValueError('No side-road terrain')
            return [dict(x=start['x'],y=start['y'],z=28.),dict(x=end['x'],y=end['y'],z=28.)]
        path=dict(target_position=dict(x=8.,y=0.,z=28.),
            points=[dict(x=8.,y=-6.,z=28.),dict(x=8.,y=3.,z=28.)])
        actor=dict(unique_id=1,estimated_size=[.5,.5,1.8],geometric_center_estimate=[8.,0.])
        meta=dict(ego=dict(x=0.,y=-10.,yaw=0.),runtime_actors=[actor])
        with patch.object(ground,'terrain_samples',side_effect=samples):
            rows,yaw,change=ground.supported_motion_direction([],path,actor,meta)
        self.assertEqual(change,90);self.assertAlmostEqual(yaw,180.)
        self.assertAlmostEqual(rows[0]['x'],14.)

    def test_missing_terrain_never_extrapolates(self):
        with self.assertRaises(ValueError):
            ground.terrain_samples(np.zeros((0,3)),dict(x=0,y=0),dict(x=2,y=0))

    def test_other_moving_actor_entire_corridor_is_reserved(self):
        path=dict(target_position=dict(x=8.,y=0.,z=28.),
            points=[dict(x=8.,y=-6.,z=28.),dict(x=8.,y=3.,z=28.)])
        actor=dict(unique_id=1,estimated_size=[.5,.5,1.8],geometric_center_estimate=[8.,0.])
        other=dict(unique_id=2,estimated_size=[.5,.5,1.8],geometric_center_estimate=[20.,0.],
            motion_path=dict(target_position=dict(x=20.,y=0.,z=28.),
                points=[dict(x=0.,y=0.,z=28.),dict(x=25.,y=0.,z=28.)]))
        meta=dict(ego=dict(x=0.,y=-10.,yaw=0.),runtime_actors=[actor,other])
        with patch.object(ground,'terrain_samples') as samples:
            with self.assertRaises(ValueError):ground.supported_motion_direction([],path,actor,meta)
        samples.assert_not_called()  # every candidate crosses the reserved corridor

    def test_terrain_step_is_not_smoothed_away(self):
        def surface(cloud,xy):
            return dict(coefficients=[0,0,0 if xy[0]<1 else .3],support_polygon_xy=[[-5,-5],[-5,5],[5,5],[5,-5]])
        with patch.object(ground,'surface_at',side_effect=surface):
            with self.assertRaisesRegex(ValueError,'step'):
                ground.terrain_samples([],dict(x=0,y=0),dict(x=2,y=0))

    def test_unverified_motion_refused_and_static_unaffected(self):
        ground.validate_motion_ground(dict(runtime_actors=[dict(motion='static')]),{})
        with self.assertRaisesRegex(ValueError,'independent ground'):
            ground.validate_motion_ground(dict(runtime_actors=[dict(name='Dog1',motion='walking',motion_path={})]),{})

    def test_ground_witness_detects_metadata_and_evidence_tampering(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'evidence';path.write_text('raw')
            points=[dict(x=0.,y=0.,z=28.),dict(x=1.,y=0.,z=28.02)]
            proof=dict(policy='independent-terrain-sampled-v46',files=[dict(path=str(path),sha256=ground.sha256(path))],
                samples=[dict(x=0.,y=0.,z=27.5),dict(x=1.,y=0.,z=27.52)],asset_pivot_offset_m=.5)
            actor=dict(name='Dog1',unique_id=1,motion='walking',position=copy.deepcopy(points[0]),
                motion_path=dict(points=points,ground_path_evidence=proof))
            scene=dict(waypointDataList=[dict(pathName='PedPath - 1',waypointData=[dict(pos=gen.geo.world_to_morai_waypoint(p)) for p in points])])
            meta=dict(runtime_actors=[actor]);ground.validate_motion_ground(meta,scene)
            actor['motion_path']['points'][1]['z']+=.1
            with self.assertRaisesRegex(ValueError,'ground height'):ground.validate_motion_ground(meta,scene)
            actor['motion_path']['points'][1]['z']-=.1
            path.write_text('changed')
            with self.assertRaisesRegex(ValueError,'checksum'):ground.validate_motion_ground(meta,scene)

    def test_unknown_obstacle_placement_proxy_not_tiny_box(self):
        size,offset,known=gen.geometry(dict(name='Bicycle_city_unseen',ontology_class='Obstacle'),dict(geometry={}))
        self.assertFalse(known)
        self.assertGreaterEqual(size[0],3.)
        self.assertGreaterEqual(size[1],3.)

    def test_static_reference_keeps_asset_target_and_is_calibration_only(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);sp=p/'scene.json';mp=p/'metadata.json';source=p/'manifest.json'
            actor=dict(UNIQUEID=1,pos=gen.geo.formatted_vector(0.,0.,28.),rot=gen.geo.formatted_rotation(90.),
                speed=7.92,isWaypointMode=True,waypointEventList=[{},{}])
            target=dict(x=3.,y=4.,z=28.2)
            meta=dict(runtime_actors=[dict(unique_id=1,name='Woman2',source_list='pedestrianList',motion='running',
                position=dict(x=0.,y=0.,z=28.),motion_path=dict(target_position=target))],spec=dict(motion='running'))
            ground.atomic_json(sp,dict(pedestrianList=[actor],waypointDataList=[dict(pathName='PedPath - 1')]))
            ground.atomic_json(mp,meta)
            ground.atomic_json(source,dict(scenes=[dict(id='a',scenario=str(sp),metadata=str(mp),scenario_sha256=ground.sha256(sp),motion='running')]))
            out=ground.static_reference(source,p/'reference');row=gen.geo.load_json(out)['scenes'][0]
            scene=gen.geo.load_json(Path(row['scenario']));m=gen.geo.load_json(Path(row['metadata']))
            self.assertTrue(m['calibration_only']);self.assertEqual(m['spec'],meta['spec'])
            self.assertEqual(scene['pedestrianList'][0]['speed'],0.)
            self.assertEqual(scene['pedestrianList'][0]['rot']['yaw'],'90.000')
            self.assertEqual(m['runtime_actors'][0]['position'],target)
            self.assertEqual(scene['waypointDataList'],[])

    def test_far_moving_actor_ground_cannot_be_skipped(self):
        from base5_ground_support import ground_review
        box=dict(unique_id=1,center=np.array([25.,0.,.5]),size=np.array([.4,.4,1.]),rotation_rpy=np.zeros(3))
        legacy=ground_review(np.zeros((0,3)),[box],np.eye(4))
        self.assertFalse(legacy['actors'][0]['required'])
        enforced=ground_review(np.zeros((0,3)),[box],np.eye(4),required_uids=[1])
        self.assertTrue(enforced['actors'][0]['required'])
        self.assertIn('actor_ground_plane_unverified',enforced['review_reasons'])

    def test_unknown_avoidance_cannot_request_twenty_cm_pass(self):
        import random
        route=[dict(x=float(i),y=0.,z=0.,yaw=0.,link_idx='test',point_idx=i) for i in range(201)]
        ego,pose,ref=gen.avoidance.observation(gen,dict(geometry={}),route,list(range(201)),
            dict(route[100],route_arc_m=100.),dict(name='unknown',ontology_class='Obstacle'),
            dict(family='obstacle',avoidance_phase='alongside',avoidance_side='left',avoidance_gap_bounds_m=[.2,.3]),random.Random(1))
        self.assertEqual(ref['requested_surface_gap_m'],1.)
        self.assertLess(ref['original_gap_request_m'],.3)


if __name__=='__main__':unittest.main()
