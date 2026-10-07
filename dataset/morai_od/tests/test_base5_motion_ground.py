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
