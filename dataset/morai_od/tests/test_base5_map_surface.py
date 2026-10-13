import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_map_surface import MapSurface,configure,initial_pose,ego_road_support


class MapSurfaceTest(unittest.TestCase):
    def surface(self):
        return MapSurface({'road':dict(points=[[0,0,10],[10,0,11]],lane_mark_left=['line']),
            'virtual':dict(points=[[0,1,50],[10,1,50]],lazy_init=True)},
            {'line':dict(points=[[0,2,10.2],[10,2,11.2]])})

    def test_original_segment_and_associated_cross_slope(self):
        s=self.surface();r=s.estimate([5,1])
        self.assertEqual(r['link_id'],'road');self.assertAlmostEqual(r['height_m'],10.6)
        self.assertEqual(r['confidence'],'road_strip');self.assertFalse(r['contact_verified'])

    def test_virtual_lane_change_is_never_terrain(self):
        s=MapSurface({'virtual':dict(points=[[0,0,99],[10,0,99]],lane_ch_link_path=['a','b'])})
        self.assertFalse(s.estimate([5,0])['usable'])

    def test_ambiguous_levels_and_offroad_are_not_certified(self):
        s=MapSurface({'a':dict(points=[[0,0,10],[10,0,10]]),'b':dict(points=[[0,0,20],[10,0,20]])})
        self.assertEqual(s.estimate([5,0])['reason'],'ambiguous_road_levels')
        self.assertFalse(self.surface().estimate([5,20])['usable'])

    def test_final_xy_not_ego_height_and_asset_source_local_offset(self):
        ctx={'links':{'road':dict(points=[[0,0,10],[10,0,11]])},'static_height_reference_sources':{}}
        ctx=configure(ctx,{'enabled':True})
        asset=dict(data_id=1,source_list='objectList',source_file='test',template=dict(pos=dict(x=0,y=0,z=10.2)))
        result,init,ground=initial_pose(dict(x=5,y=0,z=99,yaw=0),asset,[1,1,1],[0,0,.5],ctx)
        self.assertAlmostEqual(result['z'],10.7);self.assertAlmostEqual(init['pivot_height_candidate_m'],.2)
        self.assertFalse(ground['literal_contact_verified'])

    def test_legacy_opt_out_unchanged(self):
        ctx={'links':{'road':dict(points=[[0,0,10],[10,0,11]])}}
        self.assertIs(configure(ctx,{}),ctx)

    def test_tied_boundary_proof_is_deterministic(self):
        row=dict(points=[[0,2,10.2],[10,2,11.2]])
        s=MapSurface({'road':dict(points=[[0,0,10],[10,0,11]],lane_mark_left=['z','a'])}, {'z':row,'a':row})
        self.assertEqual(s.estimate([5,1])['boundary']['boundary_id'],'a')

    def test_ego_rejects_roadside_height_even_when_actor_height_is_usable(self):
        s=self.surface();pose=dict(x=5.,y=4.,z=10.,yaw=0.,link_idx='road')
        self.assertTrue(s.estimate([5,4])['usable'])
        self.assertFalse(ego_road_support(s,pose)['road_supported'])

    def test_ego_uses_final_xy_and_oriented_footprint_not_original_link_anchor(self):
        s=MapSurface({'road':dict(points=[[0,0,10],[30,0,10]],width_start=3.5,width_end=3.5)})
        pose=dict(x=10.,y=0.,z=99.,yaw=0.,link_idx='road')
        proof=ego_road_support(s,pose)
        self.assertTrue(proof['road_supported']);self.assertEqual(proof['height_m'],10.)
        self.assertFalse(proof['contact_verified'])
        self.assertFalse(ego_road_support(s,dict(pose,y=8.))['road_supported'])
        self.assertFalse(ego_road_support(s,dict(pose,yaw=45.))['road_supported'])
        self.assertEqual(ego_road_support(s,dict(pose,x='10.0',y='0.0',yaw='0.0')),proof)
        with self.assertRaises(ValueError):ego_road_support(s,dict(pose,yaw='nan'))

    def test_unsupported_pair_translation_preserves_close_view_and_gap(self):
        from base5_avoidance_geometry import road_supported_pair
        s=MapSurface({'road':dict(points=[[0,0,10],[30,0,10]])})
        support=dict(x=10.,y=0.,z=10.,yaw=0.,link_idx='road')
        ego=dict(support,x=10.,y=6.,yaw=10.)
        target=dict(x=11.,y=3.,z=10.,yaw=90.)
        e,a,proof=road_supported_pair(ego,target,support,s)
        self.assertTrue(ego_road_support(s,e)['road_supported'])
        self.assertAlmostEqual(a['x']-e['x'],target['x']-ego['x'])
        self.assertAlmostEqual(a['y']-e['y'],target['y']-ego['y'])
        self.assertEqual((e['yaw'],a['yaw']),(ego['yaw'],target['yaw']))
        self.assertFalse(proof['object_ground_contact_verified'])
        self.assertEqual(road_supported_pair(e,a,support,s),(e,a,None))
        with self.assertRaises(ValueError):road_supported_pair(dict(ego,yaw=45.),target,support,s)


if __name__=='__main__':unittest.main()
