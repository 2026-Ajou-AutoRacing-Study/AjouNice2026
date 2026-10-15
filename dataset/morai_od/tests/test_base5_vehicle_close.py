import copy
import sys
import unittest
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import generate_base5_independent as gen


class VehicleCloseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=gen.geo.load_json(gen.CONFIG)
        cls.ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)

    def test_only_near_mission_cells_changed(self):
        before=copy.deepcopy(self.config);before.pop('mission_vehicle_close')
        old={r['index']:r for r in gen.specs(before)}
        changed=[]
        for row in gen.specs(self.config):
            if row.get('vehicle_close_phase'):
                changed.append(row)
                self.assertTrue(row['mission'])
                self.assertLessEqual(old[row['index']]['distance_bounds'][1],20)
                self.assertEqual(row['asset_index'],old[row['index']]['asset_index'])
                self.assertEqual(row['actors'],old[row['index']]['actors'])
                self.assertEqual(row['topology'],old[row['index']]['topology'])
            else:self.assertEqual(row,old[row['index']])
        self.assertEqual(Counter(r['family'] for r in changed),dict(highway=66,roundabout=67))

    def test_every_phase_stays_on_real_npc_path(self):
        rows=gen.specs(self.config)
        for family,phase in {(r['family'],r['vehicle_close_phase']) for r in rows if r.get('vehicle_close_phase')}:
            row=next(r for r in rows if r['family']==family and r.get('vehicle_close_phase')==phase)
            for attempt in range(512):
                try:scene,meta=gen.build(row,self.config,self.ctx,attempt);break
                except ValueError:continue
            else:self.fail('Unfillable vehicle close phase')
            actor=meta['runtime_actors'][0];proof=actor['vehicle_close_observation']
            self.assertFalse(proof['npc_jitter_applied'])
            self.assertGreaterEqual(actor['planned_surface_clearance_m'],.3)
            self.assertLessEqual(actor['planned_surface_clearance_m'],2.5)
            if family=='highway':
                placement=actor['sample_highway_placement']
                corridor=next(c for c in self.ctx['sample_corridors'] if c['spawn_uid']==placement['spawn_uid'])
                self.assertTrue(0<=proof['reference_arc_m']<=corridor['arc'][-1])
            else:corridor=self.ctx['roundabout_close_route']
            expected=gen.geo.continuous_route_pose_at_arc(corridor['route'],corridor['arc'],proof['reference_arc_m'])
            actual=next(a for a in scene['vehicleList'] if a['UNIQUEID']==actor['unique_id'])['initPosition']['pos']
            self.assertAlmostEqual(actual['x'],expected['x'])
            self.assertAlmostEqual(actual['y'],expected['y'])
            yaw=float(next(a for a in scene['vehicleList'] if a['UNIQUEID']==actor['unique_id'])['initPosition']['rot']['yaw'])
            import math
            e=meta['ego'];angle=math.radians(e['yaw'])
            ego_poly=gen.rectangle([e['x']+1.35*math.cos(angle),e['y']+1.35*math.sin(angle)],[4.8,2.],e['yaw'])
            gap=ego_poly.distance(gen.rectangle(actor['geometric_center_estimate'],actor['estimated_size'],yaw))
            self.assertAlmostEqual(gap,proof['planned_surface_gap_m'],places=8)


if __name__=='__main__':unittest.main()
