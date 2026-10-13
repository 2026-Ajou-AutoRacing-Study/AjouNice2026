import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_base5_parked_npc_plan import audit
from base5_capture_safety import atomic_json, sha256
import base5_parked_npc_placement as placement


class NpcPlanAuditTests(unittest.TestCase):
    def fixture(self, root):
        spec = dict(family='shadow', actors=1, motion='static', role='front', distance_bounds=[10, 30],
                    static_yaw_policy=dict(scope='shadow_static', primary_mode='uniform'))
        ground = lambda confidence: dict(footprint=dict(all_supported=True,
            samples=[dict(usable=True, confidence=confidence, nearest_road_distance_m=.1)]*5))
        actor = dict(data_id=10, unique_id=8500, name='bus', source_list='vehicleList', motion='static',
                     estimated_size=[12, 3, 4], map_ground_initializer=ground('roadside_estimate'),
                     static_yaw_policy=dict(mode='uniform'))
        before = dict(spec=spec, runtime_actors=[actor], split='train', scene_group_id='group')
        old_scene = root / 'old_scene.json'; atomic_json(old_scene, dict(pose='original'))
        old_meta = root / 'old_meta.json'; atomic_json(old_meta, before)
        original = dict(id='a', scenario=str(old_scene), metadata=str(old_meta), scenario_sha256=sha256(old_scene))
        source = root / 'source.json'; atomic_json(source, dict(scenes=[original]))
        after = copy.deepcopy(before)
        override = dict(policy=placement.POLICY, mode='road_aligned', data_id=10, original_yaw=actor['static_yaw_policy'])
        after['spec'].update(frozen_population_data_ids=[10], parked_npc_yaw_overrides={'0': override})
        after['runtime_actors'][0].update(map_ground_initializer=ground('road_strip'), static_yaw_policy=dict(mode='road_aligned'))
        repaired = after['runtime_actors'][0]
        repaired['parked_npc_placement'] = placement.actor_screen(repaired, placement.DEFAULT, after['spec'])
        after['parked_npc_support_source'] = dict(source_id='a', source_scenario_sha256=sha256(old_scene),
            source_metadata_sha256=sha256(old_meta), original_risk_ordinals=[0], geometry_changed=True,
            explicit_yaw_overrides={'0': override})
        new_scene = root / 'new_scene.json'; atomic_json(new_scene, dict(pose='repaired'))
        new_meta = root / 'new_meta.json'; atomic_json(new_meta, after)
        frozen = root / 'frozen.json'; atomic_json(frozen, [after['spec']])
        candidate = root / 'candidate.json'
        atomic_json(candidate, dict(config=dict(parked_npc_placement=placement.DEFAULT,
            frozen_specs_path=str(frozen), frozen_specs_sha256=sha256(frozen)), scenes=[dict(original,
            scenario=str(new_scene), metadata=str(new_meta), scenario_sha256=sha256(new_scene))]))
        return source, candidate, new_meta

    def test_explicit_long_npc_delta_preserves_slot_contract(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, candidate, _ = self.fixture(Path(temporary))
            result = audit(source, candidate)
            self.assertEqual(result['errors'], []); self.assertEqual(result['changed_scenes'], 1)
            self.assertEqual(result['explicit_yaw_fallback_actors'], 1)

    def test_range_or_motion_change_is_not_authorized_by_footprint_repair(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, candidate, meta = self.fixture(Path(temporary))
            changed = json.loads(meta.read_text()); changed['spec']['distance_bounds'] = [3, 10]
            atomic_json(meta, changed)
            result = audit(source, candidate)
            self.assertTrue(any('outside NPC footprint delta' in e[1] for e in result['errors']))


if __name__ == '__main__': unittest.main()
