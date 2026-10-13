import math
import sys
import unittest
import tempfile
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import generate_base5_independent as gen


class DynamicObservationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = gen.geo.load_json(gen.CONFIG)
        cls.ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)

    def test_exact_phase_quota_and_front_priority(self):
        rows = [r for r in gen.specs(self.config) if r.get('dynamic_observation_phase')]
        self.assertEqual(len(rows), 250)
        self.assertEqual(Counter(r['dynamic_observation_phase'] for r in rows),
                         dict(approach=113, crossing=62, yielding=50, clearing=25))
        self.assertEqual(sum(r['role'] == 'rear' for r in rows), 25)

    def test_only_route_crossing_crosswalks(self):
        ref = self.ctx['mission_templates']['dynamic_obstacle'][0][1]
        candidates = gen.dynamic_crossing_candidates(self.ctx, ref, self.config['mission_dynamic_observation'])
        self.assertEqual({c['crosswalk_id'] for c in candidates}, {'CW000000', 'CW000002'})

    def test_each_phase_builds_fixed_map_target(self):
        rows = gen.specs(self.config)
        for phase in self.config['mission_dynamic_observation']['phases']:
            row = next(r for r in rows if r.get('dynamic_observation_phase') == phase)
            for attempt in range(512):
                try:
                    _, meta = gen.build(row, self.config, self.ctx, attempt)
                    break
                except ValueError:
                    continue
            else:
                self.fail('Unfillable phase ' + phase)
            obs = meta['details']['dynamic_observation']
            actor = meta['runtime_actors'][0]
            target = obs['target_center']
            self.assertLess(math.dist(actor['geometric_center_estimate'], [target['x'], target['y']]), 1e-6)
            self.assertEqual(actor['actual_ego_role'], 'rear' if phase == 'clearing' else 'front')
            delta = obs['ego_crossing_arc_displacement_m']
            self.assertGreater(delta, 0) if phase == 'clearing' else self.assertLess(delta, 0)
            if 'motion_path' in actor:
                points = actor['motion_path']['points']
                axis = obs['crossing']['axis']
                dx, dy = points[1]['x'] - points[0]['x'], points[1]['y'] - points[0]['y']
                self.assertLess(abs(dx * axis[1] - dy * axis[0]), 1e-6)

    def test_legacy_config_remains_unchanged(self):
        old = gen.geo.load_json(gen.CONFIG.with_name('base5_independent_v2.json'))
        self.assertFalse(any('dynamic_observation_phase' in r for r in gen.specs(old)))

    def test_offline_audit_checks_map_target_and_rejects_tamper(self):
        import json
        import audit_base5_scenarios as audit
        with tempfile.TemporaryDirectory() as tmp:
            manifest = gen.generate(self.config, Path(tmp), 23, 1)
            self.assertEqual(audit.audit(manifest)['errors'], [])
            entry = json.loads(manifest.read_text())['scenes'][0]
            path = Path(entry['metadata']);meta = json.loads(path.read_text())
            meta['details']['dynamic_observation']['target_center']['x'] += 2
            path.write_text(json.dumps(meta))
            self.assertTrue(any('dynamic primary left map target' in e for _, e in audit.audit(manifest)['errors']))


if __name__ == '__main__':
    unittest.main()
