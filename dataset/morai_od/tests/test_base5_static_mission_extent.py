import copy
from collections import Counter
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_static_mission_extent as extent
import generate_base5_independent as gen
import prepare_base5_static_mission_extent as preparation


def source_metadata():
    result = {}
    for index in range(600):
        split = 'train' if index < 444 else 'val'
        phase = ('approach', 'entry', 'alongside', 'clearing')[index % 4]
        result['scene_'+str(index).zfill(4)] = dict(split=split,
            spec=dict(family='obstacle', mission=True, avoidance_phase=phase,
                seed=index+120, actors=1, yaw_limit=3, role='front',
                avoidance_gap_bounds_m=[.3, .5], motion='static'),
            runtime_actors=[dict(data_id=index % 122, unique_id=8500, name='Box',
                source_list='objectList', motion='static')])
    result['non_target'] = dict(split='train', spec=dict(family='obstacle', mission=False), runtime_actors=[])
    return result


class ExtentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)

    def setUp(self):
        self.settings = extent.configuration(self.ctx)
        self.config = {extent.CONFIG_KEY: self.settings}
        self.originals = source_metadata()

    def planned(self, name='after'):
        assigned = extent.assign(self.originals, self.config)
        ident = next(ident for ident, row in assigned.items() if row[extent.PLAN_KEY]['stratum'] == name)
        return ident, assigned[ident]

    def built_metadata(self, name='after'):
        ident, row = self.planned(name)
        _, _, base, details = extent.anchor(gen, self.ctx, row, self.config)
        actors = copy.deepcopy(self.originals[ident]['runtime_actors'])
        actors[0]['geometric_center_estimate'] = [base['x'], base['y']]
        return ident, dict(spec=row, split=row[extent.PLAN_KEY]['split'], base=base,
            details=details, runtime_actors=actors, ego=dict(base))

    def test_bounded_mgeo_geometry_not_safe_link_margin_helper(self):
        self.assertFalse(self.settings['traffic_signal_interaction_constraint'])
        self.assertFalse(self.settings['official_georeferenced_polygon'])
        intervals = self.settings['intervals_m']
        self.assertAlmostEqual(intervals['before']['train'][0][0], 250.)
        self.assertAlmostEqual(intervals['before']['train'][0][1], 316.156096924624)
        self.assertAlmostEqual(intervals['intersection']['val'][0][0], 316.156096924624)
        self.assertAlmostEqual(intervals['after']['val'][-1][-1], 390.8)
        self.assertLess(intervals['after']['train'][-1][1], 366.1)
        self.assertEqual(set(intervals['intersection']), {'val'})

    def test_configuration_rejects_unreviewed_range_and_quotas(self):
        for bounds in ((250, 427.85), (230, 390.8), (300, 250), (250, float('nan'))):
            with self.assertRaises(extent.ExtentContractError):
                extent.configuration(self.ctx, bounds=bounds)
        for counts in (dict(before=300, intersection=120, after=179),
                dict(before=300, intersection=120, after=True), dict(before=600)):
            with self.assertRaises(extent.ExtentContractError):
                extent.configuration(self.ctx, counts=counts)

    def test_assignment_is_deterministic_preserves_split_phase_asset_seed_and_input(self):
        snapshot = copy.deepcopy(self.originals)
        assigned = extent.assign(self.originals, self.config)
        self.assertEqual(assigned, extent.assign(dict(reversed(list(self.originals.items()))), self.config))
        self.assertEqual(self.originals, snapshot)
        self.assertEqual(Counter(row[extent.PLAN_KEY]['stratum'] for row in assigned.values()),
            Counter(before=300, intersection=120, after=180))
        self.assertEqual(Counter((row[extent.PLAN_KEY]['stratum'], row[extent.PLAN_KEY]['split'])
            for row in assigned.values()), Counter({('before', 'train'):300, ('intersection', 'val'):120,
                ('after', 'train'):144, ('after', 'val'):36}))
        for ident, row in assigned.items():
            source = self.originals[ident]
            actual = copy.deepcopy(source)
            actual['spec'] = row
            self.assertTrue(extent.verify_transition(source, actual, row))
            for key in source['spec']:
                self.assertEqual(row[key], source['spec'][key])
        self.assertNotIn('non_target', assigned)

    def test_inside_180_is_impossible_without_changing_split(self):
        config = {extent.CONFIG_KEY: extent.configuration(self.ctx,
            dict(before=240, intersection=180, after=180))}
        with self.assertRaisesRegex(extent.ExtentContractError, 'train/val'):
            extent.assign(self.originals, config)

    def test_partial_or_already_allocated_source_is_rejected(self):
        del self.originals['scene_0001']
        with self.assertRaises(extent.ExtentContractError):
            extent.assign(self.originals, self.config)
        self.originals = source_metadata()
        self.originals['scene_0001']['spec'][extent.PLAN_KEY] = {}
        with self.assertRaises(extent.ExtentContractError):
            extent.assign(self.originals, self.config)

    def test_plan_rejects_config_split_identity_and_stratum_tampering(self):
        _, row = self.planned()
        for key, value in (('split', 'foreign'), ('stratum', 'before'),
                ('source_spec_sha256', 'short'), ('source_identity', [])):
            changed = copy.deepcopy(row)
            changed[extent.PLAN_KEY][key] = value
            with self.assertRaises(extent.ExtentContractError):
                extent.validate_plan(changed, self.config)
        config = copy.deepcopy(self.config)
        config[extent.CONFIG_KEY]['counts']['after'] += 1
        with self.assertRaises(extent.ExtentContractError):
            extent.validate_plan(row, config)

    def test_retry_anchors_are_continuous_but_keep_stratum_and_split(self):
        for name in extent.STRATA:
            _, row = self.planned(name)
            seen = set()
            for retry in (0, 1, 63, 64, 255, 256, 4096):
                _, _, pose, details = extent.anchor(gen, self.ctx, row, self.config, retry)
                plan = row[extent.PLAN_KEY]
                self.assertTrue(any(a < pose['route_arc_m'] < b for a, b in plan['intervals_m']))
                self.assertEqual(extent.split_for(pose['link_idx']), plan['split'])
                self.assertEqual(details['static_mission_extent']['stratum'], name)
                seen.add(pose['route_arc_m'])
            self.assertEqual(len(seen), 7)

    def test_route_and_retry_contract_tampering_is_fatal(self):
        _, row = self.planned()
        ctx = dict(self.ctx, route=copy.deepcopy(self.ctx['route']))
        ctx['route'][0]['z'] += .001
        with self.assertRaises(extent.ExtentContractError):
            extent.anchor(gen, ctx, row, self.config)
        ctx = dict(self.ctx, arc=list(self.ctx['arc']))
        ctx['arc'][100] += .001
        with self.assertRaises(extent.ExtentContractError):
            extent.anchor(gen, ctx, row, self.config)
        for retry in (-1, True, 1.5):
            with self.assertRaises(extent.ExtentContractError):
                extent.anchor(gen, self.ctx, row, self.config, retry)

    def test_coverage_uses_actual_primary_not_ego_or_target_base(self):
        ident, meta = self.built_metadata('after')
        ego = gen.geo.continuous_route_pose_at_arc(self.ctx['route'], self.ctx['arc'], 210.)
        meta['ego'] = ego
        finished = extent.finish(gen, self.ctx, meta, self.config)
        proof = finished[extent.PLAN_KEY]
        self.assertTrue(proof['primary_center_not_ego_used_for_coverage'])
        self.assertAlmostEqual(proof['final_ego_route']['arc_m'], 210.)
        self.assertGreater(proof['primary_center_route']['arc_m'], 346.)
        before = gen.geo.continuous_route_pose_at_arc(self.ctx['route'], self.ctx['arc'], 290.)
        meta['runtime_actors'][0]['geometric_center_estimate'] = [before['x'], before['y']]
        with self.assertRaisesRegex(ValueError, 'Primary center left'):
            extent.finish(gen, self.ctx, meta, self.config)

    def test_finish_rejects_actor_and_split_changes(self):
        _, meta = self.built_metadata()
        changed = copy.deepcopy(meta)
        changed['runtime_actors'][0]['data_id'] += 1
        with self.assertRaises(extent.ExtentContractError):
            extent.finish(gen, self.ctx, changed, self.config)
        changed = copy.deepcopy(meta)
        changed['split'] = 'other'
        with self.assertRaises(extent.ExtentContractError):
            extent.finish(gen, self.ctx, changed, self.config)

    def test_outside_primary_is_not_clamped_to_coverage_boundary(self):
        _, meta = self.built_metadata('before')
        outside = gen.geo.continuous_route_pose_at_arc(self.ctx['route'], self.ctx['arc'], 248.)
        meta['runtime_actors'][0]['geometric_center_estimate'] = [outside['x'], outside['y']]
        with self.assertRaisesRegex(ValueError, 'Primary center left'):
            extent.finish(gen, self.ctx, meta, self.config)

    def test_extent_contract_error_is_not_an_ordinary_geometry_deficit(self):
        self.assertTrue(issubclass(extent.ExtentContractError, RuntimeError))
        self.assertFalse(issubclass(extent.ExtentContractError, ValueError))

    def test_geometry_density_counts_primary_and_actual_ego_separately(self):
        rows = []
        for name in extent.STRATA:
            _, meta = self.built_metadata(name)
            meta['runtime_actors'][0]['name'] = 'Box'
            extent.finish(gen, self.ctx, meta, self.config)
            rows.append(meta)
        result = preparation.density(rows)
        self.assertEqual(set(result), set(extent.STRATA))
        for row in result.values():
            self.assertEqual(row['frames'], 1)
            self.assertEqual(row['bins']['2m']['maximum_frames_in_bin'], 1)
            self.assertEqual(row['assets'], {'Box':1})

    def test_prepare_refuses_existing_or_runtime_outputs_before_reading_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FileExistsError):
                preparation.prepare(Path(tmp)/'missing', tmp)
        with self.assertRaisesRegex(ValueError, 'runtime'):
            preparation.prepare('/missing', gen.geo.DEFAULT_RUNTIME_ASSET_DIR/'new_static_extent')


if __name__ == '__main__':
    unittest.main()
