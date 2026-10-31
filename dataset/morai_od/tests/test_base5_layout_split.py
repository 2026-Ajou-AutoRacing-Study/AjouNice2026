"""Known-map split regressions. CPU/files only; no simulator, ROS or GPU."""
import copy
import json
from pathlib import Path
import pickle
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_layout_split as layout
import base5_spatial_diversity as spatial
import base5_static_mission_extent as extent
from base5_capture_safety import atomic_json, sha256
from build_base5_bevfusion_infos import convert
from dynamic_capture_metadata import normalized_sensor_calibration
from prepare_base5_layout_split import prepare


def record(index, asset=1, family='obstacle', actors=True):
    ego = dict(x=float(index), y=0., z=0., yaw=0., link_idx='shared-map-link')
    actor = dict(data_id=asset, unique_id=8500, name='Box', source_list='objectList',
        position=dict(x=float(index)+2, y=1., z=0.), placement_yaw_deg=0.,
        estimated_size=[1., 1., 1.], motion='static')
    return dict(id='scene_'+str(index).zfill(4), scenario_sha256='payload-'+str(index),
        meta=dict(ego=ego, base=dict(ego), split='train', runtime_actors=[actor] if actors else [],
            logical_quota_slot_id='same-quota-slot',
            spec=dict(family=family, mission=family!='background', role='front',
                distance_bin=index % 2, yaw_limit=(3, 10, 15)[index % 3],
                topology='route_general', avoidance_phase='approach')))


class LayoutTests(unittest.TestCase):
    def test_deterministic_stratified_known_map_split_and_train_floors(self):
        records = [record(i, asset=99 if i < 3 else 1) for i in range(100)]
        overlay = layout.allocate(records)
        self.assertEqual(overlay, layout.allocate(list(reversed(records))))
        audit = overlay['audit']
        self.assertEqual(audit['status'], 'CPU_PASS')
        self.assertEqual(audit['splits'], dict(train=85, val=15))
        self.assertEqual(audit['train_missing_assets'], [])
        self.assertEqual(audit['asset_independent_group_counts']['99'], dict(train=3))
        self.assertEqual(audit['final_ego_coverage']['train_missing_cells'], [])
        self.assertEqual(audit['final_ego_coverage']['val_links'], 1)
        self.assertEqual(audit['exact_geometry_cross_split'], 0)
        self.assertEqual(audit['near_geometry_cross_split'], 0)

    def test_exact_weather_and_near_layouts_group_but_not_equal_logical_slots(self):
        first, different = record(1), record(2)
        weather = copy.deepcopy(first);weather['id']='foggy-scene';weather['meta']['spec']['seed']=900
        near = copy.deepcopy(first);near['id']='near-scene'
        near['meta']['ego']['x'] += .02
        near['meta']['runtime_actors'][0]['position']['x'] += .02
        records = [first, different, weather, near]
        groups, hashes, pairs = layout.groups(records, layout.configuration())
        self.assertEqual(groups[first['id']], groups[weather['id']])
        self.assertEqual(groups[first['id']], groups[near['id']])
        self.assertNotEqual(groups[first['id']], groups[different['id']])
        self.assertEqual(hashes[first['id']], hashes[weather['id']])
        self.assertTrue(pairs)
        changed = copy.deepcopy(weather);changed['meta']['runtime_actors'][0]['estimated_size'][0]=2.
        groups, _, _ = layout.groups([first, changed], layout.configuration())
        self.assertNotEqual(groups[first['id']], groups[changed['id']])

    def test_declared_clip_is_indivisible_and_legacy_link_group_is_not(self):
        records = [record(i) for i in range(30)]
        for row in records:
            row['meta']['split_group'] = 'same-physical-link'
        records[0]['meta']['scene_group_id'] = records[-1]['meta']['scene_group_id'] = 'same-real-clip'
        overlay = layout.allocate(records)
        self.assertEqual(overlay['assignments'][records[0]['id']]['group_id'],
            overlay['assignments'][records[-1]['id']]['group_id'])
        self.assertEqual(overlay['assignments'][records[0]['id']]['split'],
            overlay['assignments'][records[-1]['id']]['split'])
        self.assertGreater(overlay['audit']['independent_layout_groups'], 1)

    def test_audit_detects_geometry_group_payload_and_frame_set_tampering(self):
        records = [record(i) for i in range(30)]
        overlay = layout.allocate(records)
        for key, value in (('scenario_sha256', 'wrong'), ('geometry_sha256', 'wrong'),
                           ('group_id', 'unbound'), ('split', 'test')):
            changed = copy.deepcopy(overlay['assignments']);changed[records[0]['id']][key]=value
            self.assertTrue(layout.audit_assignments(records, changed, overlay['policy'])['errors'])
        changed = copy.deepcopy(overlay['assignments']);changed.pop(records[0]['id'])
        self.assertTrue(layout.audit_assignments(records, changed, overlay['policy'])['errors'])

    def test_ratio_is_soft_when_every_observed_cell_is_a_singleton(self):
        records = [record(i*20) for i in range(10)]
        overlay = layout.allocate(records)
        self.assertEqual(overlay['audit']['splits'], dict(train=10))
        self.assertEqual(overlay['audit']['validation_shortfall'], 2)
        self.assertEqual(overlay['audit']['status'], 'CPU_PASS')

    def test_invalid_policy_and_nonfinite_geometry_fail_closed(self):
        self.assertFalse(layout.enabled({}))
        with self.assertRaises(ValueError):layout.enabled({layout.CONFIG_KEY: dict(policy='unknown')})
        with self.assertRaises(ValueError):layout.configuration(train_asset_floor=True)
        row = record(1);row['meta']['ego']['x']=float('nan')
        with self.assertRaises(ValueError):layout.geometry(row['meta'])

    def test_route_coverage_does_not_claim_unseen_geometry_was_created(self):
        records = [record(1)]
        overlay = layout.allocate(records)
        result = layout.route_coverage(records, overlay['assignments'],
            [dict(x=0, y=0, route_arc_m=0), dict(x=300, y=0, route_arc_m=300)])
        self.assertEqual(result['splits']['train']['samples_with_final_ego_within']['25m'], 1)
        self.assertEqual(len(result['splits']['train']['samples_without_final_ego_within_25m']), 1)


class GeneratorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ctx = spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR, spatial.gen.geo.DEFAULT_MGEO)
        from base5_map_surface import MapSurface
        cls.ctx['map_surface_index'] = MapSurface(cls.ctx['links'], cls.ctx.get('lane_boundaries'))

    def test_all_link_continuous_intervals_are_the_legacy_union(self):
        old = spatial.allowed_intervals(self.ctx, {})
        new = spatial.allowed_intervals(self.ctx, {layout.CONFIG_KEY: layout.configuration()})
        self.assertEqual(set(new), {('background', 'all'), ('other', 'all')})
        for scope in ('background', 'other'):
            length = lambda pieces: sum(b-a for a, b in pieces)
            self.assertAlmostEqual(length(new[(scope, 'all')]),
                sum(length(pieces) for (name, _), pieces in old.items() if name==scope))
        plan = dict(policy=spatial.POLICY, split='all', split_policy=layout.POLICY,
            scope='background', intervals_m=new[('background', 'all')], stratum_index=0, stratum_count=1)
        old_splits = {spatial.split_for(spatial.sample_anchor(self.ctx, plan, seed, 0)[2]['link_idx']) for seed in range(150)}
        self.assertEqual(old_splits, {'train', 'val'})
        bad = dict(plan, split_policy='unrecognized')
        with self.assertRaises(ValueError):spatial.sample_anchor(self.ctx, bad, 1, 0)

    def test_optin_extent_can_train_inside_without_changing_assets_phases_or_seed(self):
        originals = {}
        for i in range(600):
            row = record(i)
            row['meta']['split'] = 'train' if i < 444 else 'val'
            row['meta']['spec'].update(seed=i, actors=1, avoidance_phase=('approach','entry','alongside','clearing')[i % 4])
            originals[row['id']] = row['meta']
        config = {layout.CONFIG_KEY: layout.configuration(),
            extent.CONFIG_KEY: extent.configuration(self.ctx, dict(before=240, intersection=180, after=180),
                split_policy=layout.POLICY)}
        assigned = extent.assign(originals, config)
        for name in extent.STRATA:
            self.assertEqual(set(config[extent.CONFIG_KEY]['intervals_m'][name]), {'train','val'})
        self.assertEqual(sum(row[extent.PLAN_KEY]['stratum']=='intersection' for row in assigned.values()), 180)
        ident, row = next((ident, row) for ident, row in assigned.items()
            if row[extent.PLAN_KEY]['stratum']=='intersection' and row[extent.PLAN_KEY]['split']=='train')
        _, _, base, _ = extent.anchor(spatial.gen, self.ctx, row, config)
        self.assertEqual(extent.split_for(base['link_idx']), 'val')
        after = copy.deepcopy(originals[ident]);after['spec']=row
        self.assertTrue(extent.verify_transition(originals[ident], after, row))
        self.assertEqual(layout.provisional_split(dict(after, ego=base), config), 'train')
        for key, value in originals[ident]['spec'].items():self.assertEqual(row[key], value)


class OverlayIOTests(unittest.TestCase):
    def source(self, root):
        entries = []
        for i in range(30):
            row = record(i, family='background', actors=False)
            meta_path, scenario_path = root/'source'/(row['id']+'_metadata.json'), root/'source'/(row['id']+'.json')
            atomic_json(meta_path, row['meta']);atomic_json(scenario_path, dict(example=i))
            entries.append(dict(id=row['id'], split='train', metadata=str(meta_path),
                scenario=str(scenario_path), scenario_sha256=sha256(scenario_path)))
        manifest = root/'source'/'manifest.json'
        atomic_json(manifest, dict(count=len(entries), scenes=entries, config={}, provenance={}))
        return manifest

    def test_prepare_preserves_sources_and_capture_split_but_tags_new_metadata(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);source=self.source(root);before={p:p.read_bytes() for p in source.parent.iterdir()}
            result=prepare(source, root/'candidate')
            candidate=json.loads(Path(result['manifest']).read_text())
            overlay=layout.read_overlay(result['split_overlay'])
            self.assertEqual(layout.validate_manifest_overlay(candidate)['errors'], [])
            self.assertEqual({p:p.read_bytes() for p in source.parent.iterdir()}, before)
            for entry in candidate['scenes']:
                meta=json.loads(Path(entry['metadata']).read_text())
                self.assertEqual(entry['split'], meta['split'])
                self.assertEqual(meta['split'], 'train')
                self.assertEqual(layout.resolve(overlay,entry['id'],entry,meta)['split'],entry['derived_training_split'])
            with self.assertRaises(FileExistsError):prepare(source, root/'candidate')
            changed=copy.deepcopy(overlay);changed['assignments'][next(iter(changed['assignments']))]['split']='test'
            atomic_json(root/'bad_overlay.json', changed)
            with self.assertRaises(ValueError):layout.read_overlay(root/'bad_overlay.json')
            subset=copy.deepcopy(candidate);subset['scenes']=subset['scenes'][:1]
            self.assertEqual(layout.validate_manifest_overlay(subset,allow_subset=True)['frames'],1)
            meta_path=Path(candidate['scenes'][0]['metadata']);meta=json.loads(meta_path.read_text())
            meta['support_note']='unapproved nongeometry edit';atomic_json(meta_path,meta)
            with self.assertRaisesRegex(ValueError,'non-overlay'):layout.validate_manifest_overlay(candidate)

    def test_recovery_rebind_requires_mapping_and_preserves_effective_split_group(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);source=self.source(root);result=prepare(source,root/'candidate')
            document=json.loads(Path(result['manifest']).read_text())
            entry=document['scenes'][5];original=json.loads(Path(entry['metadata']).read_text())
            proposed=copy.deepcopy(original);proposed['ego']['x']+=.2
            proposed['fresh_completion_source']=dict(source_cell_id=entry['id'],
                source_scenario_sha256=entry['scenario_sha256'],ordinal=1)
            candidate=copy.deepcopy(entry);candidate.update(id=entry['id']+'_fc01',
                metadata=str(root/'proposal_metadata.json'),scenario=str(root/'proposal.json'))
            atomic_json(Path(candidate['scenario']),dict(new_geometry=True))
            candidate['scenario_sha256']=sha256(candidate['scenario'])
            atomic_json(Path(candidate['metadata']),proposed)
            old=layout.read_overlay(result['split_overlay'])
            with self.assertRaises(ValueError):layout.resolve(old,candidate['id'],dict(candidate,source_cell_id=entry['id']),proposed)
            invalid=copy.deepcopy(proposed);invalid['fresh_completion_source']['ordinal']=0
            with self.assertRaises(ValueError):layout.register_recovery_overlay(entry,original,candidate,invalid,root/'invalid')
            reference=layout.register_recovery_overlay(entry,original,candidate,proposed,root/'rebound')
            proposed['training_split_overlay']=reference;atomic_json(Path(candidate['metadata']),proposed)
            overlay=layout.read_overlay(reference['path'])
            assignment=layout.resolve(overlay,candidate['id'],dict(candidate,source_cell_id=entry['id']),proposed)
            self.assertEqual(assignment['split'],old['assignments'][entry['id']]['split'])
            self.assertEqual(assignment['group_id'],old['assignments'][entry['id']]['group_id'])
            child=dict(document, scenes=[candidate],split_overlay=dict(reference,policy=layout.POLICY))
            self.assertEqual(layout.validate_manifest_overlay(child,allow_subset=True)['errors'],[])
            self.assertFalse(overlay['audit']['native_recovery_qualified'])

    def test_converter_auto_discovers_overlay_and_keeps_capture_metadata_unchanged(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);source=self.source(root);result=prepare(source,root/'candidate')
            document=json.loads(Path(result['manifest']).read_text())
            entry=next(row for row in document['scenes'] if row['derived_training_split']=='val')
            cap=root/'capture';cap.mkdir()
            payload=Path(entry['metadata']).read_bytes();(cap/'metadata.json').write_bytes(payload)
            atomic_json(cap/'audit.json',dict(review_reasons=[],components=[]))
            folder=cap/'LIDAR_5';folder.mkdir();(folder/'0_Intensity.bin').write_bytes(bytes(16))
            sensor=Path(__file__).resolve().parents[3]/'configs/sensors/Sensor_competition_local.json'
            calibration=normalized_sensor_calibration(sensor)
            for cam in calibration['cameras'].values():
                folder=cap/('CAMERA_'+str(cam['sensor_unique_id']));folder.mkdir()
                Image.new('RGB',tuple(cam['resolution_px'])).save(folder/'0_Intensity.png')
            ledger=root/'selected.jsonl'
            ledger.write_text(json.dumps(dict(id='runtime-alias',source_cell_id=entry['id'],
                scenario_sha256=entry['scenario_sha256'],status='accepted_candidate',capture_root=str(cap)))+'\n')
            report=convert(ledger,root/'infos',sensor)
            self.assertEqual(report['splits'],dict(train=0,val=1,test=0))
            info=pickle.loads((root/'infos'/'morai_base5_infos_val.pkl').read_bytes())['data_list'][0]
            self.assertEqual(info['source_split'],'train')
            self.assertEqual(info['scene_group_id'],entry['derived_layout_group_id'])
            self.assertEqual((cap/'metadata.json').read_bytes(),payload)
            overlay=layout.read_overlay(result['split_overlay'])
            meta=json.loads(payload)
            bad=dict(meta,ego=dict(meta['ego'],x=meta['ego']['x']+.1))
            with self.assertRaises(ValueError):layout.resolve(overlay,entry['id'],entry,bad)


if __name__ == '__main__':unittest.main()
