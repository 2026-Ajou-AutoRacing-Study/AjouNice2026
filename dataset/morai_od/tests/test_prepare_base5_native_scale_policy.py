import copy
import itertools
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_asset_scale as scale
import base5_native_box_corners as native
import prepare_base5_native_scale_policy as prepare
from audit_morai_lidar_gt_visibility import parse_instances
from base5_capture_safety import sha256


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True) + '\n')


class NativeScalePolicyTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.manifest, self.ledger, self.review = (self.root/name for name in ('manifest.json', 'ledger.jsonl', 'review.json'))
        self.reference, self.output = self.root/'units.json', self.root/'qualified.json'
        self.size, self.offset = [1., 2., .5], [.01, -.02, .25]
        write(self.reference, dict(assets=dict(CargoBox=dict(bbox_size_m=self.size,
            pivot_to_bbox_center_local_m=self.offset))))
        self.rows, entries = [], []
        for yaw in (0, 90):
            for profile, factor in (('original', 1.), ('iso125', 1.25)):
                ident = f'CargoBox_y{yaw}_{profile}'
                root = self.root/'captures'/ident
                lidar = root/'LIDAR_5'
                lidar.mkdir(parents=True)
                center, rpy = np.asarray([5., 1., 2.]), np.asarray([.02, -.03, .7])
                extent = np.asarray(self.size)*factor
                corners = np.asarray(list(itertools.product((-1, 1), repeat=3))) * extent / 2 @ native._rotation(rpy).T + center
                annotation, corner = lidar/'time_Instance.txt', lidar/'time_Instance_8Points.txt'
                annotation.write_text(' '.join(['Obstacle', *map(str, center), *map(str, rpy),
                    *map(str, self.size), '5', '0', '0', '0', '8500']) + '\n')
                corner.write_text(' '.join(['Obstacle', *map(str, corners.reshape(-1)), *map(str, rpy),
                    '5', '0', '0', '0', '8500']) + '\n')
                box = parse_instances(annotation)[0]
                derived = native.derive_geometry(box, native.load_matching_corners(annotation)[(8500, 'Obstacle', 0)],
                                                 allow_dimension_change=profile != 'original')
                scene = dict(objectList=[dict(UNIQUEID=8500, DataID=40100001,
                    scale=dict(x=factor, y=factor, z=factor), rot=dict(yaw=str(yaw)))])
                actor = dict(name='CargoBox', data_id=40100001, source_list='objectList', unique_id=8500,
                    motion='static', position=dict(x=0., y=0., z=3.02),
                    map_ground_initializer=dict(usable=True, confidence='near_centerline', height_m=3.))
                metadata = dict(spec=dict(seed=1), runtime_actors=[actor], scale_probe=dict(
                    asset='CargoBox', parent_uid=8500, profile=profile, requested_scale=[factor]*3,
                    requested_yaw_deg=yaw))
                component = dict(parent_uid=8500, raw_class='Obstacle', training_class='obstacle',
                    asset='CargoBox', data_id=40100001, size_m=derived['size'],
                    center_lidar_m=derived['center'], rotation_rpy=derived['rotation_rpy'])
                if profile != 'original':
                    component['native_corner_geometry'] = derived['evidence']
                audit = dict(accepted_candidate=True, raw_complete=True, review_reasons=[], components=[component],
                    ground_support=dict(review_reasons=[], actors=[dict(uid=8500, verified=True,
                        reason=None, minimum_annotation_ground_gap_m=.01)]),
                    capture_safety=dict(collision=dict(observed=True, no_collision_verified=True)))
                write(root/'scenario.json', scene)
                write(root/'metadata.json', metadata)
                write(root/'audit.json', audit)
                camera = root/'CAMERA_1'
                camera.mkdir()
                for suffix in ('BBox2D.txt', 'BBox3D.txt', 'Instance.png', 'Intensity.png'):
                    (camera/('time_'+suffix)).write_bytes(b'unit-test evidence only')
                source_scenario, source_meta = self.root/'sources'/(ident+'.json'), self.root/'sources'/(ident+'_meta.json')
                write(source_scenario, scene)
                write(source_meta, metadata)
                entry = dict(id=ident, scenario=str(source_scenario), metadata=str(source_meta),
                             scenario_sha256=sha256(source_scenario))
                entries.append(entry)
                self.rows.append(dict(id=ident, status='accepted_candidate', capture_root=str(root),
                                      scenario_sha256=entry['scenario_sha256'], audit=audit))
        write(self.manifest, dict(count=4, scenes=entries, status='DIAGNOSTIC_ONLY; unit fixture',
                                  config=dict(capture=dict(asset_scale_probe=True))))
        self.save_ledger()
        write(self.review, dict(accepted=dict(CargoBox=['iso125']), rejected={}))

    def save_ledger(self):
        self.ledger.write_text(''.join(json.dumps(row, sort_keys=True)+'\n' for row in self.rows))

    def qualify(self, **kwargs):
        return prepare.prepare(self.manifest, self.ledger, self.review, [self.reference], self.output, **kwargs)

    def update_audit(self, row, mutate):
        audit = copy.deepcopy(row['audit'])
        mutate(audit)
        row['audit'] = audit
        write(Path(row['capture_root'])/'audit.json', audit)
        self.save_ledger()

    def test_explicit_profile_two_yaws_native_corners_and_all_evidence_are_qualified(self):
        before = {str(path): sha256(path) for path in self.root.rglob('*') if path.is_file()}
        self.qualify()
        policy = json.loads(self.output.read_text())
        scale.validate_policy(policy)
        profile = policy['assets']['CargoBox']['profiles']['iso125']
        np.testing.assert_allclose(profile['native_geometry']['bbox_size_m'], np.asarray(self.size)*1.25)
        np.testing.assert_allclose(profile['native_geometry']['pivot_to_bbox_center_local_m'], np.asarray(self.offset)*1.25)
        self.assertAlmostEqual(profile['native_geometry']['pivot_height_above_ground_m'], .02)
        qualification = profile['native_qualification']
        self.assertEqual(qualification['observations'], 2)
        self.assertEqual(qualification['observed_yaws_deg'], [0., 90.])
        self.assertFalse(qualification['ego_yaw_only_world_pivot_estimation_used'])
        paths = [pin['path'] for pin in qualification['evidence']]
        self.assertTrue(any(path.endswith('_Instance_8Points.txt') for path in paths))
        self.assertTrue(any(path.endswith('_BBox2D.txt') for path in paths))
        self.assertTrue(any(path.endswith('_BBox3D.txt') for path in paths))
        self.assertTrue(any(path.endswith('_Instance.png') for path in paths))
        for path, expected in before.items():
            self.assertEqual(sha256(path), expected)

    def test_ground_aligned_z_proxy_is_explicit_and_not_used_for_native_bbox_truth(self):
        self.qualify(ground_aligned_pivot_z=['CargoBox'])
        profile = json.loads(self.output.read_text())['assets']['CargoBox']['profiles']['iso125']
        self.assertAlmostEqual(profile['native_geometry']['pivot_to_bbox_center_local_m'][2], .625/2 - .02)
        np.testing.assert_allclose(profile['native_geometry']['bbox_size_m'], [1.25, 2.5, .625])
        self.assertFalse(profile['native_geometry']['placement_offset_is_training_gt'])

    def test_no_approval_or_overlapping_rejection_cannot_qualify(self):
        for review in (dict(accepted={}, rejected={}),
                       dict(accepted=dict(CargoBox=['iso125']), rejected=dict(CargoBox=['iso125']))):
            write(self.review, review)
            with self.assertRaises(scale.ScaleContractError):
                self.qualify()
        self.assertFalse(self.output.exists())

    def test_explicit_positive_profile_weights_override_only_declared_profiles(self):
        review=json.loads(self.review.read_text())
        review['profile_weights']=dict(CargoBox=dict(iso125=35))
        write(self.review,review)
        self.qualify()
        policy=json.loads(self.output.read_text())
        self.assertEqual(policy['assets']['CargoBox']['profiles']['iso125']['weight'],35)
        self.assertEqual(policy['assets']['CargoBox']['variation_fraction'],.3)
        self.assertEqual(policy['qualification']['reviewed']['profile_weights'],review['profile_weights'])

    def test_unknown_unaccepted_nonfinite_boolean_or_nonpositive_weights_are_rejected(self):
        for weights in ([],dict(UnknownAsset=dict(iso125=1)),dict(CargoBox=[]),
                dict(CargoBox=dict(original=1)),dict(CargoBox=dict(iso300=1)),
                *[dict(CargoBox=dict(iso125=value)) for value in (0,-1,True,'1',None,float('nan'),float('inf'))]):
            with self.subTest(weights=weights):
                review=dict(accepted=dict(CargoBox=['iso125']),rejected={},profile_weights=weights)
                write(self.review,review)
                with self.assertRaises(scale.ScaleContractError):
                    self.qualify()
                self.assertFalse(self.output.exists())

    def test_single_observation_or_same_yaw_not_enough(self):
        self.rows = [row for row in self.rows if not row['id'].endswith('y90_iso125')]
        self.save_ledger()
        with self.assertRaisesRegex(scale.ScaleContractError, 'two independent yaw'):
            self.qualify()

    def test_review_or_failed_native_observation_cannot_be_promoted(self):
        self.rows[1]['status'] = 'review'
        self.save_ledger()
        with self.assertRaisesRegex(scale.ScaleContractError, 'QA-failed'):
            self.qualify()

    def test_audit_acceptance_missing_ground_or_collision_is_rejected(self):
        self.update_audit(self.rows[1], lambda a: a['ground_support']['actors'][0].update(verified=False))
        with self.assertRaisesRegex(scale.ScaleContractError, 'ground-plane support'):
            self.qualify()

    def test_scaled_size_must_match_raw_corners_not_original_summary_cache(self):
        self.update_audit(self.rows[1], lambda a: a['components'][0].update(size_m=self.size))
        with self.assertRaisesRegex(scale.ScaleContractError, 'validated native corner'):
            self.qualify()

    def test_profile_name_cannot_mix_actual_scale_vectors(self):
        row = self.rows[3]
        root = Path(row['capture_root'])
        scene, metadata = json.loads((root/'scenario.json').read_text()), json.loads((root/'metadata.json').read_text())
        scene['objectList'][0]['scale'] = dict(x=1.5, y=1.5, z=1.5)
        metadata['scale_probe']['requested_scale'] = [1.5]*3
        entry = json.loads(self.manifest.read_text())['scenes'][3]
        for path in (root/'scenario.json', Path(entry['scenario'])):
            write(path, scene)
        for path in (root/'metadata.json', Path(entry['metadata'])):
            write(path, metadata)
        manifest = json.loads(self.manifest.read_text())
        manifest['scenes'][3]['scenario_sha256'] = sha256(root/'scenario.json')
        write(self.manifest, manifest)
        row['scenario_sha256'] = manifest['scenes'][3]['scenario_sha256']
        self.save_ledger()
        with self.assertRaisesRegex(scale.ScaleContractError, 'mix scenario scale'):
            self.qualify()

    def test_wrong_source_hash_and_unit_reference_require_explicit_correction(self):
        write(self.reference, dict(assets=dict(CargoBox=dict(bbox_size_m=[9., 9., 9.],
            pivot_to_bbox_center_local_m=self.offset))))
        with self.assertRaisesRegex(scale.ScaleContractError, 'reference override required'):
            self.qualify()

    def test_checksum_bound_sources_cannot_change_during_qualification(self):
        original = prepare._evidence_capture
        def mutate(*args, **kwargs):
            result = original(*args, **kwargs)
            self.review.write_text(self.review.read_text() + '\n')
            return result
        with patch.object(prepare, '_evidence_capture', side_effect=mutate):
            with self.assertRaisesRegex(scale.ScaleContractError, 'evidence changed'):
                self.qualify()
        self.assertFalse(self.output.exists())

    def split_cohorts(self):
        source=json.loads(self.manifest.read_text())
        extra_manifest,extra_ledger=self.root/'extra.json',self.root/'extra.jsonl'
        first,second=copy.deepcopy(source),copy.deepcopy(source)
        first.update(count=2,scenes=source['scenes'][:2])
        second.update(count=2,scenes=source['scenes'][2:])
        first['config']['dataset_id']='first-screen'
        second['config']['dataset_id']='barrier-followup'
        write(self.manifest,first)
        write(extra_manifest,second)
        self.ledger.write_text(''.join(json.dumps(row)+'\n' for row in self.rows[:2]))
        extra_ledger.write_text(''.join(json.dumps(row)+'\n' for row in self.rows[2:]))
        return extra_manifest,extra_ledger

    def test_explicit_multiple_cohorts_preserve_each_pin_and_do_not_rename_sources(self):
        extra_manifest,extra_ledger=self.split_cohorts()
        before={str(p):sha256(p) for p in self.root.rglob('*') if p.is_file()}
        self.qualify(additional_cohorts=[(extra_manifest,extra_ledger)])
        policy=json.loads(self.output.read_text())
        self.assertEqual(policy['qualification']['combined_scene_count'],4)
        self.assertEqual(len(policy['qualification']['diagnostic_cohorts']),2)
        evidence={p['path']:p['sha256'] for p in policy['assets']['CargoBox']['profiles']['iso125']['native_qualification']['evidence']}
        for p in (self.manifest,self.ledger,extra_manifest,extra_ledger):
            self.assertEqual(evidence[str(p.resolve())],before[str(p)])
        for p,digest in before.items():
            self.assertEqual(sha256(p),digest)

    def test_duplicate_ids_or_cross_cohort_ledger_ownership_fail_closed(self):
        extra_manifest,extra_ledger=self.split_cohorts()
        extra=json.loads(extra_manifest.read_text())
        extra['scenes'][0]['id']=json.loads(self.manifest.read_text())['scenes'][0]['id']
        write(extra_manifest,extra)
        with self.assertRaisesRegex(scale.ScaleContractError,'Duplicate probe source IDs'):
            self.qualify(additional_cohorts=[(extra_manifest,extra_ledger)])
        extra['scenes'][0]['id']=self.rows[2]['id']
        write(extra_manifest,extra)
        extra_ledger.write_text(json.dumps(self.rows[0])+'\n')
        with self.assertRaisesRegex(scale.ScaleContractError,'foreign cohort'):
            self.qualify(additional_cohorts=[(extra_manifest,extra_ledger)])

    def test_mixed_capture_contract_or_reused_raw_cannot_qualify(self):
        extra_manifest,extra_ledger=self.split_cohorts()
        extra=json.loads(extra_manifest.read_text())
        extra['config']['capture']['ground_support_check']=False
        write(extra_manifest,extra)
        with self.assertRaisesRegex(scale.ScaleContractError,'contracts differ'):
            self.qualify(additional_cohorts=[(extra_manifest,extra_ledger)])
        extra['config']['capture'].pop('ground_support_check')
        write(extra_manifest,extra)
        row=copy.deepcopy(self.rows[2])
        row['capture_root']=self.rows[0]['capture_root']
        extra_ledger.write_text(json.dumps(row)+'\n')
        with self.assertRaisesRegex(scale.ScaleContractError,'Duplicate physical capture'):
            self.qualify(additional_cohorts=[(extra_manifest,extra_ledger)])

    def test_nonapproved_transport_failure_does_not_require_nonexistent_capture_root(self):
        source=json.loads(self.manifest.read_text())
        entry=copy.deepcopy(source['scenes'][0])
        entry['id']='unselected_failure'
        scenario=json.loads(Path(entry['scenario']).read_text())
        scenario['objectList'][0]['scale']=dict(x=3.,y=3.,z=3.)
        metadata=json.loads(Path(entry['metadata']).read_text())
        metadata['scale_probe'].update(profile='iso300',requested_scale=[3.]*3)
        raw,meta=self.root/'unused.json',self.root/'unused_metadata.json'
        write(raw,scenario)
        write(meta,metadata)
        entry.update(scenario=str(raw),metadata=str(meta),scenario_sha256=sha256(raw))
        source['scenes'].append(entry)
        source['count']=5
        write(self.manifest,source)
        self.rows.append(dict(id=entry['id'],status='failed',error='transport failure before capture'))
        self.save_ledger()
        self.qualify()
        policy=json.loads(self.output.read_text())
        self.assertEqual(set(policy['assets']['CargoBox']['profiles']),{'iso125'})

    def test_combined_limit_and_source_metadata_mutation_fail_closed(self):
        extra_manifest,extra_ledger=self.split_cohorts()
        extra=json.loads(extra_manifest.read_text())
        extra['scenes']=[dict(extra['scenes'][0],id='extra_'+str(i)) for i in range(63)]
        extra['count']=63
        write(extra_manifest,extra)
        with self.assertRaisesRegex(scale.ScaleContractError,'scene cap is 64'):
            self.qualify(additional_cohorts=[(extra_manifest,extra_ledger)])
        first=json.loads(self.manifest.read_text())
        # Restore the original fixture entries after the independent cap test.
        first.update(count=4,scenes=[])
        for row in self.rows:
            scenario=self.root/'sources'/(row['id']+'.json')
            first['scenes'].append(dict(id=row['id'],scenario=str(scenario),
                metadata=str(self.root/'sources'/(row['id']+'_meta.json')),
                scenario_sha256=sha256(scenario)))
        write(self.manifest,first)
        self.save_ledger()
        original=prepare._evidence_capture
        def mutate(*args,**kwargs):
            result=original(*args,**kwargs)
            metadata_path=json.loads(self.manifest.read_text())['scenes'][0]['metadata']
            with Path(metadata_path).open('a') as stream:
                stream.write('\n')
            return result
        with patch.object(prepare,'_evidence_capture',side_effect=mutate):
            with self.assertRaisesRegex(scale.ScaleContractError,'evidence changed'):
                self.qualify()
        self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
