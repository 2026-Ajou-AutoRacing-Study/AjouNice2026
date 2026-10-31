#!/usr/bin/env python3
"""Qualify reviewed native scale profiles; never infer approval or rewrite GT.

Only explicitly accepted profiles with accepted, ground-verified observations
at two independent yaw orientations can become generator candidates. Native
8-corners define box dimensions. Existing unit offsets and observed pivot/map
height define placement initializers only, never training-label geometry.
"""
import argparse
from collections import defaultdict
import copy
import json
import math
from pathlib import Path

import numpy as np

from audit_morai_lidar_gt_visibility import parse_instances
import base5_asset_scale as scale
import base5_native_box_corners as native
from base5_capture_safety import atomic_json, sha256


SCHEMA = 'base5_native_scale_policy_qualification_v1'
AXES = [2, 0, 1]


def read(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise scale.ScaleContractError(message)


def pin(path):
    path = Path(path).absolute()
    require(path.is_file() and not path.is_symlink(), 'Regular qualification source required')
    path = path.resolve(strict=True)
    return dict(path=str(path), sha256=sha256(path))


def _review(path):
    review = read(path)
    require(isinstance(review, dict) and isinstance(review.get('accepted'), dict)
            and isinstance(review.get('rejected', {}), dict), 'Explicit accepted/rejected visual review required')
    accepted = set()
    rejected = set()
    for field, target in (('accepted', accepted), ('rejected', rejected)):
        for name, profiles in review.get(field, {}).items():
            if field == 'rejected' and isinstance(profiles, dict):
                require(all(isinstance(reason, str) and reason for reason in profiles.values()),
                        'Rejection reasons must be nonempty strings')
                profiles = list(profiles)
            require(name in scale.WHITELIST and isinstance(profiles, list)
                    and len(profiles) == len(set(profiles))
                    and all(isinstance(p, str) and p and p != scale.BASE for p in profiles),
                    'Visual review must name whitelisted nonunit profile IDs')
            target.update((name, profile) for profile in profiles)
    require(accepted and not accepted & rejected, 'Approval must be nonempty and must not contradict rejection')
    weights=review.get('profile_weights',{})
    require(isinstance(weights,dict), 'Explicit profile weights must be an asset/profile mapping')
    for name,profiles in weights.items():
        require(name in {name for name,_ in accepted} and isinstance(profiles,dict),
                'Profile weights require an explicitly accepted asset/profile')
        for profile,weight in profiles.items():
            require((name,profile) in accepted, 'Profile weight names an unknown or unaccepted profile')
            require(type(weight) in (int,float) and math.isfinite(weight) and weight>0,
                    'Explicit profile weight must be finite and positive')
    return review, accepted


def _unit_references(paths):
    require(paths, 'Pinned scale-1 placement references required')
    assets, sources = {}, {}
    for path in paths:
        current = pin(path)
        document = read(path)
        require(isinstance(document.get('assets'), dict), 'Unit reference requires assets mapping')
        for name, record in document['assets'].items():
            if name not in scale.WHITELIST:
                continue
            if 'scale_xyz' in record:
                require(record['scale_xyz'] == [1., 1., 1.], 'Unit placement reference is not scale 1')
            size = scale._vector(record.get('bbox_size_m'), positive=True)
            offset = scale._vector(record.get('pivot_to_bbox_center_local_m'))
            assets[name] = dict(bbox_size_m=size, pivot_to_bbox_center_local_m=offset)
            sources[name] = current
    return assets, sources


def _yaw(value):
    value = scale._number(value)
    return round(value % 360., 5)


def _two_yaws(observations):
    yaws = sorted({_yaw(observation['yaw_deg']) for observation in observations})
    return (len(yaws) >= 2 and any(min(abs(a-b), 360-abs(a-b)) >= 45.
                                  for a in yaws for b in yaws))


def _evidence_capture(entry, source, source_entry, source_metadata):
    """Validate one actual accepted frame against its registered source deck."""
    require(entry['status'] == 'accepted_candidate', 'Requested profile contains a QA-failed observation')
    root = Path(entry['capture_root']).resolve(strict=True)
    raw = read(root/'scenario.json')
    metadata = read(root/'metadata.json')
    audit = read(root/'audit.json')
    require(sha256(root/'scenario.json') == source_entry['scenario_sha256'] == entry['scenario_sha256'],
            'Captured/source scenario hashes differ')
    require(metadata.get('scale_probe') == source_metadata.get('scale_probe')
            and metadata.get('spec') == source_metadata.get('spec'), 'Captured probe contract differs from source')
    require(entry.get('audit') == audit and audit.get('accepted_candidate') is True
            and audit.get('raw_complete') is True and not audit.get('review_reasons'),
            'Actual accepted ledger/audit agreement required')
    actors = metadata.get('runtime_actors', [])
    require(len(actors) == 1, 'Native scale qualification requires a single parent')
    actor = actors[0]
    probe = metadata['scale_probe']
    name, uid = actor['name'], actor['unique_id']
    require(name in scale.WHITELIST and actor['data_id'] == scale.WHITELIST[name][0]
            and actor['source_list'] == 'objectList' and actor['motion'] in ('static', 'standing')
            and probe.get('asset') == name and probe.get('parent_uid') == uid,
            'Qualification parent identity/class/motion differs')
    source_actors = source_metadata.get('runtime_actors', [])
    require(len(source_actors) == 1 and actor == source_actors[0],
            'Captured parent differs from registered actor')
    payloads = raw.get('objectList', [])
    require(len(payloads) == 1 and payloads[0]['UNIQUEID'] == uid
            and payloads[0]['DataID'] == actor['data_id'], 'Captured scenario parent differs')
    actual_scale = [float(payloads[0]['scale'][key]) for key in ('x', 'y', 'z')]
    requested_scale = scale._vector(probe.get('requested_scale'), positive=True)
    require(actual_scale == requested_scale, 'Scenario and registered scale differ')
    require(_yaw(float(payloads[0]['rot']['yaw'])) == _yaw(probe['requested_yaw_deg']),
            'Scenario and registered yaw differ')
    components = audit.get('components', [])
    require(len(components) == 1 and components[0]['parent_uid'] == uid
            and components[0]['raw_class'] == 'Obstacle' and components[0]['training_class'] == 'obstacle'
            and components[0]['asset'] == name and components[0]['data_id'] == actor['data_id'],
            'Only single-component physical native Obstacle can qualify')
    component = components[0]
    annotation = list(root.glob('LIDAR_*/*_Instance.txt'))
    require(len(annotation) == 1, 'One exact native LiDAR annotation required')
    boxes = parse_instances(annotation[0])
    require(len(boxes) == 1, 'One native annotation component required')
    records = native.load_matching_corners(annotation[0])
    require(set(records) == {(uid, 'Obstacle', 0)}, 'Native corner component mapping differs')
    nonunit = requested_scale != [1., 1., 1.]
    derived = native.derive_geometry(boxes[0], records[(uid, 'Obstacle', 0)], allow_dimension_change=nonunit)
    require(np.allclose(component['size_m'], derived['size'], atol=1.e-4, rtol=1.e-5)
            and np.allclose(component['center_lidar_m'], derived['center'], atol=1.e-5, rtol=0)
            and np.allclose(component['rotation_rpy'], derived['rotation_rpy'], atol=1.e-7, rtol=0),
            'Accepted audit does not use validated native corner geometry')
    if nonunit:
        require(component.get('native_corner_geometry') == derived['evidence'],
                'Scaled native-corner provenance differs from raw derivation')
    ground = audit.get('ground_support', {})
    observed = [row for row in ground.get('actors', []) if row.get('uid') == uid]
    require(len(observed) == 1 and observed[0].get('verified') is True
            and observed[0].get('reason') is None and not ground.get('review_reasons')
            and abs(scale._number(observed[0].get('minimum_annotation_ground_gap_m'))) <= .15,
            'Qualified profile requires actual ground-plane support at every yaw')
    mapped = actor.get('map_ground_initializer', {})
    require(mapped.get('usable') is True and mapped.get('confidence') in ('road_strip', 'near_centerline'),
            'Supported per-capture map height required for placement initializer')
    above = scale._number(actor['position']['z']) - scale._number(mapped.get('height_m'))
    require(-.5 <= above <= 1., 'Observed profile pivot/map height initializer out of bounds')
    collision = audit.get('capture_safety', {}).get('collision', {})
    require(collision.get('observed') is True and collision.get('no_collision_verified') is True,
            'Actual observed collision-clear capture required')
    files = {str(path.resolve()): sha256(path) for path in root.rglob('*') if path.is_file()}
    require(all(not path.is_symlink() for path in root.rglob('*')), 'Qualification raw cannot contain symlinks')
    require(any('_BBox2D.txt' in path for path in files)
            and any('_BBox3D.txt' in path for path in files)
            and any('_Instance.png' in path for path in files), 'Native camera 2D/3D/mask evidence required')
    return dict(id=entry['id'], name=name, profile_id=probe['profile'], yaw_deg=probe['requested_yaw_deg'],
        scale_xyz=requested_scale, native_size_m=derived['size'],
        pivot_above_map_height_m=above, native_corner_evidence=derived['evidence'],
        raw_files=files, source_scene=pin(source_entry['scenario']), source_metadata=pin(source_entry['metadata']),
        annotation_ground_gap_m=observed[0]['minimum_annotation_ground_gap_m'],
        contact_is_initializer_not_collider_certificate=True)


def prepare(manifest, ledger, review, unit_references, output, *, ground_aligned_pivot_z=(),
            additional_cohorts=()):
    manifest, ledger, review, output = map(lambda path: Path(path).resolve(), (manifest, ledger, review, output))
    require(not output.exists(), 'New qualified-policy output required')
    cohorts = [(manifest, ledger)] + [(Path(m).resolve(), Path(l).resolve()) for m,l in additional_cohorts]
    require(len(set(cohorts)) == len(cohorts), 'Duplicate diagnostic cohort')
    cohort_pins = [pin(path) for cohort in cohorts for path in cohort]
    shared = cohort_pins + [pin(review)]
    initial_pins = shared + [pin(path) for path in unit_references]
    visual, accepted = _review(review)
    ground_aligned = set(ground_aligned_pivot_z)
    require(ground_aligned <= {name for name, _ in accepted}, 'Ground-aligned pivot override must be explicitly approved asset')
    units, unit_pins = _unit_references(unit_references)
    sources, entries, source_metadata, rows = {}, {}, {}, []
    source_file_pins, cohort_summaries = [], []
    total, capture_roots, capture_config = 0, set(), None
    for cohort_index, (cohort_manifest, cohort_ledger) in enumerate(cohorts):
        source = read(cohort_manifest)
        require(source.get('config', {}).get('capture', {}).get('asset_scale_probe') is True
                and str(source.get('status', '')).startswith('DIAGNOSTIC_ONLY')
                and 1 <= source.get('count', 0) == len(source.get('scenes', [])) <= 64,
                'Explicit bounded scale-probe source manifest required')
        total += source['count']
        require(total <= 64, 'Combined diagnostic cohort scene cap is 64')
        comparable_config={k:v for k,v in source['config'].items() if k!='dataset_id'}
        if capture_config is None:
            capture_config=comparable_config
        require(comparable_config==capture_config,
                'Diagnostic cohort capture/environment/placement contracts differ')
        own_ids = set()
        for entry in source['scenes']:
            ident = entry['id']
            require(ident not in entries, 'Duplicate probe source IDs across diagnostic cohorts')
            require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Probe source payload changed')
            source_file_pins.extend((pin(entry['scenario']),pin(entry['metadata'])))
            own_ids.add(ident)
            entries[ident], source_metadata[ident], sources[ident] = entry, read(entry['metadata']), source
        own_rows = [json.loads(line) for line in cohort_ledger.read_text().splitlines() if line.strip()]
        require(len({row['id'] for row in own_rows}) == len(own_rows),
                'Duplicate ledger observations cannot qualify as repeats')
        for row in own_rows:
            require(row['id'] in own_ids, 'Capture ledger includes foreign cohort source IDs')
            # Renaming a scene must not count the same physical raw twice.
            if row.get('capture_root'):
                capture_root = str(Path(row['capture_root']).resolve(strict=True))
                require(capture_root not in capture_roots, 'Duplicate physical capture root across cohorts')
                capture_roots.add(capture_root)
            else:
                require(row.get('status')!='accepted_candidate', 'Accepted cohort observation lacks physical raw')
        rows.extend(own_rows)
        cohort_summaries.append(dict(index=cohort_index,manifest=pin(cohort_manifest),
            ledger=pin(cohort_ledger),source_count=source['count'],ledger_count=len(own_rows),
            scene_ids=sorted(own_ids),config=copy.deepcopy(source['config']),
            source_status=source['status']))
    groups, controls = defaultdict(list), defaultdict(list)
    for row in rows:
        probe = source_metadata[row['id']].get('scale_probe', {})
        key = (probe.get('asset'), probe.get('profile'))
        needed = key in accepted or (probe.get('profile') == scale.BASE and probe.get('asset') in {name for name, _ in accepted})
        if not needed:
            continue
        observation = _evidence_capture(row, sources[row['id']], entries[row['id']], source_metadata[row['id']])
        (controls[observation['name']] if observation['profile_id'] == scale.BASE else groups[key]).append(observation)
    policy = dict(schema=scale.SCHEMA, policy=scale.POLICY, continuous_jitter=False,
                  training_gt_modified=False, assets={})
    summaries = []
    for name, profile_id in sorted(accepted):
        observations, original = groups[(name, profile_id)], controls[name]
        require(_two_yaws(observations) and _two_yaws(original), 'Accepted profile and controls require two independent yaw observations: ' + name + '/' + profile_id)
        require(name in units, 'Missing declared unit placement reference: ' + name)
        factors = observations[0]['scale_xyz']
        require(factors != [1., 1., 1.] and all(row['scale_xyz'] == factors for row in observations),
                'One profile ID cannot mix scenario scale vectors')
        size = np.median([row['native_size_m'] for row in observations], axis=0)
        unit_size = np.median([row['native_size_m'] for row in original], axis=0)
        expected = unit_size * np.asarray(factors)[AXES]
        require(np.allclose(size, expected, rtol=.02, atol=.01), 'Native 8-corner scale axes/profile differ from unit controls')
        require(np.allclose(units[name]['bbox_size_m'], unit_size, rtol=.05, atol=.03),
                'Declared unit geometry differs from observed native controls; explicit reference override required')
        require(all(np.allclose(row['native_size_m'], size, rtol=.02, atol=.01) for row in observations),
                'Profile native geometry changes across yaw observations')
        height = float(np.median([row['pivot_above_map_height_m'] for row in observations]))
        offset = np.asarray(units[name]['pivot_to_bbox_center_local_m']) * np.asarray(factors)[AXES]
        mode = 'unit_offset_times_measured_axis_scale_placement_proxy'
        if name in ground_aligned:
            offset[2] = size[2] / 2 - height
            mode = 'unit_xy_scaled_native_h_half_minus_observed_pivot_height_z_placement_proxy'
        evidence = {item['path']: item['sha256'] for item in shared + [unit_pins[name]]}
        for observation in observations + original:
            evidence.update(observation['raw_files'])
            for item in (observation['source_scene'], observation['source_metadata']):
                evidence[item['path']] = item['sha256']
        group = scale.WHITELIST[name][1]
        record = policy['assets'].setdefault(name, dict(data_id=scale.WHITELIST[name][0], group=group,
            source_list='objectList', semantic_class='obstacle', variation_fraction=.3 if group == 'box' else .2,
            minimum_original=2, bbox_axis_to_scale_axis=AXES, profiles={}))
        record['profiles'][profile_id] = dict(status='qualified', scale_xyz=factors,
            weight=visual.get('profile_weights',{}).get(name,{}).get(
                profile_id,7 if max(factors) <= 1.25 else 3),
            native_geometry=dict(bbox_size_m=size.tolist(), pivot_to_bbox_center_local_m=offset.tolist(),
                pivot_height_above_ground_m=height,
                bbox_source='validated_native_8corners_q50_no_scale_multiplication',
                pivot_offset_source=mode, unit_offset_reference=unit_pins[name],
                contact_source='captured_parent_position_z_minus_supported_map_height_q50',
                placement_offset_is_training_gt=False, contact_is_initializer_not_collider_certificate=True),
            native_qualification=dict(observations=len(observations), semantic_visual_review='accepted',
                contact_review='passed', sensor_gt_review='passed', axis_mapping_verified=True,
                single_component_obstacle_verified=True,
                evidence=[dict(path=path, sha256=expected_sha) for path, expected_sha in sorted(evidence.items())],
                observed_yaws_deg=sorted({_yaw(row['yaw_deg']) for row in observations}),
                control_observations=len(original), native_corner_policy=native.POLICY,
                raw_preserved=True, ego_yaw_only_world_pivot_estimation_used=False))
        summaries.append(dict(name=name, profile_id=profile_id, observations=len(observations),
            native_size_q50_m=size.tolist(), source_unit_size_m=unit_size.tolist(), scale_xyz=factors,
            pivot_offset_placement_proxy_m=offset.tolist(), pivot_above_map_q50_m=height,
            initializer_mode=mode, observation_ids=[row['id'] for row in observations]))
    scale.validate_policy(policy)
    for record in policy['assets'].values():
        for profile in record['profiles'].values():
            scale._pins(profile['native_qualification']['evidence'])
    scale._pins(initial_pins)
    scale._pins(source_file_pins)
    atomic_json(output, dict(policy, qualification=dict(schema=SCHEMA,
        sources=shared, unit_references=[pin(path) for path in unit_references],
        diagnostic_cohorts=cohort_summaries,combined_scene_count=total,
        source_files=source_file_pins,cohort_ids_and_raw_preserved=True,
        reviewed=visual, profiles=summaries, training_gt_from_native_corners_only=True,
        raw_files_modified=False, generator_adoption_not_run=True, bulk_started=False)))
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--ledger', type=Path, required=True)
    parser.add_argument('--additional-cohort',type=Path,nargs=2,action='append',default=[],
                        metavar=('MANIFEST','LEDGER'),
                        help='Additional bounded diagnostic cohort; all source IDs must be unique and combined count <=64')
    parser.add_argument('--review', type=Path, required=True)
    parser.add_argument('--unit-reference', type=Path, action='append', required=True)
    parser.add_argument('--ground-aligned-pivot-z', action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.ledger, args.review, args.unit_reference, args.output,
                  ground_aligned_pivot_z=args.ground_aligned_pivot_z,
                  additional_cohorts=args.additional_cohort))
