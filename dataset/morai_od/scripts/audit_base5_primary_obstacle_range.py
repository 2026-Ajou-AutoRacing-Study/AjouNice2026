#!/usr/bin/env python3
"""Read-only, CPU-only native-size/primary-range audit for a new output directory.

Rebuild size quantiles from all native components, not just retained GT. Join
the canonical selected ledger, both 512 upright infos, exact model-camera
support records and scale-one scenario parents. Report the broad research
scope separately from the strict preparer scope (no avoidance/shadow phase).
This tool never changes a source capture, GT, manifest, generator or policy.
Pickle inputs must be trusted local release files; no model/GPU/ROS is loaded.
"""
import argparse
import hashlib
import json
import math
import pickle
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

from base5_upright_boxes import component_bbox


SCRIPT_DIR = Path(__file__).resolve().parent
REPO = SCRIPT_DIR.parents[2]
NATIVE = REPO / 'artifacts/handoff/base5_sunny13_jpeg90_upright_v1_261011_release'
PLAN = REPO / 'artifacts/evaluation/base5_generator_261015/candidate_v151/manifest.json'
SUPPORT = REPO / 'artifacts/evaluation/bevfusion/base5_range_evidence_release_261012/objects.json'
LEDGER = Path.home() / ('morai_cap/base5_v2_261008/final_appearance_5k_v132_release/'
                        'collection_5k_sunny13_flexible_v140/selected_ledger.jsonl')
KST = timezone(timedelta(hours=9))
BANDS = (0, 10, 20, 30, 40, 50, 60, 80, 100, math.inf)


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def unique_index(rows, key, description):
    result = {}
    for row in rows:
        value = key(row)
        require(value not in result, f'Duplicate {description}: {value}')
        result[value] = row
    return result


def physical(actor):
    return (actor.get('source_list') == 'objectList'
            and str(actor.get('population_pool', '')).startswith('obstacle'))


def cap_for_size(size, compact=True):
    require(len(size) == 3 and all(not isinstance(x, bool) and isinstance(x, (int, float))
            and math.isfinite(x) and x > 0 for x in size), 'Invalid native dimensions')
    minimum = min(size)
    if minimum < .25:
        return 20
    if minimum < .35 or (compact and minimum < .40 and max(size[:2]) < 1.5):
        return 30
    return 40 if minimum < .60 else None


def scope(spec, actor, strict=True):
    return (spec.get('mission') is False and spec.get('family') == 'obstacle'
            and physical(actor) and (not strict or (not spec.get('avoidance_phase')
                and not spec.get('shadow_phase') and actor.get('motion') in ('static', 'standing'))))


def component_key(row):
    return str(row['token']), str(row['component_id'])


def component_signature(row):
    keys = ('parent_uid', 'component_id', 'data_id', 'asset', 'training_class',
            'center_lidar_m', 'size_m', 'rotation_rpy', 'lidar_points')
    return {key: row[key] for key in keys}


def verify_scale_parent(scenario, actor, count):
    require(count == 1, 'A physical parent must have exactly one native component')
    require(actor.get('motion') == 'static', 'Size reference requires static observations')
    parents = unique_index(scenario.get('objectList', []), lambda x: int(x['UNIQUEID']),
                           'scenario parent UID')
    parent = parents.get(int(actor['unique_id']))
    require(parent is not None and int(parent['DataID']) == int(actor['data_id']),
            'Native scenario parent/data identity differs')
    require(all(float(parent['scale'][axis]) == 1. for axis in 'xyz'),
            'Size reference requires scale-one parents')


def verify_gt_join(component, instance, support):
    box = np.asarray(component_bbox(component), dtype=np.float64)
    require(instance.get('asset') == component['asset']
            and int(instance.get('parent_uid')) == int(component['parent_uid']),
            'GT parent/asset identity differs')
    require(int(instance['bbox_label_3d']) == 2, 'Physical obstacle GT class differs')
    require(np.allclose(instance['bbox_3d'], box, rtol=0, atol=1e-6),
            'Native upright geometry differs from retained GT')
    require(support.get('asset') == component['asset'] and support.get('cls') == 'obstacle',
            'Support record class/asset differs')
    require(np.allclose(support['center_lidar_m'], box[:3], rtol=0, atol=1e-6),
            'Support record center differs')


def band(distance):
    for low, high in zip(BANDS, BANDS[1:]):
        if low <= distance < high:
            return f'{low}_{high}m' if math.isfinite(high) else '100plus_m'
    raise ValueError('Invalid observation distance')


def stats(values):
    return None if not values else dict(n=len(values), mean=float(np.mean(values)),
        p50=float(np.median(values)), p90=float(np.percentile(values, 90)),
        p95=float(np.percentile(values, 95)), max=float(max(values)))


def summarize(rows):
    retained = [x for x in rows if x['retained512']]
    return dict(native_components=len(rows), primary_components=sum(x['primary'] for x in rows),
        retained512_components=len(retained), native_zero_points=sum(x['native_points'] == 0 for x in rows),
        native_lidar_ge20=sum(x['native_points'] >= 20 for x in rows),
        native_points=stats([x['native_points'] for x in rows]),
        native_camera_pixels=stats([x['native_camera_pixels'] for x in rows]),
        retained_model_camera_gate=sum(x['model512_camera_gate'] for x in retained),
        retained_model_camera_rich=sum(x['model512_camera_rich'] for x in retained),
        retained_rich_support=sum(x['model512_camera_rich'] or x['model512_points'] >= 20
                                  for x in retained),
        retained_model_mask_pixels=stats([x['model512_mask_pixels'] for x in retained]))


def observation(component, actor, token, split, spec, retained):
    box = component_bbox(component)
    return dict(token=token, component_id=component['component_id'], parent_uid=component['parent_uid'],
        data_id=component['data_id'], asset=component['asset'], split=split,
        family=spec['family'], mission=spec['mission'], primary=actor['_primary'],
        role=actor.get('actual_ego_role'), planned_ego_center_distance_m=actor.get('planned_center_distance_m'),
        actual_lidar_center_range_m=float(np.linalg.norm(box[:2])), upright_size_m=box[3:6],
        native_size_m=component['size_m'], native_points=component['lidar_points'],
        native_camera_pixels=max([c.get('native_instance_pixels', 0) for c in component['cameras']] or [0]),
        retained512=retained is not None,
        model512_points=retained['points'] if retained else None,
        model512_camera_gate=bool(retained and retained['camera_gate']),
        model512_camera_rich=bool(retained and retained['camera_rich_proxy']),
        model512_mask_pixels=retained['best_visible_mask_pixels'] if retained else None)


def audit_native(native, ledger_path, support_path, expected_frames):
    manifest = read_json(native / 'manifest.json')
    ledger_sha = sha256(ledger_path)
    require(ledger_sha == manifest['source_ledger_sha256'], 'Selected ledger checksum differs')
    with open(ledger_path) as stream:
        ledger = unique_index((json.loads(line) for line in stream if line.strip()),
                              lambda x: x['id'], 'selected ledger ID')
    require(len(ledger) == expected_frames == manifest['scenes'], 'Canonical ledger/frame count differs')
    require(all(x['status'] == 'accepted_candidate' for x in ledger.values()),
            'Selected ledger contains an unaccepted capture')
    support = unique_index(read_json(support_path), component_key, 'support component')
    frames, gt = {}, {}
    sources = {str(ledger_path): ledger_sha, str(support_path): sha256(support_path),
               str(native / 'manifest.json'): sha256(native / 'manifest.json')}
    split_counts = Counter()
    for split in ('train', 'val'):
        path = native / f'infos/512x1056/morai_base5_infos_{split}.pkl'
        sources[str(path)] = sha256(path)
        with path.open('rb') as stream:
            rows = pickle.load(stream)['data_list']
        for row in rows:
            token = row['token']
            require(token not in frames, 'Train/val token overlap')
            frames[token] = split
            split_counts[split] += 1
            for item in row['instances']:
                key = (token, str(item['component_id']))
                require(key not in gt, 'Duplicate retained GT component')
                gt[key] = item
    require(set(frames) == set(ledger), 'Selected ledger/GT token sets differ')
    require(set(gt) == set(support), '512 GT/support component sets differ')
    require(dict(split_counts) == manifest['split_counts'], 'Train/val count differs')
    observations, inputs, verified_scenarios = [], {}, set()
    physical_gt_keys = set()
    for token, split in sorted(frames.items()):
        directory = native / 'data' / token
        capture = {}
        for name in ('audit.json', 'metadata.json'):
            path = directory / name
            checksum = sha256(path)
            require(checksum == manifest['files'][f'data/{token}/{name}']['sha256'],
                    'Native release capture checksum differs')
            inputs[str(path)] = checksum
            capture[name] = read_json(path)
        components = capture['audit.json']['components']
        original = ledger[token]['audit']['components']
        require([component_signature(x) for x in components] == [component_signature(x) for x in original],
                'Selected ledger/native component geometry differs')
        meta = capture['metadata.json']
        actors = unique_index(meta['runtime_actors'], lambda x: int(x['unique_id']), 'runtime UID')
        primary_uid = int(meta['runtime_actors'][0]['unique_id']) if meta['runtime_actors'] else None
        parent_counts = Counter(int(x['parent_uid']) for x in components)
        scenario = None
        for component in components:
            actor = actors.get(int(component['parent_uid']))
            if not actor or not physical(actor) or component['training_class'] != 'obstacle':
                continue
            require(component['asset'] == actor['name']
                    and int(component['data_id']) == int(actor['data_id']), 'Native runtime identity differs')
            if scenario is None:
                path = directory / 'scenario.json'
                checksum = sha256(path)
                require(checksum == manifest['files'][f'data/{token}/scenario.json']['sha256']
                        == ledger[token]['scenario_sha256'], 'Native scenario checksum differs')
                inputs[str(path)] = checksum
                scenario = read_json(path)
                verified_scenarios.add(token)
            verify_scale_parent(scenario, actor, parent_counts[int(component['parent_uid'])])
            key = (token, str(component['component_id']))
            retained = support.get(key)
            if key in gt:
                verify_gt_join(component, gt[key], retained)
                physical_gt_keys.add(key)
            actor = dict(actor, _primary=int(actor['unique_id']) == primary_uid)
            observations.append(observation(component, actor, token, split, meta['spec'], retained))
    grouped = defaultdict(list)
    for row in observations:
        grouped[str(row['data_id'])].append(row)
    assets, by_asset = {}, {}
    for data_id, rows in sorted(grouped.items()):
        sizes = np.asarray([x['upright_size_m'] for x in rows])
        n = len(rows)
        assets[data_id] = dict(name=rows[0]['asset'], size_q50_m=np.median(sizes, axis=0).tolist(),
            size_q10_m=np.percentile(sizes, 10, axis=0).tolist(),
            size_q90_m=np.percentile(sizes, 90, axis=0).tolist(), observations=n,
            native_raw_local_size_q50_m=np.median([x['native_size_m'] for x in rows], axis=0).tolist(),
            minimum_axis_q50_m=float(np.min(np.median(sizes, axis=0))),
            parent_observations=n, frames=len({x['token'] for x in rows}),
            primary_observations=sum(x['primary'] for x in rows),
            retained512_observations=sum(x['retained512'] for x in rows),
            static_observations_verified=n, scale1_observations_verified=n,
            single_component_parent_observations_verified=n)
        by_asset[rows[0]['asset']] = dict(summary=summarize(rows),
            ranges={key: summarize([x for x in rows if band(x['actual_lidar_center_range_m']) == key])
                    for key in sorted({band(x['actual_lidar_center_range_m']) for x in rows})})
    provenance = dict(native_release=str(native), native_release_manifest_sha256=sha256(native / 'manifest.json'),
        native_inputs_sha256=sources,
        native_capture_inputs_sha256=inputs, source_ledger_sha256=ledger_sha,
        geometry_helper=dict(path=str(SCRIPT_DIR / 'base5_upright_boxes.py'),
                             sha256=sha256(SCRIPT_DIR / 'base5_upright_boxes.py')),
        observations=len(observations), native_release_frames=len(frames),
        retained_gt_components=len(gt), retained_physical_gt_components=len(physical_gt_keys),
        scale1_scenarios_verified=len(verified_scenarios), single_component_assets=len(assets),
        size_method='q50 of scale-one static native full-RPY, yaw-removed upright extents',
        collider_or_spacing_geometry_changed=False, native_annotations_or_gt_changed=False,
        release_v151_changed=False)
    return dict(schema='base5_primary_obstacle_size_reference_v1', assets=assets, provenance=provenance), dict(
        schema='base5_physical_obstacle_native_range_audit_reproduced_v1',
        summary=summarize(observations), by_asset=by_asset, observations=observations,
        checks=dict(ledger_frames=len(ledger), split_counts=dict(split_counts), gt_support_join=len(gt),
                    retained_physical_join=len(physical_gt_keys), native_scale_one_static=len(observations)))


def impact_summary(rows):
    return dict(scenes=len(rows), assets=dict(Counter(x['asset'] for x in rows)),
        roles=dict(Counter(x['role'] for x in rows)), splits=dict(Counter(x['split'] for x in rows)),
        cap_m=dict(Counter(str(x['cap_m']) for x in rows)),
        original_far85_subset=sum(x['planned_distance_m'] >= 30 for x in rows),
        front_plan_cap_groups=dict(Counter(f"front={x['front_plan']},cap={x['cap_m']}" for x in rows)),
        scene_ids=[x['id'] for x in rows])


def audit_plan(path, assets, expected_frames):
    manifest = read_json(path)
    rows = manifest['scenes']
    require(len(rows) == expected_frames == manifest['count'], 'Plan frame count differs')
    unique_index(rows, lambda x: x['id'], 'plan scene ID')
    eligible = defaultdict(list)
    sources = {str(path): sha256(path)}
    protected_near, vehicle_far, physical_count, generic_far = 0, 0, 0, 0
    all_actors = 0
    for row in rows:
        meta_path, scenario_path = Path(row['metadata']), Path(row['scenario'])
        sources[str(meta_path)] = sha256(meta_path)
        require(sha256(scenario_path) == row['scenario_sha256'], 'Plan scenario checksum differs')
        meta = read_json(meta_path)
        spec, actors = meta['spec'], meta['runtime_actors']
        all_actors += len(actors)
        require(spec['family'] == row['family'] and spec['mission'] == row['mission']
                and spec.get('distance_bounds', []) == row['distance_bounds'], 'Manifest/spec contract differs')
        scenario = read_json(scenario_path)
        actual = {}
        # AI pedestrians use MORAI's literal misspelling `aiPedstrianList`.
        # Read the declared container, never collapse it into pedestrianList.
        for container in {actor['source_list'] for actor in actors}:
            for parent in scenario.get(container, []):
                key = (container, int(parent['UNIQUEID']))
                require(key not in actual, 'Duplicate plan scenario container/UID')
                actual[key] = int(parent['DataID'])
        for actor in actors:
            require(actual.get((actor['source_list'], int(actor['unique_id']))) == int(actor['data_id']),
                    'Plan actor/scenario identity differs')
        if not actors:
            continue
        actor = actors[0]
        if 'primary_identity' in spec:
            require(int(spec['primary_identity']) == int(actor['data_id']), 'Primary ordinal identity differs')
        distance = actor['planned_center_distance_m']
        require(math.isfinite(distance), 'Nonfinite primary distance')
        require(abs(distance - meta['planned_primary_distance_m']) < 1e-8,
                'Primary ordinal/planned distance differs')
        protected_near += spec['distance_bounds'][1] <= 20
        vehicle_far += actor.get('population_pool') in ('vehicle', 'static_vehicle') and distance >= 30
        physical_count += physical(actor)
        generic_far += scope(spec, actor, strict=False) and distance >= 30
        if not scope(spec, actor, strict=False):
            continue
        ref = assets.get(str(actor['data_id']))
        require(ref and ref['name'] == actor['name'], 'Plan primary lacks matching native size reference')
        for variant in ('provisional', 'final_compact'):
            cap = cap_for_size(ref['size_q50_m'], compact=variant == 'final_compact')
            if cap is None:
                continue
            for strict in (False, True):
                if not scope(spec, actor, strict=strict):
                    continue
                detail = dict(id=row['id'], asset=actor['name'], cap_m=cap, split=row['split'],
                    role=actor['actual_ego_role'], planned_distance_m=distance,
                    source_distance_bounds_m=spec['distance_bounds'],
                    front_plan=bool(spec.get('secondary_obstacle_front_support')),
                    avoidance_phase=spec.get('avoidance_phase'))
                name = f"{variant}_{'strict' if strict else 'broad'}"
                if spec['distance_bounds'][1] > cap:
                    eligible[(name, 'metadata_contract')].append(detail)
                if distance >= cap:
                    eligible[(name, 'actual_pose')].append(detail)
    impacts = {f'{variant}_{narrow}': {kind: impact_summary(eligible[(f'{variant}_{narrow}', kind)])
        for kind in ('metadata_contract', 'actual_pose')}
        for variant in ('provisional', 'final_compact') for narrow in ('broad', 'strict')}
    for rows_changed in eligible.values():
        require(all(x['source_distance_bounds_m'][1] > 20 for x in rows_changed),
                'Protected original <=20m source selected')
    return dict(schema='base5_primary_obstacle_range_strict_scope_audit_v1', source_sha256=sources,
        scope=dict(broad='family obstacle / mission false / objectList + obstacle population pool',
                   strict='broad, no avoidance_phase, no shadow_phase, static/standing'),
        checks=dict(plan_frames=len(rows), planned_actor_occurrences=all_actors,
            physical_obstacle_primaries=physical_count, generic_far_primary_scenes=generic_far,
            all_primary_source_upper_le20_preserved=protected_near,
            vehicle_primary_30plus_preserved=vehicle_far, selected_vehicle_primaries=0,
            no_source_files_modified=True), variants=impacts,
        changed_details={f'{key[0]}/{key[1]}': value for key, value in eligible.items()},
        limitations=['Metadata-contract changes and actual pose replacements are distinct.',
            'Completed 5K and v151 are unpaired scenes; observed support is not future capture approval or AP.',
            'Native pixels and exact model512 mask support use different image coordinates.',
            'No native/GT deletion, source overwrite, policy activation, GPU, ROS or MORAI operation.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-release', type=Path, default=NATIVE)
    parser.add_argument('--selected-ledger', type=Path, default=LEDGER)
    parser.add_argument('--support', type=Path, default=SUPPORT)
    parser.add_argument('--manifest', type=Path, default=PLAN)
    parser.add_argument('--expected-frames', type=int, default=5000)
    parser.add_argument('--output', required=True, type=Path, help='Must not already exist')
    args = parser.parse_args()
    require(not args.output.exists(), 'Output must be a new path; existing audits are immutable')
    reference, native = audit_native(args.native_release.resolve(), args.selected_ledger.resolve(),
                                    args.support.resolve(), args.expected_frames)
    plan = audit_plan(args.manifest.resolve(), reference['assets'], args.expected_frames)
    timestamp = datetime.now(KST).isoformat()
    source = dict(path=str(Path(__file__).resolve()), sha256=sha256(__file__))
    args.output.mkdir(parents=True)
    for name, data in (('range_size_reference_reproduced.json', reference),
                       ('native_observation_reproduced.json', native), ('strict_scope.json', plan)):
        data.update(generated_at_kst=timestamp, producer=source)
        (args.output / name).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    counts = {variant: {kind: report['scenes'] for kind, report in reports.items()}
              for variant, reports in plan['variants'].items()}
    print(json.dumps(dict(output=str(args.output.resolve()), native=native['checks'],
                         plan=plan['checks'], impact_counts=counts), indent=2))


if __name__ == '__main__':
    main()
