#!/usr/bin/env python3
"""Prepare 16 static highway support probes, offline only and never training.

Reuse the current generator's legal road paths, asset geometry, height and Park
contracts. Lane identity is checked against actual MGeo road-support link IDs,
not inferred from the target's Ego lateral coordinate. No ROS or runtime writes.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path

import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm


ASSETS = ('2021_Hyundai_G80', '2021_Hyundai_Genesis_GV80', '2013_Hyundai_Xcient(P520)')


def specifications():
    result = []
    for asset_index, asset in enumerate(ASSETS):
        for distance_index, distance in enumerate((85, 95)):
            for role_index, role in enumerate(('front', 'rear')):
                result.append(dict(asset=asset, distance_m=distance, role=role,
                    lane='same' if (asset_index + distance_index + role_index) % 2 == 0 else 'adjacent'))
    for role in ('front', 'rear'):
        for lane in ('same', 'adjacent'):
            result.append(dict(asset=ASSETS[0], distance_m=60, role=role, lane=lane))
    return result


def make_row(base, condition, identity, ordinal):
    row = copy.deepcopy(base)
    for key in list(row):
        if ('avoidance' in key or key.startswith('original_') or key in
                ('required_secondary_pool', 'frozen_population_data_ids', 'population_budget_bounds',
                 'population_rng_reference', 'spatial_sampling_plan', 'secondary_obstacle_support',
                 'vehicle_close_phase', 'vehicle_close_observation')):
            row.pop(key)
    distance = condition['distance_m']
    row.update(family='highway', mission=False, role=condition['role'], motion='static',
        topology='highway_three_lane', actors=1, population_mode='generic_single',
        population_layout='single', population_max_actors=1, vehicle_only_population=True,
        primary_pool_override='vehicle', primary_identity=identity, asset_index=0,
        distance_bounds=[distance - 1., distance + 1.], distance_bin='far_vehicle_diagnostic',
        yaw_limit=3, seed=261015101 + ordinal * 100019)
    row['far_vehicle_diagnostic_contract'] = dict(condition, training=False)
    return row


def lane_proof(meta, paths):
    """Return a same/one-neighbor-lane proof, only on named legal mainlines."""
    lookup = {link: lane for lane, links in paths.items() for link in links}
    actor = meta['runtime_actors'][0]
    ego_link = meta['base']['link_idx']
    actor_link = actor.get('map_ground_initializer', {}).get('link_id')
    ego_lane, actor_lane = lookup.get(ego_link), lookup.get(actor_link)
    if ego_lane is None or actor_lane is None:
        return None
    ordering = {lane: i for i, lane in enumerate(paths)}
    difference = abs(ordering[ego_lane] - ordering[actor_lane])
    if difference > 1:
        return None
    delta = (actor['placement_yaw_deg'] - meta['base']['yaw'] + 180) % 360 - 180
    if abs(delta) > 5.:
        return None  # Explicitly keep this diagnostic deck on near-straight road.
    return dict(ego_link=ego_link, actor_link=actor_link, ego_lane=ego_lane,
        actor_lane=actor_lane, lane='same' if difference == 0 else 'adjacent',
        road_heading_difference_deg=delta,
        road_lateral_m=actor['reference_frames']['road_lateral_m'],
        ego_lateral_m=actor['reference_frames']['ego_lateral_m'],
        basis='MGeo mainline membership and actual road-support link; not Ego lateral offset')


def prepare(source, output):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError('A new diagnostic output is required')
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output.resolve() == runtime or runtime in output.resolve().parents:
        raise ValueError('Offline diagnostic must not be written to the simulator runtime')
    source_hash = sha256(source)
    deck = json.loads(source.read_text())
    base_entry = next(e for e in deck['scenes'] if e['family'] == 'highway' and not e['mission'])
    metadata = Path(base_entry['metadata'])
    if not metadata.is_absolute():
        metadata = source.parent / metadata
    base_hash = sha256(metadata)
    base = json.loads(metadata.read_text())['spec']
    config = copy.deepcopy(deck['config'])
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                    spatial.gen.geo.DEFAULT_MGEO), config)
    assets = {a['name']: a for a in ctx['pools']['vehicle']}
    if any(name not in assets for name in ASSETS):
        raise ValueError('Required ordinary NPC vehicle unavailable')
    entries, rejections = [], Counter()
    for ordinal, condition in enumerate(specifications()):
        row = make_row(base, condition, assets[condition['asset']]['data_id'], ordinal)
        for attempt in range(2048):
            try:
                scene, meta = spatial.build(copy.deepcopy(row), config, ctx, attempt)
                proof = lane_proof(meta, spatial.gen.geo.HIGHWAY_LANE_LINK_PATHS)
                if proof is None or proof['lane'] != condition['lane']:
                    rejections['wrong_or_unverified_lane_or_road_bend'] += 1
                    continue
                actor = meta['runtime_actors'][0]
                if (len(meta['runtime_actors']) != 1 or actor['source_list'] != 'vehicleList'
                        or actor['name'] != condition['asset'] or actor['motion'] != 'static'
                        or actor['actual_ego_role'] != condition['role']):
                    raise RuntimeError('Diagnostic vehicle contract changed')
                break
            except ValueError as error:
                rejections[str(error)] += 1
        else:
            raise ValueError('Unfillable diagnostic: ' + str(condition))
        ident = f'far_{ordinal:02}_{condition["distance_m"]:02}m_{condition["role"]}_{condition["lane"]}'
        meta['far_vehicle_diagnostic'] = dict(condition, lane_proof=proof,
            actual_center_distance_m=actor['planned_center_distance_m'],
            source_manifest_sha256=source_hash, not_training=True,
            native_acceptance_unchanged=True,
            limitation='Factor-matched conditions; Ego anchors differ. Not an exact-pose causal A/B or detector AP.')
        scenario_path = output / 'scenarios' / (ident + '.json')
        meta_path = output / 'scenarios' / (ident + '_metadata.json')
        atomic_json(scenario_path, scene)
        atomic_json(meta_path, meta)
        entries.append(dict(id=ident, scenario=str(scenario_path.resolve()),
            metadata=str(meta_path.resolve()), scenario_sha256=sha256(scenario_path),
            split=meta['split'], family='highway', mission=False,
            distance_bounds=row['distance_bounds'], role=condition['role'], motion='static',
            subfamily=None, scene_group_id=f'diagnostic261015:{ident}'))
    if source_hash != sha256(source) or base_hash != sha256(metadata):
        raise ValueError('Source changed during offline preparation')
    config.update(environment_tag='qualification_sunny13',
        collection_environment=dict(weather='sunny', hour=13,
            policy='Operator-confirmed Sunny13 diagnostic; generator does not change UI'),
        diagnostic_only=True)
    result = copy.deepcopy(deck)
    result.update(config=config, count=len(entries), scenes=entries,
        status='FAR_VEHICLE_DIAGNOSTIC_ONLY; not training; no automatic quota replacement')
    atomic_json(output / 'manifest.json', result)
    atomic_json(output / 'registration.json', dict(source_manifest_sha256=source_hash,
        source_metadata_sha256=base_hash, script_sha256=sha256(__file__), frames=len(entries),
        assets=ASSETS, rejections=dict(rejections), bulk_started=False,
        purpose='Near-straight highway vehicle support at 60/85/95m, front/rear and real neighboring lanes',
        limitations=['No occluder added: measure clean bootstrap support before harder partial occlusion.',
                     'Static appearance snapshots do not test tracker bootstrap or temporal continuity.',
                     'Use a new deck for failed conditions; generic replacements do not certify this lane contract.']))
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
