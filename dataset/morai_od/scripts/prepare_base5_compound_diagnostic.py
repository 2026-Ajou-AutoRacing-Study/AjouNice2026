#!/usr/bin/env python3
"""Prepare <=24 static compound diagnostics; offline only, not training quota.

Nominal front/side/rear denotes the parent local +X observation angle, not a
certification of each component mesh's human-facing front. Actual road-relative
yaw and observation angles are saved independently. No runtime/ROS writes.
"""
import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path

import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm


STROLLERS = ('BabyStroller1', 'BabyStroller2', 'BabyStroller3')
VIEWS = {
    'front': dict(nominal_road_relative_yaw_deg=180., sector=11),
    'side': dict(nominal_road_relative_yaw_deg=90., sector=8),
    'rear': dict(nominal_road_relative_yaw_deg=0., sector=6),
}
VIEW_TOLERANCE_DEG = 35.


def specifications():
    rows = [dict(asset=asset, pool='stroller', distance_m=distance, view=view)
            for asset in STROLLERS for distance in (6, 12) for view in VIEWS]
    rows += [dict(asset=asset, pool=pool, distance_m=8, view=view)
             for asset, pool in [('Ped_Pet1', 'pet_person'), ('Bike3', 'rider')]
             for view in ('front', 'side')]
    return rows


def make_row(base, condition, identity, ordinal):
    row = copy.deepcopy(base)
    for key in list(row):
        if (key.startswith(('original_', 'shadow_', 'roundabout_', 'crossing_'))
                or 'avoidance' in key or key in (
                    'required_secondary_pool', 'frozen_population_data_ids',
                    'population_budget_bounds', 'population_rng_reference',
                    'secondary_obstacle_support', 'vehicle_close_phase',
                    'vehicle_close_observation', 'stationary_mission_pool',
                    'dynamic_mission_pool', 'dynamic_observation_phase')):
            row.pop(key)
    distance = condition['distance_m']
    row.update(family='pedestrian_related', mission=False, role='front',
        motion='static', topology='route_general', subfamily=condition['pool'],
        actors=1, population_mode='generic_single', population_layout='single',
        population_max_actors=1, primary_pool_override=condition['pool'],
        primary_identity=identity, asset_index=0, yaw_limit=3,
        distance_bounds=[distance-.75, distance+.75],
        distance_bin='compound_diagnostic', seed=261015701+ordinal*100019)
    policy = copy.deepcopy(row['static_yaw_policy'])
    if policy['policy'] != 'static-free-sector-yaw-v130' or identity not in policy['eligible_data_ids']:
        raise ValueError('Source must opt in the current compound static-yaw policy')
    policy.update(scope='generic', primary_mode='uniform',
                  primary_sector=VIEWS[condition['view']]['sector'])
    row['static_yaw_policy'] = policy
    row['compound_diagnostic_contract'] = dict(condition, training=False,
        nominal_road_relative_yaw_deg=VIEWS[condition['view']]['nominal_road_relative_yaw_deg'],
        requested_road_yaw_sector=policy['primary_sector'])
    return row


def wrap_degrees(angle):
    return (angle+180.) % 360. - 180.


def view_proof(metadata, condition):
    actor = metadata['runtime_actors'][0]
    center = actor['geometric_center_estimate']
    ego = metadata['ego']
    dx, dy = ego['x']-center[0], ego['y']-center[1]
    if not all(math.isfinite(x) for x in (dx, dy, actor['placement_yaw_deg'], ego['yaw'])):
        raise ValueError('Nonfinite diagnostic geometry')
    if math.hypot(dx, dy) < .1:
        raise ValueError('Coincident observation and actor')
    actor_to_ego = math.degrees(math.atan2(dy, dx))
    angle = wrap_degrees(actor_to_ego-actor['placement_yaw_deg'])
    view = condition['view']
    error = (abs(angle) if view == 'front' else abs(abs(angle)-90.) if view == 'side'
             else abs(abs(angle)-180.) if view == 'rear' else math.inf)
    if not math.isfinite(error):
        raise ValueError('Unknown diagnostic view')
    return dict(nominal_view=view, actual_actor_local_observer_angle_deg=angle,
        nominal_view_error_deg=error, within_nominal_view=error <= VIEW_TOLERANCE_DEG,
        tolerance_deg=VIEW_TOLERANCE_DEG,
        actual_road_relative_yaw_deg=actor['static_yaw_policy']['road_relative_yaw_deg'],
        actual_placement_yaw_deg=actor['placement_yaw_deg'],
        actual_bearing_from_ego_deg=wrap_degrees(actor_to_ego+180.-ego['yaw']),
        actual_center_distance_m=math.hypot(dx, dy),
        observer_reference='Ego reference point, not camera optical center',
        axis_reference='Parent local +X; child mesh-facing direction is not certified')


def prepare(source, output):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError('A new diagnostic output directory is required')
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output.resolve() == runtime or runtime in output.resolve().parents:
        raise ValueError('Offline preparation must not write into simulator runtime')
    source_hash = sha256(source)
    deck = json.loads(source.read_text())
    base_entry = next(e for e in deck['scenes'] if e['family'] == 'pedestrian_related'
                      and not e['mission'] and e['role'] == 'front')
    source_metadata = Path(base_entry['metadata'])
    if not source_metadata.is_absolute():
        source_metadata = source.parent / source_metadata
    metadata_hash = sha256(source_metadata)
    base = json.loads(source_metadata.read_text())['spec']
    config = copy.deepcopy(deck['config'])
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                    spatial.gen.geo.DEFAULT_MGEO), config)
    assets = {(pool, a['name']): a for pool in ('stroller', 'pet_person', 'rider')
              for a in ctx['pools'][pool]}
    entries, rejections = [], Counter()
    conditions = specifications()
    if len(conditions) > 24:
        raise ValueError('Small diagnostic deck exceeded 24 captures')
    for ordinal, condition in enumerate(conditions):
        asset = assets.get((condition['pool'], condition['asset']))
        if asset is None:
            raise ValueError('Required compound asset unavailable: ' + condition['asset'])
        row = make_row(base, condition, asset['data_id'], ordinal)
        for attempt in range(4096):
            try:
                scene, meta = spatial.build(copy.deepcopy(row), config, ctx, attempt)
                actor = meta['runtime_actors'][0]
                if (len(meta['runtime_actors']) != 1 or actor['name'] != condition['asset']
                        or actor['motion'] not in ('static', 'standing') or actor.get('motion_path')
                        or actor['actual_ego_role'] != 'front'):
                    raise RuntimeError('Single static compound contract changed')
                if row.get('spatial_sampling_plan') and meta['split'] != row['spatial_sampling_plan']['split']:
                    raise RuntimeError('Diagnostic changed physical split')
                proof = view_proof(meta, condition)
                if not proof['within_nominal_view']:
                    rejections['nominal_view_angle_outside_tolerance'] += 1
                    continue
                break
            except ValueError as error:
                rejections[str(error)] += 1
        else:
            raise ValueError('Unfillable diagnostic: ' + str(condition))
        ident = f'compound_{ordinal:02}_{condition["asset"]}_{condition["distance_m"]:02}m_{condition["view"]}'
        meta['compound_diagnostic'] = dict(condition, view_proof=proof,
            source_manifest_sha256=source_hash, not_training=True,
            native_acceptance_unchanged=True,
            limitation='Factor-matched diagnostic, not identical-pose A/B, animation, detector AP or mesh-contact certification')
        scenario_path = output/'scenarios'/(ident+'.json')
        metadata_path = output/'scenarios'/(ident+'_metadata.json')
        atomic_json(scenario_path, scene)
        atomic_json(metadata_path, meta)
        entries.append(dict(id=ident, scenario=str(scenario_path.resolve()),
            metadata=str(metadata_path.resolve()), scenario_sha256=sha256(scenario_path),
            split=meta['split'], family='pedestrian_related', mission=False,
            distance_bounds=row['distance_bounds'], role='front', motion=actor['motion'],
            subfamily=condition['pool'], scene_group_id=f'diagnostic261015:{ident}'))
    if source_hash != sha256(source) or metadata_hash != sha256(source_metadata):
        raise ValueError('Source changed during preparation')
    config.update(environment_tag='qualification_sunny13', diagnostic_only=True,
        collection_environment=dict(weather='sunny', hour=13,
            policy='Operator-confirmed Sunny13; generator does not change UI'))
    result = copy.deepcopy(deck)
    result.update(config=config, count=len(entries), scenes=entries,
        status='COMPOUND_DIAGNOSTIC_ONLY; not training; no automatic quota replacement')
    atomic_json(output/'manifest.json', result)
    atomic_json(output/'registration.json', dict(source_manifest_sha256=source_hash,
        source_metadata_sha256=metadata_hash, script_sha256=sha256(__file__), frames=len(entries),
        conditions=conditions, rejections=dict(rejections), bulk_started=False,
        purpose='Separate parent coverage, same-class component correspondence and sparse child support',
        limitations=['Nominal parent +X view angles, not a child mesh-facing front guarantee.',
                     'Static snapshots do not validate animation or tracker continuity.',
                     'Existing training quotas, labels and native capture gates are unchanged.']))
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
