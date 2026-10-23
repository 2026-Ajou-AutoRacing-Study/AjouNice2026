#!/usr/bin/env python3
"""Offline thin/low/small shape probes using two separated legal route anchors.

These are factor-matched static captures, not exact-pose A/B or training data.
The existing generator controls geometry, road support, Park and native QA.
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


ASSETS = ('Lane_Separation_Bar', 'Bicycle_mountain', 'Bollard04', 'CargoBox')


def specifications():
    return [dict(asset=asset, distance_m=distance, nominal_road_yaw_deg=yaw,
                 source_anchor_index=(asset_index + distance_index) % 2)
            for asset_index, asset in enumerate(ASSETS)
            for distance_index, distance in enumerate((8, 22)) for yaw in (0, 90)]


def yaw_error(actual, nominal):
    return (actual - nominal + 180) % 360 - 180


def make_row(base, condition, identity, ordinal):
    row = copy.deepcopy(base)
    for key in list(row):
        if ('avoidance' in key or key.startswith('original_') or key in
                ('required_secondary_pool', 'frozen_population_data_ids', 'population_budget_bounds',
                 'population_rng_reference', 'secondary_obstacle_support')):
            row.pop(key)
    distance = condition['distance_m']
    row.update(family='obstacle', mission=False, role='front', motion='static',
        actors=1, population_mode='generic_single', population_layout='single',
        population_max_actors=1, primary_pool_override='obstacle', primary_identity=identity,
        asset_index=0, distance_bounds=[distance - 1., distance + 1.],
        distance_bin='near_shape_diagnostic', seed=261015211 + ordinal * 100019)
    # Free sectors remain continuous. Select the positive half, then accept only
    # its first 15 degrees; this does not claim exact 0/90 degree placement.
    sector = 6 if condition['nominal_road_yaw_deg'] == 0 else 9
    policy = copy.deepcopy(row['static_yaw_policy'])
    policy.update(primary_mode='uniform', primary_sector=sector)
    row['static_yaw_policy'] = policy
    row['near_shape_diagnostic_contract'] = dict(condition, training=False)
    return row


def separated_sources(deck):
    candidates = []
    for entry in deck['scenes']:
        if (entry['family'] != 'obstacle' or entry['mission'] or entry['role'] != 'front'
                or entry['distance_bounds'] != [3, 10]):
            continue
        metadata = Path(entry['metadata'])
        meta = json.loads(metadata.read_text())
        if meta['spec'].get('spatial_sampling_plan'):
            candidates.append((entry, meta))
    if not candidates:
        raise ValueError('Continuous generic obstacle anchor unavailable')
    first = candidates[0]
    for candidate in candidates[1:]:
        a, b = first[1]['base'], candidate[1]['base']
        if (a['link_idx'] != b['link_idx'] and
                math.hypot(a['x'] - b['x'], a['y'] - b['y']) >= 150.):
            return (first, candidate)
    raise ValueError('Two physically separated source links unavailable')


def prepare(source, output):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError('A new diagnostic output is required')
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output.resolve() == runtime or runtime in output.resolve().parents:
        raise ValueError('Offline diagnostic must not be written to the simulator runtime')
    source_hash = sha256(source)
    deck = json.loads(source.read_text())
    sources = separated_sources(deck)
    config = copy.deepcopy(deck['config'])
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                    spatial.gen.geo.DEFAULT_MGEO), config)
    assets = {a['name']: a for a in ctx['pools']['obstacle']}
    if any(name not in assets for name in ASSETS):
        raise ValueError('Required diagnostic obstacle unavailable')
    entries, rejections = [], Counter()
    for ordinal, condition in enumerate(specifications()):
        source_entry, base = sources[condition['source_anchor_index']]
        row = make_row(base['spec'], condition, assets[condition['asset']]['data_id'], ordinal)
        for attempt in range(4096):
            try:
                scene, meta = spatial.build(copy.deepcopy(row), config, ctx, attempt)
                actor = meta['runtime_actors'][0]
                actual = actor['static_yaw_policy']['road_relative_yaw_deg']
                if not 0 < yaw_error(actual, condition['nominal_road_yaw_deg']) <= 15.:
                    rejections['yaw_outside_nominal_plus_0_15deg'] += 1
                    continue
                if (len(meta['runtime_actors']) != 1 or actor['name'] != condition['asset']
                        or actor['motion'] != 'static' or meta['split'] != base['split']):
                    raise RuntimeError('Diagnostic actor/physical split contract changed')
                break
            except ValueError as error:
                rejections[str(error)] += 1
        else:
            raise ValueError('Unfillable diagnostic: ' + str(condition))
        ident = f'near_{ordinal:02}_{condition["distance_m"]:02}m_yaw{condition["nominal_road_yaw_deg"]:03}'
        meta['near_shape_diagnostic'] = dict(condition,
            actual_road_relative_yaw_deg=actual, permitted_yaw_delta_deg=[0., 15.],
            actual_center_distance_m=actor['planned_center_distance_m'],
            source_id=source_entry['id'], source_manifest_sha256=source_hash,
            actual_anchor_xy_m=[meta['ego']['x'], meta['ego']['y']], not_training=True,
            native_acceptance_unchanged=True,
            limitation='Factor-matched, separately generated Ego/actor poses; not exact-pose causal A/B or AP.')
        scenario_path = output / 'scenarios' / (ident + '.json')
        meta_path = output / 'scenarios' / (ident + '_metadata.json')
        atomic_json(scenario_path, scene)
        atomic_json(meta_path, meta)
        entries.append(dict(id=ident, scenario=str(scenario_path.resolve()),
            metadata=str(meta_path.resolve()), scenario_sha256=sha256(scenario_path),
            split=meta['split'], family='obstacle', mission=False,
            distance_bounds=row['distance_bounds'], role='front', motion='static', subfamily=None,
            scene_group_id=f'diagnostic261015:{ident}'))
    if source_hash != sha256(source):
        raise ValueError('Source manifest changed during offline preparation')
    config.update(environment_tag='qualification_sunny13', diagnostic_only=True,
        collection_environment=dict(weather='sunny', hour=13,
            policy='Operator-confirmed Sunny13 diagnostic; generator does not change UI'))
    result = copy.deepcopy(deck)
    result.update(config=config, count=len(entries), scenes=entries,
        status='NEAR_SHAPE_DIAGNOSTIC_ONLY; not training')
    atomic_json(output / 'manifest.json', result)
    atomic_json(output / 'registration.json', dict(source_manifest_sha256=source_hash,
        script_sha256=sha256(__file__), frames=len(entries), assets=ASSETS,
        source_anchors=[dict(id=e['id'], metadata_sha256=sha256(e['metadata']),
            link=m['base']['link_idx'], xy_m=[m['base']['x'], m['base']['y']]) for e, m in sources],
        rejections=dict(rejections), bulk_started=False,
        purpose='Native support of thin/low/small obstacles at 8/22m with road-yaw sectors 0..15/90..105deg',
        limitation='Two source links and separated anchors are not a map-wide success-rate certificate.'))
    return output / 'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
