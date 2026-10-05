#!/usr/bin/env python3
"""Plan Base5-Static or emit a bounded batch. Never installs/loads/captures."""
import argparse
import copy
import hashlib
import json
import math
import random
from collections import Counter
from pathlib import Path

import generate_dynamic_object_capture_set as geo
import generate_static_classcentric_base3_batch as base3

DEFAULT_CONFIG = geo.PROJECT_ROOT / 'dataset/morai_od/configs/base5_static_v1.json'


def deck(values, count, seed):
    result = [values[i % len(values)] for i in range(count)]
    random.Random(seed).shuffle(result)
    return result


def validate_config(config):
    if config['captures_per_scene'] != 1:
        raise ValueError('Static snapshots must not pad the dataset with identical captures')
    counts = config['families']
    if set(counts) != {'near_obstacle', 'far_vehicle', 'pedestrian', 'background'}:
        raise ValueError('Unsupported family')
    if any(not isinstance(n, int) or n <= 0 for n in counts.values()):
        raise ValueError('Family quotas must be positive integers')
    if sum(counts.values()) != config['target_frames']:
        raise ValueError('Frame target differs from scene quotas')
    if not 1 <= config['maximum_batch_scenes'] <= 64:
        raise ValueError('Batch guard must stay within 1..64')
    if config['split']['train'] + config['split']['val'] != 1:
        raise ValueError('Train/val ratios must sum to one')
    for bins in config['distance_bins_m'].values():
        if any(not 0 < lo < hi for lo, hi in bins):
            raise ValueError('Invalid distance bin')


def specifications(config):
    """Small in-memory design deck; no scenario files or simulator calls."""
    validate_config(config)
    seed = config['seed']
    result = []
    topology = list(geo.HIGHWAY_STATIC_PHASE_VARIANTS) + [
        'highway_three_lane', 'route_general_two_lane_left',
        'route_general_two_lane_right', 'roundabout', 'route_general']
    for fi, (family, count) in enumerate(config['families'].items()):
        # Independent seeded decks avoid lane/distance/asset/yaw correlation.
        zones = deck(['highway_three_lane'] if family == 'far_vehicle' else topology,
                     count, seed + 101 * fi)
        roles = deck(['front', 'left', 'right', 'rear'], count, seed + 1021 + fi)
        bins = deck(list(range(len(config['distance_bins_m'].get(family, [[0, 1]])))),
                    count, seed + 2027 + fi)
        offsets = deck(config['ego_lateral_offsets_m'], count, seed + 3023 + fi)
        yaws = deck(config['obstacle_yaws_deg'], count, seed + 4027 + fi)
        for i in range(count):
            result.append(dict(family=family, family_index=i, topology=zones[i],
                               role=roles[i], distance_bin=bins[i],
                               ego_offset_m=offsets[i], yaw_delta_deg=yaws[i],
                               seed=seed + fi * 100000 + i))
    random.Random(seed + 5003).shuffle(result)
    for i, row in enumerate(result):
        row['index'] = i
    return result


def split_for_group(group, config):
    value = int(hashlib.sha256(group.encode()).hexdigest()[:12], 16) / float(16 ** 12)
    return 'train' if value < config['split']['train'] else 'val'


def load_context(runtime_dir, mgeo_dir):
    loaders = {'near_obstacle': geo.load_runtime_obstacle_pool,
               'far_vehicle': geo.load_runtime_vehicle_pool,
               'pedestrian': geo.load_runtime_pedestrian_pool}
    pools = {name: loader(geo.DEFAULT_ASSET_CATALOG, geo.DEFAULT_ASSET_COMPATIBILITY,
                         runtime_dir) for name, loader in loaders.items()}
    pools['near_obstacle'] = [a for a in pools['near_obstacle']
                             if a.get('ontology_class') == 'Obstacle']
    if not pools['near_obstacle']:
        raise ValueError('No positive Obstacle assets')
    links = {row['idx']: row for row in geo.load_json(mgeo_dir / 'link_set.json')}
    guide, _ = geo.load_route(geo.DEFAULT_ROUTE)
    route = geo.build_mgeo_guided_competition_route(guide,
                                                 list(links.values()))
    # The shared builder returns a route and provenance metadata.
    if isinstance(route, tuple):
        route = route[0]
    return dict(pools=pools, links=links, route=route,
                arc=geo.route_arc_coordinates(route),
                highway_routes=[geo.route_from_link_path(links, path)
                                for path in geo.HIGHWAY_LANE_LINK_PATHS.values()],
                template=geo.load_json(geo.DEFAULT_TEMPLATE))


def build_scene(spec, config, context, all_specs):
    rng = random.Random(spec['seed'])
    family_rows = [r for r in all_specs if r['family'] == spec['family']]
    family = dict(count=len(family_rows), roundabout_phases=base3.ROUNDABOUT_PHASES,
                  global_spatial_stratum_schedule=[r['topology'] for r in family_rows])
    # family_index is preserved even though the global deck is shuffled.
    family['global_spatial_stratum_schedule'] = [r['topology'] for r in sorted(
        family_rows, key=lambda r: r['family_index'])]
    route, arc, anchor, details = geo.static_spatial_target(
        family, spec['family_index'], rng, context['route'], context['arc'], context['links'])
    distance = None
    if spec['family'] != 'background':
        lo, hi = config['distance_bins_m'][spec['family']][spec['distance_bin']]
        distance = rng.uniform(lo, hi)
    if spec['family'] == 'far_vehicle':
        if arc[-1] < 2 * distance + 10:
            raise ValueError('Route too short for far-vehicle placement')
        safe_arc = max(distance + 5, min(arc[-1] - distance - 5, anchor['route_arc_m']))
        anchor = geo.continuous_route_pose_at_arc(route, arc, safe_arc)
    base_pose = dict(anchor)
    # Offset is fixed for the scene. Narrow topology variants use a smaller bound.
    offset = spec['ego_offset_m']
    if 'taper' in spec['topology'] or 'merge' in spec['topology']:
        offset = max(-0.15, min(0.15, offset))
    theta = math.radians(base_pose['yaw'])
    ego_pose = dict(base_pose, x=base_pose['x'] - math.sin(theta) * offset,
                    y=base_pose['y'] + math.cos(theta) * offset)
    scenario = copy.deepcopy(context['template'])
    geo.clear_actor_lists(scenario)
    geo.set_ego_pose(scenario['egoVehicle'], ego_pose, initial_speed_kph=0.)
    # Geographic grouping keeps all asset/yaw variants at one anchor out of both splits.
    group = '{}:{}:{}'.format(base_pose['link_idx'], int(base_pose['x'] // 50),
                              int(base_pose['y'] // 50))
    runtime = []
    observed_design = dict(spec, actual_ego_offset_m=offset, **details)
    if spec['family'] != 'background':
        pool = context['pools'][spec['family']]
        asset_order = deck(list(range(len(pool))), len(family_rows), config['seed'] + 7919)
        asset = pool[asset_order[spec['family_index']]]
        role = spec['role']
        if spec['family'] == 'far_vehicle':
            # Legal route centerline, not a tangent extrapolated off a curved road.
            actor_route, actor_arc = route, arc
            origin_arc = base_pose['route_arc_m']
            direction = 1 if role == 'front' else -1
            if role in ('left', 'right'):
                candidates = []
                for other_route, other_arc in context['highway_routes']:
                    index = geo.nearest_index(other_route, base_pose['x'], base_pose['y'])
                    point = other_route[index]
                    dx, dy = point['x']-base_pose['x'], point['y']-base_pose['y']
                    lateral = -math.sin(theta)*dx + math.cos(theta)*dy
                    if ((role == 'left' and 1.5 < lateral < 9.) or
                            (role == 'right' and -9. < lateral < -1.5)):
                        candidates.append((abs(lateral), other_route, other_arc, other_arc[index]))
                if candidates:
                    _, actor_route, actor_arc, origin_arc = min(candidates, key=lambda r: r[0])
                    direction = rng.choice([-1, 1])
                else:
                    # Edge lane has no outward neighbour. Preserve this fallback in metadata.
                    role = rng.choice(['front', 'rear'])
                    direction = 1 if role == 'front' else -1
            requested = origin_arc + direction * distance
            if not 0 <= requested <= actor_arc[-1]:
                raise ValueError('Requested actor arc outside legal route; choose another scene')
            pose = geo.continuous_route_pose_at_arc(actor_route, actor_arc, requested)
            position = {k: pose[k] for k in ('x', 'y', 'z')}
            yaw = pose['yaw']
            actor = copy.deepcopy(asset['template'])
            geo.set_static_vehicle_actor(actor, pose, position, 8500, yaw_override=yaw)
            saved = actor['initPosition'].get('vehicleSaveData', {})
            saved['velocity'] = 0.
            geo.zero_vector(saved.get('lastVelocity'))
            geo.zero_vector(saved.get('vehicle', {}).get('lastVelocity'))
            scenario['vehicleList'] = [actor]
            position = actor['initPosition']['pos']
            observed_design['actor_route_arc_delta_m'] = direction * distance
        else:
            angles = {'front': 0., 'left': 75., 'right': -75., 'rear': 180.}
            bearing = math.radians(base_pose['yaw'] + angles[role] + rng.uniform(-12., 12.))
            position = dict(x=ego_pose['x'] + distance * math.cos(bearing),
                            y=ego_pose['y'] + distance * math.sin(bearing), z=base_pose['z'])
            yaw = base_pose['yaw'] + spec['yaw_delta_deg']
            actor = copy.deepcopy(asset['template'])
            geo.set_actor_world_position(actor, position, yaw)
            actor['UNIQUEID'] = 8500
            actor['scale'] = dict(x=1., y=1., z=1.)
            if spec['family'] == 'pedestrian':
                actor.update(speed=0., active=True, loop=False, movingDistance=0.,
                             movingDistanceAmount=0., animSpeed=0., isWaypointMode=False,
                             waypointEventList=[])
                scenario['pedestrianList'] = [actor]
            else:
                scenario['objectList'] = [actor]
        cls = {'near_obstacle': 'obstacle', 'far_vehicle': 'vehicle',
               'pedestrian': 'pedestrian'}[spec['family']]
        runtime = [dict(unique_id=8500, data_id=asset['data_id'], name=asset['name'],
                        asset_name=asset['name'], motion='static', **{'class': cls},
                        position={k: float(position[k]) for k in ('x', 'y', 'z')}, yaw_deg=yaw)]
        observed_design.update(asset=asset['name'], role=role,
                               requested_pivot_distance_m=distance,
                               placed_pivot_distance_xy_m=math.hypot(position['x']-ego_pose['x'],
                                                                     position['y']-ego_pose['y']))
        dx, dy = position['x'] - ego_pose['x'], position['y'] - ego_pose['y']
        observed_design.update(placed_forward_m=math.cos(theta)*dx + math.sin(theta)*dy,
                               placed_left_m=-math.sin(theta)*dx + math.cos(theta)*dy)
    return scenario, runtime, group, observed_design


def generate_batch(config, output, start, count, runtime_dir=geo.DEFAULT_RUNTIME_ASSET_DIR,
                   mgeo_dir=geo.DEFAULT_MGEO):
    specs = specifications(config)
    if not 1 <= count <= config['maximum_batch_scenes'] or not 0 <= start < len(specs) or start+count > len(specs):
        raise ValueError('Invalid bounded batch range')
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Refusing to overwrite nonempty output')
    context = load_context(runtime_dir, mgeo_dir)
    # Build everything before writing: a geometry failure leaves no partial batch.
    built = [(spec, build_scene(spec, config, context, specs)) for spec in specs[start:start+count]]
    scenes = []
    for spec, (scenario, actors, group, design) in built:
        stem = 'b5s{:05d}'.format(spec['index'])
        scene_path = output / (stem + '.json')
        meta_path = output / (stem + '_metadata.json')
        geo.write_json(scene_path, scenario)
        geo.write_json(meta_path, dict(runtime_actors=actors, design=design))
        scenes.append(dict(id=stem, file=stem, file_name=stem, scenario=str(scene_path.resolve()),
                           metadata=str(meta_path.resolve()), anchor_index=base_pose_index(scenario, context),
                           expected_ego=scenario['egoVehicle']['initPosition']['pos'],
                           expected_counts=dict(npc=len(scenario['vehicleList']),
                                                pedestrian=len(scenario['pedestrianList']),
                                                obstacle=len(scenario['objectList'])),
                           split=split_for_group(group, config), split_group=group,
                           mission_family=spec['family'], asset_probe=design))
    sensor = geo.PROJECT_ROOT / 'configs/sensors/Sensor_competition_local.json'
    manifest = dict(schema_version=1, dataset_id=config['dataset_id'], scenario_group=config['dataset_id'],
                    waypoints=str(geo.DEFAULT_ROUTE), sensor_config=str(sensor),
                    sensor_config_sha256=geo.sha256(sensor), environment=dict(weather='Sunny', time='13:00'),
                    capture_policy=dict(captures_per_scene=1, capture_period_sec=.5,
                                        ego_target_speed_kph=0., maximum_motion_m=.25),
                    generation=dict(config=config, start=start, count=count, runtime='MR',
                                    asset_pool_counts={k: len(v) for k, v in context['pools'].items()},
                                    asset_pool_names={k: [a['name'] for a in v]
                                                      for k, v in context['pools'].items()},
                                    template_sha256=geo.sha256(geo.DEFAULT_TEMPLATE),
                                    eligibility_sha256=geo.sha256(geo.DEFAULT_ASSET_COMPATIBILITY),
                                    mgeo_links_sha256=geo.sha256(mgeo_dir / 'link_set.json'),
                                    acquisition='SaveSensorData; stationary; geometric Instance GT',
                                    deskew=False, validation_status='offline_generated_NOT_live_validated',
                                    required_live_audits=['Instance annotation present', 'actor UID present',
                                                         'actual GT distance/visibility', 'no collision/ego drift',
                                                         'ground support; lane width; sensor cadence'],
                                    asset_catalog_sha256=geo.sha256(geo.DEFAULT_ASSET_CATALOG)), scenes=scenes)
    geo.write_json(output / 'manifest.json', manifest)
    return output / 'manifest.json'


def base_pose_index(scenario, context):
    p = scenario['egoVehicle']['initPosition']['pos']
    return geo.nearest_index(context['route'], p['x'], p['y'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--count', type=int, default=12)
    parser.add_argument('--runtime-asset-dir', type=Path, default=geo.DEFAULT_RUNTIME_ASSET_DIR)
    parser.add_argument('--mgeo-dir', type=Path, default=geo.DEFAULT_MGEO)
    args = parser.parse_args()
    config = geo.load_json(args.config)
    specs = specifications(config)
    if args.plan_only:
        print(json.dumps(dict(target_frames=len(specs), counts=Counter(r['family'] for r in specs),
                              generated_scenarios=0, maximum_batch_scenes=config['maximum_batch_scenes']), indent=2))
    elif args.output is None:
        parser.error('--output required unless --plan-only')
    else:
        print(generate_batch(config, args.output, args.start, args.count,
                             args.runtime_asset_dir, args.mgeo_dir))


if __name__ == '__main__':
    main()
