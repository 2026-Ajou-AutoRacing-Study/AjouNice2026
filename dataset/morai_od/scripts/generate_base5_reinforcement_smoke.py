#!/usr/bin/env python3
"""Small Base5 near-field / highway long-range deck; no production replacement."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import generate_dynamic_object_capture_set as geo

ROOT = Path(__file__).resolve().parents[3]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + '\n')


def generate(output, variant='smoke'):
    output = output.resolve()
    if (output/'manifest.json').exists():
        raise FileExistsError('Use a new output directory; preserve existing capture manifests')
    template = geo.load_json(ROOT/'dataset/morai_od/scenarios/woodbox_alignment_260920/wba015.json')
    vehicle_template = geo.load_json(ROOT/'configs/scenario/2026_molit_comp_sample_scene.json')['vehicleList'][0]
    catalog = geo.load_json(geo.DEFAULT_ASSET_CATALOG)
    assets = {a['name']: a['data_id'] for key in ('vehicles', 'obstacles_and_related') for a in catalog[key]}
    links = {x['idx']: x for x in geo.load_json(geo.DEFAULT_MGEO/'link_set.json')}
    # Both paths are legal post-taper paths in the existing Base3 topology contract.
    path_ids = ('A2256W000432', 'A2256W000427', 'A2256W000444')
    samples, arc = geo.link_path_samples(links, path_ids)
    ego_highway = geo.continuous_route_pose_at_arc(samples, arc, 90.)
    specs = []
    for asset in ('CargoBox', 'WoodBox', 'PE_Drum', 'Concrete_Barricade01', '2016_Hyundai_Ioniq'):
        for role, forward, left, yaw in [('approach', 8., 0., 0.), ('side', 0., 3.2, 90.), ('passed', -5., -3.2, 45.)]:
            specs.append(dict(asset=asset, family='nearfield', role=role, forward=forward, left=left, yaw=yaw))
    for asset in ('2016_Hyundai_Ioniq', '2015_Kia_K5'):
        for direction in (1, -1):
            for distance in (30., 40., 60.):
                specs.append(dict(asset=asset, family='highway_range', role='front' if direction == 1 else 'rear',
                                  forward=direction*distance, left=0., yaw=0.))
    if variant == 'diversity':
        specs = []
        for anchor_shift in (0., 15.):
            for asset in ('CargoBox', 'WoodBox', 'PE_Drum', 'Concrete_Barricade01'):
                for role, forward, left, yaw in (
                        ('approach', 6., 0., 0.), ('left', 1., 3.2, 45.),
                        ('right', -1., -3.2, 90.), ('passed', -6., 2.8, 135.)):
                    specs.append(dict(asset=asset, family='nearfield', role=role,
                                      forward=forward, left=left, yaw=yaw, anchor_shift=anchor_shift))
        for ego_arc in (80., 120.):
            for asset in ('2016_Hyundai_Ioniq', '2015_Kia_K5'):
                for direction in (1, -1):
                    for distance in (25., 40., 55.):
                        specs.append(dict(asset=asset, family='highway_range',
                                          role='front' if direction == 1 else 'rear',
                                          forward=direction*distance, left=0., yaw=0., ego_arc=ego_arc))
    rows = []
    for i, spec in enumerate(specs):
        scenario = copy.deepcopy(template)
        geo.clear_actor_lists(scenario)
        uid = 8500+i
        if spec['family'] == 'highway_range':
            ego_arc = spec.get('ego_arc', 90.)
            ego_pose = geo.continuous_route_pose_at_arc(samples, arc, ego_arc)
            geo.set_ego_pose(scenario['egoVehicle'], ego_pose, initial_speed_kph=0.)
            actor_sample = geo.continuous_route_pose_at_arc(samples, arc, ego_arc+spec['forward'])
            position = {k: actor_sample[k] for k in ('x', 'y', 'z')}
        else:
            ego = scenario['egoVehicle']['initPosition']
            theta = math.radians(float(ego['rot']['yaw']))
            shift = spec.get('anchor_shift', 0.)
            ego['pos']['x'] += math.cos(theta)*shift
            ego['pos']['y'] += math.sin(theta)*shift
            for axis in ('x', 'y'):
                ego['pos']['_'+axis] = str(ego['pos'][axis])
            position = dict(x=ego['pos']['x']+math.cos(theta)*spec['forward']-math.sin(theta)*spec['left'],
                            y=ego['pos']['y']+math.sin(theta)*spec['forward']+math.cos(theta)*spec['left'],
                            z=template['objectList'][0]['pos']['z'])
            actor_sample = dict(yaw=float(ego['rot']['yaw']), link_idx=ego['initLink'], point_ratio=ego['initLinkRatio'])
        is_vehicle = spec['asset'] in ('2016_Hyundai_Ioniq', '2015_Kia_K5')
        if is_vehicle:
            actor = copy.deepcopy(vehicle_template)
            actor['DataID'] = assets[spec['asset']]
            geo.set_static_vehicle_actor(actor, actor_sample, position, uid,
                                         yaw_override=actor_sample['yaw']+spec['yaw'])
            saved = actor['initPosition'].get('vehicleSaveData', {})
            saved['velocity'] = 0.
            geo.zero_vector(saved.get('lastVelocity'))
            geo.zero_vector(saved.get('vehicle', {}).get('lastVelocity'))
            scenario['vehicleList'] = [actor]
            pivot = actor['initPosition']['pos']
        else:
            actor = copy.deepcopy(template['objectList'][0])
            actor['DataID'] = assets[spec['asset']]
            actor['UNIQUEID'] = uid
            actor['scale'] = {'x': 1., 'y': 1., 'z': 1.}
            geo.set_actor_world_position(actor, position, actor_sample['yaw']+spec['yaw'])
            scenario['objectList'] = [actor]
            pivot = actor['pos']
        stem = ('b5dv' if variant == 'diversity' else 'b5rf')+'{:02d}'.format(i+1)
        scene_path = output/(stem+'.json')
        metadata_path = output/(stem+'_metadata.json')
        write_json(scene_path, scenario)
        write_json(metadata_path, dict(runtime_actors=[dict(
            name=spec['asset'], asset_name=spec['asset'], unique_id=uid,
            data_id=assets[spec['asset']], motion='static',
            **{'class': 'vehicle' if is_vehicle else 'obstacle'},
            position={k: float(pivot[k]) for k in ('x', 'y', 'z')})]))
        rows.append(dict(id=stem, file=stem, file_name=stem, scenario=str(scene_path), metadata=str(metadata_path),
                         anchor_index=619, expected_ego=scenario['egoVehicle']['initPosition']['pos'],
                         expected_counts=dict(npc=int(is_vehicle), pedestrian=0, obstacle=int(not is_vehicle)),
                         split='val' if (i % 6 == 5 if variant == 'diversity' else i in (2, 5, 8, 11, 17, 23)) else 'train',
                         mission_family=spec['family'], asset_probe=spec))
    sensor = ROOT/'configs/sensors/Sensor_competition_local.json'
    manifest = dict(schema_version=1, dataset_id='base5_reinforcement_smoke_261004',
                    scenario_group='base5_reinforcement_smoke',
                    waypoints='configs/global_path/2026_molit_comp_global_path.txt',
                    sensor_config=str(sensor), sensor_config_sha256=hashlib.sha256(sensor.read_bytes()).hexdigest(),
                    environment=dict(weather='Sunny', time='13:00'),
                    capture_policy=dict(captures_per_scene=3, capture_period_sec=.4, ego_target_speed_kph=5., maximum_motion_m=2.),
                    generation=dict(purpose='input and GT smoke only', split_unit='scene', highway_link_path=list(path_ids)),
                    scenes=rows)
    manifest['generation'].update(asset_catalog=str(geo.DEFAULT_ASSET_CATALOG),
                                  asset_catalog_sha256=hashlib.sha256(geo.DEFAULT_ASSET_CATALOG.read_bytes()).hexdigest(),
                                  capture_runtime='competition_launcher', woodbox='included; measured GT calibration required')
    result = output/'manifest.json'
    if variant == 'diversity':
        manifest['dataset_id'] = 'base5_diversity_pilot_261004'
        manifest['scenario_group'] = 'base5_diversity_pilot'
        manifest['generation'].update(range_contract='pivot arc <=55m; verify corrected centers within +/-60m',
                                      purpose='stationary diversity addon; not dynamic highway coverage')
    write_json(result, manifest)
    if variant == 'diversity':
        return result
    pass_rows = []
    passing_sources = [next(row for row in rows if row['asset_probe']['asset'] == asset
                            and row['asset_probe']['role'] == 'side')
                       for asset in ('CargoBox', 'Concrete_Barricade01')]
    for index, source in enumerate(passing_sources):
        scene = geo.load_json(Path(source['scenario']))
        row = copy.deepcopy(source)
        stem = 'b5ps{:02d}'.format(index+1)
        ego = scene['egoVehicle']['initPosition']
        angle = math.radians(float(ego['rot']['yaw']))
        # Fixed adjacent object: ego stays on the existing route, no cut-in.
        pos = dict(x=float(ego['pos']['x'])+5.*math.cos(angle)-3.2*math.sin(angle),
                   y=float(ego['pos']['y'])+5.*math.sin(angle)+3.2*math.cos(angle),
                   z=template['objectList'][0]['pos']['z'])
        geo.set_actor_world_position(scene['objectList'][0], pos, float(ego['rot']['yaw']))
        row.update(id=stem, file=stem, file_name=stem, scenario=str(output/(stem+'.json')),
                   metadata=str(output/(stem+'_metadata.json')), split='train', mission_family='nearfield_pass_smoke')
        row['asset_probe'].update(role='low_speed_adjacent_pass', forward=5., left=3.2, yaw=0.)
        write_json(Path(row['scenario']), scene)
        meta = geo.load_json(Path(source['metadata']))
        meta['runtime_actors'][0]['position'] = pos
        write_json(Path(row['metadata']), meta)
        pass_rows.append(row)
    passing = copy.deepcopy(manifest)
    passing.update(dataset_id='base5_low_speed_pass_smoke_261004', scenario_group='base5_low_speed_pass_smoke', scenes=pass_rows)
    passing['capture_policy'].update(captures_per_scene=6, capture_spacing_m=1., capture_motion_arm_m=1.,
                                    ego_target_speed_kph=10., maximum_motion_m=25.,
                                    maximum_precontrol_drift_m=.25, route_deviation_limit_m=.75,
                                    minimum_ego_speed_ratio=.5, lookahead_m=3., maximum_steering_rad=.2)
    write_json(output/'pass_manifest.json', passing)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', choices=('smoke', 'diversity'), default='smoke')
    args = parser.parse_args()
    print(generate(args.output, args.variant))
