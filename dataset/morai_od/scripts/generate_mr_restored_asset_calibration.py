#!/usr/bin/env python3
"""Small MR-only geometric-GT calibration deck; never infer boxes from sizes."""
import argparse
import copy
import hashlib
import math
from pathlib import Path

import generate_dynamic_object_capture_set as geo
from generate_base5_reinforcement_smoke import write_json


def generate(output, runtime_dir=None):
    output = output.resolve()
    if (output/'manifest.json').exists():
        raise FileExistsError('Use a new output directory')
    runtime_dir = runtime_dir or geo.DEFAULT_RUNTIME_ASSET_DIR
    arguments = (geo.DEFAULT_ASSET_CATALOG, geo.DEFAULT_ASSET_COMPATIBILITY, runtime_dir)
    pools = {kind: {r['name']: r for r in loader(*arguments)} for kind, loader in (
        ('obstacle', geo.load_runtime_obstacle_pool), ('pedestrian', geo.load_runtime_pedestrian_pool))}
    template = geo.load_json(geo.PROJECT_ROOT/'dataset/morai_od/scenarios/woodbox_alignment_260920/wba015.json')
    targets = [('obstacle', 'CargoBox'), ('obstacle', 'WoodBox'),
               ('pedestrian', 'police_f_1'), ('pedestrian', 'Boy2'), ('obstacle', 'Scooter_01')]
    scenes = []
    for kind, name in targets:
        for distance in (8., 12.):
            for relative_yaw in (0., 90.):
                scene = copy.deepcopy(template)
                geo.clear_actor_lists(scene)
                ego = scene['egoVehicle']['initPosition']
                ego['vehicleSaveData']['velocity'] = 0.
                geo.zero_vector(ego['vehicleSaveData'].get('lastVelocity'))
                angle = math.radians(float(ego['rot']['yaw']))
                position = dict(x=float(ego['pos']['x'])+distance*math.cos(angle),
                                y=float(ego['pos']['y'])+distance*math.sin(angle),
                                z=float(template['objectList'][0]['pos']['z']))
                asset = pools[kind][name]
                actor = copy.deepcopy(asset['template'])
                uid = 9300+len(scenes)
                actor['UNIQUEID'] = uid
                geo.set_actor_world_position(actor, position, float(ego['rot']['yaw'])+relative_yaw)
                if kind == 'pedestrian':
                    actor.update(speed=0., active=True, loop=False, movingDistance=0., movingDistanceAmount=0.)
                    scene['pedestrianList'] = [actor]
                else:
                    actor['scale'] = dict(x=1., y=1., z=1.)
                    scene['objectList'] = [actor]
                stem = 'b5gt{:02d}'.format(len(scenes)+1)
                path, meta = output/(stem+'.json'), output/(stem+'_metadata.json')
                write_json(path, scene)
                write_json(meta, dict(runtime_actors=[dict(
                    name=name, asset_name=name, data_id=asset['data_id'], unique_id=uid,
                    motion='static', position=position, **{'class': kind})]))
                scenes.append(dict(id=stem, file=stem, file_name=stem, scenario=str(path), metadata=str(meta),
                                   expected_ego=ego['pos'], anchor_index=619, split='train',
                                   expected_counts=dict(npc=0, pedestrian=int(kind == 'pedestrian'), obstacle=int(kind == 'obstacle')),
                                   mission_family='geometric_gt_calibration',
                                   asset_probe=dict(asset=name, forward_m=distance, relative_yaw_deg=relative_yaw)))
    sensor = geo.PROJECT_ROOT/'configs/sensors/Sensor_competition_local.json'
    manifest = dict(schema_version=1, dataset_id='base5_mr_restored_gt_calibration_261004',
                    scenario_group='base5_mr_restored_gt_calibration', scenes=scenes,
                    sensor_config=str(sensor), sensor_config_sha256=hashlib.sha256(sensor.read_bytes()).hexdigest(),
                    waypoints='configs/global_path/2026_molit_comp_global_path.txt',
                    environment=dict(weather='Sunny', time='13:00'),
                    capture_policy=dict(captures_per_scene=2, capture_period_sec=.5,
                                        ego_target_speed_kph=0., maximum_motion_m=0.),
                    generation=dict(capture_runtime='competition_launcher', purpose='measured GT reference, not model training',
                                    requires_lidar_instance_annotation=True, ego_heading_correction_deg=-1.347,
                                    controls=['CargoBox']))
    result = output/'manifest.json'
    write_json(result, manifest)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(generate(parser.parse_args().output))
