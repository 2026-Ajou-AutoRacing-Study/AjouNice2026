#!/usr/bin/env python3
"""Twelve static single-actor captures; review assets are not training positives."""
import argparse
import copy
import json
import math
from pathlib import Path

import generate_dynamic_object_capture_set as geo
import build_base5_mr_asset_catalog as assets


def generate(output, targets=None, prefix='b5a'):
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Use a fresh output directory')
    catalog = json.loads(assets.DEFAULT_CATALOG.read_text())
    rows = {r['name']: r for r in catalog['assets']}
    targets = targets or [('CargoBox', 8., 0.), ('WoodBox', 8., 45.),
               ('OBJ_Hyundai_Ioniq', 8., 0.), ('OBJ_Hyundai_Ioniq', 6., 90.),
               ('OBJ_G80_Open_Door', 8., 0.), ('OBJ_G80_Open_Door', 6., 90.),
               ('Dog1', 8., 0.), ('Deer1', 8., 90.), ('Ped_Pet1', 8., 0.),
               ('Bicycle_Female1', 8., 90.), ('BabyStroller1', 8., 0.), ('Bike1', 8., 90.)]
    template = geo.load_json(geo.PROJECT_ROOT / 'dataset/morai_od/scenarios/woodbox_alignment_260920/wba015.json')
    scenes = []
    for name, distance, relative_yaw in targets:
        row = rows[name]
        doc = geo.load_json(assets.DEFAULT_RUNTIME / row['source_file'])
        actor = copy.deepcopy(next(a for a in doc[row['source_list']] if int(a['DataID']) == row['data_id']))
        scene = copy.deepcopy(template)
        geo.clear_actor_lists(scene)
        init = scene['egoVehicle']['initPosition']
        init['vehicleSaveData']['velocity'] = 0.
        geo.zero_vector(init['vehicleSaveData'].get('lastVelocity'))
        scene['egoVehicle']['currentVelocity'] = 0.
        yaw = float(init['rot']['yaw'])
        angle = math.radians(yaw)
        position = dict(x=float(init['pos']['x']) + distance * math.cos(angle),
                        y=float(init['pos']['y']) + distance * math.sin(angle),
                        z=float(template['objectList'][0]['pos']['z']))
        geo.set_actor_world_position(actor, position, yaw + relative_yaw)
        actor['UNIQUEID'] = 9500 + len(scenes)
        actor.update(moveSpeed=0., isWaypointMode=False, waypointEventList=[])
        actor['scale'] = dict(x=1., y=1., z=1.)
        if row['source_list'] in ('pedestrianList', 'aiPedstrianList'):
            actor.update(speed=0., active=True, loop=False, movingDistance=0., movingDistanceAmount=0., animSpeed=0.)
        scene[row['source_list']] = [actor]
        stem = '{}{:02d}'.format(prefix,len(scenes)+1)
        path, meta = output / (stem+'.json'), output / (stem+'_metadata.json')
        geo.write_json(path, scene)
        geo.write_json(meta, dict(runtime_actors=[dict(name=name, asset_name=name,
            data_id=row['data_id'], unique_id=actor['UNIQUEID'], position=position,
            motion='static', **{'class': 'obstacle' if row['source_list']=='objectList' else 'pedestrian'},
            training_class=row['semantic_class'], source_list=row['source_list'])],
            collection_role=row['collection_role'], training_eligible=row['collection_role']=='positive'))
        scenes.append(dict(id=stem, file=stem, file_name=stem, scenario=str(path.resolve()),
                           metadata=str(meta.resolve()), expected_ego=init['pos'], anchor_index=619,
                           split='review', expected_counts=dict(npc=0,
                           pedestrian=int(row['source_list']!='objectList'), obstacle=int(row['source_list']=='objectList')),
                           asset_probe=dict(asset=name, forward_m=distance, relative_yaw_deg=relative_yaw)))
    manifest = dict(schema_version=1, dataset_id='base5_asset_review_261005',
                    scenario_group='base5_asset_review', scenes=scenes,
                    sensor_config=str(geo.PROJECT_ROOT/'configs/sensors/Sensor_competition_local.json'),
                    waypoints='configs/global_path/2026_molit_comp_global_path.txt',
                    environment=dict(weather='Sunny', time='13:00'),
                    capture_policy=dict(captures_per_scene=1, capture_period_sec=.5, ego_target_speed_kph=0., maximum_motion_m=0.),
                    generation=dict(capture_runtime='competition_launcher', requires_lidar_instance_annotation=True,
                                    purpose='Asset GT review only, no model training'))
    geo.write_json(output/'manifest.json', manifest)
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    print(generate(parser.parse_args().output))
