#!/usr/bin/env python3
"""Small stationary visibility/GT probe, not a production training campaign."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from generate_dynamic_object_capture_set import (
    set_actor_world_position, set_static_vehicle_actor, zero_vector)

ROOT = Path(__file__).resolve().parents[3]


def generate(output):
    template = json.loads((ROOT/'dataset/morai_od/scenarios/woodbox_alignment_260920/wba015.json').read_text())
    vehicle = json.loads((ROOT/'configs/scenario/2026_molit_comp_sample_scene.json').read_text())['vehicleList'][0]
    sensor = ROOT/'configs/sensors/Sensor_competition_local.json'
    specs = [('cargo_f5', 'CargoBox', 40100001, 5., 0., 0.),
             ('cargo_l3', 'CargoBox', 40100001, 0., 3., 90.),
             ('barrier_l3', 'Concrete_Barricade01', 40100004, 0., 3., 90.)]
    catalog = json.loads((ROOT/'dataset/morai_od/configs/morai_asset_catalog_260807.json').read_text())
    ids = {a['name']: a['data_id'] for a in catalog['obstacles_and_related']}
    specs[2] = ('barrier_l3', 'Concrete_Barricade01', ids['Concrete_Barricade01'], 0., 3., 90.)
    specs += [('car_f'+str(d), '2016_Hyundai_Ioniq', 30100007, float(d), 0., 0.) for d in (30,40,60,80)]
    scenes = []
    output.mkdir(parents=True, exist_ok=True)
    for i,(label,asset,data_id,forward,left,relative_yaw) in enumerate(specs):
        scene = copy.deepcopy(template)
        ego = scene['egoVehicle']['initPosition']
        angle = math.radians(float(ego['rot']['yaw']))
        pos = dict(x=ego['pos']['x']+math.cos(angle)*forward-math.sin(angle)*left,
                   y=ego['pos']['y']+math.sin(angle)*forward+math.cos(angle)*left,
                   z=template['objectList'][0]['pos']['z'])
        scene['vehicleList'] = []
        scene['objectList'] = []
        unique_id = 8100+i
        if asset == '2016_Hyundai_Ioniq':
            actor = copy.deepcopy(vehicle)
            actor['DataID'] = data_id
            sample = dict(yaw=float(ego['rot']['yaw']), link_idx=ego['initLink'], point_ratio=ego['initLinkRatio'])
            set_static_vehicle_actor(actor, sample, pos, unique_id)
            saved = actor['initPosition'].get('vehicleSaveData',{})
            saved['velocity']=0.0
            zero_vector(saved.get('lastVelocity'))
            zero_vector(saved.get('vehicle',{}).get('lastVelocity'))
            scene['vehicleList']=[actor]
            counts=dict(npc=1,pedestrian=0,obstacle=0)
        else:
            actor=copy.deepcopy(template['objectList'][0]);actor['DataID']=data_id;actor['UNIQUEID']=unique_id
            set_actor_world_position(actor,pos,float(ego['rot']['yaw'])+relative_yaw)
            scene['objectList']=[actor]
            counts=dict(npc=0,pedestrian=0,obstacle=1)
        scene['spawnPointList']=[]
        stem='b5vp'+str(i+1).zfill(2)
        path=output/(stem+'.json');path.write_text(json.dumps(scene,indent=2)+'\n')
        metadata_path=output/(stem+'_metadata.json')
        metadata_path.write_text(json.dumps(dict(runtime_actors=[dict(
            unique_id=unique_id,asset_name=asset,data_id=data_id,
            **{'class':'vehicle' if counts['npc'] else 'obstacle'},
            motion='static',position=pos)]),indent=2)+'\n')
        scenes.append(dict(id=stem,file=stem,file_name=stem,scenario=str(path.resolve()),
            anchor_index=619,expected_ego={k:ego['pos'][k] for k in ('x','y','z')},
            expected_counts=counts,split='train',mission_family='base5_visibility_diagnostic',
            asset_probe=dict(label=label,asset_name=asset,data_id=data_id,unique_id=unique_id,
                             forward_m=forward,left_m=left,relative_yaw_deg=relative_yaw),
            metadata=str(metadata_path.resolve())))
    manifest=dict(schema_version=1,dataset_id='base5_visibility_diagnostic_261004',
        scenario_group='base5_visibility_probe',waypoints='configs/global_path/2026_molit_comp_global_path.txt',
        sensor_config=str(sensor),sensor_config_sha256=hashlib.sha256(sensor.read_bytes()).hexdigest(),
        environment=dict(weather='Sunny',time='13:00'),
        capture_policy=dict(captures_per_scene=3,capture_period_sec=.4,ego_target_speed_kph=5.,maximum_motion_m=2.),
        scenes=scenes)
    path=output/'manifest.json';path.write_text(json.dumps(manifest,indent=2)+'\n');return path


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    print(generate(p.parse_args().output))
