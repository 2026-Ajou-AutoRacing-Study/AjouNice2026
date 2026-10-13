#!/usr/bin/env python3
"""Factorized manual-vs-generated G80 placement probes; no simulator commands."""
import argparse
import copy
import math
from pathlib import Path
import generate_dynamic_object_capture_set as geo
import build_base5_mr_asset_catalog as assets


def generate(output):
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Use a fresh output directory')
    manual = geo.load_json(assets.DEFAULT_RUNTIME/'new_obstacle_list.json')
    smoke = geo.load_json(geo.PROJECT_ROOT/'dataset/morai_od/scenarios/woodbox_alignment_260920/wba015.json')
    original = next(a for a in manual['objectList'] if a['DataID']==40100034)
    control = next(a for a in manual['objectList'] if a['DataID']==40100001)
    scenes = []
    variants = [('manual_original_ego', 'manual', False, True, False),
                ('manual_other_ego', 'manual', True, True, False),
                ('manual_waypoint_false_only', 'manual', True, False, False),
                ('manual_initpos_only', 'manual', True, True, True),
                ('smoke_manual_fields', 'smoke', True, True, False),
                ('smoke_automatic_fields', 'smoke', True, False, True),
                ('smoke_waypoint_false_only', 'smoke', True, False, False),
                ('smoke_initpos_only', 'smoke', True, True, True),
                ('manual_original_uid', 'manual', False, True, False)]
    for label, location, move_ego, waypoint, set_internal in variants:
        scene = copy.deepcopy(manual if location=='manual' else smoke)
        geo.clear_actor_lists(scene)
        actor = copy.deepcopy(original)
        if location=='manual':
            pos = {k:float(original['pos'][k]) for k in ('x','y','z')}
            yaw = float(original['rot']['yaw'])
        else:
            init = smoke['egoVehicle']['initPosition']
            yaw = float(init['rot']['yaw'])
            angle = math.radians(yaw)
            pos = dict(x=float(init['pos']['x'])+8*math.cos(angle),
                       y=float(init['pos']['y'])+8*math.sin(angle),
                       z=float(smoke['objectList'][0]['pos']['z']))
            actor['pos'] = geo.formatted_vector(**pos)
            actor['rot'] = geo.formatted_rotation(yaw)
        if move_ego:
            angle = math.radians(yaw)
            ego_pose = dict(x=pos['x']-8*math.cos(angle),y=pos['y']-8*math.sin(angle),
                            z=float(scene['egoVehicle']['initPosition']['pos']['z']),yaw=yaw)
            init=scene['egoVehicle']['initPosition']
            init['initPositionMode']='Absolute'
            init['pos']=geo.formatted_vector(ego_pose['x'],ego_pose['y'],ego_pose['z'])
            init['rot']=geo.formatted_rotation(yaw)
        scene['egoVehicle']['initPosition']['vehicleSaveData']['velocity']=0.
        geo.zero_vector(scene['egoVehicle']['initPosition']['vehicleSaveData'].get('vehicle',{}).get('lastVelocity'))
        scene['egoVehicle']['currentVelocity']=0.
        if set_internal:
            actor['initPos']=geo.world_to_morai_waypoint(pos)
        actor.update(UNIQUEID=original['UNIQUEID'] if label=='manual_original_uid' else 9700+len(scenes),
                     isWaypointMode=waypoint,moveSpeed=0.,waypointEventList=[])
        scene['objectList']=[actor]
        # CargoBox on a non-overlapping lateral point at the same road patch.
        angle=math.radians(yaw)
        cargo=copy.deepcopy(control)
        cargo_pos=dict(x=pos['x']-4*math.sin(angle),y=pos['y']+4*math.cos(angle),z=pos['z'])
        cargo['pos']=geo.formatted_vector(**cargo_pos)
        cargo['rot']=geo.formatted_rotation(yaw)
        cargo.update(UNIQUEID=9800+len(scenes),moveSpeed=0.,waypointEventList=[])
        scene['objectList'].append(cargo)
        stem='b5g{:02d}'.format(len(scenes)+1)
        path,meta=output/(stem+'.json'),output/(stem+'_metadata.json')
        geo.write_json(path,scene)
        geo.write_json(meta,dict(probe=label,runtime_actors=[
            dict(name='OBJ_G80_Open_Door',asset_name='OBJ_G80_Open_Door',unique_id=actor['UNIQUEID'],
                 data_id=40100034,motion='static',**{'class':'obstacle'},training_class='vehicle',position=pos),
            dict(name='CargoBox',asset_name='CargoBox',unique_id=cargo['UNIQUEID'],data_id=40100001,
                 motion='static',**{'class':'obstacle'},training_class='obstacle',position=cargo_pos)]))
        scenes.append(dict(id=stem,file=stem,file_name=stem,scenario=str(path.resolve()),
                           metadata=str(meta.resolve()),expected_ego=scene['egoVehicle']['initPosition']['pos'],
                           expected_counts=dict(npc=0,pedestrian=0,obstacle=2),anchor_index=619,split='review',
                           asset_probe=dict(label=label,location=location,waypoint=waypoint,set_internal=set_internal)))
    manifest=dict(schema_version=1,dataset_id='base5_g80_placement_probe_261005',scenes=scenes,
                  scenario_group='base5_g80_placement_probe',
                  sensor_config=str(geo.PROJECT_ROOT/'configs/sensors/Sensor_competition_local.json'),
                  waypoints='configs/global_path/2026_molit_comp_global_path.txt',
                  environment=dict(weather='Sunny',time='13:00'),
                  capture_policy=dict(captures_per_scene=1,capture_period_sec=.5,ego_target_speed_kph=0.,maximum_motion_m=0.))
    geo.write_json(output/'manifest.json',manifest)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    print(generate(p.parse_args().output))
