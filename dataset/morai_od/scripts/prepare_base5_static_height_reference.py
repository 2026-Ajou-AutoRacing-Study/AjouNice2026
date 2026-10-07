#!/usr/bin/env python3
"""Look at a static actor from nearby; calibration only, never fills quota."""
import argparse
import copy
import math
from pathlib import Path
import generate_base5_independent as gen
from prepare_base5_empty_ground_probe import target_probe_pose
from base5_capture_safety import atomic_json,sha256
from base5_motion_ground import moving_actors


def prepare(source,index,output):
    if output.exists():raise FileExistsError('New reference output required')
    deck=gen.geo.load_json(source);old=deck['scenes'][index]
    scene=gen.geo.load_json(Path(old['scenario']));meta=gen.geo.load_json(Path(old['metadata']))
    if moving_actors(meta) or not meta['runtime_actors']:raise ValueError('Static actor required')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    pose=target_probe_pose(ctx['links'],meta['runtime_actors'][0])
    gen.geo.set_ego_pose(scene['egoVehicle'],pose,initial_speed_kph=0.)
    meta=copy.deepcopy(meta);meta.update(ego=pose,base=pose,calibration_only=True,
        static_height_reference=dict(source_scenario_sha256=old['scenario_sha256'],
            source_metadata_sha256=sha256(Path(old['metadata'])),policy='same actor payload, nearby observation only'))
    meta['spec']['distance_bounds']=[0.,100.]
    for actor in meta['runtime_actors']:
        center=actor['geometric_center_estimate'];actor['planned_center_distance_m']=math.hypot(center[0]-pose['x'],center[1]-pose['y'])
    meta['planned_primary_distance_m']=meta['runtime_actors'][0]['planned_center_distance_m']
    sid=old['id']+'_heightref';output.mkdir(parents=True)
    sp=output/(sid+'.json');mp=output/(sid+'_metadata.json');atomic_json(sp,scene);atomic_json(mp,meta)
    deck.update(count=1,scenes=[dict(old,id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp))],
        status='STATIC_HEIGHT_DIAGNOSTIC_ONLY; NOT TRAINING DATA')
    atomic_json(output/'manifest.json',deck);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--index',type=int,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(prepare(a.manifest,a.index,a.output))
