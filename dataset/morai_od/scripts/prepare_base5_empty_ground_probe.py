#!/usr/bin/env python3
"""Move stationary Ego back along its route and capture empty terrain, <=8 scenes."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen


def prepare(source,indices,output,backward_route_m=12.):
    if output.exists():raise FileExistsError('Probe overwrite refused')
    if not 1<=len(indices)<=8 or len(indices)!=len(set(indices)):raise ValueError('Ground probe limited to 1–8 cells')
    if not 0<=backward_route_m<=20:raise ValueError('Ground probe backward offset must be 0–20m')
    m=gen.geo.load_json(source);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    output.mkdir(parents=True);rows=[]
    for i in indices:
        old=m['scenes'][i];original=gen.geo.load_json(Path(old['metadata']));scene=gen.geo.load_json(Path(old['scenario']))
        # Current near-general pilots use the ordinary competition route. Refuse
        # an unrelated route rather than silently move to a different road level.
        arc=original['base']['route_arc_m']
        ref=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],arc)
        if gen.math.hypot(ref['x']-original['base']['x'],ref['y']-original['base']['y'])>1.:
            raise ValueError('Source base is not on the ordinary probe route')
        pose=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],max(0.,arc-backward_route_m))
        gen.geo.clear_actor_lists(scene);gen.geo.set_ego_pose(scene['egoVehicle'],pose,initial_speed_kph=0.)
        sid=old['id']+'_terrain';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        meta=copy.deepcopy(original);meta.update(ego=pose,base=pose,runtime_actors=[],planned_primary_distance_m=None,
            calibration_only=True,ground_probe_source=dict(id=old['id'],scenario_sha256=old['scenario_sha256'],
                scenario=old['scenario'],metadata=old['metadata'],targets=original['runtime_actors'],
                source_manifest_sha256=gen.geo.sha256(source),ego_backward_route_m=backward_route_m))
        meta['spec'].update(actors=0,family='background',motion='static')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        row=copy.deepcopy(old);row.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=gen.geo.sha256(sp),family='background',motion='static');rows.append(row)
    m.update(scenes=rows,count=len(rows),status='EMPTY_TERRAIN_CALIBRATION_ONLY; NOT TRAINING DATA')
    gen.geo.write_json(output/'manifest.json',m);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--indices',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(prepare(a.manifest,[int(i) for i in a.indices.split(',')],a.output))
