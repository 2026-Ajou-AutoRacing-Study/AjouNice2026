"""Opt-in parked static NPC snapshots and actual pose checks, no velocity units."""
import math

POLICY='park-static-snapshot-v119'


def park(actor):
    """Synchronize the three saved gear selectors; do not change geometric pose."""
    actor['initPosition']['gear']=1
    saved=actor['initPosition'].get('vehicleSaveData',{})
    if isinstance(saved.get('inputData'),list) and len(saved['inputData'])>6:
        saved['inputData'][6]=1
    if isinstance(saved.get('gearbox'),dict):
        saved['gearbox']['gearMode']=1
        saved['gearbox']['vehicleMoving']=False


def runtime_review(metadata):
    actors=[a for a in metadata['runtime_actors'] if a.get('stationary_vehicle_policy')==POLICY]
    if not actors:return [],None
    snapshots=metadata.get('capture_runtime_objects',[])
    reasons=[];rows=[]
    for actor in actors:
        positions=[]
        for snapshot in snapshots:
            obj=next((o for o in (snapshot or {}).get('npc_list',[]) if o['unique_id']==actor['unique_id']),None)
            if obj:positions.append(obj['position'])
        distances=[math.hypot(p['x']-actor['position']['x'],p['y']-actor['position']['y']) for p in positions]
        finite=all(math.isfinite(v) for v in distances)
        verified=len(positions)>=2 and finite and max(distances)<=.25
        if len(positions)<2:reasons.append('stationary_vehicle_runtime_pose_unavailable')
        elif not finite or max(distances)>.25:reasons.append('stationary_vehicle_moved_from_placement')
        rows.append(dict(uid=actor['unique_id'],asset=actor['name'],position_samples=len(positions),
            maximum_pivot_xy_error_m=max(distances) if finite and distances else None,verified=verified))
    return sorted(set(reasons)),dict(policy=POLICY,actors=rows,maximum_pivot_xy_error_m=.25,
        limitation='Uses observed pivot positions before/after capture, not assumed velocity units or collider certification.')


def validate_serialized(meta,scene,policy):
    if not policy:return
    if policy!=POLICY:raise ValueError('Unknown static vehicle state policy')
    for runtime in meta['runtime_actors']:
        if runtime['source_list']!='vehicleList':continue
        if runtime.get('stationary_vehicle_policy')!=POLICY:
            raise ValueError('Static NPC runtime pose gate metadata missing')
        actor=next(a for a in scene['vehicleList'] if a['UNIQUEID']==runtime['unique_id'])
        initial=actor['initPosition'];saved=initial.get('vehicleSaveData',{})
        if initial['gear']!=1 or (saved.get('inputData') and saved['inputData'][6]!=1) or (
                saved.get('gearbox') and saved['gearbox']['gearMode']!=1):
            raise ValueError('Static NPC saved gear selectors differ from Park')
        if any(float(actor.get(key,0.))!=0. for key in ('linkVelocity','constantVelocity',
                'desiredVelocity','currentVelocity','viewCurrentVelocity')):
            raise ValueError('Static NPC nonzero requested velocity')
