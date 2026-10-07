"""Opt-in Base5 placement adapter; unsupported paths keep the independent QA route."""
import copy
import math
from pathlib import Path

from base5_intersection_ground import POLICY,QA_POLICY,height,load_lookup
from base5_capture_safety import sha256

SCOPE='intersection-only-v116'


def regional_motion(asset,requested,row,config):
    scope=config.get('capture',{}).get('dynamic_region')
    if scope is None:return None
    if scope!=SCOPE:raise ValueError('Unknown dynamic region policy')
    # Observation family is not runtime class. Preserve the asset and quota;
    # only its capture motion changes outside the designated intersection.
    if not (row.get('mission') and row.get('dynamic_observation_phase')):
        return ('static' if requested=='static' else 'standing','outside-intersection-static-appearance-v116')
    if 'NCAP' in asset['name']:
        return ('standing','dummy-fixed-appearance-v116')
    return None


def lookup_data(config,ctx):
    options=config.get('capture',{}).get('intersection_ground_lookup')
    if options is None:return None
    key=(options['path'],options['sha256'])
    cached=ctx.get('_intersection_ground_lookup')
    if cached is None or cached[0]!=key:
        data=load_lookup(Path(key[0]),key[1])
        if data.get('status')!='LIVE_VALIDATED':raise ValueError('Production generation needs live-validated manual lookup')
        # Freeze the evidence used by this generator invocation.
        proof=data['live_evidence']
        if sha256(Path(proof['path']))!=proof['sha256']:raise ValueError('Manual live report changed')
        ctx['_intersection_ground_lookup']=(key,data)
    return ctx['_intersection_ground_lookup'][1]


def apply_path(path,asset,crossing_id,config,ctx):
    data=lookup_data(config,ctx)
    if data is None:return path,'not_requested'
    required=config['capture']['intersection_ground_lookup'].get('require_covered_moving_paths',False)
    if crossing_id is None:
        if required:raise ValueError('Moving actor lacks covered intersection')
        return path,'not_requested'
    key=crossing_id+'/'+asset['name'];profile=data['profiles'].get(key)
    if profile is None:
        if required:raise ValueError('Moving asset lacks live manual profile')
        return path,'profile_unavailable_keep_strict_preparation'
    if profile['data_id']!=asset['data_id']:raise ValueError('Manual lookup/catalog DataID mismatch')
    start,end=path['points'][0],path['points'][-1]
    n=max(1,math.ceil(math.hypot(end['x']-start['x'],end['y']-start['y'])/.5));points=[]
    try:
        target=height(profile,path['target_position']['x'],path['target_position']['y'])
        for i in range(n+1):
            t=i/n;q=height(profile,start['x']+t*(end['x']-start['x']),start['y']+t*(end['y']-start['y']))
            points.append({k:q[k] for k in ('x','y','z')})
    except ValueError as exc:
        if required:raise ValueError('Moving path outside measured editor heights; regenerate same quota') from exc
        return path,'outside_measured_path_keep_strict_preparation'
    result=copy.deepcopy(path);options=config['capture']['intersection_ground_lookup']
    result.update(points=points,ground_path_evidence=dict(policy=POLICY,lookup_file=options['path'],
        lookup_sha256=options['sha256'],profile_key=key),
        limitation='Editor-height initialization; original-capture advisory ground inspection, severe errors still rejected')
    result['target_position']={k:target[k] for k in ('x','y','z')}
    return result,'editor_path_initialization_v116'


def ready(metadata):
    moving=[a for a in metadata.get('runtime_actors',[]) if a.get('motion') not in ('static','standing')]
    return bool(moving) and all(a.get('motion_path',{}).get('ground_path_evidence',{}).get('policy')==POLICY for a in moving)


def finish_metadata(metadata):
    if ready(metadata):
        if not metadata.get('ground_support_check',{}).get('enabled'):
            raise ValueError('Manual prepared paths require original-capture ground audit')
        metadata['ground_support_check']['manual_policy']=QA_POLICY
    return metadata
