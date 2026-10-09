"""Opt-in continuous route anchor adapter; old generator and old resume stay frozen.

Plans are stored in each spec so quality replacements use the SAME allowed
spatial stratum, not a new whole-map random anchor. These are road-height/region
screened intervals, not a sensor visibility or physical contact certificate.
"""
from collections import defaultdict
from contextlib import contextmanager
import hashlib
import math
import random

import generate_base5_independent as gen
from base5_map_surface import configure, ego_road_support
from base5_region_contract import location, regions

POLICY = 'continuous-allowed-arc-strata-v141'
ORIGINAL_BUILD = gen.build


def seed_for(environment,slot,seed):
    return int(hashlib.sha256(f'{POLICY}:{seed}:{environment}:{slot}'.encode()).hexdigest()[:12],16)


def split_for(link):
    return 'val' if int(hashlib.sha256(link.encode()).hexdigest()[:8],16)/2**32 < .15 else 'train'


def eligible(spec):
    return (not spec['mission'] and not spec.get('shadow_phase')
            and spec['family'] not in ('highway','roundabout') and spec['topology']=='route_general')


def allowed_intervals(ctx,config):
    configure(ctx,config.get('placement_ground'));regions(gen,ctx)
    out=defaultdict(list)
    for a,b in zip(ctx['arc'][:-1],ctx['arc'][1:]):
        if b-a < 1e-6: continue
        pose=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],float((a+b)/2))
        # Avoid ambiguous seam segments before computing their split contract.
        begin=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],float(a)+1e-7)
        end=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],float(b)-1e-7)
        if begin.get('link_idx') != end.get('link_idx'):continue
        if not ego_road_support(ctx['map_surface_index'],pose)['road_supported']:continue
        area=location(gen,ctx,[pose['x'],pose['y']]);split=split_for(pose['link_idx'])
        for scope in (['background','other'] if area=='other' else ['background']):
            key=(scope,split)
            if out[key] and abs(out[key][-1][1]-a)<1e-7:out[key][-1][1]=float(b)
            else:out[key].append([float(a),float(b)])
    return dict(out)


def slice_intervals(intervals,start,end):
    """Slice valid arc length, not the full route including excluded gaps."""
    if not 0 <= start < end <= sum(b-a for a,b in intervals)+1e-6:
        raise ValueError('Invalid cumulative stratum')
    pieces=[];offset=0.
    for a,b in intervals:
        lo=max(start,offset);hi=min(end,offset+b-a)
        if hi-lo>1e-9:pieces.append([a+lo-offset,a+hi-offset])
        offset += b-a
    if not pieces:raise ValueError('Empty spatial stratum')
    return pieces


def feasible_intervals(ctx,intervals,spec):
    """Screen primary requested range/sector before allocating strata.

    Point proxy only: full build still checks footprint, actual yaw, mixture and
    final ROI. Far route-following obstacles cannot fill a straight-ahead range
    bin at every curved-road anchor; equal whole-route strata were unfillable.
    """
    if spec['family']=='background' or spec.get('avoidance_phase'):return intervals
    lo,hi=spec['distance_bounds'];role=spec['role'];key=(spec['family'],role,lo,hi,
        tuple(tuple(v) for v in intervals))
    cache=ctx.setdefault('_spatial_feasibility_cache',{})
    if key in cache:return cache[key]
    valid=[]
    for start,end in intervals:
        count=max(1,math.ceil((end-start)/2.))
        for i in range(count):
            a=start+(end-start)*i/count;b=start+(end-start)*(i+1)/count
            base=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],(a+b)/2)
            theta=math.radians(base['yaw']);c,s=math.cos(theta),math.sin(theta);ok=False
            for d in (lo+.1,(lo+hi)/2,hi-.1):
                route_target=spec['family']=='obstacle' and role in ('front','rear') and d>15
                for bend in (0.,-15.,15.):
                    if route_target:
                        arc=(a+b)/2+(1 if role=='front' else -1)*d
                        if not 0<=arc<=ctx['arc'][-1]:continue
                        p=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],arc);xy=[p['x'],p['y']]
                    else:
                        angle=theta+math.radians({'front':0,'rear':180,'left':90,'right':-90}[role]+bend)
                        xy=[base['x']+d*math.cos(angle),base['y']+d*math.sin(angle)]
                    dx,dy=xy[0]-base['x'],xy[1]-base['y'];lon=c*dx+s*dy;lat=-s*dx+c*dy
                    actual=('front' if lon>=0 else 'rear') if abs(lon)>=abs(lat) else ('left' if lat>=0 else 'right')
                    if (actual==role and lo-.5<=math.hypot(dx,dy)<=hi+.5 and abs(lat)<29.5
                            and abs(lon)<99.5 and location(gen,ctx,xy)=='other'):
                        ok=True;break
                if ok:break
            if ok:
                if valid and abs(valid[-1][1]-a)<1e-7:valid[-1][1]=b
                else:valid.append([a,b])
    if not valid:raise ValueError('No primary-feasible continuous intervals')
    cache[key]=valid;return valid


def assign_plans(items,intervals,ctx=None):
    buckets=defaultdict(list)
    for slot,spec,split in items:
        if eligible(spec):
            # Balance within compatible range/sector, not across incompatible
            # far-front and near-side layouts on the same curved road.
            layout=(spec['role'],tuple(spec.get('distance_bounds') or []),bool(spec.get('avoidance_phase')))
            buckets[(spec['family'],split,layout)].append((slot,spec))
    for (family,split,layout),values in buckets.items():
        allowed=intervals[('background' if family=='background' else 'other',split)]
        if ctx is not None:allowed=feasible_intervals(ctx,allowed,values[0][1])
        length=sum(b-a for a,b in allowed)
        values.sort(key=lambda v:hashlib.sha256(('spatial-order:'+v[0]).encode()).hexdigest())
        for i,(slot,spec) in enumerate(values):
            center=length*(i+.5)/len(values)
            # A 1.5m-wide bin can be unfillable for a long rear primary near a
            # vehicle-only corridor. Keep stratified centers but allow bounded
            # LOCAL windows, never resample the complete map after a miss.
            windows=[]
            for width in (max(12.,length/len(values)),24.,48.):
                windows.append(slice_intervals(allowed,max(0.,center-width/2),min(length,center+width/2)))
            spec['spatial_sampling_plan']=dict(policy=POLICY,split=split,
                scope='background' if family=='background' else 'other',stratum_index=i,
                stratum_count=len(values),valid_length_m=length,
                quota_intervals_m=slice_intervals(allowed,length*i/len(values),length*(i+1)/len(values)),
                intervals_m=windows[0],local_retry_windows_m=windows,
                retry_window_policy='12m initial / 24m after64 / 48m after256; same allowed split and region')


def sample_anchor(ctx,plan,seed,attempt):
    if plan['policy'] != POLICY:raise ValueError('Unknown spatial policy')
    level=0 if attempt<64 else 1 if attempt<256 else 2
    windows=plan.get('local_retry_windows_m',[plan['intervals_m']]*3)
    pieces=windows[level];length=sum(b-a for a,b in pieces)
    rng=random.Random(seed_for('anchor',str(attempt),seed));distance=rng.uniform(0,length)
    for a,b in pieces:
        if distance <= b-a:
            arc=a+distance;break
        distance -= b-a
    else:arc=pieces[-1][1]
    pose=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],arc)
    if split_for(pose['link_idx']) != plan['split']:raise ValueError('Sampled anchor changed physical-link split')
    if plan['scope']=='other' and location(gen,ctx,[pose['x'],pose['y']])!='other':
        raise ValueError('Sampled anchor crossed reserved region')
    return ctx['route'],ctx['arc'],pose,dict(spatial_stratum='route_general',
        continuous_anchor=dict(policy=POLICY,sampled_arc_m=arc,used_intervals_m=pieces,retry_window_level=level,
            **{k:plan[k] for k in ('scope','stratum_index','stratum_count','split','intervals_m')}))


def build(row,config,ctx,attempt=0):
    plan=row.get('spatial_sampling_plan')
    if not plan:return ORIGINAL_BUILD(row,config,ctx,attempt)
    anchor=sample_anchor(ctx,plan,row['seed'],attempt)
    original=gen.anchor_bank
    def bank(context,topology):
        if topology != 'route_general':raise ValueError('Spatial plan on incompatible topology')
        return [anchor]
    gen.anchor_bank=bank
    try:scene,meta=ORIGINAL_BUILD(row,config,ctx,attempt)
    finally:gen.anchor_bank=original
    if meta['split'] != plan['split']:raise ValueError('Final spatial split changed')
    meta['continuous_ego_sampling']=dict(meta['details']['continuous_anchor'],
        base_xy_m=[meta['base']['x'],meta['base']['y']],
        final_xy_m=[meta['ego']['x'],meta['ego']['y']],
        final_route_arc_m=meta['ego'].get('route_arc_m'),
        final_support_link=meta['ego_ground_initializer']['center'].get('link_id'),
        actual_height_revalidated=True,
        limitation='Anchor strata are continuous; final bypass pose can shift along route. Nearby views can overlap across link splits.')
    return scene,meta


@contextmanager
def installed():
    previous=gen.build;gen.build=build
    try:yield
    finally:gen.build=previous
