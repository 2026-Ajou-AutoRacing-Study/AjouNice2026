"""Opt-in tunnel observation overlay; not a simulated competition traffic rollout."""
import math
from collections import Counter

POLICY = 'tunnel-observation-v120'
APPROACH_CONTRACT = 'outside-to-inside-v122'
STAGE_BOUNDS = {'approach_far': (-50., -30.), 'approach_near': (-30., -10.),
                'approach_threshold': (-10., -.5), 'entrance': (0., 12.),
                'interior': (15., -15.), 'exit': (-12., 15.)}


def stage_bounds(stage, span):
    lo, hi = STAGE_BOUNDS[stage]
    if stage == 'interior': hi += span
    elif stage == 'exit': lo += span; hi += span
    return lo, hi


def project_arc(route, arc, xy, ctx):
    """Continuous route projection; never infer inside/outside from phase names."""
    import numpy as np
    if '_shadow_projection' not in ctx:
        points = np.asarray([(p['x'], p['y']) for p in route])
        delta = points[1:] - points[:-1]
        ctx['_shadow_projection'] = points, delta, np.maximum((delta*delta).sum(1), 1e-12)
    points, delta, denominator = ctx['_shadow_projection']
    t = np.clip(((np.asarray(xy)-points[:-1])*delta).sum(1)/denominator, 0., 1.)
    j = int(((points[:-1]+t[:, None]*delta-np.asarray(xy))**2).sum(1).argmin())
    return float(arc[j]+t[j]*(arc[j+1]-arc[j]))


def assign(rows, config):
    plan=config.get('shadow_observation')
    if not plan:return
    if plan['policy']!=POLICY:raise ValueError('Unknown shadow policy')
    overrides=plan.get('spec_overrides')
    if overrides is None:raise ValueError('Shadow release requires frozen donor spec overrides')
    if len(overrides)!=plan['scenes']:raise ValueError('Shadow override quota mismatch')
    by_index={r['index']:r for r in rows}
    for r in overrides:
        old=by_index[r['index']]
        if old['mission'] or old['family']=='roundabout' or old.get('avoidance_phase') or old.get('vehicle_close_phase'):
            raise ValueError('Protected shadow donor')
        if old['family']!=r['family'] or r['mission']:
            raise ValueError('Shadow override changed family/mission')
        old.clear();old.update(r)


def bounds(gen, ctx):
    from build_team_evaluation_scenarios import gps_shadow_route_bounds
    sample = gen.geo.load_json(gen.HIGHWAY_SAMPLE)
    span, first, last = gps_shadow_route_bounds(sample, ctx['route'], ctx['arc'])
    return float(ctx['arc'][first]), float(ctx['arc'][last]), span


def anchor(gen, ctx, row, config, rng):
    plan = config.get('shadow_observation')
    if not plan or plan['policy'] != POLICY:
        raise ValueError('Shadow overlay requires explicit policy')
    start, end, span = bounds(gen, ctx)
    intervals = {'entrance': (-15., 10.), 'interior': (15., span-15.),
                 'exit': (span-10., span+15.)}
    stage = row.get('shadow_observation_stage')
    lo, hi = stage_bounds(stage, span) if stage else intervals[row['shadow_phase']]
    arc = start+rng.uniform(lo, hi)
    base = gen.geo.continuous_route_pose_at_arc(ctx['route'], ctx['arc'], arc)
    lateral = row.get('shadow_ego_lane_offset_m', 3.5 if row['role']=='right' else 0.)
    if lateral:
        a=math.radians(base['yaw'])
        base=dict(base,x=base['x']-lateral*math.sin(a),y=base['y']+lateral*math.cos(a))
    details = dict(spatial_stratum='tunnel_'+row['shadow_phase'], shadow_observation=dict(
        policy=POLICY, phase=row['shadow_phase'], route_volume_bounds_m=[start, end],
        ego_volume_relative_arc_m=arc-start, source_sample=str(gen.HIGHWAY_SAMPLE),
        source_sample_sha256=gen.geo.sha256(gen.HIGHWAY_SAMPLE),
        limitation='Shade volume is a route anchor, not a measured camera-darkness boundary.'))
    if stage:
        details['shadow_observation'].update(observation_contract=APPROACH_CONTRACT, stage=stage,
            ego_relative_arc_bounds_m=[lo, hi], primary_inside_fraction_bounds=row['shadow_target_fraction_bounds'],
            primary_lane_offset_m=row['shadow_primary_lane_offset_m'],
            ego_lane_offset_m=lateral, layout=row['shadow_layout'])
    return ctx['route'], ctx['arc'], base, details


def pose(gen, ctx, route, arc, base, ego, asset, size, off, row, index, rng):
    """Road-following snapshots; opposite heading is not MGeo reverse driving."""
    lower, upper = row['distance_bounds'] if index == 0 else (8., 55.)
    distance = rng.uniform(lower+.1, upper-.1)
    role = row['role'] if index == 0 else rng.choice(['front', 'rear'])
    if row.get('shadow_observation_stage'):
        start, end, _ = bounds(gen, ctx)
        fraction = row['shadow_target_fraction_bounds'] if index == 0 else [0.10, 0.90]
        low, high = [start+v*(end-start) for v in fraction]
        # Select Ego/object jointly: a central object + an outside Ego can
        # otherwise exceed the 100m input range. Exact radial bin is checked
        # by the normal generator after lane offsets and yaw are applied.
        if index == 0:
            origin = base['route_arc_m']
            interval = (origin+lower+.1, origin+upper-.1) if role == 'front' else (origin-upper+.1, origin-lower-.1)
            low, high = max(low, interval[0]), min(high, interval[1])
        if high <= low: raise ValueError('Shadow stage/target/distance intersection empty')
        goal = rng.uniform(low, high)
        primary_lane = row['shadow_primary_lane_offset_m']
        lateral = primary_lane if index == 0 else (3.5-primary_lane if row['shadow_layout']=='staggered' and index%2 else primary_lane)
    elif role in ('left', 'right'):
        # The real road's adjacent lane, not the 30m-wide roadside background.
        goal = base['route_arc_m']+rng.uniform(-1.5, 1.5)
        lateral = rng.uniform(3.3, 4.1) if role=='left' else rng.uniform(-.15,.15)
    else:
        goal = base['route_arc_m']+(1 if role == 'front' else -1)*distance
        # Moving composite envelopes need the middle of the road strip,
        # not a parallel path along its curb. Static snapshots keep both lanes.
        lateral = 0. if row.get('shadow_moving') else rng.choice([0., 0., 3.5])
    if index==0 and row['shadow_phase']=='interior':
        start,end,_=bounds(gen,ctx)
        if not start<=goal<=end:
            raise ValueError('Interior shadow primary outside tunnel route interval')
    p = gen.geo.continuous_route_pose_at_arc(route, arc, goal)
    a = math.radians(p['yaw'])
    p = dict(p, x=p['x']-math.sin(a)*lateral, y=p['y']+math.cos(a)*lateral)
    # Refuse positions beyond the actual two-lane MGeo road strip. The live
    # collider/ground/visibility audits still decide capture acceptance.
    surface = ctx['map_surface_index'].estimate((p['x'], p['y']), preferred_link=p['link_idx'])
    if surface.get('confidence') not in ('near_centerline', 'road_strip'):
        raise ValueError('Shadow actor center outside supported road strip')
    yaw = p['yaw']
    if row.get('shadow_moving'):
        yaw += 180. if row['shadow_direction_sign'] < 0 else 0.
    elif asset['source_list'] == 'vehicleList':
        yaw += rng.choice([0., 0., 180., 35., -35.])
    else:
        yaw += rng.choice([0., 90., 180., 270.])+rng.uniform(-12., 12.)
    from base5_static_yaw import sample
    static_yaw=sample(asset,row,'walking' if row.get('shadow_moving') else 'static',index,p['yaw'],row.get('_yaw_attempt',0),size=size)
    if static_yaw is not None:
        yaw=static_yaw['placement_yaw_deg']
        p=dict(p,_static_yaw_road_deg=p['yaw'])
    p = dict(p, yaw=yaw)
    a = math.radians(yaw)
    # Select geometric-center observations, not axle/pivot distance bins.
    return gen.pivot_for_center(p, off, yaw)


def validate_approach(gen, ctx, meta):
    """Independent map projection of final XY, including source replacements."""
    row = meta['spec']
    if not row.get('shadow_observation_stage'): return None
    proof = meta['details']['shadow_observation']
    if proof.get('observation_contract') != APPROACH_CONTRACT:
        raise ValueError('Missing shadow approach contract')
    start, end, _ = bounds(gen, ctx); span = end-start
    er = project_arc(ctx['route'], ctx['arc'], [meta['ego']['x'], meta['ego']['y']], ctx)-start
    lo, hi = stage_bounds(row['shadow_observation_stage'], span)
    if not lo-.1 <= er <= hi+.1: raise ValueError('Actual Ego outside shadow stage')
    actors = meta['runtime_actors']; positions=[]
    for index, actor in enumerate(actors):
        fraction = (project_arc(ctx['route'], ctx['arc'], actor['geometric_center_estimate'], ctx)-start)/span
        limits = row['shadow_target_fraction_bounds'] if index == 0 else [.1, .9]
        if not limits[0]-.002 <= fraction <= limits[1]+.002:
            raise ValueError('Actual shadow actor outside inside-target band')
        if actor['planned_center_distance_m'] > 100. or abs(actor['lateral_m']) > 29.5:
            raise ValueError('Actual shadow target outside input ROI')
        if index == 0 and row['shadow_observation_stage'].startswith('approach') and actor['longitudinal_m'] <= 0:
            raise ValueError('Outside Ego lacks forward inside primary')
        positions.append(fraction)
    return dict(stage=row['shadow_observation_stage'], actual_ego_relative_arc_m=er,
                actual_actor_inside_fractions=positions,
                limitation='Planned map geometry, not sensor visibility or planning success.')


def visibility_review(metadata, components):
    """Early approach quotas require a real positive primary ahead, not placement alone."""
    row = metadata['spec']; stage = row.get('shadow_observation_stage')
    if not stage or metadata.get('calibration_only'): return [], None
    actors = metadata['runtime_actors']
    if not actors: return [], dict(stage=stage, empty=True)
    uid = actors[0]['unique_id']
    positive = [c for c in components if c['parent_uid']==uid and c['positive_lidar1_camera0'] and
        all(lo<=v<hi for v,lo,hi in zip(c['center_lidar_m'],[-100.,-30.,-3.],[100.,30.,2.]))]
    forward = any(c.get('center_ego_m',[0.])[0] > 0 for c in positive)
    front_camera = any(v['camera']=='CAMERA_1' and v['crop_320x896_instance_pixels']>=16 and
        min(v['model_extent_px'])>=4 and not c.get('camera_component_ambiguous')
        for c in positive for v in c.get('cameras',[]))
    reasons = ['shadow_approach_visible_forward_primary_deficit'] if stage.startswith('approach') and not forward else []
    return reasons, dict(stage=stage, positive_primary=bool(positive), forward_primary=forward,
        front_camera_primary=front_camera, primary_uid=uid,
        limitation='Native sensor evidence; camera-only/LiDAR-only both allowed, not detector/planner performance.')


def validate_overlay(deck, metas):
    plan = deck['config']['shadow_observation']
    selected = [m for m in metas if m['spec'].get('shadow_phase')]
    if len(selected) != plan['scenes']:
        raise ValueError('Shadow scene quota mismatch')
    if Counter(m['spec']['shadow_phase'] for m in selected) != Counter(plan['phase_counts']):
        raise ValueError('Shadow phase quota mismatch')
    positives = [m for m in selected if m['runtime_actors']]
    if len({m['runtime_actors'][0]['data_id'] for m in positives}) != plan['primary_assets']:
        raise ValueError('Shadow primary asset coverage mismatch')
    if sum(bool(m['spec'].get('shadow_moving')) for m in selected) != plan['moving_scenes']:
        raise ValueError('Shadow moving quota mismatch')
    if any(m['spec']['mission'] or m['spec'].get('avoidance_phase') or
           m['spec'].get('vehicle_close_phase') for m in selected):
        raise ValueError('Shadow donor consumed a protected mission/close cell')
    return dict(scenes=len(selected), primary_assets=plan['primary_assets'],
                phases=dict(Counter(m['spec']['shadow_phase'] for m in selected)),
                moving_scenes=plan['moving_scenes'], empty_scenes=len(selected)-len(positives))
