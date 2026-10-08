"""Opt-in tunnel observation overlay; not a simulated competition traffic rollout."""
import math
from collections import Counter

POLICY = 'tunnel-observation-v120'


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
    lo, hi = intervals[row['shadow_phase']]
    arc = start+rng.uniform(lo, hi)
    base = gen.geo.continuous_route_pose_at_arc(ctx['route'], ctx['arc'], arc)
    if row['role']=='right':
        a=math.radians(base['yaw'])
        base=dict(base,x=base['x']-3.5*math.sin(a),y=base['y']+3.5*math.cos(a))
    details = dict(spatial_stratum='tunnel_'+row['shadow_phase'], shadow_observation=dict(
        policy=POLICY, phase=row['shadow_phase'], route_volume_bounds_m=[start, end],
        ego_volume_relative_arc_m=arc-start, source_sample=str(gen.HIGHWAY_SAMPLE),
        source_sample_sha256=gen.geo.sha256(gen.HIGHWAY_SAMPLE),
        limitation='Shade volume is a route anchor, not a measured camera-darkness boundary.'))
    return ctx['route'], ctx['arc'], base, details


def pose(gen, ctx, route, arc, base, ego, asset, size, off, row, index, rng):
    """Road-following snapshots; opposite heading is not MGeo reverse driving."""
    lower, upper = row['distance_bounds'] if index == 0 else (8., 55.)
    distance = rng.uniform(lower+.1, upper-.1)
    role = row['role'] if index == 0 else rng.choice(['front', 'rear'])
    if role in ('left', 'right'):
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
    p = dict(p, yaw=yaw)
    a = math.radians(yaw)
    # Select geometric-center observations, not axle/pivot distance bins.
    return gen.pivot_for_center(p, off, yaw)


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
