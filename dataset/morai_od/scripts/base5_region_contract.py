"""Opt-in placement priors, not inference class masking or GT suppression."""
import math
from collections import Counter

POLICY = 'vehicle-only-highway-roundabout-v123'
QUOTAS = {'highway': (740, 600), 'roundabout': (0, 350),
          'other_vehicle': (80, 50), 'obstacle': (770, 600),
          'pedestrian_related': (830, 250), 'shadow': (500, 0),
          'background': (230, 0)}


def category(row):
    return 'shadow' if row.get('shadow_phase') else row['family']


def foreground_angular_coverage(ego_xy, primary_box, blocker_box):
    """Conservative flat-footprint proxy; not native 3D visibility evidence.

    Only wholly nearer boxes can block the primary. Partial occlusion remains
    allowed, including actual sensor-visible overlapping angular silhouettes.
    """
    ex, ey = ego_xy
    ref = math.atan2(primary_box.centroid.y-ey, primary_box.centroid.x-ex)
    def envelope(box):
        points = list(box.exterior.coords)[:-1]
        angles = [(math.atan2(y-ey,x-ex)-ref+math.pi)%(2*math.pi)-math.pi for x,y in points]
        ranges = [math.hypot(x-ex,y-ey) for x,y in points]
        return min(angles), max(angles), min(ranges), max(ranges)
    plo, phi, pnear, _ = envelope(primary_box)
    blo, bhi, _, bfar = envelope(blocker_box)
    if bfar >= pnear or phi <= plo or bhi-blo >= math.pi:
        return 0.
    return max(0.,min(phi,bhi)-max(plo,blo))/(phi-plo)


def regions(gen, ctx):
    if '_competition_regions' in ctx:
        return ctx['_competition_regions']
    from shapely.geometry import LineString
    from shapely.ops import unary_union
    highway = set(gen.geo.HIGHWAY_RESTRICTED_LINK_IDS)
    for paths in (gen.geo.HIGHWAY_LANE_LINK_PATHS, gen.geo.HIGHWAY_BOTTLENECK_LANE_LINK_PATHS):
        for path in paths.values(): highway.update(path)
    for variants in gen.geo.HIGHWAY_STATIC_PHASE_VARIANTS.values():
        for variant in variants:
            highway.update(variant['route'])
            for path in variant['side_routes']: highway.update(path)
    def corridor(ids, width):
        return unary_union([LineString([p[:2] for p in ctx['links'][key]['points']]).buffer(width)
                            for key in sorted(ids) if key in ctx['links']])
    result = {'highway': corridor(highway, 4.5),
              'roundabout': corridor(gen.roundabout_mission.PATH, 4.5)}
    # Include the Ego approach/exit road, not only the NPC circle.
    points = [p for p, a in zip(ctx['route'], ctx['arc']) if 870 <= a <= 934]
    result['roundabout'] = result['roundabout'].union(
        LineString([(p['x'], p['y']) for p in points]).buffer(4.5))
    from base5_shadow_observation import bounds
    start, end, _ = bounds(gen, ctx)
    points = [p for p, a in zip(ctx['route'], ctx['arc']) if start <= a <= end]
    result['shadow'] = LineString([(p['x'], p['y']) for p in points]).buffer(4.5)
    ctx['_competition_regions'] = result
    return result


def location(gen, ctx, xy):
    from shapely.geometry import Point
    point = Point(*xy[:2])
    areas = regions(gen, ctx)
    # Tunnel assets have their own explicit quota; no highway class restriction.
    for name in ('shadow', 'roundabout', 'highway'):
        if areas[name].covers(point): return name
    return 'other'


def validate(gen, ctx, meta, config):
    if not config.get('regional_policy'): return None
    if config['regional_policy']['policy'] != POLICY:
        raise ValueError('Unknown regional placement policy')
    row = meta['spec']; cat = category(row)
    ego_region = location(gen, ctx, [meta['ego']['x'], meta['ego']['y']])
    if cat not in ('highway', 'roundabout', 'shadow', 'background') and not row['mission']:
        if ego_region != 'other': raise ValueError('Generic Ego in reserved vehicle/shadow region')
    actor_regions = []
    for actor in meta['runtime_actors']:
        region = location(gen, ctx, actor['geometric_center_estimate'])
        actor_regions.append(region)
        if cat in ('highway', 'roundabout') and actor['population_pool'] != 'vehicle':
            raise ValueError('Vehicle-only family has nonvehicle secondary')
        if cat != 'shadow' and region in ('highway', 'roundabout') and actor['population_pool'] != 'vehicle':
            raise ValueError('Nonvehicle actor in reserved traffic region')
        if cat != 'shadow' and region == 'shadow':
            raise ValueError('Nonshadow actor in independent shadow quota region')
    if cat == 'highway' and not row['mission'] and len(meta['runtime_actors']) > 1:
        actors = meta['runtime_actors']
        boxes = [gen.rectangle(a['geometric_center_estimate'], a['estimated_size'], a['placement_yaw_deg']) for a in actors]
        for box in boxes[1:]:
            if foreground_angular_coverage((meta['ego']['x'],meta['ego']['y']),boxes[0],box) >= .95:
                raise ValueError('Generic highway primary nearly fully angular-occluded')
    return dict(policy=POLICY, category=cat, ego_region=ego_region, actor_regions=actor_regions,
                scope='Map corridor placement audit; not collider, camera visibility or inference masking.')


def validate_quotas(metas):
    actual = Counter((category(m['spec']), bool(m['spec']['mission'])) for m in metas)
    expected = Counter({(k, flag): n for k, pair in QUOTAS.items()
                        for flag, n in zip((False, True), pair) if n})
    if actual != expected: raise ValueError('Regional semantic quota mismatch: '+str(actual))
    return {k: {'generic': pair[0], 'mission': pair[1], 'total': sum(pair)} for k, pair in QUOTAS.items()}
