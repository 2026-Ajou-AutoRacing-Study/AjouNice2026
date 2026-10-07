"""Placement-cost budgets, not LiDAR points or visibility certification."""
import math


def cost(size):
    length, width = map(float, size[:2])
    if not all(math.isfinite(v) and v > 0 for v in (length, width)):
        raise ValueError('Invalid footprint for population cost')
    area, longest = length * width, max(length, width)
    if area <= 1. and longest <= 1.5:
        return 2
    if area <= 3. and longest <= 3.:
        return 3
    if area <= 14. and longest <= 6.:
        return 6
    return 8


def choose(primary, poolkey, pools, geometry, row, rng):
    """Keep primary identity; exact budget band sampled once in specs, not retries."""
    mode = row.get('population_mode', 'legacy')
    if row.get('actors') == 0:
        return []
    if mode not in ('generic_regular', 'generic_dense'):
        result = [primary]
        for _ in range(1, row['actors']):
            key = poolkey if mode == 'mission_situation' else rng.choices(
                [poolkey, 'vehicle', 'obstacle', 'person'], weights=[6, 1, 2, 1])[0]
            result.append(rng.choice(pools[key]))
        return result
    lower, upper = row['population_budget_bounds']
    candidates = []
    for key, weight in ((poolkey, 6), ('vehicle', 1), ('obstacle', 2), ('person', 1)):
        for asset in pools[key]:
            candidates.append((asset, weight / len(pools[key]), cost(geometry(asset)[0])))
    result, spent = [primary], cost(geometry(primary)[0])
    while len(result) < row['population_max_actors']:
        if len(result) >= 2 and spent >= lower:
            break
        viable = [(a, w, c) for a, w, c in candidates if spent + c <= upper]
        if not viable:
            raise ValueError('Population budget cannot be filled')
        asset, _, charge = rng.choices(viable, weights=[v[1] for v in viable])[0]
        result.append(asset)
        spent += charge
    if not lower <= spent <= upper or len(result) < 2:
        raise ValueError('Population budget cannot be filled')
    return result


def visibility_summary(actors, components, roi=(-100,-30,-3,100,30,2)):
    """Count parents separately from components; only positive in-input-ROI GT."""
    placed={int(a['unique_id']) for a in actors}
    positive=[]
    for c in components:
        center=c['center_lidar_m']
        if c['positive_lidar1_camera0'] and all(lo<=v<hi for v,lo,hi in zip(center,roi[:3],roi[3:])):
            positive.append(c)
    visible={int(c['parent_uid']) for c in positive} & placed
    return dict(placed_actors=len(placed),visible_positive_actors=len(visible),
        positive_components=len(positive),visible_uids=sorted(visible),
        no_positive_uids=sorted(placed-visible),
        camera_only_components=sum(c['lidar_points']==0 for c in positive),
        limitation='Annotation evidence, not detector accuracy; no new acceptance waiver')


def visibility_review(metadata, summary):
    if metadata.get('calibration_only'):
        return []  # One-actor ground witnesses do not fill a mixed training quota.
    required=metadata.get('population_budget',{}).get('required_visible_actors',0)
    return ['population_visible_actor_deficit'] if summary['visible_positive_actors']<required else []
