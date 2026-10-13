"""Stationary close observations with NPC pivots on real, bounded MGeo paths."""
import math
import random


def assign(gen, rows, config):
    plan = config.get('mission_vehicle_close')
    if not plan:
        return
    for family in ('highway', 'roundabout'):
        eligible = [r for r in rows if r['family'] == family and r['mission']
                    and r['distance_bounds'][1] <= plan['eligible_max_center_distance_m']]
        rng = random.Random(config['seed'] + sum(map(ord, family)) + 906)
        rng.shuffle(eligible)
        chosen = eligible[:round(len(eligible) * plan['fraction_percent'][family] / 100)]
        groups = [([r for r in chosen if r['topology'] == 'highway_three_lane'],
                   [('front_side', 1), ('alongside', 1), ('rear_side', 1)]),
                  ([r for r in chosen if r['topology'] != 'highway_three_lane'],
                   [('following_front', 1), ('following_rear', 1)])]
        for group, weights in groups:
            phases = gen.exact_deck(weights, len(group), rng)
            sides = gen.exact_deck([('left', 1), ('right', 1)], len(group), rng)
            for row, phase, side in zip(group, phases, sides):
                row.update(original_vehicle_distance_bounds=list(row['distance_bounds']),
                           original_vehicle_role=row['role'], vehicle_close_phase=phase,
                           vehicle_close_side=side, distance_bounds=[0., 30.],
                           distance_bin='vehicle_surface_close',
                           role=side if phase == 'alongside' else 'rear' if phase in ('rear_side', 'following_rear') else 'front')


def primary_pose(gen, ctx, ego, asset, row, rng):
    size, offset, known = gen.geometry(asset, ctx)
    if not known:
        raise ValueError('Close route vehicle requires measured geometry')
    phase = row['vehicle_close_phase'];side = row['vehicle_close_side']
    angle = math.radians(ego['yaw']);c, s = math.cos(angle), math.sin(angle)
    ego_poly = gen.rectangle([ego['x'] + 1.35*c, ego['y'] + 1.35*s], [4.8, 2.], ego['yaw'])
    if row['family'] == 'highway':
        routes = ctx['sample_corridors']
    else:
        if 'roundabout_close_route' not in ctx:
            route, arc = gen.geo.route_from_link_path(ctx['links'], gen.geo.ROUNDABOUT_NPC_LINK_PATH)
            ctx['roundabout_close_route'] = dict(route=route, arc=arc, path=gen.geo.ROUNDABOUT_NPC_LINK_PATH)
        routes = [ctx['roundabout_close_route']]
    goal_gap = rng.uniform(.5, 1.5)
    goal_lon = 0. if phase == 'alongside' else (4.8 + size[0])*.6 * (-1 if phase in ('rear_side', 'following_rear') else 1)
    candidates = []
    for corridor in routes:
        route, arc = corridor['route'], corridor['arc']
        for j in range(0, len(route), 3):
            p = route[j]
            if math.hypot(p['x'] - ego['x'], p['y'] - ego['y']) > 32:
                continue
            pose = gen.geo.continuous_route_pose_at_arc(route, arc, float(arc[j]))
            # Scenario rotation is serialized to three decimal places. Compute
            # the footprint witness from that same yaw, not an unrounded proxy.
            pose = dict(pose, yaw=round(pose['yaw'], 3))
            pa = math.radians(pose['yaw'])
            center = [pose['x'] + math.cos(pa)*offset[0] - math.sin(pa)*offset[1],
                      pose['y'] + math.sin(pa)*offset[0] + math.cos(pa)*offset[1]]
            dx, dy = center[0] - ego['x'], center[1] - ego['y']
            lon, lat = c*dx + s*dy, -s*dx + c*dy
            delta = abs((pose['yaw'] - ego['yaw'] + 180) % 360 - 180)
            if math.hypot(dx, dy) >= 30 or delta > (25 if row['family'] == 'highway' else 65):
                continue
            if phase in ('front_side', 'alongside', 'rear_side'):
                if not 2 <= abs(lat) <= 5.5 or (lat > 0) != (side == 'left'):
                    continue
                if phase == 'alongside' and abs(lon) > 1.2:
                    continue
                if phase == 'front_side' and not 2 < lon < 15:
                    continue
                if phase == 'rear_side' and not -15 < lon < -2:
                    continue
            elif (lon > 0) != (phase == 'following_front') or abs(lat) > 8:
                continue
            gap = ego_poly.distance(gen.rectangle(center, size, pose['yaw']))
            if not .3 <= gap <= 2.5:
                continue
            score = abs(gap - goal_gap) + .12*abs(lon - goal_lon)
            candidates.append((score, pose, corridor, float(arc[j]), gap, lon, lat))
    if not candidates:
        raise ValueError('No legal close NPC phase at Ego anchor')
    _, pose, corridor, arc_m, gap, lon, lat = min(candidates, key=lambda r: r[0])
    proof = dict(phase=phase, side=side, reference_arc_m=arc_m,
                 route_links=list(corridor['path']), planned_surface_gap_m=gap,
                 target_surface_gap_m=goal_gap, longitudinal_m=lon, lateral_m=lat,
                 npc_pivot_route_exact=True, npc_jitter_applied=False,
                 limitation='Static observation; not collision-safe overtaking control or live ground certification.')
    placement = None
    if row['family'] == 'highway':
        placement = dict(spawn_uid=corridor['spawn_uid'], route_links=corridor['path'],
                         start=corridor['start'], end=corridor['end'], reference_arc_m=arc_m,
                         corridor_length_m=float(corridor['arc'][-1]))
    return pose, placement, proof
