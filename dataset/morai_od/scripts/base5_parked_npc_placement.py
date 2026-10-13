"""Opt-in road footprint screening of long parked physics NPCs.

This estimates road support from MGeo; native stationary, ground and collision
QA remains mandatory. Passive OBJ assets and people retain their yaw policy.
"""
import math

POLICY = 'long-parked-npc-road-support-v150'
DEFAULT = dict(policy=POLICY, minimum_length_m=8., maximum_nonroad_corners=1,
               maximum_corner_road_distance_m=2.5,
               scopes=['shadow_static', 'stationary_avoidance'])
ROAD_CONFIDENCE = ('road_strip', 'near_centerline')


def screen(asset, motion, size, ground, policy, row=None):
    if not policy:
        return None
    if policy.get('policy') != POLICY:
        raise ValueError('Unknown parked NPC placement policy')
    if row is not None and (row.get('family') in ('highway', 'roundabout', 'background')
            or row.get('static_yaw_policy', {}).get('scope') not in policy['scopes']):
        return None
    if asset['source_list'] != 'vehicleList' or motion != 'static' or size[0] < policy['minimum_length_m']:
        return None
    footprint = (ground or {}).get('footprint', {})
    samples = footprint.get('samples', [])
    corners = samples[1:] if len(samples) == 5 else []
    nonroad = [p for p in corners if not p.get('usable') or p.get('confidence') not in ROAD_CONFIDENCE]
    distances = [p.get('nearest_road_distance_m') for p in corners]
    finite = len(distances) == 4 and all(isinstance(d, (int, float)) and math.isfinite(d) for d in distances)
    supported = (len(corners) == 4 and len(nonroad) <= policy['maximum_nonroad_corners']
                 and finite and max(distances) <= policy['maximum_corner_road_distance_m'])
    return dict(policy=POLICY, data_id=asset['data_id'], road_supported=supported,
                nonroad_corners=len(nonroad), maximum_corner_road_distance_m=max(distances) if finite else None,
                minimum_length_m=policy['minimum_length_m'], estimated_length_m=size[0],
                contact_verified=False,
                limitation='MGeo footprint screening only; not tire support, mesh contact or collider certification')


def actor_screen(actor, policy, row=None):
    return screen(actor, actor['motion'], actor['estimated_size'], actor.get('map_ground_initializer'), policy, row)


def yaw_override(row, asset, ordinal, motion, size=None):
    override = row.get('parked_npc_yaw_overrides', {}).get(str(ordinal))
    if override is None:
        return None
    if (override.get('policy') != POLICY or override.get('mode') != 'road_aligned'
            or override.get('data_id') != asset['data_id'] or asset['source_list'] != 'vehicleList'
            or motion != 'static' or size is None or size[0] < DEFAULT['minimum_length_m']):
        raise ValueError('Invalid explicit long NPC yaw override')
    return override['mode']
