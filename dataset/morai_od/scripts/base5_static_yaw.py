"""Opt-in static appearance diversity; never rotate a moving actor off its path."""
import hashlib
import random

POLICY = 'static-appearance-yaw-v128'
FINAL_POLICY = 'static-free-sector-yaw-v130'
PED_POOLS = ('person', 'dummy', 'animal', 'pet_person', 'stroller', 'rider')


def final_sample(asset, row, motion, ordinal, road_yaw, attempt=0, size=None):
    """Balanced primary sectors, continuous angles; retry retains its stratum."""
    policy = row['static_yaw_policy']
    scope = policy['scope']
    if scope not in ('generic', 'stationary_avoidance', 'shadow_static'):
        raise ValueError('Unknown final static yaw scope')
    if (motion not in ('static', 'standing') or row.get('shadow_moving')
            or row.get('dynamic_observation_phase') or row.get('roundabout_phase')
            or row.get('family') in ('highway', 'roundabout')):
        return None
    if asset['data_id'] not in policy['eligible_data_ids']:
        return None
    vehicle = asset['source_list'] == 'vehicleList'
    if vehicle and scope == 'generic':
        return None
    obj_vehicle = asset['data_id'] in policy.get('obj_vehicle_data_ids', [])
    aligned_fraction = .7 if vehicle else .3 if obj_vehicle else 0.
    key = f"{row['seed']}:{ordinal}:{asset['data_id']}:{attempt}:{FINAL_POLICY}"
    rng = random.Random(int(hashlib.sha256(key.encode()).hexdigest()[:16], 16))
    mode = policy.get('primary_mode') if ordinal == 0 else None
    from base5_parked_npc_placement import yaw_override
    override = yaw_override(row, asset, ordinal, motion, size)
    if override is not None:
        mode = override
    if mode is None:
        mode = 'road_aligned' if rng.random() < aligned_fraction else 'uniform'
    if mode == 'uniform':
        sector = policy.get('primary_sector') if ordinal == 0 and override is None else None
        sector = rng.randrange(12) if sector is None else sector
        if type(sector) is not int or not 0 <= sector < 12:
            raise ValueError('Static yaw sector must be 0..11')
        offset = -180. + 30.*sector + rng.uniform(.001, 29.999)
    elif mode == 'road_aligned' and aligned_fraction:
        sector = None
        offset = rng.choice((0., 180.)) + rng.uniform(-15., 15.)
    else:
        raise ValueError('Invalid final yaw mode for asset')
    return dict(policy=FINAL_POLICY, scope=scope, mode=mode, sector=sector,
                placement_attempt=attempt, road_relative_yaw_deg=offset,
                reference_road_yaw_deg=road_yaw, placement_yaw_deg=road_yaw+offset,
                moving_path_alignment_changed=False)


def sample(asset, row, motion, ordinal, road_yaw, attempt=0, size=None):
    policy = row.get('static_yaw_policy')
    if not policy:
        return None
    if policy.get('policy') == FINAL_POLICY:
        return final_sample(asset, row, motion, ordinal, road_yaw, attempt, size)
    if policy.get('policy') != POLICY:
        raise ValueError('Unknown static yaw policy')
    if motion not in ('static', 'standing') or asset['source_list'] == 'vehicleList':
        return None
    scope = policy['scope']
    if scope == 'generic':
        # objectList already has uniform yaw. Do not replace that distribution.
        if asset['source_list'] not in ('pedestrianList', 'aiPedstrianList'):
            return None
    elif scope == 'stationary_avoidance':
        if asset['data_id'] not in policy['eligible_data_ids']:
            return None
    else:
        raise ValueError('Unknown static yaw scope')
    # Independent stream: yaw sampling does not shift XY/population RNG draws.
    key = f"{row['seed']}:{ordinal}:{asset['data_id']}:{attempt}:{POLICY}"
    rng = random.Random(int(hashlib.sha256(key.encode()).hexdigest()[:16], 16))
    mode = policy.get('primary_mode') if ordinal == 0 else None
    if mode is None:
        mode = 'cardinal' if rng.random() < .7 else 'uniform'
    if mode == 'cardinal':
        offset = rng.choice((0., 90., 180., -90.)) + rng.uniform(-15., 15.)
    elif mode == 'uniform':
        offset = rng.uniform(-180., 180.)
    else:
        raise ValueError('Unknown static yaw mode')
    return dict(policy=POLICY, scope=scope, mode=mode, placement_attempt=attempt,
                road_relative_yaw_deg=offset, placement_yaw_deg=road_yaw + offset,
                moving_path_alignment_changed=False)
