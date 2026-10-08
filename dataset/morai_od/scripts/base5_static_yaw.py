"""Opt-in static appearance diversity; never rotate a moving actor off its path."""
import hashlib
import random

POLICY = 'static-appearance-yaw-v128'
PED_POOLS = ('person', 'dummy', 'animal', 'pet_person', 'stroller', 'rider')


def sample(asset, row, motion, ordinal, road_yaw, attempt=0):
    policy = row.get('static_yaw_policy')
    if not policy:
        return None
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
