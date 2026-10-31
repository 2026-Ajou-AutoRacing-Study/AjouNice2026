"""Opt-in v151 front support for an existing v145 *added* obstacle only.

This module never adds/removes an actor, changes a primary or edits native GT.
Plans are allocated once over a complete candidate deck and then frozen in the
spec. A pilot/subset and every quality replacement reuse those plans; they must
not call ``assign_plans`` again to rebalance the remaining scenes.

No-plan and unselected actors return the exact legacy support proof. Selected
actors retain free yaw and all existing geometry/visibility/safety checks.
"""
from collections import defaultdict
import copy
import hashlib
import json
import math
from pathlib import Path
import random

from base5_obstacle_rich_floor import POLICY as BASE_POLICY
from base5_obstacle_support_policy import eligible


POLICY = 'generic-obstacle-front-support-v151'
PLAN_KEY = 'secondary_obstacle_front_support'
ASSIGNMENT = 'asset-balanced-hash-order-alternating-half-v1'


def implementation_sha256():
    """Pin this helper too, not only the caller's generator source."""
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def distance_bounds(size):
    """Use the same large-footprint split as v145; no sensor-richness gate."""
    if len(size) != 3 or any(isinstance(v, bool) or not math.isfinite(v) or v <= 0
                             for v in size):
        raise ValueError('Front support requires three finite positive dimensions')
    large = max(size[:2]) > 3 or size[0] * size[1] > 3
    return [8., 14.] if large else [6., 10.]


def _candidate(row, actors):
    """Return the sole v145 added ordinal, or None for an untouched scene."""
    if PLAN_KEY in row:
        raise ValueError('Front support plans are already frozen; do not reassign')
    base = row.get('secondary_obstacle_support', {})
    added = base.get('new_actor_ordinals', [])
    if not added:
        return None
    if (base.get('policy') != BASE_POLICY or not eligible(row)
            or any(a['motion'] not in ('static', 'standing') for a in actors)
            or base.get('primary_preserved') is not True
            or base.get('mission_and_moving_excluded') is not True):
        raise ValueError('Front support is restricted to static v145 generic additions')
    if len(added) != 1 or not isinstance(added[0], int) or isinstance(added[0], bool):
        raise ValueError('Front support requires exactly one added ordinal')
    ordinal = added[0]
    if (not 0 < ordinal == len(actors) - 1 or ordinal not in base.get('ordinals', [])
            or row.get('actors') != len(actors)
            or row.get('frozen_population_data_ids') != [a['data_id'] for a in actors]):
        raise ValueError('Front support cannot change primary or frozen population')
    asset = actors[ordinal]
    if (asset.get('source_list') != 'objectList'
            or not asset.get('population_pool', '').startswith('obstacle')):
        raise ValueError('Front support added actor must be a physical obstacle')
    distance_bounds(asset['estimated_size'])
    return ordinal


def assign_plans(items):
    """Return copied specs keyed by slot for ``(slot, spec, runtime_actors)``.

    Within each asset, deterministic hash ordering and an asset-specific parity
    select floor/ceil(n/2) additions. The asset's selected/unselected counts
    differ by at most one. Neither manifest iteration order nor global RNG
    state affects the result. Existing actor counts, seeds and population IDs
    are copied without modification.
    """
    result, buckets = {}, defaultdict(list)
    source_hash = implementation_sha256()
    for slot, row, actors in items:
        if not isinstance(slot, str) or not slot or slot in result:
            raise ValueError('Front support requires unique nonempty logical slots')
        result[slot] = copy.deepcopy(row)
        ordinal = _candidate(row, actors)
        if ordinal is None:
            continue
        asset = actors[ordinal]
        order = _digest([POLICY, slot, row['seed'], ordinal, asset['data_id']])
        buckets[asset['data_id']].append((order, slot, ordinal, asset))
    for data_id, values in sorted(buckets.items()):
        values.sort(key=lambda v: (v[0], v[1]))
        phase = int(_digest([POLICY, data_id])[:2], 16) % 2
        selected_count = sum((rank + phase) % 2 == 0 for rank in range(len(values)))
        for rank, (_, slot, ordinal, asset) in enumerate(values):
            row = result[slot]
            row[PLAN_KEY] = dict(
                policy=POLICY, base_policy=BASE_POLICY, assignment=ASSIGNMENT,
                implementation_sha256=source_hash, logical_slot_id=slot,
                frozen_seed=row['seed'], ordinal=ordinal, data_id=data_id,
                frozen_population_data_ids=list(row['frozen_population_data_ids']),
                asset_stratum_index=rank, asset_stratum_count=len(values),
                asset_phase=phase, asset_selected_count=selected_count,
                selected=(rank + phase) % 2 == 0, role='front',
                distance_bounds_m=distance_bounds(asset['estimated_size']),
                large_footprint=distance_bounds(asset['estimated_size']) == [8., 14.],
                primary_preserved=True, original_actor_ids_and_count_preserved=True,
                free_yaw_preserved=True, acceptance_unchanged=True)
    return result


def validate_plan(row):
    """Reject a changed frozen assignment before building a replacement."""
    plan = row.get(PLAN_KEY)
    if plan is None:
        return None
    base = row.get('secondary_obstacle_support', {})
    if (plan.get('policy') != POLICY or plan.get('base_policy') != BASE_POLICY
            or plan.get('assignment') != ASSIGNMENT
            or plan.get('implementation_sha256') != implementation_sha256()
            or base.get('policy') != BASE_POLICY or not eligible(row)
            or base.get('primary_preserved') is not True
            or base.get('mission_and_moving_excluded') is not True
            or plan.get('frozen_seed') != row.get('seed')):
        raise ValueError('Frozen front support policy/source/seed mismatch')
    ordinal = plan.get('ordinal')
    ids = row.get('frozen_population_data_ids', [])
    if (not isinstance(ordinal, int) or isinstance(ordinal, bool)
            or not 0 < ordinal == len(ids) - 1
            or base.get('new_actor_ordinals') != [ordinal]
            or ordinal not in base.get('ordinals', [])
            or plan.get('frozen_population_data_ids') != ids
            or plan.get('data_id') != ids[ordinal]
            or not isinstance(plan.get('logical_slot_id'), str)
            or not plan['logical_slot_id']
            or row.get('actors') != len(ids)):
        raise ValueError('Frozen front support actor/count contract mismatch')
    rank, count, phase = (plan.get(k) for k in
                          ('asset_stratum_index', 'asset_stratum_count', 'asset_phase'))
    if (any(not isinstance(v, int) or isinstance(v, bool) for v in (rank, count, phase))
            or not 0 <= rank < count or phase not in (0, 1)
            or phase != int(_digest([POLICY, plan['data_id']])[:2], 16) % 2
            or not isinstance(plan.get('selected'), bool)
            or plan['selected'] != ((rank + phase) % 2 == 0)
            or plan.get('asset_selected_count') != sum(
                (i + phase) % 2 == 0 for i in range(count))):
        raise ValueError('Frozen front support assignment mismatch')
    bounds = [8., 14.] if plan.get('large_footprint') is True else [6., 10.]
    if (not isinstance(plan.get('large_footprint'), bool)
            or plan.get('distance_bounds_m') != bounds or plan.get('role') != 'front'
            or any(plan.get(key) is not True for key in
                   ('primary_preserved', 'original_actor_ids_and_count_preserved',
                    'free_yaw_preserved', 'acceptance_unchanged'))):
        raise ValueError('Frozen front support geometry/preservation mismatch')
    return plan


def apply(row, ordinal, asset, size, attempt, legacy_proof):
    """Override only a selected added actor, using an independent attempt RNG."""
    plan = validate_plan(row)
    if plan is None or ordinal != plan['ordinal'] or not plan['selected']:
        return legacy_proof
    if (asset.get('data_id') != plan['data_id']
            or asset.get('source_list') != 'objectList'
            or asset.get('semantic_class') != 'obstacle'
            or distance_bounds(size) != plan['distance_bounds_m']
            or not isinstance(attempt, int) or isinstance(attempt, bool) or attempt < 0):
        raise ValueError('Front support asset/size/attempt mismatch')
    if (not legacy_proof or legacy_proof.get('policy') != BASE_POLICY
            or legacy_proof.get('ordinal') != ordinal
            or legacy_proof.get('added_actor') is not True
            or legacy_proof.get('count_changed') is not True):
        raise ValueError('Front support requires the legacy added-actor proof')
    key = [POLICY, row['seed'], ordinal, asset['data_id'], attempt]
    rng = random.Random(int(_digest(key)[:16], 16))
    lower, upper = plan['distance_bounds_m']
    result = copy.deepcopy(legacy_proof)
    result.update(role='front', band='near', distance_bounds_m=[lower, upper],
                  target_center_distance_m=rng.uniform(lower + .1, upper - .1),
                  size_aware_large=plan['large_footprint'],
                  front_support=copy.deepcopy(plan))
    return result


def verify_actual(row, ordinal, actual_distance, actual_role):
    """Check actual pose too; a sampled role alone is not directional proof."""
    plan = validate_plan(row)
    if plan is None or ordinal != plan['ordinal'] or not plan['selected']:
        return
    lower, upper = plan['distance_bounds_m']
    if (not math.isfinite(actual_distance) or not lower <= actual_distance < upper
            or actual_role != 'front'):
        raise ValueError('Front support actual pose escaped frozen front/band')
