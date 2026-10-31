"""Opt-in, declared primary range limits from repeated native asset dimensions.

This policy never changes a distance inside the builder. A new preparation
freezes explicit spec bounds first; every generator/recovery then validates the
same bounds, source/reference hashes and complete actor identities. Old specs
without a plan remain legacy data and cannot acquire a foreign donor plan.
"""
import copy
import hashlib
import json
import math
from pathlib import Path

import base5_front_completion_adapter as front_completion
from base5_obstacle_support_policy import verify_actual, verify_protected_near


POLICY = 'native-size-primary-physical-obstacle-range-v152'
PLAN_KEY = 'primary_obstacle_range'
CONFIG_KEY = 'primary_obstacle_range_policy'
REFERENCE_SCHEMA = 'base5_primary_obstacle_size_reference_v1'
TIERS = ((.25, 20), (.35, 30), (.60, 40))
MAX_PROPOSALS, RANKED_CANDIDATES, RETRY_BLOCK = 512, 8, 4096
IDENTITY_KEYS = ('data_id', 'unique_id', 'name', 'source_list', 'population_pool', 'motion')
_REFERENCE_CACHE = {}


class RangeContractError(RuntimeError):
    """A source/config/reference/identity violation, never an ordinary deficit."""


class RangeGeometryExhausted(ValueError):
    """Finite geometry search exhausted without relaxing any native QA gate."""


def implementation_sha256():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def classifier():
    return dict(quantity='minimum_axis_of_repeated_native_size_q50_m',
                strict_upper_thresholds_and_caps_m=[list(pair) for pair in TIERS],
                compact_30m_extension=dict(minimum_axis_strict_upper_m=.40,
                                          maximum_xy_strict_upper_m=1.5),
                otherwise='uncapped', provisional=True)


def has_plan(meta):
    return PLAN_KEY in meta['spec']


def _size(size):
    if (not isinstance(size, (list, tuple)) or len(size) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or v <= 0 for v in size)):
        raise RangeContractError('Three finite positive repeated native dimensions required')
    return list(size)


def classify(size):
    size = _size(size)
    minimum = min(size)
    if minimum < .25: return 20
    if minimum < .35 or (minimum < .40 and max(size[:2]) < 1.5): return 30
    return 40 if minimum < .60 else None


def bounds(original, cap):
    if (not isinstance(original, (list, tuple)) or len(original) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in original)
            or not 0 <= original[0] < original[1]):
        raise RangeContractError('Finite nonnegative increasing original range required')
    if cap is not None and cap not in (20, 30, 40):
        raise RangeContractError('Unknown provisional physical primary range cap')
    if cap is None or original[1] <= cap:
        return list(original)
    return [cap - 10, cap] if original[0] >= cap else [original[0], cap]


def configuration(reference):
    pin = dict(path=str(Path(reference).resolve()),
               sha256=hashlib.sha256(Path(reference).read_bytes()).hexdigest())
    _reference(pin)
    return dict(policy=POLICY, source_sha256=implementation_sha256(), reference=pin,
                classifier=classifier(), bulk_started=False)


def _reference(pin):
    if (not isinstance(pin, dict) or set(pin) != {'path', 'sha256'}
            or not Path(pin['path']).is_absolute()):
        raise RangeContractError('Absolute pinned native size reference required')
    raw = Path(pin['path']).read_bytes()
    if hashlib.sha256(raw).hexdigest() != pin['sha256']:
        raise RangeContractError('Native size reference checksum changed')
    key = (pin['path'], pin['sha256'])
    if key not in _REFERENCE_CACHE:
        data = json.loads(raw)
        if data.get('schema') != REFERENCE_SCHEMA or not isinstance(data.get('assets'), dict):
            raise RangeContractError('Repeated native physical asset size reference required')
        _REFERENCE_CACHE[key] = data
    return _REFERENCE_CACHE[key]


def _asset_reference(pin, data_id, name):
    record = _reference(pin)['assets'].get(str(data_id))
    if record is None:
        return None
    if not isinstance(record, dict):
        raise RangeContractError('Malformed repeated native asset size record')
    if record.get('name') != name:
        raise RangeContractError('Native size reference asset identity differs')
    _size(record.get('size_q50_m'))
    observations = record.get('observations')
    if not isinstance(observations, int) or isinstance(observations, bool) or observations < 2:
        raise RangeContractError('At least two actual native observations required')
    for key in ('scale1_observations_verified', 'static_observations_verified',
                'single_component_parent_observations_verified'):
        if key in record and record[key] != observations:
            raise RangeContractError('Reference mixes scale/motion/component observations')
    return record


def identity(actors):
    return [{key: actor.get(key) for key in IDENTITY_KEYS} for actor in actors]


def eligible(row, primary, asset):
    return (row.get('mission') is False and not row.get('shadow_phase')
            and not row.get('avoidance_phase') and row.get('family') == 'obstacle'
            and primary.get('source_list') == asset.get('source_list') == 'objectList'
            and primary.get('data_id') == asset.get('data_id')
            and asset.get('semantic_class') == 'obstacle'
            and primary.get('motion') in ('static', 'standing'))


def attach(row, actors, asset, policy, logical_slot, split):
    """Return a new frozen spec and an explicit decision; never edit the source."""
    if PLAN_KEY in row:
        raise RangeContractError('A primary range plan is already frozen; do not reassign')
    result = copy.deepcopy(row)
    if not actors or not eligible(row, actors[0], asset):
        return result, dict(changed=False, reason='outside_scope')
    record = _asset_reference(policy['reference'], actors[0]['data_id'], actors[0]['name'])
    if record is None:
        return result, dict(changed=False, reason='missing_repeated_native_size_reference')
    cap = classify(record['size_q50_m'])
    target = bounds(row['distance_bounds'], cap)
    if target == row['distance_bounds']:
        return result, dict(changed=False, reason='uncapped_asset' if cap is None else 'already_within_cap', cap_m=cap)
    ids = [actor['data_id'] for actor in actors]
    if row.get('actors') != len(ids) or row.get('frozen_population_data_ids', ids) != ids:
        raise RangeContractError('Source actor population differs from its declared spec')
    result.update(distance_bounds=target, distance_bin=f'primary_native_cap_{cap}m_v152',
                  frozen_population_data_ids=ids)
    plan = dict(policy=POLICY, implementation_sha256=implementation_sha256(),
        reference=copy.deepcopy(policy['reference']), classifier=classifier(),
        logical_slot_id=logical_slot, frozen_seed=row['seed'], split=split,
        primary_data_id=actors[0]['data_id'], primary_name=actors[0]['name'],
        semantic_class='obstacle', source_list='objectList',
        size_q50_m=list(record['size_q50_m']), observations=record['observations'], cap_m=cap,
        original_distance_bounds_m=list(row['distance_bounds']),
        original_distance_bin=row.get('distance_bin'), distance_bounds_m=target,
        distance_bin=result['distance_bin'], frozen_population_data_ids=ids,
        actor_identity=identity(actors), source_spec_sha256=digest(row),
        frozen_spec_fields_sha256=digest(result),
        front_plan_sha256=digest(row.get('secondary_obstacle_front_support')),
        primary_identity_preserved=True, actor_population_preserved=True,
        free_yaw_preserved=True, acceptance_unchanged=True)
    result[PLAN_KEY] = plan
    validate_plan(result, {CONFIG_KEY: policy})
    return result, dict(changed=True, reason='native_size_declared_range_cap', cap_m=cap,
                        original_distance_bounds_m=list(row['distance_bounds']), distance_bounds_m=target)


def _validate(row):
    if PLAN_KEY not in row:
        return None
    plan = row[PLAN_KEY]
    if (not isinstance(plan, dict) or plan.get('policy') != POLICY
            or plan.get('implementation_sha256') != implementation_sha256()
            or plan.get('classifier') != classifier()):
        raise RangeContractError('Frozen primary range helper/policy/classifier mismatch')
    if (row.get('mission') is not False or row.get('shadow_phase') or row.get('avoidance_phase')
            or row.get('family') != 'obstacle' or plan.get('semantic_class') != 'obstacle'
            or plan.get('source_list') != 'objectList'
            or plan.get('frozen_seed') != row.get('seed')
            or plan.get('split') not in ('train', 'val')
            or not isinstance(plan.get('logical_slot_id'), str) or not plan['logical_slot_id']):
        raise RangeContractError('Frozen primary range scope/seed/slot mismatch')
    record = _asset_reference(plan['reference'], plan['primary_data_id'], plan['primary_name'])
    if (record is None or plan.get('size_q50_m') != record['size_q50_m']
            or plan.get('observations') != record['observations']
            or plan.get('cap_m') != classify(record['size_q50_m']) or plan['cap_m'] is None):
        raise RangeContractError('Frozen primary range native asset classification mismatch')
    target = bounds(plan['original_distance_bounds_m'], plan['cap_m'])
    ids = row.get('frozen_population_data_ids')
    actors = plan.get('actor_identity', [])
    if (target == plan['original_distance_bounds_m'] or row.get('distance_bounds') != target
            or plan.get('distance_bounds_m') != target
            or row.get('distance_bin') != f"primary_native_cap_{plan['cap_m']}m_v152"
            or plan.get('distance_bin') != row['distance_bin']
            or not isinstance(ids, list) or not ids
            or plan.get('frozen_population_data_ids') != ids or row.get('actors') != len(ids)
            or [actor.get('data_id') for actor in actors] != ids
            or actors[0].get('data_id') != plan['primary_data_id']
            or actors[0].get('name') != plan['primary_name']
            or actors[0].get('source_list') != 'objectList'
            or actors[0].get('motion') not in ('static', 'standing')):
        raise RangeContractError('Frozen primary range bounds/population mismatch')
    other = {key: value for key, value in row.items() if key != PLAN_KEY}
    if (digest(other) != plan.get('frozen_spec_fields_sha256')
            or digest(row.get('secondary_obstacle_front_support')) != plan.get('front_plan_sha256')
            or any(plan.get(key) is not True for key in ('primary_identity_preserved',
                'actor_population_preserved', 'free_yaw_preserved', 'acceptance_unchanged'))):
        raise RangeContractError('Frozen primary range original spec fields changed')
    return plan


def validate_plan(row, config=None):
    """A plan requires its explicit global config; no hidden distance override."""
    try:
        plan = _validate(row)
        if plan is None:
            return None
        policy = (config or {}).get(CONFIG_KEY)
        if (not isinstance(policy, dict) or policy.get('policy') != POLICY
                or policy.get('source_sha256') != implementation_sha256()
                or policy.get('reference') != plan['reference']
                or policy.get('classifier') != classifier()):
            raise RangeContractError('Primary range plan lacks matching explicit config/reference')
        return plan
    except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
        if isinstance(exc, RangeContractError): raise
        raise RangeContractError(str(exc)) from exc


def verify_preserved(source, actual):
    try:
        if not has_plan(source):
            if has_plan(actual): raise RangeContractError('Foreign primary range plan introduced')
            return None
        plan = _validate(source['spec'])
        _validate(actual['spec'])
        if actual['spec'] != source['spec']:
            raise RangeContractError('Frozen primary range spec/seed/assignment changed')
        if (identity(source['runtime_actors']) != plan['actor_identity']
                or identity(actual['runtime_actors']) != plan['actor_identity']
                or source['split'] != actual['split'] or actual['split'] != plan['split']):
            raise RangeContractError('Frozen primary range runtime identity/motion/split changed')
        actor = actual['runtime_actors'][0]
        distance = actor['planned_center_distance_m']
        lower, upper = plan['distance_bounds_m']
        if not math.isfinite(distance) or not lower <= distance < upper or actor['actual_ego_role'] != actual['spec']['role']:
            raise RangeContractError('Actual primary escaped the declared native-size range/role')
        for ordinal, actor in enumerate(actual['runtime_actors']):
            verify_protected_near(actual['spec'], ordinal, actor['planned_center_distance_m'])
            verify_actual(actor.get('secondary_obstacle_support'), actor['planned_center_distance_m'])
        front_completion.verify_preserved(source, actual)
        return plan
    except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
        if isinstance(exc, RangeContractError): raise
        raise RangeContractError(str(exc)) from exc


def choose(source, config, ctx, calibration, ordinal, ident, known_shas,
           build, proposal_score, novelty):
    """Range-only source-preserving completion; front overlap uses its adapter."""
    if not isinstance(ordinal, int) or isinstance(ordinal, bool) or not 1 <= ordinal <= 5:
        raise RangeContractError('Finite completion ordinal between one and five required')
    validate_plan(source['spec'], config)
    if not has_plan(source): raise RangeContractError('Primary range source plan required')
    verify_preserved(source, source)
    first = source.get('retry', 0) + 1 + (ordinal - 1) * RETRY_BLOCK
    ranked, tried = [], 0
    for retry in range(first, first + MAX_PROPOSALS):
        tried += 1
        validate_plan(source['spec'], config)
        try: scene, meta = build(copy.deepcopy(source['spec']), config, ctx, retry)
        except RangeContractError: raise
        except ValueError: continue
        verify_preserved(source, meta)
        payload = hashlib.sha256((json.dumps(scene, ensure_ascii=False, indent=2) + '\n').encode()).hexdigest()
        if payload in known_shas: continue
        fresh = novelty(meta, [source])
        if not math.isfinite(fresh) or fresh < .25: continue
        proxy = proposal_score(meta, calibration, 'all_actor_visibility')
        if not math.isfinite(proxy['score']): continue
        ranked.append((proxy['score'] + min(1., fresh), -retry, scene, meta, proxy, fresh))
        if len(ranked) >= RANKED_CANDIDATES: break
    if not ranked:
        validate_plan(source['spec'], config)
        raise RangeGeometryExhausted('No distinct primary-range geometry within bounded search')
    _, negative_retry, scene, meta, proxy, fresh = max(ranked, key=lambda item: item[:2])
    meta['flexible_completion'] = dict(policy=POLICY, source_cell_id=ident, attempt=ordinal,
        generation_retry=-negative_retry, retry_block_start=first, proposals_tried=tried,
        valid_candidates=len(ranked), maximum_proposals=MAX_PROPOSALS,
        ranked_candidate_cap=RANKED_CANDIDATES, source_seed=source['spec']['seed'],
        donor_selection='disabled; frozen primary range, source seed and actor identities retained',
        relative_view_novelty=fresh, proxy=proxy, qa_policy='Unchanged native capture QA and safety gates')
    return scene, meta
