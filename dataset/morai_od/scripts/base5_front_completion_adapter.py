"""Finite fresh geometry for a frozen v151 assignment, without seed mutation.

The original front-support helper is deliberately unchanged: its source hash is
already part of saved plans. This adapter never reallocates selected strata,
changes an added asset, or treats geometry proxies as native QA approval.
"""
import copy
import hashlib
import json
import math

import base5_obstacle_front_support as front
from base5_obstacle_support_policy import verify_actual, verify_protected_near


POLICY = 'frozen-front-source-geometry-completion-v151'
MAX_PROPOSALS = 512
RANKED_CANDIDATES = 8
RETRY_BLOCK = 4096


class FrozenFrontContractError(ValueError):
    """A frozen identity/source/policy violation, never an ordinary deficit."""


class FrontGeometryExhausted(ValueError):
    """Only the finite new-layout search was exhausted; native QA is unchanged."""


def has_plan(meta):
    return front.PLAN_KEY in meta['spec']


def verify_preserved(source, actual):
    """Verify planned metadata; unchanged native capture QA is still required."""
    try:
        return _verify_preserved(source, actual)
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        raise FrozenFrontContractError(str(exc)) from exc


def _verify_preserved(source, actual):
    if not has_plan(source):
        if has_plan(actual):
            raise ValueError('Completion introduced a foreign frozen front assignment')
        return None
    plan = front.validate_plan(source['spec'])
    if plan is None:
        raise ValueError('Frozen front source contains an empty assignment')
    front.validate_plan(actual['spec'])
    if actual['spec'] != source['spec']:
        raise ValueError('Completion changed the frozen front spec/seed/assignment')
    before, after = source['runtime_actors'], actual['runtime_actors']
    ids = lambda actors: [a['data_id'] for a in actors]
    if (ids(after) != ids(before) or ids(after) != plan['frozen_population_data_ids']
            or len(after) != source['spec']['actors']):
        raise ValueError('Completion changed frozen front actor identities/count')
    identity = lambda actors: [tuple(a.get(key) for key in
        ('data_id', 'unique_id', 'name', 'source_list', 'population_pool')) for a in actors]
    if identity(after) != identity(before):
        raise ValueError('Completion changed frozen front runtime actor identity')
    if ([a['motion'] for a in after] != [a['motion'] for a in before]
            or any(a['motion'] not in ('static', 'standing') for a in after)
            or actual['split'] != source['split']):
        raise ValueError('Completion changed frozen front motion/split')
    for ordinal, actor in enumerate(after):
        distance = actor['planned_center_distance_m']
        if not math.isfinite(distance):
            raise ValueError('Completion contains a nonfinite actor distance')
        verify_protected_near(actual['spec'], ordinal, distance)
        support = actor.get('secondary_obstacle_support')
        verify_actual(support, distance)
        front.verify_actual(actual['spec'], ordinal, distance, actor['actual_ego_role'])
        if ordinal == plan['ordinal']:
            proof = (support or {}).get('front_support')
            if plan['selected'] and proof != plan:
                raise ValueError('Selected completion actor lacks frozen front proof')
            if not plan['selected'] and proof is not None:
                raise ValueError('Unselected completion actor gained a front override')
    return plan


def choose(source, config, ctx, calibration, ordinal, ident, known_shas,
           build, proposal_score, novelty):
    """Search a new retry block while retaining the source spec byte-for-byte.

    ``ordinal`` is the parent's durable attempt number, not an invocation count.
    A resumed attempt therefore cannot restart its search block or spend a new
    budget. The caller retains the existing total-attempt/time/safety limits.
    """
    if not isinstance(ordinal, int) or isinstance(ordinal, bool) or not 1 <= ordinal <= 5:
        raise ValueError('Finite completion attempt ordinal between one and five required')
    if not has_plan(source):
        raise ValueError('Frozen front adapter requires a source assignment')
    plan = verify_preserved(source, source)
    first = source.get('retry', 0) + 1 + (ordinal - 1) * RETRY_BLOCK
    ranked, tried = [], 0
    for retry in range(first, first + MAX_PROPOSALS):
        tried += 1
        # A helper/policy change during the search is a contract failure, not
        # 512 ordinary infeasible layouts. Keep this outside the geometry catch.
        verify_preserved(source, source)
        try:
            scene, meta = build(copy.deepcopy(source['spec']), config, ctx, retry)
        except ValueError:
            continue
        verify_preserved(source, meta)
        encoded = (json.dumps(scene, ensure_ascii=False, indent=2) + '\n').encode()
        if hashlib.sha256(encoded).hexdigest() in known_shas:
            continue
        fresh = novelty(meta, [source])
        if not math.isfinite(fresh) or fresh < .25:
            continue
        proxy = proposal_score(meta, calibration, 'all_actor_visibility')
        if not math.isfinite(proxy['score']):
            continue
        ranked.append((proxy['score'] + min(1., fresh), -retry, scene, meta, proxy, fresh))
        if len(ranked) >= RANKED_CANDIDATES:
            break
    if not ranked:
        verify_preserved(source, source)
        raise FrontGeometryExhausted('No distinct frozen-front candidate within bounded search')
    _, negative_retry, scene, meta, proxy, fresh = max(ranked, key=lambda item: item[:2])
    meta['flexible_completion'] = dict(
        policy=POLICY, source_cell_id=ident, attempt=ordinal,
        generation_retry=-negative_retry, retry_block_start=first,
        proposals_tried=tried, valid_candidates=len(ranked),
        maximum_proposals=MAX_PROPOSALS, ranked_candidate_cap=RANKED_CANDIDATES,
        donor_selection='disabled; frozen source actor identities and seed retained',
        source_seed=source['spec']['seed'], frozen_front_plan=copy.deepcopy(plan),
        source_actor_ids=list(plan['frozen_population_data_ids']),
        relative_view_novelty=fresh, proxy=proxy,
        qa_policy='Unchanged native annotation, visibility, motion, ground, Ego, collision and Park gates')
    return scene, meta
