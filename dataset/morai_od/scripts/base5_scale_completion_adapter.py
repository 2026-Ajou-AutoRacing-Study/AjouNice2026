"""Finite new geometry for a frozen scaled population, never donor/seed changes."""
import copy
import hashlib
import json
import math

import base5_asset_scale as scale
import base5_front_completion_adapter as front
import base5_primary_obstacle_range as primary_range
import base5_secondary_far_vehicle as secondary_far


POLICY = 'frozen-scale-population-completion-v155'
MAX_PROPOSALS, RANKED_CANDIDATES, RETRY_BLOCK = 512, 8, 4096


class ScaleGeometryExhausted(ValueError):
    """Only the bounded geometry search failed; frozen scale remains unchanged."""


def has_plan(metadata):
    return scale.PLAN_KEY in metadata.get('spec', {})


def verify_preserved(source, actual, config):
    """Contract failures are RuntimeError, never ordinary placement deficits."""
    if not has_plan(source):
        if has_plan(actual) or any(a.get('asset_scale') is not None
                                   for a in actual.get('runtime_actors', [])):
            raise scale.ScaleContractError('Unplanned completion gained a foreign scale assignment')
        return None
    try:
        plan = scale.validate_plan(source['spec'], config)
        scale.validate_plan(actual['spec'], config)
        if actual['spec'] != source['spec'] or actual['split'] != source['split']:
            raise scale.ScaleContractError('Completion changed frozen scale spec/seed/split')
        before, after = source['runtime_actors'], actual['runtime_actors']
        fields = ('data_id', 'unique_id', 'name', 'source_list', 'population_pool', 'motion')
        identity = lambda actors: [tuple(a.get(k) for k in fields) for a in actors]
        if len(after) != len(plan['actor_identity']) or identity(before) != identity(after):
            raise scale.ScaleContractError('Completion changed frozen scaled actor population')
        for ordinal, (old, new) in enumerate(zip(before, after)):
            if any(old.get(key)!=value for key,value in plan['actor_identity'][ordinal].items()):
                raise scale.ScaleContractError('Source scaled actor differs from frozen identity')
            asset = scale._identity_asset(old)
            expected = scale.select(asset, source['spec'], ordinal, old['motion'],
                                    config[scale.CONFIG_KEY], profile_id=plan['profile_ids'][ordinal])
            if old.get('asset_scale') != expected or new.get('asset_scale') != expected:
                raise scale.ScaleContractError('Completion changed frozen scale profile/assignment')
            if expected and expected['profile_id'] != scale.BASE:
                profile = config[scale.CONFIG_KEY]['assets'][old['name']]['profiles'][expected['profile_id']]
                if new['estimated_size'] != profile['native_geometry']['bbox_size_m']:
                    raise scale.ScaleContractError('Completion changed qualified scaled placement geometry')
                initializer=scale.height_initializer(expected,config[scale.CONFIG_KEY],
                    profile['native_geometry']['bbox_size_m'],profile['native_geometry']['pivot_to_bbox_center_local_m'])
                if old.get('static_height_initializer')!=initializer or new.get('static_height_initializer')!=initializer:
                    raise scale.ScaleContractError('Completion changed qualified scale height initializer')
        # Existing frozen helper contracts remain mandatory when overlays coexist.
        front.verify_preserved(source, actual)
        primary_range.verify_preserved(source, actual)
        secondary_far.verify_preserved(source, actual)
        return plan
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        raise scale.ScaleContractError('Frozen scale completion contract mismatch: ' + str(exc)) from exc


def choose(source, config, ctx, calibration, ordinal, ident, known_shas,
           build, proposal_score, novelty):
    if type(ordinal) is not int or not 1 <= ordinal <= 5:
        raise scale.ScaleContractError('Durable finite scale completion ordinal 1 to 5 required')
    if not has_plan(source):
        raise scale.ScaleContractError('Frozen scale adapter requires a scale plan')
    config=scale.bind_config(config)
    plan = verify_preserved(source, source, config)
    first = source.get('retry', 0) + 1 + (ordinal - 1) * RETRY_BLOCK
    ranked, tried = [], 0
    for retry in range(first, first + MAX_PROPOSALS):
        tried += 1
        # Helper/profile/source errors occur outside the ordinary geometry catch.
        verify_preserved(source, source, config)
        try:
            scene, actual = build(copy.deepcopy(source['spec']), config, ctx, retry)
        except ValueError:
            continue
        verify_preserved(source, actual, config)
        catalog = {a['data_id']: scale._identity_asset(a) for a in actual['runtime_actors']}
        scale.verify_metadata(scene, actual, config, catalog)
        payload = hashlib.sha256((json.dumps(scene, ensure_ascii=False, indent=2) + '\n').encode()).hexdigest()
        if payload in known_shas:
            continue
        fresh = novelty(actual, [source])
        if not math.isfinite(fresh) or fresh < .25:
            continue
        proxy = proposal_score(actual, calibration, 'all_actor_visibility')
        if not math.isfinite(proxy['score']):
            continue
        ranked.append((proxy['score'] + min(1., fresh), -retry, scene, actual, proxy, fresh))
        if len(ranked) >= RANKED_CANDIDATES:
            break
    if not ranked:
        verify_preserved(source, source, config)
        scale.verify_frozen_policy(config[scale.CONFIG_KEY])
        raise ScaleGeometryExhausted('No distinct frozen-scale layout within bounded search')
    _, negative_retry, scene, actual, proxy, fresh = max(ranked, key=lambda item: item[:2])
    actual['flexible_completion'] = dict(policy=POLICY, source_cell_id=ident, attempt=ordinal,
        generation_retry=-negative_retry, retry_block_start=first, proposals_tried=tried,
        valid_candidates=len(ranked), maximum_proposals=MAX_PROPOSALS,
        ranked_candidate_cap=RANKED_CANDIDATES, source_seed=source['spec']['seed'],
        source_profile_ids=list(plan['profile_ids']), donor_selection='disabled',
        scale_profile_fallback=False, actor_population_changed=False,
        relative_view_novelty=fresh, proxy=proxy,
        qa_policy='Unchanged native eight-corner GT, visibility, motion, ground, Ego, collision and Park gates')
    scale.verify_frozen_policy(config[scale.CONFIG_KEY])
    return scene, actual
