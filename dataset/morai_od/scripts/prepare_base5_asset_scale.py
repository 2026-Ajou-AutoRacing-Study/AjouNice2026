#!/usr/bin/env python3
"""Freeze native-qualified discrete scales into a new 5K candidate, never capture.

This is an offline generator layer, not an extra family or a GT transform.
Unapproved profiles cannot be adopted. Actors/assets, scene count, distance
contracts, splits and previous source files remain preserved. A placement or
budget failure is explicit; there is no silent variant-to-original fallback.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import time

import audit_base5_scenarios
import base5_asset_scale as scale
import base5_obstacle_front_support as front
import base5_population_budget as population
import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm


def read(path):
    return json.loads(Path(path).read_text())


def inventory(source):
    source = Path(source).resolve()
    deck = read(source)
    if deck.get('count') != 5000 or len(deck.get('scenes', [])) != 5000:
        raise scale.ScaleContractError('Complete 5K source required')
    frozen = Path(deck['config']['frozen_specs_path'])
    if not frozen.is_absolute():
        frozen = source.parent / frozen
    if sha256(frozen) != deck['config']['frozen_specs_sha256']:
        raise scale.ScaleContractError('Source frozen spec checksum changed')
    specs = read(frozen)
    if len(specs) != 5000:
        raise scale.ScaleContractError('Source frozen spec count changed')
    pinned = {str(source): sha256(source), str(frozen): sha256(frozen)}
    metadata = {}
    for entry, spec in zip(deck['scenes'], specs):
        if entry['id'] in metadata or sha256(entry['scenario']) != entry['scenario_sha256']:
            raise scale.ScaleContractError('Duplicate or changed source scenario')
        meta = read(entry['metadata'])
        if meta['spec'] != spec or meta['split'] != entry['split'] or scale.PLAN_KEY in spec:
            raise scale.ScaleContractError('Source spec/split differs or scale plan already frozen')
        metadata[entry['id']] = meta
        for key in ('scenario', 'metadata'):
            pinned[str(Path(entry[key]).resolve())] = sha256(entry[key])
    return deck, metadata, pinned


def _items(deck, metadata, policy, catalog):
    result = []
    for entry in deck['scenes']:
        meta = metadata[entry['id']]
        for ordinal, actor in enumerate(meta['runtime_actors']):
            asset = catalog[actor['data_id']]
            if scale._record(asset, policy, actor['motion']) is None:
                continue
            row = dict(meta['spec'], split=meta['split'])
            result.append(dict(asset=asset, row=row, ordinal=ordinal, motion=actor['motion'],
                               scene_id=entry['id']))
    return result


def _profile_geometry(actor, profile_id, policy):
    if profile_id == scale.BASE:
        return actor['estimated_size']
    return policy['assets'][actor['name']]['profiles'][profile_id]['native_geometry']['bbox_size_m']


def feasible_allocation(deck, metadata, policy, desired, catalog):
    """Resolve budget/tier feasibility *before* committing profiles into specs.

    Native-qualified alternatives are considered only for the same selected
    actor/asset. Initial original slots stay protected. This is declared plan
    design, not a retry fallback, and it never reads capture QA outcomes.
    """
    scale.validate_policy(policy)
    actual = copy.deepcopy(desired)
    decisions, desired_counts, actual_counts = [], Counter(), Counter()
    grouped = {}
    for entry in deck['scenes']:
        before, ident = metadata[entry['id']], entry['id']
        row, actors = before['spec'], before['runtime_actors']
        baseline_costs = [population.cost(actor['estimated_size']) for actor in actors]
        budget = row.get('population_budget_bounds') if row.get('population_mode') in ('generic_regular', 'generic_dense') else None
        if budget and not budget[0] <= sum(baseline_costs) <= budget[1]:
            raise scale.ScaleContractError('Source population already violates size budget: ' + ident)
        front_plan = front.validate_plan(row)
        profiles, keys = [], []
        for ordinal, actor in enumerate(actors):
            key = scale.actor_key(catalog[actor['data_id']], row, ordinal)
            keys.append(key)
            profile = desired.get(key, scale.BASE)
            profiles.append(profile)
            if profile != scale.BASE:
                scale.select(catalog[actor['data_id']], row, ordinal, actor['motion'], policy,
                             profile_id=profile)
                desired_counts[(actor['name'], profile)] += 1
        current_costs = list(baseline_costs)
        selected = [i for i, profile in enumerate(profiles) if profile != scale.BASE]
        # Charge-increasing proposals precede reductions from old conservative
        # unknown envelopes. Ties are reproducible, not manifest iteration based.
        selected.sort(key=lambda ordinal: (
            -(population.cost(_profile_geometry(actors[ordinal], profiles[ordinal], policy)) - baseline_costs[ordinal]),
            scale.digest([keys[ordinal], 'feasibility'])))
        scene_decisions = []
        for ordinal in selected:
            actor, requested, key = actors[ordinal], profiles[ordinal], keys[ordinal]
            record = policy['assets'][actor['name']]
            approved = scale._available(record, False)
            alternatives = sorted((name for name in approved if name != requested),
                key=lambda name: (max(approved[name]['scale_xyz']),
                    np_product(approved[name]['scale_xyz']), name))
            attempts, chosen = [], scale.BASE
            for profile in [requested] + alternatives:
                size = _profile_geometry(actor, profile, policy)
                cost = population.cost(size)
                total = sum(current_costs) - current_costs[ordinal] + cost
                reasons = []
                if budget and not budget[0] <= total <= budget[1]:
                    reasons.append('scaled_population_budget_outside_declared_bounds')
                if (front_plan and ordinal == front_plan['ordinal']
                        and front.distance_bounds(size) != front_plan['distance_bounds_m']):
                    reasons.append('scaled_actor_changes_frozen_front_size_tier')
                attempts.append(dict(profile_id=profile, placement_size_m=list(size),
                    actor_cost=cost, scene_cost_if_selected=total, reasons=reasons))
                if not reasons:
                    chosen = profile
                    current_costs[ordinal] = cost
                    break
            actual[key] = chosen
            if chosen != scale.BASE:
                actual_counts[(actor['name'], chosen)] += 1
            decision = dict(id=ident, ordinal=ordinal, name=actor['name'], data_id=actor['data_id'],
                split=before['split'], family=row.get('family'), mission=bool(row.get('mission')),
                role=row.get('role'), desired_profile_id=requested, actual_profile_id=chosen,
                decision=('requested_profile_feasible' if chosen == requested else
                          'same_asset_qualified_alternative' if chosen != scale.BASE else
                          'original_preserved_before_freeze'), attempts=attempts,
                source_scene_cost=sum(baseline_costs), final_scene_cost=None,
                actor_population_changed=False, native_qa_outcomes_used=False,
                frozen_retry_profile_fallback=False)
            decisions.append(decision)
            scene_decisions.append(decision)
        for decision in scene_decisions:
            decision['final_scene_cost'] = sum(current_costs)
        if budget and not budget[0] <= sum(current_costs) <= budget[1]:
            raise scale.ScaleContractError('Feasible scale allocation escaped declared budget')
        for ordinal, actor in enumerate(actors):
            if keys[ordinal] not in desired:
                continue
            group = grouped.setdefault((actor['name'], before['split']), dict(
                name=actor['name'], split=before['split'], total=0, desired_variants=0, actual_variants=0))
            group['total'] += 1
            group['desired_variants'] += profiles[ordinal] != scale.BASE
            group['actual_variants'] += actual[keys[ordinal]] != scale.BASE
    groups = []
    for _, group in sorted(grouped.items()):
        group['desired_fraction'] = group['desired_variants'] / group['total']
        group['actual_fraction'] = group['actual_variants'] / group['total']
        groups.append(group)
    return actual, dict(schema='base5_asset_scale_feasibility_v1', policy_sha256=scale.digest(policy),
        desired_variants=sum(desired_counts.values()), actual_variants=sum(actual_counts.values()),
        original_retained_before_freeze=sum(d['actual_profile_id'] == scale.BASE for d in decisions),
        qualified_alternative_before_freeze=sum(d['decision'] == 'same_asset_qualified_alternative' for d in decisions),
        by_asset_profile_desired={str(key): value for key, value in sorted(desired_counts.items())},
        by_asset_profile_actual={str(key): value for key, value in sorted(actual_counts.items())},
        groups=groups, decisions=decisions,
        scene_actor_lists_preserved=True, native_qa_outcomes_used=False,
        minimum_original_protection_preserved=True, frozen_retry_profile_fallback=False)


def np_product(values):
    result = 1.
    for value in values:
        result *= value
    return result


def validate_allocation(deck, metadata, policy, mapping, catalog):
    """Preflight the frozen choices before any output or expensive rebuilding."""
    plans, counts = {}, Counter()
    for entry in deck['scenes']:
        before = metadata[entry['id']]
        actors = before['runtime_actors']
        profiles = []
        for ordinal, actor in enumerate(actors):
            key = scale.actor_key(catalog[actor['data_id']], before['spec'], ordinal)
            profile = mapping.get(key, scale.BASE)
            profiles.append(profile)
            if profile != scale.BASE:
                counts[actor['name']] += 1
        if all(ident == scale.BASE for ident in profiles):
            continue
        row = before['spec']
        sizes = [_profile_geometry(actor, ident, policy) for actor, ident in zip(actors, profiles)]
        if row.get('population_mode') in ('generic_regular', 'generic_dense'):
            total = sum(population.cost(size) for size in sizes)
            lo, hi = row['population_budget_bounds']
            if not lo <= total <= hi:
                raise scale.ScaleContractError(
                    'Frozen scale allocation exceeds existing size budget: ' + entry['id'])
        plan = front.validate_plan(row)
        if plan and front.distance_bounds(sizes[plan['ordinal']]) != plan['distance_bounds_m']:
            raise scale.ScaleContractError(
                'Frozen scale allocation changes existing front-support size tier: ' + entry['id'])
        enriched = [dict(catalog[actor['data_id']], **actor) for actor in actors]
        plans[entry['id']] = scale.attach_plan(row, enriched, policy, profiles)
    return plans, dict(counts)


def verify_transition(before, after, scene, spec, config, catalog):
    if after['spec'] != spec or after['split'] != before['split']:
        raise scale.ScaleContractError('Scale rebuild changed frozen spec/split')
    fields = ('data_id', 'name', 'source_list', 'motion', 'unique_id', 'population_pool')
    identities = lambda actors: [tuple(actor.get(k) for k in fields) for actor in actors]
    if identities(before['runtime_actors']) != identities(after['runtime_actors']):
        raise scale.ScaleContractError('Scale rebuild changed actors/assets/motion/order')
    scale.verify_metadata(scene, after, config, catalog)
    projected = copy.deepcopy(before)
    projected['spec'] = spec
    from base5_primary_obstacle_range import verify_preserved as verify_range
    verify_range(projected, after)
    from base5_front_completion_adapter import verify_preserved as verify_front
    verify_front(projected, after)


def prepare(source, native_policy, output, maximum_attempts=512):
    source, native_policy, output = map(lambda p: Path(p).resolve(), (source, native_policy, output))
    if output.exists():
        raise FileExistsError('New scale candidate output required')
    runtime = spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output == runtime or runtime in output.parents:
        raise scale.ScaleContractError('Offline preparation cannot write MORAI runtime')
    if not 1 <= maximum_attempts <= 4096:
        raise scale.ScaleContractError('Finite scale preparation proposal budget required')
    policy = scale.freeze_policy(read(native_policy))
    qualified = [(name, ident) for name, record in policy['assets'].items()
                 for ident, profile in record['profiles'].items() if scale._qualified(profile)]
    if not qualified:
        raise scale.ScaleContractError('No native-qualified scale profiles; full 5K adoption is blocked')
    deck, metadata, pins = inventory(source)
    if scale.CONFIG_KEY in deck['config']:
        raise scale.ScaleContractError('Source scale policy already frozen; do not reassign')
    before_audit = audit_base5_scenarios.audit(source)
    if before_audit.get('status') != 'OFFLINE_PASS' or before_audit.get('errors'):
        raise scale.ScaleContractError('Source full-deck audit failed')
    catalog_path = spatial.gen.catalog.DEFAULT_CATALOG
    catalog = {a['data_id']: a for a in read(catalog_path)['assets']}
    pins.update({str(native_policy): sha256(native_policy), str(catalog_path.resolve()): sha256(catalog_path)})
    scripts = Path(__file__).resolve().parent
    watched = {str(path): sha256(path) for path in (Path(__file__).resolve(),
        scripts / 'base5_asset_scale.py', scripts / 'generate_base5_independent.py',
        scripts / 'base5_map_surface.py', scripts / 'audit_base5_scenarios.py')}
    desired_mapping, allocation = scale.stratified_profiles(_items(deck, metadata, policy, catalog), policy)
    mapping, feasibility = feasible_allocation(deck, metadata, policy, desired_mapping, catalog)
    plans, asset_counts = validate_allocation(deck, metadata, policy, mapping, catalog)
    if not plans:
        raise scale.ScaleContractError('No scale variants assigned; an original-only deck is not scale qualification')
    config = copy.deepcopy(deck['config'])
    config[scale.CONFIG_KEY] = policy
    output.mkdir(parents=True)
    atomic_json(output / 'allocation.json', dict(allocation, desired_mapping=desired_mapping,
        mapping=mapping, feasibility=feasibility,
        assigned_scenes=len(plans), assigned_actors=sum(asset_counts.values()), by_asset=asset_counts))
    ctx = None
    entries, frozen, changes, rejected = [], [], [], Counter()
    start = time.monotonic()
    for index, entry in enumerate(deck['scenes']):
        ident, before = entry['id'], metadata[entry['id']]
        if ident not in plans:
            entries.append(copy.deepcopy(entry))
            frozen.append(copy.deepcopy(before['spec']))
            continue
        if ctx is None:
            ctx = prewarm(spatial.gen.context(runtime, spatial.gen.geo.DEFAULT_MGEO), config)
        spec = plans[ident]
        for attempt in range(before['retry'], before['retry'] + maximum_attempts):
            try:
                scene, meta = spatial.build(spec, config, ctx, attempt)
            except ValueError as error:
                rejected[str(error)] += 1
                continue
            verify_transition(before, meta, scene, spec, config, catalog)
            break
        else:
            atomic_json(output / 'failure.json', dict(id=ident, completed=index,
                maximum_offline_proposals=maximum_attempts, reasons=dict(rejected),
                scale_profile_fallback=False, actor_population_changed=False, bulk_started=False))
            raise scale.ScaleContractError('Finite scale geometry preparation exhausted: ' + ident)
        for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                    'fresh_environment_provenance'):
            if key in before:
                meta[key] = copy.deepcopy(before[key])
        meta['asset_scale_preparation'] = dict(source_id=ident,
            source_scenario_sha256=entry['scenario_sha256'],
            source_metadata_sha256=pins[str(Path(entry['metadata']).resolve())],
            source_spec_sha256=scale.digest(before['spec']), original_retry=before['retry'],
            actual_retry=meta['retry'], primary_range_rebound=spec[scale.PLAN_KEY]['range_plan_rebind'],
            native_qa_unchanged=True, actor_population_preserved=True, scale_profile_fallback=False)
        scenario, mpath = output / 'scenarios' / (ident + '.json'), output / 'scenarios' / (ident + '_metadata.json')
        atomic_json(scenario, scene)
        atomic_json(mpath, meta)
        changed = copy.deepcopy(entry)
        changed.update(scenario=str(scenario), metadata=str(mpath), scenario_sha256=sha256(scenario))
        entries.append(changed)
        frozen.append(copy.deepcopy(spec))
        changes.append(dict(id=ident, source_scenario_sha256=entry['scenario_sha256'],
            scenario_sha256=changed['scenario_sha256'], profile_ids=spec[scale.PLAN_KEY]['profile_ids'],
            primary_range_rebound=spec[scale.PLAN_KEY]['range_plan_rebind'],
            original_primary_range_plan_sha256=scale.digest(before['spec'].get('primary_obstacle_range')),
            rebound_primary_range_plan_sha256=scale.digest(spec.get('primary_obstacle_range'))))
        if len(changes) % 50 == 0:
            print('asset-scale', len(changes), '/', len(plans), round(time.monotonic() - start, 1), flush=True)
    frozen_path = output / 'frozen_specs.json'
    atomic_json(frozen_path, frozen)
    config.update(frozen_specs_path=str(frozen_path), frozen_specs_sha256=sha256(frozen_path))
    result = copy.deepcopy(deck)
    result.update(config=scale.plain_config(config), scenes=entries,
                  status='NATIVE_QUALIFIED_SCALE_CPU_ONLY; new live regression pending')
    result.setdefault('provenance', {})['asset_scale'] = dict(policy=scale.POLICY,
        source_manifest=str(source), source_manifest_sha256=pins[str(source)],
        policy_path=str(native_policy), policy_sha256=pins[str(native_policy)], implementation_sha256=watched[str(Path(__file__).resolve())])
    manifest = output / 'manifest.json'
    atomic_json(manifest, result)
    errors = []
    for path, expected in dict(pins, **watched).items():
        if sha256(path) != expected:
            errors.append([path, 'source/runtime changed during preparation'])
    for old, entry in zip(deck['scenes'], entries):
        if entry['id'] != old['id'] or sha256(entry['scenario']) != entry['scenario_sha256']:
            errors.append([entry['id'], 'scene identity/checksum changed'])
        if entry['id'] in plans:
            verify_transition(metadata[entry['id']], read(entry['metadata']), read(entry['scenario']),
                              plans[entry['id']], config, catalog)
        elif entry != old:
            errors.append([entry['id'], 'untargeted source entry changed'])
    audit = audit_base5_scenarios.audit(manifest)
    atomic_json(output / 'scenario_audit.json', audit)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('errors'):
        errors.append(['full5000', 'offline scenario audit failed'])
    scale.verify_frozen_policy(policy)
    report = dict(schema='base5_asset_scale_preparation_v1', passed=not errors, errors=errors,
        policy=scale.POLICY, source_manifest=str(source), source_manifest_sha256=pins[str(source)],
        manifest_sha256=sha256(manifest), source_files=pins, runtime_files=watched,
        frames=5000, assigned_scenes=len(plans), assigned_actors=sum(asset_counts.values()),
        by_asset=asset_counts, unchanged_entries=5000 - len(plans), changes=changes,
        desired_variant_actors=feasibility['desired_variants'],
        actual_variant_actors=feasibility['actual_variants'],
        scale_feasibility=feasibility,
        rejected_geometry=dict(rejected), qualified_profiles=[list(pair) for pair in qualified],
        primary_assets_preserved=True, actor_population_preserved=True, split_preserved=True,
        training_gt_modified=False, scale_profile_fallback=False,
        bulk_ready=False, bulk_started=False, new_native_regression_required=True)
    atomic_json(output / 'preservation_audit.json', report)
    if errors:
        raise scale.ScaleContractError('Scale candidate preservation/audit failed')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--native-policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maximum-attempts', type=int, default=512)
    args = parser.parse_args()
    print(prepare(args.manifest, args.native_policy, args.output, args.maximum_attempts))
