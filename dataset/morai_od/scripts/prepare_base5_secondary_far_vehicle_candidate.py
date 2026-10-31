#!/usr/bin/env python3
"""Prepare a new 5K candidate by moving 160 existing highway secondary NPCs.

No simulator access, bulk authorization or old source edits. Every unchanged
entry retains its original scenario/metadata path and hash. Feasibility misses
advance to another deterministically ranked source, never relax a target band.
"""
import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path
import time

import audit_base5_scenarios
import base5_secondary_far_vehicle as far
import generate_base5_independent as gen
import run_base5_adaptive_completion as adaptive
from base5_capture_safety import atomic_json
from run_base5_failure_aware_replenishment_fast import prewarm


DEFAULT_QUOTAS = (((60, 80), 'front', 30), ((60, 80), 'rear', 30),
    ((80, 90), 'front', 25), ((80, 90), 'rear', 25),
    ((90, 95), 'front', 12), ((90, 95), 'rear', 12),
    ((95, 99), 'front', 13), ((95, 99), 'rear', 13))


def read(path):
    return json.loads(Path(path).read_text())


def sampling_bounds(bounds, allocation_index, quota_count):
    """Keep the outer quota; explicitly freeze the near/far halves of its tail."""
    if tuple(bounds) != (95, 99):
        return list(bounds)
    if quota_count != 13 or not 0 <= allocation_index < quota_count:
        raise far.FarContractError('95--99m allocation requires six lower and seven upper slots per role')
    return [95, 97] if allocation_index < 6 else [97, 99]


def candidates(deck, originals):
    result = []
    for entry in deck['scenes']:
        meta = originals[entry['id']]
        if (meta['spec']['family'] != 'highway' or meta['spec']['mission']
                or meta['spec'].get('shadow_phase') or far.has_plan(meta)):
            continue
        for ordinal, actor in enumerate(meta['runtime_actors'][1:], 1):
            if (actor['source_list'] == 'vehicleList' and actor['motion'] == 'static'
                    and 30 <= actor['planned_center_distance_m'] < 50
                    and actor['actual_ego_role'] in ('front', 'rear')):
                result.append(dict(id=entry['id'], ordinal=ordinal, data_id=actor['data_id'],
                    role=actor['actual_ego_role'], name=actor['name'], split=meta['split']))
    return result


def order_candidates(values, counts, stratum, used):
    return sorted((c for c in values if c['id'] not in used and c['role'] == stratum[1]),
        key=lambda c: (counts[c['data_id']], far.digest([far.POLICY, stratum, c['id'], c['ordinal']])))


def preservation(before, after, scene, source_scene, spec, config):
    if after['spec'] != spec:
        raise far.FarContractError('Prepared far-secondary spec differs from frozen plan')
    far.validate_plan(after['spec'], config)
    far.verify_preserved(after, after)
    far.verify_payload(scene, source_scene, spec[far.PLAN_KEY], after)
    if after['runtime_actors'][0] != before['runtime_actors'][0] or after['ego'] != before['ego']:
        raise far.FarContractError('Far preparation changed primary or Ego')


def prepare(source, output, maximum_attempts=512, quotas=DEFAULT_QUOTAS):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('New secondary-far candidate output required')
    if not 1 <= maximum_attempts <= far.MAX_PROPOSALS:
        raise ValueError('Finite target search between one and 512 attempts required')
    runtime = gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output == runtime or runtime in output.parents:
        raise ValueError('Candidate must never be written to simulator runtime')
    started = time.monotonic()
    source_hash = far.sha256(source)
    deck = read(source)
    if deck['count'] != 5000 or len(deck['scenes']) != 5000:
        raise far.FarContractError('Far-secondary candidate requires a complete frozen 5K source')
    if far.CONFIG_KEY in deck['config']:
        raise far.FarContractError('Do not reallocate an existing far-secondary plan')
    entries = {e['id']: e for e in deck['scenes']}
    if len(entries) != 5000:
        raise far.FarContractError('Duplicate source scene ID')
    originals, source_pins = {}, {}
    for entry in deck['scenes']:
        if far.sha256(entry['scenario']) != entry['scenario_sha256']:
            raise far.FarContractError('Frozen source scenario changed')
        originals[entry['id']] = read(entry['metadata'])
        source_pins[entry['id']] = dict(scenario_sha256=entry['scenario_sha256'],
                                      metadata_sha256=far.sha256(entry['metadata']))
    config = copy.deepcopy(deck['config'])
    config[far.CONFIG_KEY] = far.configuration(config['environment_tag'], source)
    config[far.CONFIG_KEY]['allocation'] = dict(total=sum(n for _, _, n in quotas),
        strata=[dict(distance_bounds_m=list(b), role=r, count=n,
            sampling_windows=[dict(distance_bounds_m=list(window), count=count)
                for window, count in Counter(tuple(sampling_bounds(b, i, n)) for i in range(n)).items()])
            for b, r, n in quotas],
        policy='one-secondary-per-scene; least-selected-asset then source hash order; feasible source fallback',
        interpretation='Bounded distribution pilot, not an AP-optimal allocation claim')
    watched = {str(Path(__file__).resolve()): far.sha256(__file__),
               str(Path(far.__file__).resolve()): far.sha256(far.__file__)}
    visibility_policy = config[far.CONFIG_KEY]['proposal_visibility']
    for key in ('implementation', 'ray_helper', 'geometry_reference', 'sensor', 'normalizer'):
        watched[visibility_policy[key]['path']] = visibility_policy[key]['sha256']
    pool = candidates(deck, originals)
    ctx = prewarm(gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO), config)
    calibration = far.visibility.validate_policy(visibility_policy)
    asset_counts, used, chosen, rejected, unavailable = Counter(), set(), {}, [], set()
    output.mkdir(parents=True)
    # Interleave target cells so the first stratum does not monopolize rare assets.
    requests = [(bounds, role, i) for bounds, role, n in quotas for i in range(n)]
    requests.sort(key=lambda item: (item[2], far.digest([far.POLICY, item[:2]])))
    for bounds, role, allocation_index in requests:
        quota_count = next(n for b, r, n in quotas if b == bounds and r == role)
        window = sampling_bounds(bounds, allocation_index, quota_count)
        selected = None
        for candidate in order_candidates(pool, asset_counts, (bounds, role), used):
            failure_key = (candidate['id'], candidate['ordinal'], tuple(bounds), tuple(window))
            if failure_key in unavailable:
                continue
            entry = entries[candidate['id']]
            before = originals[candidate['id']]
            source_scene = read(entry['scenario'])
            try:
                spec = far.attach(entry, before, source_scene, candidate['ordinal'], bounds,
                                  config[far.CONFIG_KEY], sampling_bounds_m=window)
            except far.FarGeometryExhausted as error:
                rejected.append(dict(**candidate, bounds=list(bounds), sampling_bounds_m=window,
                                     reason=str(error), proposals=0))
                unavailable.add(failure_key)
                continue
            reason, ranked, known = None, [], set()
            visited, point_count, draw_count, built_count = set(), None, 0, 0
            for attempt in range(maximum_attempts):
                if point_count is not None and len(visited) == point_count:
                    break
                draw_count += 1
                try:
                    point_index, point_count = far.proposal_index(spec, ctx, attempt)
                    if point_index in visited:
                        continue
                    visited.add(point_index)
                    built_count += 1
                    scene, meta = far.build(spec, config, ctx, attempt)
                except far.FarGeometryExhausted as error:
                    reason = str(error)
                    if 'No same-lane far secondary road point' in reason:
                        break
                    continue
                preservation(before, meta, scene, source_scene, spec, config)
                payload = far.digest(scene)
                if payload in known:
                    continue
                known.add(payload)
                view = copy.deepcopy(meta)
                actors, index = view['runtime_actors'], candidate['ordinal']
                view['runtime_actors'] = [actors[index]] + actors[:index] + actors[index+1:]
                proxy = far.score_proposal(view, calibration, 'all_actor_visibility', adaptive.proposal_score)
                if not math.isfinite(proxy['score']):
                    continue
                ranked.append((proxy['score'], -attempt, scene, meta, proxy))
                if len(ranked) >= min(far.RANKED_CANDIDATES, point_count):
                    break
            if ranked:
                _, negative_attempt, scene, meta, proxy = max(ranked, key=lambda value: value[:2])
                meta['secondary_far_vehicle_generation'].update(initial_target_first_proxy=proxy,
                    proposals_tried=draw_count, ranked_candidate_count=len(ranked),
                    distinct_geometry_proposals=built_count, available_legal_road_points=point_count,
                    limitation='Proposal-only conservative single-body 3-D blockage guard; partial/uncertain cases retained; native QA unchanged')
                selected = dict(candidate=candidate, scene=scene, meta=meta, attempt=-negative_attempt,
                                bounds=list(bounds), sampling_bounds_m=window, role=role)
            if selected is not None:
                break
            rejected.append(dict(**candidate, bounds=list(bounds), sampling_bounds_m=window, reason=reason,
                                 proposals=draw_count, distinct_geometry_proposals=built_count,
                                 available_legal_road_points=point_count))
            unavailable.add(failure_key)
        if selected is None:
            atomic_json(output/'allocation_failure.json', dict(policy=far.POLICY,
                requested_bounds=list(bounds), sampling_bounds_m=window, role=role, completed=len(chosen),
                requested_total=len(requests), rejection_count=len(rejected), rejections=rejected,
                bulk_started=False, live_qualified=False, source_preserved=True))
            raise far.FarGeometryExhausted('Not enough feasible frozen-source donors; inspect allocation_failure.json')
        ident = selected['candidate']['id']
        chosen[ident] = selected
        used.add(ident)
        asset_counts[selected['candidate']['data_id']] += 1
        if len(chosen) % 10 == 0:
            print('secondary-far', len(chosen), '/', len(requests), 'elapsed_sec',
                  round(time.monotonic()-started, 1), flush=True)
    changed_entries, deltas, frozen = [], [], []
    for entry in deck['scenes']:
        ident = entry['id']
        if ident not in chosen:
            changed_entries.append(copy.deepcopy(entry))
            frozen.append(copy.deepcopy(originals[ident]['spec']))
            continue
        selected = chosen[ident]
        scene, meta = selected['scene'], selected['meta']
        scenario_path = output/'scenarios'/(ident+'.json')
        metadata_path = output/'scenarios'/(ident+'_metadata.json')
        atomic_json(scenario_path, scene)
        atomic_json(metadata_path, meta)
        changed = copy.deepcopy(entry)
        changed.update(scenario=str(scenario_path), metadata=str(metadata_path),
                       scenario_sha256=far.sha256(scenario_path))
        changed_entries.append(changed)
        frozen.append(copy.deepcopy(meta['spec']))
        index = meta['spec'][far.PLAN_KEY]['target_ordinal']
        old, new = originals[ident]['runtime_actors'][index], meta['runtime_actors'][index]
        deltas.append(dict(id=ident, ordinal=index, target_uid=new['unique_id'], data_id=new['data_id'],
            name=new['name'], split=meta['split'], role=selected['role'],
            distance_bounds_m=selected['bounds'], original_distance_m=old['planned_center_distance_m'],
            sampling_bounds_m=selected['sampling_bounds_m'],
            current_distance_m=new['planned_center_distance_m'],
            lane_relation=meta['spec'][far.PLAN_KEY]['lane_relation'],
            lane_identity=meta['spec'][far.PLAN_KEY]['lane_identity'], attempt=selected['attempt'],
            source_scenario_sha256=entry['scenario_sha256'], scenario_sha256=changed['scenario_sha256']))
    specs = output/'frozen_specs.json'
    atomic_json(specs, frozen)
    config.update(frozen_specs_path=str(specs), frozen_specs_sha256=far.sha256(specs))
    result = copy.deepcopy(deck)
    result.update(config=config, count=len(changed_entries), scenes=changed_entries,
        status='OFFLINE_ONLY_SECONDARY_FAR_V153; native qualification pending; bulk not authorized')
    result.setdefault('provenance', {})['secondary_far_vehicle_source_hashes'] = watched
    manifest = output/'manifest.json'
    atomic_json(manifest, result)
    errors = []
    for entry in changed_entries:
        ident = entry['id']
        before, after = originals[ident], read(entry['metadata'])
        if ident in chosen:
            preservation(before, after, read(entry['scenario']), read(entries[ident]['scenario']),
                         after['spec'], config)
        elif entry != entries[ident]:
            errors.append([ident, 'Unchanged entry path/SHA/header changed'])
        if after['runtime_actors'][:1] != before['runtime_actors'][:1]:
            errors.append([ident, 'Primary changed'])
        if (after['split'] != before['split'] or after['spec']['seed'] != before['spec']['seed']
                or len(after['runtime_actors']) != len(before['runtime_actors'])):
            errors.append([ident, 'Seed/split/actor count changed'])
        if (after['spec'].get('secondary_obstacle_front_support') != before['spec'].get('secondary_obstacle_front_support')
                or after['spec'].get('primary_obstacle_range') != before['spec'].get('primary_obstacle_range')):
            errors.append([ident, 'v151 front or v152 primary-range plan changed'])
        original = entries[ident]
        if (far.sha256(original['scenario']) != source_pins[ident]['scenario_sha256']
                or far.sha256(original['metadata']) != source_pins[ident]['metadata_sha256']):
            errors.append([ident, 'Original source changed'])
    if far.sha256(source) != source_hash:
        errors.append(['manifest', 'Original manifest changed'])
    for path, expected in watched.items():
        if far.sha256(path) != expected:
            errors.append([path, 'Implementation changed during prepare'])
    audit = audit_base5_scenarios.audit(manifest)
    atomic_json(output/'scenario_audit.json', audit)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('errors'):
        errors.append(['full5000', 'Existing scenario audit failed'])
    group_counts = Counter(str((d['distance_bounds_m'], d['role'])) for d in deltas)
    window_counts = Counter(str((d['distance_bounds_m'], d['role'], d['sampling_bounds_m'])) for d in deltas)
    expected_groups = Counter({str((list(b), r)): n for b, r, n in quotas})
    expected_windows = Counter(str((list(b), r, sampling_bounds(b, i, n)))
                              for b, r, n in quotas for i in range(n))
    if group_counts != expected_groups or window_counts != expected_windows:
        errors.append(['allocation', 'Frozen outer or sampling-window quota differs'])
    preservation_audit = dict(schema='base5_secondary_far_vehicle_preservation_v1', policy=far.POLICY,
        passed=not errors, errors=errors, source_manifest=str(source), source_manifest_sha256=source_hash,
        candidate_manifest_sha256=far.sha256(manifest), frames=len(changed_entries),
        declared_plan_count=len(deltas), geometry_changed_scenes=len(deltas),
        unchanged_entry_paths_and_payloads=len(changed_entries)-len(deltas),
        primary_metadata_exact=True, ego_metadata_exact=True, non_target_metadata_and_payload_exact=True,
        actor_count_and_identity_preserved=True, seed_and_split_preserved=True,
        original_front_and_primary_range_plans_preserved=True,
        target_asset_counts=dict(asset_counts), target_distinct_assets=len(asset_counts),
        stratum_counts=dict(group_counts), split_counts=dict(Counter(d['split'] for d in deltas)),
        sampling_window_counts=dict(window_counts),
        lane_relation_counts=dict(Counter(d['lane_relation'] for d in deltas)),
        source_files=source_pins, source_hashes=watched, source_preserved=True,
        rejections=rejected, elapsed_sec=time.monotonic()-started,
        live_qualified=False, bulk_ready=False, bulk_started=False,
        limitation='CPU legal geometry and preservation only; unchanged native visibility/ground/collision/Park gates still required')
    atomic_json(output/'preservation_audit.json', preservation_audit)
    atomic_json(output/'secondary_far_vehicle_delta.json', dict(policy=far.POLICY, changes=deltas,
        requested_quotas=config[far.CONFIG_KEY]['allocation'], live_qualified=False, bulk_started=False))
    atomic_json(output/'registration.json', dict(schema='base5_secondary_far_vehicle_registration_v1',
        policy=far.POLICY, source_manifest=str(source), source_manifest_sha256=source_hash,
        candidate_manifest=str(manifest), candidate_manifest_sha256=far.sha256(manifest),
        helper_sha256=far.sha256(far.__file__), script_sha256=far.sha256(__file__),
        planned_count=len(deltas), unchanged_count=5000-len(deltas),
        actual_asset_counts=dict(asset_counts), actual_stratum_counts=dict(group_counts),
        actual_sampling_window_counts=dict(window_counts),
        bulk_ready=False, live_qualified=False, bulk_started=False))
    if errors:
        raise far.FarContractError('Full far-secondary candidate audit failed; preserve diagnostic output')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maximum-attempts', type=int, default=512)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output, args.maximum_attempts))
