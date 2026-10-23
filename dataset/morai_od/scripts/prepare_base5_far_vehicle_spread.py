#!/usr/bin/env python3
"""Spread an existing far allocation inside its unchanged frozen windows.

Visibility is a feasibility guard, not an objective that silently minimizes
distance. This CPU-only preparation never edits the original candidate or its
plans, Ego, primary, actor identities, split, native QA, or recovery windows.
"""
import argparse
from collections import Counter, defaultdict
import copy
import json
from pathlib import Path
import time

import audit_base5_scenarios
import base5_secondary_far_vehicle as far
import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm


POLICY = 'frozen-far-window-stratified-initial-distance-v154'


def read(path):
    return json.loads(Path(path).read_text())


def assignments(metadata):
    """Preregister equal quantiles per role/window before inspecting outcomes."""
    groups = defaultdict(list)
    for ident, meta in metadata.items():
        plan = meta.get('spec', {}).get(far.PLAN_KEY)
        if plan:
            key = (plan['role'], tuple(plan['sampling_bounds_m']))
            groups[key].append(ident)
    result = {}
    for (role, window), ids in groups.items():
        ordered = sorted(ids, key=lambda ident: (far.digest([POLICY, ident]), ident))
        lo, hi = window
        for ordinal, ident in enumerate(ordered):
            fraction = (ordinal + .5) / len(ordered)
            result[ident] = dict(role=role, sampling_bounds_m=list(window),
                rank=ordinal, group_count=len(ordered), quantile=fraction,
                target_distance_ego_xy_m=lo + (hi-lo)*fraction)
    return result


def proposal_order(plan, candidates, target, maximum=512):
    """Map each distinct road point to its first original deterministic draw."""
    if not candidates:
        raise far.FarGeometryExhausted('Empty legal road-point set')
    retries = {}
    for attempt in range(maximum):
        index = far._draw_index(plan, attempt, len(candidates))
        retries.setdefault(index, attempt)
        if len(retries) == len(candidates):
            break
    return sorted(retries.items(), key=lambda row:
        (abs(candidates[row[0]][4]-target), row[1], row[0]))


def verify_transition(before, after, scene, assignment):
    """Recheck the serialized output, not just the objects used by the builder."""
    plan = far.verify_preserved(before, after)
    if (plan is None or after['spec'] != before['spec'] or after['ego'] != before['ego']
            or after['runtime_actors'][0] != before['runtime_actors'][0]
            or after['split'] != before['split']):
        raise far.FarContractError('Spread changed frozen spec/Ego/primary/split')
    frozen_scene, _ = far._read_source(plan)
    far.verify_payload(scene, frozen_scene, plan, after)
    proof = after['secondary_far_vehicle_generation']['initial_distance_spread']
    if any(proof.get(key) != value for key, value in assignment.items()):
        raise far.FarContractError('Spread assignment differs from its preregistered quantile')
    actual = after['runtime_actors'][plan['target_ordinal']]['planned_center_distance_m']
    if (proof.get('policy') != POLICY or proof.get('actual_distance_ego_xy_m') != actual
            or proof.get('target_error_m') != abs(actual-assignment['target_distance_ego_xy_m'])):
        raise far.FarContractError('Spread output distance proof differs from target metadata')


def prepare(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('New spread-candidate output required')
    runtime = gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    if output == runtime or runtime in output.parents:
        raise ValueError('Preparation cannot write to simulator runtime')
    deck = read(source)
    if deck.get('count') != 5000 or len(deck.get('scenes', [])) != 5000:
        raise ValueError('A complete frozen 5K candidate is required')
    from prepare_base5_far_vehicle_qualification import source_audit_pins
    audit_pins = source_audit_pins(source)
    frozen_path = Path(deck['config']['frozen_specs_path'])
    if not frozen_path.is_absolute():
        frozen_path = source.parent/frozen_path
    if sha256(frozen_path) != deck['config']['frozen_specs_sha256']:
        raise far.FarContractError('Source frozen specs checksum changed')
    frozen_specs = read(frozen_path)
    if len(frozen_specs) != 5000:
        raise far.FarContractError('Source frozen spec count differs')
    original, pins = {}, {str(source): sha256(source), str(Path(__file__).resolve()): sha256(__file__)}
    pins.update(audit_pins)
    pins[str(frozen_path)] = sha256(frozen_path)
    for entry, spec in zip(deck['scenes'], frozen_specs):
        if entry['id'] in original or sha256(entry['scenario']) != entry['scenario_sha256']:
            raise far.FarContractError('Duplicate or changed source scenario')
        meta = read(entry['metadata'])
        if meta['spec'] != spec:
            raise far.FarContractError('Source metadata differs from its frozen spec')
        if far.has_plan(meta):
            plan = far.validate_plan(meta['spec'], deck['config'])
            far.verify_preserved(meta, meta)
            frozen_scene, _ = far._read_source(plan)
            far.verify_payload(read(entry['scenario']), frozen_scene, plan, meta)
        original[entry['id']] = meta
        for key in ('scenario', 'metadata'):
            pins[str(Path(entry[key]).resolve())] = sha256(entry[key])
    assigned = assignments(original)
    if len(assigned) != 160:
        raise ValueError('Preserve the existing 160-slot allocation')
    ctx = prewarm(gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO), deck['config'])
    entries, changes, started = [], [], time.monotonic()
    for entry in deck['scenes']:
        ident, before = entry['id'], original[entry['id']]
        if ident not in assigned:
            entries.append(copy.deepcopy(entry))
            continue
        plan = far.validate_plan(before['spec'], deck['config'])
        frozen_scene, canonical = far._read_source(plan)
        candidates, _ = far._candidates(canonical, plan, ctx)
        assignment = assigned[ident]
        target = assignment['target_distance_ego_xy_m']
        ranked = proposal_order(plan, candidates, target)
        rejected, selected = [], None
        for index, attempt in ranked[:far.RANKED_CANDIDATES]:
            try:
                scene, meta = far.build(copy.deepcopy(before['spec']), deck['config'], ctx, attempt)
            except far.FarGeometryExhausted as error:
                rejected.append(dict(index=index, retry=attempt, reason=str(error)))
                continue
            selected = (scene, meta, index, attempt)
            break
        if selected is None:
            raise far.FarGeometryExhausted('No feasible stratified proposal: ' + ident)
        scene, meta, index, attempt = selected
        far.verify_preserved(before, meta)
        far.verify_payload(scene, frozen_scene, plan, meta)
        if (meta['spec'] != before['spec'] or meta['ego'] != before['ego']
                or meta['runtime_actors'][0] != before['runtime_actors'][0]
                or meta['split'] != before['split']):
            raise far.FarContractError('Spread changed frozen spec/Ego/primary/split')
        ordinal = plan['target_ordinal']
        actual = meta['runtime_actors'][ordinal]['planned_center_distance_m']
        proof = dict(policy=POLICY, **assignment, actual_distance_ego_xy_m=actual,
            target_error_m=abs(actual-target), available_road_points=len(candidates),
            selected_road_point=index, retry=attempt, rejected=rejected,
            selection='closest preregistered distance among guarded legal proposals; no outcome-based ranking',
            native_qa_unchanged=True, recovery_sampling_window_unchanged=True)
        meta['secondary_far_vehicle_generation']['initial_distance_spread'] = proof
        scenario = output/'scenarios'/(ident+'.json')
        metadata = output/'scenarios'/(ident+'_metadata.json')
        atomic_json(scenario, scene)
        atomic_json(metadata, meta)
        new_entry = copy.deepcopy(entry)
        new_entry.update(scenario=str(scenario), metadata=str(metadata), scenario_sha256=sha256(scenario))
        entries.append(new_entry)
        changes.append(dict(id=ident, name=plan['target_name'],
            lane_relation=plan['lane_relation'], split=meta['split'],
            source_scenario_sha256=entry['scenario_sha256'], scenario_sha256=new_entry['scenario_sha256'],
            previous_distance_m=before['runtime_actors'][ordinal]['planned_center_distance_m'], **proof))
        if len(changes) % 20 == 0:
            print('far-spread', len(changes), '/160', round(time.monotonic()-started, 1), flush=True)
    result = copy.deepcopy(deck)
    result.update(scenes=entries, status='FAR_DISTANCE_SPREAD_CPU_ONLY; native qualification pending')
    result.setdefault('provenance', {})['far_distance_spread'] = dict(policy=POLICY,
        source_manifest=str(source), source_manifest_sha256=pins[str(source)],
        implementation_sha256=pins[str(Path(__file__).resolve())])
    manifest = output/'manifest.json'
    atomic_json(manifest, result)
    errors = []
    for path, digest in pins.items():
        if sha256(path) != digest:
            errors.append([path, 'source changed during preparation'])
    source_entries = {entry['id']:entry for entry in deck['scenes']}
    for entry in entries:
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            errors.append([entry['id'], 'prepared scenario checksum mismatch'])
        ident = entry['id']
        if ident in assigned:
            verify_transition(original[ident], read(entry['metadata']), read(entry['scenario']), assigned[ident])
        elif entry != source_entries[ident]:
            errors.append([ident, 'non-target entry path/header changed'])
    if [entry['id'] for entry in entries] != [entry['id'] for entry in deck['scenes']]:
        errors.append(['all', 'scene ID/order changed'])
    expected_counts = Counter(str((r['role'], r['sampling_bounds_m'])) for r in assigned.values())
    if Counter(str((r['role'], r['sampling_bounds_m'])) for r in changes) != expected_counts:
        errors.append(['all', 'role/window allocation changed'])
    audit = audit_base5_scenarios.audit(manifest)
    atomic_json(output/'scenario_audit.json', audit)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('errors'):
        errors.append(['full5000', 'offline scenario audit failed'])
    report = dict(schema='base5_far_distance_spread_v1', policy=POLICY, passed=not errors,
        errors=errors, frames=5000, candidate_manifest_sha256=sha256(manifest),
        source_manifest=str(source), source_manifest_sha256=pins[str(source)],
        source_files=pins, changes=changes, preserved_plans=160,
        declared_plan_count=160, geometry_changed_scenes=sum(
            c['scenario_sha256'] != c['source_scenario_sha256'] for c in changes),
        unchanged_entry_paths_and_payloads=4840,
        changed_scenario_payloads=sum(c['scenario_sha256'] != c['source_scenario_sha256'] for c in changes),
        role_window_counts=dict(Counter(str((c['role'], c['sampling_bounds_m'])) for c in changes)),
        bulk_ready=False, bulk_started=False, live_qualified=False,
        limitation='Initial distance diversity, not AP or exact uniform retained-GT distribution; bounded recovery retains original windows.')
    atomic_json(output/'preservation_audit.json', report)
    if errors:
        raise far.FarContractError('Spread candidate audit failed')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.manifest, args.output))
