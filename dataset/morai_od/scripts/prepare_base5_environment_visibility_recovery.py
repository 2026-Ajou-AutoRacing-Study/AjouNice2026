#!/usr/bin/env python3
"""Sunny15의 관측된 가림 두 셀만 유한 CPU 후보로 다시 준비한다.

원본 5K, raw, runtime은 읽기 전용이다. 기존 보충기의 all-parent 점수와
unit/layout 복구 계약을 재사용한다. 서로 독립적인 1장 manifest 두 개를
등록하며 native 선택 ledger나 공통 승인 overlay를 만들어 내지 않는다.
"""
import argparse
from collections import Counter, defaultdict
import copy
import hashlib
import json
import math
from pathlib import Path
import time

import base5_front_completion_adapter as front
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json, sha256
from dynamic_capture_metadata import normalized_sensor_calibration
import prepare_base5_unit_environment as environment
import run_base5_adaptive_completion as adaptive
from run_base5_failure_aware_replenishment_fast import prewarm
import run_base5_quality_loop as quality


ROOT = Path(__file__).resolve().parents[3]
ARTIFACT = ROOT/'artifacts/evaluation/base5_sunny15_261017'
DEFAULT_SOURCE = ARTIFACT/'environment_v4/candidate_sunny15/manifest.json'
DEFAULT_BASELINE = ROOT/'artifacts/evaluation/base5_unit_release_261017/close_primary_fullsource_v1/candidate_v158a/manifest.json'
DEFAULT_LEDGER = ARTIFACT/'environment_native_v1/chunk_1/ledger.jsonl'
EXPECTED_SOURCE_SHA = '1df9c2948b507a8bc141de052202bac6e995e97253e37afb899df011efcd2e12'
EXPECTED_BASELINE_SHA = '4f7fc3b31e23688b43913f2457d570cf014c19e04b2c2e391b463647809e02c5'
EXPECTED_LEDGER_SHA = 'ff021129c0df66a1a94dc5cc69f4c8428101ff48edeb409ce04ca23069a540ee'
TARGETS = {
    'sunny15_sunny13_sunny11_b5v2_03754': dict(
        scenario_sha256='1c3a5bc190c7da4aa8241728f2cf07b1e6755ab3a84613d2c6de208b6800b396',
        reasons=['population_visible_actor_deficit']),
    'sunny15_sunny13_sunny11_b5v2_04027': dict(
        scenario_sha256='e84a49fe4cd265d1b8ab311484ed461c4868f6e64f8842462da92f56110a5900',
        reasons=['primary_no_sensor_evidence']),
}
POLICY = 'sunny15-two-observed-visibility-cells-bounded-cpu-recovery-v160'
MAX_PROPOSALS = 64
MAX_BINDINGS = 8


class RecoveryContractError(RuntimeError):
    """Source/raw/identity mismatches are not recoverable geometry deficits."""


def read(path):
    return json.loads(Path(path).read_text())


def require(condition, message):
    if not condition:
        raise RecoveryContractError(message)


def event(stage, **fields):
    print(json.dumps(dict(event=stage, **fields), ensure_ascii=False), flush=True)


def encoded_scene(scene):
    return (json.dumps(scene, ensure_ascii=False, indent=2)+'\n').encode()


def actor_roster(meta):
    return [(a.get('data_id'), a.get('unique_id'), a.get('name'), a.get('source_list'),
             a.get('population_pool'), a.get('motion'), a.get('asset_scale'))
            for a in meta['runtime_actors']]


def verify_transition(before, after):
    require(after['spec'] == before['spec'], 'Frozen spec/seed/range/phase/yaw policy changed')
    require(after['split'] == before['split'], 'Capture split changed')
    require(actor_roster(after) == actor_roster(before), 'Complete actor identity/motion/scale roster changed')
    require(after.get('retry', -1) > before['retry'], 'Geometry retry did not advance')
    require(after.get('population_budget') == before.get('population_budget'), 'Population budget changed')
    require(all(a.get('motion') in ('static', 'standing') for a in after['runtime_actors']),
            'Only the two observed static scenes may use this recovery')
    primary = after['runtime_actors'][0]
    distance = primary.get('planned_center_distance_m')
    lo, hi = before['spec']['distance_bounds']
    require(isinstance(distance, (int, float)) and math.isfinite(distance)
            and lo <= distance < hi, 'Primary left its frozen distance window')
    require(primary.get('actual_ego_role') == before['runtime_actors'][0].get('actual_ego_role'),
            'Primary observation direction changed')
    if front.has_plan(before):
        front.verify_preserved(before, after)


def snapshot(root):
    root = Path(root).resolve(strict=True)
    require(all(not p.is_symlink() for p in root.rglob('*')), 'Raw evidence cannot contain symlinks')
    return {str(p.resolve()): sha256(p) for p in root.rglob('*') if p.is_file()}


def inspect_reviews(ledger, entries, metadata):
    """Read only the previously fixed two visibility outcomes; never new results."""
    ledger = Path(ledger).resolve(strict=True)
    require(sha256(ledger) == EXPECTED_LEDGER_SHA, 'Frozen original review ledger changed')
    rows = [json.loads(line) for line in ledger.read_text().splitlines() if line.strip()]
    require(1 <= len(rows) <= 16 and len({r['id'] for r in rows}) == len(rows),
            'A bounded unique original chunk ledger is required')
    park = ledger.parent/'shutdown_safety.json'
    require(read(park).get('park_verified') is True, 'Original chunk terminal Park is unverified')
    evidence = {str(ledger): sha256(ledger), str(park): sha256(park)}
    failures = {}
    for row in rows:
        ident = row['id']
        require(ident in entries and row['scenario_sha256'] == entries[ident]['scenario_sha256'],
                'Native row is foreign or has a different source payload')
        require(row['status'] in ('accepted_candidate', 'review'), 'Unsafe original chunk cannot be recovered')
        raw = Path(row['capture_root']).resolve(strict=True)
        require(ledger.parent/'captures' in raw.parents, 'Review raw escaped its original chunk')
        audit = read(raw/'audit.json')
        require(audit == row['audit'] and audit.get('raw_complete') is True, 'Original native audit differs')
        collision = audit.get('capture_safety', {}).get('collision', {})
        require(collision.get('observed') is True and collision.get('no_collision_verified') is True
                and not collision.get('non_ego_uids')
                and audit.get('capture_safety', {}).get('support', {}).get('stationary_support_observed') is True,
                'Collision-clear and stationary receive evidence is required')
        if ident not in TARGETS:
            # A separate ground-quality review in the same chunk remains an
            # untouched deficit. Its outcome cannot expand these two targets.
            continue
        require(audit.get('accepted_candidate') is False and row['status'] == 'review'
                and audit.get('review_reasons') == TARGETS[ident]['reasons']
                and not audit.get('missing_uids') and not audit.get('runtime_missing_uids'),
                'Original registered visibility review differs')
        captured = read(raw/'metadata.json')
        require(captured['spec'] == metadata[ident]['spec']
                and actor_roster(captured) == actor_roster(metadata[ident])
                and sha256(raw/'scenario.json') == TARGETS[ident]['scenario_sha256']
                == entries[ident]['scenario_sha256'], 'Review raw differs from the frozen source')
        evidence.update(snapshot(raw))
        failures[ident] = dict(capture_root=str(raw), reasons=audit['review_reasons'],
            audit_sha256=sha256(raw/'audit.json'), scenario_sha256=row['scenario_sha256'])
    require(set(failures) == set(TARGETS), 'Both registered original reviews are required')
    return failures, evidence


def observed_payloads(native_root):
    """Exclude all payloads actually exported in the original smoke, not filenames."""
    hashes, evidence = set(), {}
    ledgers = sorted(Path(native_root).glob('chunk_*/ledger.jsonl'))
    require(ledgers, 'Original native payload inventory is missing')
    for ledger in ledgers:
        park = ledger.parent/'shutdown_safety.json'
        require(read(park).get('park_verified') is True, 'Original native inventory has unsafe termination')
        evidence.update({str(ledger.resolve()): sha256(ledger), str(park.resolve()): sha256(park)})
        for row in (json.loads(line) for line in ledger.read_text().splitlines() if line.strip()):
            require(row.get('status') in ('accepted_candidate', 'review'), 'Unsafe native inventory row')
            raw = Path(row['capture_root']).resolve(strict=True)
            require(ledger.parent.resolve()/'captures' in raw.parents, 'Native raw escaped its chunk')
            scene = raw/'scenario.json'
            require(sha256(scene) == row['scenario_sha256'], 'Observed native payload changed')
            hashes.add(row['scenario_sha256'])
            evidence[str(scene)] = sha256(scene)
    return hashes, evidence


def search(before, config, ctx, calibration, known, maximum=32, build=None, scorer=None):
    """Only metadata proxy ranking. Exactly bounded attempted retries, not valid count."""
    require(type(maximum) is int and maximum in (32, MAX_PROPOSALS), 'Explicit CPU attempt cap must be 32 or 64')
    build, scorer = build or spatial.build, scorer or adaptive.proposal_score
    audit, ranked = [], []
    first = before['retry']+1
    for retry in range(first, first+maximum):
        try:
            scene, meta = build(copy.deepcopy(before['spec']), config, ctx, retry)
        except ValueError as error:
            audit.append(dict(retry=retry, status='generation_rejected', reason=str(error)))
            continue
        try:
            verify_transition(before, meta)
        except RecoveryContractError as error:
            audit.append(dict(retry=retry, status='contract_candidate_rejected', reason=str(error)))
            continue
        checksum = hashlib.sha256(encoded_scene(scene)).hexdigest()
        if checksum in known:
            audit.append(dict(retry=retry, status='observed_or_source_payload_rejected', scenario_sha256=checksum))
            continue
        proxy = scorer(meta, calibration, 'all_actor_visibility')
        require(isinstance(proxy.get('score'), (int, float)) and math.isfinite(proxy['score']),
                'Nonfinite all-parent proposal score')
        row = dict(retry=retry, scenario_sha256=checksum, proxy=proxy, status='rankable_cpu_proposal')
        audit.append(row)
        # A local proposal preference, never a change to capture/GT acceptance.
        primary = proxy['primary']
        plausible = ((primary['lidar_beam_proxy'] > 0 or primary['camera_min_extent_proxy_px'] >= 4)
                     and primary['angular_occlusion_proxy'] < .85
                     and proxy['predicted_visible_parents'] >= proxy['required_visible_parents'])
        row['same_existing_proxy_predicts_required_parents'] = bool(plausible)
        if plausible:
            ranked.append((float(proxy['score']), -retry, scene, meta, row))
    ranked.sort(key=lambda value: value[:2], reverse=True)
    require(ranked, 'Bounded CPU search found no same-contract plausible visibility proposal')
    return ranked, dict(maximum_geometry_attempts=maximum, geometry_attempts=maximum,
        rankable_candidates=sum(r['status'] == 'rankable_cpu_proposal' for r in audit),
        plausible_candidates=len(ranked), proposal_score='run_base5_adaptive_completion.proposal_score/all_actor_visibility',
        attempts=audit, selection_uses_new_native_outcomes=False, native_qa_unchanged=True,
        limitation='Legacy camera rectangle/beam/angular proxies; no rendered road-mesh visibility certificate')


def joint_audit(source_rows, selections, base):
    """Audit both planned replacements together without a fake selected-native deck."""
    rows, assignments = [], copy.deepcopy(base['assignments'])
    for old in source_rows:
        ident = old['id']
        current = selections.get(ident)
        row = dict(old)
        if current:
            row.update(meta=current['meta'], entry=current['entry'], scenario_sha256=current['entry']['scenario_sha256'])
            binding = layout.resolve(current['overlay'], current['entry']['id'], current['entry'], current['meta'])
            expected = base['assignments'][ident]
            require(binding['split'] == expected['split'] and binding['group_id'] == expected['group_id'],
                    'Effective split/layout group changed')
            assignments[ident] = copy.deepcopy(binding)
        rows.append(row)
    audit = layout.audit_assignments(rows, assignments, base['policy'])
    hard_groups = defaultdict(set)
    for row in rows:
        binding = assignments[row['id']]
        if binding['split'] == 'train':
            hard_groups[binding['group_id']].update(layout.features(row['meta'], base['policy'])[0])
    remaining = Counter()
    for values in hard_groups.values():
        remaining.update(values)
    floors = base['audit'].get('train_floors', {})
    missing = [key for key, floor in floors.items() if remaining[key] < floor]
    require(not audit['errors'] and not missing, 'Joint virtual full-deck layout/coverage floor failed: '+str(audit['errors']+missing))
    audit.update(train_floors_preserved=True, changed_cells=len(selections), unchanged_cells=len(rows)-len(selections),
        virtual_planned_deck_only=True, common_capture_overlay_created=False,
        native_selected_ledger_created=False, native_qualified=False, bulk_ready=False)
    return rows, audit


def bind_candidate(entry, before, ranked, deck, output, failure, ledger, context, source):
    """Try no more than eight independently audited existing layout bindings."""
    failures = []
    for rank, (_, _, scene, proposed, proof) in enumerate(ranked[:MAX_BINDINGS], 1):
        folder = output/('binding_'+str(rank).zfill(2))
        folder.mkdir(parents=True)
        meta = copy.deepcopy(proposed)
        for key in ('scene_group_id', 'environment_split_contract', 'logical_quota_slot_id',
                    'fresh_environment_provenance', 'obstacle_support_pair'):
            if key in before:
                meta[key] = copy.deepcopy(before[key])
        meta['environment_visibility_recovery'] = dict(policy=POLICY, source_cell_id=entry['id'],
            original_source_manifest=str(source), original_source_manifest_sha256=sha256(source),
            original_source_entry=copy.deepcopy(entry), original_metadata_sha256=sha256(entry['metadata']),
            review_ledger=str(ledger), review_ledger_sha256=sha256(ledger), original_failure=copy.deepcopy(failure),
            selected_cpu_proposal=copy.deepcopy(proof), native_qualified=False, bulk_ready=False,
            native_qa_unchanged=True, selection_uses_new_native_outcomes=False)
        ident = entry['id']+'_visibility01'
        scenario, metadata = folder/(ident+'.json'), folder/(ident+'_metadata.json')
        atomic_json(scenario, scene)
        atomic_json(metadata, meta)
        updated = copy.deepcopy(entry)
        updated.update(id=ident, source_cell_id=entry['id'], scenario=str(scenario), metadata=str(metadata),
            scenario_sha256=sha256(scenario))
        require(updated['scenario_sha256'] == proof['scenario_sha256'], 'Serialized proposed payload changed')
        candidate = copy.deepcopy(deck)
        candidate.update(count=1, scenes=[updated], status='VISIBILITY_RETRY_CPU_REGISTERED; native pending',
            native_qualified=False, bulk_ready=False)
        try:
            reference = quality.bind_replacement_contract(entry, before, updated, meta, candidate, folder, context=context)
        except quality.ReplacementGeometryRejected as error:
            failures.append(dict(rank=rank, retry=proof['retry'], reason=str(error)))
            continue
        require(reference is not None, 'Existing recovery overlay registration is mandatory')
        verify_transition(before, meta)
        check = layout.validate_manifest_overlay(candidate, allow_subset=True)
        manifest = folder/'manifest.json'
        atomic_json(manifest, candidate)
        return dict(manifest=str(manifest), entry=updated, meta=meta,
            overlay=layout.overlay_for_metadata(meta), overlay_reference=reference,
            binding_rank=rank, source_cell_id=entry['id'], cpu_proposal=proof,
            prior_layout_binding_rejections=failures, subset_overlay_audit=check)
    raise RecoveryContractError('No same-contract geometry within eight existing layout bindings: '+str(failures))


def prepare(source, ledger, baseline, output, maximum=32):
    started = time.monotonic()
    source, ledger, baseline = (Path(p).resolve(strict=True) for p in (source, ledger, baseline))
    raw_output = Path(output).expanduser()
    require(not raw_output.is_symlink(), 'Output cannot be a symlink')
    output = raw_output.resolve()
    require(not output.exists(), 'Fresh visibility recovery output required; no implicit resume')
    require(type(maximum) is int and maximum in (32, MAX_PROPOSALS), 'Explicit CPU attempt cap must be 32 or 64')
    for protected in (source.parent, baseline.parent, ledger.parent.parent, spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()):
        require(output != protected and protected not in output.parents, 'Output overlaps immutable source/raw/runtime')
    require(sha256(source) == EXPECTED_SOURCE_SHA, 'Only the frozen Sunny15 environment_v4 source may recover')
    require(sha256(baseline) == EXPECTED_BASELINE_SHA, 'Frozen Sunny13 baseline changed')
    event('checking_complete_source_and_original_reviews')
    deck, rows = environment.records(source)
    require(deck.get('count') == len(rows) == 5000 and deck['config'].get('environment_tag') == 'sunny15',
            'Complete Sunny15 5K source required')
    entries, metadata = {r['id']:r['entry'] for r in rows}, {r['id']:r['meta'] for r in rows}
    require(set(TARGETS) <= set(entries), 'Registered visibility targets are missing')
    require(all(entries[ident]['scenario_sha256'] == target['scenario_sha256'] for ident, target in TARGETS.items()),
            'Registered original target payload changed')
    layout.validate_manifest_overlay(deck)
    base = layout.overlay_for_metadata(rows[0]['meta'])
    failure_rows, evidence = inspect_reviews(ledger, entries, metadata)
    known, observed_pins = observed_payloads(ledger.parent.parent)
    original_native_payload_count = len(known)
    known.update(entry['scenario_sha256'] for entry in deck['scenes'])
    evidence.update(observed_pins)
    event('pinning_readonly_source_raw_and_runtime', original_native_payloads=original_native_payload_count)
    inputs = pins.collect_inputs(source)
    baseline_doc, baseline_rows = environment.records(baseline)
    require(baseline_doc.get('count') == len(baseline_rows) == 5000, 'Complete Sunny13 baseline required')
    inputs[str(baseline)] = sha256(baseline)
    for row in baseline_rows:
        inputs[str(Path(row['entry']['metadata']).resolve())] = sha256(row['entry']['metadata'])
        inputs[str(Path(row['entry']['scenario']).resolve())] = row['scenario_sha256']
    inputs.update(evidence)
    runtime = pins.collect_runtime(entrypoints=(Path(__file__).name,))
    output.mkdir(parents=True)
    atomic_json(output/'request.json', dict(policy=POLICY, source=str(source), source_sha256=sha256(source),
        original_review_ledger=str(ledger), original_review_ledger_sha256=sha256(ledger),
        baseline=str(baseline), baseline_sha256=sha256(baseline), targets=TARGETS,
        maximum_geometry_attempts_per_cell=maximum, maximum_layout_bindings_per_cell=MAX_BINDINGS,
        native_capture_started=False, bulk_ready=False, native_qualified=False))
    ctx = prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR, spatial.gen.geo.DEFAULT_MGEO), deck['config'])
    calibration = normalized_sensor_calibration(ROOT/'configs/sensors/Sensor_competition_local.json')
    selections = {}
    for ident in sorted(TARGETS):
        event('searching_same_contract_visibility_geometry', cell=ident, maximum_cpu_attempts=maximum)
        frozen = quality.replacement_contract(deck, entries[ident], metadata[ident])
        ranked, audit = search(metadata[ident], deck['config'], ctx, calibration, known, maximum)
        folder = output/'cells'/ident
        atomic_json(folder/'cpu_proposals.json', audit)
        selected = bind_candidate(entries[ident], metadata[ident], ranked, deck, folder, failure_rows[ident], ledger, frozen, source)
        selections[ident] = selected
        known.add(selected['entry']['scenario_sha256'])
        event('one_cell_cpu_overlay_registered', cell=ident, manifest=selected['manifest'],
            retry=selected['cpu_proposal']['retry'], binding_rank=selected['binding_rank'])
    virtual_rows, joint = joint_audit(rows, selections, base)
    atomic_json(output/'virtual_full5000_audit.json', joint)
    cross = environment.cross_environment_audit(baseline_rows, virtual_rows)
    atomic_json(output/'cross_environment_audit.json', cross)
    require(cross['passed'], 'Proposed two-cell virtual deck duplicates Sunny13 geometry')
    pins.verify_hashes(inputs, 'Preserved original source/native evidence')
    pins.verify_hashes(runtime, 'Unchanged source/runtime implementation')
    registrations = []
    for ident, selected in selections.items():
        registrations.append(dict(source_cell_id=ident, candidate_id=selected['entry']['id'],
            source_manifest_index=next(i for i, entry in enumerate(deck['scenes']) if entry['id'] == ident),
            manifest=selected['manifest'], manifest_sha256=sha256(selected['manifest']), capture_indices=[0],
            scenario_sha256=selected['entry']['scenario_sha256'], metadata_sha256=sha256(selected['entry']['metadata']),
            split_overlay=selected['overlay_reference'], cpu_proposal=selected['cpu_proposal'],
            binding_rank=selected['binding_rank'], prior_layout_binding_rejections=selected['prior_layout_binding_rejections'],
            native_qualified=False))
    result = dict(schema='base5_environment_visibility_recovery_cpu_registration_v1', policy=POLICY,
        status='TWO_INDEPENDENT_ONE_CELL_CPU_MANIFESTS; native pending', registrations=registrations,
        source_manifest=str(source), source_manifest_sha256=sha256(source), source_input_hashes=inputs,
        original_review_ledger=str(ledger), original_review_ledger_sha256=sha256(ledger), runtime_hashes=runtime,
        virtual_full_deck_audit=dict(path=str(output/'virtual_full5000_audit.json'), sha256=sha256(output/'virtual_full5000_audit.json')),
        cross_environment_audit=dict(path=str(output/'cross_environment_audit.json'), sha256=sha256(output/'cross_environment_audit.json')),
        maximum_geometry_attempts_per_cell=maximum, maximum_layout_bindings_per_cell=MAX_BINDINGS,
        elapsed_sec=time.monotonic()-started, source_and_raw_modified=False, common_overlay_created=False,
        native_selected_ledger_created=False, native_qualified=False, bulk_ready=False, native_capture_started=False,
        limitation='Independent proposal overlays are not a finalized joint selected-native overlay; only later actual capture may qualify them.')
    atomic_json(output/'registration.json', result)
    event('cpu_registration_complete', registration=str(output/'registration.json'), native_qualified=False)
    return output/'registration.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--original-review-ledger', type=Path, default=DEFAULT_LEDGER)
    parser.add_argument('--sunny13-baseline', type=Path, default=DEFAULT_BASELINE)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--maximum-proposals', type=int, choices=(32, 64), default=32)
    args = parser.parse_args()
    prepare(args.manifest, args.original_review_ledger, args.sunny13_baseline, args.output, args.maximum_proposals)
