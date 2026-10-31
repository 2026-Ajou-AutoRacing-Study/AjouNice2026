#!/usr/bin/env python3
"""Freeze v152 range contracts after current-source, bounded native trials.

No previous release/raw is edited and no capture or training is launched.
When a real pilot has no deficit, qualification names the distinct evidence:
terminal no-capture replay, four planned native branches and CPU orchestration.
It never manufactures a failed parent or upgrades a native QA review.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import base5_front_release_inputs as pins
import base5_primary_obstacle_range as primary
from base5_capture_safety import atomic_json, load_ledger, sha256
import finalize_base5_front_support_release as legacy
import prepare_base5_primary_range_qualification as qualification
import validate_base5_primary_range_recovery as recovery
from verify_base5_completed_resume import snapshot as capture_snapshot

ORCHESTRATION_SCHEMA = 'base5_primary_range_orchestration_v1'
ORCHESTRATION_CHECKS = ('no_deficit_terminal_resume_no_capture', 'frozen_primary_range_replacement',
    'front_overlap_preserved', 'source_contract_error_halts', 'ordinary_geometry_exhaustion_is_bounded',
    'cumulative_attempt_cap', 'native_review_never_approved')


def read(path):
    return json.loads(Path(path).read_text())


def master_path(path):
    path = Path(path)
    return path / 'registration.json' if path.is_dir() else path


def verify_registration(path, manifest, deck, entries, metadata, planned):
    path = master_path(path)
    registered = read(path)
    inventory = [dict(id=e['id'], scenario_sha256=e['scenario_sha256'],
                      metadata_sha256=sha256(e['metadata'])) for e in deck['scenes']]
    if (registered.get('schema') != qualification.SCHEMA or registered.get('policy') != qualification.POLICY
            or registered.get('source_manifest_sha256') != sha256(manifest)
            or registered.get('source_inventory_sha256') != primary.digest(inventory)
            or registered.get('range_helper_sha256') != primary.implementation_sha256()
            or registered.get('front_helper_sha256') != legacy.front.implementation_sha256()
            or registered.get('front_allocation_selected') != 338):
        raise ValueError('Registered v152 complete source/helper/allocation differs')
    pins.verify_hashes(registered['input_sha256'], 'Range qualification source')
    references = [registered['front_registration']] + list(registered['manifests'].values())
    pins.verify_hashes({row['path']: row['sha256'] for row in references}, 'Range qualification manifests')
    front, static_manifest, moving_manifest = legacy.verify_registration(
        Path(registered['front_registration']['path']), manifest, entries, metadata)
    cohort = registered.get('range_static', [])
    if (registered.get('range_static_count') != len(planned) or len(cohort) != len(planned)
            or [row['id'] for row in cohort] != planned):
        raise ValueError('Every declared range contract must enter the registered cohort')
    for row in cohort:
        ident = row['id']
        if (row['range_plan'] != metadata[ident]['spec'][primary.PLAN_KEY]
                or row['scenario_sha256'] != entries[ident]['scenario_sha256']
                or row['metadata_sha256'] != sha256(entries[ident]['metadata'])):
            raise ValueError('Range cohort source payload/plan changed')
    strata = qualification.select_strata(entries, metadata, planned)
    if registered.get('strata4') != strata:
        raise ValueError('Range four-stratum preregistration changed')
    for name, ids in (('static64', [row['id'] for row in front['static']]),
                      ('animated8', [row['id'] for row in front['animated']]),
                      ('range_static', planned), ('range_recovery4', [row['source_id'] for row in strata])):
        subset = read(registered['manifests'][name]['path'])
        if subset.get('count') != len(ids) or subset.get('scenes') != [entries[ident] for ident in ids]:
            raise ValueError('Range qualification subset differs from its frozen source')
    return registered, front, static_manifest, moving_manifest


def verify_range_rows(registered, rows, metadata):
    by_id = {row['source_cell_id']: row for row in rows}
    if len(by_id) != len(rows) or not set(by_id) <= {row['id'] for row in registered['range_static']}:
        raise ValueError('Range native selection duplicates or imports another source')
    observed, missing = [], []
    for row in registered['range_static']:
        ident = row['id']
        if ident not in by_id:
            missing.append(ident)
            continue
        selected = by_id[ident]
        if selected['status'] != 'accepted_candidate' or selected.get('audit', {}).get('review_reasons'):
            raise ValueError('Range QA review cannot become an approval')
        actual = read(Path(selected['capture_root']) / 'metadata.json')
        plan = primary.verify_preserved(metadata[ident], actual)
        if plan != row['range_plan']:
            raise ValueError('Captured range plan/actor identity differs')
        observed.append(dict(id=ident, data_id=plan['primary_data_id'], cap_m=plan['cap_m'],
            actual_primary_distance_m=actual['runtime_actors'][0]['planned_center_distance_m'],
            has_front_plan=legacy.adapter.has_plan(actual), frozen_spec_ids_seed_split_preserved=True))
    return observed, missing


def verify_probe_proof(path, manifest, registration, deck, metadata, pools):
    path = Path(path)
    path = path / 'proof.json' if path.is_dir() else path
    proof = read(path)
    if (proof.get('schema') != recovery.SCHEMA or proof.get('passed') is not True
            or proof.get('source_manifest_sha256') != sha256(manifest)
            or proof.get('registration_sha256') != sha256(registration)
            or proof.get('prepared_count') != 4 or proof.get('accepted_count') != 4
            or proof.get('planned_probe') is not True or proof.get('actual_parent_deficit') is not False
            or proof.get('training_data') is not False or proof.get('bulk_started') is not False
            or proof.get('additional_completed_resume_raw') != 0):
        raise ValueError('Four current-source planned native branches and terminal replay required')
    state_path = Path(proof['state_path'])
    if state_path.resolve() != (path.parent / 'state.json').resolve() or sha256(state_path) != proof['state_sha256']:
        raise ValueError('Range recovery state path/checksum changed')
    state = read(state_path)
    snapshot_deck, entries, source_metadata, strata = recovery.source_inputs(manifest, registration)
    if (snapshot_deck != deck or state.get('stage') != 'complete' or len(state.get('attempts', [])) != 4
            or state.get('contract') != recovery.contract(manifest, registration, deck, strata)
            or proof.get('strata4') != strata):
        raise ValueError('Range recovery full-source/runtime contract differs')
    recovery.verify_prepared(path.parent, state, deck, metadata)
    before = recovery.snapshot(state)
    if (before != state.get('terminal_inventory') or before.get('captured_rows') != 4
            or before.get('ledger_sha256') != proof.get('ledger_sha256')):
        raise ValueError('Range recovery terminal raw/ledger evidence changed')
    pins.verify_hashes(state['contract']['runtime_hashes'], 'Range recovery runtime')
    pins.verify_hashes(proof['ledger_sha256'], 'Range recovery terminal ledgers')
    for attempt in state['attempts']:
        ident = attempt['source_id']
        actual = recovery.verify_attempt(attempt, deck, metadata[ident], pools)
        if actual != attempt.get('result') or actual.get('status') != 'accepted_candidate':
            raise ValueError('Range branch is not an official native approval')
        primary.verify_preserved(metadata[ident], read(Path(actual['capture_root']) / 'metadata.json'))
    if recovery.snapshot(state) != before:
        raise ValueError('Range raw changed during official verification')
    return proof, path


def verify_orchestration(path, manifest, runtime_hashes):
    doc = read(path)
    tests = doc.get('tests', {})
    if (doc.get('schema') != ORCHESTRATION_SCHEMA or doc.get('passed') is not True
            or doc.get('source_manifest_sha256') != sha256(manifest)
            or doc.get('runtime_hashes') != runtime_hashes
            or tests.get('exit_code') != 0 or tests.get('failures') != 0 or tests.get('errors') != 0
            or not isinstance(tests.get('tests_run'), int) or isinstance(tests['tests_run'], bool)
            or tests['tests_run'] < 1 or not tests.get('command')
            or any(doc.get('checks', {}).get(name) is not True for name in ORCHESTRATION_CHECKS)):
        raise ValueError('Current-source CPU orchestration receipt incomplete')
    log = doc['test_log']
    pins.verify_hashes({log['path']: log['sha256']}, 'CPU orchestration test log')
    pins.verify_hashes(runtime_hashes, 'CPU orchestration runtime')
    if doc.get('test_files'):
        pins.verify_hashes(doc['test_files'], 'CPU orchestration tests')
    return doc


def qualification_basis(real_completion, replenished, unresolved, range_missing, front_count):
    blockers = []
    if unresolved:
        blockers.append('Native static completion retains %d unresolved slot(s): %s' % (len(unresolved), ', '.join(unresolved)))
    if range_missing:
        blockers.append('Declared primary-range cohort retains %d native deficit(s): %s' % (len(range_missing), ', '.join(range_missing)))
    if front_count != 38:
        blockers.append('Native front strata approved %d/38' % front_count)
    mode = 'actual_native_parent_deficit_replenished' if real_completion else 'zero_deficit_terminal_replay_planned_native_branches_and_cpu_orchestration'
    return dict(mode=mode, actual_deficit_replenished=replenished,
        planned_recovery_native_approved=4, completed_resume_additional_native_captures=0,
        cpu_orchestration_qualified=True, fabricated_parent_deficit=False), blockers


def _verify_zero_parent_partial(partial, static, manifest):
    """Use the existing partial16 evidence contract, including real parent raw."""
    if (partial.get('passed') is not True or partial.get('preserved_selected') != 16
            or partial.get('prior_raw_unchanged') is not True
            or Path(partial['parent_root']).resolve() != static.resolve()
            or partial.get('source_manifest_sha256') != sha256(manifest)):
        raise ValueError('Zero-deficit partial16 parent/manifest evidence differs')
    before, after = legacy.read_snapshot(partial, 'before'), legacy.read_snapshot(partial, 'after')
    if (Path(after['parent_root']).resolve() != static.resolve()
            or after.get('source_manifest_sha256') != sha256(manifest)):
        raise ValueError('Zero-deficit partial after snapshot source differs')
    before_rows, after_rows = before['selected_rows'], after['selected_rows']
    by_id = {row['source_cell_id']: row for row in after_rows}
    if (len(before_rows) != 16 or len({row['source_cell_id'] for row in before_rows}) != 16
            or len(by_id) != len(after_rows)
            or any(by_id.get(row['source_cell_id']) != row for row in before_rows)
            or after_rows != load_ledger(static / 'selected_ledger.jsonl')):
        raise ValueError('Zero-deficit partial16 selections were not preserved')
    if not before['snapshot'].get('raw_files'):
        raise ValueError('Zero-deficit partial16 lacks its original parent raw')
    for name in ('raw_files', 'event_logs'):
        if any(after['snapshot'].get(name, {}).get(key) != value
               for key, value in before['snapshot'].get(name, {}).items()):
            raise ValueError('Zero-deficit partial16 raw/event evidence changed')
    legacy.verify_file_snapshot(after['snapshot'], static)


def _verify_zero_native_absent(completed):
    """An empty completion journal cannot hide owned native/control evidence."""
    native_names = {'ledger.jsonl', 'session_contract.json', 'shutdown_safety.json',
                    'event_cmd_requests.jsonl'}
    for path in completed.rglob('*'):
        if not path.is_file():
            continue
        relative = path.relative_to(completed)
        if ('captures' in relative.parts or path.name in native_names
                or any(part.startswith(('CAMERA_', 'LIDAR_')) for part in relative.parts)
                or path.suffix.lower() in ('.bin', '.png', '.jpg', '.jpeg', '.pcd', '.bag')):
            raise ValueError('Zero-deficit completion contains unexpected owned native/control data')


def verify_resume(resume_path, partial_path, static, completed, manifest):
    """Accept empty *owned* raw only for a genuinely empty v152 completion.

Real deficit recovery retains the unchanged v151 verifier. The zero branch
does not fabricate raw: its accepted rows point to the verified parent's raw,
while its own journal has no proposal, ROS child or native capture.
"""
    state, status = read(completed / 'state.json'), read(completed / 'status.json')
    if not isinstance(state.get('deficits'), list):
        raise ValueError('Completion lacks an explicit deficit journal')
    if state['deficits']:
        return legacy.verify_resume(resume_path, partial_path, static, completed, manifest)
    parent_rows = load_ledger(static / 'selected_ledger.jsonl')
    selected = load_ledger(completed / 'selected_ledger.jsonl')
    quota = read(completed / 'quota.json')
    if (state.get('stage') != 'complete' or status.get('stage') != 'complete'
            or state.get('attempts') != [] or state.get('baseline_selected') != parent_rows
            or selected != parent_rows or len(parent_rows) != 64
            or len({row['source_cell_id'] for row in parent_rows}) != 64
            or any(row.get('status') != 'accepted_candidate' for row in selected)
            or status.get('requested') != 64 or status.get('accepted_candidate') != 64
            or status.get('replenished') != 0 or status.get('review') != 0
            or status.get('remaining_deficit') != 0
            or quota.get('target') != 64 or quota.get('accepted') != 64
            or quota.get('deficit') != 0 or quota.get('pending') != 0 or quota.get('cells') != []):
        raise ValueError('Empty-owned-raw exception requires a genuine zero-deficit64 completion')
    _verify_zero_native_absent(completed)
    contract = state.get('contract', {})
    if (contract.get('source_sha256') != sha256(manifest)
            or Path(contract['baseline']).resolve() != static.resolve()
            or contract.get('capture_contract') != legacy.completion.adaptive.fa.batch.campaign.contract(
                manifest, list(range(64)), 2)):
        raise ValueError('Zero-deficit completion source/runtime contract changed')
    code_pins = {}
    for name, digest in contract.get('code_hashes', {}).items():
        if Path(name).name != name:
            raise ValueError('Zero-deficit runtime pin is not a local script basename')
        code_pins[str(Path(__file__).with_name(name).resolve())] = digest
    pins.verify_hashes(code_pins, 'Zero-deficit completion runtime')
    required_parent = {'selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json'}
    if set(contract.get('parent_hashes', {})) != required_parent:
        raise ValueError('Zero-deficit completion parent input pins incomplete')
    pins.verify_hashes({str((static / name).resolve()): digest
                       for name, digest in contract['parent_hashes'].items()}, 'Zero-deficit parent inputs')
    proof, partial = read(resume_path), read(partial_path)
    if proof.get('passed') is not True or proof.get('additional_native_captures') != 0:
        raise ValueError('Zero-deficit terminal replay must add no native data')
    expected_ledgers = {(root / 'selected_ledger.jsonl').resolve() for root in (static, completed)}
    if not expected_ledgers <= {Path(path).resolve() for path in proof.get('ledger_sha256', {})}:
        raise ValueError('Zero-deficit terminal proof must pin both selected ledgers')
    pins.verify_hashes(proof['ledger_sha256'], 'Zero-deficit terminal ledgers')
    _verify_zero_parent_partial(partial, static, manifest)
    before, after = legacy.read_snapshot(proof, 'before'), legacy.read_snapshot(proof, 'after')
    if (before != after or after.get('source_manifest_sha256') != sha256(manifest)
            or Path(after['parent_root']).resolve() != static.resolve()
            or Path(after['completion_root']).resolve() != completed.resolve()):
        raise ValueError('Zero-deficit terminal snapshots/source roots changed')
    legacy.verify_file_snapshot(after['parent'], static)
    empty = after['completion']
    if (empty.get('raw_files') != {} or empty.get('event_logs') != {}
            or empty.get('selected_sha256') != sha256(completed / 'selected_ledger.jsonl')):
        raise ValueError('Zero-deficit terminal completion snapshot is not empty-owned-raw')
    for key, root in (('parent', static), ('completion', completed)):
        if after[key].get('cumulative_elapsed_sec') != read(root / 'status.json').get('elapsed_sec'):
            raise ValueError('Zero-deficit terminal cumulative elapsed changed')
    state_path = str((completed / 'state.json').resolve())
    if (after.get('state_sha256', {}).get(state_path) != sha256(completed / 'state.json')):
        raise ValueError('Zero-deficit terminal snapshot does not pin its unchanged state')
    for name, required in (('state_sha256', {completed / 'state.json'}),
                           ('status_sha256', {static / 'status.json', completed / 'status.json'})):
        values = proof.get(name, {})
        if not {path.resolve() for path in required} <= {Path(path).resolve() for path in values}:
            raise ValueError('Zero-deficit terminal state/status proof incomplete')
        pins.verify_hashes(values, 'Zero-deficit terminal state/status')
    return proof, partial


def verify_range_completion(root, parent, manifest, deck, baseline):
    """Reconstruct finite completion for the registered range subset, not64."""
    count = len(deck['scenes'])
    status = legacy.verify_native(root, count)
    state = read(root / 'state.json')
    ids = [row['id'] for row in deck['scenes']]
    deficits = set(ids) - {row['source_cell_id'] for row in baseline}
    if (state.get('stage') not in legacy.completion.TERMINAL
            or status.get('stage') != state['stage'] or state.get('baseline_selected') != baseline
            or state.get('source_order') != ids or len(set(state.get('deficits', []))) != len(state.get('deficits', []))
            or set(state.get('deficits', [])) != deficits):
        raise ValueError('Range completion parent/registered subset/actual deficit differs')
    contract = state.get('contract', {})
    if (contract.get('source_sha256') != sha256(manifest)
            or Path(contract['baseline']).resolve() != parent.resolve()
            or contract.get('capture_contract') != legacy.completion.adaptive.fa.batch.campaign.contract(
                manifest, list(range(count)), 2)):
        raise ValueError('Range completion was produced under another source/runtime contract')
    code_pins = {}
    for name, digest in contract.get('code_hashes', {}).items():
        if Path(name).name != name:
            raise ValueError('Range completion runtime pin is not a local basename')
        code_pins[str(Path(__file__).with_name(name).resolve())] = digest
    pins.verify_hashes(code_pins, 'Range completion runtime')
    expected_parent = {'selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json'}
    if set(contract.get('parent_hashes', {})) != expected_parent:
        raise ValueError('Range completion parent pins incomplete')
    pins.verify_hashes({str((parent / name).resolve()): digest
                       for name, digest in contract['parent_hashes'].items()}, 'Range completion parent inputs')
    cap = contract.get('attempts')
    if not isinstance(cap, int) or isinstance(cap, bool) or not 1 <= cap <= 5:
        raise ValueError('Range completion needs a finite durable attempt limit')
    seen, lookup = set(), {row['id']: row for row in deck['scenes']}
    ctx = legacy.completion.spatial.gen.context(legacy.completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                               legacy.completion.spatial.gen.geo.DEFAULT_MGEO)
    pools = legacy.completion.pool_map(ctx)
    for attempt in state.get('attempts', []):
        key = (attempt['id'], attempt['ordinal'])
        if (attempt['id'] not in deficits or key in seen or not isinstance(attempt['ordinal'], int)
                or isinstance(attempt['ordinal'], bool) or not 1 <= attempt['ordinal'] <= cap
                or not attempt.get('result')):
            raise ValueError('Range completion has a duplicate/foreign/unresolved/out-of-budget attempt')
        seen.add(key)
        shutdown = Path(attempt['capture']) / 'shutdown_safety.json'
        if not shutdown.is_file() or read(shutdown).get('park_verified') is not True:
            raise ValueError('Range completion attempt lacks verified final Park')
        original = read(lookup[attempt['id']]['metadata'])
        observed = legacy.completion.verify(attempt, deck=deck, source=original, pools=pools)
        if observed != attempt['result']:
            raise ValueError('Range completion official native verification changed')
    rows, missing = legacy.completion.merged(state)
    actual_missing = set(ids) - {row['source_cell_id'] for row in rows}
    quota = read(root / 'quota.json')
    if (len({row['source_cell_id'] for row in rows}) != len(rows)
            or not {row['source_cell_id'] for row in rows} <= set(ids)
            or set(missing) != actual_missing or (state['stage'] == 'complete') != (not missing)
            or rows != load_ledger(root / 'selected_ledger.jsonl')
            or any(row.get('status') != 'accepted_candidate' or row.get('audit', {}).get('review_reasons') for row in rows)
            or status.get('requested') != count or status.get('accepted_candidate') != len(rows)
            or status.get('review') != len(missing) or status.get('remaining_deficit') != len(missing)
            or status.get('replenished') != len(rows) - len(baseline)
            or quota.get('target') != count or quota.get('accepted') != len(rows)
            or quota.get('deficit') != len(missing) or quota.get('pending') != 0
            or {row['id'] for row in quota.get('cells', [])} != actual_missing):
        raise ValueError('Range completion merged selection/count/explicit deficit differs')
    return rows, state, status


def verify_range_completion_resume(path, parent, completed, manifest):
    proof = read(path)
    if proof.get('passed') is not True or proof.get('additional_native_captures') != 0:
        raise ValueError('Range completion terminal replay must add no native capture')
    source_sha = sha256(manifest)
    if (proof.get('source_manifest_sha256') != source_sha
            or Path(proof['parent_root']).resolve() != parent.resolve()
            or Path(proof['completion_root']).resolve() != completed.resolve()):
        raise ValueError('Range terminal proof belongs to another source/parent')
    before, after = legacy.read_snapshot(proof, 'before'), legacy.read_snapshot(proof, 'after')
    if (before != after or after.get('source_manifest_sha256') != source_sha
            or Path(after['parent_root']).resolve() != parent.resolve()
            or Path(after['completion_root']).resolve() != completed.resolve()):
        raise ValueError('Range terminal source/raw/state snapshots changed')
    for name, root in (('parent', parent), ('completion', completed)):
        if not after[name].get('raw_files') or after[name] != capture_snapshot(root):
            raise ValueError('Range terminal current raw/events/selection/elapsed differs')
        legacy.verify_file_snapshot(after[name], root)
    state_path = str((completed / 'state.json').resolve())
    if after.get('state_sha256', {}).get(state_path) != sha256(completed / 'state.json'):
        raise ValueError('Range terminal snapshot state checksum changed')
    required = {'ledger_sha256': {parent / 'selected_ledger.jsonl', completed / 'selected_ledger.jsonl'},
                'state_sha256': {completed / 'state.json'},
                'status_sha256': {parent / 'status.json', completed / 'status.json'}}
    for name, paths in required.items():
        values = proof.get(name, {})
        if not {value.resolve() for value in paths} <= {Path(value).resolve() for value in values}:
            raise ValueError('Range terminal ledger/state/status pins incomplete')
        pins.verify_hashes(values, 'Range terminal ledger/state/status')
    return proof


def freeze(a):
    if a.output.exists():
        raise FileExistsError('New v152 release output required')
    range_completion = getattr(a, 'range_completion', None)
    range_completion_proof = getattr(a, 'range_completion_proof', None)
    if bool(range_completion) != bool(range_completion_proof):
        raise ValueError('Range completion and its terminal replay proof must be supplied together')
    manifest, registration = a.manifest.resolve(), master_path(a.registration).resolve()
    deck = read(manifest)
    entries, metadata, planned = qualification.validate_source(deck)
    audit_path = manifest.parent / 'scenario_audit.json'
    audit, preservation = read(audit_path), read(a.preservation_proof)
    if (audit.get('status') != 'OFFLINE_PASS' or audit.get('scenes') != 5000 or audit.get('errors')
            or preservation.get('schema') != 'base5_primary_obstacle_range_preservation_v1'
            or preservation.get('passed') is not True or preservation.get('errors')
            or preservation.get('candidate_manifest_sha256') != sha256(manifest) or preservation.get('frames') != 5000
            or preservation.get('declared_plan_count') != len(planned)
            or any(not isinstance(preservation.get(name + '_before'), str)
                   or not pins.HEX.fullmatch(preservation[name + '_before'])
                   for name in ('actor_identity_digest', 'front_allocation_digest'))
            or preservation.get('actor_identity_digest_before') != preservation.get('actor_identity_digest_after')
            or preservation.get('front_allocation_digest_before') != preservation.get('front_allocation_digest_after')
            or preservation.get('front_selected_count') != 338):
        raise ValueError('Full v152 range/front/population preservation evidence failed')
    runtime_hashes, input_hashes = pins.collect_runtime(), pins.collect_inputs(manifest)
    registered, front_reg, static_manifest, moving_manifest = verify_registration(
        registration, manifest, deck, entries, metadata, planned)
    static_rows, static_status = legacy.verify_campaign(a.static, static_manifest, 64)
    moving_rows, moving_status = legacy.verify_campaign(a.moving, moving_manifest, 8)
    if len(moving_rows) != 8:
        raise ValueError('Current runtime moving regression must approve all eight')
    ctx = legacy.completion.spatial.gen.context(legacy.completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                               legacy.completion.spatial.gen.geo.DEFAULT_MGEO)
    pools = legacy.completion.pool_map(ctx)
    rows, state, real_completion = legacy.verify_completion(a.completion, static_manifest,
        read(static_manifest), static_rows, pools)
    unresolved = sorted({row['id'] for row in read(static_manifest)['scenes']} - {row['source_cell_id'] for row in rows})
    front_rows = legacy.verify_front_rows(front_reg, rows, metadata, allowed_missing=unresolved)
    if legacy.verify_repaired_cohort(front_reg, rows, manifest) != 24:
        raise ValueError('All24 preserved NPCs need current native stationary evidence')
    range_manifest = Path(registered['manifests']['range_static']['path'])
    range_parent_rows, range_status = legacy.verify_campaign(a.range_static, range_manifest, len(planned))
    range_selected, range_state, range_supplement_status, range_terminal = range_parent_rows, None, None, None
    if range_completion is not None:
        range_selected, range_state, range_supplement_status = verify_range_completion(
            range_completion, a.range_static, range_manifest, read(range_manifest), range_parent_rows)
        range_terminal = verify_range_completion_resume(range_completion_proof, a.range_static,
                                                         range_completion, range_manifest)
    range_rows, range_missing = verify_range_rows(registered, range_selected, metadata)
    probe_proof, probe_path = verify_probe_proof(a.range_probes, manifest, registration, deck, metadata, pools)
    resume, partial = verify_resume(a.resume_proof, a.partial_resume, a.static, a.completion, static_manifest)
    orchestration = verify_orchestration(a.orchestration_proof, manifest, runtime_hashes)
    if sha256(legacy.CANONICAL) != legacy.CANONICAL_SHA256:
        raise ValueError('Completed original Sunny13 canonical selection changed')
    status = read(a.completion / 'status.json')
    basis, blockers = qualification_basis(real_completion, status['replenished'], unresolved, range_missing, len(front_rows))
    cost = legacy.cost_report(manifest, [a.static, a.moving], detailed_groups=True)
    cost['additional_range_qualification'] = dict(root=str(a.range_static.resolve()), requested=len(planned),
        initially_accepted=len(range_parent_rows), accepted=len(range_rows), elapsed_sec=range_status['elapsed_sec'],
        unique_storage=legacy.unique_storage(a.range_static), deliberately_targeted_sample=True)
    if range_supplement_status is not None:
        cost['additional_range_qualification']['finite_completion'] = dict(
            root=str(range_completion.resolve()), actual_initial_deficit=len(range_state['deficits']),
            native_attempts=len(range_state['attempts']), replenished=range_supplement_status['replenished'],
            remaining_deficit=len(range_missing), additional_elapsed_sec=range_supplement_status['replenishment_elapsed_sec'],
            total_parent_and_completion_elapsed_sec=range_supplement_status['elapsed_sec'],
            unique_storage=legacy.unique_storage(range_completion), additional_terminal_native_captures=0)
    evidence = {key: str(getattr(a, key).resolve()) for key in
        ('registration', 'static', 'moving', 'completion', 'range_static', 'range_probes',
         'orchestration_proof', 'resume_proof', 'partial_resume', 'preservation_proof')}
    proof_paths = [registration, Path(registered['front_registration']['path']), static_manifest, moving_manifest,
        range_manifest, audit_path, a.preservation_proof, a.resume_proof, a.partial_resume,
        a.orchestration_proof, Path(orchestration['test_log']['path']), probe_path,
        Path(probe_proof['state_path']), legacy.CANONICAL]
    proof_paths.extend(Path(doc[name + '_snapshot_path']) for doc in (resume, partial) for name in ('before', 'after'))
    evidence_roots = [a.static, a.moving, a.completion, a.range_static, probe_path.parent]
    if range_terminal is not None:
        evidence.update(range_completion=str(range_completion.resolve()),
                        range_completion_proof=str(range_completion_proof.resolve()))
        proof_paths.extend([range_completion_proof, range_completion / 'state.json'])
        proof_paths.extend(Path(range_terminal[name + '_snapshot_path']) for name in ('before', 'after'))
        evidence_roots.append(range_completion)
    input_hashes.update(legacy.evidence_pins(proof_paths,
        evidence_roots))
    input_hashes.update(orchestration.get('test_files', {}))
    pins.verify_hashes(runtime_hashes, 'Final v152 runtime')
    pins.verify_hashes(input_hashes, 'Final v152 inputs/evidence')
    doc = dict(schema='base5_obstacle_support_release_v1', generated_at_utc=datetime.now(timezone.utc).isoformat(),
        release_policy=primary.POLICY, policy=deck['config']['obstacle_support_policy'],
        front_policy=deck['config']['obstacle_front_support_policy'], primary_range_policy=deck['config'][primary.CONFIG_KEY],
        parked_npc_policy=deck['config']['parked_npc_placement'], qualified_environments=['sunny13'],
        bulk_ready=not blockers, bulk_blockers=blockers, completion_live_qualified=not blockers,
        qualification_basis=basis, actual_deficit_replenished=status['replenished'],
        runtime_hashes=runtime_hashes, input_hashes=input_hashes,
        environments={'sunny13': dict(manifest=str(manifest), manifest_sha256=sha256(manifest),
            capture_output=str((a.output.parent / 'collection_5k_support_v152_sunny13').resolve()))},
        evidence=evidence, initial_static_accepted=static_status['accepted_candidate'],
        initial_moving_accepted=moving_status['accepted_candidate'], completed_static_accepted=len(rows),
        native_front_strata_accepted=len(front_rows), native_front_strata_expected=38,
        primary_range_planned=len(planned), primary_range_initial_accepted=len(range_parent_rows),
        primary_range_accepted=len(range_rows), primary_range_initial_deficit=len(planned) - len(range_parent_rows),
        primary_range_replenished=range_supplement_status['replenished'] if range_supplement_status else 0,
        primary_range_remaining_deficit=len(range_missing),
        primary_range_native=range_rows, unresolved_slots=unresolved, unresolved_range_slots=range_missing,
        original_sunny13_ledger_sha256=legacy.CANONICAL_SHA256,
        estimated_5k_hours=cost['extrapolated_hours_group_weighted'],
        estimated_5k_gib=cost['extrapolated_gib_including_qa_and_rejected_group_weighted'],
        operational_hours_range=[8, 12], operational_storage_budget_gib=45.,
        operational_budget_basis='Historical conservative budget; current64+8 cost estimate is a targeted small-sample extrapolation.',
        bulk_started=False, limitation='Sunny13 bounded native/source qualification only; no new5K collection, other weather, AP or full ground-contact certification.')
    atomic_json(a.output / 'cost_estimate.json', cost)
    atomic_json(a.output / 'release.json', doc)
    return a.output / 'release.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'registration', 'static', 'moving', 'completion', 'range-static', 'range-probes',
                 'orchestration-proof', 'resume-proof', 'partial-resume', 'preservation-proof', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--range-completion', type=Path)
    parser.add_argument('--range-completion-proof', type=Path)
    print(freeze(parser.parse_args()))
