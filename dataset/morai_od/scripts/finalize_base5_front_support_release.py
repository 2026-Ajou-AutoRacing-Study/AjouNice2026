#!/usr/bin/env python3
"""Freeze new v151 native evidence without starting a 5K collection.

An otherwise valid zero-deficit pilot is preserved as a blocked release: it
does not fabricate a failed parent in order to claim completion qualification.
Old v150 evidence, manifests, raw files and source snapshots are never edited.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import base5_front_completion_adapter as adapter
import base5_front_release_inputs as inputs
import base5_obstacle_front_support as front
import prepare_base5_front_support_qualification as registration_tool
import validate_base5_front_recovery_live as recovery
from base5_capture_safety import atomic_json, load_ledger, sha256
from finalize_base5_obstacle_support_release import verify as verify_native
from finalize_base5_parked_npc_release import verify_campaign, verify_repaired_cohort
from report_base5_prebulk_cost import report as cost_report, unique_storage
import run_base5_fresh_completion as completion

CANONICAL = (Path.home() / 'morai_cap/base5_v2_261008/final_appearance_5k_v132_release'
             / 'collection_5k_sunny13_flexible_v140/selected_ledger.jsonl')
CANONICAL_SHA256 = '026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43'


def read(path):
    return json.loads(Path(path).read_text())


def registration_path(path):
    path = Path(path)
    return path / 'registration.json' if path.is_dir() else path


def verify_registration(path, manifest, entries, metadata):
    path = registration_path(path)
    doc = read(path)
    if (doc.get('policy') != registration_tool.POLICY
            or doc.get('source_manifest_sha256') != sha256(manifest)
            or doc.get('front_helper_sha256') != front.implementation_sha256()
            or len(doc.get('static', [])) != 64 or len(doc.get('animated', [])) != 8):
        raise ValueError('Current source static64/moving8 registration required')
    groups = {'static64': doc['static'], 'animated8': doc['animated']}
    ids = [row['id'] for rows in groups.values() for row in rows]
    if len(set(ids)) != 72 or any(ident not in entries for ident in ids):
        raise ValueError('Registration scene identities changed')
    strata = set()
    npc = set()
    for name, rows in groups.items():
        subset_path = path.parent / name / 'manifest.json'
        subset = read(subset_path)
        if subset.get('count') != len(rows) or subset.get('scenes') != [entries[row['id']] for row in rows]:
            raise ValueError('Registered pilot differs from full source payloads')
        for row in rows:
            ident = row['id']
            if (row['sha256'] != entries[ident]['scenario_sha256']
                    or row['metadata_sha256'] != sha256(entries[ident]['metadata'])
                    or bool(registration_tool.moving_actors(metadata[ident])) != (name == 'animated8')):
                raise ValueError('Registered payload, metadata or motion changed')
            if name != 'static64':
                continue
            role = row['qualification_role']
            if role['kind'] in ('front_selected', 'front_legacy'):
                plan = front.validate_plan(metadata[ident]['spec'])
                if (not plan or role.get('data_id') != plan['data_id']
                        or role.get('selected') != plan['selected']
                        or (role['kind'] == 'front_selected') != plan['selected']):
                    raise ValueError('Registered asset front stratum changed')
                key = (plan['data_id'], plan['selected'])
                if key in strata:
                    raise ValueError('Duplicate registered asset front stratum')
                strata.add(key)
            elif role['kind'] == 'parked_npc_repaired_regression':
                npc.add(ident)
            elif role['kind'] not in ('historical_deficit_negative_regression', 'fixed_static_regression_control'):
                raise ValueError('Unknown static qualification role')
    assets = {asset for asset, _ in strata}
    if (len(assets) != 19 or strata != {(asset, selected) for asset in assets for selected in (False, True)}
            or len(npc) != 24 or npc != set(doc.get('all_changed_slots', []))):
        raise ValueError('Full selected/legacy19 and NPC24 coverage required')
    inputs.verify_hashes(doc['input_sha256'], 'Qualification registration input')
    return doc, path.parent / 'static64/manifest.json', path.parent / 'animated8/manifest.json'


def verify_completion(root, manifest, deck, static_rows, pools):
    status = verify_native(root, 64)
    state = read(root / 'state.json')
    expected_deficits = {row['id'] for row in deck['scenes']} - {row['source_cell_id'] for row in static_rows}
    if (state.get('stage') not in completion.TERMINAL or status.get('stage') != state.get('stage')
            or state.get('baseline_selected') != static_rows
            or set(state.get('deficits', [])) != expected_deficits):
        raise ValueError('Actual terminal parent/completion identity differs')
    contract = state.get('contract', {})
    expected_capture = completion.adaptive.fa.batch.campaign.contract(manifest, list(range(64)), 2)
    if (contract.get('source_sha256') != sha256(manifest)
            or contract.get('capture_contract') != expected_capture):
        raise ValueError('Completion was verified under another source/runtime contract')
    code_hashes = {str(Path(__file__).with_name(name).resolve()): digest
                   for name, digest in contract.get('code_hashes', {}).items()}
    inputs.verify_hashes(code_hashes, 'Completion source runtime')
    lookup = {row['id']: row for row in deck['scenes']}
    for attempt in state.get('attempts', []):
        if not attempt.get('result') or attempt['id'] not in expected_deficits:
            raise ValueError('Completion has unresolved or foreign attempt')
        original = read(lookup[attempt['id']]['metadata'])
        observed = completion.verify(attempt, deck=deck, source=original, pools=pools)
        if observed != attempt['result']:
            raise ValueError('Official native completion verification changed')
    rows, missing = completion.merged(state)
    expected_missing = set(lookup) - {row['source_cell_id'] for row in rows}
    if (set(missing) != expected_missing or status.get('accepted_candidate') != len(rows)
            or status.get('review') != len(missing) or status.get('remaining_deficit') != len(missing)
            or (state['stage'] == 'complete') != (not missing)
            or rows != load_ledger(root / 'selected_ledger.jsonl')
            or any(row['status'] != 'accepted_candidate' or row.get('audit', {}).get('review_reasons') for row in rows)):
        raise ValueError('Final completion approvals/explicit remaining deficits differ')
    replenished = len(rows) - len(static_rows)
    if status.get('replenished') != replenished:
        raise ValueError('Completion replenishment count changed')
    return rows, state, bool(expected_deficits and replenished >= 1)


def verify_front_rows(registration, rows, metadata, allowed_missing=()):
    by_id = {row['source_cell_id']: row for row in rows}
    checked = []
    for registered in registration['static']:
        if registered['qualification_role']['kind'] not in ('front_selected', 'front_legacy'):
            continue
        ident = registered['id']
        if ident not in by_id:
            if ident in allowed_missing:
                continue
            raise ValueError('Registered front stratum lacks native selection')
        original = metadata[ident]
        actual = read(Path(by_id[ident]['capture_root']) / 'metadata.json')
        plan = adapter.verify_preserved(original, actual)
        if plan is None:
            raise ValueError('Captured front stratum lost its frozen plan')
        checked.append(dict(id=ident, data_id=plan['data_id'], selected=plan['selected'],
                            frozen_spec_actor_ids_and_band_preserved=True))
    registered_missing = {row['id'] for row in registration['static']
                          if row['qualification_role']['kind'] in ('front_selected', 'front_legacy')
                          and row['id'] in allowed_missing}
    if len(checked) + len(registered_missing) != 38:
        raise ValueError('Every selected and legacy asset stratum needs native proof')
    return checked


def verify_probe_proof(path, manifest, deck, metadata, pools):
    path = Path(path)
    path = path / 'proof.json' if path.is_dir() else path
    proof = read(path)
    if (proof.get('schema') != 'base5_front_recovery_probe_v1' or proof.get('passed') is not True
            or proof.get('source_manifest_sha256') != sha256(manifest)
            or Path(proof['source_manifest']).resolve() != manifest.resolve()
            or proof.get('prepared_count') != 4 or proof.get('accepted_count') != 4
            or proof.get('planned_probe') is not True or proof.get('fabricated_parent_deficit') is not False
            or proof.get('training_data') is not False or proof.get('bulk_started') is not False
            or proof.get('additional_completed_resume_raw') != 0):
        raise ValueError('Four real frozen-front native recoveries and terminal resume required')
    state_path = Path(proof['state_path'])
    if state_path.resolve() != (path.parent / 'state.json').resolve() or sha256(state_path) != proof['state_sha256']:
        raise ValueError('Front recovery state checksum changed')
    state = read(state_path)
    if (state.get('stage') != 'complete' or len(state.get('attempts', [])) != 4
            or state['contract']['source_manifest_sha256'] != sha256(manifest)
            or state['contract'].get('strata') != proof.get('strata')):
        raise ValueError('Front recovery state/contract changed')
    snapshot = recovery.source_snapshot(manifest)
    if (snapshot[0] != deck or state['contract'] != recovery.contract(manifest, *snapshot)
            or proof.get('source_inventory_sha256') != snapshot[3]
            or proof.get('full_deck_plan_counts') != snapshot[4]):
        raise ValueError('Front recovery full source/runtime/preparation contract changed')
    runtime_pins = {}
    for name, digest in state['contract']['runtime_hashes'].items():
        if Path(name).name != name:
            raise ValueError('Front recovery runtime pin must be a local basename')
        runtime_pins[str(Path(__file__).with_name(name).resolve())] = digest
    inputs.verify_hashes(runtime_pins, 'Front recovery runtime')
    recovery.verify_prepared(path.parent, state, deck, metadata)
    native_inventory = recovery.raw_inventory(state)
    if (native_inventory != state.get('terminal_raw_inventory')
            or native_inventory['captured_rows'] != 4
            or native_inventory['ledger_sha256map'] != proof.get('ledger_sha256map')):
        raise ValueError('Front recovery terminal raw/ledger inventory changed')
    strata = proof.get('strata', [])
    if (len(strata) != 4 or len({row['source_id'] for row in strata}) != 4
            or {(row['selected'], row['large_footprint']) for row in strata}
            != {(selected, large) for selected in (False, True) for large in (False, True)}):
        raise ValueError('Front recovery selected/legacy and small/large strata incomplete')
    source = {row['id']: row for row in deck['scenes']}
    expected = {row['source_id']: row for row in strata}
    seen = set()
    for attempt in state['attempts']:
        ident = attempt['source_id']
        if (ident in seen or ident not in expected or attempt.get('id') != ident
                or attempt['stratum'] != expected[ident]):
            raise ValueError('Front recovery attempt stratum changed')
        seen.add(ident)
        stratum = expected[ident]
        original = metadata[ident]
        plan = front.validate_plan(original['spec'])
        if (plan is None or plan['data_id'] != stratum['data_id']
                or plan['selected'] != stratum['selected'] or plan['large_footprint'] != stratum['large_footprint']
                or source[ident]['scenario_sha256'] != stratum['source_scenario_sha256']
                or sha256(source[ident]['metadata']) != stratum['source_metadata_sha256']):
            raise ValueError('Front recovery source stratum/hash differs')
        observed = recovery.verify_attempt(attempt, deck, original, pools)
        if observed != attempt.get('result') or observed.get('status') != 'accepted_candidate':
            raise ValueError('Front recovery is not a current official native approval')
        adapter.verify_preserved(original, read(Path(observed['capture_root']) / 'metadata.json'))
    if seen != set(expected):
        raise ValueError('Front recovery four source IDs differ')
    inputs.verify_hashes(proof['ledger_sha256map'], 'Front recovery terminal ledger')
    if recovery.raw_inventory(state) != native_inventory:
        raise ValueError('Front recovery raw changed during official verification')
    return proof, path


def read_snapshot(proof, which):
    path = Path(proof[which + '_snapshot_path'])
    if not path.is_absolute() or sha256(path) != proof[which + '_snapshot_sha256']:
        raise ValueError('Resume snapshot checksum/path changed')
    return read(path)


def verify_file_snapshot(snapshot, root):
    """Recheck relative owned raw paths, digests and file identity metadata."""
    root = root.resolve()
    raw = snapshot.get('raw_files', {})
    if not raw:
        raise ValueError('Resume snapshot lacks owned raw evidence')
    for relative, expected in raw.items():
        path = (root / relative).resolve()
        if Path(relative).is_absolute() or root not in path.parents or not path.is_file():
            raise ValueError('Resume raw path escaped its parent or disappeared')
        stat = path.stat()
        actual = dict(sha256=sha256(path), inode=stat.st_ino, mtime_ns=stat.st_mtime_ns, size=stat.st_size)
        if actual != expected:
            raise ValueError('Resume raw content or file identity changed')
    for relative, digest in snapshot.get('event_logs', {}).items():
        path = (root / relative).resolve()
        if Path(relative).is_absolute() or root not in path.parents or sha256(path) != digest:
            raise ValueError('Resume event evidence changed')
    if snapshot.get('selected_sha256') != sha256(root / 'selected_ledger.jsonl'):
        raise ValueError('Resume snapshot does not match current selected ledger')


def verify_resume(resume_path, partial_path, static, completed, manifest):
    proof, partial = read(resume_path), read(partial_path)
    if proof.get('passed') is not True or proof.get('additional_native_captures') != 0:
        raise ValueError('Terminal resume must add no raw captures')
    pins = proof.get('ledger_sha256', {})
    expected = {(Path(root) / 'selected_ledger.jsonl').resolve() for root in (static, completed)}
    if not expected <= {Path(path).resolve() for path in pins}:
        raise ValueError('Terminal resume must cover both parent and completion ledgers')
    inputs.verify_hashes(pins, 'Terminal resume ledger')
    if (partial.get('passed') is not True or partial.get('preserved_selected') != 16
            or partial.get('prior_raw_unchanged') is not True):
        raise ValueError('Partial resume must preserve the first16 successful raw selections')
    source_sha = sha256(manifest)
    if (Path(partial['parent_root']).resolve() != static.resolve()
            or partial.get('source_manifest_sha256') != source_sha):
        raise ValueError('Partial resume belongs to another parent/manifest')
    before, after = read_snapshot(partial, 'before'), read_snapshot(partial, 'after')
    if (Path(after['parent_root']).resolve() != static.resolve()
            or after.get('source_manifest_sha256') != source_sha):
        raise ValueError('Partial after snapshot parent/manifest differs')
    before_rows, after_rows = before['selected_rows'], after['selected_rows']
    by_id = {row['source_cell_id']: row for row in after_rows}
    if (len(before_rows) != 16 or len(by_id) != len(after_rows)
            or len({row['source_cell_id'] for row in before_rows}) != 16
            or any(by_id.get(row['source_cell_id']) != row for row in before_rows)
            or after_rows != load_ledger(static / 'selected_ledger.jsonl')):
        raise ValueError('Partial resume selected16 are not preserved in final parent')
    for name in ('raw_files', 'event_logs'):
        if any(after['snapshot'].get(name, {}).get(key) != value
               for key, value in before['snapshot'].get(name, {}).items()):
            raise ValueError('Partial resume changed earlier raw/event evidence')
    verify_file_snapshot(after['snapshot'], static)
    terminal_before, terminal_after = read_snapshot(proof, 'before'), read_snapshot(proof, 'after')
    if (terminal_before != terminal_after
            or terminal_after.get('source_manifest_sha256') != source_sha
            or Path(terminal_after['parent_root']).resolve() != static.resolve()
            or Path(terminal_after['completion_root']).resolve() != completed.resolve()):
        raise ValueError('Terminal resume snapshots or parent/manifest differ')
    for key, root in (('parent', static), ('completion', completed)):
        verify_file_snapshot(terminal_after[key], root)
    required = {'state_sha256': {completed / 'state.json'},
                'status_sha256': {static / 'status.json', completed / 'status.json'}}
    for name, expected_paths in required.items():
        values = proof.get(name, {})
        if not {p.resolve() for p in expected_paths} <= {Path(path).resolve() for path in values}:
            raise ValueError('Terminal resume state/status evidence incomplete')
        inputs.verify_hashes(values, 'Terminal resume state/status')
    return proof, partial


def evidence_pins(paths, roots):
    """Pin immutable evidence, excluding process locks and mutable text logs."""
    result = {str(Path(path).resolve()): sha256(path) for path in paths}
    for root in roots:
        for path in Path(root).rglob('*'):
            if path.is_file() and path.suffix in ('.json', '.jsonl', '.txt', '.bin', '.png', '.pth'):
                result[str(path.resolve())] = sha256(path)
    return result


def freeze(a):
    if a.output.exists():
        raise FileExistsError('New v151 release output required')
    manifest = a.manifest.resolve()
    deck = read(manifest)
    entries, metadata, _ = registration_tool.validate_source(deck)
    scenario_audit = manifest.parent / 'scenario_audit.json'
    audit = read(scenario_audit)
    if audit.get('status') != 'OFFLINE_PASS' or audit.get('scenes') != 5000:
        raise ValueError('Final 5K scenario offline audit failed')
    preservation = read(a.preservation_proof)
    if (preservation.get('passed') is not True or preservation.get('errors')
            or preservation.get('candidate_manifest_sha256') != sha256(manifest)
            or preservation.get('frames') != 5000):
        raise ValueError('Final v151 full-deck preservation audit failed')
    runtime_hashes, input_hashes = inputs.collect_runtime(), inputs.collect_inputs(manifest)
    registration, static_manifest, moving_manifest = verify_registration(a.registration, manifest, entries, metadata)
    static_rows, static_status = verify_campaign(a.static, static_manifest, 64)
    moving_rows, moving_status = verify_campaign(a.moving, moving_manifest, 8)
    if len(moving_rows) != 8:
        raise ValueError('New runtime moving regression must approve all eight')
    context = completion.spatial.gen.context(completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                             completion.spatial.gen.geo.DEFAULT_MGEO)
    pools = completion.pool_map(context)
    rows, state, real_completion = verify_completion(a.completion, static_manifest, read(static_manifest), static_rows, pools)
    unresolved = sorted({entry['id'] for entry in read(static_manifest)['scenes']}
                        - {row['source_cell_id'] for row in rows})
    front_evidence = verify_front_rows(registration, rows, metadata, allowed_missing=unresolved)
    repaired = verify_repaired_cohort(registration, rows, manifest)
    if repaired != 24:
        raise ValueError('All24 original repaired NPCs require current stationary native proof')
    probe_proof, probe_path = verify_probe_proof(a.front_probes, manifest, deck, metadata, pools)
    resume_proof, partial_proof = verify_resume(a.resume_proof, a.partial_resume, a.static, a.completion, static_manifest)
    if sha256(CANONICAL) != CANONICAL_SHA256:
        raise ValueError('Completed original Sunny13 selection changed')
    cost = cost_report(manifest, [a.static, a.moving], detailed_groups=True)
    status = read(a.completion / 'status.json')
    cost['finite_completion_evidence'] = dict(root=str(a.completion.resolve()),
        additional_elapsed_sec=status.get('replenishment_elapsed_sec'), unique_storage=unique_storage(a.completion),
        actual_parent_deficit=len(state['deficits']), native_replenished=status['replenished'])
    registration_file = registration_path(a.registration)
    evidence = dict(registration=str(registration_file.resolve()), static=str(a.static.resolve()),
        moving=str(a.moving.resolve()), completion=str(a.completion.resolve()),
        front_probes=str(probe_path.resolve()), resume_proof=str(a.resume_proof.resolve()),
        partial_resume=str(a.partial_resume.resolve()), preservation_proof=str(a.preservation_proof.resolve()))
    proof_paths = [registration_file, static_manifest, moving_manifest, scenario_audit, a.resume_proof,
                   a.partial_resume, a.preservation_proof, probe_path, Path(probe_proof['state_path']), CANONICAL]
    proof_paths.extend(Path(doc[name + '_snapshot_path']) for doc in (resume_proof, partial_proof)
                       for name in ('before', 'after'))
    pins = evidence_pins(proof_paths, [a.static, a.moving, a.completion, probe_path.parent])
    input_hashes.update(pins)
    inputs.verify_hashes(runtime_hashes, 'Final release runtime')
    inputs.verify_hashes(input_hashes, 'Final release input/evidence')
    blockers = [] if real_completion else [
        'Static64 has no actual missing parent slot: finite family completion remains unqualified; no deficit was fabricated.']
    if unresolved:
        blockers.append('Finite native completion retained %d unresolved slot(s): %s; no gate was relaxed.'
                        % (len(unresolved), ', '.join(unresolved)))
    if len(front_evidence) != 38:
        blockers.append('Native front asset strata approved %d/38; missing strata remain explicit.' % len(front_evidence))
    doc = dict(schema='base5_obstacle_support_release_v1', generated_at_utc=datetime.now(timezone.utc).isoformat(),
        policy=deck['config']['obstacle_support_policy'], front_policy=deck['config']['obstacle_front_support_policy'],
        parked_npc_policy=deck['config']['parked_npc_placement'], qualified_environments=['sunny13'],
        completion_live_qualified=real_completion, front_recovery_live_qualified=True,
        bulk_ready=not blockers, bulk_blockers=blockers, runtime_hashes=runtime_hashes, input_hashes=input_hashes,
        environments={'sunny13': dict(manifest=str(manifest), manifest_sha256=sha256(manifest),
            capture_output=str((a.output.parent / 'collection_5k_support_v151_sunny13').resolve()))},
        evidence=evidence, evidence_ledger_sha256={str(root / 'selected_ledger.jsonl'): sha256(root / 'selected_ledger.jsonl')
            for root in (a.static, a.moving, a.completion)},
        initial_static_accepted=static_status['accepted_candidate'], initial_moving_accepted=moving_status['accepted_candidate'],
        completed_static_accepted=len(rows), unresolved_slots=unresolved,
        unresolved_static_count=len(unresolved), native_front_strata_accepted=len(front_evidence),
        native_front_strata_expected=38, repaired_slots_natively_approved=repaired, front_strata_native=front_evidence,
        original_sunny13_ledger_sha256=CANONICAL_SHA256,
        estimated_5k_hours=cost['extrapolated_hours_group_weighted'],
        estimated_5k_gib=cost['extrapolated_gib_including_qa_and_rejected_group_weighted'],
        operational_hours_range=[8, 12], operational_storage_budget_gib=45.,
        operational_budget_basis='Historical v150 conservative budget, not a measured v151 full-5K guarantee; new64+8 estimate stored separately.',
        bulk_started=False,
        limitation='Sunny13 small live qualification only. Four deliberate fresh-recovery probes are not fabricated parent deficits or training data. Other weather, all meshes, model AP and full-5K success are not certified.')
    atomic_json(a.output / 'cost_estimate.json', cost)
    atomic_json(a.output / 'release.json', doc)
    return a.output / 'release.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'registration', 'static', 'moving', 'completion', 'front-probes',
                 'resume-proof', 'partial-resume', 'preservation-proof', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    print(freeze(parser.parse_args()))
