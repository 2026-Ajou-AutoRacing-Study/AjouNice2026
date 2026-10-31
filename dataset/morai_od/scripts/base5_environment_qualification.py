#!/usr/bin/env python3
"""새 환경의 실제 native 대체를 full5K CPU source와 후속 검증 계획으로 묶는다.

과거 원본/ledger/audit/registration을 수정하거나 review를 승인으로 바꾸지
않는다. 본수집 전이므로 가짜 5K selected ledger도 만들지 않는다. 결과는
항상 bulk_ready=false이며 새 exact native/input/resume 근거가 필요하다.
"""
import argparse
from collections import Counter
import copy
import json
import math
from pathlib import Path
import re
import shutil
import time

import numpy as np

import audit_base5_scenarios as scenario_audit
import base5_close_primary_observation as close
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_shadow_ground_reuse as shadow
import base5_unit_capture_provenance as provenance
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, collision_window, sha256
from finalize_base5_layout_split import planned_fields_match
import prepare_base5_layout_split as split_prepare
import prepare_base5_unit_environment_smoke as environment_smoke
import prepare_base5_unit_only_stability as raw_unit
from base5_scale_capture_contract import validate_entry
import base5_motion_ground as ground
from report_base5_compound_evidence import _validate_native_geometry, point_memberships
import run_base5_capture_static_attitude as attitude


SCHEMA = 'base5_new_environment_cpu_adoption_v1'
POLICY = 'native-bound-replacement-full-source-assembly-v1'
REGISTRATION_SCHEMA = 'base5_assembled_environment_exact_smoke_v1'
REGISTRATION_POLICY = 'native-adopted-new-environment-exact-smoke-v1'
ENVIRONMENTS = ('sunny11', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
TOKEN = re.compile(r'^[A-Za-z0-9_.-]+$')


def runtime_hashes():
    return pins.collect_runtime(entrypoints=(Path(__file__).name, 'run_base5_capture_static_attitude.py'))


def read(path):
    return json.loads(Path(path).read_text())


def require(value, message):
    if not value:
        raise ValueError(message)


def rows(path):
    payload = Path(path).read_bytes()
    require(payload and payload.endswith(b'\n'), 'Complete immutable ledger required')
    values = [json.loads(line) for line in payload.splitlines() if line.strip()]
    require(values and len({r['id'] for r in values}) == len(values), 'Unique nonempty ledger IDs required')
    return values


def add_pin(result, path, expected=None):
    path = Path(path)
    require(not path.is_symlink(), 'Regular immutable input file required')
    path = path.resolve(strict=True)
    require(path.is_file(), 'Regular immutable input file required')
    digest = sha256(path)
    require(expected is None or expected == digest, 'Declared input changed: ' + str(path))
    require(str(path) not in result or result[str(path)] == digest, 'Conflicting input pin')
    result[str(path)] = digest
    return path


def raw_pins(root):
    root = Path(root).resolve(strict=True)
    result = {}
    for path in root.rglob('*'):
        require(not path.is_symlink(), 'Native raw symlink is not immutable evidence')
        if path.is_file():
            stat = path.stat()
            result[str(path.resolve())] = dict(sha256=sha256(path), inode=stat.st_ino,
                mtime_ns=stat.st_mtime_ns, size=stat.st_size)
    require(result, 'Native evidence directory is empty')
    return result


def native_session_files(session):
    session = Path(session).resolve(strict=True)
    files = {}
    for path in sorted(session.iterdir()):
        if path.is_file():
            require(not path.is_symlink(), 'Native session symlink is not immutable evidence')
            stat = path.stat()
            files[str(path.resolve())] = dict(sha256=sha256(path), inode=stat.st_ino,
                mtime_ns=stat.st_mtime_ns, size=stat.st_size)
    require(files, 'Native session evidence is missing')
    return files


def native_observation_snapshot(root):
    root = Path(root).resolve(strict=True)
    return dict(raw=raw_pins(root), session_files=native_session_files(root.parents[1]))


def verify_preserved_native(receipt):
    for root, before in receipt.get('original_native_inventory', {}).items():
        require(raw_pins(root) == before, 'Original native inventory changed after assembly')
    for session, before in receipt.get('original_session_inventory', {}).items():
        require(native_session_files(session) == before, 'Original native session changed after assembly')
    for record in receipt.get('replacement_native', []):
        observed = native_observation_snapshot(record['capture_root'])
        require(observed['raw'] == record['original_native_evidence'],
            'Replacement native inventory changed after assembly')
        require(observed['session_files'] == record['session_inventory'],
            'Replacement native session changed after assembly')
        pins.verify_hashes(record['session_files'])


def historical_registration_contract(document, source, helper_snapshot=None):
    """당시 등록 source/runtime를 보존본으로 검증하며 현재 승인으로 재해석하지 않는다."""
    require(document.get('schema') == environment_smoke.SCHEMA
        and document.get('policy') == environment_smoke.POLICY
        and document.get('source_manifest') == str(Path(source).resolve())
        and document.get('source_manifest_sha256') == sha256(source),
        'Matching historical registration schema/policy/source required')
    stamp = document.get('registered_at_unix')
    require(type(stamp) in (int, float) and math.isfinite(stamp) and 0 < stamp <= time.time()
        and document.get('native_observations_must_follow_registration') is True
        and document.get('native_qa_unchanged') is True, 'Historical preregistration timestamp/gates invalid')
    old_hash = document.get('helper_sha256')
    helper = Path(environment_smoke.__file__).resolve();current_hash = sha256(helper)
    require(document.get('runtime_hashes', {}).get(str(helper)) == old_hash,
        'Historical helper SHA and registered runtime pin differ')
    historical_runtime = dict(document['runtime_hashes'])
    del historical_runtime[str(helper)]
    pins.verify_hashes(historical_runtime)
    evidence = helper
    if old_hash != current_hash:
        require(helper_snapshot is not None, 'Changed historical helper requires an explicit byte-exact preserved snapshot')
        evidence = Path(helper_snapshot).resolve(strict=True)
        require(evidence != helper and sha256(evidence) == old_hash, 'Historical helper snapshot differs from registered SHA')
    elif helper_snapshot is not None:
        evidence = Path(helper_snapshot).resolve(strict=True)
        require(sha256(evidence) == old_hash, 'Historical helper snapshot differs from registered SHA')
    historical_runtime[str(evidence)] = old_hash
    pins.verify_hashes(historical_runtime)
    return dict(registered_helper_sha256=old_hash, current_helper_sha256=current_hash,
        helper_differs_from_current=old_hash != current_hash, helper_evidence_path=str(evidence),
        historical_runtime_pins=historical_runtime, historical_registered_at_unix=stamp,
        historical_contract_verified=True, current_helper_replay_performed=False,
        historical_observations_promoted_to_current_approvals=False)


def protected_registration_roots(manifest, receipt):
    result = {Path(manifest).resolve().parent, Path(receipt['source_manifest']).resolve().parent}
    for root in receipt.get('original_native_inventory', {}):
        root = Path(root).resolve();result.update((root, root.parents[1]))
    for record in receipt.get('replacement_native', []):
        root = Path(record['capture_root']).resolve();result.update((root, root.parents[1]))
    for mapping in receipt.get('mapping', []):
        result.add(Path(mapping['replacement_manifest']).resolve().parent)
    return result


def require_safe_registration_output(output, manifest, receipt):
    output = Path(output).resolve()
    for protected in protected_registration_roots(manifest, receipt):
        require(output != protected and protected not in output.parents,
            'Registration output must be outside every preserved source/native/session directory')


def roster(meta):
    return [(a['unique_id'], a['data_id'], a['name'], a['source_list'], a['motion'],
        (a.get('asset_scale') or {}).get('profile_id')) for a in meta['runtime_actors']]


def replacement_shape(original, old, candidate, new, deck):
    """Native 확인과 별개인 canonical/spec/actor/recovery-overlay 계약이다."""
    canonical = original['id']
    require(TOKEN.fullmatch(candidate['id']) and candidate['id'] != canonical,
        'A distinct safe runtime candidate ID is required')
    require(old['spec'] == new['spec'] and old['split'] == new['split'] == original['split'] == candidate['split'],
        'Replacement changed full frozen spec or capture split')
    require(roster(old) == roster(new), 'Replacement changed ordered actor identity/motion/unit roster')
    require(old.get(unit.KEY) == new.get(unit.KEY) == deck['config'].get(unit.KEY), 'Unit reference changed')
    require(new.get('fresh_environment_provenance') == old.get('fresh_environment_provenance'),
        'Replacement changed fresh environment identity')
    declaration = new.get('fresh_completion_source', {})
    proof = new.get('quality_replacement_contract', {})
    require(declaration.get('source_cell_id') == canonical
        and declaration.get('source_scenario_sha256') == original['scenario_sha256']
        and type(declaration.get('ordinal')) is int and declaration['ordinal'] >= 1,
        'Replacement must name its exact original canonical source')
    require(proof.get('root_entry') == original and proof.get('ordinal') == declaration['ordinal']
        and proof.get('root_metadata_sha256') == sha256(original['metadata'])
        and proof.get('raw_gt_or_parent_modified') is False
        and proof.get('layout_split_finalization_pending') is True,
        'Replacement root/spec provenance differs')
    require(candidate['scenario_sha256'] != original['scenario_sha256'], 'Renamed old payload is not a replacement')
    overlay = layout.overlay_for_metadata(new)
    require(overlay is not None, 'Actual preregistered recovery overlay required')
    layout.verify_source_pins(overlay)
    binding = layout.resolve(overlay, candidate['id'], candidate, new)
    require(overlay.get('recovery_aliases', {}).get(candidate['id']) == canonical,
        'Recovery overlay alias names another canonical source')
    mapping = overlay['audit'].get('recovery_mapping', {})
    require(mapping.get('source_id') == canonical and mapping.get('candidate_id') == candidate['id']
        and mapping.get('old_scenario_sha256') == original['scenario_sha256']
        and mapping.get('new_scenario_sha256') == candidate['scenario_sha256'],
        'Recovery overlay payload mapping differs')
    return dict(canonical_id=canonical, runtime_candidate_id=candidate['id'],
        old_scenario_sha256=original['scenario_sha256'], new_scenario_sha256=candidate['scenario_sha256'],
        recovery_overlay_sha256=overlay['overlay_sha256'], prior_effective_split=binding['split'],
        prior_layout_group=binding['group_id'])


def verify_accepted_native(manifest, entry, deck, row):
    """원문 audit/GT/실제 접촉/Park와 실행 manifest를 모두 연결한다."""
    root = Path(row['capture_root']).resolve(strict=True)
    require(row.get('status') == 'accepted_candidate' and row['id'] == entry['id']
        and row.get('scenario_sha256') == entry['scenario_sha256'], 'Exact accepted replacement observation required')
    before = native_observation_snapshot(root)
    metadata, audit = read(root/'metadata.json'), read(root/'audit.json')
    require(row.get('audit') == audit and audit.get('accepted_candidate') is True
        and not audit.get('review_reasons'), 'Native review cannot become an approval through assembly')
    require(planned_fields_match(read(entry['metadata']), metadata), 'Native changed planned metadata fields')
    require(sha256(root/'scenario.json') == entry['scenario_sha256'] == sha256(entry['scenario']),
        'Native/planned scenario payload differs')
    provenance.verify_capture(entry, root, deck, row)
    components = audit['components']
    require(len({c['component_id'] for c in components}) == len(components), 'Duplicate native component ID')
    _validate_native_geometry(root, components)
    paths = list(root.glob('LIDAR_*/*_Intensity.bin'))
    require(len(paths) == 1, 'One synchronous native XYZI file required')
    points = np.fromfile(paths[0], dtype=np.float32)
    require(points.size > 0 and points.size % 4 == 0 and np.isfinite(points).all(), 'Invalid native XYZI')
    points = points.reshape(-1, 4);points = points[(points[:, :3] != 0).any(axis=1)]
    for component, mask in zip(components, point_memberships(points, components)):
        require(int(mask.sum()) == component['lidar_points'], 'Native full-RPY point support differs')
    evidence = read(root/'collision_history.json');summary = evidence['summary']
    actual = collision_window(evidence['samples'], summary['window_start_monotonic'], summary['window_end_monotonic'])
    require(summary.get('observed') is True and summary.get('no_collision_verified') is True
        and summary.get('non_ego_uids') == [] and actual['observed'] and actual['no_collision_verified'],
        'Native replacement collision evidence is not clear')
    session = root.parents[1]
    require(read(session/'shutdown_safety.json').get('park_verified') is True, 'Native final Park is unverified')
    contract = read(session/'session_contract.json')
    require(contract['manifest_sha256'] == sha256(manifest)
        and contract.get(unit.KEY) == deck['config'][unit.KEY]
        and contract.get('sensor_sha256') == sha256(root/'sensor_reference.json'),
        'Native session manifest/unit/sensor binding differs')
    opt_in = contract.get('runner_options', {}).get('static_surface_attitude_adapter')
    attitude_evidence = None
    if opt_in is not None:
        require(opt_in == attitude.configuration(manifest, True, True, 'static-ready'),
            'Replacement native opt-in profile differs from the explicit frozen profile')
        attitude_evidence = attitude.verify_capture(root, opt_in)
    else:
        require(not audit.get('static_surface_attitude_adjudication'),
            'Adjudicated audit requires its actual opt-in runtime/session/native proof')
    local = rows(session/'ledger.jsonl')
    matched = [value for value in local if value['id'] == row['id']]
    require(len(matched) == 1 and matched[0] == row, 'Native local and supplied selected ledger differ')
    require(native_observation_snapshot(root) == before, 'Native raw/session changed during verification')
    return dict(capture_root=str(root), canonical_status_unchanged=True,
        opt_in_attitude_evidence=attitude_evidence,
        original_native_evidence=before['raw'], session_inventory=before['session_files'],
        session_files={path:record['sha256'] for path,record in before['session_files'].items()})


def merge_entries(originals, replacements):
    require(len({entry['id'] for entry in originals}) == len(originals), 'Duplicate canonical source IDs')
    known = {entry['id']:entry for entry in originals}
    require(set(replacements) <= set(known), 'Foreign replacement canonical ID')
    result = []
    for original in originals:
        row = copy.deepcopy(original)
        if original['id'] in replacements:
            candidate = replacements[original['id']]
            # Canonical quota ID/header stays fixed; runtime candidate alias is
            # retained in the explicit adoption mapping, not as a fake raw ID.
            for key in ('scenario', 'scenario_sha256', 'metadata'):
                row[key] = candidate[key]
        result.append(row)
    return result


def evidence_plan(environment, final_manifest, capture_output, historical_counts,
                  replacements, shadow_inventory, free_gib):
    return dict(schema='base5_environment_qualification_plan_v1', environment=environment,
        final_manifest=str(final_manifest), capture_output=str(capture_output),
        planned_source_scenes=5000, bulk_training_observations_collected=0,
        historical_native=historical_counts, newly_verified_replacement_observations=len(replacements),
        newly_assembled_source_is_historical_exact_smoke=False,
        source_and_native_parent_history_preserved=True, shadow_preparation=shadow_inventory,
        qualified_environments=[], bulk_ready=False, native_qualified=False,
        completion_live_qualified=False, bulk_started=False, training_started=False,
        operational_storage_budget_gib=45., safety_headroom_gib=10.,
        observed_free_gib=free_gib, storage_gate_passed=free_gib >= 55.,
        bulk_blockers=['new_final_exact_smoke_not_registered_or_qualified',
            'new_final_Q90_upright_CPU_inputs_and_loader_pending',
            'new_final_partial_and_completed_resume_pending', 'current_full_CPU_regression_pending',
            'production_release_not_issued'] + ([] if free_gib >= 55 else ['insufficient_storage_headroom']),
        next_steps=['Register a fresh exact final-source 32..48-scene native pilot before capture.',
            'Include independent shadow controls only after exact terrain/reference binding.',
            'Verify Q90/upright/three-class/current CPU train and val pipelines.',
            'Verify partial and completed resume against their actual immutable observations.',
            'Freeze a separate environment-specific production release only after every real gate passes.'],
        limits=['Historical 28/32 approvals, roof adjudication, replacements, calibration and final exact smoke have different denominators.',
            'This CPU assembly does not execute capture, fake 5000 selected rows, resume old captures or permit --complete.',
            'Saved sensor/declared UI environment are not detector AP or competition-PC latency certification.'])


def assemble(source, registration, historical_ledgers, replacement_pairs, output,
             capture_output, adjudications=(), historical_helper_snapshot=None):
    source, registration = Path(source).resolve(strict=True), Path(registration).resolve(strict=True)
    output, capture_output = Path(output).resolve(), Path(capture_output).resolve()
    require(not output.exists(), 'Fresh CPU assembly directory required')
    require(not capture_output.exists(), 'Capture output must be a fresh future path')
    require(output != capture_output and output not in capture_output.parents and capture_output not in output.parents,
        'Release assembly and future capture output must be separate')
    import generate_base5_independent as gen
    protected = [source.parent, gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()]
    for root in protected:
        require(output != root and root not in output.parents and capture_output != root and root not in capture_output.parents,
            'Outputs must be outside source and simulator runtime')
    source_doc, registration_doc = read(source), read(registration)
    environment = source_doc['config'].get('environment_tag')
    require(environment in ENVIRONMENTS and source_doc.get('count') == len(source_doc['scenes']) == 5000,
        'Full new-environment 5000 source required')
    require(source_doc.get('bulk_ready') is False and source_doc.get('native_qualified') is False,
        'Only unqualified CPU source may be assembled')
    require(shadow.KEY not in source_doc['config'], 'This assembly retains all fresh shadow paths as on-demand')
    require(registration_doc.get('source_manifest') == str(source)
        and registration_doc.get('source_manifest_sha256') == sha256(source)
        and registration_doc.get('environment') == environment,
        'Historical registration must name the immutable original full source')
    layout.validate_manifest_overlay(source_doc)
    original_pins = pins.collect_inputs(source)
    add_pin(original_pins, registration)
    historical_contract = historical_registration_contract(registration_doc, source, historical_helper_snapshot)
    for path, digest in historical_contract['historical_runtime_pins'].items():
        add_pin(original_pins, path, digest)
    original_entries = {e['id']:e for e in source_doc['scenes']}
    require(len(original_entries) == 5000 and all(TOKEN.fullmatch(ident) for ident in original_entries),
        'Safe unique canonical IDs required')
    registered = {r['id']:r for r in registration_doc['selected']}
    require(len(registered) == len(registration_doc['selected']) and 32 <= len(registered) <= 48,
        'Unique original bounded native registration required')
    history = [];historical_raw = {};historical_sessions = {}
    for path in historical_ledgers:
        add_pin(original_pins, path)
        for row in rows(path):
            require(row['id'] in registered and row['scenario_sha256'] == registered[row['id']]['scenario_sha256']
                == original_entries[row['id']]['scenario_sha256'], 'Historical native is outside its exact registration')
            root = Path(row['capture_root']).resolve(strict=True)
            before = native_observation_snapshot(root)
            for protected_root in (root, root.parents[1]):
                require(output != protected_root and protected_root not in output.parents
                    and capture_output != protected_root and protected_root not in capture_output.parents,
                    'Output cannot be inside a historical native session')
            require(sha256(root/'scenario.json') == row['scenario_sha256'] and read(root/'audit.json') == row.get('audit'),
                'Historical native scenario/audit changed')
            events = [
                json.loads(line) for line in (root.parents[1]/'event_cmd_requests.jsonl').read_text().splitlines() if line.strip()]
            require(events and all(type(event.get('recorded_at')) in (int, float)
                and math.isfinite(event['recorded_at'])
                and event['recorded_at'] >= historical_contract['historical_registered_at_unix'] for event in events),
                'Historical native service events predate their actual registration')
            require(native_observation_snapshot(root) == before, 'Historical native raw/session changed during verification')
            historical_raw[str(root)] = before['raw']
            session_key = str(root.parents[1])
            require(session_key not in historical_sessions
                or historical_sessions[session_key] == before['session_files'],
                'Historical native session changed between observations')
            historical_sessions[session_key] = before['session_files']
            history.append(row)
    require(len(history) == len(registered) and {r['id'] for r in history} == set(registered),
        'Keep every original registered observation, including every review')
    require(len({r['id'] for r in history}) == len(history), 'Do not merge the same parent selection twice')
    history_lookup = {r['id']:r for r in history}
    historical_counts = dict(requested=len(registered), observed=len(history),
        status_counts=dict(Counter(r['status'] for r in history)), original_review_ids=sorted(
            r['id'] for r in history if r['status'] != 'accepted_candidate'))
    changes, mapping, native_evidence = {}, [], []
    for manifest, selected in replacement_pairs:
        manifest, selected = Path(manifest).resolve(strict=True), Path(selected).resolve(strict=True)
        require(output != manifest.parent and manifest.parent not in output.parents,
            'Output cannot be inside a preserved replacement source')
        add_pin(original_pins, manifest);add_pin(original_pins, selected)
        deck = read(manifest);selected_rows = rows(selected)
        require(deck.get('count') == len(deck.get('scenes', [])) == len(selected_rows) == 1,
            'One explicitly selected native observation per replacement manifest required')
        require(deck['config'].get('environment_tag') == environment
            and deck['config'].get(unit.KEY) == source_doc['config'].get(unit.KEY), 'Replacement environment/unit differs')
        layout.validate_manifest_overlay(deck, allow_subset=True)
        entry = deck['scenes'][0];meta = read(entry['metadata'])
        canonical = meta.get('fresh_completion_source', {}).get('source_cell_id')
        require(canonical in registered and canonical not in changes
            and history_lookup[canonical]['status'] != 'accepted_candidate',
            'Only a unique historically reviewed canonical slot may be replaced')
        original = original_entries[canonical]
        proof = replacement_shape(original, read(original['metadata']), entry, meta, deck)
        evidence = verify_accepted_native(manifest, entry, deck, selected_rows[0])
        native_root = Path(evidence['capture_root'])
        for protected_root in (native_root, native_root.parents[1]):
            require(output != protected_root and protected_root not in output.parents
                and capture_output != protected_root and protected_root not in capture_output.parents,
                'Outputs cannot be inside a replacement native session')
        changes[canonical] = entry;mapping.append(dict(proof, replacement_manifest=str(manifest),
            replacement_ledger=str(selected), native_capture=evidence['capture_root']))
        native_evidence.append(evidence)
        for path, record in evidence['original_native_evidence'].items():
            add_pin(original_pins, path, record['sha256'])
        for path, digest in evidence['session_files'].items():
            add_pin(original_pins, path, digest)
        for key in ('scenario', 'metadata'):
            add_pin(original_pins, entry[key])
        overlay = layout.overlay_for_metadata(meta)
        original_pins.update(overlay.get('source_hashes', {}))
        add_pin(original_pins, meta['training_split_overlay']['path'], meta['training_split_overlay']['sha256'])
    require(changes, 'At least one explicitly native-approved replacement required')
    adjudication_pins = []
    for path in adjudications:
        path = add_pin(original_pins, path)
        adjudication_pins.append(dict(path=str(path), sha256=sha256(path), historical_approval_changed=False))
    pins.verify_hashes(original_pins)
    runtime = runtime_hashes()
    output.mkdir(parents=True, exist_ok=False)
    staged = copy.deepcopy(source_doc)
    staged['scenes'] = merge_entries(source_doc['scenes'], changes)
    staged.pop('split_overlay', None)
    staged.update(status='ENVIRONMENT_NATIVE_ADOPTIONS_CPU_ONLY; new final qualification required',
        native_qualified=False, bulk_ready=False, bulk_started=False, training_started=False)
    # Canonical per-slot frozen specs follow the actual adopted source entries.
    specs = [read(entry['metadata'])['spec'] for entry in staged['scenes']]
    frozen = output/'adopted_source'/'frozen_specs.json';atomic_json(frozen, specs)
    staged['config'].update(frozen_specs_path=str(frozen), frozen_specs_sha256=sha256(frozen))
    staged.setdefault('provenance', {})['new_environment_native_adoptions'] = dict(policy=POLICY,
        source_manifest=str(source), source_manifest_sha256=sha256(source),
        historical_registration=str(registration), historical_registration_sha256=sha256(registration),
        mapping=mapping, old_raw_and_ledger_modified=False, independent_training_observations_added=0)
    adopted = output/'adopted_source'/'manifest.json';atomic_json(adopted, staged)
    result = split_prepare.prepare(adopted, output/'final_source')
    final = Path(result['manifest']);final_doc = read(final)
    layout.validate_manifest_overlay(final_doc)
    audit = scenario_audit.audit(final)
    atomic_json(output/'full_source_audit.json', audit)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000,
        'Adopted full5000 CPU audit failed; do not use partial products')
    pins.verify_hashes(original_pins);pins.verify_hashes(runtime)
    for root, before in historical_raw.items():
        require(raw_pins(root) == before, 'Original native raw changed during assembly')
    for session, before in historical_sessions.items():
        require(native_session_files(session) == before, 'Original native session changed during assembly')
    for evidence in native_evidence:
        require(native_observation_snapshot(evidence['capture_root']) == dict(
            raw=evidence['original_native_evidence'], session_files=evidence['session_inventory']),
            'Replacement native raw/session changed')
        pins.verify_hashes(evidence['session_files'])
    ancestor = capture_output
    while not ancestor.exists():
        ancestor = ancestor.parent
    free = shutil.disk_usage(ancestor).free / 2**30
    on_demand = [entry['id'] for entry in final_doc['scenes']
        if environment_smoke.motion_kind(read(entry['metadata'])) == 'unprepared_shadow']
    plan = evidence_plan(environment, final, capture_output, historical_counts, mapping,
        dict(materialized_ids=[], exact_controls_materialized=0,
             on_demand_scene_ids=on_demand, all_source_shadow_prepared=not on_demand,
             separate_shadow_trial_is_not_bulk_source_materialization=True), free)
    atomic_json(output/'qualification_plan.json', plan)
    receipt = dict(schema=SCHEMA, policy=POLICY, source_manifest=str(source), source_sha256=sha256(source),
        final_manifest=str(final), final_manifest_sha256=sha256(final), environment=environment,
        original_input_hashes=original_pins, runtime_hashes=runtime, original_native_inventory=historical_raw,
        original_session_inventory=historical_sessions,
        replacement_native=native_evidence, mapping=mapping, historical_native=historical_counts,
        historical_registration_contract=historical_contract,
        original_source_payloads_preserved=5000-len(changes), source_scenes=5000,
        all_final_metadata_derives_new_training_overlay=True,
        native_exact_final_source_observations=0, bulk_training_observations=0,
        old_review_adjudications=adjudication_pins, adjudications_do_not_rewrite_historical_approval=True,
        common_layout_overlay_recomputed=True, fake_5000_selected_ledger_created=False,
        source_raw_ledger_and_registration_preserved=True, native_qualified=False, bulk_ready=False,
        completion_live_qualified=False, bulk_started=False, training_started=False,
        qualification_plan_sha256=sha256(output/'qualification_plan.json'),
        full_source_audit_sha256=sha256(output/'full_source_audit.json'))
    atomic_json(output/'assembly_receipt.json', receipt)
    return receipt


def assembled_source_context(manifest, receipt_path):
    """새 assembler schema를 정직하게 검증한다. 옛 BUILD_POLICY로 위장하지 않는다."""
    manifest, receipt_path = Path(manifest).resolve(strict=True), Path(receipt_path).resolve(strict=True)
    deck, receipt = read(manifest), read(receipt_path)
    environment = deck['config'].get('environment_tag')
    require(receipt.get('schema') == SCHEMA and receipt.get('policy') == POLICY
        and receipt.get('environment') == environment and environment in ENVIRONMENTS
        and receipt.get('final_manifest') == str(manifest)
        and receipt.get('final_manifest_sha256') == sha256(manifest), 'Matching new assembly receipt required')
    require(receipt.get('source_scenes') == deck.get('count') == len(deck.get('scenes', [])) == 5000
        and receipt.get('source_raw_ledger_and_registration_preserved') is True
        and receipt.get('common_layout_overlay_recomputed') is True
        and receipt.get('fake_5000_selected_ledger_created') is False,
        'Complete source assembly and preservation proof required')
    for key in ('native_qualified', 'bulk_ready', 'completion_live_qualified', 'bulk_started', 'training_started'):
        require(receipt.get(key) is False, 'CPU assembly cannot claim ' + key)
    require(deck.get('bulk_ready') is False and deck.get('native_qualified') is False,
        'Source is not yet a production release')
    declared = deck['config'].get('collection_environment', {})
    require(declared.get('weather') == environment[:-2] and declared.get('hour') == int(environment[-2:]),
        'Physical environment declaration differs from new source')
    require(shadow.KEY not in deck['config'] and 'asset_scale_policy' not in deck['config'],
        'This qualification requires unit-only, fresh on-demand shadow source')
    pins.verify_hashes(receipt['original_input_hashes']);pins.verify_hashes(receipt['runtime_hashes'])
    verify_preserved_native(receipt)
    layout_audit = layout.validate_manifest_overlay(deck)
    overlay = layout.read_overlay(deck['split_overlay']['path'])
    native = unit.load(deck['config'][unit.KEY]);catalog = read(native['catalog_reference']['path'])
    catalog = {actor['data_id']:actor for actor in catalog['assets']}
    require(len(native['assets']) == 19, 'All 19 unit physical assets required')
    entries, metadata = {}, {};scale_counts = Counter()
    for entry in deck['scenes']:
        ident = entry['id'];require(ident not in entries and TOKEN.fullmatch(ident), 'Safe unique canonical source IDs required')
        require(sha256(entry['scenario']) == entry['scenario_sha256'], 'Final scenario changed')
        meta, scenario = read(entry['metadata']), read(entry['scenario'])
        require(meta['split'] == entry['split'] and meta.get(unit.KEY) == deck['config'][unit.KEY]
            and meta.get('fresh_environment_provenance', {}).get('environment') == environment,
            'Final source unit/capture split/environment identity differs')
        raw_unit.reject_scale_metadata(meta);scale_counts.update(raw_unit.raw_unit_scales(scenario))
        raw_unit.bind_actors(scenario, meta, catalog);validate_entry(entry, deck)
        if close.KEY in meta['spec'] or close.KEY in meta.get('avoidance_observation', {}):
            close.validate_entry(entry, deck)
        for container in raw_unit.CONTAINERS:
            require([a['UNIQUEID'] for a in scenario.get(container, [])] ==
                [a['unique_id'] for a in meta['runtime_actors'] if a['source_list'] == container],
                'Final actor container/order differs')
        if environment_smoke.motion_kind(meta) == 'editor_prepared':
            ground.validate_motion_ground(meta, scenario)
        entries[ident], metadata[ident] = entry, meta
    hashes = pins.collect_inputs(manifest)
    hashes.update(overlay.get('source_hashes', {}));hashes.update(overlay.get('finalization_source_hashes', {}))
    hashes[str(receipt_path)] = sha256(receipt_path)
    add_pin(hashes, deck['split_overlay']['path'], deck['split_overlay']['sha256'])
    pins.verify_hashes(hashes)
    return dict(full=manifest, deck=deck, entries=entries, metadata=metadata, overlay=overlay,
        unit_assets=set(native['assets']), input_hashes=hashes, layout_audit=layout_audit,
        raw_scale_counts=dict(scale_counts), environment=environment, assembly_receipt=receipt_path)


def register_smoke(manifest, receipt, output, target=32, maximum=48):
    """새 source의 exact 32~48장을 결과를 보기 전에 별도로 사전 등록한다."""
    manifest, output = Path(manifest).resolve(strict=True), Path(output).resolve()
    import generate_base5_independent as gen
    for protected in (manifest.parent, gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()):
        require(output != protected and protected not in output.parents, 'Registration output must be outside source/runtime')
    require(not output.exists(), 'Fresh exact smoke registration directory required')
    receipt_document = read(receipt)
    require_safe_registration_output(output, manifest, receipt_document)
    runtime = runtime_hashes()
    context = assembled_source_context(manifest, receipt)
    selected, coverage = environment_smoke.select(context['deck'], context['metadata'], context['overlay'],
        context['unit_assets'], target, maximum)
    audit = scenario_audit.audit(manifest)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000, 'Final full source CPU audit failed')
    candidate = copy.deepcopy(context['deck'])
    candidate.update(count=len(selected), scenes=[copy.deepcopy(context['entries'][r['id']]) for r in selected],
        status='PREREGISTERED_ASSEMBLED_ENVIRONMENT_EXACT_SMOKE_ONLY', native_qualified=False,
        bulk_ready=False, bulk_started=False, training_started=False)
    output.mkdir(parents=True, exist_ok=False)
    atomic_json(output/'manifest.json', candidate);atomic_json(output/'source_audit.json', audit)
    pins.verify_hashes(runtime);pins.verify_hashes(context['input_hashes'])
    verify_preserved_native(receipt_document)
    document = dict(schema=REGISTRATION_SCHEMA, policy=REGISTRATION_POLICY,
        helper_sha256=sha256(__file__), registered_at_unix=time.time(),
        source_manifest=str(manifest), source_manifest_sha256=sha256(manifest),
        assembly_receipt=dict(path=str(context['assembly_receipt']), sha256=sha256(context['assembly_receipt'])),
        environment=context['environment'], selected=selected, coverage=coverage,
        subset_manifest_sha256=sha256(output/'manifest.json'), source_audit_sha256=sha256(output/'source_audit.json'),
        input_hashes=context['input_hashes'], runtime_hashes=runtime, raw_scale_counts=context['raw_scale_counts'],
        native_observations_must_follow_registration=True, native_qa_unchanged=True,
        native_qualified=False, bulk_ready=False, completion_live_qualified=False,
        bulk_started=False, training_started=False, capture_executed=False,
        native_runtime_profile=attitude.configuration(output/'manifest.json', True, True, 'static-ready'),
        native_profile_production_bulk_authorized=False,
        limitation='Historical observations/adjudications are not final exact smoke approvals; 24 shadow paths remain on-demand.')
    atomic_json(output/'registration.json', document)
    verify_preserved_native(receipt_document)
    return output/'manifest.json'


def registration_context(manifest, registration):
    manifest, registration = Path(manifest).resolve(strict=True), Path(registration).resolve(strict=True)
    document = read(registration)
    require(document.get('schema') == REGISTRATION_SCHEMA and document.get('policy') == REGISTRATION_POLICY
        and document.get('helper_sha256') == sha256(__file__), 'Current assembled-environment registration required')
    require(document.get('source_manifest') == str(manifest)
        and document.get('source_manifest_sha256') == sha256(manifest), 'Registered full source changed')
    for key in ('native_qualified', 'bulk_ready', 'completion_live_qualified', 'bulk_started', 'training_started', 'capture_executed'):
        require(document.get(key) is False, 'Registration cannot claim ' + key)
    require(document.get('native_observations_must_follow_registration') is True
        and document.get('native_qa_unchanged') is True, 'Exact preregistration and unchanged native gates required')
    stamp = document.get('registered_at_unix')
    require(type(stamp) in (int, float) and 0 < stamp <= time.time(), 'Valid preregistration timestamp required')
    runtime = runtime_hashes()
    require(document.get('runtime_hashes') == runtime, 'Registered runtime changed')
    pins.verify_hashes(runtime)
    reference = document['assembly_receipt']
    require(sha256(reference['path']) == reference['sha256'], 'Registered assembly receipt changed')
    context = assembled_source_context(manifest, reference['path'])
    require(document.get('input_hashes') == context['input_hashes']
        and document.get('environment') == context['environment']
        and document.get('raw_scale_counts') == context['raw_scale_counts'], 'Registered source input contract changed')
    selected, coverage = environment_smoke.select(context['deck'], context['metadata'], context['overlay'],
        context['unit_assets'], document['coverage']['target_frames'], document['coverage']['maximum_frames'])
    require(selected == document['selected'] and coverage == document['coverage'], 'Registered selection changed')
    subset = registration.parent/'manifest.json'
    expected = copy.deepcopy(context['deck'])
    expected.update(count=len(selected), scenes=[copy.deepcopy(context['entries'][r['id']]) for r in selected],
        status='PREREGISTERED_ASSEMBLED_ENVIRONMENT_EXACT_SMOKE_ONLY', native_qualified=False,
        bulk_ready=False, bulk_started=False, training_started=False)
    require(sha256(subset) == document['subset_manifest_sha256'] and read(subset) == expected
        and sha256(registration.parent/'source_audit.json') == document['source_audit_sha256'],
        'Registered exact subset or CPU audit changed')
    require(document.get('native_runtime_profile') == attitude.configuration(subset, True, True, 'static-ready')
        and document.get('native_profile_production_bulk_authorized') is False,
        'Registered opt-in native execution profile changed or incorrectly claims bulk authorization')
    context.update(doc=document, registration=registration, subset_path=subset, subset=expected,
        selected={r['id']:r for r in selected}, runtime=runtime)
    return context


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    assembly = modes.add_parser('assemble', help='Native-bound new full5000 CPU source; never a production release')
    for name in ('source', 'registration', 'output', 'capture-output'):
        assembly.add_argument('--'+name, type=Path, required=True)
    assembly.add_argument('--historical-ledger', type=Path, action='append', required=True)
    assembly.add_argument('--replacement', action='append', required=True, help='MANIFEST:SELECTED_LEDGER (one approved observation)')
    assembly.add_argument('--adjudication', type=Path, action='append', default=[])
    assembly.add_argument('--historical-helper-snapshot', type=Path,
        help='Required exact saved helper bytes when the historical registration helper changed')
    smoke = modes.add_parser('register-smoke', help='Preregister exact observations of the newly assembled source')
    for name in ('manifest', 'assembly-receipt', 'output'):
        smoke.add_argument('--'+name, type=Path, required=True)
    smoke.add_argument('--target-frames', type=int, default=32)
    smoke.add_argument('--maximum-frames', type=int, default=48)
    args = parser.parse_args()
    if args.mode == 'register-smoke':
        print(register_smoke(args.manifest, args.assembly_receipt, args.output, args.target_frames, args.maximum_frames));return
    pairs = [tuple(map(Path, item.split(':', 1))) for item in args.replacement]
    require(all(len(pair) == 2 for pair in pairs), 'Replacement requires manifest:ledger')
    result = assemble(args.source, args.registration, args.historical_ledger, pairs,
        args.output, args.capture_output, args.adjudication, args.historical_helper_snapshot)
    print(json.dumps({k:result[k] for k in ('final_manifest', 'source_scenes', 'historical_native',
        'native_exact_final_source_observations', 'bulk_training_observations', 'bulk_ready')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
