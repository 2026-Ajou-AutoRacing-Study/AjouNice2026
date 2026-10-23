#!/usr/bin/env python3
"""공통 보강 source의 실제 native 대체를 원래 여섯 release 검사에 연결한다.

기존 source/실패 raw/ledger/등록/QA는 변경하지 않는다. 재생성한 geometry는
실제 승인 manifest와 ledger로만 채택한다. 옛 geometry의 actual_retry 같은
provenance를 새 metadata에 복사하지 않고 별도 immutable binding으로 연결한다.
이 도구는 CPU 조립·검증만 한다. MORAI, native capture, 본수집은 실행하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from collections import Counter
from contextlib import contextmanager
import copy
import json
import math
from pathlib import Path
import threading
import time

import reinforcement_qualification as reinforced

profile = reinforced.profile
original = reinforced.original
assembly = original.qualification
SCHEMA = original.SCHEMA
BINDING_SCHEMA = 'base5_reinforced_native_adoption_binding_v1'
ADOPTION_SCHEMA = 'base5_reinforced_native_adoption_cpu_assembly_v1'
QUALIFICATION_SCHEMA = 'base5_reinforced_native_adopted_release_qualification_v1'
POLICY = 'actual-native-adoption-common-profile-original-six-gates-v1'
GATES = original.GATES
BASE_QUALIFY = original.qualify
BASE_ASSEMBLED_CONTEXT = assembly.assembled_source_context
_LOCK = threading.Lock()
read, require, physical, sha = profile.read, profile.require, profile.physical, profile.sha256
REFERENCES = ('source', 'build_extension', 'source_registration', 'assembly_receipt', 'manifest')
ACTIVE_RECOVERY_FIELDS = ('quality_replacement_contract','fresh_completion_source',
                          'replacement','secondary_far_vehicle_replacement')


def adapter_pin():
    return dict(path=str(physical(__file__)), sha256=sha(__file__),
                registration_schema=assembly.REGISTRATION_SCHEMA)


def verified_reference(record, label):
    require(isinstance(record, dict) and set(record) == {'path', 'sha256'}
        and profile.pin(record['path']) == record, 'Native adoption reference changed: '+label)
    return physical(record['path'])


def merge_pins(*groups):
    result = {}
    for group in groups:
        for path, digest in group.items():
            assembly.add_pin(result, path, digest)
    return result


def strip_overlay(metadata):
    return {key:value for key,value in metadata.items() if key != 'training_split_overlay'}


def canonical_metadata(metadata, canonical_id, manifest, native_metadata):
    """Rebase NEW planned metadata only; preserve the actual raw predecessor."""
    result = copy.deepcopy(metadata)
    for key in ACTIVE_RECOVERY_FIELDS:result.pop(key,None)
    result.pop('training_split_overlay',None)
    result['native_adoption_source'] = dict(policy=POLICY,canonical_id=canonical_id,
        approved_manifest=profile.pin(manifest),approved_metadata=profile.pin(native_metadata),
        actual_native_metadata_modified=False,previous_recovery_contract_is_historical=True,
        new_canonical_root=True)
    return result


def event_rows(path):
    payload = physical(path).read_bytes()
    require(payload and payload.endswith(b'\n'),'Complete actual native event log required')
    values = [json.loads(line) for line in payload.splitlines() if line.strip()]
    require(values and all(isinstance(row,dict) for row in values),'Native event objects required')
    return values


def registration_source(source, extension, registration, parent_binding=None):
    """Validate either original exact smoke or a genuine changed-row diagnostic."""
    document = read(registration)
    if document.get('schema') == assembly.REGISTRATION_SCHEMA:
        require(parent_binding is not None,'Adopted exact-smoke registration needs its verified parent binding')
        if assembly.assembled_source_context is adopted_source_context:
            context = assembly.registration_context(source,registration)
        else:
            with installed():context = assembly.registration_context(source,registration)
        return dict(kind='adopted_exact_smoke',doc=document,entries=context['entries'],
            metadata=context['metadata'],selected=document['selected'],
            registered_at_unix=document['registered_at_unix'],input_hashes=document['input_hashes'])
    if document.get('schema') == reinforced.unit_smoke.SCHEMA:
        with reinforced.installed():
            context = reinforced.unit_smoke.registration_context(source,registration)
        return dict(kind='original_exact_smoke',doc=document,entries=context['entries'],
            metadata=context['metadata'],selected=document['selected'],
            registered_at_unix=document['registered_at_unix'],input_hashes=document['input_hashes'])
    require(document.get('schema') == 'base5_reinforcement_changed_row_pilot_registration_v1'
        and document.get('helper') == profile.pin(reinforced.__file__)
        and document.get('selected_before_capture') is True
        and document.get('native_outcomes_used_for_selection') is False
        and document.get('canonical_exact32_qualification_claimed') is False
        and document.get('bulk_ready') is False,
        'Explicit unchanged diagnostic preregistration required; no fake unit-smoke registration')
    stamp = document.get('registered_at_unix')
    require(type(stamp) in (int,float) and math.isfinite(stamp) and 0 < stamp <= time.time(),
            'Finite actual diagnostic preregistration timestamp required')
    paths = {key:verified_reference(document.get(key),'diagnostic '+key)
             for key in ('manifest','build_extension','diagnostic_manifest')}
    pilot = reinforced.extension_context(paths['manifest'],paths['build_extension'],allow_pilot=True)
    pilot_extension = read(paths['build_extension']);full_extension = read(extension)
    require(pilot_extension['phase'] == 'pilot'
        and pilot_extension['source'] == full_extension['source']
        and pilot_extension['source_build'] == full_extension['source_build']
        and pilot_extension['profile']['sha256'] == full_extension['profile']['sha256']
        and pilot_extension['allocation']['sha256'] == full_extension['allocation']['sha256']
        and pilot_extension['implementation'] == full_extension['implementation'],
        'Diagnostic and full source have different profile/allocation/source/implementation')
    subset = read(paths['diagnostic_manifest']);ids = document.get('ids')
    expected_ids = [row['assignment']['id'] for row in pilot_extension['changes']]
    require(type(ids) is list and 1 <= len(ids) <= 48 and len(set(ids)) == len(ids)
        and ids == expected_ids == [row['id'] for row in subset['scenes']]
        and subset.get('count') == len(ids)
        and subset.get('config',{}).get('environment_tag') == pilot['environment'],
        'Diagnostic cohort changed after preregistration')
    full = read(source);full_entries = {row['id']:row for row in full['scenes']}
    for entry in subset['scenes']:
        ident = entry['id']
        require(entry == pilot['entries'].get(ident) and ident in full_entries
            and entry['scenario_sha256'] == full_entries[ident]['scenario_sha256'],
            'Full source does not contain exact preregistered pilot geometry')
        a,b = pilot['metadata'][ident],read(full_entries[ident]['metadata'])
        # Frozen profile/allocation documents have equal bytes but distinct
        # pilot/full paths. Validate them above; do not pretend those paths match.
        strip = lambda meta:{k:v for k,v in meta.items()
            if k not in ('training_split_overlay',profile.PROVENANCE_KEY)}
        require(strip(a) == strip(b),'Pilot/full geometry or frozen actor metadata differs')
    pins = merge_pins(pilot['input_hashes'])
    for path in tuple(paths.values())+(physical(registration),):assembly.add_pin(pins,path)
    return dict(kind='registered_changed_rows',doc=document,entries=pilot['entries'],
        metadata=pilot['metadata'],selected=[dict(id=ident,
            scenario_sha256=pilot['entries'][ident]['scenario_sha256']) for ident in ids],
        registered_at_unix=stamp,input_hashes=pins)


def validate_final_semantics(source, final, extension, receipt):
    """Deterministic allocation/spec/roster and actual resulting near budgets."""
    before, after = source['entries'], final['entries']
    require(list(before) == list(after) and len(before) == len(after) == 5000,
            'Adoption must retain every canonical slot in original order')
    mappings = receipt.get('mapping', [])
    changes = {row['canonical_id']:row for row in mappings}
    require(len(changes) == len(mappings), 'Unique actual native adoption mapping required')
    require(set(changes) <= set(before), 'Native adoption contains a foreign canonical slot')
    ignored = {'frozen_specs_path', 'frozen_specs_sha256'}
    require({k:v for k,v in source['deck']['config'].items() if k not in ignored}
        == {k:v for k,v in final['deck']['config'].items() if k not in ignored},
        'Adoption changed common generator/profile/environment policy')
    for ident, old in before.items():
        old_meta, new_meta = source['metadata'][ident], final['metadata'][ident]
        new = after[ident]
        require(old_meta['spec'] == new_meta['spec']
            and profile.actor_identity(old_meta['runtime_actors']) == profile.actor_identity(new_meta['runtime_actors'])
            and old_meta['split'] == new_meta['split'] == old['split'] == new['split'],
            'Adoption changed frozen spec, ordered roster or capture split: '+ident)
        for key in profile.PRESERVED_META:
            require(old_meta.get(key) == new_meta.get(key), 'Adoption changed preserved source lineage: '+key)
        if ident not in changes:
            require(old['scenario_sha256'] == new['scenario_sha256']
                and strip_overlay(old_meta) == strip_overlay(new_meta),
                'Unmapped source geometry/metadata changed: '+ident)
        else:
            mapping = changes[ident]
            require(mapping['old_scenario_sha256'] == old['scenario_sha256']
                and mapping['new_scenario_sha256'] == new['scenario_sha256']
                and mapping['old_scenario_sha256'] != mapping['new_scenario_sha256'],
                'Native adoption mapping does not identify actual new geometry')

    _, baseline_rows = profile.records_from_manifest(extension['source']['path'])
    baseline = {row['id']:row for row in baseline_rows}
    allocation = read(extension['allocation']['path'])
    targets = {row['id']:row for row in allocation['assignments']}
    for ident, assignment in targets.items():
        profile.verify_transition(baseline[ident]['meta'], final['metadata'][ident], assignment)
    native = profile.unit.load(source['deck']['config'][profile.unit.KEY])
    sizes = {name:row['native_size_lwh_m'] for name,row in native['assets'].items()}
    final_rows = [dict(id=ident, entry=entry, meta=final['metadata'][ident]) for ident,entry in after.items()]
    old_near, new_near = profile.near_counts(baseline_rows, sizes), profile.near_counts(final_rows, sizes)
    document = profile.validate_profile(read(extension['profile']['path']))
    loss = sum(old_near.values()) - sum(new_near.values())
    require(sum(new_near.values()) >= document['minimum_remaining_near_actors']
        and loss <= math.floor(sum(old_near.values())*document['maximum_near_loss_fraction']),
        'Adopted geometry exceeded common global near-field protection')
    for rule in document['rules']:
        for name in rule['assets']:
            require(new_near[name] >= rule['minimum_asset_near_remaining']
                and old_near[name]-new_near[name] <= math.floor(old_near[name]*rule['maximum_asset_near_fraction']),
                'Adopted geometry exceeded common per-asset near-field protection: '+name)
    old_roles, new_roles = profile.near_role_counts(baseline_rows,sizes), profile.near_role_counts(final_rows,sizes)
    require(all(new_roles[key] >= 1 for key in old_roles), 'Adoption removed an existing near-field asset/role stratum')
    quota = lambda rows:Counter(tuple((a['name'],a['data_id'],a['motion'])
        for a in row['meta']['runtime_actors']) for row in rows)
    require(quota(baseline_rows) == quota(final_rows), 'Adoption changed full-deck class/asset/motion quotas')
    return dict(canonical_scenes=5000, adopted_ids=sorted(changes),
        unchanged_payloads=5000-len(changes), profile_targets_checked=len(targets),
        final_near_parent_counts=dict(new_near), actual_near_loss=loss,
        ordered_roster_preserved=True, capture_split_preserved=True,
        common_layout_overlay_verified=True, profile_allocation_unchanged=True,
        historical_geometry_provenance_copied_to_new_metadata=False)


def adopted_source_context(manifest, receipt_path):
    """Read our actual assembly schema, without masquerading as legacy assembly."""
    manifest,receipt_path = physical(manifest),physical(receipt_path)
    receipt,deck = read(receipt_path),read(manifest)
    if receipt.get('schema') != ADOPTION_SCHEMA:
        return BASE_ASSEMBLED_CONTEXT(manifest,receipt_path)
    require(receipt.get('policy') == POLICY and receipt.get('helper') == profile.pin(__file__)
        and receipt.get('final_manifest') == str(manifest)
        and receipt.get('final_manifest_sha256') == sha(manifest)
        and receipt.get('source_scenes') == deck.get('count') == len(deck.get('scenes',[])) == 5000,
        'Exact truthful reinforced native-adoption assembly/source required')
    for flag in ('native_qualified','bulk_ready','bulk_started','training_started','completion_live_qualified'):
        require(receipt.get(flag) is False,'CPU assembly cannot claim '+flag)
    require(receipt.get('source_raw_ledger_and_registration_preserved') is True
        and receipt.get('common_layout_overlay_recomputed') is True
        and receipt.get('fake_5000_selected_ledger_created') is False
        and deck.get('native_qualified') is False and deck.get('bulk_ready') is False,
        'Preserved actual native evidence and unqualified source required')
    environment = deck['config'].get('environment_tag')
    declared = deck['config'].get('collection_environment',{})
    require(environment in assembly.ENVIRONMENTS and receipt.get('environment') == environment
        and declared.get('weather') == environment[:-2] and declared.get('hour') == int(environment[-2:])
        and original.shadow.KEY not in deck['config'] and 'asset_scale_policy' not in deck['config'],
        'Unit-only independent environment declaration required')
    profile.inputs.verify_hashes(receipt['original_input_hashes'])
    profile.inputs.verify_hashes(receipt['runtime_hashes'])
    assembly.verify_preserved_native(receipt)
    audit_path = verified_reference(receipt.get('full_source_audit'),'adopted full source audit')
    audit = read(audit_path)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000,'Adopted full source CPU audit failed')
    layout_audit = profile.layout.validate_manifest_overlay(deck)
    overlay = profile.layout.read_overlay(deck['split_overlay']['path'])
    native = profile.unit.load(deck['config'][profile.unit.KEY])
    require(len(native['assets']) == 19,'All 19 physical unit assets required')
    catalog = {row['data_id']:row for row in read(native['catalog_reference']['path'])['assets']}
    metadata,entries,scales = {},{},Counter()
    smoke = reinforced.unit_smoke
    for entry in deck['scenes']:
        ident = entry['id'];require(ident not in entries and assembly.TOKEN.fullmatch(ident),'Safe unique canonical source IDs required')
        require(sha(entry['scenario']) == entry['scenario_sha256'],'Adopted final source payload changed')
        meta,scenario = read(entry['metadata']),read(entry['scenario'])
        require(meta['split'] == entry['split'] and meta.get(profile.unit.KEY) == deck['config'][profile.unit.KEY]
            and meta.get('fresh_environment_provenance',{}).get('environment') == environment,
            'Adopted unit/capture split/environment identity differs')
        smoke.raw_unit.reject_scale_metadata(meta)
        scales.update(smoke.raw_unit.raw_unit_scales(scenario))
        smoke.raw_unit.bind_actors(scenario,meta,catalog)
        smoke.validate_entry(entry,deck)
        if assembly.close.KEY in meta['spec'] or assembly.close.KEY in meta.get('avoidance_observation',{}):
            assembly.close.validate_entry(entry,deck)
        for container in smoke.raw_unit.CONTAINERS:
            require([row['UNIQUEID'] for row in scenario.get(container,[])]
                == [row['unique_id'] for row in meta['runtime_actors'] if row['source_list'] == container],
                'Adopted raw actor container/order changed')
        if smoke.motion_kind(meta) == 'editor_prepared':smoke.ground.validate_motion_ground(meta,scenario)
        metadata[ident],entries[ident] = meta,entry
    pins = merge_pins(profile.inputs.collect_inputs(manifest),receipt['original_input_hashes'],receipt['runtime_hashes'])
    for path in (receipt_path,audit_path,physical(__file__)):assembly.add_pin(pins,path)
    return dict(full=manifest,deck=deck,entries=entries,metadata=metadata,overlay=overlay,
        unit_assets=set(native['assets']),input_hashes=pins,layout_audit=layout_audit,
        raw_scale_counts=dict(scales),environment=environment,assembly_receipt=receipt_path)


def verify_history(source, registration, receipt):
    """Re-read actual original accepted/review rows, never parent recovery aliases."""
    selected = {row['id']:row for row in registration['selected']}
    require(len(selected) == len(registration['selected']) and 1 <= len(selected) <= 48,
            'Actual bounded original preregistration required')
    observations = {};pins = {}
    stamp = registration['registered_at_unix']
    roots = receipt.get('original_native_inventory', {})
    require(len(roots) == len(selected), 'Every original accepted/review observation must be preserved')
    for raw in roots:
        root = physical(raw);session = root.parents[1]
        matches = [row for row in assembly.rows(session/'ledger.jsonl')
                   if physical(row['capture_root']) == root]
        require(len(matches) == 1, 'Original native ledger must identify one actual raw capture')
        row = matches[0];ident = row['id']
        require(ident in selected and ident not in observations
            and row['scenario_sha256'] == selected[ident]['scenario_sha256']
            == source['entries'][ident]['scenario_sha256']
            == registration['entries'][ident]['scenario_sha256'] == sha(root/'scenario.json')
            and row.get('audit') == read(root/'audit.json'),
            'Historical observation differs from its registered original payload/audit')
        require(row.get('status') in ('accepted_candidate', 'review')
            and read(session/'shutdown_safety.json').get('park_verified') is True,
            'Actual original accepted/review row and verified final Park required')
        events = event_rows(session/'event_cmd_requests.jsonl')
        require(events and all(type(event.get('recorded_at')) in (int,float)
            and math.isfinite(event['recorded_at']) and event['recorded_at'] >= stamp for event in events),
            'Original native events predate their actual registration')
        observations[ident] = row
        assembly.add_pin(pins, session/'ledger.jsonl')
        assembly.add_pin(pins, session/'event_cmd_requests.jsonl')
    require(set(observations) == set(selected), 'Original history omitted or duplicated a registered observation')
    expected = dict(requested=len(selected), observed=len(observations),
        status_counts=dict(Counter(row['status'] for row in observations.values())),
        original_review_ids=sorted(ident for ident,row in observations.items() if row['status'] != 'accepted_candidate'))
    require(receipt.get('historical_native') == expected, 'Assembly historical counts differ from actual native history')
    require(set(receipt['mapping'][index]['canonical_id'] for index in range(len(receipt['mapping'])))
        == set(expected['original_review_ids']), 'Every reviewed registered slot needs an actual approved adoption')
    return observations, pins


def verify_native_mapping(source, final, receipt, history, registration):
    native = receipt.get('replacement_native', [])
    by_root = {row['capture_root']:row for row in native}
    mappings = receipt['mapping']
    require(len(by_root) == len(native) == len(mappings), 'One actual approved native observation per adoption required')
    pins = {};proofs = []
    reference = profile.unit.load(source['deck']['config'][profile.unit.KEY])
    for mapping in mappings:
        ident = mapping['canonical_id']
        require(ident in history and history[ident]['status'] == 'review', 'Only actual reviewed source slots may be adopted')
        manifest, ledger = physical(mapping['replacement_manifest']), physical(mapping['replacement_ledger'])
        deck, rows = read(manifest), assembly.rows(ledger)
        require(deck.get('count') == len(deck.get('scenes',[])) == len(rows) == 1,
                'One exact approved native manifest/ledger pair per adoption required')
        candidate = deck['scenes'][0];metadata = read(candidate['metadata'])
        proof = assembly.replacement_shape(registration['entries'][ident],registration['metadata'][ident],candidate,metadata,deck)
        expected = dict(proof,replacement_manifest=str(manifest),replacement_ledger=str(ledger),
                        native_capture=rows[0]['capture_root'])
        require(mapping == expected, 'Adoption mapping differs from actual source/recovery contract')
        observed = assembly.verify_accepted_native(manifest,candidate,deck,rows[0])
        require(by_root.get(observed['capture_root']) == observed, 'Adoption native receipt differs from preserved actual evidence')
        expected_metadata = canonical_metadata(metadata,ident,manifest,candidate['metadata'])
        require(final['entries'][ident]['scenario_sha256'] == candidate['scenario_sha256']
            and strip_overlay(final['metadata'][ident]) == expected_metadata,
            'Final canonical slot does not use the actual approved replacement metadata/geometry')
        root = physical(observed['capture_root'])
        measured = original.verify_unit_corners(root,read(root/'metadata.json'),read(root/'audit.json'),reference)
        original.pin_files(pins,observed['original_native_evidence'])
        original.pin_files(pins,observed['session_inventory'])
        for path in (manifest, ledger, candidate['scenario'], candidate['metadata']):
            assembly.add_pin(pins,path)
        proofs.append(dict(canonical_id=ident,runtime_candidate_id=candidate['id'],
            capture_root=str(root),scenario_sha256=candidate['scenario_sha256'],
            actual_native_verified=True,unit_corner_observations=measured))
    return proofs,pins


def effective_diagnostic(registration, history, final, observations):
    """Fixed preregistered IDs, each backed by original or actually adopted native."""
    by_id = {row['canonical_id']:row for row in observations}
    pins = {};verified = []
    reference = profile.unit.load(final['deck']['config'][profile.unit.KEY])
    for record in registration['selected']:
        ident = record['id'];row = history[ident]
        if row['status'] == 'review':
            require(ident in by_id,'Reviewed diagnostic ID lacks actual new native approval')
            verified.append(dict(canonical_id=ident,geometry='actual_native_adoption',
                scenario_sha256=by_id[ident]['scenario_sha256'],capture_root=by_id[ident]['capture_root']))
            continue
        root = physical(row['capture_root']);session = root.parents[1]
        # Profile diagnostics are static generic obstacles and use the original
        # guarded regular chunk. Do not fabricate a matching manifest for it.
        chunk_manifest = session.parent/'manifest.json'
        native_manifest = original.driver.verify_execution_source(chunk_manifest,session)
        original.driver.verify_regular(session,chunk_manifest)
        deck = read(native_manifest)
        exact = [entry for entry in deck['scenes'] if entry['id'] == ident]
        require(len(exact) == 1 and exact[0] == registration['entries'][ident]
            and exact[0]['scenario_sha256'] == final['entries'][ident]['scenario_sha256'],
            'Original approved diagnostic geometry is not the final canonical geometry')
        actual = assembly.verify_accepted_native(native_manifest,exact[0],deck,row)
        require(actual['opt_in_attitude_evidence'] is not None,
                'Original diagnostic must use the current static-attitude native wrapper')
        original.verify_unit_corners(root,read(root/'metadata.json'),read(root/'audit.json'),reference)
        original.pin_files(pins,actual['original_native_evidence'])
        original.pin_files(pins,actual['session_inventory'])
        for path in (chunk_manifest,native_manifest):assembly.add_pin(pins,path)
        verified.append(dict(canonical_id=ident,geometry='original_native_approval',
            scenario_sha256=row['scenario_sha256'],capture_root=str(root)))
    require(len(verified) == len(registration['selected']), 'Effective diagnostic omitted preregistered IDs')
    return dict(preregistered_ids=[row['id'] for row in registration['selected']],
        original_status_counts=dict(Counter(row['status'] for row in history.values())),
        actual_final_geometry_successes=verified,
        original_review_outcomes_preserved=True,
        original_unmodified_exact_pilot_pass_claimed=False,
        native_outcomes_used_to_reselect_ids=False),pins


def binding_context(document):
    require(document.get('schema') == BINDING_SCHEMA and document.get('policy') == POLICY
        and document.get('helper') == profile.pin(__file__)
        and document.get('native_qualified') is False and document.get('bulk_ready') is False
        and document.get('bulk_started') is False,
        'Explicit CPU-only reinforced native-adoption binding required')
    paths = {name:verified_reference(document.get(name),name) for name in REFERENCES}
    parent = None
    if document.get('parent_binding') is not None:
        paths['parent_binding'] = verified_reference(document['parent_binding'],'parent_binding')
        parent_document = read(paths['parent_binding'])
        require(parent_document.get('adoption_depth',0) == 1
            and document.get('adoption_depth') == 2
            and parent_document.get('build_extension') == document['build_extension'],
            'At most two explicit native-adoption stages with one common source/profile are supported')
        parent = verify_binding(paths['parent_binding'],paths['source'])
    else:
        require(document.get('adoption_depth') == 1,'First explicit native-adoption stage required')
    if document.get('pilot_receipt') is not None:
        paths['pilot_receipt'] = verified_reference(document['pilot_receipt'],'pilot_receipt')
    source = parent['context'] if parent is not None else reinforced.extension_context(paths['source'],paths['build_extension'])
    registration = registration_source(paths['source'],paths['build_extension'],paths['source_registration'],
        paths.get('parent_binding'))
    diagnostic = None
    if registration['kind'] == 'original_exact_smoke':
        require('pilot_receipt' in paths,'Actual changed-row diagnostic evidence is required for exact-smoke adoption')
        diagnostic = reinforced.verify_required_diagnostic(dict(manifest=str(paths['source']),
            reinforcement_pilot_receipt=str(paths['pilot_receipt'])))
    elif 'pilot_receipt' in paths:
        require(False,'A reviewed diagnostic must not be disguised as an unchanged successful pilot receipt')
    extension, receipt = read(paths['build_extension']), read(paths['assembly_receipt'])
    require(receipt.get('source_manifest') == str(paths['source'])
        and receipt.get('source_sha256') == sha(paths['source'])
        and receipt.get('final_manifest') == str(paths['manifest'])
        and receipt.get('final_manifest_sha256') == sha(paths['manifest']),
        'Native assembly is not derived from the exact reinforced source')
    require(receipt.get('source_registration') == profile.pin(paths['source_registration'])
        and receipt.get('historical_registration_kind') == registration['kind'],
        'Native assembly names another actual preregistration/kind')
    final = adopted_source_context(paths['manifest'],paths['assembly_receipt'])
    require(document.get('environment') == source['environment'] == final['environment'],
            'Reinforced/native-adopted environment identity differs')
    history, history_pins = verify_history(source,registration,receipt)
    semantics = validate_final_semantics(source,final,extension,receipt)
    observations, native_pins = verify_native_mapping(source,final,receipt,history,registration)
    # Old pilot observations certify their exact OLD geometry. Replaced pilot
    # IDs are certified by their explicit actual new native observations above.
    replaced = set(semantics['adopted_ids'])
    diagnostic_pins = {}
    if registration['kind'] == 'registered_changed_rows':
        diagnostic_summary,diagnostic_pins = effective_diagnostic(registration,history,final,observations)
        pilot_ids = {row['id'] for row in registration['selected']}
    elif parent is not None:
        pilot_ids = set(parent['summary']['diagnostic']['preregistered_ids'])
        diagnostic_summary = dict(preregistered_ids=sorted(pilot_ids),
            inherited_verified_binding=profile.pin(paths['parent_binding']),
            new_native_adoptions_for_inherited_ids=sorted(replaced & pilot_ids),
            historical_pilot_relabelled_as_new_geometry=False)
        diagnostic_pins = parent['evidence_hashes']
    else:
        diagnostic_summary = dict(original_unchanged_exact_pilot_verified=True,
            preserved_receipt=profile.pin(paths['pilot_receipt']),
            preregistered_ids=[row['id'] for row in diagnostic['observations']])
        diagnostic_pins = diagnostic['evidence_hashes']
        pilot_ids = {row['id'] for row in diagnostic['observations']}
    summary = dict(semantics,actual_native_adoptions=observations,
        historical_native=receipt['historical_native'],
        diagnostic=diagnostic_summary,
        original_pilot_history_preserved=True,
        pilot_ids_with_explicit_new_native_geometry=sorted(replaced & pilot_ids),
        historical_pilot_relabelled_as_new_geometry=False,
        final_exact_smoke_approvals_added=0,bulk_training_observations_added=0)
    pins = merge_pins(source['input_hashes'],registration['input_hashes'],
        final['input_hashes'],receipt['original_input_hashes'],
        receipt['runtime_hashes'],diagnostic_pins,history_pins,native_pins)
    for path in tuple(paths.values())+(physical(__file__),):
        assembly.add_pin(pins,path)
    for path in (physical(profile.__file__),physical(reinforced.__file__),
                 physical(profile.__file__).with_name('reinforcement_screen.py')):
        if path.is_file():assembly.add_pin(pins,path)
    profile.inputs.verify_hashes(pins,'Reinforced native-adoption source/native/pilot evidence')
    return dict(source=source,context=final,summary=summary,evidence_hashes=pins)


def verify_binding(binding, manifest=None):
    binding = physical(binding);document = read(binding)
    if manifest is not None:
        require(document.get('manifest') == profile.pin(manifest), 'Binding names a different final manifest')
    result = binding_context(document)
    require(document.get('summary') == result['summary'], 'Adoption summary differs from actual immutable evidence')
    require(document.get('evidence_hashes') == result['evidence_hashes'], 'Adoption evidence inventory changed')
    assembly.add_pin(result['evidence_hashes'],binding)
    return result


def prepare(source, build_extension, registration, historical_ledgers, replacement_pairs,
            pilot_receipt, output, capture_output, *, parent_binding=None):
    """Truthful CPU adoption assembly; original review/native evidence stays intact."""
    source,build_extension,registration,output,capture_output = map(physical,
        (source,build_extension,registration,output,capture_output))
    pilot_receipt = physical(pilot_receipt) if pilot_receipt is not None else None
    parent_binding = physical(parent_binding) if parent_binding is not None else None
    require(not output.exists() and not capture_output.exists(), 'Fresh adoption and future capture outputs required')
    parent = verify_binding(parent_binding,source) if parent_binding is not None else None
    if parent is not None:
        before = read(parent_binding)
        require(before.get('adoption_depth') == 1 and before['build_extension'] == profile.pin(build_extension),
                'Only one bounded follow-up adoption with unchanged profile/source implementation is supported')
    source_context = parent['context'] if parent is not None else reinforced.extension_context(source,build_extension)
    registered = registration_source(source,build_extension,registration,parent_binding)
    require((registered['kind'] != 'original_exact_smoke') == (pilot_receipt is None),
            'Changed-row history or separately verified prior pilot is required, not a fake successful receipt')
    if pilot_receipt is not None:
        reinforced.verify_required_diagnostic(dict(manifest=str(source),reinforcement_pilot_receipt=str(pilot_receipt)))
    require(output != capture_output and output not in capture_output.parents and capture_output not in output.parents,
            'Adoption and future capture outputs must remain separate')
    for path in (source,build_extension,registration)+(tuple([pilot_receipt]) if pilot_receipt else ()):
        require(output != path.parent and path.parent not in output.parents and output not in path.parents,
                'Adoption output overlaps preserved source/registration/pilot')
    runtime_root = physical(original.driver.attitude.candidate.gen.geo.DEFAULT_RUNTIME_ASSET_DIR)
    for target in (output,capture_output):
        require(target != runtime_root and runtime_root not in target.parents,'Output overlaps simulator runtime')
    original_pins = merge_pins(source_context['input_hashes'],registered['input_hashes'])
    raw_inventory,session_inventory,history = {},{},{}
    for ledger in historical_ledgers:
        assembly.add_pin(original_pins,ledger)
        for row in assembly.rows(ledger):
            ident = row['id'];require(ident not in history,'Duplicate original historical native row')
            root = physical(row['capture_root']);session = root.parents[1]
            for protected in (root,session):
                for target in (output,capture_output):
                    require(target != protected and protected not in target.parents and target not in protected.parents,
                            'Output overlaps original native/session evidence')
            snapshot = assembly.native_observation_snapshot(root)
            raw_inventory[str(root)] = snapshot['raw']
            previous = session_inventory.setdefault(str(session),snapshot['session_files'])
            require(previous == snapshot['session_files'],'Original session changed during history collection')
            original.pin_files(original_pins,snapshot['raw'])
            original.pin_files(original_pins,snapshot['session_files'])
            history[ident] = row
    historical = dict(requested=len(registered['selected']),observed=len(history),
        status_counts=dict(Counter(row['status'] for row in history.values())),
        original_review_ids=sorted(ident for ident,row in history.items() if row['status'] != 'accepted_candidate'))
    changes,mappings,native = {},[],[]
    for manifest,ledger in replacement_pairs:
        manifest,ledger = physical(manifest),physical(ledger)
        deck,rows = read(manifest),assembly.rows(ledger)
        require(deck.get('count') == len(deck.get('scenes',[])) == len(rows) == 1,
                'One explicitly approved native manifest/ledger pair per adoption required')
        entry = deck['scenes'][0];meta = read(entry['metadata'])
        ident = meta.get('fresh_completion_source',{}).get('source_cell_id')
        require(ident in history and history[ident]['status'] == 'review' and ident not in changes,
                'Only unique actually reviewed registered slots may be replaced')
        require(deck['config'].get('environment_tag') == source_context['environment']
            and deck['config'].get(profile.unit.KEY) == source_context['deck']['config'][profile.unit.KEY],
            'Replacement environment or unit reference differs')
        profile.layout.validate_manifest_overlay(deck,allow_subset=True)
        proof = assembly.replacement_shape(registered['entries'][ident],registered['metadata'][ident],entry,meta,deck)
        observation = assembly.verify_accepted_native(manifest,entry,deck,rows[0])
        for protected in (physical(observation['capture_root']),physical(observation['capture_root']).parents[1]):
            for target in (output,capture_output):
                require(target != protected and protected not in target.parents and target not in protected.parents,
                        'Output overlaps actual replacement native/session evidence')
        mapping = dict(proof,replacement_manifest=str(manifest),replacement_ledger=str(ledger),
            native_capture=observation['capture_root'])
        changes[ident] = entry;mappings.append(mapping);native.append(observation)
        for path in (manifest,ledger,entry['scenario'],entry['metadata']):assembly.add_pin(original_pins,path)
        original.pin_files(original_pins,observation['original_native_evidence'])
        original.pin_files(original_pins,observation['session_inventory'])
    partial_receipt = dict(original_native_inventory=raw_inventory,original_session_inventory=session_inventory,
        replacement_native=native,mapping=mappings,historical_native=historical)
    verify_history(source_context,registered,partial_receipt)
    assembly.verify_preserved_native(partial_receipt)
    profile.inputs.verify_hashes(original_pins)
    runtime = assembly.runtime_hashes()
    require(not output.exists(),'Never overwrite an existing adoption assembly')
    output.mkdir(parents=True,exist_ok=False)
    assembled = output/'assembled_source'
    staged = copy.deepcopy(source_context['deck'])
    staged['scenes'] = assembly.merge_entries(staged['scenes'],changes)
    for entry in staged['scenes']:
        if entry['id'] in changes:
            candidate = changes[entry['id']]
            mapping = next(row for row in mappings if row['canonical_id'] == entry['id'])
            metadata = canonical_metadata(read(candidate['metadata']),entry['id'],
                mapping['replacement_manifest'],candidate['metadata'])
            path = assembled/'canonical_metadata'/(entry['id']+'.json')
            profile.atomic_json(path,metadata)
            entry['metadata'] = str(path)
    staged.pop('split_overlay',None)
    staged.update(status='REINFORCED_NATIVE_ADOPTIONS_CPU_ONLY; new exact qualification required',
        native_qualified=False,bulk_ready=False,bulk_started=False,training_started=False)
    frozen = assembled/'adopted_source/frozen_specs.json'
    profile.atomic_json(frozen,[read(entry['metadata'])['spec'] for entry in staged['scenes']])
    staged['config'].update(frozen_specs_path=str(frozen),frozen_specs_sha256=sha(frozen))
    staged.setdefault('provenance',{})['reinforced_native_adoptions'] = dict(policy=POLICY,
        source_manifest=str(source),source_manifest_sha256=sha(source),source_registration=profile.pin(registration),
        mapping=mappings,old_raw_and_ledger_modified=False,independent_training_observations_added=0)
    staged_path = assembled/'adopted_source/manifest.json';profile.atomic_json(staged_path,staged)
    result = profile.split_builder.prepare(staged_path,assembled/'final_source')
    final_manifest = physical(result['manifest'])
    audit = profile.scenario_audit.audit(final_manifest)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000,
        'Adopted complete full5K source CPU audit failed; preserve partial products')
    audit_path = assembled/'full_source_audit.json';profile.atomic_json(audit_path,audit)
    receipt = dict(partial_receipt,schema=ADOPTION_SCHEMA,policy=POLICY,helper=profile.pin(__file__),
        source_manifest=str(source),source_sha256=sha(source),final_manifest=str(final_manifest),
        final_manifest_sha256=sha(final_manifest),environment=source_context['environment'],source_scenes=5000,
        source_registration=profile.pin(registration),historical_registration_kind=registered['kind'],
        original_input_hashes=original_pins,runtime_hashes=runtime,
        original_source_payloads_preserved=5000-len(changes),common_layout_overlay_recomputed=True,
        source_raw_ledger_and_registration_preserved=True,fake_5000_selected_ledger_created=False,
        native_qualified=False,bulk_ready=False,bulk_started=False,training_started=False,completion_live_qualified=False,
        full_source_audit=dict(path=str(audit_path),sha256=sha(audit_path)))
    receipt_path = assembled/'assembly_receipt.json';profile.atomic_json(receipt_path,receipt)
    document = dict(schema=BINDING_SCHEMA,policy=POLICY,helper=profile.pin(__file__),
        source=profile.pin(source),build_extension=profile.pin(build_extension),
        source_registration=profile.pin(registration),pilot_receipt=profile.pin(pilot_receipt) if pilot_receipt else None,
        assembly_receipt=profile.pin(receipt_path),manifest=profile.pin(receipt['final_manifest']),
        parent_binding=profile.pin(parent_binding) if parent_binding else None,
        adoption_depth=2 if parent_binding else 1,
        environment=receipt['environment'],native_qualified=False,bulk_ready=False,bulk_started=False)
    result = binding_context(document)
    document.update(summary=result['summary'],evidence_hashes=result['evidence_hashes'])
    path = output/'binding.json'
    require(not path.exists(), 'Never overwrite a native-adoption binding')
    profile.atomic_json(path,document)
    verify_binding(path,receipt['final_manifest'])
    return path


def register(manifest,binding,output,target=32,maximum=48):
    verified = verify_binding(binding,manifest)
    with installed():
        return assembly.register_smoke(manifest,verified['context']['assembly_receipt'],output,target,maximum)


def qualify_with_adoption(arguments):
    binding = arguments.get('reinforcement_adoption_binding')
    require(binding is not None, 'Actual reinforced native-adoption binding is required')
    verified = verify_binding(binding,arguments['manifest'])
    result = BASE_QUALIFY(arguments)
    require(set(result['results']) == set(GATES)
        and all(value.get('passed') is True for value in result['results'].values()),
        'All six original exact-native/shadow/input/CPU/resume gates must pass')
    require(physical(result['context']['assembly_receipt'])
        == physical(verified['context']['assembly_receipt']), 'Qualification uses another native assembly')
    result['input_hashes'] = merge_pins(result['input_hashes'],verified['evidence_hashes'])
    return result


@contextmanager
def installed():
    """Select truthful schema and extra source checks; all six original gates stay."""
    require(_LOCK.acquire(blocking=False), 'Concurrent/nested native-adoption qualifier forbidden')
    previous = original.QUALIFICATION_SCHEMA,original.POLICY,original.qualify,assembly.assembled_source_context
    original.QUALIFICATION_SCHEMA,original.POLICY,original.qualify = QUALIFICATION_SCHEMA,POLICY,qualify_with_adoption
    assembly.assembled_source_context = adopted_source_context
    try:
        yield
    finally:
        changed = (original.QUALIFICATION_SCHEMA != QUALIFICATION_SCHEMA or original.POLICY != POLICY
                   or original.qualify is not qualify_with_adoption
                   or assembly.assembled_source_context is not adopted_source_context)
        original.QUALIFICATION_SCHEMA,original.POLICY,original.qualify,assembly.assembled_source_context = previous
        _LOCK.release()
        require(not changed,'Native-adoption qualifier ownership changed')


def freeze(arguments,output):
    with installed():
        return original.freeze(arguments,output)


def qualified_release_context(release,environment,manifest):
    with installed():
        return original.qualified_release_context(release,environment,manifest)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    commands = parser.add_subparsers(dest='action',required=True)
    prepare_parser = commands.add_parser('prepare')
    for name in ('source','build-extension','registration','output','capture-output'):
        prepare_parser.add_argument('--'+name,type=Path,required=True)
    prepare_parser.add_argument('--pilot-receipt',type=Path)
    prepare_parser.add_argument('--parent-binding',type=Path,
        help='One bounded follow-up exact-cohort adoption after an actual first adopted source')
    prepare_parser.add_argument('--historical-ledger',type=Path,action='append',required=True)
    prepare_parser.add_argument('--replacement',action='append',default=[],
        help='MANIFEST:SELECTED_LEDGER, one actual approved native observation')
    check = commands.add_parser('verify')
    check.add_argument('--binding',type=Path,required=True)
    check.add_argument('--manifest',type=Path)
    registration = commands.add_parser('register')
    for name in ('manifest','binding','output'):
        registration.add_argument('--'+name,type=Path,required=True)
    registration.add_argument('--target',type=int,default=32)
    registration.add_argument('--maximum',type=int,default=48)
    final = commands.add_parser('freeze')
    for name in ('manifest','smoke-registration','capture-root','shadow-cases','partial-snapshot',
        'partial-resume-receipt','completed-resume-receipt','training-input-report','loader-report',
        'cpu-regression-receipt','capture-output','reinforcement-adoption-binding','output'):
        final.add_argument('--'+name,type=Path,required=True)
    final.add_argument('--pytest-xml',type=Path,action='append',required=True)
    args = parser.parse_args(argv)
    if args.action == 'prepare':
        pairs = []
        for value in args.replacement:
            parts = value.split(':')
            require(len(parts) == 2 and all(parts), 'Replacement must be MANIFEST:SELECTED_LEDGER')
            pairs.append(tuple(map(physical,parts)))
        print(prepare(args.source,args.build_extension,args.registration,args.historical_ledger,pairs,
            args.pilot_receipt,args.output,args.capture_output,parent_binding=args.parent_binding))
    elif args.action == 'verify':
        result = verify_binding(args.binding,args.manifest)
        print(dict(passed=True,summary=result['summary'],bulk_ready=False))
    elif args.action == 'register':
        print(register(args.manifest,args.binding,args.output,args.target,args.maximum))
    else:
        values = {key:([str(physical(p)) for p in value] if isinstance(value,list) else str(physical(value)))
            for key,value in vars(args).items() if key not in ('action','output')}
        print(freeze(values,args.output))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
