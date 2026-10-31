#!/usr/bin/env python3
"""새 환경의 실제 exact native·입력·재개·음영 근거만 release로 묶는다.

기존 Sunny13 release나 역사 승인 수를 새 환경 승인으로 바꾸지 않는다.
이 파일은 CPU 검증과 새 receipt 작성만 하며 ROS·수집·학습을 시작하지 않는다.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import importlib
import math
from pathlib import Path
import shutil
import tempfile

import audit_base5_scenarios as scenario_audit
import base5_environment_qualification as qualification
import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_shadow_ground_reuse as shadow
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, prepare_session, sha256
import finalize_base5_final_bounded_release as common
from finalize_base5_layout_split import planned_fields_match
from finalize_base5_unit_release import verify_unit_corners
import prepare_base5_final_bounded_smoke as cpu
import prepare_base5_unit_environment_smoke as smoke
import run_base5_batch_static_attitude as driver
from verify_base5_completed_resume import snapshot as completed_snapshot


SCHEMA = 'base5_obstacle_support_release_v1'
QUALIFICATION_SCHEMA = 'base5_new_environment_release_qualification_v1'
POLICY = 'new-environment-exact-native-static-attitude-qualified-v159'
PROFILE = 'static-attitude-regular-batch+strict-spatial-recovery'
SHADOW_SCHEMA = 'base5_environment_independent_shadow_cases_v1'
PARTIAL_SCHEMA = 'base5_environment_partial_resume_snapshot_v1'
PARTIAL_RECEIPT = 'base5_environment_partial_resume_receipt_v1'
GATES = driver.GATES
read, require = qualification.read, qualification.require


def file_inventory(root):
    """내용뿐 아니라 새 파일·inode·mtime 변화도 보존 위반으로 본다."""
    root = Path(root).resolve(strict=True);result = {}
    for path in sorted(root.rglob('*')):
        require(not path.is_symlink(), 'Native evidence cannot contain symlinks')
        if path.is_file():
            stat = path.stat()
            result[str(path)] = dict(sha256=sha256(path), inode=stat.st_ino,
                mtime_ns=stat.st_mtime_ns, size=stat.st_size)
    require(result, 'Nonempty actual evidence directory required')
    return result


def pin_files(result, records):
    for path, record in records.items():
        qualification.add_pin(result, path, record['sha256'])


def exact_child(manifest, context):
    manifest = Path(manifest).resolve(strict=True);deck = read(manifest)
    require({k:v for k,v in deck.items() if k not in ('count','scenes')} ==
        {k:v for k,v in context['subset'].items() if k not in ('count','scenes')},
        'Native child header/config differs from preregistered exact subset')
    entries = deck.get('scenes', [])
    require(deck.get('count') == len(entries) and 1 <= len(entries) <= 16
        and len({entry['id'] for entry in entries}) == len(entries)
        and all(entry == context['entries'].get(entry['id']) for entry in entries)
        and all(entry['id'] in context['selected'] for entry in entries),
        'Only current preregistered source entries may appear in native child')
    expected_order = [entry['id'] for entry in context['subset']['scenes'] if entry['id'] in {e['id'] for e in entries}]
    require([entry['id'] for entry in entries] == expected_order, 'Native child changed registered source order')
    return deck


def native_child(session, context):
    """등록 chunk와 실제 실행 사본을 경로·원문 bytes 양쪽으로 연결한다."""
    session = Path(session).resolve(strict=True)
    original = session.parent/'manifest.json'
    deck = exact_child(original, context)
    execution = driver.verify_execution_source(original, session)
    require(exact_child(execution, context) == deck
        and execution.read_bytes() == original.read_bytes(), 'Actual execution copy changed exact child source')
    return original, execution, deck


def verify_native(context, capture):
    capture = Path(capture).resolve(strict=True);before = file_inventory(capture)
    expected = context['selected'];count = len(expected)
    require(32 <= count <= 48, '32..48 preregistered exact native scenes required')
    state = read(capture/'status.json');guard = read(capture/driver.GUARD)
    require(state.get('stage') == 'complete' and state.get('requested') == count
        and state.get('accepted_candidate') == count and state.get('review') == state.get('failed') == 0,
        'Every exact native scene must be accepted with no hidden deficit')
    require(guard.get('manifest_sha256') == sha256(context['subset_path'])
        and guard.get('environment') == context['environment']
        and guard.get('indices') == list(range(count)) and 1 <= guard.get('chunk_size', 0) <= 16
        and guard.get('runtime_profile') == driver.profiles()
        and guard.get('runtime_hashes') == driver.runtime_hashes()
        and guard.get('prototype_only') is True and guard.get('qualified_release') is None,
        'Actual exact pilot did not use the explicit new regular/recovery runtime')
    rows = qualification.rows(capture/'selected_ledger.jsonl')
    require(len(rows) == count and {row.get('source_cell_id') for row in rows} == set(expected)
        and all(row['id'] == row['source_cell_id'] for row in rows),
        'Recovered/renamed geometry cannot count as a preregistered exact native observation')
    reference = unit.load(context['deck']['config'][unit.KEY])
    covered = set();physical = [];compounds = [];observations = [];sessions = {};hashes = {}
    all_roots = set()
    for row in rows:
        ident = row['source_cell_id'];entry = context['entries'][ident]
        root = Path(row['capture_root']).resolve(strict=True);root.relative_to(capture)
        require(root not in all_roots, 'The same raw capture cannot count twice');all_roots.add(root)
        session = root.parents[1]
        if session not in sessions:
            original_manifest, manifest, deck = native_child(session, context)
            session_contract = read(session/'session_contract.json')
            with tempfile.TemporaryDirectory(prefix='base5-environment-contract-') as temp:
                prepare_session(Path(temp)/'session', manifest,
                    pins.ROOT/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
                    pins.ROOT/'configs/sensors/Sensor_competition_local.json',
                    runner_options=session_contract.get('runner_options'))
                require(read(Path(temp)/'session'/'session_contract.json') == session_contract,
                    'Complete current native session safety/input contract differs')
            driver.verify_regular(session, original_manifest)
            local = qualification.rows(session/'ledger.jsonl')
            require(len(local) == deck['count'] and {r['id'] for r in local} == {e['id'] for e in deck['scenes']}
                and all(r.get('status') == 'accepted_candidate'
                    and r.get('between_scene_park', {}).get('park_verified') is True for r in local),
                'Native child contains a hidden review/failure or lacks between-scene Park')
            events = [json.loads(line) for line in (session/'event_cmd_requests.jsonl').read_text().splitlines() if line.strip()]
            require(events and all(type(event.get('recorded_at')) in (int,float)
                and math.isfinite(event['recorded_at']) and event['recorded_at'] >= context['doc']['registered_at_unix']
                for event in events), 'Native events predate exact preregistration')
            sessions[session] = (manifest, deck, {r['id']:r for r in local})
        manifest, deck, local = sessions[session]
        require(ident in local and {k:v for k,v in row.items() if k != 'source_cell_id'} == local[ident],
            'Parent selected ledger differs from its actual unchanged child ledger')
        native = qualification.verify_accepted_native(manifest, entry, deck, local[ident])
        require(native['opt_in_attitude_evidence'] is not None, 'Current exact native must prove the opt-in attitude wrapper')
        meta, audit = read(root/'metadata.json'), read(root/'audit.json')
        effective = layout.resolve(context['overlay'], ident, entry, meta)['split']
        tokens = smoke.features(meta, effective, context['unit_assets'], context['deck']['config'])
        measured = verify_unit_corners(root, meta, audit, reference)
        physical.extend(dict(scene_id=ident, **item) for item in measured)
        real_compounds = set()
        for actor in meta['runtime_actors']:
            if actor['name'] not in smoke.COMPOSITES:continue
            components = [c for c in audit['components'] if c['parent_uid'] == actor['unique_id']]
            require(len(components) > 1, 'Compound coverage requires actual native child components')
            real_compounds.add('compound/'+actor['name'])
            compounds.append(dict(scene_id=ident, asset=actor['name'], parent_uid=actor['unique_id'],
                component_ids=[c['component_id'] for c in components],
                raw_class_counts=dict(Counter(c['raw_class'] for c in components)),
                native_corners_verified=True, raw_or_audit_modified=False))
        tokens -= {token for token in tokens if token.startswith('compound/') and token not in real_compounds}
        covered.update(tokens)
        observations.append(dict(id=ident, capture_root=str(root), scenario_sha256=row['scenario_sha256'],
            components=len(audit['components']), coverage=sorted(tokens), native_attitude_evidence=native['opt_in_attitude_evidence']))
    require({p.parent.resolve() for p in capture.rglob('session_contract.json')} == set(sessions),
        'Orphan native sessions cannot be omitted from exact qualification')
    require(set(context['doc']['coverage']['required']) <= covered, 'Actual native coverage is incomplete')
    require({r['asset'] for r in physical} == context['unit_assets'], 'Actual native corners must cover every unit QA19 asset')
    require({r['asset'] for r in compounds} == set(smoke.COMPOSITES), 'Every compound needs actual native component evidence')
    require(file_inventory(capture) == before, 'Native evidence changed during final qualification')
    pin_files(hashes, before)
    return dict(passed=True, requested=count, accepted=count, historical_observations_counted=0,
        bulk_training_observations=0, source_ledger=str(capture/'selected_ledger.jsonl'),
        observations=observations, covered=sorted(covered), physical_corner_observations=physical,
        compound_components=compounds, raw_inventory=before, evidence_hashes=hashes)


def save_partial_snapshot(manifest, capture, output):
    """정지된 실제 chunk 경계에서 저장한다. 수집·재개는 실행하지 않는다."""
    manifest, capture, output = Path(manifest).resolve(strict=True), Path(capture).resolve(strict=True), Path(output).resolve()
    require(not output.exists() and output != capture and capture not in output.parents, 'Fresh snapshot outside raw required')
    before = file_inventory(capture);state = read(capture/'status.json');guard = read(capture/driver.GUARD)
    rows = qualification.rows(capture/'selected_ledger.jsonl');count = len(read(manifest)['scenes'])
    require(state.get('stage') == 'paused' and guard.get('manifest_sha256') == sha256(manifest)
        and guard.get('runtime_hashes') == driver.runtime_hashes() and guard.get('runtime_profile') == driver.profiles()
        and type(guard.get('chunk_size')) is int and 1 <= guard['chunk_size'] <= 16
        and 0 < len(rows) < count <= 48 and len(rows) % guard['chunk_size'] == 0,
        'Paused exact pilot at a completed chunk boundary required')
    expected_ids = [e['id'] for e in read(manifest)['scenes'][:len(rows)]]
    require([row.get('source_cell_id') for row in rows] == expected_ids,
        'Partial approvals must be the registered prefix')
    require(all(row.get('status') == 'accepted_candidate'
        and read(Path(row['capture_root']).resolve().parents[1]/'shutdown_safety.json').get('park_verified') is True for row in rows),
        'Partial capture needs accepted originals and verified final Park')
    snap = completed_snapshot(capture)
    native_snapshots = {row['capture_root']:qualification.native_observation_snapshot(row['capture_root']) for row in rows}
    require(snap['raw_files'] and snap['event_logs'] and file_inventory(capture) == before,
        'Actual partial raw/event evidence must remain stable')
    atomic_json(output, dict(schema=PARTIAL_SCHEMA, helper_sha256=sha256(__file__), manifest_sha256=sha256(manifest),
        capture_root=str(capture), parent_contract_sha256=sha256(capture/driver.GUARD),
        driver_sha256=sha256(driver.__file__), runtime_profile=driver.profiles(), selected_rows=rows,
        preserved_selected=len(rows), before=snap, partial_raw_full_inventory=before,
        partial_native_snapshots=native_snapshots,
        capture_executed=False, resume_executed=False, bulk_ready=False))
    return output


def verify_partial(saved, manifest, capture, output=None):
    saved, manifest, capture = Path(saved).resolve(strict=True), Path(manifest).resolve(strict=True), Path(capture).resolve(strict=True)
    before = read(saved);current = completed_snapshot(capture)
    rows = qualification.rows(capture/'selected_ledger.jsonl');deck = read(manifest)
    count = before.get('preserved_selected');guard = read(capture/driver.GUARD)
    require(before.get('schema') == PARTIAL_SCHEMA and before.get('helper_sha256') == sha256(__file__)
        and before.get('manifest_sha256') == sha256(manifest) and before.get('capture_root') == str(capture)
        and before.get('driver_sha256') == sha256(driver.__file__)
        and before.get('runtime_profile') == driver.profiles()
        and before.get('parent_contract_sha256') == sha256(capture/driver.GUARD)
        and type(count) is int and 0 < count < len(deck['scenes']) <= 48
        and type(guard.get('chunk_size')) is int and 1 <= guard['chunk_size'] <= 16
        and count % guard['chunk_size'] == 0 and len(before.get('selected_rows', [])) == count,
        'Current exact partial snapshot/profile/chunk contract required')
    require([r.get('source_cell_id') for r in before['selected_rows']] == [e['id'] for e in deck['scenes'][:count]],
        'Partial snapshot is not the registered exact prefix')
    actual = {r['source_cell_id']:r for r in rows}
    require(len(actual) == len(rows) == len(deck['scenes']) and all(actual.get(r['source_cell_id']) == r for r in before['selected_rows']),
        'Resume lost or rewrote prior approved source slots')
    require(all(before['before'].get(key) and all(current[key].get(path) == record for path,record in before['before'][key].items())
        for key in ('raw_files','event_logs')) and current['cumulative_elapsed_sec'] >= before['before']['cumulative_elapsed_sec'],
        'Resume changed prior full raw/event inventory or cumulative timing')
    # 세션의 audit·계약·Park도 원래 bytes/inode/mtime를 유지해야 한다.
    now = file_inventory(capture)
    protected = {path:record for path,record in before['partial_raw_full_inventory'].items()
        if 'captures' in Path(path).relative_to(capture).parts
        or Path(path).name in ('session_contract.json','shutdown_safety.json','ledger.jsonl')}
    require(protected and all(now.get(path) == record for path,record in protected.items()),
        'Resume modified an existing native/session evidence file')
    native_snapshots = before.get('partial_native_snapshots', {})
    require(set(native_snapshots) == {row['capture_root'] for row in before['selected_rows']}
        and all(qualification.native_observation_snapshot(root) == records for root,records in native_snapshots.items()),
        'Resume changed an existing raw/session inventory, including new files')
    result = dict(schema=PARTIAL_RECEIPT, passed=True, helper_sha256=sha256(__file__),
        source_partial_sha256=sha256(saved), manifest_sha256=sha256(manifest), capture_root=str(capture),
        parent_contract_sha256=sha256(capture/driver.GUARD), preserved_selected=count,
        prior_raw_unchanged=True, before=before['before'], after=current,
        selected_after=len(rows), raw_event_and_selection_unchanged=True, resume_executed=False)
    if output is not None:
        output = Path(output).resolve()
        require(not output.exists() and output != capture and capture not in output.parents, 'Fresh resume receipt outside raw required')
        atomic_json(output, result)
        return output
    return result


def verify_completed(receipt, manifest, capture):
    document = read(receipt);capture = Path(capture).resolve(strict=True)
    current = completed_snapshot(capture);guard = read(capture/driver.GUARD)
    require(document.get('schema') == 'base5_static_attitude_batch_terminal_resume_v1'
        and document.get('passed') is True and document.get('driver_sha256') == sha256(driver.__file__)
        and document.get('runtime_profile') == driver.profiles()
        and document.get('manifest_sha256') == sha256(manifest) and document.get('capture_root') == str(capture)
        and document.get('requested') == len(read(manifest)['scenes'])
        and document.get('chunk_size') == guard['chunk_size']
        and document.get('parent_contract_sha256') == sha256(capture/driver.GUARD)
        and document.get('additional_native_captures') == 0 and document.get('new_subprocesses_permitted') is False
        and document.get('before') == document.get('after') == current,
        'Actual new-driver completed no-capture replay receipt required')
    return dict(passed=True, receipt_sha256=sha256(receipt), additional_native_captures=0,
        actual_new_driver_terminal_replay_verified=True)


def shadow_category(metadata, audit):
    """배치 subfamily를 개명하지 않고 실제 primary roster와 native 성분으로 분류한다."""
    actors = metadata.get('runtime_actors', [])
    require(actors, 'Actual shadow primary roster required')
    actor = actors[0]
    components = [c for c in audit.get('components', []) if c['parent_uid'] == actor['unique_id']]
    require(components and actor.get('motion') in ('walking','running'), 'Actual moving primary components required')
    if actor.get('population_pool') == 'animal' and actor.get('training_class') == 'animal':
        require(all(c.get('training_class') == 'animal' for c in components), 'Animal primary native semantics differ')
        return 'animal'
    if actor['name'] in smoke.COMPOSITES:
        require(len(components) > 1 and any(c.get('training_class') == 'pedestrian' for c in components),
            'Compound shadow needs actual pedestrian and child native components')
        return 'pedestrian_or_compound'
    require(actor.get('population_pool') in ('person','dummy','rider')
        and any(c.get('training_class') == 'pedestrian' for c in components),
        'Human shadow primary native semantics differ')
    return 'pedestrian_or_compound'


def direction_case(case, canonical, original, old, hashes):
    """새 adapter를 명시한 source에만 새 GUARD/recipe API를 적용한다."""
    module = importlib.import_module('run_base5_capture_campaign_direction_reference')
    require(callable(getattr(module, 'verify_completed_campaign', None)),
        'Completed direction-reference native verifier is unavailable; shadow gate remains closed')
    manifest = Path(case['campaign_manifest']).resolve(strict=True)
    deck = read(manifest);index = case.get('campaign_index')
    require(type(index) is int and 0 <= index < len(deck['scenes']), 'Explicit valid shadow campaign source index required')
    entry = deck['scenes'][index];meta = read(entry['metadata']);old_meta = read(old['metadata'])
    require(entry['id'] == canonical and entry['scenario_sha256'] == old['scenario_sha256']
        and planned_fields_match(old_meta, meta) and planned_fields_match(meta, old_meta)
        and deck['config'].get('environment_tag') == original['config'].get('environment_tag')
        and deck['config'].get(unit.KEY) == original['config'].get(unit.KEY),
        'Direction campaign source must exactly bind canonical original geometry/unit/spec/roster')
    proof = module.verify_completed_campaign(manifest, index, Path(case['campaign_root']).resolve(strict=True))
    require(proof.get('passed') is True and proof.get('source_id') == canonical
        and proof.get('native_qa_unchanged') is True, 'Actual new direction campaign completion required')
    pins.verify_hashes(proof['input_hashes'], 'Actual direction campaign evidence')
    for path,digest in proof['input_hashes'].items():qualification.add_pin(hashes, path, digest)
    qualification.add_pin(hashes, manifest);qualification.add_pin(hashes, module.__file__)
    return proof


def shadow_declared_protected_roots(document):
    """검증 결과를 쓰기 전에도 명시된 모든 음영 native/source 구역을 보호한다."""
    result = set()
    for raw in document.get('preserved_review_captures', []):
        root = Path(raw).resolve(strict=True);result.update((root, root.parents[1]))
    for case in document.get('cases', []):
        if case.get('adapter', 'legacy_explicit_reuse') == 'final_direction_reference':
            result.add(Path(case['campaign_root']).resolve(strict=True))
            result.add(Path(case['campaign_manifest']).resolve(strict=True).parent)
        else:
            result.add(Path(case['prepared_manifest']).resolve(strict=True).parent)
            for row in qualification.rows(case['selected_ledger']):
                root = Path(row['capture_root']).resolve(strict=True);result.update((root, root.parents[1]))
    require(result, 'Explicit preserved shadow native/source roots required')
    return result


def require_shadow_output_safe(output, roots):
    output = Path(output).resolve()
    for protected in roots:
        protected = Path(protected).resolve(strict=True)
        require(output != protected and protected not in output.parents and output not in protected.parents,
            'Release and planned capture output cannot overlap preserved shadow native/session/source')


def verify_shadow_preserved(result):
    inventories = result.get('native_inventories', {})
    require(inventories and result.get('protected_native_roots'), 'Actual shadow native inventory and protection scope required')
    require(all(file_inventory(root) == records for root,records in inventories.items()),
        'Shadow native/session/campaign inventory changed, including new files')
    pins.verify_hashes(result['evidence_hashes'], 'Preserved independent shadow evidence')


def verify_shadow(context, cases):
    """별도 지면 시험은 exact32나 full5K 촬영 수에 합산하지 않는다."""
    document = read(cases);receipt = read(context['assembly_receipt'])
    original_manifest = Path(receipt['source_manifest']);original = read(original_manifest)
    originals = {e['id']:e for e in original['scenes']};hashes = {};observations = [];categories = set();seen = set()
    qualification.add_pin(hashes, cases);qualification.add_pin(hashes, original_manifest, receipt['source_sha256'])
    require(document.get('schema') == SHADOW_SCHEMA and document.get('environment') == context['environment']
        and document.get('source_manifest_sha256') == sha256(original_manifest)
        and isinstance(document.get('cases'), list) and len(document['cases']) >= 2,
        'Explicit independent animal and pedestrian/compound shadow cases required')
    protected = shadow_declared_protected_roots(document)
    inventories = {str(root):file_inventory(root) for root in protected}
    reviews = document.get('preserved_review_captures', [])
    require(isinstance(reviews, list) and len(reviews) == len(set(reviews)),
        'Separate shadow failure/review originals must remain explicit and preserved')
    review_inventory = {}
    for raw in reviews:
        root = Path(raw).resolve(strict=True);before = qualification.native_observation_snapshot(root)
        audit = read(root/'audit.json');local = qualification.rows(root.parents[1]/'ledger.jsonl')
        require(audit.get('accepted_candidate') is False and audit.get('review_reasons')
            and any(Path(row['capture_root']).resolve() == root and row.get('status') == 'review'
                and row.get('audit') == audit and sha256(root/'scenario.json') == row.get('scenario_sha256') for row in local)
            and read(root.parents[1]/'shutdown_safety.json').get('park_verified') is True,
            'Actual immutable shadow review and final Park evidence required')
        review_inventory[str(root)] = before
        pin_files(hashes, before['raw']);pin_files(hashes, before['session_files'])
    sensor_sha = sha256(pins.ROOT/'configs/sensors/Sensor_competition_local.json')
    for case in document['cases']:
        ident = case['source_id'];require(ident in originals and ident not in seen, 'Unique shadow canonical source required');seen.add(ident)
        old, final = originals[ident], context['entries'][ident]
        old_meta, final_meta = read(old['metadata']), context['metadata'][ident]
        require(old['scenario_sha256'] == final['scenario_sha256']
            and planned_fields_match(old_meta, final_meta) and planned_fields_match(final_meta, old_meta),
            'Separate shadow evidence differs from final source beyond explicit common training overlay')
        adapter = case.get('adapter', 'legacy_explicit_reuse')
        if adapter == 'final_direction_reference':
            proof = direction_case(case, ident, original, old, hashes)
            prepared_manifest, selected_ledger = proof['prepared_manifest'], proof['selected_ledger']
        else:
            require(adapter == 'legacy_explicit_reuse', 'Unknown shadow evidence adapter')
            proof = shadow._selection(original, old, case, sensor_sha, hashes)
            prepared_manifest, selected_ledger = case['prepared_manifest'], case['selected_ledger']
        prepared = read(prepared_manifest);meta = read(prepared['scenes'][0]['metadata'])
        require(meta['spec']['family'] == 'shadow' and meta['spec'].get('shadow_moving') is True,
            'Independent shadow proof must retain actual moving targets')
        selected = [r for r in qualification.rows(selected_ledger)
            if r.get('scenario_sha256') == prepared['scenes'][0]['scenario_sha256']]
        require(len(selected) == 1 and selected[0]['status'] == 'accepted_candidate', 'Shadow review cannot qualify as success')
        audit = read(Path(selected[0]['capture_root'])/'audit.json')
        category = shadow_category(meta, audit);categories.add(category)
        require(audit.get('motion_capture', {}).get('required') is True
            and not audit['motion_capture'].get('review_reasons')
            and audit.get('motion_capture_actors') and all(not a.get('review_reasons') for a in audit['motion_capture_actors']),
            'Actual moving native/ground success required, not stationary preparation alone')
        raw = Path(selected[0]['capture_root']).resolve(strict=True)
        for root in (raw, raw.parents[1]):
            protected.add(root)
            inventories.setdefault(str(root), file_inventory(root))
        local = [row for row in qualification.rows(raw.parents[1]/'ledger.jsonl') if row['id'] == selected[0]['id']]
        require(len(local) == 1, 'Actual shadow native local ledger must identify one capture')
        native = qualification.verify_accepted_native(prepared_manifest, prepared['scenes'][0], prepared, local[0])
        pin_files(hashes, native['original_native_evidence']);pin_files(hashes, native['session_inventory'])
        observations.append(dict(source_id=ident, category=category, original_subfamily=meta['spec'].get('subfamily'), proof=proof,
            training_overlay_audited_separately=True, raw_approvals_added_to_exact_smoke=0))
    require(categories == {'animal','pedestrian_or_compound'}, 'Animal and pedestrian/compound shadow successes are both required')
    pins.verify_hashes(hashes)
    require(all(qualification.native_observation_snapshot(root) == records for root,records in review_inventory.items()),
        'Shadow review originals changed during qualification')
    result = dict(passed=True, observations=observations, evidence_hashes=hashes,
        preserved_review_inventory=review_inventory,
        protected_native_roots=sorted(str(root) for root in protected), native_inventories=inventories,
        materialized_bulk_source_scenes=0, bulk_source_remains_on_demand=True,
        declared_historical_reviews_preserved=True, exact_smoke_approvals_added=0)
    verify_shadow_preserved(result)
    return result


def qualify(arguments):
    """실제 파일을 다시 읽는 검증만 수행하며 결과를 쓰지 않는다."""
    context = qualification.registration_context(arguments['manifest'], arguments['smoke_registration'])
    capture = Path(arguments['capture_root']).resolve(strict=True);before = file_inventory(capture)
    runtime = cpu.runtime_hashes();require(all(runtime.get(p) == h for p,h in driver.runtime_hashes().items()),
        'Complete CPU runtime must include the new batch driver closure')
    native = verify_native(context, capture)
    shadow_result = verify_shadow(context, arguments['shadow_cases'])
    inputs = common.verify_inputs(arguments['training_input_report'], arguments['loader_report'], capture, native['accepted'])
    partial = verify_partial(arguments['partial_snapshot'], context['subset_path'], capture)
    require(read(arguments['partial_resume_receipt']) == partial, 'Partial receipt differs from actual preservation recomputation')
    completed = verify_completed(arguments['completed_resume_receipt'], context['subset_path'], capture)
    tests = common.verify_cpu_regression(arguments['cpu_regression_receipt'], arguments['pytest_xml'], runtime)
    results = dict(native_exact=native, independent_shadow=shadow_result, training_inputs=dict(passed=True, **inputs),
        cpu_regression=dict(passed=True, test_results=tests), partial_resume=partial, completed_resume=completed)
    hashes = dict(context['input_hashes']);hashes.update(runtime)
    hashes.update(native['evidence_hashes']);hashes.update(shadow_result['evidence_hashes']);hashes.update(inputs['evidence_hashes'])
    for key,value in arguments.items():
        if key in ('capture_root','capture_output'):continue
        for path in value if isinstance(value,list) else [value]:qualification.add_pin(hashes, path)
    qualification.add_pin(hashes, __file__)
    pins.verify_hashes(hashes);pins.verify_hashes(runtime)
    require(file_inventory(capture) == before, 'Native evidence changed during release verification')
    verify_shadow_preserved(shadow_result)
    return dict(context=context, results=results, input_hashes=hashes, runtime_hashes=runtime)


def freeze(arguments, output):
    output = Path(output).resolve();manifest = Path(arguments['manifest']).resolve(strict=True)
    capture = Path(arguments['capture_root']).resolve(strict=True);planned = Path(arguments['capture_output']).resolve()
    receipt = read(read(arguments['smoke_registration'])['assembly_receipt']['path'])
    require(not output.exists(), 'Fresh separate release output required')
    qualification.require_safe_registration_output(output, manifest, receipt)
    runtime = driver.attitude.candidate.gen.geo.DEFAULT_RUNTIME_ASSET_DIR.resolve()
    require(output != runtime and runtime not in output.parents and planned != runtime and runtime not in planned.parents,
        'Release and future capture outputs must remain outside simulator runtime')
    for protected in (capture, planned):
        require(output != protected and protected not in output.parents and output not in protected.parents,
            'Release output cannot overlap native or future bulk output')
    require(not planned.exists() and planned != capture and capture not in planned.parents,
        'Bulk must use a fresh separate output, not the native pilot')
    qualification.require_safe_registration_output(planned, manifest, receipt)
    declared_shadow_roots = shadow_declared_protected_roots(read(arguments['shadow_cases']))
    for proposed in (output, planned):require_shadow_output_safe(proposed, declared_shadow_roots)
    verified = qualify(arguments);context = verified['context']
    shadow_result = verified['results']['independent_shadow']
    for proposed in (output, planned):require_shadow_output_safe(proposed, shadow_result['protected_native_roots'])
    audit = scenario_audit.audit(manifest)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000, 'Current full5K CPU audit failed')
    verify_shadow_preserved(shadow_result)
    output.mkdir(parents=True, exist_ok=False);gates = {}
    for name,result in verified['results'].items():
        path = output/(name+'.json');atomic_json(path, result)
        gates[name] = dict(passed=True, path=str(path), sha256=sha256(path))
    atomic_json(output/'offline_audit.json', audit)
    ancestor = planned
    while not ancestor.exists():ancestor = ancestor.parent
    free = shutil.disk_usage(ancestor).free/2**30
    release = dict(schema=SCHEMA, qualification_schema=QUALIFICATION_SCHEMA, release_policy=POLICY,
        generated_at_utc=datetime.now(timezone.utc).isoformat(), helper_sha256=sha256(__file__),
        bulk_ready=True, bulk_started=False, training_started=False, completion_live_qualified=False,
        qualified_environments=[context['environment']], environment=context['environment'],
        full_manifest=str(manifest), full_manifest_sha256=sha256(manifest), capture_output=str(planned),
        native_exact_observations=verified['results']['native_exact']['accepted'], bulk_training_observations=0,
        qualified_runtime=dict(profile=PROFILE, entrypoints=[str(Path(driver.__file__).resolve()), str(driver.CHILD), str(driver.RECOVERY)],
            runtime_profile=driver.profiles()), runtime_hashes=verified['runtime_hashes'], input_hashes=verified['input_hashes'],
        qualification_inputs=arguments, gate_evidence=gates,
        full_cpu_audit=dict(path=str(output/'offline_audit.json'), sha256=sha256(output/'offline_audit.json')),
        environments={context['environment']:dict(manifest=str(manifest), manifest_sha256=sha256(manifest), capture_output=str(planned))},
        operational_storage_budget_gib=45., safety_headroom_gib=10., observed_free_gib=free,
        start_blockers=[] if free >= 55 else ['insufficient_storage_headroom_55GiB'],
        limitations=['32..48 exact native approvals are not 5000 collected training frames.',
            '24 source shadow paths remain on-demand; separate trials are not materialized source approvals.',
            'CPU/Q90/native qualification is not detector AP, GPU latency or competition full-stack certification.',
            'Disk, explicit environment, run and bulk approval are checked again by the batch driver.'])
    pins.verify_hashes(verified['input_hashes']);verify_shadow_preserved(shadow_result)
    atomic_json(output/'release.json', release)
    verify_shadow_preserved(shadow_result)
    return output/'release.json'


def qualified_release_context(release, environment, manifest):
    """batch의 bulk 진입 시 각 실제 gate와 현재 파일을 다시 대조한다."""
    release = Path(release).resolve(strict=True);document = read(release);manifest = Path(manifest).resolve(strict=True)
    require(document.get('schema') == SCHEMA and document.get('qualification_schema') == QUALIFICATION_SCHEMA
        and document.get('release_policy') == POLICY and document.get('helper_sha256') == sha256(__file__)
        and document.get('bulk_ready') is True and document.get('bulk_started') is False
        and document.get('training_started') is False and document.get('completion_live_qualified') is False,
        'Current separately qualified environment release required')
    require(environment in qualification.ENVIRONMENTS and document.get('qualified_environments') == [environment]
        and document.get('environment') == environment and document.get('full_manifest') == str(manifest)
        and document.get('full_manifest_sha256') == sha256(manifest)
        and document.get('qualification_inputs', {}).get('manifest') == str(manifest),
        'Qualified source/environment binding differs')
    require(document.get('capture_output') == document['qualification_inputs'].get('capture_output')
        and document.get('environments') == {environment:dict(manifest=str(manifest),
            manifest_sha256=sha256(manifest), capture_output=document['capture_output'])},
        'Qualified future output/environment binding differs')
    profile = document.get('qualified_runtime', {})
    require(profile.get('profile') == PROFILE and profile.get('runtime_profile') == driver.profiles()
        and profile.get('entrypoints') == [str(Path(driver.__file__).resolve()), str(driver.CHILD), str(driver.RECOVERY)],
        'Qualified regular/recovery runtime profiles differ')
    pins.verify_hashes(document['runtime_hashes']);pins.verify_hashes(document['input_hashes'])
    require(document['runtime_hashes'] == cpu.runtime_hashes(), 'Qualified complete runtime changed')
    verified = qualify(document['qualification_inputs'])
    require(document['input_hashes'] == verified['input_hashes'], 'Qualified input inventory changed')
    planned = Path(document['capture_output'])
    require(planned.is_absolute() and planned.resolve() == planned
        and planned != Path(document['qualification_inputs']['capture_root']).resolve()
        and Path(document['qualification_inputs']['capture_root']).resolve() not in planned.parents,
        'Qualified bulk output must remain separate from actual native evidence')
    qualification.require_safe_registration_output(planned, manifest,
        read(verified['context']['assembly_receipt']))
    require_shadow_output_safe(planned, verified['results']['independent_shadow']['protected_native_roots'])
    require(document.get('native_exact_observations') == verified['results']['native_exact']['accepted']
        and document.get('bulk_training_observations') == 0, 'Exact pilot/bulk denominators differ')
    require(set(document.get('gate_evidence', {})) == set(GATES), 'All six actual qualification gates required')
    for name,result in verified['results'].items():
        gate = document['gate_evidence'][name]
        require(gate.get('passed') is True and sha256(gate['path']) == gate['sha256'] and read(gate['path']) == result,
            'Qualified gate differs from actual evidence: '+name)
    audit = document['full_cpu_audit']
    require(sha256(audit['path']) == audit['sha256'] and read(audit['path']).get('status') == 'OFFLINE_PASS',
        'Qualified full CPU audit changed')
    return dict(passed=True, bulk_ready=True, environment=environment, manifest_sha256=sha256(manifest),
        capture_output=document['capture_output'], runtime_hashes=document['runtime_hashes'], input_hashes=document['input_hashes'],
        runtime_profile=driver.profiles(), native_exact_observations=document['native_exact_observations'],
        bulk_training_observations=0, gate_evidence=document['gate_evidence'])


def main():
    parser = argparse.ArgumentParser(description=__doc__);modes = parser.add_subparsers(dest='mode', required=True)
    partial = modes.add_parser('partial-snapshot')
    partial_verify = modes.add_parser('verify-partial')
    for command in (partial, partial_verify):
        for name in ('manifest','capture-root','output'):command.add_argument('--'+name, type=Path, required=True)
    partial_verify.add_argument('--snapshot', type=Path, required=True)
    final = modes.add_parser('freeze')
    for name in ('manifest','smoke-registration','capture-root','shadow-cases','partial-snapshot','partial-resume-receipt',
                 'completed-resume-receipt','training-input-report','loader-report','cpu-regression-receipt','capture-output','output'):
        final.add_argument('--'+name, type=Path, required=True)
    final.add_argument('--pytest-xml', type=Path, action='append', required=True)
    args = parser.parse_args()
    if args.mode == 'partial-snapshot':
        print(save_partial_snapshot(args.manifest, args.capture_root, args.output));return
    if args.mode == 'verify-partial':
        print(verify_partial(args.snapshot, args.manifest, args.capture_root, args.output));return
    inputs = {k:([str(p.resolve(strict=True)) for p in v] if isinstance(v,list) else str(v.resolve()))
        for k,v in vars(args).items() if k not in ('mode','output')}
    print(freeze(inputs, args.output))


if __name__ == '__main__':main()
