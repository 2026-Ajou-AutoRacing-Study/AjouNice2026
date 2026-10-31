#!/usr/bin/env python3
"""새로 생성한 환경의 unit 사전 등록과 실제 6개 자격 검사를 연결한다.

과거 native 대체가 없는 fresh source는 원형 unit 등록 검증기를 사용한다.
assembled-source receipt나 실패 장면을 만들어 내지 않는다. 원형 native,
shadow 방향/지면/성분, Q90/loader, CPU, 두 resume 검사는 그대로 수행한다.
이 도구는 CPU 검증과 새 release 작성만 하며 MORAI·수집을 실행하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import threading

SCRIPTS = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import finalize_base5_environment_release as original
import prepare_base5_unit_environment_smoke as unit_smoke

SCHEMA = original.SCHEMA
QUALIFICATION_SCHEMA = 'base5_fresh_environment_release_qualification_v1'
POLICY = 'fresh-unit-source-exact-native-six-gate-qualified-v1'
GATES = original.GATES
read, require, sha = original.read, original.require, original.sha256
_LOCK = threading.Lock()


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
        'Exact nonsymlink fresh qualification path required')
    return path


def adapter_pin():
    return dict(path=str(physical(__file__)), sha256=sha(__file__), registration_schema=unit_smoke.SCHEMA)


def registration_context(manifest, registration):
    """정식 fresh unit 등록만 받는다. 역사적 대체·승인 계보를 합성하지 않는다."""
    manifest, registration = physical(manifest), physical(registration)
    context = unit_smoke.registration_context(manifest, registration)
    receipt = read(context['build_receipt'])
    require(receipt.get('final_manifest') == str(manifest)
        and receipt.get('final_manifest_sha256') == sha(manifest)
        and context['doc'].get('schema') == unit_smoke.SCHEMA
        and context['environment'] == context['deck']['config']['environment_tag']
        and context['doc'].get('source_manifest') == str(manifest)
        and context['doc'].get('source_manifest_sha256') == sha(manifest)
        and context['deck'].get('count') == len(context['deck']['scenes']) == 5000,
        'Exact fresh source/build/unit-registration binding required')
    require('assembly_receipt' not in context and 'assembly_receipt' not in context['doc'],
        'Fresh qualification cannot masquerade as historical native replacement assembly')
    context['full'] = manifest
    context['fresh_source_adapter'] = adapter_pin()
    return context


def require_safe_output(output, context, *, protected=()):
    """source·등록·지도/runtime·원본 native를 포함하거나 그 안에 쓰지 않는다."""
    output = physical(output)
    roots = {physical(context['full']).parent, physical(context['registration']).parent,
        physical(original.driver.attitude.candidate.gen.geo.DEFAULT_RUNTIME_ASSET_DIR)}
    roots.update(physical(path) for path in protected)
    for root in roots:
        require(output != root and root not in output.parents and output not in root.parents,
            'Fresh output cannot overlap preserved source/registration/native/runtime roots')
    for filename in context['input_hashes']:
        path = physical(filename)
        require(output != path and output not in path.parents,
            'Fresh output cannot contain or replace any pinned input file')
    return output


def verify_shadow(context, cases):
    """원형 shadow 검사를 유지하되 비교 원본은 실제 새 full source 자체다."""
    document = read(cases)
    original_manifest = physical(context['full']);deck = read(original_manifest)
    require(deck == context['deck'], 'Fresh full source changed before independent shadow verification')
    originals = {entry['id']:entry for entry in deck['scenes']}
    hashes, observations, categories, seen = {}, [], set(), set()
    original.qualification.add_pin(hashes, cases)
    original.qualification.add_pin(hashes, original_manifest)
    require(document.get('schema') == original.SHADOW_SCHEMA
        and document.get('environment') == context['environment']
        and document.get('source_manifest_sha256') == sha(original_manifest)
        and isinstance(document.get('cases'), list) and len(document['cases']) >= 2,
        'Explicit independent animal and pedestrian/compound shadow cases required')
    protected = original.shadow_declared_protected_roots(document)
    inventories = {str(root):original.file_inventory(root) for root in protected}
    reviews = document.get('preserved_review_captures', [])
    require(isinstance(reviews, list) and len(reviews) == len(set(reviews)),
        'Separate shadow failure/review originals must remain explicit and preserved')
    review_inventory = {}
    for raw in reviews:
        root = physical(raw);before = original.qualification.native_observation_snapshot(root)
        audit = read(root/'audit.json');local = original.qualification.rows(root.parents[1]/'ledger.jsonl')
        require(audit.get('accepted_candidate') is False and audit.get('review_reasons')
            and any(physical(row['capture_root']) == root and row.get('status') == 'review'
                and row.get('audit') == audit and sha(root/'scenario.json') == row.get('scenario_sha256') for row in local)
            and read(root.parents[1]/'shutdown_safety.json').get('park_verified') is True,
            'Actual immutable shadow review and final Park evidence required')
        review_inventory[str(root)] = before
        original.pin_files(hashes, before['raw']);original.pin_files(hashes, before['session_files'])
    sensor_sha = sha(original.pins.ROOT/'configs/sensors/Sensor_competition_local.json')
    for case in document['cases']:
        ident = case['source_id']
        require(ident in originals and ident not in seen, 'Unique shadow canonical source required')
        seen.add(ident)
        old, final = originals[ident], context['entries'][ident]
        old_meta, final_meta = read(old['metadata']), context['metadata'][ident]
        require(old['scenario_sha256'] == final['scenario_sha256']
            and original.planned_fields_match(old_meta, final_meta)
            and original.planned_fields_match(final_meta, old_meta),
            'Separate shadow evidence differs from exact fresh source geometry/spec/roster')
        adapter = case.get('adapter', 'legacy_explicit_reuse')
        if adapter == 'final_direction_reference':
            proof = original.direction_case(case, ident, deck, old, hashes)
            prepared_manifest, selected_ledger = proof['prepared_manifest'], proof['selected_ledger']
        else:
            require(adapter == 'legacy_explicit_reuse', 'Unknown shadow evidence adapter')
            proof = original.shadow._selection(deck, old, case, sensor_sha, hashes)
            prepared_manifest, selected_ledger = case['prepared_manifest'], case['selected_ledger']
        prepared = read(prepared_manifest);meta = read(prepared['scenes'][0]['metadata'])
        require(meta['spec']['family'] == 'shadow' and meta['spec'].get('shadow_moving') is True,
            'Independent shadow proof must retain actual moving targets')
        selected = [row for row in original.qualification.rows(selected_ledger)
            if row.get('scenario_sha256') == prepared['scenes'][0]['scenario_sha256']]
        require(len(selected) == 1 and selected[0]['status'] == 'accepted_candidate',
            'Shadow review cannot qualify as success')
        audit = read(physical(selected[0]['capture_root'])/'audit.json')
        category = original.shadow_category(meta, audit);categories.add(category)
        require(audit.get('motion_capture', {}).get('required') is True
            and not audit['motion_capture'].get('review_reasons')
            and audit.get('motion_capture_actors')
            and all(not actor.get('review_reasons') for actor in audit['motion_capture_actors']),
            'Actual moving native/ground success required, not stationary preparation alone')
        raw = physical(selected[0]['capture_root'])
        for root in (raw, raw.parents[1]):
            protected.add(root);inventories.setdefault(str(root), original.file_inventory(root))
        local = [row for row in original.qualification.rows(raw.parents[1]/'ledger.jsonl')
            if row['id'] == selected[0]['id']]
        require(len(local) == 1, 'Actual shadow native local ledger must identify one capture')
        native = original.qualification.verify_accepted_native(prepared_manifest,
            prepared['scenes'][0], prepared, local[0])
        original.pin_files(hashes, native['original_native_evidence'])
        original.pin_files(hashes, native['session_inventory'])
        observations.append(dict(source_id=ident, category=category,
            original_subfamily=meta['spec'].get('subfamily'), proof=proof,
            training_overlay_audited_separately=True, raw_approvals_added_to_exact_smoke=0))
    require(categories == {'animal','pedestrian_or_compound'},
        'Animal and pedestrian/compound shadow successes are both required')
    original.pins.verify_hashes(hashes)
    require(all(original.qualification.native_observation_snapshot(root) == records
        for root, records in review_inventory.items()), 'Shadow review originals changed during qualification')
    result = dict(passed=True, observations=observations, evidence_hashes=hashes,
        preserved_review_inventory=review_inventory,
        protected_native_roots=sorted(str(root) for root in protected), native_inventories=inventories,
        materialized_bulk_source_scenes=0, bulk_source_remains_on_demand=True,
        declared_historical_reviews_preserved=True, exact_smoke_approvals_added=0)
    original.verify_shadow_preserved(result)
    return result


@contextmanager
def fresh_source_scope():
    """원형 6개 검사 호출은 재사용하고 source context만 명시적으로 바꾼다."""
    require(_LOCK.acquire(blocking=False), 'Concurrent or nested fresh qualification is forbidden')
    previous_registration = original.qualification.registration_context
    previous_shadow = original.verify_shadow
    original.qualification.registration_context = registration_context
    original.verify_shadow = verify_shadow
    try:
        yield
    finally:
        changed = (original.qualification.registration_context is not registration_context
            or original.verify_shadow is not verify_shadow)
        original.qualification.registration_context = previous_registration
        original.verify_shadow = previous_shadow
        _LOCK.release()
        require(not changed, 'Fresh qualification adapter ownership changed')


def qualify(arguments):
    with fresh_source_scope():
        result = original.qualify(arguments)
    require(set(result['results']) == set(GATES)
        and all(gate.get('passed') is True for gate in result['results'].values()),
        'All six original native/shadow/input/CPU/resume gates must pass')
    context = result['context'];registration = context['doc']
    build = read(context['build_receipt'])
    # The common launcher may reuse this completed deep verification only
    # after hashing EVERY byte it depended on at the next session boundary.
    for group in (context['input_hashes'], registration['input_hashes'], registration['runtime_hashes'],
            build['original_input_hashes'], build['runtime_hashes']):
        for path,digest in group.items():original.qualification.add_pin(result['input_hashes'], path, digest)
    for path,digest in ((context['registration'], None),
            (context['subset_path'], registration['subset_manifest_sha256']),
            (Path(context['registration']).parent/'source_audit.json', registration['source_audit_sha256']),
            (context['build_receipt'], registration['build_receipt']['sha256'])):
        original.qualification.add_pin(result['input_hashes'], path, digest)
    original.qualification.add_pin(result['input_hashes'], __file__)
    original.pins.verify_hashes(result['input_hashes'])
    return result


def freeze(arguments, output):
    manifest = physical(arguments['manifest']);capture = physical(arguments['capture_root'])
    planned = physical(arguments['capture_output']);output = physical(output)
    require(not output.exists() and not planned.exists(), 'Fresh release and future bulk outputs required')
    context = registration_context(manifest, arguments['smoke_registration'])
    for target in (output, planned):require_safe_output(target, context, protected=(capture,))
    require(output != planned and planned not in output.parents and output not in planned.parents,
        'Release and future bulk outputs must be separate')
    declared = original.shadow_declared_protected_roots(read(arguments['shadow_cases']))
    for target in (output, planned):original.require_shadow_output_safe(target, declared)
    verified = qualify(arguments);context = verified['context']
    shadow_result = verified['results']['independent_shadow']
    for target in (output, planned):
        original.require_shadow_output_safe(target, shadow_result['protected_native_roots'])
    audit = original.scenario_audit.audit(manifest)
    require(audit.get('status') == 'OFFLINE_PASS' and not audit.get('errors')
        and audit.get('scenes') == audit.get('unique_payloads') == 5000, 'Current full5K CPU audit failed')
    original.verify_shadow_preserved(shadow_result)
    output.mkdir(parents=True, exist_ok=False);gates = {}
    for name, result in verified['results'].items():
        path = output/(name+'.json');original.atomic_json(path, result)
        gates[name] = dict(passed=True, path=str(path), sha256=sha(path))
    original.atomic_json(output/'offline_audit.json', audit)
    ancestor = planned
    while not ancestor.exists():ancestor = ancestor.parent
    free = shutil.disk_usage(ancestor).free/2**30
    environment = context['environment']
    release = dict(schema=SCHEMA, qualification_schema=QUALIFICATION_SCHEMA, release_policy=POLICY,
        generated_at_utc=datetime.now(timezone.utc).isoformat(), helper_sha256=sha(original.__file__),
        fresh_source_adapter=adapter_pin(), bulk_ready=True, bulk_started=False, training_started=False,
        completion_live_qualified=False, qualified_environments=[environment], environment=environment,
        full_manifest=str(manifest), full_manifest_sha256=sha(manifest), capture_output=str(planned),
        native_exact_observations=verified['results']['native_exact']['accepted'], bulk_training_observations=0,
        qualified_runtime=dict(profile=original.PROFILE,
            entrypoints=[str(Path(original.driver.__file__).resolve()), str(original.driver.CHILD), str(original.driver.RECOVERY)],
            runtime_profile=original.driver.profiles()), runtime_hashes=verified['runtime_hashes'],
        input_hashes=verified['input_hashes'], qualification_inputs=arguments, gate_evidence=gates,
        full_cpu_audit=dict(path=str(output/'offline_audit.json'), sha256=sha(output/'offline_audit.json')),
        environments={environment:dict(manifest=str(manifest),manifest_sha256=sha(manifest),capture_output=str(planned))},
        operational_storage_budget_gib=45., safety_headroom_gib=10., observed_free_gib=free,
        start_blockers=[] if free >= 55 else ['insufficient_storage_headroom_55GiB'],
        limitations=['32..48 exact pilot observations are not 5000 training observations.',
            'Fresh unit preregistration; no historical native replacement assembly is claimed.',
            'Independent shadow ground/native, input, CPU and both resume gates are unchanged.',
            'Common runtime requires its own bounded pause/resume qualification before operator bulk start.'])
    original.pins.verify_hashes(verified['input_hashes']);original.verify_shadow_preserved(shadow_result)
    original.atomic_json(output/'release.json', release)
    original.verify_shadow_preserved(shadow_result)
    return output/'release.json'


def qualified_release_context(release, environment, manifest):
    release, manifest = physical(release), physical(manifest);document = read(release)
    require(document.get('schema') == SCHEMA and document.get('qualification_schema') == QUALIFICATION_SCHEMA
        and document.get('release_policy') == POLICY and document.get('helper_sha256') == sha(original.__file__)
        and document.get('fresh_source_adapter') == adapter_pin()
        and document.get('bulk_ready') is True and document.get('bulk_started') is False
        and document.get('training_started') is False and document.get('completion_live_qualified') is False,
        'Current explicitly qualified fresh environment release required')
    require(environment in unit_smoke.ENVIRONMENTS and document.get('qualified_environments') == [environment]
        and document.get('environment') == environment and document.get('full_manifest') == str(manifest)
        and document.get('full_manifest_sha256') == sha(manifest)
        and document.get('qualification_inputs', {}).get('manifest') == str(manifest),
        'Fresh qualified source/environment binding differs')
    planned = physical(document['capture_output'])
    require(document['qualification_inputs'].get('capture_output') == str(planned)
        and document.get('environments') == {environment:dict(manifest=str(manifest),
            manifest_sha256=sha(manifest),capture_output=str(planned))},
        'Fresh qualified future output/environment binding differs')
    profile = document.get('qualified_runtime', {})
    require(profile.get('profile') == original.PROFILE and profile.get('runtime_profile') == original.driver.profiles()
        and profile.get('entrypoints') == [str(Path(original.driver.__file__).resolve()),
            str(original.driver.CHILD),str(original.driver.RECOVERY)], 'Fresh regular/recovery runtime profiles differ')
    original.pins.verify_hashes(document['runtime_hashes']);original.pins.verify_hashes(document['input_hashes'])
    require(document['runtime_hashes'] == original.cpu.runtime_hashes(), 'Fresh complete runtime changed')
    require(document['input_hashes'].get(str(physical(__file__))) == sha(__file__), 'Fresh adapter input pin missing')
    verified = qualify(document['qualification_inputs'])
    require(document['input_hashes'] == verified['input_hashes'], 'Fresh qualified input inventory changed')
    require_safe_output(planned, verified['context'], protected=(document['qualification_inputs']['capture_root'],release.parent))
    original.require_shadow_output_safe(planned, verified['results']['independent_shadow']['protected_native_roots'])
    require(type(document.get('native_exact_observations')) is int
        and 32 <= document['native_exact_observations'] <= 48
        and document['native_exact_observations'] == verified['results']['native_exact']['accepted']
        and document.get('bulk_training_observations') == 0, 'Fresh exact pilot/bulk denominators differ')
    require(set(document.get('gate_evidence', {})) == set(GATES), 'All six actual fresh qualification gates required')
    for name, result in verified['results'].items():
        gate = document['gate_evidence'][name]
        require(gate.get('passed') is True and sha(gate['path']) == gate['sha256'] and read(gate['path']) == result,
            'Fresh qualified gate differs from actual evidence: '+name)
    audit = document['full_cpu_audit']
    require(sha(audit['path']) == audit['sha256'] and read(audit['path']).get('status') == 'OFFLINE_PASS',
        'Fresh qualified full CPU audit changed')
    return dict(passed=True, bulk_ready=True, environment=environment, manifest_sha256=sha(manifest),
        capture_output=str(planned), runtime_hashes=document['runtime_hashes'], input_hashes=document['input_hashes'],
        runtime_profile=original.driver.profiles(),native_exact_observations=document['native_exact_observations'],
        bulk_training_observations=0,gate_evidence=document['gate_evidence'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    modes = parser.add_subparsers(dest='mode',required=True)
    partial = modes.add_parser('partial-snapshot');resume = modes.add_parser('verify-partial')
    for command in (partial,resume):
        for name in ('manifest','capture-root','output'):command.add_argument('--'+name,type=Path,required=True)
    resume.add_argument('--snapshot',type=Path,required=True)
    final = modes.add_parser('freeze')
    for name in ('manifest','smoke-registration','capture-root','shadow-cases','partial-snapshot',
        'partial-resume-receipt','completed-resume-receipt','training-input-report','loader-report',
        'cpu-regression-receipt','capture-output','output'):
        final.add_argument('--'+name,type=Path,required=True)
    final.add_argument('--pytest-xml',type=Path,action='append',required=True)
    args = parser.parse_args(argv)
    if args.mode == 'partial-snapshot':
        print(original.save_partial_snapshot(args.manifest,args.capture_root,args.output));return 0
    if args.mode == 'verify-partial':
        print(original.verify_partial(args.snapshot,args.manifest,args.capture_root,args.output));return 0
    arguments = {key:([str(physical(path)) for path in value] if isinstance(value,list) else str(physical(value)))
        for key,value in vars(args).items() if key not in ('mode','output')}
    print(freeze(arguments,args.output));return 0


if __name__ == '__main__':raise SystemExit(main())
