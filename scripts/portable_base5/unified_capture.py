#!/usr/bin/env python3
"""환경과 무관한 Base5 수집 실행기. 기본은 계획, 본수집은 사용자가 시작한다.

과거 세션을 수정하지 않고 승인·부족분을 새 출력으로 인계한다. 고정 입력은
시작/재개/종료 경계에서 검산하며 새 자료와 native QA는 원형대로 검사한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import ExitStack, contextmanager
import copy
import fcntl
import hashlib
import importlib
import json
import os
from pathlib import Path
import runpy
import stat
import subprocess
import time

import recovery_population
import session_input_boundary as boundary_module
import unified_paused_import as handoff


SCHEMA = 'base5_unified_capture_v1'
POLICY = 'fixed-input-boundaries-and-common-native-profiles-v1'
FRESH_AUTHORITY_SCHEMA = 'base5_unified_fresh_source_authority_v1'
FRESH_ORIGIN_SCHEMA = 'base5_unified_fresh_empty_origin_v1'
SCRIPTS = Path('/home/luv/AjouNice2026/dataset/morai_od/scripts')
TARGETS = {'run_base5_capture_static_attitude.py',
           'run_base5_capture_campaign_direction_reference.py',
           'run_base5_capture.py'}
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
SEALS = (fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL)


def require(value, message):
    if not value:
        raise RuntimeError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink path required: ' + str(path))
    return path


def sha(path):
    h = hashlib.sha256()
    with physical(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(physical(path).read_text())


def write_new(path, document):
    filename = physical(path)
    fd = os.open(filename, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def inside(path, root):
    return path == root or root in path.parents


def modules():
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    parent = importlib.import_module('run_base5_batch_static_attitude')
    direction = importlib.import_module('run_base5_capture_campaign_direction_reference')
    return parent, direction


def helpers(*, fresh_start=False):
    paths = [Path(__file__).absolute(), Path(boundary_module.__file__).absolute(),
             Path(handoff.__file__).absolute(), Path(handoff.historical.__file__).absolute(),
             Path(recovery_population.__file__).absolute()]
    if fresh_start:
        for name in ('fresh_route_compat', 'fresh_environment_qualification'):
            paths.append(Path(__file__).absolute().with_name(name+'.py'))
    return {str(physical(p)): sha(p) for p in paths}


def profile_helpers(doc):
    return helpers(fresh_start=True) if doc.get('fresh_start') is True else helpers()


def merge(*groups):
    result = {}
    for group in groups:
        for path, digest in group.items():
            require(path not in result or result[path] == digest, 'Conflicting fixed pin: ' + path)
            result[path] = digest
    return dict(sorted(result.items()))


def source_only(pins, forbidden):
    """원본 시나리오/코드만 재사용한다. native 증거/출력은 매번 실제 검산한다."""
    result = {}
    for filename, digest in pins.items():
        p = physical(filename)
        require(not any(inside(p, root) for root in forbidden), 'Output cannot be a fixed input')
        if p.suffix.lower() not in {'.json', '.py', '.yaml', '.yml', '.csv', '.scn', '.sh'}:
            continue
        if p.suffix != '.py' and any('native' in part.lower() or 'training_inputs' in part.lower()
                for part in p.parent.parts):
            continue
        if any(part.lower() in {'capture', 'captures', 'raw', 'scene_parks'}
                or part.upper().startswith(('LIDAR_', 'CAMERA_')) for part in p.parts):
            continue
        name = p.name.lower()
        if name in {'audit.json', 'status.json', 'report.json', 'shutdown_safety.json',
                    'session_contract.json', 'quota.json', 'batch_contract.json'} \
                or name.startswith(('audit', 'ledger', 'selected', 'receipt', 'event_cmd')) \
                or 'receipt' in name or 'snapshot' in name or name.endswith(('_audit.json', '_proof.json')):
            continue
        result[filename] = digest
    require(result, 'Nonempty fixed source pin inventory required')
    return result


@contextmanager
def route_compatibility(doc):
    """Python 3.8/3.10 차이는 기존 승인된 누적거리 witness만 재사용한다."""
    witness = doc.get('route_witness')
    if not witness:
        yield
        return
    route_compat = importlib.import_module('fresh_route_compat' if doc.get('fresh_start') is True else 'route_compat')
    gen = importlib.import_module('generate_base5_independent')
    extent = importlib.import_module('base5_static_mission_extent')
    with route_compat.installed(witness['path'], generator=gen, extent=extent,
            manifest=doc.get('canonical_manifest', doc['manifest']), expected_witness_sha256=witness['sha256']):
        yield


def fresh_release_verifier(value):
    """새 원형과 실제 대체를 반영한 source 모두 기존 검증기를 그대로 쓴다."""
    name = ('finalize_base5_environment_release' if value.get('qualification_schema') ==
        'base5_new_environment_release_qualification_v1' else 'fresh_environment_qualification')
    return importlib.import_module(name)


def source_authority(doc, parent):
    """구 release는 원래 source의 자격만 입증한다. 새 실행기 자격과 혼동하지 않는다."""
    release = physical(doc['release'])
    require(sha(release) == doc['release_sha256'] and sha(doc['manifest']) == doc['manifest_sha256'],
            'Frozen source/release changed')
    value = read(release)
    fresh = doc.get('fresh_start') is True
    if fresh:
        validate_fresh_release(doc, value)
    saved = doc.get('source_authority')
    if saved is not None:
        # The actual original verifier ran during prepare. Reuse its result
        # only after the boundary has checked EVERY original evidence byte,
        # including historical native files and bags (never hot-cached).
        required = merge(value['input_hashes'], value['runtime_hashes'])
        if fresh:
            audit = value['full_cpu_audit']
            required = merge(required, {audit['path']:audit['sha256']})
        for gate in value.get('gate_evidence', {}).values():
            required = merge(required, {gate['path']: gate['sha256']})
        require(all(doc['fixed_pins'].get(path) == digest for path, digest in required.items())
            and saved.get('path') == str(release) and saved.get('sha256') == doc['release_sha256']
            and saved.get('original_runtime_count') == len(value['runtime_hashes']),
            'Cached original qualification lacks complete boundary evidence pins')
        if fresh:
            validate_fresh_authority(doc, value, saved, parent)
        elif 'qualified_runtime' in value:
            original = saved.get('original_authority', {})
            require(saved.get('source_qualification') == 'original-six-gate-environment-release'
                and original.get('input_hashes') == value['input_hashes']
                and original.get('gate_evidence') == value['gate_evidence']
                and original.get('runtime_profile') == parent.profiles(),
                'Cached six-gate source result differs from unchanged original evidence')
        else:
            require(saved.get('source_qualification') == 'original-support-release',
                    'Cached support source result differs')
        return copy.deepcopy(saved)
    if fresh:
        adapter = fresh_release_verifier(value)
        authority = adapter.qualified_release_context(release, doc['environment'], physical(doc['manifest']))
        require(authority.get('passed') is True and authority.get('bulk_ready') is True
            and authority.get('environment') == doc['environment']
            and authority.get('manifest_sha256') == doc['manifest_sha256']
            and authority.get('capture_output') == doc['output']
            and authority.get('runtime_profile') == parent.profiles()
            and type(authority.get('native_exact_observations')) is int
            and 32 <= authority['native_exact_observations'] <= 48
            and authority.get('bulk_training_observations') == 0
            and authority.get('runtime_hashes') == value['runtime_hashes']
            and all(authority['runtime_hashes'].get(path) == digest
                for path, digest in doc['runtime_hashes'].items()),
            'Fresh source requires actual 32..48 native approvals and unchanged complete runtime/output')
        result = dict(schema=FRESH_AUTHORITY_SCHEMA, path=str(release), sha256=doc['release_sha256'],
            source_qualification='fresh-six-gate-environment-release', environment=doc['environment'],
            manifest=doc['manifest'], manifest_sha256=doc['manifest_sha256'],
            original_runtime_count=len(value['runtime_hashes']), original_authority=authority)
        validate_fresh_authority(doc, value, result, parent)
        return result
    if 'qualified_runtime' not in value:
        launcher = importlib.import_module('launch_base5_obstacle_support')
        _, env = launcher.preflight(release, doc['environment'])
        require(env['manifest'] == doc['manifest'] and value.get('bulk_ready') is True,
                'Original support source is not bulk-qualified')
        return dict(path=str(release), sha256=doc['release_sha256'],
                    source_qualification='original-support-release', original_runtime_count=len(value['runtime_hashes']))
    old_parent = read(Path(doc['old_output']) / parent.GUARD)
    original_destination = physical(value['capture_output'])
    authority = parent.qualified_release(release, doc['environment'], physical(doc['manifest']),
        original_destination, old_parent['runtime_hashes'])
    return dict(path=str(release), sha256=doc['release_sha256'],
                source_qualification='original-six-gate-environment-release',
                original_runtime_count=len(value['runtime_hashes']), original_authority=authority)


def validate_fresh_release(doc, value):
    """새 환경의 source 증거를 과거 환경의 자격이나 빈 prefix로 대체하지 않는다."""
    source = doc.get('canonical_manifest', doc['manifest'])
    source_sha = sha(source)
    environment = doc['environment']
    deck = read(source)
    require(deck.get('count') == len(deck.get('scenes', [])) == 5000
        and deck.get('config', {}).get('environment_tag') == environment
        and value.get('bulk_ready') is True and 'qualified_runtime' in value
        and value.get('qualified_environments') == [environment]
        and value.get('environment') == environment and value.get('full_manifest') == source
        and value.get('full_manifest_sha256') == source_sha
        and value.get('qualification_inputs', {}).get('manifest') == source
        and value.get('capture_output') == doc.get('canonical_output', doc['output'])
        and value.get('environments') == {environment:dict(manifest=source,
            manifest_sha256=source_sha, capture_output=value['capture_output'])},
        'Fresh release must bind this exact new source/environment/output')
    adapter = fresh_release_verifier(value)
    if value.get('qualification_schema') == 'base5_new_environment_release_qualification_v1':
        require(value.get('release_policy') == adapter.POLICY
            and value.get('helper_sha256') == sha(adapter.__file__)
            and doc['fixed_pins'].get(str(physical(adapter.__file__))) == sha(adapter.__file__),
            'Existing native-adopted six-gate release verifier is not pinned')
        return
    registration = importlib.import_module('prepare_base5_unit_environment_smoke')
    require(value.get('qualification_schema') == adapter.QUALIFICATION_SCHEMA
        and value.get('release_policy') == adapter.POLICY
        and value.get('fresh_source_adapter') == dict(path=str(physical(adapter.__file__)),
        sha256=sha(adapter.__file__), registration_schema=registration.SCHEMA),
        'Fresh release requires its explicitly pinned unit-registration six-gate adapter')
    require(doc['fixed_pins'].get(str(physical(adapter.__file__))) == sha(adapter.__file__),
            'Fresh source adapter is missing from the fixed boundary inventory')


def validate_fresh_authority(doc, value, saved, parent):
    original = saved.get('original_authority', {})
    require(saved.get('schema') == FRESH_AUTHORITY_SCHEMA
        and saved.get('source_qualification') == 'fresh-six-gate-environment-release'
        and saved.get('environment') == doc['environment']
        and saved.get('manifest') == doc['manifest']
        and saved.get('manifest_sha256') == doc['manifest_sha256']
        and original.get('passed') is True and original.get('bulk_ready') is True
        and original.get('environment') == doc['environment']
        and original.get('manifest_sha256') == doc['manifest_sha256']
        and original.get('capture_output') == doc['output']
        and type(original.get('native_exact_observations')) is int
        and 32 <= original['native_exact_observations'] <= 48
        and original.get('native_exact_observations') == value.get('native_exact_observations')
        and original.get('bulk_training_observations') == value.get('bulk_training_observations') == 0
        and original.get('runtime_hashes') == value['runtime_hashes']
        and all(value['runtime_hashes'].get(path) == digest for path,digest in doc['runtime_hashes'].items())
        and original.get('input_hashes') == value['input_hashes']
        and original.get('gate_evidence') == value['gate_evidence']
        and original.get('runtime_profile') == parent.profiles()
        and set(original.get('gate_evidence', {})) == set(parent.GATES),
        'Fresh six-gate source authority differs from this exact source or original evidence')
    for gate in original['gate_evidence'].values():
        require(gate.get('passed') is True and sha(gate['path']) == gate['sha256'],
                'Fresh six-gate native evidence changed')


def validate_profile(doc):
    require(doc.get('schema') == SCHEMA and doc.get('policy') == POLICY
        and doc.get('environment') in ENVIRONMENTS and doc.get('helper_pins') == profile_helpers(doc),
        'Unified profile/helper version changed')
    old, output, manifest = map(physical, (doc['old_output'], doc['output'], doc['manifest']))
    require(old != output and not inside(old, output) and not inside(output, old)
        and not inside(manifest, output), 'Historical/source/new output must be separate')
    require(doc.get('native_qa_unchanged') is True and doc.get('common_static_attitude') is True
        and doc.get('common_final_direction_reference') is True and doc.get('automatic_completion') is False,
        'Unified QA/safety policy differs')
    if 'fresh_start' in doc:
        canonical_output = physical(doc.get('canonical_output', doc['output']))
        require(doc['fresh_start'] is True
            and old == canonical_output.parent/(canonical_output.name+'_fresh_origin')
            and doc.get('fresh_origin_schema') == FRESH_ORIGIN_SCHEMA,
            'Fresh profile must declare a separate exact empty origin, never historical capture')
    return doc


def build_profile(manifest, release, old_output, output, route_witness=None, *, fresh_start=False):
    manifest, release, output = map(physical, (manifest, release, output))
    require((old_output is None) == fresh_start, 'Fresh prepare forbids historical output; paused prepare requires it')
    old = output.parent/(output.name+'_fresh_origin') if fresh_start else physical(old_output)
    parent, _ = modules()
    value, deck = read(release), read(manifest)
    require(deck['count'] == len(deck['scenes']) == 5000, 'Exact frozen 5K manifest required')
    require(not output.exists() and output.parent.is_dir(), 'Fresh sibling output required')
    doc = dict(schema=SCHEMA, policy=POLICY, environment=deck['config']['environment_tag'],
        manifest=str(manifest), manifest_sha256=sha(manifest), release=str(release), release_sha256=sha(release),
        old_output=str(old), output=str(output),
        native_qa_unchanged=True, common_static_attitude=True, common_final_direction_reference=True,
        automatic_completion=False, chunk_size=16)
    if fresh_start:
        require(not old.exists(), 'Fresh origin must not reuse any existing directory')
        doc.update(fresh_start=True, fresh_origin_schema=FRESH_ORIGIN_SCHEMA)
    doc['helper_pins'] = profile_helpers(doc)
    if route_witness:
        doc['route_witness'] = dict(path=str(physical(route_witness)), sha256=sha(route_witness))
    supplemental = parent.runtime_hashes()
    require(all(supplemental.get(name, digest) == digest for name, digest in value['runtime_hashes'].items()),
            'A shared frozen runtime file was edited; never qualify a mixed runtime')
    extras = {str(release): sha(release), str(manifest): sha(manifest), **doc['helper_pins']}
    if fresh_start:
        audit = value['full_cpu_audit']
        extras = merge(extras, {audit['path']:audit['sha256']})
    if route_witness:
        route_compat = importlib.import_module('fresh_route_compat' if fresh_start else 'route_compat')
        extras[str(physical(route_witness))] = sha(route_witness)
        extras[str(Path(route_compat.__file__).absolute())] = sha(route_compat.__file__)
    doc['fixed_pins'] = merge(value['input_hashes'], value['runtime_hashes'], supplemental, extras,
        {gate['path']: gate['sha256'] for gate in value.get('gate_evidence', {}).values()})
    require(not any(inside(physical(filename), root) for filename in doc['fixed_pins'] for root in (old, output)),
            'Live/historical capture output cannot be declared a source boundary input')
    doc['runtime_hashes'] = supplemental
    if fresh_start:
        validate_fresh_release(doc, value)
    return validate_profile(doc)


def boundary_for(doc):
    boundary = boundary_module.SessionInputBoundary(doc['fixed_pins'],
        forbidden_roots=(doc['output'], doc['old_output']))
    boundary.verify()
    return boundary


def warm_boundary(boundary, doc, direction):
    mapping = direction.input_pins(physical(doc['manifest']))
    boundary.remember_input_pins(physical(doc['manifest']), mapping)


def historical_verifiers(doc, parent):
    old = Path(doc['old_output'])
    if not (old / parent.GUARD).is_file():
        return {}
    guard = old / parent.GUARD
    def context(source, root):
        saved = read(guard)
        require(saved['manifest'] == str(source) and saved['manifest_sha256'] == sha(source)
            and saved['environment'] == doc['environment'] and saved['indices'] == list(range(5000))
            and saved['runtime_profile'] == parent.profiles()
            and saved['runtime_hashes'] == parent.runtime_hashes(), 'Historical static parent profile changed')
        parent.pins.verify_hashes(saved['input_hashes'])
        parent.pins.verify_hashes(saved['runtime_hashes'])
    def recovery(source, index, root):
        expected = dict(parent_contract_sha256=sha(guard), runtime_profile=parent.profiles()['isolated_recovery'],
            source_manifest=str(physical(source)), source_manifest_sha256=sha(source), index=index,
            runtime_hashes=read(guard)['runtime_hashes'])
        return parent.verify_strict_recovery(root, expected)
    return dict(verify_context=context, verify_regular=parent.verify_regular, verify_recovery_extra=recovery)


@contextmanager
def qualified_handoff(parent, doc, authority):
    saved = parent.qualified_release
    def qualified(release, environment, manifest, output, runtime):
        require(str(release) == doc['release'] and environment == doc['environment']
            and str(manifest) == doc['manifest'] and str(output) == doc['output']
            and runtime == doc['runtime_hashes'], 'Unified destination/source/runtime changed')
        return dict(authority, unified_runtime_policy=POLICY, runtime_profile=parent.profiles(),
            original_source_qualification_only=True, new_output_handoff=True,
            live_profile_qualification_separate=True)
    parent.qualified_release = qualified
    try:
        yield
    finally:
        changed = parent.qualified_release is not qualified
        parent.qualified_release = saved
        require(not changed, 'Qualified handoff adapter ownership changed')


def arguments(doc, run=False, stop_after_chunks=0):
    return argparse.Namespace(manifest=Path(doc['manifest']), output=Path(doc['output']),
        environment=doc['environment'], confirmed_environment=doc['environment'] if run else None,
        qualified_release=Path(doc['release']), limit=5000, chunk_size=16, run=run,
        prepare_only=not run, approve_bulk_capture=run, resume=True, stop_after_chunks=stop_after_chunks)


def initialize(parent, doc, authority):
    """원형 parent가 즉시 재검증할 새 초기 profile만 만든다. 구 output은 불변이다."""
    output = Path(doc['output']); deck = read(doc['manifest']); indices = list(range(5000))
    runtime = parent.runtime_hashes()
    inputs = parent.source_pins(Path(doc['manifest']), deck, indices)
    legacy = parent.batch.campaign.contract(Path(doc['manifest']), indices, 2)
    legacy.update(batch_driver_sha256=parent.sha256(parent.batch.__file__), chunk_size=16, bulk=True,
        replenishment_driver_sha256=parent.sha256(SCRIPTS / 'base5_campaign_replenishment.py'))
    guard = dict(schema=parent.SCHEMA, policy=parent.POLICY, manifest=doc['manifest'],
        manifest_sha256=doc['manifest_sha256'], environment=doc['environment'], indices=indices,
        chunk_size=16, runtime_profile=parent.profiles(), runtime_hashes=runtime, input_hashes=inputs,
        qualified_release=parent.qualified_release(Path(doc['release']), doc['environment'],
            Path(doc['manifest']), output, runtime), legacy_batch_contract=legacy,
        storage_gate=dict(minimum_gib=parent.storage_budget_gib(5000), scope='5K bulk',
            conservative_prototype_bytes_per_frame=100_000_000, prototype_maximum_frames=64),
        prototype_only=False, no_automatic_completion=True, bulk_training_observations_at_qualification=0)
    write_new(output / parent.GUARD, guard)
    require(parent.run(arguments(doc)) == 0, 'Original parent rejected the imported unified profile')


def fresh_empty_prefix(doc):
    """실제 과거 수집이 없는 새 환경은 명시적인 0장 origin으로만 시작한다."""
    validate_profile(doc)
    require(doc.get('fresh_start') is True, 'Explicit fresh prepare is required for an empty prefix')
    origin, output = physical(doc['old_output']), physical(doc['output'])
    require(not origin.exists() and not output.exists(), 'Fresh origin/output must both be new')
    receipt = dict(schema=FRESH_ORIGIN_SCHEMA, fresh_start=True, historical_capture=False,
        manifest=doc['manifest'], manifest_sha256=doc['manifest_sha256'], environment=doc['environment'],
        release=doc['release'], release_sha256=doc['release_sha256'], output=doc['output'],
        selected=[], deficits=[], elapsed_sec=0, helper_pins=doc['helper_pins'])
    origin.mkdir(mode=0o700)
    proof = origin/'fresh_empty_origin.json'
    write_new(proof, receipt)
    output.mkdir(mode=0o700)
    write_new(output/'imported_prefix.json', dict(schema=1, manifest_sha256=doc['manifest_sha256'],
        original_output=str(origin), selected=[], deficits=[], elapsed_sec=0,
        hashes={str(proof):sha(proof)}, sensor_stats={}, fresh_start=True,
        fresh_origin_schema=FRESH_ORIGIN_SCHEMA,
        raw_integrity='No historical capture exists; empty source prefix, no inherited observations'))
    verify_fresh_prefix(doc)
    return dict(imported=0, deficits=0, pending=5000, elapsed_sec=0, historical_capture=False,
        fresh_origin=str(origin))


def verify_fresh_prefix(doc):
    if doc.get('fresh_start') is not True:
        return
    origin, output = physical(doc['old_output']), physical(doc['output'])
    proof = origin/'fresh_empty_origin.json'
    require(set(origin.iterdir()) == {proof}, 'Fresh empty origin contains unexpected historical data')
    receipt = read(proof)
    expected = dict(schema=FRESH_ORIGIN_SCHEMA, fresh_start=True, historical_capture=False,
        manifest=doc['manifest'], manifest_sha256=doc['manifest_sha256'], environment=doc['environment'],
        release=doc['release'], release_sha256=doc['release_sha256'], output=doc['output'],
        selected=[], deficits=[], elapsed_sec=0, helper_pins=doc['helper_pins'])
    require(receipt == expected, 'Fresh origin does not prove the exact new source and zero inherited observations')
    prefix = read(output/'imported_prefix.json')
    require(prefix.get('schema') == 1 and prefix.get('fresh_start') is True
        and prefix.get('fresh_origin_schema') == FRESH_ORIGIN_SCHEMA
        and prefix.get('original_output') == str(origin)
        and prefix.get('manifest_sha256') == doc['manifest_sha256']
        and prefix.get('selected') == [] and prefix.get('deficits') == []
        and type(prefix.get('elapsed_sec')) is int and prefix['elapsed_sec'] == 0
        and prefix.get('hashes') == {str(proof):sha(proof)} and prefix.get('sensor_stats') == {},
        'Fresh prefix cannot inherit any observations, deficit, elapsed time or unbound evidence')


def prepare(doc, profile):
    parent, direction = modules()
    boundary = boundary_for(doc)
    try:
        with boundary.installed(scripts_dir=SCRIPTS, direction_module=direction), route_compatibility(doc):
            authority = source_authority(doc, parent)
            warm_boundary(boundary, doc, direction)
            if doc.get('fresh_start') is True:
                summary = fresh_empty_prefix(doc)
            else:
                summary = handoff.export(Path(doc['manifest']), Path(doc['old_output']), Path(doc['output']),
                    **historical_verifiers(doc, parent))
            doc['source_authority'] = authority
            with qualified_handoff(parent, doc, authority):
                initialize(parent, doc, authority)
            doc['import_sha256'] = sha(Path(doc['output']) / 'imported_prefix.json')
    finally:
        boundary.verify()
    write_new(profile, doc)
    return dict(event='unified_capture_prepared', output=doc['output'], imported=summary,
        native_captures=0, automatically_started=False)


def refresh_prepared_profile(profile):
    """소량 자격시험 전의 native-zero 실행기 보정만 새 계약으로 기록한다.

    수집·자격시험을 마친 출력에는 붙일 수 없다. 이전 profile 사본을 보존하고
    source/import/원형 parent 계약을 실제 재검증한다. 과거 수집 출력은 변경하지 않는다.
    """
    profile = physical(profile)
    previous = read(profile); previous_sha = sha(profile)
    require(previous.get('fresh_start') is not True,
            'Fresh empty-origin contracts are immutable; prepare a new separate output after helper changes')
    require(previous.get('schema') == SCHEMA and previous.get('policy') == POLICY,
            'Only the same explicit unified policy can be revised before capture')
    root = physical(previous['output'])
    state = read(root/'status.json')
    require(state.get('stage') == 'prepared_waiting_for_resume' and state.get('current') is None
        and state.get('failed') == 0 and not (root/'unified_live_qualification.json').exists()
        and not any(path.is_dir() for path in root.iterdir())
        and not list(root.rglob('session_contract.json'))
        and not (root/'STOP').exists() and not (root/'PAUSE').exists(),
        'Profile revision is forbidden after any main native capture, qualification or halt')
    require(sha(root/'imported_prefix.json') == previous['import_sha256'], 'Prepared import changed')
    current_helpers = profile_helpers(previous)
    require(set(previous['helper_pins']) == set(current_helpers), 'Helper scope changed; use a new output')
    doc = copy.deepcopy(previous)
    doc['helper_pins'] = current_helpers
    for filename, digest in current_helpers.items():
        require(doc['fixed_pins'].get(filename) == previous['helper_pins'][filename],
                'Previous prepared helper binding differs')
        doc['fixed_pins'][filename] = digest
    release = read(doc['release'])
    doc['fixed_pins'] = merge(doc['fixed_pins'], release['input_hashes'], release['runtime_hashes'],
        {gate['path']: gate['sha256'] for gate in release.get('gate_evidence', {}).values()})
    validate_profile(doc)
    parent, direction = modules(); boundary = boundary_for(doc)
    try:
        with boundary.installed(scripts_dir=SCRIPTS, direction_module=direction), route_compatibility(doc):
            authority = source_authority(doc, parent)
            require(authority == doc['source_authority'], 'Original source authority changed')
            with qualified_handoff(parent, doc, authority):
                require(parent.run(arguments(doc)) == 0, 'Original prepared parent profile changed')
    finally:
        boundary.verify()
    archive = profile.with_name(profile.stem+'.prequalification_'+previous_sha[:16]+'.json')
    require(not archive.exists(), 'Previous prequalification revision already exists')
    write_new(archive, previous)
    require(sha(profile) == previous_sha, 'Prepared profile changed during revision')
    parent.batch.atomic_json(profile, doc)
    os.chmod(profile, 0o600)
    write_new(root/('prepared_profile_revision_'+str(time.time_ns())+'.json'),
        dict(policy=POLICY, previous_profile=str(archive), previous_sha256=previous_sha,
            current_profile=str(profile), current_sha256=sha(profile),
            main_native_captures=0, original_source_and_import_unchanged=True))
    return dict(event='unified_native_zero_profile_revised', output=str(root),
        previous_profile=str(archive), main_native_captures=0, capture_started=False)


def identity(pid):
    root = Path('/proc') / str(pid)
    value = (root / 'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, uid=root.stat().st_uid, start_ticks=value[19])


def sealed_payload(payload):
    fd = os.memfd_create('base5-verified-input-boundary', os.MFD_ALLOW_SEALING | os.MFD_CLOEXEC)
    try:
        content = json.dumps(payload, separators=(',', ':'), allow_nan=False).encode()
        view = memoryview(content)
        while view:
            count = os.write(fd, view); view = view[count:]
        os.lseek(fd, 0, os.SEEK_SET)
        fcntl.fcntl(fd, fcntl.F_ADD_SEALS, SEALS)
        return fd
    except BaseException:
        os.close(fd)
        raise


def worker_payload(fd, expected_parent):
    require(expected_parent == identity(os.getppid()) and expected_parent['uid'] == os.getuid(),
            'Worker must belong to its live invoking parent')
    command = (Path('/proc') / str(os.getppid()) / 'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
    command = [os.fsdecode(value) for value in command]
    index = 1
    while len(command) > index and command[index] in ('-B', '-u'):
        index += 1
    require(len(command) > index and Path(command[index]).name == Path(__file__).name,
            'Worker parent must be the unified launcher, not a standalone Python process')
    script = Path(command[index])
    if not script.is_absolute():
        script = Path('/proc', str(os.getppid()), 'cwd').resolve() / script
    require(script == Path(__file__).absolute()
        and any(flag in command[index+1:] for flag in ('--run', '--worker', '--qualify-prototype')),
        'Worker parent script/mode identity changed')
    require(fcntl.fcntl(fd, fcntl.F_GET_SEALS) & SEALS == SEALS,
            'Worker requires a sealed anonymous parent snapshot')
    require(stat.S_ISREG(os.fstat(fd).st_mode) and '/memfd:base5-verified-input-boundary' in os.readlink('/proc/self/fd/' + str(fd)),
            'Worker snapshot must be an anonymous memfd')
    # Shared file offsets must not race concurrently spawning children.
    size = os.fstat(fd).st_size
    require(0 < size <= 100_000_000, 'Bounded snapshot required')
    return json.loads(os.pread(fd, size, 0))


def validate_worker_arguments(target, values, doc):
    target = physical(target)
    require(target.parent == SCRIPTS and target.name in TARGETS, 'Unknown native worker target')
    for flag in ('--manifest', '--output'):
        require(values.count(flag) == 1, 'One exact worker ' + flag + ' required')
    source = physical(values[values.index('--manifest') + 1])
    output = physical(values[values.index('--output') + 1])
    root = physical(doc['output'])
    require(output != root and inside(output, root), 'Worker output escaped its campaign')
    require(source == physical(doc['manifest']) or inside(source, root), 'Worker source escaped its lineage')
    require(not any(v.startswith('--com') for v in values), 'Automatic completion is not qualified')
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument('--manifest', required=True); parser.add_argument('--output', required=True)
    if target.name == 'run_base5_capture_static_attitude.py':
        parser.add_argument('--enable-static-surface-attitude', action='store_true', required=True)
        parser.add_argument('--allow-static-pedestrians', action='store_true', required=True)
        parser.add_argument('--setup-profile', choices=['static-ready'], required=True)
        parser.add_argument('--shared-ros-batch', action='store_true', required=True)
        parser.add_argument('--limit', type=int, required=True)
    elif target.name == 'run_base5_capture_campaign_direction_reference.py':
        parser.add_argument('--enable-final-direction-reference', action='store_true', required=True)
        parser.add_argument('--indices', required=True); parser.add_argument('--resume', action='store_true', required=True)
        parser.add_argument('--seed-capture')
    else:
        parser.add_argument('--limit', type=int, required=True)
    try:
        options = parser.parse_args(values)
    except SystemExit as error:
        raise RuntimeError('Unknown or malformed bounded native worker flags') from error
    flags = [value for value in values if value.startswith('--')]
    require(len(flags) == len(set(flags)), 'Duplicate native worker flags are forbidden')
    if target.name == 'run_base5_capture_static_attitude.py':
        require(1 <= options.limit <= 16 and output.name == 'capture'
            and source == output.parent/'execution_source/manifest.json',
            'Exact independent 1..16-scene regular chunk required')
    elif target.name == 'run_base5_capture_campaign_direction_reference.py':
        require(options.indices.isdecimal(), 'Strict recovery requires one exact source index')
        index = int(options.indices); entries = read(doc['manifest'])['scenes']
        require(source == physical(doc['manifest']) and 0 <= index < len(entries)
            and options.indices == str(index)
            and output == root/('chunk_%04d' % (index//doc.get('chunk_size', 16)))/'recovery'/entries[index]['id'],
            'Strict recovery must stay in its exact frozen source quota cell')
        require(options.seed_capture is None or physical(options.seed_capture)
            == output.parent/(output.name+'_seed.json'), 'Seed escaped the exact quota cell')
    else:
        require(options.limit == 1, 'Nested native preparation is limited to one scene')
    return target


@contextmanager
def routed_workers(doc, boundary):
    saved = subprocess.Popen
    def routed(command, *args, **kwargs):
        if isinstance(command, (str, bytes)):
            require(not any(name in os.fsdecode(command) for name in TARGETS), 'Native shell dispatch is forbidden')
            return saved(command, *args, **kwargs)
        argv = [os.fsdecode(v) for v in command]
        index = 1
        while len(argv) > index and argv[index] in ('-B', '-u'):
            index += 1
        if len(argv) <= index or Path(argv[index]).name not in TARGETS:
            require(not any(Path(v).name in TARGETS for v in argv), 'Unknown native command shape')
            return saved(command, *args, **kwargs)
        require(argv[0] == sys.executable and not kwargs.get('shell'), 'Native interpreter changed')
        target = validate_worker_arguments(argv[index], argv[index + 1:], doc)
        parent = identity(os.getpid())
        fd = sealed_payload(dict(profile=doc, boundary=boundary.snapshot(), parent=parent))
        command = [sys.executable, '-B', str(Path(__file__).absolute()), '--worker', str(target),
            '--session-fd', str(fd), '--parent-pid', str(parent['pid']),
            '--parent-start-ticks', parent['start_ticks'], '--', *argv[index + 1:]]
        kwargs['pass_fds'] = tuple(set(kwargs.get('pass_fds', ())) | {fd})
        try:
            return saved(command, *args, **kwargs)
        finally:
            os.close(fd)
    subprocess.Popen = routed
    try:
        yield
    finally:
        changed = subprocess.Popen is not routed
        subprocess.Popen = saved
        require(not changed, 'Unified worker router ownership changed')


def run_worker(args):
    payload = worker_payload(args.session_fd,
        dict(pid=args.parent_pid, uid=os.getuid(), start_ticks=args.parent_start_ticks))
    require(payload['parent'] == identity(os.getppid()), 'Snapshot belongs to another parent')
    doc = payload['profile']
    # No disk profile/native evidence is trusted through this capability.
    validate_profile(doc)
    values = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
    target = validate_worker_arguments(args.worker, values, doc)
    _, direction = modules()
    boundary = boundary_module.SessionInputBoundary.from_verified_snapshot(payload['boundary'], authority_verified=True)
    with boundary.installed(scripts_dir=SCRIPTS, direction_module=direction,
            delta_input_manifest=doc.get('canonical_manifest', doc['manifest'])), \
            route_compatibility(doc), recovery_population.installed(), routed_workers(doc, boundary):
        previous = sys.argv
        try:
            sys.argv = [str(target), *values]
            if target.name == 'run_base5_capture_campaign_direction_reference.py':
                # run_path would define a second unpatched input_pins function
                # in __main__, silently repeating the entire reference graph.
                direction.main()
            else:
                runpy.run_path(str(target), run_name='__main__')
            return 0
        except SystemExit as error:
            return error.code if isinstance(error.code, int) else (0 if error.code is None else 1)
        finally:
            sys.argv = previous


def release_pause(root):
    state = read(root / 'status.json')
    require(state['stage'] in ('prepared_waiting_for_resume', 'paused'),
            'Only prepared or safely paused unified output can resume; inspect any halt')
    require(not (root / 'STOP').exists(), 'STOP is never released by ordinary resume')
    marker = root / 'PAUSE'
    if state['stage'] == 'paused':
        handoff.verify_exit(root)
        require(marker.is_file(), 'Paused marker is missing')
        os.rename(marker, root / ('PAUSE.resumed_' + str(time.time_ns())))
    else:
        require(not marker.exists(), 'Prepared output has an unexpected pause marker')


@contextmanager
def completed_readonly_replay(parent, output):
    """원형의 read-only verifier를 사용해 완료 이동 장면도 재실행하지 않는다.

    원형 default completed replay는 지면 준비 셀의 recover subprocess를 다시
    시작한다. _verification 경로는 같은 실제 증거 검증기를 호출하고 load·
    capture·선택/시간 재작성은 하지 않는다. parent의 추가 QA callback도 유지한다.
    """
    saved = parent.batch.execute
    observed = {}
    def verify(*args, **kwargs):
        require(len(args) >= 3 and physical(args[2]) == physical(output)
            and read(Path(output)/'status.json')['stage'] in ('complete', 'complete_with_deficit')
            and '_verification' not in kwargs, 'Completed replay requires exact terminal output')
        return saved(*args, **dict(kwargs, _verification=observed))
    parent.batch.execute = verify
    try:
        yield observed
    finally:
        changed = parent.batch.execute is not verify
        parent.batch.execute = saved
        require(not changed, 'Completed replay verifier ownership changed')


def qualify_prototype(doc, indices):
    """별도 소량 촬영·정상 종료·재개 시험. 본수집 출력에는 한 장도 추가하지 않는다."""
    validate_profile(doc)
    require(2 <= len(indices) <= 8 and len(indices) == len(set(indices))
        and all(type(index) is int and 0 <= index < 5000 for index in indices),
        'Qualification requires 2..8 explicit unique source indices')
    root = physical(doc['output'])
    if doc.get('fresh_start') is True:
        require(len(indices) == 5, 'Fresh common-runtime qualification requires exactly five native scenes')
        verify_fresh_prefix(doc)
        require(sha(root/'imported_prefix.json') == doc['import_sha256'], 'Fresh imported prefix changed')
        state = read(root/'status.json')
        require(state.get('stage') == 'prepared_waiting_for_resume' and state.get('current') is None
            and all(state.get(key) == 0 for key in ('finished','accepted_candidate','review','failed'))
            and not list(root.rglob('session_contract.json')),
            'Fresh qualification must precede every main native capture')
    qualification_path = root/'unified_live_qualification.json'
    require(not qualification_path.exists(), 'Qualification already exists; preserve its actual evidence')
    parent, direction = modules()
    deck = read(doc['manifest'])
    token = str(time.time_ns())
    folder = root.parent/(root.name+'_qualification_'+token)
    folder.mkdir(mode=0o700)
    source_folder = folder/'source'
    source_folder.mkdir(mode=0o700)
    source = source_folder/'manifest.json'
    write_new(source, parent.batch.campaign.subset_deck(deck, [deck['scenes'][index] for index in indices]))
    pilot = copy.deepcopy(doc)
    pilot.update(canonical_manifest=doc['manifest'], manifest=str(source), manifest_sha256=sha(source),
                 output=str(folder/'capture'), chunk_size=1)
    if doc.get('fresh_start') is True:
        pilot['canonical_output'] = doc['output']
    pilot_boundary = boundary_for(doc)
    started = time.perf_counter()
    try:
        with pilot_boundary.installed(scripts_dir=SCRIPTS, direction_module=direction), route_compatibility(pilot):
            source_authority(doc, parent)
            warm_boundary(pilot_boundary, doc, direction)
        with pilot_boundary.installed(scripts_dir=SCRIPTS, direction_module=direction,
                delta_input_manifest=doc['manifest']), route_compatibility(pilot):
            # The pilot manifest itself is fresh, never a trusted fixed-input pin.
            args = argparse.Namespace(manifest=source, output=Path(pilot['output']),
                environment=doc['environment'], confirmed_environment=doc['environment'],
                qualified_release=None, limit=len(indices), chunk_size=1, run=True, prepare_only=False,
                approve_bulk_capture=False, resume=False, stop_after_chunks=1)
            with routed_workers(pilot, pilot_boundary):
                require(parent.run(args) == 0, 'Bounded normal capture/pause failed')
                paused = read(Path(pilot['output'])/'status.json')
                require(paused['stage'] == 'paused' and paused['finished'] == 1,
                        'Actual one-chunk prototype pause was not recorded')
                handoff.verify_exit(Path(pilot['output']), require_marker=False)
                args.resume = True; args.stop_after_chunks = 0
                require(parent.run(args) == 0, 'Bounded prototype resume failed')
            state = read(Path(pilot['output'])/'status.json')
            require(state['stage'] in ('complete', 'complete_with_deficit')
                and state['finished'] == len(indices) and state['accepted_candidate'] == len(indices)
                and state['review'] == 0 and state['failed'] == 0,
                'Not all requested native qualification cells were approved')
            before = parent.completed_snapshot(Path(pilot['output']))
            # Actual terminal replay must not send another simulator command.
            previous = subprocess.Popen
            def forbidden(*args, **kwargs):
                raise RuntimeError('Completed pilot replay may not launch a worker')
            subprocess.Popen = forbidden
            try:
                with completed_readonly_replay(parent, Path(pilot['output'])) as verified:
                    require(parent.run(args) == 0, 'Completed prototype replay failed')
                require(len(verified.get('selected', {})) == len(indices)
                    and not verified.get('deficits'), 'Completed replay selection proof differs')
            finally:
                subprocess.Popen = previous
            after = parent.completed_snapshot(Path(pilot['output']))
            require(before == after, 'Completed replay changed native/ledger/time evidence')
    finally:
        pilot_boundary.verify()
    # Hash actual proof files; a JSON-only passed marker cannot authorize bulk.
    proof = {}
    for path in sorted(Path(pilot['output']).rglob('*')):
        if path.is_file():
            proof[str(physical(path))] = sha(path)
    receipt = dict(schema='base5_unified_native_profile_qualification_v1', policy=POLICY, passed=True,
        manifest_sha256=doc['manifest_sha256'], environment=doc['environment'], helper_pins=doc['helper_pins'],
        pilot_manifest=str(source), pilot_manifest_sha256=sha(source), source_indices=indices,
        pilot_output=pilot['output'], captures=len(indices), approved=len(indices),
        actual_pause_resume=True, completed_replay_no_new_capture=True,
        proof_hashes=proof, elapsed_sec=time.perf_counter()-started,
        main_output_native_captures=0, boundary_stats=pilot_boundary.stats)
    write_new(qualification_path, receipt)
    return receipt


def run(doc, stop_after_chunks=0):
    validate_profile(doc)
    root = physical(doc['output'])
    require(sha(root / 'imported_prefix.json') == doc['import_sha256'], 'Imported prefix changed')
    verify_fresh_prefix(doc)
    require((root / 'unified_live_qualification.json').is_file(),
            'Common native-profile live qualification must pass before operator bulk start')
    qualification = read(root / 'unified_live_qualification.json')
    if doc.get('fresh_start') is True:
        require(qualification.get('schema') == 'base5_unified_native_profile_qualification_v1'
            and qualification.get('environment') == doc['environment']
            and qualification.get('captures') == qualification.get('approved') == 5
            and qualification.get('main_output_native_captures') == 0
            and len(qualification.get('source_indices', [])) == 5
            and len(set(qualification['source_indices'])) == 5
            and qualification.get('proof_hashes'),
            'Fresh bulk start requires its actual five-scene common-runtime qualification')
    require(qualification.get('passed') is True and qualification.get('policy') == POLICY
        and qualification.get('manifest_sha256') == doc['manifest_sha256']
        and qualification.get('helper_pins') == doc['helper_pins'], 'Live qualification differs')
    require(sha(qualification['pilot_manifest']) == qualification['pilot_manifest_sha256']
        and qualification.get('actual_pause_resume') is True
        and qualification.get('completed_replay_no_new_capture') is True,
        'Actual native pause/resume qualification is required')
    for filename, digest in qualification['proof_hashes'].items():
        require(sha(filename) == digest, 'Native qualification proof changed: '+filename)
    parent, direction = modules()
    boundary = boundary_for(doc)
    session = str(time.time_ns())
    result = 1
    try:
        with boundary.installed(scripts_dir=SCRIPTS, direction_module=direction), route_compatibility(doc):
            authority = source_authority(doc, parent)
            require(authority == doc['source_authority'], 'Original source qualification changed')
            warm_boundary(boundary, doc, direction)
        with boundary.installed(scripts_dir=SCRIPTS, direction_module=direction,
                delta_input_manifest=doc['manifest']), route_compatibility(doc):
            release_pause(root)
            with qualified_handoff(parent, doc, authority), routed_workers(doc, boundary):
                result = parent.run(arguments(doc, True, stop_after_chunks))
            state = read(root/'status.json')
            if result == 0 and stop_after_chunks and state.get('stage') == 'paused' and not (root/'PAUSE').exists():
                handoff.verify_exit(root, require_marker=False)
                write_new(root/'PAUSE', dict(reason='bounded_chunk_stop', policy=POLICY,
                    semantics='Verified normal chunk boundary; operator may explicitly resume'))
    finally:
        try:
            boundary.verify()
        except Exception as error:
            state = read(root / 'status.json')
            parent.batch.atomic_json(root / 'status.json', dict(state, stage='halted', failed=1,
                error='Fixed input boundary verification failed; session requires manual quarantine: ' + str(error)))
            write_new(root / ('integrity_failed_' + session + '.json'),
                dict(policy=POLICY, passed=False, error=str(error), selected_not_certified=True))
            raise
        write_new(root / ('integrity_boundary_' + session + '.json'),
            dict(policy=POLICY, passed=True, result=result, fixed_inputs=len(doc['fixed_pins']),
                boundary_stats=boundary.stats, hot_path_native_qa_cached=False))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--profile', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare', action='store_true')
    mode.add_argument('--prepare-fresh', action='store_true',
        help='Prepare an explicitly empty new-environment prefix; never inherit historical captures')
    mode.add_argument('--run', action='store_true')
    mode.add_argument('--qualify-prototype', action='store_true')
    mode.add_argument('--refresh-prepared-profile', action='store_true')
    parser.add_argument('--manifest', type=Path); parser.add_argument('--release', type=Path)
    parser.add_argument('--old-output', type=Path); parser.add_argument('--output', type=Path)
    parser.add_argument('--route-witness', type=Path)
    parser.add_argument('--confirmed-environment', choices=ENVIRONMENTS)
    parser.add_argument('--stop-after-chunks', type=int, default=0)
    parser.add_argument('--indices', help='2..8 source indices for separate bounded native qualification')
    parser.add_argument('--worker', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--session-fd', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--parent-pid', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--parent-start-ticks', help=argparse.SUPPRESS)
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.worker is not None:
        require(not args.prepare and not args.prepare_fresh and not args.run and not args.qualify_prototype
            and not args.refresh_prepared_profile and args.session_fd is not None
            and args.parent_pid is not None and args.parent_start_ticks, 'Explicit authenticated worker required')
        return run_worker(args)
    require(args.profile is not None and not args.arguments, 'Explicit unified profile required')
    require(args.stop_after_chunks >= 0, 'Nonnegative bounded chunk limit required')
    if args.prepare or args.prepare_fresh:
        require(all(v is not None for v in (args.manifest, args.release, args.output))
            and ((args.old_output is None) if args.prepare_fresh else (args.old_output is not None))
            and args.confirmed_environment is None and args.indices is None,
            'Prepare requires explicit source/separate output and correct fresh or historical origin, no capture')
        require(not physical(args.profile).exists(), 'Prepare requires a new profile; never overwrite an existing contract')
        doc = build_profile(args.manifest, args.release, args.old_output, args.output, args.route_witness,
            **({'fresh_start':True} if args.prepare_fresh else {}))
        print(json.dumps(prepare(doc, args.profile), ensure_ascii=False), flush=True)
        return 0
    require(all(v is None for v in (args.manifest, args.release, args.old_output, args.output, args.route_witness)),
            'Source paths are frozen in the prepared profile')
    if args.refresh_prepared_profile:
        require(args.confirmed_environment is None and args.indices is None,
                'Native-zero profile revision is CPU-only')
        with physical(args.profile).open('rb') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            print(json.dumps(refresh_prepared_profile(args.profile), ensure_ascii=False), flush=True)
        return 0
    doc = validate_profile(read(args.profile))
    if args.qualify_prototype:
        require(args.confirmed_environment == doc['environment'] and args.indices is not None,
                'Confirm the actual environment and explicit bounded qualification indices')
        indices = [int(value) for value in args.indices.split(',')]
        with physical(args.profile).open('rb') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            print(json.dumps(qualify_prototype(doc, indices), ensure_ascii=False), flush=True)
        return 0
    require(args.indices is None, 'Indices are only for a bounded qualification prototype')
    if not args.run:
        print(json.dumps(dict(event='unified_capture_plan', output=doc['output'],
            environment=doc['environment'], files_written=False, captures_started=False), ensure_ascii=False))
        return 0
    require(args.confirmed_environment == doc['environment'], 'Confirm the actual MORAI UI environment')
    with physical(args.profile).open('rb') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run(doc, args.stop_after_chunks)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='unified_capture_blocked', error=str(error), fallback_allowed=False),
            ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
