#!/usr/bin/env python3
"""빌린 PC의 crash checkpoint를 보존한 별도 공통 수집 출력을 준비·실행한다.

기본은 읽기 전용 계획이다. prepare는 CPU 인계만 하고 run은 사용자가
명시한 Sunny13 본수집만 실행한다. 기존 halted/Park 실패를 고치지 않는다.
공통 수집기·QA·worker 인증 코드와 기존 동결 파일은 수정하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import copy
import fcntl
import json
import os
from pathlib import Path
import subprocess


SCHEMA = 'base5_unified_crash_continuation_v1'
RUNTIME_REL = Path('work/unified-runtime-v1')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load_modules(root):
    helpers = root/'checks'
    if str(helpers) not in sys.path:
        sys.path.insert(0, str(helpers))
    import unified_borrowed as borrowed
    import unified_capture as common
    import unified_crash_import as crash
    return borrowed, common, crash


def scope(ctx, common, source_profile=None, profile=None, output=None):
    root = ctx['workspace']/RUNTIME_REL
    require(common.physical(__file__) == root/'recover_unified_borrowed.py',
            'Run only the separately deployed borrowed-host recovery helper')
    source = common.physical(source_profile or ctx['paths']['profile'])
    target = common.physical(profile or root/'crash-continuation-v1/sunny13_profile.json')
    require(root in source.parents and root in target.parents and source != target,
            'Separate source and continuation profiles must remain inside the approved runtime')
    previous = common.validate_profile(common.read(source))
    require(previous['environment'] == 'sunny13'
        and previous['manifest'] == ctx['intent']['manifest']
        and previous['release'] == ctx['intent']['release'],
        'Only the unchanged approved Sunny13 source may continue')
    old = common.physical(previous['output'])
    destination = common.physical(output or old.with_name(old.name+'_after_crash_v1'))
    collections = ctx['workspace']/'collections'
    require(old.parent == collections and destination.parent == collections
            and old != destination, 'Separate sibling collection outputs required')
    return dict(source=source, profile=target, old=old, output=destination,
                previous=previous, root=root)


def validate_qualification(common, previous):
    path = Path(previous['output'])/'unified_live_qualification.json'
    value = common.read(path)
    require(value.get('passed') is True and value.get('policy') == common.POLICY
        and value.get('manifest_sha256') == previous['manifest_sha256']
        and value.get('helper_pins') == common.helpers()
        and value.get('actual_pause_resume') is True
        and value.get('completed_replay_no_new_capture') is True,
        'The exact unchanged common runtime needs its original native qualification')
    require(common.sha(value['pilot_manifest']) == value['pilot_manifest_sha256']
        and isinstance(value.get('proof_hashes'), dict) and value['proof_hashes'],
        'Complete original qualification proof inventory required')
    for filename, digest in value['proof_hashes'].items():
        require(common.sha(filename) == digest, 'Native qualification proof changed: '+filename)
    return path, value


def prepared(common, selection):
    profile = selection['profile']
    doc = common.validate_profile(common.read(profile))
    require(doc['old_output'] == str(selection['old']) and doc['output'] == str(selection['output'])
        and doc['manifest'] == selection['previous']['manifest']
        and doc['manifest_sha256'] == selection['previous']['manifest_sha256']
        and doc['helper_pins'] == selection['previous']['helper_pins'],
        'Prepared continuation source, destination or common runtime changed')
    receipt = common.read(selection['output']/'unified_crash_continuation.json')
    qualification_path, _ = validate_qualification(common, selection['previous'])
    qualification_sha = common.sha(qualification_path)
    require(receipt.get('schema') == SCHEMA
        and receipt.get('source_profile') == str(selection['source'])
        and receipt.get('source_profile_sha256') == common.sha(selection['source'])
        and receipt.get('profile') == str(profile)
        and receipt.get('profile_sha256') == common.sha(profile)
        and receipt.get('original_output') == str(selection['old'])
        and receipt.get('output') == str(selection['output'])
        and receipt.get('qualification_source') == str(qualification_path)
        and receipt.get('qualification_sha256') == qualification_sha
        and common.sha(selection['output']/'unified_live_qualification.json') == qualification_sha
        and receipt.get('qualification_reused_unchanged') is True
        and receipt.get('new_native_qualification_claimed') is False
        and receipt.get('native_qa_unchanged') is True
        and receipt.get('unsafe_suffix_imported') is False
        and receipt.get('captures_started_during_prepare') == 0,
        'Explicit crash continuation receipt changed')
    return doc, receipt


def copy_new(source, destination):
    """검증한 원 보고서의 bytes를 별도 새 파일에만 그대로 보존한다."""
    content = Path(source).read_bytes()
    fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(content)


def prepare_inside(common, crash, selection):
    require(not selection['profile'].exists() and not selection['output'].exists(),
            'Continuation already exists; never overwrite or re-prepare captured data')
    previous = selection['previous']
    qualification_path, qualification = validate_qualification(common, previous)
    doc = copy.deepcopy(previous)
    doc.update(old_output=str(selection['old']), output=str(selection['output']))
    doc.pop('import_sha256', None)
    extras = {str(selection['source']): common.sha(selection['source']),
              str(Path(__file__).absolute()): common.sha(__file__),
              str(Path(crash.__file__).absolute()): common.sha(crash.__file__)}
    doc['fixed_pins'] = common.merge(doc['fixed_pins'], extras)
    common.validate_profile(doc)
    parent, direction = common.modules()
    boundary = common.boundary_for(doc)
    try:
        with boundary.installed(scripts_dir=common.SCRIPTS, direction_module=direction), \
                common.route_compatibility(doc):
            authority = common.source_authority(doc, parent)
            common.warm_boundary(boundary, doc, direction)
            summary = crash.export(Path(doc['manifest']), selection['old'], selection['output'],
                **common.historical_verifiers(doc, parent))
            doc['source_authority'] = authority
            with common.qualified_handoff(parent, doc, authority):
                common.initialize(parent, doc, authority)
            doc['import_sha256'] = common.sha(selection['output']/'imported_prefix.json')
            # 새 live 시험을 했다고 쓰지 않는다. 동일 코드/입력의 실제 기존
            # 자격과 원본 proof bytes를 검증한 뒤 그 보고서를 그대로 계승한다.
            copy_new(qualification_path, selection['output']/'unified_live_qualification.json')
    finally:
        boundary.verify()
    selection['profile'].parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    common.write_new(selection['profile'], doc)
    receipt = dict(schema=SCHEMA, source_profile=str(selection['source']),
        source_profile_sha256=common.sha(selection['source']), profile=str(selection['profile']),
        profile_sha256=common.sha(selection['profile']), original_output=str(selection['old']),
        output=str(selection['output']), imported=summary,
        qualification_source=str(qualification_path), qualification_sha256=common.sha(qualification_path),
        qualification_reused_unchanged=True, new_native_qualification_claimed=False,
        unsafe_suffix_imported=False, captures_started_during_prepare=0,
        old_halted_status_retained=True, native_qa_unchanged=True)
    common.write_new(selection['output']/'unified_crash_continuation.json', receipt)
    prepared(common, selection)
    return dict(event='unified_crash_continuation_prepared', output=doc['output'],
        selected=summary['selected_count'], deficits=summary['deficit_count'],
        finished=summary['finished'], captures_started=0,
        unsafe_suffix_imported=False, qualification_reused_unchanged=True)


@contextmanager
def preserved_namespace(borrowed, selection):
    original = borrowed.namespace
    def namespace(ctx, argv):
        command = original(ctx, argv)
        index = command.index('--chdir')
        paths = [selection['old'], selection['source'], Path(__file__).absolute(),
                 selection['root']/'unified_crash_import.py']
        if selection['profile'].is_file():
            paths.append(selection['profile'])
        mounts = [word for path in paths for word in ('--ro-bind', str(path), str(path))]
        return command[:index]+mounts+command[index:]
    borrowed.namespace = namespace
    try:
        yield
    finally:
        require(borrowed.namespace is namespace, 'Recovery namespace ownership changed')
        borrowed.namespace = original


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('action', nargs='?', default='plan', choices=('plan', 'prepare', 'run', 'status'))
    parser.add_argument('--source-profile', type=Path)
    parser.add_argument('--profile', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--confirmed-environment', choices=('sunny13',))
    parser.add_argument('--inside', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    require((args.confirmed_environment is not None) == (args.action == 'run'),
            'Only actual run requires explicit Sunny13 UI confirmation')
    require(not args.inside or args.action in ('plan', 'prepare'),
            'Inside dispatch is CPU-only, never native capture')
    root = Path(__file__).absolute().parent
    workspace = root.parents[1]
    require(workspace.name == 'ajounice_base5_capture' and root == workspace/RUNTIME_REL,
            'Run only the approved borrowed-host deployment')
    borrowed, common, crash = load_modules(root)
    if args.inside:
        # PID namespace 내부에서 host ROS의 PID 신원을 다시 해석하지 않는다.
        # 바깥 launcher가 검증한 read-only intent와 같은 배포 경로만 사용한다.
        ctx = dict(workspace=workspace, paths=dict(profile=root/'sunny13_profile.json'),
                   intent=common.read(workspace/'checks/population_recovery_intent.json'))
    else:
        ctx = borrowed.load()
    selection = scope(ctx, common, args.source_profile, args.profile, args.output)
    if args.inside:
        if args.action == 'plan':
            result = crash.plan(Path(selection['previous']['manifest']), selection['old'], selection['output'])
        else:
            with selection['source'].open('rb') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = prepare_inside(common, crash, selection)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    if args.action == 'status':
        doc, _ = prepared(common, selection)
        print(json.dumps(dict(output=doc['output'], state=common.read(Path(doc['output'])/'status.json')),
                         ensure_ascii=False, indent=2))
        return 0
    if args.action == 'run':
        doc, _ = prepared(common, selection)
        command = ['/usr/bin/python3', '-B', '-u', str(ctx['paths']['checks']/'unified_capture.py'),
                   '--profile', str(selection['profile']), '--run', '--confirmed-environment', 'sunny13']
    else:
        command = ['/usr/bin/python3', '-B', '-u', str(Path(__file__).absolute()), args.action, '--inside',
                   '--source-profile', str(selection['source']), '--profile', str(selection['profile']),
                   '--output', str(selection['output'])]
    # 원형 supervisor와 동일 capture.lock·정상 PAUSE 신호 처리를 사용하되,
    # PAUSE 대상만 명시적으로 새 출력으로 연결한다. frozen 모듈은 수정하지 않는다.
    updated = dict(ctx, paths=dict(ctx['paths'], old=selection['old'], output=selection['output'],
                                  profile=selection['profile']))
    with preserved_namespace(borrowed, selection):
        return borrowed.supervise(updated, 'run' if args.action == 'run' else 'prepare', command)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(event='unified_crash_continuation_blocked', error=str(error),
            old_output_unchanged=True, force_resume=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
