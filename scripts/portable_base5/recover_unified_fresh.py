#!/usr/bin/env python3
"""새 환경의 안전 checkpoint를 별도 출력으로 이어 주는 공통 crash 실행기.

기본은 읽기 전용 계획이며 --prepare는 CPU 인계만 수행한다. 기존 halted,
Park false, 원본과 실패 suffix는 변경하지 않는다. --run만 실제 촬영한다.
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

import unified_capture as common
import unified_crash_import as crash


SCHEMA = 'base5_unified_fresh_crash_continuation_v1'


def selection(source_profile, profile, output):
    source, target, output = map(common.physical, (source_profile, profile, output))
    previous = common.validate_profile(common.read(source))
    common.require(previous.get('fresh_start') is True and 'canonical_output' not in previous
        and 'crash_continuation' not in previous,
        'Only an original qualified fresh-environment profile may be handed off')
    old = common.physical(previous['output'])
    common.require(old.parent == output.parent and old != output
        and not common.inside(source, old) and not common.inside(target, old)
        and not common.inside(source, output) and not common.inside(target, output)
        and target != source, 'Separate sibling output and external fresh profile required')
    return dict(source=source, profile=target, previous=previous, old=old, output=output)


def qualification(previous):
    path = common.physical(previous['output'])/'unified_live_qualification.json'
    value = common.read(path)
    common.require(value.get('schema') == 'base5_unified_native_profile_qualification_v1'
        and value.get('passed') is True and value.get('policy') == common.POLICY
        and value.get('environment') == previous['environment']
        and value.get('manifest_sha256') == previous['manifest_sha256']
        and value.get('helper_pins') == previous['helper_pins']
        and value.get('captures') == value.get('approved') == 5
        and value.get('main_output_native_captures') == 0
        and len(value.get('source_indices', [])) == len(set(value['source_indices'])) == 5
        and value.get('actual_pause_resume') is True
        and value.get('completed_replay_no_new_capture') is True
        and value.get('proof_hashes'), 'Exact original five-scene qualification required')
    common.require(common.sha(value['pilot_manifest']) == value['pilot_manifest_sha256'],
        'Original qualification manifest changed')
    for filename, digest in value['proof_hashes'].items():
        common.require(common.sha(filename) == digest, 'Original qualification proof changed: '+filename)
    return path, value


def _extra_pins(s):
    return {str(s['source']):common.sha(s['source']),
        str(common.physical(__file__)):common.sha(__file__),
        str(common.physical(crash.__file__)):common.sha(crash.__file__)}


def prepared(s):
    doc = common.validate_profile(common.read(s['profile']))
    expected = copy.deepcopy(s['previous'])
    expected.update(output=str(s['output']), canonical_output=str(s['old']),
        crash_continuation=dict(schema=SCHEMA, source_profile=str(s['source']),
            source_profile_sha256=common.sha(s['source']), original_output=str(s['old'])))
    expected['fixed_pins'] = common.merge(expected['fixed_pins'], _extra_pins(s))
    expected['import_sha256'] = common.sha(s['output']/'imported_prefix.json')
    common.require(doc == expected, 'Continuation profile/source/runtime changed')
    qpath, _ = qualification(s['previous'])
    receipt = common.read(s['output']/'unified_fresh_crash_continuation.json')
    common.require(receipt.get('schema') == SCHEMA
        and receipt.get('source_profile') == str(s['source'])
        and receipt.get('source_profile_sha256') == common.sha(s['source'])
        and receipt.get('profile') == str(s['profile'])
        and receipt.get('profile_sha256') == common.sha(s['profile'])
        and receipt.get('original_output') == str(s['old'])
        and receipt.get('output') == str(s['output'])
        and receipt.get('qualification_source') == str(qpath)
        and receipt.get('qualification_sha256') == common.sha(qpath)
        and common.sha(s['output']/'unified_live_qualification.json') == common.sha(qpath)
        and receipt.get('qualification_reused_unchanged') is True
        and receipt.get('new_native_qualification_claimed') is False
        and receipt.get('unsafe_suffix_imported') is False
        and receipt.get('captures_started_during_prepare') == 0,
        'Explicit fresh crash continuation receipt changed')
    handoff = s['output']/'unified_crash_handoff.json'
    common.require(receipt.get('handoff_sha256') == common.sha(handoff),
        'Crash handoff receipt changed')
    proof = common.read(handoff)
    common.require(proof.get('schema') == crash.SCHEMA
        and proof.get('original_output') == str(s['old'])
        and proof.get('output') == str(s['output'])
        and proof.get('imported_prefix_sha256') == doc['import_sha256']
        and proof.get('authoritative_rows_unchanged') is True
        and proof.get('original_import_verifier_passed') is True
        and proof.get('unsafe_suffix', {}).get('imported_rows') == 0,
        'Exact safe-prefix crash handoff required')
    return doc, receipt


def _copy_new(source, destination):
    content = Path(source).read_bytes()
    fd = os.open(destination, os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(content)


def prepare(s):
    common.require(not s['profile'].exists() and not s['output'].exists(),
        'Never overwrite or re-prepare a continuation')
    previous = s['previous']
    qpath, _ = qualification(previous)
    # This is still the original empty origin, not a fictitious fresh restart.
    common.verify_fresh_prefix(previous)
    doc = copy.deepcopy(previous)
    doc.update(output=str(s['output']), canonical_output=str(s['old']),
        crash_continuation=dict(schema=SCHEMA, source_profile=str(s['source']),
            source_profile_sha256=common.sha(s['source']), original_output=str(s['old'])))
    doc.pop('import_sha256', None)
    doc['fixed_pins'] = common.merge(doc['fixed_pins'], _extra_pins(s))
    common.validate_profile(doc)
    parent, direction = common.modules()
    boundary = common.boundary_for(doc)
    try:
        with boundary.installed(scripts_dir=common.SCRIPTS, direction_module=direction), \
                common.route_compatibility(doc):
            # Verify source authority against its actual original destination.
            authority = common.source_authority(previous, parent)
            common.require(authority == previous['source_authority'], 'Original source authority changed')
            common.warm_boundary(boundary, doc, direction)
            historical = dict(previous, old_output=str(s['old']))
            summary = crash.export(Path(doc['manifest']), s['old'], s['output'],
                **common.historical_verifiers(historical, parent))
            with common.qualified_handoff(parent, doc, authority):
                common.initialize(parent, doc, authority)
            doc['source_authority'] = authority
            doc['import_sha256'] = common.sha(s['output']/'imported_prefix.json')
            _copy_new(qpath, s['output']/'unified_live_qualification.json')
    finally:
        boundary.verify()
    s['profile'].parent.mkdir(parents=True, mode=0o700, exist_ok=True)
    common.write_new(s['profile'], doc)
    receipt = dict(schema=SCHEMA, source_profile=str(s['source']),
        source_profile_sha256=common.sha(s['source']), profile=str(s['profile']),
        profile_sha256=common.sha(s['profile']), original_output=str(s['old']),
        output=str(s['output']), imported=summary, qualification_source=str(qpath),
        qualification_sha256=common.sha(qpath), qualification_reused_unchanged=True,
        new_native_qualification_claimed=False, unsafe_suffix_imported=False,
        captures_started_during_prepare=0,
        handoff_sha256=common.sha(s['output']/'unified_crash_handoff.json'))
    common.write_new(s['output']/'unified_fresh_crash_continuation.json', receipt)
    prepared(s)
    return dict(event='unified_fresh_crash_continuation_prepared', output=str(s['output']),
        selected=summary['selected_count'], deficits=summary['deficit_count'],
        finished=summary['finished'], captures_started=0, unsafe_suffix_imported=False)


@contextmanager
def installed(s=None):
    """원형 검증/worker capability를 보존하는 명시적 continuation 어댑터."""
    original_authority = common.source_authority
    original_prefix = common.verify_fresh_prefix
    original_payload = common.worker_payload
    original_popen = subprocess.Popen
    common_file = str(common.physical(common.__file__))
    wrapper = str(common.physical(__file__))

    def authority(doc, parent):
        common.require(s is not None, 'Workers cannot re-issue source authority')
        expected, _ = prepared(s)
        common.require(doc == expected, 'Source authority requested for another continuation')
        return original_authority(s['previous'], parent)

    def prefix(doc):
        common.require(s is not None, 'Workers cannot replace historical prefix verification')
        expected, _ = prepared(s)
        common.require(doc == expected, 'Prefix requested for another continuation')
        original_prefix(s['previous'])
        _, importer = common.handoff._modules()
        selected, deficits, elapsed = importer.verify(s['output']/'imported_prefix.json',
            Path(doc['manifest']), list(range(5000)))
        handoff = common.read(s['output']/'unified_crash_handoff.json')
        common.require(len(selected) == handoff['selected_count']
            and len(deficits) == handoff['deficit_count'] and elapsed == handoff['elapsed_sec'],
            'Imported prefix counts/elapsed changed')

    def payload(fd, parent):
        # Reuse EVERY original /proc identity/mode/memfd/seal check, replacing
        # only the explicitly pinned launcher's path for this invocation.
        before = common.__file__
        common.__file__ = wrapper
        try:
            result = original_payload(fd, parent)
        finally:
            changed = common.__file__ != wrapper
            common.__file__ = before
            common.require(not changed, 'Worker identity adapter ownership changed')
        doc = result['profile']
        binding = doc.get('crash_continuation', {})
        common.require(binding.get('schema') == SCHEMA
            and doc.get('fresh_start') is True
            and doc['fixed_pins'].get(wrapper) == common.sha(wrapper),
            'Worker capability lacks the pinned explicit continuation')
        return result

    def popen(command, *args, **kwargs):
        if isinstance(command, (tuple, list)):
            argv = [os.fsdecode(value) for value in command]
            index = 1
            while len(argv) > index and argv[index] in ('-B', '-u'):
                index += 1
            if len(argv) > index and argv[index] == common_file and '--worker' in argv[index+1:]:
                common.require(argv[0] == sys.executable and not kwargs.get('shell'),
                    'Continuation worker interpreter changed')
                argv[index] = wrapper
                command = argv
        return original_popen(command, *args, **kwargs)

    common.source_authority, common.verify_fresh_prefix = authority, prefix
    common.worker_payload, subprocess.Popen = payload, popen
    try:
        yield
    finally:
        changed = (common.source_authority is not authority or common.verify_fresh_prefix is not prefix
            or common.worker_payload is not payload or subprocess.Popen is not popen)
        common.source_authority, common.verify_fresh_prefix = original_authority, original_prefix
        common.worker_payload, subprocess.Popen = original_payload, original_popen
        common.require(not changed, 'Continuation adapter ownership changed')


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--worker' in argv:
        with installed():
            return common.main(argv)
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--prepare', action='store_true')
    action.add_argument('--run', action='store_true')
    action.add_argument('--status', action='store_true')
    parser.add_argument('--source-profile', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confirmed-environment', choices=common.ENVIRONMENTS)
    parser.add_argument('--stop-after-chunks', type=int, default=0)
    args = parser.parse_args(argv)
    common.require((args.confirmed_environment is not None) == args.run
        and (not args.stop_after_chunks or args.run and args.stop_after_chunks > 0),
        'Only actual run accepts explicit environment/bounded chunk controls')
    s = selection(args.source_profile, args.profile, args.output)
    if args.run:
        doc, _ = prepared(s)
        common.require(args.confirmed_environment == doc['environment'], 'Actual MORAI environment differs')
        with s['profile'].open('rb') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
            with installed(s):
                return common.run(doc, args.stop_after_chunks)
    if args.status:
        doc, receipt = prepared(s)
        result = dict(output=doc['output'], state=common.read(s['output']/'status.json'),
            imported=receipt['imported'])
    elif args.prepare:
        with s['source'].open('rb') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
            result = prepare(s)
    else:
        result = crash.plan(s['previous']['manifest'], s['old'], s['output'])
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(event='unified_fresh_crash_continuation_blocked', error=str(error),
            old_output_unchanged=True, force_resume=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
