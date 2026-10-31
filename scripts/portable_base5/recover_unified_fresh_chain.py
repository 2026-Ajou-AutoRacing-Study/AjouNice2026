#!/usr/bin/env python3
"""이미 인계된 fresh 수집의 다음 crash를 새 sibling 출력으로 보존 복구한다.

기존 동결 실행기와 halted 상태는 수정하지 않는다. 기본은 읽기 전용 계획,
--prepare는 원형 CPU 검증/인계, --run만 실제 촬영이다.
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

import recover_unified_fresh as first

common, crash = first.common, first.crash
# Prefix verification can precede common.modules(), including in native workers.
if str(common.SCRIPTS) not in sys.path:
    sys.path.insert(0, str(common.SCRIPTS))

SCHEMA = 'base5_unified_fresh_crash_chain_v1'
RECEIPT = 'unified_fresh_crash_chain.json'


def _resolve(source, seen=()):
    source = common.physical(source)
    common.require(source not in seen and len(seen) < 32,
        'Crash profile lineage is cyclic or exceeds the explicit bound')
    previous = common.validate_profile(common.read(source))
    binding = previous.get('crash_continuation', {})
    common.require(previous.get('fresh_start') is True and 'canonical_output' in previous,
        'An already prepared fresh crash continuation is required')
    ancestor = common.physical(binding.get('source_profile', ''))
    if binding.get('schema') == first.SCHEMA:
        state = first.selection(ancestor, source, previous['output'])
        checked, _ = first.prepared(state)
        return dict(previous=checked, origin_profile=ancestor,
            origin=state['previous'], sources=(source, ancestor), outputs=(Path(checked['output']),
                Path(state['previous']['output'])), depth=1)
    common.require(binding.get('schema') == SCHEMA,
        'Unknown crash continuation schema; no implicit lineage conversion')
    parent = _resolve(ancestor, (*seen, source))
    state = dict(source=ancestor, profile=source, previous=parent['previous'],
        old=common.physical(parent['previous']['output']),
        output=common.physical(previous['output']), origin_profile=parent['origin_profile'],
        origin=parent['origin'], depth=parent['depth']+1)
    checked, _ = prepared(state)
    return dict(previous=checked, origin_profile=parent['origin_profile'], origin=parent['origin'],
        sources=(source, *parent['sources']), outputs=(Path(checked['output']), *parent['outputs']),
        depth=parent['depth']+1)


def selection(source_profile, profile, output):
    source, target, output = map(common.physical, (source_profile, profile, output))
    lineage = _resolve(source)
    previous, old = lineage['previous'], common.physical(lineage['previous']['output'])
    common.require(old.parent == output.parent and output not in lineage['outputs']
        and target not in lineage['sources']
        and all(not common.inside(target, root) and not common.inside(source, root)
            for root in (*lineage['outputs'], output)),
        'New sibling output and separate external profile required')
    return dict(source=source, profile=target, previous=previous, old=old, output=output,
        origin_profile=lineage['origin_profile'], origin=lineage['origin'], depth=lineage['depth']+1)


def _extra_pins(s):
    paths = (s['source'], s['origin_profile'], Path(__file__), Path(first.__file__), Path(crash.__file__))
    return {str(common.physical(path)):common.sha(path) for path in paths}


def _document(s):
    doc = copy.deepcopy(s['previous'])
    doc.update(output=str(s['output']), crash_continuation=dict(schema=SCHEMA,
        source_profile=str(s['source']), source_profile_sha256=common.sha(s['source']),
        original_output=str(s['old']), origin_profile=str(s['origin_profile']),
        origin_profile_sha256=common.sha(s['origin_profile']), depth=s['depth']))
    # canonical_output and old_output remain the FIRST source authority / empty origin.
    doc['fixed_pins'] = common.merge(doc['fixed_pins'], _extra_pins(s))
    doc.pop('import_sha256', None)
    common.validate_profile(doc)
    return doc


def prepared(s):
    doc = common.validate_profile(common.read(s['profile']))
    expected = _document(s)
    expected['import_sha256'] = common.sha(s['output']/'imported_prefix.json')
    common.require(doc == expected, 'Crash chain profile/source/runtime changed')
    qpath, _ = first.qualification(s['origin'])
    receipt = common.read(s['output']/RECEIPT)
    checks = dict(schema=SCHEMA, source_profile=str(s['source']),
        source_profile_sha256=common.sha(s['source']), profile=str(s['profile']),
        profile_sha256=common.sha(s['profile']), original_output=str(s['old']),
        output=str(s['output']), origin_profile=str(s['origin_profile']),
        origin_profile_sha256=common.sha(s['origin_profile']), depth=s['depth'],
        qualification_source=str(qpath), qualification_sha256=common.sha(qpath),
        qualification_reused_unchanged=True, new_native_qualification_claimed=False,
        unsafe_suffix_imported=False, captures_started_during_prepare=0,
        handoff_sha256=common.sha(s['output']/'unified_crash_handoff.json'))
    common.require(all(receipt.get(key) == value for key, value in checks.items())
        and common.sha(s['output']/'unified_live_qualification.json') == common.sha(qpath),
        'Explicit crash chain receipt or original qualification changed')
    proof = common.read(s['output']/'unified_crash_handoff.json')
    common.require(proof.get('schema') == crash.SCHEMA
        and proof.get('original_output') == str(s['old']) and proof.get('output') == str(s['output'])
        and proof.get('imported_prefix_sha256') == doc['import_sha256']
        and proof.get('authoritative_rows_unchanged') is True
        and proof.get('original_import_verifier_passed') is True
        and proof.get('unsafe_suffix', {}).get('imported_rows') == 0,
        'Exact nested safe-prefix handoff required')
    return doc, receipt


def prepare(s):
    common.require(not s['profile'].exists() and not s['output'].exists(),
        'Never overwrite or re-prepare a crash continuation')
    # Validate the predecessor receipt before retaining its canonical authority.
    lineage = _resolve(s['source'])
    common.require(lineage['previous'] == s['previous'] and lineage['origin'] == s['origin'],
        'Crash predecessor changed after selection')
    qpath, _ = first.qualification(s['origin'])
    common.verify_fresh_prefix(s['origin'])
    doc = _document(s)
    parent, direction = common.modules()
    boundary = common.boundary_for(doc)
    try:
        with boundary.installed(scripts_dir=common.SCRIPTS, direction_module=direction), \
                common.route_compatibility(doc):
            authority = common.source_authority(s['origin'], parent)
            common.require(authority == s['previous']['source_authority'],
                'Original canonical source authority changed')
            common.warm_boundary(boundary, doc, direction)
            historical = dict(s['previous'], old_output=str(s['old']))
            summary = crash.export(Path(doc['manifest']), s['old'], s['output'],
                **common.historical_verifiers(historical, parent))
            with common.qualified_handoff(parent, doc, authority):
                common.initialize(parent, doc, authority)
            doc['source_authority'] = authority
            doc['import_sha256'] = common.sha(s['output']/'imported_prefix.json')
            first._copy_new(qpath, s['output']/'unified_live_qualification.json')
    finally:
        boundary.verify()
    s['profile'].parent.mkdir(parents=True, mode=0o700, exist_ok=True)
    common.write_new(s['profile'], doc)
    receipt = dict(schema=SCHEMA, source_profile=str(s['source']),
        source_profile_sha256=common.sha(s['source']), profile=str(s['profile']),
        profile_sha256=common.sha(s['profile']), original_output=str(s['old']),
        output=str(s['output']), origin_profile=str(s['origin_profile']),
        origin_profile_sha256=common.sha(s['origin_profile']), depth=s['depth'], imported=summary,
        qualification_source=str(qpath), qualification_sha256=common.sha(qpath),
        qualification_reused_unchanged=True, new_native_qualification_claimed=False,
        unsafe_suffix_imported=False, captures_started_during_prepare=0,
        handoff_sha256=common.sha(s['output']/'unified_crash_handoff.json'))
    common.write_new(s['output']/RECEIPT, receipt)
    prepared(s)
    return dict(event='unified_fresh_crash_chain_prepared', output=str(s['output']),
        selected=summary['selected_count'], deficits=summary['deficit_count'],
        finished=summary['finished'], depth=s['depth'], captures_started=0, unsafe_suffix_imported=False)


@contextmanager
def installed(s=None):
    original_authority, original_prefix = common.source_authority, common.verify_fresh_prefix
    original_payload, original_popen = common.worker_payload, subprocess.Popen
    common_file, wrapper = map(lambda p:str(common.physical(p)), (common.__file__, __file__))

    def authority(doc, parent):
        common.require(s is not None, 'Workers cannot re-issue source authority')
        expected, _ = prepared(s)
        common.require(doc == expected, 'Authority requested for another crash chain')
        return original_authority(s['origin'], parent)

    def prefix(doc):
        common.require(s is not None, 'Workers cannot replace prefix verification')
        expected, _ = prepared(s)
        common.require(doc == expected, 'Prefix requested for another crash chain')
        original_prefix(s['origin'])
        _, importer = common.handoff._modules()
        selected, deficits, elapsed = importer.verify(s['output']/'imported_prefix.json',
            Path(doc['manifest']), list(range(5000)))
        handoff = common.read(s['output']/'unified_crash_handoff.json')
        common.require(len(selected) == handoff['selected_count']
            and len(deficits) == handoff['deficit_count'] and elapsed == handoff['elapsed_sec'],
            'Nested imported prefix counts/elapsed changed')

    def payload(fd, parent):
        before = common.__file__
        common.__file__ = wrapper
        try:
            result = original_payload(fd, parent)
        finally:
            changed = common.__file__ != wrapper
            common.__file__ = before
            common.require(not changed, 'Worker identity adapter ownership changed')
        doc = result['profile']
        common.require(doc.get('crash_continuation', {}).get('schema') == SCHEMA
            and doc.get('fresh_start') is True
            and doc['fixed_pins'].get(wrapper) == common.sha(wrapper),
            'Worker capability lacks the pinned explicit crash chain')
        return result

    def popen(command, *args, **kwargs):
        if isinstance(command, (tuple, list)):
            argv = [os.fsdecode(value) for value in command]
            index = 1
            while len(argv) > index and argv[index] in ('-B', '-u'):
                index += 1
            if len(argv) > index and argv[index] == common_file and '--worker' in argv[index+1:]:
                common.require(argv[0] == sys.executable and not kwargs.get('shell'),
                    'Crash chain worker interpreter changed')
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
        common.require(not changed, 'Crash chain adapter ownership changed')


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
        print(json.dumps(dict(event='unified_fresh_crash_chain_blocked', error=str(error),
            old_output_unchanged=True, force_resume=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
