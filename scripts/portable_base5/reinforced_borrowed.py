#!/usr/bin/env python3
"""강화 source의 logical helper 경로를 보존하는 별도 borrowed 진입점.

기존 fresh_borrowed의 격리·수신·소유권·파일 검사를 그대로 사용한다. 강화
extension의 helper pin이 논리 경로를 포함하므로 payload의 같은 모듈을 우선
import한다. 기존 Sunny11 deployment와 helper는 변경하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import json
import os
from pathlib import Path

import fresh_borrowed as original

LOGICAL_HELPERS = original.PROJECT/'scripts/portable_base5'
LOGICAL_REQUIRED = original.REQUIRED_HELPERS | {
    'reinforcement_profile.py', 'reinforcement_screen.py',
    'reinforcement_qualification.py', 'reinforcement_native_adoption.py',
    'reinforced_environment_workflow.py',
}
WORKFLOW_PHASES = {'adopt', 'register', 'first16', 'snapshot', 'remaining16', 'resume-proof',
    'inputs', 'shadow-register', 'shadow', 'shadow-proof', 'cpu-check', 'freeze', 'common-plan'}
NATIVE_PHASES = {'first16', 'remaining16', 'shadow'}
BASE_ENVIRONMENT = original.environment
BASE_COMMAND = original.command


def environment(ctx):
    result = BASE_ENVIRONMENT(ctx)
    result['PYTHONPATH'] = str(LOGICAL_HELPERS)+':'+result['PYTHONPATH']
    return result


def command(ctx, action, selected=None):
    """Direct script directory takes precedence over PYTHONPATH in Python.

    Run the byte-identical common script at its pinned logical location. Its
    unmodified worker router then uses the same logical path and exact parent
    argv checks. No worker capability or helper-identity check is weakened.
    """
    argv = BASE_COMMAND(ctx, action, selected)
    if action == 'probe':
        return argv
    expected = str(ctx['paths']['checks']/'unified_capture.py')
    original.require(len(argv) >= 3 and argv[2] == expected,
                     'Original common command shape changed; do not reroute another program')
    return argv[:2]+[str(LOGICAL_HELPERS/'unified_capture.py')]+argv[3:]


def validate_helpers(ctx):
    """Both physical deployment and path-bearing logical helper copies are exact."""
    pins = ctx['deployment']['helper_hashes']
    here = original.physical(__file__)
    original.require(pins.get(str(here)) == original.sha(here), 'Reinforced borrowed adapter must be pinned')
    for name in LOGICAL_REQUIRED:
        deployed = ctx['paths']['checks']/name
        logical = LOGICAL_HELPERS/name
        original.require(pins.get(str(deployed)) == original.sha(deployed)
            and ctx['deployment']['input_hashes'].get(str(logical)) == original.sha(deployed)
            and original.sha(original.mapped(ctx, str(logical))) == original.sha(deployed),
            'Logical/deployed reinforced helper mismatch: '+name)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--environment', choices=original.ENVIRONMENTS, required=True)
    parser.add_argument('action', choices=('source-plan', 'plan', 'probe', 'prepare', 'qualify', 'run', 'status', 'workflow'))
    args, remaining = parser.parse_known_args(argv)
    ctx = original.load(args.deployment, args.environment, verify=args.action != 'status')
    validate_helpers(ctx)
    previous_env = original.environment
    previous_command = original.command
    previous_umask = os.umask(0o077)
    original.environment = environment
    original.command = command
    try:
        if args.action != 'workflow':
            return original.main(argv)
        remaining = remaining[1:] if remaining[:1] == ['--'] else remaining
        original.require(remaining and remaining[0] in WORKFLOW_PHASES,
                         'Explicit original bounded qualification phase required')
        runner = ctx['paths']['checks']/'reinforced_environment_workflow.py'
        original.require(ctx['deployment']['helper_hashes'].get(str(runner)) == original.sha(runner),
                         'Actual bounded workflow must be pinned')
        # The same direct-script import rule applies to the qualification
        # runner. validate_helpers binds both copies byte-for-byte above.
        workflow_argv = ['/usr/bin/python3', '-B', str(LOGICAL_HELPERS/runner.name),
                         '--environment', args.environment, *remaining]
        action = 'prototype' if remaining[0] in NATIVE_PHASES else 'prepare'
        return original.supervise(ctx, action, workflow_argv)
    finally:
        original.environment = previous_env
        original.command = previous_command
        os.umask(previous_umask)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(event='reinforced_borrowed_blocked', error=str(error),
                              fallback_allowed=False)), flush=True)
        raise SystemExit(1)
