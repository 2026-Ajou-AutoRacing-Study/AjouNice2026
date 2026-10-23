#!/usr/bin/env python3
"""원래 강화 source의 여섯 native 검사를 새 환경에 명시적으로 연결한다.

검증된 orchestration 본문은 SHA로 동결해 재사용한다. 새 환경의 native 증거를
과거 환경으로 대체하지 않으며, worker마다 같은 환경·본문·입력 pin을 확인한다.
본수집은 실행하지 않는다. native 단계에는 별도 --run/환경 확인이 필요하다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path

PROJECT = Path('/home/luv/AjouNice2026')
TEMPLATE = PROJECT/'artifacts/evaluation/base5_foggy13_design_261019/local_workflow.py'
TEMPLATE_SHA256 = 'e255792210e6aa98c47ab11beb57704ff86d2195ff801f871e95c17de187f482'
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
INTENT_SCHEMA = 'base5_reinforced_environment_bounded_native_intent_v1'


def require(value, message):
    if not value:
        raise RuntimeError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not item.is_symlink() for item in (path, *path.parents)),
            'Exact nonsymlink orchestration input required')
    return path


def sha(path):
    return hashlib.sha256(physical(path).read_bytes()).hexdigest()


def parse_environment(argv):
    """Worker environment is provisional until the original exact intent check."""
    if '--worker' in argv:
        parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
        parser.add_argument('--intent', type=Path, required=True)
        args, _ = parser.parse_known_args(argv)
        document = json.loads(physical(args.intent).read_text())
        environment = document.get('environment')
        require(environment in ENVIRONMENTS, 'Explicit valid worker environment required')
        return environment, list(argv)
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    args, remaining = parser.parse_known_args(argv)
    require(argv.count('--environment') == 1 and not any(value.startswith('--environment=') for value in argv),
            'One canonical explicit environment flag required')
    return args.environment, remaining


def load_workflow(environment):
    require(environment in ENVIRONMENTS, 'One of the six environments is required')
    require(sha(TEMPLATE) == TEMPLATE_SHA256, 'Original six-gate orchestration bytes changed')
    spec = importlib.util.spec_from_file_location('_base5_frozen_reinforced_environment_workflow', TEMPLATE)
    workflow = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(workflow)
    # Original functions consult their own module globals at call time. Only
    # identity/paths change; original native/QA/adoption bodies remain identical.
    workflow.ENVIRONMENT = environment
    workflow.INTENT_SCHEMA = INTENT_SCHEMA
    workflow.__file__ = str(physical(__file__))
    workflow.DESIGN = PROJECT/('artifacts/evaluation/base5_'+environment+'_design_261019')
    workflow.DEFAULT_OUTPUT = workflow.DESIGN/'qualification_v1'
    workflow.DEFAULT_CAPTURE = Path('/home/luv/morai_cap')/('base5_'+environment+'_final_v163_261019/collection_5k_v1')
    workflow.__doc__ = 'Explicit '+environment+' reinforced source qualification; original native/six gates remain unchanged.'
    original_intent = workflow.intent_for
    original_verify = workflow.verify_intent
    original_save = workflow.save

    def intent_for(*args, **kwargs):
        document = original_intent(*args, **kwargs)
        document['pins'][str(TEMPLATE)] = TEMPLATE_SHA256
        document['orchestration_template'] = dict(path=str(TEMPLATE), sha256=TEMPLATE_SHA256)
        return document

    def verify_intent(filename, expected):
        # The existing checker validates the full intent SHA and all pins,
        # canonical/subset membership, output bounds, runner and explicit limits.
        document = original_verify(filename, expected)
        require(document.get('orchestration_template') == dict(path=str(TEMPLATE), sha256=TEMPLATE_SHA256)
                and document.get('pins', {}).get(str(TEMPLATE)) == TEMPLATE_SHA256,
                'Native worker lacks the exact original orchestration authority')
        return document

    def save(filename, document):
        if document.get('schema') == 'base5_foggy13_common_command_plan_v1':
            document = dict(document, schema='base5_reinforced_environment_common_command_plan_v1',
                            environment=environment)
        return original_save(filename, document)

    workflow.intent_for = intent_for
    workflow.verify_intent = verify_intent
    workflow.save = save
    return workflow


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    environment, remaining = parse_environment(argv)
    previous = os.umask(0o077)
    try:
        return load_workflow(environment).main(remaining)
    finally:
        os.umask(previous)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(event='reinforced_environment_workflow_blocked', error=str(error),
                              main_started=False), ensure_ascii=False), flush=True)
        raise SystemExit(1)
