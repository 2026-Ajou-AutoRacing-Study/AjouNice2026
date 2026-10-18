#!/usr/bin/env python3
"""빌린 PC의 완료 Sunny13 승인분을 보존하고 공통 유연 보충을 실행한다.

기존 runtime/checks/profile은 수정하지 않는다. 별도 배포에서 원래 공통
모듈을 그대로 import하고 같은 namespace·capture.lock·안전 종료를 사용한다.
prepare는 빈 exact 계보와 최대 한 개의 CPU 후보만 준비하며 촬영하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import pwd
import stat
import time


SCHEMA = 'base5_borrowed_flexible_completion_deployment_v1'
RECEIPT_SCHEMA = 'base5_zero_capture_exact_completion_seed_v1'
RUNTIME_REL = Path('work/completion-runtime-v1')
COMMON_REL = Path('work/unified-runtime-v1')
FILES = {'complete_borrowed.py', 'complete_unified.py', 'flexible_complete.py', 'overlay_read_cache.py'}
SEED_NAME = 'sunny13_5k_completion_seed_v1'
OUTPUT_NAME = 'sunny13_5k_flexible_v1'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load_modules(root):
    """common.helpers()의 기존 절대 경로와 SHA를 바꾸지 않는다."""
    checks = root.parent/'unified-runtime-v1/checks'
    if str(checks) not in sys.path:
        sys.path.insert(0, str(checks))
    import unified_borrowed as borrowed
    import complete_unified as exact
    import flexible_complete as flexible
    return borrowed, exact, flexible


def deployment(root, borrowed, exact):
    common = exact.common
    root = borrowed.owned(root, True)
    workspace = borrowed.owned(root.parents[1], True)
    # The ROS namespace intentionally supplies an isolated HOME. The read-only
    # system account database, not that per-session environment, owns the bundle.
    account_home = Path(pwd.getpwuid(os.getuid()).pw_dir).resolve()
    require(root == workspace/RUNTIME_REL and workspace.name == borrowed.WORKSPACE
        and workspace.parent == account_home, 'Run only the approved borrowed-host completion deployment')
    require(common.physical(__file__) == root/'complete_borrowed.py', 'Separate completion wrapper path required')
    path = root/'deployment.json'
    doc = borrowed.read(path)
    require(doc.get('schema') == SCHEMA and doc.get('workspace') == str(workspace)
        and doc.get('environment') == 'sunny13', 'Completion deployment scope changed')
    expected = {str(root/name) for name in FILES}
    pins = doc.get('helper_hashes')
    require(isinstance(pins, dict) and set(pins) == expected
        and {str(p) for p in root.glob('*.py')} == expected, 'Completion helper inventory changed')
    require(all(borrowed.sha(borrowed.owned(name)) == digest for name,digest in pins.items()),
            'Completion helper SHA changed')
    require(Path(exact.__file__).absolute() == root/'complete_unified.py', 'Exact helper must use the isolated deployment')
    source = common.physical(doc['source_profile'])
    require(workspace/COMMON_REL in source.parents and source.name == 'sunny13_profile.json'
        and borrowed.sha(borrowed.owned(source)) == doc.get('source_profile_sha256'),
        'Original common profile escaped scope or changed')
    previous = common.validate_profile(common.read(source))
    baseline = common.physical(previous['output'])
    seed = common.physical(doc['seed'])
    output = common.physical(doc['output'])
    collections = workspace/'collections'
    require(doc.get('baseline') == str(baseline) and baseline.parent == seed.parent == output.parent == collections
        and seed.name == SEED_NAME and output.name == OUTPUT_NAME
        and len({baseline,seed,output}) == 3, 'Separate fixed completion outputs required')
    return dict(root=root,workspace=workspace,profile=source,previous=previous,
        baseline=baseline,seed=seed,output=output,pins=pins,deployment_path=path,
        deployment_sha256=borrowed.sha(path))


def seed_pins(exact, folder):
    return {str(folder/name):exact.common.sha(folder/name) for name in
        ('state.json','status.json','quota.json','selected_ledger.jsonl','adopted_manifest.json')}


def prepare_seed(exact, scope):
    """원래 exact 형식의 0회 시도 checkpoint다. 기존 native QA를 재실행하지 않는다."""
    common = exact.common
    folder = scope['seed']
    if folder.exists():
        receipt = exact.read(folder/'zero_capture_seed.json')
        require(receipt.get('schema') == RECEIPT_SCHEMA and receipt.get('native_captures') == 0
            and receipt.get('seed') == str(folder)
            and receipt.get('source_profile_sha256') == common.sha(scope['profile'])
            and receipt.get('baseline') == str(scope['baseline'])
            and all(common.sha(name) == digest for name,digest in receipt.get('pins',{}).items())
            and receipt.get('pins') == seed_pins(exact,folder), 'Existing CPU seed changed; never overwrite')
        state = exact.read(folder/'state.json')
        require(state.get('stage') == 'prepared' and state.get('attempts') == [],
                'CPU seed cannot contain native attempts')
        return receipt
    doc,baseline,output,deck,selected,deficits,pins = exact.summary(scope['profile'],folder)
    require(baseline == scope['baseline'] and output == folder, 'Seed parent changed')
    boundary = exact.verify_baseline(doc,baseline,selected,deficits)
    try:
        with common.handoff.idle_locks(baseline):
            require(all(common.sha(name) == digest for name,digest in pins.items()), 'Terminal parent changed')
            folder.mkdir(mode=0o700,parents=False,exist_ok=False)
            with (folder/'.completion.lock').open('a') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                code_pins = {str(Path(exact.__file__).absolute()):common.sha(exact.__file__),
                    str(Path(exact.overlay_read_cache.__file__).absolute()):common.sha(exact.overlay_read_cache.__file__),
                    **common.helpers()}
                contract = dict(policy=exact.POLICY,baseline=str(baseline),profile=str(common.physical(scope['profile'])),
                    profile_sha256=common.sha(scope['profile']),baseline_pins=pins,code_pins=code_pins,
                    attempts=2,source_sha256=doc['manifest_sha256'])
                state = dict(contract=contract,stage='prepared',baseline_selected=selected,
                    deficits=[entry['id'] for entry in deficits],source_order=[entry['id'] for entry in deck['scenes']],
                    baseline_elapsed_sec=exact.read(baseline/'status.json')['elapsed_sec'],elapsed_sec=0.,
                    attempts=[],current=None,initial_known_payloads=sorted(exact.prior_payloads(baseline,deck)))
                # Original serializer preserves native selected scenarios/metadata,
                # not the original planned payload of a recovered scene.
                exact.publish(folder,state,deck)
                require(exact.rows(folder/'selected_ledger.jsonl') == selected, 'CPU seed changed approved prefix')
                require(all(common.sha(name) == digest for name,digest in pins.items()), 'CPU seed changed parent evidence')
                receipt = dict(schema=RECEIPT_SCHEMA,baseline=str(baseline),seed=str(folder),
                    source_profile_sha256=common.sha(scope['profile']),native_captures=0,geometry_proposals=0,
                    approved=len(selected),deficit=len(deficits),old_native_qa_recomputed=False,
                    pins=seed_pins(exact,folder))
                common.write_new(folder/'zero_capture_seed.json',receipt)
                return receipt
    finally:
        boundary.verify()


@contextmanager
def preserved_namespace(borrowed, scope):
    original = borrowed.namespace
    def namespace(ctx, argv):
        command = original(ctx,argv)
        index = command.index('--chdir')
        paths = [scope['root'],scope['profile'],scope['baseline']]
        # The main native output is physically read-only. The new zero-capture
        # seed needs its existing .completion.lock append-open, but its five
        # immutable control records are pinned by flexible_complete.summary().
        mounts = [word for path in paths for word in ('--ro-bind',str(path),str(path))]
        return command[:index]+mounts+command[index:]
    borrowed.namespace = namespace
    try:
        yield
    finally:
        changed = borrowed.namespace is not namespace
        borrowed.namespace = original
        require(not changed, 'Completion namespace ownership changed')


def flexible_args(flexible, args, scope):
    command = ['--profile',str(scope['profile']),'--parent',str(scope['seed']),
        '--output',str(scope['output']),'--attempts',str(args.attempts),
        '--time-budget-sec',str(args.time_budget_sec),'--stop-after-cells',str(args.stop_after_cells)]
    if args.action == 'prepare':
        command += ['--prepare','--stop-after-cells','1']
    elif args.action == 'run':
        command += ['--run','--confirmed-environment','sunny13']
    elif args.action == 'finalize':
        command += ['--finalize']
    return flexible.parser().parse_args(command)


def resume_pause(exact, scope):
    """capture.lock 안에서 정상 PAUSE만 이력으로 옮긴다. STOP·halt는 우회하지 않는다."""
    output = scope['output']
    require(not (output/'STOP').exists(), 'STOP is not an automatic resume request')
    marker = output/'PAUSE'
    if not marker.exists() and not marker.is_symlink():
        return None
    exact.common.physical(marker)
    info = marker.lstat()
    require(stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid(), 'Owned regular PAUSE marker required')
    status, state = exact.read(output/'status.json'), exact.read(output/'state.json')
    require(status.get('stage') == state.get('stage') == 'paused'
        and status.get('error') is None and state.get('error') is None
        and status.get('failed') == 0 and not status.get('selected_not_certified')
        and status.get('current') is None and state.get('current') is None
        and state.get('contract',{}).get('parent') == str(scope['seed'])
        and state['contract'].get('profile_sha256') == exact.common.sha(scope['profile']),
        'Only a safe unchanged paused completion can consume PAUSE')
    destination = output/('PAUSE.resumed_'+str(time.time_ns()))
    require(not destination.exists(), 'Pause resume history already exists')
    marker.rename(destination)
    return destination


def parser():
    p = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    p.add_argument('action',nargs='?',default='plan',choices=('plan','prepare','run','finalize','status'))
    p.add_argument('--confirmed-environment',choices=('sunny13',))
    p.add_argument('--attempts',type=int,default=3)
    p.add_argument('--time-budget-sec',type=int,default=7200)
    p.add_argument('--stop-after-cells',type=int,default=0)
    p.add_argument('--inside',action='store_true',help=argparse.SUPPRESS)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    require(1 <= args.attempts <= 5 and 30 <= args.time_budget_sec <= 7200 and args.stop_after_cells >= 0,
            'Finite completion attempt/time/cell budget required')
    require((args.confirmed_environment is not None) == (args.action == 'run'),
            'Only actual native run requires explicit Sunny13 environment confirmation')
    root = Path(__file__).absolute().parent
    borrowed,exact,flexible = load_modules(root)
    scope = deployment(root,borrowed,exact)
    if args.inside:
        require(args.action in ('prepare','run','finalize'), 'Inside dispatch only executes explicit work')
        if args.action == 'prepare':
            prepare_seed(exact,scope)
        else:
            require(scope['seed'].is_dir() and scope['output'].is_dir(), 'Run prepare before native completion')
            prepare_seed(exact,scope)  # existing receipt and control hashes only
        if args.action == 'run':
            resume_pause(exact,scope)  # outer borrowed supervisor owns capture.lock
        result = flexible.execute(flexible_args(flexible,args,scope))
        print(json.dumps(result,ensure_ascii=False),flush=True)
        return 0
    ctx = borrowed.load()
    require(scope['previous']['manifest'] == ctx['intent']['manifest']
        and scope['previous']['release'] == ctx['intent']['release'], 'Completion source differs from approved Sunny13')
    if args.action in ('plan','status'):
        target = scope['output']/'status.json'
        print(json.dumps(dict(event='borrowed_flexible_completion_'+args.action,
            source_profile=str(scope['profile']),baseline=str(scope['baseline']),seed=str(scope['seed']),
            output=str(scope['output']),parent_status=exact.read(scope['baseline']/'status.json'),
            status=exact.read(target) if target.is_file() else None,
            captures_started=False,files_written=False,native_qa_unchanged=True),ensure_ascii=False,indent=2))
        return 0
    command = ['/usr/bin/python3','-B','-u',str(root/'complete_borrowed.py'),args.action,'--inside',
        '--attempts',str(args.attempts),'--time-budget-sec',str(args.time_budget_sec),
        '--stop-after-cells',str(args.stop_after_cells)]
    if args.action == 'run':
        command += ['--confirmed-environment','sunny13']
    updated = dict(ctx,paths=dict(ctx['paths'],output=scope['output'],profile=scope['profile']))
    with preserved_namespace(borrowed,scope):
        return borrowed.supervise(updated,'run' if args.action == 'run' else 'prepare',command)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError,ValueError,RuntimeError) as error:
        print(json.dumps(dict(event='borrowed_flexible_completion_blocked',error=str(error),
            preserved_main_unchanged=True,fallback_allowed=False),ensure_ascii=False),file=sys.stderr)
        raise SystemExit(1)
