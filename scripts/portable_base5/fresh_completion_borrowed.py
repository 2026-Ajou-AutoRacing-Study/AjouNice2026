#!/usr/bin/env python3
"""독립 환경의 정상 종료 부모에 공통 유연 보충을 격리 연결한다.

기존 fresh 배포·프로파일·native 원본은 읽기 전용이다. 별도 SHA 고정
배포에서 기존 prepare_seed/complete_unified/flexible_complete를 그대로 쓴다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import pwd
import stat

SCHEMA = 'base5_fresh_borrowed_completion_deployment_v1'
FILES = {'fresh_completion_borrowed.py', 'complete_borrowed.py',
         'complete_unified.py', 'flexible_complete.py', 'overlay_read_cache.py'}
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')


def require(value, message):
    if not value:
        raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink completion path required')
    return path


def owned(value, directory=False):
    path = physical(value); info = path.stat()
    require(info.st_uid == os.getuid() and not info.st_mode & 0o022
            and (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)),
            'Private owned completion input required: '+str(path))
    return path


def sha(value):
    digest = hashlib.sha256()
    with owned(value).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(value):
    return json.loads(owned(value).read_text())


def scope_for(root, environment):
    root = owned(root, True)
    account_home = Path(pwd.getpwuid(os.getuid()).pw_dir).resolve()
    workspace = root.parents[1]
    require(workspace.name == 'ajounice_base5_capture' and workspace.parent == account_home
            and root.parent == workspace/'work', 'Approved separate borrowed work deployment required')
    require(physical(__file__) == root/'fresh_completion_borrowed.py', 'Run the deployed completion wrapper')
    deployment_path = root/'deployment.json'; doc = read(deployment_path)
    require(doc.get('schema') == SCHEMA and doc.get('environment') == environment
            and doc.get('workspace') == str(workspace), 'Explicit matching environment/workspace required')
    pins = doc.get('helper_hashes')
    expected = {str(root/name) for name in FILES}
    require(type(pins) is dict and set(pins) == expected
            and {str(p) for p in root.glob('*.py')} == expected
            and all(sha(name) == digest for name, digest in pins.items()),
            'Exact isolated completion helper inventory/SHA required')
    fresh_deployment = owned(doc['fresh_deployment'])
    require(fresh_deployment.parent.parent == workspace/'work'
            and fresh_deployment.parent != root
            and sha(fresh_deployment) == doc['fresh_deployment_sha256'],
            'Preserved original fresh deployment changed')
    fresh_doc = read(fresh_deployment)
    checks = owned(fresh_deployment.parent/'checks', True)
    fresh_helper = owned(checks/'fresh_borrowed.py')
    require(fresh_doc.get('helper_hashes', {}).get(str(fresh_helper)) == sha(fresh_helper)
            and fresh_doc.get('environment') == environment
            and fresh_doc.get('workspace') == str(workspace), 'Original fresh helper identity changed')
    profile = owned(doc['source_profile'])
    require(profile == fresh_deployment.parent/'profile.json'
            and sha(profile) == doc['source_profile_sha256'], 'Original frozen profile changed')
    previous = read(profile)
    baseline, seed, output = map(physical, (doc['baseline'], doc['seed'], doc['output']))
    require(previous.get('environment') == environment and previous.get('output') == str(baseline)
            and previous.get('manifest') == fresh_doc.get('manifest')
            and previous.get('manifest_sha256') == fresh_doc.get('manifest_sha256')
            and previous.get('release') == fresh_doc.get('release')
            and previous.get('release_sha256') == fresh_doc.get('release_sha256')
            and all(p.parent == workspace/'collections' and p.name.startswith(environment+'_')
                    for p in (baseline, seed, output)) and len({baseline, seed, output}) == 3,
            'Independent same-environment baseline/seed/output required')
    return dict(root=root, workspace=workspace, profile=profile, previous=previous,
                baseline=baseline, seed=seed, output=output, pins=pins,
                deployment_path=deployment_path, deployment_sha256=sha(deployment_path),
                fresh_deployment=fresh_deployment, fresh_helper=fresh_helper, checks=checks,
                environment=environment)


def load_components(scope):
    # Import common from its original frozen fresh checks path, never replace it.
    sys.path.insert(0, str(scope['checks']))
    sys.path.insert(0, str(scope['root']))
    import complete_borrowed as seed_helper
    import complete_unified as exact
    import flexible_complete as flexible
    require(physical(exact.__file__) == scope['root']/'complete_unified.py'
            and physical(flexible.__file__) == scope['root']/'flexible_complete.py'
            and physical(exact.common.__file__) == scope['checks']/'unified_capture.py',
            'Completion imports must retain the original frozen common runtime')
    spec = importlib.util.spec_from_file_location('_base5_fresh_completion_platform', scope['fresh_helper'])
    fresh = importlib.util.module_from_spec(spec); spec.loader.exec_module(fresh)
    return fresh, seed_helper, exact, flexible


@contextmanager
def namespace_for_completion(fresh, scope):
    original = fresh.namespace
    def namespace(ctx, argv):
        command = original(ctx, argv); index = command.index('--chdir')
        mounts = ['--ro-bind', str(scope['root']), str(scope['root']),
                  '--ro-bind', str(scope['profile']), str(scope['profile']),
                  '--ro-bind', str(scope['baseline']), str(scope['baseline'])]
        # The exact seed lock is append-opened, but five control records are
        # pinned by the unmodified flexible summary before and after capture.
        if scope['seed'].exists():
            mounts += ['--bind', str(scope['seed']), str(scope['seed'])]
            for name in ('state.json', 'status.json', 'quota.json', 'selected_ledger.jsonl',
                         'adopted_manifest.json', 'zero_capture_seed.json'):
                path = owned(scope['seed']/name)
                mounts += ['--ro-bind', str(path), str(path)]
        return command[:index]+mounts+command[index:]
    fresh.namespace = namespace
    try:
        yield
    finally:
        changed = fresh.namespace is not namespace
        fresh.namespace = original
        require(not changed, 'Completion namespace ownership changed')


def parser():
    p = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    p.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    p.add_argument('action', choices=('plan', 'prepare', 'run', 'finalize', 'status'))
    p.add_argument('--confirmed-environment', choices=ENVIRONMENTS)
    p.add_argument('--attempts', type=int, default=3)
    p.add_argument('--time-budget-sec', type=int, default=7200)
    p.add_argument('--stop-after-cells', type=int, default=0)
    p.add_argument('--inside', action='store_true', help=argparse.SUPPRESS)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    require(1 <= args.attempts <= 5 and 30 <= args.time_budget_sec <= 7200
            and args.stop_after_cells >= 0, 'Finite completion budgets required')
    require((args.action == 'run') == (args.confirmed_environment is not None)
            and (args.confirmed_environment is None or args.confirmed_environment == args.environment),
            'Only actual native run requires explicit matching environment confirmation')
    scope = scope_for(Path(__file__).absolute().parent, args.environment)
    fresh, seed_helper, exact, flexible = load_components(scope)
    if args.inside:
        require(args.action in ('prepare', 'run', 'finalize'), 'Inside dispatch must execute explicit work')
        seed_helper.prepare_seed(exact, scope)
        if args.action != 'prepare':
            require(scope['output'].is_dir(), 'Run prepare before native completion')
        if args.action == 'run':
            seed_helper.resume_pause(exact, scope)
        command = ['--profile', str(scope['profile']), '--parent', str(scope['seed']),
                   '--output', str(scope['output']), '--attempts', str(args.attempts),
                   '--time-budget-sec', str(args.time_budget_sec), '--stop-after-cells', str(args.stop_after_cells)]
        command += {'prepare':['--prepare', '--stop-after-cells', '1'],
                    'run':['--run', '--confirmed-environment', args.environment],
                    'finalize':['--finalize']}[args.action]
        print(json.dumps(flexible.execute(flexible.parser().parse_args(command)), ensure_ascii=False), flush=True)
        return 0
    ctx = fresh.load(scope['fresh_deployment'], args.environment)
    require(fresh.validate_profile(ctx) == scope['previous'], 'Completion original fresh profile differs')
    if args.action in ('plan', 'status'):
        path = scope['output']/'status.json'
        print(json.dumps(dict(event='fresh_borrowed_completion_'+args.action,
            baseline=str(scope['baseline']), seed=str(scope['seed']), output=str(scope['output']),
            parent_status=read(scope['baseline']/'status.json'), status=read(path) if path.is_file() else None,
            captures_started=False, files_written=False, existing_native_qa_recomputed=False), ensure_ascii=False, indent=2))
        return 0
    command = ['/usr/bin/python3', '-B', '-u', str(scope['root']/'fresh_completion_borrowed.py'),
               '--environment', args.environment, args.action, '--inside', '--attempts', str(args.attempts),
               '--time-budget-sec', str(args.time_budget_sec), '--stop-after-cells', str(args.stop_after_cells)]
    if args.action == 'run':
        command += ['--confirmed-environment', args.environment]
    updated = dict(ctx, paths=dict(ctx['paths'], output=scope['output']))
    previous_umask = os.umask(0o077)
    try:
        with namespace_for_completion(fresh, scope):
            return fresh.supervise(updated, 'run' if args.action == 'run' else 'prepare', command)
    finally:
        os.umask(previous_umask)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        print(json.dumps(dict(event='fresh_borrowed_completion_blocked', error=str(error),
                             preserved_parent_unchanged=True, fallback_allowed=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
