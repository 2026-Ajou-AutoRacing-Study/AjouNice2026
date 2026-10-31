#!/usr/bin/env python3
"""빌린 PC의 동결 ROS를 유지하고 공통 수집기만 격리 실행한다.

설치·배포·기존 수집 수정 기능은 없다. 준비는 CPU 인계만 수행하고,
실제 소량 검증/본수집은 각각 qualify/run과 Sunny13 확인을 명시해야 한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time


WORKSPACE = 'ajounice_base5_capture'
RUNTIME_REL = 'work/unified-runtime-v1'
SCHEMA = 'base5_unified_borrowed_deployment_v1'
LIVE_SHA = '7850aed3a4a2881393af75ed2da96752e8fe292606c837abd5790281e481fd00'
LOGICAL_SCRIPTS = Path('/home/luv/AjouNice2026/dataset/morai_od/scripts')
OLD_OUTPUTS = ('sunny13_5k', 'sunny13_5k_recovery_v2', 'smoke_v1')
NEW_OUTPUT = 'sunny13_5k_unified_v1'
EXTRA_SCRIPTS = {
    'base5_environment_qualification.py', 'base5_static_surface_attitude_candidate.py',
    'finalize_base5_environment_release.py', 'prepare_base5_environment_rgb_pilot.py',
    'prepare_base5_environment_visibility_recovery.py', 'prepare_base5_unit_environment.py',
    'prepare_base5_unit_environment_smoke.py', 'report_base5_environment_rgb.py',
    'run_base5_batch_static_attitude.py', 'run_base5_capture_campaign_direction_reference.py',
    'run_base5_capture_static_attitude.py',
}
REQUIRED_HELPERS = {
    'unified_borrowed.py', 'unified_capture.py', 'session_input_boundary.py',
    'unified_paused_import.py', 'recovery_import.py', 'recovery_population.py', 'route_compat.py',
}


def require(value, message):
    if not value:
        raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink path required: '+str(path))
    return path


def owned(value, directory=False):
    path = physical(value)
    info = path.stat()
    require(info.st_uid == os.getuid() and not info.st_mode & 0o022,
            'Private owned deployment required: '+str(path))
    require(stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode),
            'Unexpected deployment path type: '+str(path))
    return path


def sha(path):
    value = hashlib.sha256()
    with physical(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            value.update(block)
    return value.hexdigest()


def read(path):
    return json.loads(owned(path).read_text())


def load_original(workspace):
    """원형 live의 __file__/contract/서비스 신원을 바꾸지 않는다."""
    path = owned(workspace/'checks/live_v2.py')
    require(sha(path) == LIVE_SHA, 'Historical live_v2 bytes changed')
    spec = importlib.util.spec_from_file_location('_base5_original_live_for_unified', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_inventory(root, pins, label):
    root = owned(root, True)
    require(type(pins) is dict and pins, 'Nonempty '+label+' pins required')
    entries = list(root.iterdir())
    require(all(p.suffix == '.py' and p.is_file() and not p.is_symlink() for p in entries),
            label+' must contain only the flat pinned Python files')
    require({str(p) for p in entries} == set(pins), 'Unexpected '+label+' file inventory')
    for filename, expected in pins.items():
        path = owned(filename)
        require(path.parent == root and type(expected) is str and len(expected) == 64
                and sha(path) == expected, label+' SHA mismatch: '+filename)


def paths(workspace):
    root = workspace/RUNTIME_REL
    return dict(root=root, scripts=root/'scripts', checks=root/'checks',
        deployment=root/'deployment.json', profile=root/'sunny13_profile.json',
        old=workspace/'collections/sunny13_5k_recovery_v2',
        output=workspace/'collections'/NEW_OUTPUT,
        intent=workspace/'checks/population_recovery_intent.json')


def load(workspace=None, verify=True):
    workspace = owned(Path.home()/WORKSPACE if workspace is None else workspace, True)
    require(workspace.name == WORKSPACE and workspace.parent == Path.home().resolve(),
            'Only the explicitly approved HOME workspace is supported')
    p = paths(workspace)
    require(physical(__file__) == p['checks']/'unified_borrowed.py',
            'Run the separately deployed unified launcher; never rewrite the old checks')
    deployment = read(p['deployment'])
    require(deployment.get('schema') == SCHEMA and deployment.get('workspace') == str(workspace)
            and deployment.get('environment') == 'sunny13', 'Unexpected unified deployment scope')
    verify_inventory(p['scripts'], deployment.get('runtime_hashes'), 'common scripts')
    verify_inventory(p['checks'], deployment.get('helper_hashes'), 'unified helpers')
    require(REQUIRED_HELPERS <= {Path(name).name for name in deployment['helper_hashes']},
            'Required common helpers are missing')
    old = load_original(workspace)
    ctx = old.load(workspace, verify=verify)
    old.ready(ctx)
    old.verify_qualification(ctx)
    intent = read(p['intent'])
    require(sha(p['intent']) == deployment.get('recovery_intent_sha256')
            and intent.get('workspace') == str(workspace) and intent.get('environment') == 'sunny13'
            and intent.get('output') == str(p['old'])
            and intent.get('scripts') == str(LOGICAL_SCRIPTS)
            and intent.get('manifest') == ctx['graph']['manifest']['source']
            and intent.get('release') == ctx['graph']['release']['source'],
            'Historical recovery intent/source identity changed')
    witness = workspace/'checks/canonical_route_v1.json'
    require(intent.get('witness') == str(witness) and sha(witness) == intent.get('witness_sha256'),
            'Historical route witness changed')
    # 427/438은 dataset scripts만의 수가 아니다. 기존 BEVFusion runtime 67개는
    # 원형 bundle의 logical binding에 그대로 남고 dataset scripts만 360→371개다.
    originals = {entry['source']:entry['sha256'] for entry in ctx['graph']['files'].values()
                 if 'runtime' in entry.get('roles', [])}
    source_runtime = deployment.get('source_runtime_hashes')
    require(type(source_runtime) is dict and len(originals) == 427 and len(source_runtime) == 438
            and set(source_runtime) == set(originals)|{str(LOGICAL_SCRIPTS/name) for name in EXTRA_SCRIPTS}
            and all(source_runtime.get(name) == digest for name, digest in originals.items()),
            'Common runtime must preserve all 427 originals and exactly 11 reviewed additions')
    expected_scripts = {str(p['scripts']/Path(name).name):digest for name,digest in source_runtime.items()
                        if Path(name).parent == LOGICAL_SCRIPTS}
    require(len(expected_scripts) == 371 and deployment['runtime_hashes'] == expected_scripts,
            'Common dataset overlay must contain exactly the 371 qualified scripts')
    for name in OLD_OUTPUTS:
        owned(workspace/'collections'/name, True)
    return dict(ctx, old_live=old, deployment=deployment, paths=p, intent=intent)


def environment(ctx):
    env = ctx['old_live'].environment(ctx)
    env['PYTHONPATH'] = str(ctx['paths']['checks'])+':'+env['PYTHONPATH']
    return env


def namespace(ctx, command):
    argv = ctx['old_live'].namespace(ctx, command)
    index = argv.index('--chdir')
    p = ctx['paths']
    mounts = ['--ro-bind', str(p['scripts']), str(LOGICAL_SCRIPTS),
              '--ro-bind', str(p['scripts']), str(p['scripts']),
              '--ro-bind', str(p['checks']), str(p['checks']),
              '--ro-bind', str(p['deployment']), str(p['deployment']),
              '--ro-bind', str(ctx['workspace']/'checks'), str(ctx['workspace']/'checks')]
    for name in OLD_OUTPUTS:
        path = ctx['workspace']/'collections'/name
        mounts += ['--ro-bind', str(path), str(path)]
    for name in ('host_contract.json', 'qualification.json'):
        path = ctx['session']/name
        mounts += ['--ro-bind', str(path), str(path)]
    # workdir 자체는 writable이다. CPU prepare가 고정된 profile을 새로 기록한다.
    return argv[:index]+mounts+argv[index:]


def indices(value):
    require(type(value) is str and value, 'Explicit bounded qualification indices required')
    words = value.split(',')
    require(all(word.isascii() and word.isdecimal() for word in words), 'Decimal comma-separated indices required')
    result = [int(word) for word in words]
    require(1 <= len(result) <= 32 and len(result) == len(set(result))
            and all(0 <= number < 5000 for number in result), 'Qualification must select 1–32 unique source indices')
    return ','.join(map(str, result))


def validate_profile(ctx):
    p = ctx['paths']
    doc = read(p['profile'])
    require(type(doc) is dict and type(doc.get('route_witness')) is dict,
            'Explicit unified profile with a route witness is required')
    require(doc.get('environment') == 'sunny13' and doc.get('old_output') == str(p['old'])
            and doc.get('output') == str(p['output']) and doc.get('manifest') == ctx['intent']['manifest']
            and doc.get('release') == ctx['intent']['release']
            and doc.get('route_witness', {}).get('path') == ctx['intent']['witness']
            and doc.get('route_witness', {}).get('sha256') == ctx['intent']['witness_sha256'],
            'Unified prepared profile escaped its approved borrowed-host scope')
    return doc


def command(ctx, action, selected=None, stop_after_chunks=0):
    p = ctx['paths']
    argv = ['/usr/bin/python3', '-B', str(p['checks']/'unified_capture.py'), '--profile', str(p['profile'])]
    if action == 'prepare':
        require(not p['profile'].exists() and not p['output'].exists(), 'Fresh unified handoff required; never re-prepare collected data')
        return argv+['--prepare', '--manifest', ctx['intent']['manifest'], '--release', ctx['intent']['release'],
            '--old-output', str(p['old']), '--output', str(p['output']), '--route-witness', ctx['intent']['witness']]
    if action == 'probe':
        code = ('import json; import unified_capture as common; common.modules(); '
                'print(json.dumps(dict(event="unified_borrowed_import_probe_pass",'
                'native_captures=0,ros_started=False,bulk_started=False)))')
        return ['/usr/bin/python3', '-B', '-c', code]
    require(type(stop_after_chunks) is int and stop_after_chunks >= 0, 'Nonnegative chunk bound required')
    require(action in ('plan', 'run', 'qualify', 'refresh'), 'Unknown common command')
    validate_profile(ctx)
    if action == 'plan':
        return argv
    if action == 'refresh':
        require(selected is None and stop_after_chunks == 0,
                'CPU profile refresh cannot accept capture selection flags')
        return argv+['--refresh-prepared-profile']
    if action == 'qualify':
        require(stop_after_chunks == 0, 'Qualification uses explicit indices, not bulk chunk flags')
        return argv+['--qualify-prototype', '--indices', indices(selected), '--confirmed-environment', 'sunny13']
    require(selected is None, 'Bulk cannot accept qualification indices')
    return argv+['--run', '--confirmed-environment', 'sunny13', '--stop-after-chunks', str(stop_after_chunks)]


def request_pause(ctx):
    root = owned(ctx['paths']['output'], True)
    marker = physical(root/'PAUSE')
    old = ctx['old_live']
    if marker.exists():
        owned(marker)
    else:
        old.write_new(marker, dict(reason='operator_graceful_pause', requested_at=time.time(),
            semantics='finish current common chunk and verify Park; do not stop ROS/bridge'))
    return marker


def require_services(ctx):
    old = ctx['old_live']
    for role, port in (('ros', 11311), ('bridge', 9090)):
        require(old.port_open(port) and old.service_owned(ctx, role),
                'Start the existing approved '+role+' terminal; do not reuse a foreign service')


def supervise(ctx, action, argv):
    """구 capture.lock과 직렬화하며 Ctrl-C도 native 안전 종료를 기다린다."""
    old = ctx['old_live']
    path = physical(ctx['session']/'capture.lock')
    if path.exists():
        owned(path)
    fd = os.open(path, os.O_RDWR|os.O_CREAT|os.O_APPEND|os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        if action in ('run', 'qualify'):
            require_services(ctx)
        child, previous = None, {}
        def interrupt(signum, frame):
            if child is not None and child.poll() is None:
                if action == 'run':
                    marker = request_pause(ctx)
                    print(json.dumps(dict(event='graceful_pause_requested', marker=str(marker))), flush=True)
                else:
                    old.signal_inner_app(child, argv, signal.SIGINT)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.signal(signum, interrupt)
        try:
            child = subprocess.Popen(namespace(ctx, argv), env=environment(ctx), start_new_session=True)
            return child.wait()
        finally:
            if child is not None and child.poll() is None:
                interrupt(signal.SIGINT, None)
                child.wait()
            for signum, handler in previous.items():
                signal.signal(signum, handler)


def snapshot(ctx):
    p = ctx['paths']; path = p['output']/'status.json'
    return dict(event='unified_borrowed_status', output=str(p['output']),
        prepared=p['profile'].is_file(), state=read(path) if path.is_file() else None,
        files_written=False, capture_started=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('action', choices=('plan', 'probe', 'prepare', 'refresh', 'status', 'qualify', 'run'))
    parser.add_argument('--confirmed-environment', choices=('sunny13',))
    parser.add_argument('--indices')
    parser.add_argument('--stop-after-chunks', type=int, default=0)
    args = parser.parse_args(argv)
    require(args.stop_after_chunks >= 0 and (args.action == 'run' or args.stop_after_chunks == 0),
            'Chunk bound only applies to operator bulk run')
    require((args.indices is not None) == (args.action == 'qualify'), 'Only qualification accepts explicit indices')
    require((args.confirmed_environment is not None) == (args.action in ('run', 'qualify')),
            'Actual MORAI UI environment confirmation is required only for capture commands')
    ctx = load(verify=args.action != 'status')
    if args.action == 'status':
        print(json.dumps(snapshot(ctx), ensure_ascii=False, indent=2))
        return 0
    argv = command(ctx, args.action, args.indices, args.stop_after_chunks)
    if args.action == 'plan':
        print(json.dumps(dict(event='unified_borrowed_plan', command=argv, files_written=False,
            capture_started=False, original_services_unchanged=True), ensure_ascii=False, indent=2))
        return 0
    if args.action == 'probe':
        return subprocess.run(namespace(ctx, argv), env=environment(ctx), check=False).returncode
    return supervise(ctx, args.action, argv)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(event='unified_borrowed_blocked', error=str(error),
            fallback_allowed=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
