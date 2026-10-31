#!/usr/bin/env python3
"""독립 새 환경 수집의 읽기 전용 웹 관측기. 기본은 실행 없는 계획이다.

배포 root/fresh_dashboard.py에 단독 배포한다. 기존 checks·수집기·프로필·
ROS·MORAI·기존 웹 서버를 변경하지 않는다. --run만 별도 관측기를 시작한다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import stat
import subprocess

ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
SCRIPT_SHA = '8fab975eda002a5192987d28b8327cf7c6571ca4566e674d6d56096a906fc8c9'
HTML_SHA = '7e364fd1e2ac25ebd0ceeebc7f8328c395cb88d2d550706f204c65e3e1969c58'


def require(value, message):
    if not value:
        raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink observer path required: '+str(path))
    return path


def owned(value, directory=False):
    path = physical(value); info = path.stat()
    require(info.st_uid == os.getuid() and not info.st_mode & 0o022
            and (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)),
            'Private owned observer input required: '+str(path))
    return path


def sha(value):
    digest = hashlib.sha256()
    with owned(value).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inside(path, root):
    return path == root or root in path.parents


def load_fresh(deployment, environment):
    deployment = owned(deployment)
    require(physical(__file__) == deployment.parent/'fresh_dashboard.py',
            'Deploy the observer beside deployment.json, outside frozen checks')
    helpers = deployment.parent/'checks'
    filename = owned(helpers/'fresh_borrowed.py')
    if str(helpers) not in sys.path:
        sys.path.insert(0, str(helpers))
    spec = importlib.util.spec_from_file_location('_base5_fresh_readonly_observer', filename)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    ctx = module.load(deployment, environment)
    return module, ctx


def ui_pin(ctx, doc, key):
    """새 source의 pin을 우선하며, 충돌하는 이전 원형 UI는 허용하지 않는다."""
    values = []
    for mapping in (ctx['deployment'].get('source_runtime_hashes', {}),
                    ctx['deployment'].get('input_hashes', {}), doc.get('fixed_pins', {}),
                    doc.get('runtime_hashes', {})):
        require(type(mapping) is dict, 'Explicit source pin mappings required')
        if key in mapping:
            values.append(mapping[key])
    # entrypoint closure에 UI가 없는 경우, 해시 검증된 기존 플랫폼 graph의
    # 동일 logical source pin을 사용한다. UI 코드·HTML 자체는 아래서 다시 검산한다.
    for entry in ctx.get('graph', {}).get('files', {}).values():
        if entry.get('source') == key:
            values.append(entry['sha256'])
    require(values and all(value == SCRIPT_SHA for value in values),
            'Frozen dashboard source pin is missing or conflicting')
    return SCRIPT_SHA


def configuration(deployment, environment, port=8075, storage=(), once=False):
    require(environment in ENVIRONMENTS, 'Explicit supported environment required')
    require(type(port) is int and 1024 <= port <= 65535, 'Unprivileged observer port required')
    require(type(once) is bool, 'Explicit snapshot mode required')
    module, ctx = load_fresh(deployment, environment)
    doc = module.validate_profile(ctx)
    paths = ctx['paths']; root = owned(paths['output'], True)
    require(doc.get('schema') == 'base5_unified_capture_v1'
            and doc.get('policy') == 'fixed-input-boundaries-and-common-native-profiles-v1'
            and doc.get('fresh_start') is True and doc.get('environment') == environment
            and doc.get('manifest') == ctx['deployment']['manifest']
            and doc.get('manifest_sha256') == ctx['deployment']['manifest_sha256']
            and doc.get('output') == str(root), 'Exact prepared fresh profile required')
    manifest = module.logical(doc['manifest'])
    require(module.sha(module.mapped(ctx, str(manifest))) == doc['manifest_sha256'],
            'Fresh observer manifest changed')
    script = ctx['workspace']/'work/web-dashboard-sunny13-v1/base5_capture_dashboard.py'
    key = str(module.SCRIPTS/'base5_capture_dashboard.py')
    require(sha(script) == ui_pin(ctx, doc, key), 'Frozen dashboard source changed')
    require(sha(script.with_suffix('.html')) == HTML_SHA, 'Frozen dashboard HTML changed')
    state = physical(paths['root']/('observer-state-port'+str(port)))
    require(not inside(state, root) and not inside(root, state)
            and not inside(state, paths['checks']) and not inside(state, paths['scripts']),
            'Observer state must remain outside capture and frozen helpers')
    ancestors = set()
    for value in storage:
        previous = owned(value, True)
        require(previous.parent == ctx['workspace']/'collections' and previous != root,
                'Storage roots must be separate direct borrowed collections')
        ancestors.add(previous)
    return dict(module=module, ctx=ctx, doc=doc, profile=paths['profile'],
        root=root, manifest=manifest, script=script, state=state,
        history=state/'history.jsonl', storage=sorted(ancestors), port=port, once=once)


def state_directory(config, create=False):
    state = physical(config['state'])
    if create:
        state.mkdir(mode=0o700, exist_ok=True)
    if state.exists():
        require(state.is_dir() and state.stat().st_uid == os.getuid()
                and not state.stat().st_mode & 0o077, 'Private observer state directory required')
        for filename in state.rglob('*'):
            filename = physical(filename); info = filename.stat()
            require(info.st_uid == os.getuid() and not info.st_mode & 0o022
                    and (filename.is_dir() or stat.S_ISREG(info.st_mode) and info.st_nlink == 1),
                    'Observer state must not alias protected source or raw data')


def check_port(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
        try:
            stream.bind(('127.0.0.1', port))
        except OSError as error:
            raise ValueError('포트 '+str(port)+'가 사용 중이야. 기존 대시보드 터미널에서 '
                             'Ctrl+C로 종료하거나 다른 포트를 지정해 줘. 기존 프로세스는 건드리지 않았어.') from error


def dashboard_command(config):
    command = ['/usr/bin/python3', '-B', str(config['script']),
        '--capture-root', str(config['root']), '--manifest', str(config['manifest']),
        '--history-file', str(config['history']), '--port', str(config['port'])]
    for root in config['storage']:
        command += ['--storage-root', str(root)]
    if config['once']:
        command.append('--once')
    return command


def namespace_command(config):
    ctx = config['ctx']; command = config['module'].namespace(ctx, dashboard_command(config))
    require(command.count('--chdir') == 1, 'Exact fresh namespace structure required')
    index = command.index('--chdir')
    return command[:index]+['--ro-bind', str(ctx['workspace']), str(ctx['workspace']),
        '--ro-bind', str(ctx['mr']), str(ctx['mr']),
        '--bind', str(config['state']), str(config['state'])]+command[index:]


def environment(config):
    state = str(config['state'])
    return dict(PATH='/usr/bin:/bin', LANG='C.UTF-8', LC_ALL='C.UTF-8', HOME=state,
        TMPDIR=state, TMP=state, TEMP=state, XDG_CACHE_HOME=state,
        XDG_CONFIG_HOME=state, XDG_DATA_HOME=state, PYTHONDONTWRITEBYTECODE='1',
        PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--deployment', type=Path, required=True)
    parser.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    parser.add_argument('--port', type=int, default=8075)
    parser.add_argument('--storage-root', type=Path, action='append', default=[])
    parser.add_argument('--once', action='store_true', help='With --run, emit one read-only snapshot instead of serving HTTP')
    parser.add_argument('--run', action='store_true', help='Start only the separate read-only observer')
    args = parser.parse_args(argv)
    config = configuration(args.deployment, args.environment, args.port, args.storage_root, args.once)
    state_directory(config)
    if not args.once:
        check_port(config['port'])
    command = namespace_command(config)
    if not args.run:
        print(json.dumps(dict(event='fresh_dashboard_plan', environment=args.environment,
            url='http://127.0.0.1:'+str(config['port']), capture_root=str(config['root']),
            profile=str(config['profile']), manifest=str(config['manifest']),
            history_file=str(config['history']), collection_read_only=True,
            writable_state_only=str(config['state']), files_written=False,
            process_started=False, capture_controls=False, command=command), ensure_ascii=False, indent=2))
        return 0
    state_directory(config, create=True)
    previous = os.umask(0o077)
    try:
        child = subprocess.Popen(command, env=environment(config))
    finally:
        os.umask(previous)
    return child.wait()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        print(json.dumps(dict(event='fresh_dashboard_blocked', error=str(error),
            existing_processes_untouched=True, capture_controls=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
