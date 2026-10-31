#!/usr/bin/env python3
"""완료한 fresh 환경 전달본을 원본 RO·새 export 한 곳 RW로 실행한다.

배포 manifest에 명시한 수집 workspace 외부는 수정하지 않는다. 수집·재개·
센서 조작·네트워크 전송·PNG 삭제는 실행하지 않는다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path, PurePosixPath
import stat
import subprocess

SCHEMA = 'base5_cohort_training_export_deployment_v1'
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
FILES = {'cohort_training_export_host.py', 'cohort_training_export.py', 'borrowed_training_export.py',
    'training_convert_adapter.py', 'pack_training.py', 'finish.py', 'sunny15_conversion_reference.json'}
CONTROL_FILES = {'status.json', 'state.json', 'quota.json', 'selected_ledger.jsonl',
    'adopted_manifest.json', 'layout_split_final/result.json', 'layout_split_final/manifest.json',
    'layout_split_final/derived_selected_ledger.jsonl', 'layout_split_final/split_overlay.json'}
PROJECT = Path('/home/luv/AjouNice2026')


def require(value, message):
    if not value:
        raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
        'Exact nonsymlink host export path required')
    return path


def owned(value, directory=False):
    path = physical(value)
    info = path.stat()
    require(info.st_uid == os.getuid() and not info.st_mode & 0o022
        and (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)),
        'Private owned host export input required: ' + str(path))
    return path


def sha(value):
    digest = hashlib.sha256()
    with owned(value).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(value):
    return json.loads(owned(value).read_text())


def scope(root, environment):
    root = owned(root, True)
    workspace = root.parents[1]
    require(root.parent == workspace / 'work' and workspace.parent == Path.home().resolve()
        and workspace.name == 'ajounice_base5_capture', 'Separate approved HOME work deployment required')
    require(physical(__file__) == root / 'cohort_training_export_host.py', 'Run the deployed CPU host wrapper')
    deployment = read(root / 'deployment.json')
    require(deployment.get('schema') == SCHEMA and deployment.get('environment') == environment
        and environment in ENVIRONMENTS and deployment.get('workspace') == str(workspace),
        'Explicit matching environment/workspace required')
    expected = deployment.get('files')
    require(type(expected) is dict and set(expected) == FILES
        and {name for name in FILES if sha(root / name) != expected[name]} == set(),
        'Exact frozen CPU deployment file inventory/SHA required')
    require({path.name for path in root.glob('*.py')} == {name for name in FILES if name.endswith('.py')},
        'Unexpected Python helper in CPU deployment')
    fresh_deployment = owned(deployment['fresh_deployment'])
    require(fresh_deployment.parent.parent == workspace / 'work' and fresh_deployment.parent != root
        and sha(fresh_deployment) == deployment['fresh_deployment_sha256'],
        'Original fresh deployment changed')
    fresh_doc = read(fresh_deployment)
    checks = owned(fresh_deployment.parent / 'checks', True)
    helper = owned(checks / 'fresh_borrowed.py')
    require(fresh_doc.get('environment') == environment and fresh_doc.get('workspace') == str(workspace)
        and fresh_doc.get('helper_hashes', {}).get(str(helper)) == sha(helper),
        'Exact preserved fresh platform/environment required')
    source = owned(deployment['selected_root'], True)
    export = owned(deployment['export_root'], True)
    require(source.parent == workspace / 'collections' and source.name.startswith(environment + '_')
        and export.parent == workspace / 'work' and export != root
        and export.name.startswith(environment + '-training-export-'),
        'Separate same-environment selected/export roots required')
    usage_roots = [owned(value, True) for value in deployment.get('usage_roots', [])]
    require(1 <= len(usage_roots) <= 4 and len(set(usage_roots)) == len(usage_roots)
        and source in usage_roots and physical(fresh_doc['output']) in usage_roots
        and all(path.parent == workspace / 'collections' and path.name.startswith(environment + '_')
            for path in usage_roots),
        'Budget must include this same-environment source/baseline only; global disk reserve remains enforced')
    pins = deployment.get('source_pins')
    require(type(pins) is dict and set(pins) == CONTROL_FILES
        and all(PurePosixPath(name).as_posix() == name and not PurePosixPath(name).is_absolute()
            and '..' not in PurePosixPath(name).parts and sha(source / name) == digest
            for name, digest in pins.items()), 'Exact finalized native/control receipts required')
    status = read(source / 'status.json')
    require(status.get('stage') == 'complete' and status.get('accepted_candidate') == 5000
        and status.get('review') == 0 and status.get('failed') == 0 and status.get('error') is None
        and status.get('current') is None and status.get('selected_not_certified') is False,
        'Certified complete 5K native selection required')
    quota = read(source / 'quota.json')
    require(quota.get('deficit') == 0 and not quota.get('cells'), 'No unfilled quota may be exported')
    archive = deployment['archive_name']
    require(type(archive) is str and archive.startswith('base5_' + environment + '_5k_jpeg90_upright3_')
        and '/' not in archive and archive.endswith('.tar.zst'), 'Explicit cohort archive identity required')
    return dict(root=root, workspace=workspace, deployment=deployment, source=source,
        export=export, checks=checks, fresh_helper=helper, fresh_deployment=fresh_deployment,
        environment=environment, archive=archive, usage_roots=usage_roots)


def load_platform(scoped):
    sys.path.insert(0, str(scoped['checks']))
    spec = importlib.util.spec_from_file_location('_base5_cohort_cpu_export_platform', scoped['fresh_helper'])
    fresh = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fresh)
    ctx = fresh.load(scoped['fresh_deployment'], scoped['environment'])
    fresh.validate_profile(ctx)
    return fresh, ctx


def command(scoped, action):
    argv = ['/usr/bin/python3', '-B', '-u', str(scoped['root'] / 'cohort_training_export.py'), action,
        '--repo', str(PROJECT), '--export-root', str(scoped['export']), '--cohort', scoped['environment'],
        '--archive-name', scoped['archive'],
        '--selected', str(scoped['source'] / 'layout_split_final/derived_selected_ledger.jsonl'),
        '--split-overlay', str(scoped['source'] / 'layout_split_final/split_overlay.json'),
        '--sensor', str(PROJECT / 'configs/sensors/Sensor_competition_local.json'),
        '--expected-helpers', str(scoped['root'] / 'sunny15_conversion_reference.json')]
    for path in scoped['usage_roots']:
        argv += ['--usage-root', str(path)]
    if action in ('qualify', 'pack'):
        argv.append('--run')
    return argv


def isolated_namespace(scoped, fresh, ctx, action):
    argv = fresh.namespace(ctx, command(scoped, action))
    index = argv.index('--chdir')
    mounts = ['--unshare-net', '--ro-bind', str(scoped['workspace']), str(scoped['workspace']),
        '--ro-bind', str(ctx['mr']), str(ctx['mr']),
        '--ro-bind', str(scoped['source']), str(scoped['source']),
        '--bind', str(scoped['export']), str(scoped['export'])]
    return argv[:index] + mounts + argv[index:]


def execute(action, environment):
    scoped = scope(Path(__file__).absolute().parent, environment)
    fresh, ctx = load_platform(scoped)
    argv = isolated_namespace(scoped, fresh, ctx, action)
    env = fresh.environment(ctx)
    env.update(TMPDIR=str(scoped['export'] / 'tmp'), TMP=str(scoped['export'] / 'tmp'),
        TEMP=str(scoped['export'] / 'tmp'), OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
        MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1',
        PYTHONPATH=str(scoped['root']) + ':' + env['PYTHONPATH'])
    previous_umask = os.umask(0o077)
    try:
        os.nice(10)
        with (ctx['session'] / 'capture.lock').open('r') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            print(json.dumps(dict(event='cohort_cpu_export_namespace', action=action, environment=environment,
                source_read_only=True, network_isolated=True, only_writable_root=str(scoped['export']),
                native_captures=0, original_png_deleted=False, training_started=False)), flush=True)
            result = subprocess.run(argv, env=env)
        require(all(sha(scoped['source'] / name) == digest
            for name, digest in scoped['deployment']['source_pins'].items()),
            'Final native/split evidence changed during CPU export')
        return result.returncode
    finally:
        os.umask(previous_umask)


def parser():
    result = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    result.add_argument('action', choices=('plan', 'qualify', 'pack', 'validate'), nargs='?', default='plan')
    result.add_argument('--environment', choices=ENVIRONMENTS, required=True)
    return result


if __name__ == '__main__':
    try:
        args = parser().parse_args()
        raise SystemExit(execute(args.action, args.environment))
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        print(json.dumps(dict(event='cohort_cpu_export_blocked', error=str(error),
            source_unchanged=True, original_png_deleted=False, partial_products_preserved=True)), file=sys.stderr)
        raise SystemExit(1)
