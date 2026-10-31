#!/usr/bin/env python3
"""빌린 PC 수집을 읽기만 하는 별도 localhost 웹 대시보드.

기본은 읽기 전용 계획이다. --run도 HTTP 관측만 시작하며 ROS·MORAI·수집기,
수집 잠금·중단 표식·기존 원본을 변경하지 않는다. 실제 시작은 운영자만 한다.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess
import sys


DIRECTORY = 'web-dashboard-sunny13-v1'
OUTPUT_NAME = 'sunny13_5k_recovery_v2'
PORT = 8075
LIVE_SHA = '7850aed3a4a2881393af75ed2da96752e8fe292606c837abd5790281e481fd00'
MANIFEST_SHA = '4f7fc3b31e23688b43913f2457d570cf014c19e04b2c2e391b463647809e02c5'
RELEASE_SHA = '0a3dd5c96b4522bc9d0547bed845c97ebb02d228b30ff398e1d7ed688b726180'
SCHEMA = 'base5_borrowed_population_recovery_v1'
ASSETS = {
    'base5_capture_dashboard.py':'8fab975eda002a5192987d28b8327cf7c6571ca4566e674d6d56096a906fc8c9',
    'base5_capture_dashboard.html':'7e364fd1e2ac25ebd0ceeebc7f8328c395cb88d2d550706f204c65e3e1969c58'}


def require(value,message):
    if not value:
        raise ValueError(message)


def exact_path(value):
    result = Path(value).expanduser().absolute()
    require('..' not in result.parts and all(not p.is_symlink() for p in (result,*result.parents)),
        'Only exact nonsymlink dashboard paths are allowed')
    return result


def owned(value,directory=False):
    result = exact_path(value);info = result.stat()
    require(info.st_uid == os.getuid() and not info.st_mode & 0o022
        and (stat.S_ISDIR(info.st_mode) if directory else stat.S_ISREG(info.st_mode)),
        'Private owned dashboard input required: '+str(result))
    return result


def sha(value):
    h = hashlib.sha256()
    with exact_path(value).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def read(value):
    return json.loads(owned(value).read_text())


def frozen_json(value,expected):
    """구 bundle mode는 보존한다. 고정 원본 하나만 실제 읽은 bytes로 대조한다."""
    filename = exact_path(value);info = filename.stat()
    require(info.st_uid == os.getuid() and stat.S_ISREG(info.st_mode),
        'Owned regular frozen bundle source required')
    payload = filename.read_bytes()
    require(hashlib.sha256(payload).hexdigest() == expected,'Immutable bundle source changed')
    return filename,json.loads(payload)


def within(value,root):
    try:
        Path(value).relative_to(root)
        return True
    except ValueError:
        return False


def load_live(directory):
    filename = directory/'live_v2.py'
    if not filename.exists() and not filename.is_symlink():
        filename = directory/'live.py'
    require(sha(owned(filename)) == LIVE_SHA,'Exact frozen live_v2 helper required')
    spec = importlib.util.spec_from_file_location('_base5_readonly_web_original_live',filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def configuration(workspace):
    """Only bounded headers/small pins; never repeats native/5000-file QA."""
    workspace = owned(workspace,True)
    helper = owned(__file__)
    require(helper == workspace/'checks/remote_web_dashboard.py',
        'Deploy only the explicitly scoped new checks helper')
    live = load_live(helper.parent)
    ctx = live.load(workspace,verify=False)
    live.ready(ctx)
    require(ctx['workspace'] == workspace,'Frozen host workspace differs')
    graph = ctx['graph'];source = graph['manifest'];release = graph['release']
    require(source.get('sha256') == MANIFEST_SHA and release.get('sha256') == RELEASE_SHA,
        'Exact original Sunny13 source/release binding required')
    logical_source = exact_path(source['source'])
    physical_source = exact_path(ctx['bundle']/source['bundle_path'])
    require(within(physical_source,ctx['bundle']), 'Frozen source escaped its immutable bundle')
    physical_source,deck = frozen_json(physical_source,MANIFEST_SHA)
    require(deck.get('count') == len(deck.get('scenes',[])) == 5000,
        'Exact complete 5K planned denominators required')
    allowed = {row['id'] for row in deck['scenes']}
    require(len(allowed) == 5000,'Source IDs must be unique')
    root = owned(workspace/'collections'/OUTPUT_NAME,True)
    previous = owned(workspace/'collections/sunny13_5k',True)
    intent_file = owned(workspace/'checks/population_recovery_intent.json')
    intent = read(intent_file)
    require(intent.get('schema') == SCHEMA and intent.get('environment') == 'sunny13'
        and intent.get('workspace') == str(workspace) and intent.get('output') == str(root)
        and intent.get('previous_output') == str(previous)
        and intent.get('manifest') == str(logical_source) and intent.get('release') == release['source']
        and intent.get('source_runtime_count') == 427 and intent.get('qa_changed') is False,
        'Current recovery source/output binding changed')
    guard = read(root/'population_recovery_contract.json')
    require(guard.get('schema') == SCHEMA and guard.get('intent_sha256') == sha(intent_file)
        and guard.get('previous_output_preserved') == str(previous)
        and guard.get('original_native_qa') is True and guard.get('original_shutdown_safety') is True
        and guard.get('helper_hashes') == intent.get('files'),
        'Actual recovery collection does not match its current intent')
    prefix_file = owned(root/'imported_prefix.json')
    require(intent.get('import_receipt') == str(prefix_file)
        and sha(prefix_file) == intent.get('import_receipt_sha256') == guard.get('imported_prefix_sha256'),
        'Imported69 receipt changed')
    prefix = read(prefix_file);rows = prefix.get('selected',[])
    require(prefix.get('schema') == 1 and prefix.get('manifest_sha256') == MANIFEST_SHA
        and prefix.get('original_output') == str(previous) and not prefix.get('deficits')
        and len(rows) == 69 and len({row.get('source_cell_id') for row in rows}) == 69
        and all(row.get('source_cell_id') in allowed and row.get('status') == 'accepted_candidate'
            and not row.get('audit',{}).get('review_reasons')
            and within(exact_path(row['capture_root']),previous) for row in rows),
        'Exactly 69 preserved accepted rows, without path aliases, are required')
    work = owned(workspace/'work'/DIRECTORY,True)
    state = owned(work/'state',True)
    assets = {name:owned(work/name) for name in ASSETS}
    require(all(sha(assets[name]) == digest for name,digest in ASSETS.items()),
        'Exact byte-identical original dashboard Python/HTML assets required')
    for item in state.rglob('*'):
        owned(item,item.is_dir())
        require(item.is_dir() or item.stat().st_nlink == 1,
            'Dashboard state cannot contain hardlinks to protected files')
    return dict(live=live,ctx=ctx,workspace=workspace,root=root,previous=previous,work=work,state=state,
        assets=assets,manifest=logical_source,intent_sha256=sha(intent_file),imported_count=69,
        helper_sha256=sha(helper))


def dashboard_command(config):
    return ['/usr/bin/python3','-B',str(config['assets']['base5_capture_dashboard.py']),
        '--capture-root',str(config['root']),'--manifest',str(config['manifest']),
        '--history-file',str(config['state']/'history.jsonl'),'--port',str(PORT)]


def namespace_command(config):
    command = config['live'].namespace(config['ctx'],dashboard_command(config))
    require(command.count('--chdir') == 1,'Exact original namespace layout required')
    index = command.index('--chdir')
    # These later directory mounts override the historical workspace/MORAI RW
    # binds only in THIS process. Directory binds see collector atomic replaces.
    command[index:index] = ['--ro-bind',str(config['workspace']),str(config['workspace']),
        '--ro-bind',str(config['ctx']['mr']),str(config['ctx']['mr']),
        '--bind',str(config['state']),str(config['state'])]
    return command


def environment(config):
    state = str(config['state'])
    return dict(PATH='/usr/bin:/bin',LANG='C.UTF-8',LC_ALL='C.UTF-8',HOME=state,
        PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
        TMPDIR=state,TMP=state,TEMP=state,XDG_CACHE_HOME=state,XDG_CONFIG_HOME=state,XDG_DATA_HOME=state,
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--workspace',type=Path,default=Path.home()/'ajounice_base5_capture')
    parser.add_argument('--port',type=int,choices=[PORT],default=PORT)
    parser.add_argument('--run',action='store_true')
    args = parser.parse_args(argv)
    config = configuration(args.workspace)
    command = namespace_command(config)
    if not args.run:
        print(json.dumps(dict(event='borrowed_web_dashboard_plan_only',command=command,
            url='http://127.0.0.1:8075',capture_root=str(config['root']),
            immutable_asset_hashes=ASSETS,helper_sha256=config['helper_sha256'],
            imported_accepted=69,parent_counts_already_include_imported=True,child_counts_added=False,
            writable_host_directory=str(config['state']),collection_read_only=True,morai_read_only=True,
            files_written=False,process_started=False,signals_sent=False),ensure_ascii=False,indent=2))
        return 0
    # This is only the separate HTTP observer, not the collector or ROS process.
    # Keep this detached helper alive as bwrap's real parent. execve would
    # instead make a short-lived SSH launcher its parent, so --die-with-parent
    # could terminate the observer immediately after SSH disconnects.
    child = subprocess.Popen(command,env=environment(config))
    return child.wait()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError,ValueError,RuntimeError) as error:
        print(json.dumps(dict(event='borrowed_web_dashboard_blocked',error=str(error),
            fallback_allowed=False,capture_controls=False),ensure_ascii=False),file=sys.stderr)
        raise SystemExit(1)
