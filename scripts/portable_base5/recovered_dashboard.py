#!/usr/bin/env python3
"""안전 crash 인계 출력의 별도 읽기 전용 대시보드. 기본은 실행 없는 계획이다."""
import sys
sys.dont_write_bytecode = True

import argparse
import json
import os
from pathlib import Path
import subprocess

import recover_unified_borrowed as recovery
import unified_dashboard as observer


def configuration(port=8075):
    root = observer.physical(__file__).parent
    workspace = root.parents[1]
    observer.require(workspace.name == 'ajounice_base5_capture'
        and root == workspace/recovery.RUNTIME_REL,
        'Run the recovered observer from the approved borrowed-host runtime')
    borrowed, common, _ = recovery.load_modules(root)
    ctx = borrowed.load(verify=False)
    selection = recovery.scope(ctx, common)
    doc, receipt = recovery.prepared(common, selection)
    # 원형 UI·HTML·namespace와 서비스 신원 검증을 재사용한다. 데이터만
    # 검증된 별도 복구 profile로 연결하고 capture.lock/PAUSE는 조작하지 않는다.
    config = observer.configuration(port=port, borrowed=True)
    previous = config['doc']
    observer.require(config['ctx']['workspace'] == workspace
        and all(doc.get(key) == previous.get(key) for key in
            ('schema', 'policy', 'environment', 'manifest', 'manifest_sha256',
             'release', 'release_sha256', 'helper_pins', 'runtime_hashes')),
        'Recovered observer must retain the same frozen source/runtime/UI')
    state = observer.physical(root/('recovered-dashboard-state-port'+str(port)))
    new, old, manifest = map(observer.physical, (doc['output'], doc['old_output'], doc['manifest']))
    observer.require(new == selection['output'] and old == selection['old']
        and new.is_dir() and old.is_dir() and new != old,
        'Only the explicitly verified crash continuation output may be observed')
    observer.require(not any(observer.inside(state, p) or observer.inside(p, state)
        for p in (new, old, manifest.parent)), 'Observer state must be separate from raw/source')
    config.update(profile=selection['profile'], doc=doc, root=new, old=old, manifest=manifest,
                  state=state, history=state/'history.jsonl')
    # 승인 분모는 모든 인계 자료를 포함하므로 디스크 추정의 원본 범위도
    # 보존 계보 전체를 포함한다. 외부 경로나 임의 source를 스캔하지 않는다.
    storage = {old}
    for record in receipt.get('imported', {}).get('import_lineage', []):
        for key in ('output', 'original_output'):
            path = observer.physical(record[key])
            observer.require(path.parent == workspace/'collections' and path.is_dir(),
                             'Storage ancestry must remain inside preserved borrowed collections')
            storage.add(path)
    config['storage_ancestors'] = sorted(storage)
    return config


def namespace_command(config):
    command = observer.namespace_command(config)
    for root in config.get('storage_ancestors', []):
        if root != config['old']:
            command += ['--storage-root', str(root)]
    return command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--port', type=int, default=8075)
    parser.add_argument('--run', action='store_true', help='Start only the read-only web observer')
    args = parser.parse_args(argv)
    config = configuration(args.port)
    observer.state_directory(config)
    observer.check_port(config['port'])
    command = namespace_command(config)
    if not args.run:
        print(json.dumps(dict(event='recovered_dashboard_plan', capture_root=str(config['root']),
            profile=str(config['profile']), url='http://127.0.0.1:'+str(config['port']),
            history_file=str(config['history']), collection_read_only=True,
            writable_state_only=str(config['state']), files_written=False,
            process_started=False, capture_controls=False, command=command), ensure_ascii=False, indent=2))
        return 0
    observer.state_directory(config, create=True)
    previous = os.umask(0o077)
    try:
        child = subprocess.Popen(command, env=observer.environment(config))
    finally:
        os.umask(previous)
    return child.wait()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps(dict(event='recovered_dashboard_blocked', error=str(error),
            existing_processes_untouched=True, capture_controls=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
