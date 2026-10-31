#!/usr/bin/env python3
"""빌린 PC 유연 보충 출력의 별도 읽기 전용 관측기. 기본은 계획만 출력한다.

기존 UI·공통 namespace를 그대로 사용하며 수집·PAUSE·capture.lock을
조작하지 않는다. --run도 observer 전용 상태 폴더와 웹 서버만 만든다.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess


OBSERVER_REL = Path('work/completion-observer-v1')
TERMINAL_OBSERVER_REL = Path('work/completion-observer-terminal-animal-v1')
TERMINAL_OUTPUT_NAME = 'sunny13_5k_terminal_animal_v1'
TERMINAL_POLICY = 'terminal-static-animal-readable-distance-cell-v1'
FLEXIBLE_POLICY = 'same-family-purpose-fresh-donor-completion-v1'
ORIGINAL_REL = Path('work/unified-runtime-v1/unified_dashboard.py')
COMPLETION_REL = Path('work/completion-runtime-v1')
ORIGINAL_SHA = 'db8d388c5518bd42053a2d1258ce0749a1f877a60d3c42ad1d7181ec28899ff4'
IDENTITY = ('schema', 'policy', 'environment', 'manifest', 'manifest_sha256',
            'release', 'release_sha256', 'helper_pins', 'runtime_hashes')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def physical(value):
    path = Path(value).expanduser().absolute()
    require('..' not in path.parts and all(not p.is_symlink() for p in (path, *path.parents)),
            'Exact nonsymlink observer path required')
    return path


def load_file(path, name):
    path = physical(path)
    require(path.is_file() and path.stat().st_uid == os.getuid(), 'Owned observer module required')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_components(root):
    workspace = root.parents[1]
    original = physical(workspace/ORIGINAL_REL)
    require(hashlib.sha256(original.read_bytes()).hexdigest() == ORIGINAL_SHA,
            'Original read-only observer implementation changed')
    observer = load_file(original, '_base5_completion_original_observer')
    completion_root = workspace/COMPLETION_REL
    if str(completion_root) not in sys.path:
        sys.path.insert(0, str(completion_root))
    completion = load_file(completion_root/'complete_borrowed.py', '_base5_completion_dashboard_runtime')
    borrowed, exact, _ = completion.load_modules(completion_root)
    scope = completion.deployment(completion_root, borrowed, exact)
    return observer, scope


def storage_ancestry(observer, workspace, baseline, seed, manifest_sha256):
    """기존 import 계보의 제한된 collection 폴더만 읽는다. 원본 재심사는 아니다."""
    roots, seen = {seed}, set()
    current = baseline
    while True:
        current = observer.physical(current)
        observer.require(current.parent == workspace/'collections' and current.is_dir(),
                         'Storage ancestry must remain inside preserved borrowed collections')
        observer.require(current not in seen and len(seen) < 16, 'Cyclic or excessive storage ancestry')
        seen.add(current); roots.add(current)
        imported = current/'imported_prefix.json'
        if not imported.exists():
            break
        value = observer.read(imported)
        observer.require(value.get('schema') == 1 and value.get('manifest_sha256') == manifest_sha256,
                         'Storage ancestry source identity changed')
        # The explicitly preserved immediate parent defines the full chain.
        current = value['original_output']
    return sorted(roots)


def ledger(observer, path):
    content = observer.regular(path).read_bytes()
    observer.require(not content or content.endswith(b'\n'), 'Complete newline-terminated selection required')
    return [json.loads(line) for line in content.splitlines() if line.strip()]


def terminal_animal_output(observer, scope):
    """최종 출력의 작은 계약과 선택 계보만 확인한다. 원본 QA는 반복하지 않는다."""
    workspace, parent = scope['workspace'], observer.physical(scope['output'])
    target = observer.physical(workspace/'collections'/TERMINAL_OUTPUT_NAME)
    observer.require(target.parent == parent.parent == workspace/'collections'
        and target != parent and target.is_dir(), 'Independent fixed terminal-animal output required')
    state, status, quota = [observer.read(target/name) for name in ('state.json', 'status.json', 'quota.json')]
    old_state, old_status, old_quota = [observer.read(parent/name)
        for name in ('state.json', 'status.json', 'quota.json')]
    contract, receipt = state.get('contract', {}), state.get('import_receipt', {})
    observer.require(state.get('stage') == status.get('stage') == 'complete'
        and status.get('error') is None and state.get('error') is None
        and status.get('failed') == 0 and not status.get('selected_not_certified')
        and not state.get('selected_not_certified') and state.get('current') is None
        and state.get('current_child') is None and status.get('current') is None
        and status.get('accepted_candidate') == quota.get('accepted') == quota.get('target') == 5000
        and status.get('review') == quota.get('deficit') == quota.get('pending') == 0,
        'Only a certified complete 5K terminal-animal output may be observed')
    observer.require(old_state.get('stage') == old_status.get('stage') == 'complete_with_deficit'
        and old_state.get('error') is None and old_status.get('error') is None
        and old_status.get('failed') == 0 and not old_status.get('selected_not_certified')
        and not old_state.get('selected_not_certified') and old_state.get('current') is None
        and old_status.get('current') is None and old_quota.get('pending') == 0
        and old_status.get('accepted_candidate') == old_quota.get('accepted') == 4999
        and old_status.get('review') == old_quota.get('deficit') == 1,
        'Terminal-animal observer requires the preserved safe 4,999-selection parent')
    observer.require(contract.get('policy') == FLEXIBLE_POLICY
        and contract.get('parent') == str(parent)
        and contract.get('profile') == str(scope['profile'])
        and contract.get('profile_sha256') == observer.sha(scope['profile'])
        and contract.get('source_sha256') == scope['previous']['manifest_sha256']
        and receipt.get('policy') == TERMINAL_POLICY
        and receipt.get('approved_prefix_preserved') == 4999
        and state.get('baseline_selected_sha256') == observer.sha(parent/'selected_ledger.jsonl'),
        'Terminal-animal selection source/parent/policy identity changed')
    pins = contract.get('parent_pins')
    observer.require(isinstance(pins, dict) and pins
        and all(observer.sha(path) == digest for path, digest in pins.items())
        and isinstance(receipt.get('pins'), dict) and receipt['pins']
        and all(observer.sha(path) == digest for path, digest in receipt['pins'].items()),
        'Terminal-animal preserved parent pins changed')
    prior, chosen = ledger(observer, parent/'selected_ledger.jsonl'), ledger(observer, target/'selected_ledger.jsonl')
    observer.require(len(prior) == 4999 and len(chosen) == 5000,
        'Terminal-animal ledger count differs from authoritative status')
    for key in ('source_cell_id', 'capture_root', 'scenario_sha256'):
        values = [row.get(key) for row in chosen]
        observer.require(all(isinstance(value, str) and value for value in values)
            and len(set(values)) == 5000, 'Terminal-animal selection contains duplicate/invalid '+key)
    lookup = {row['source_cell_id']:row for row in chosen}
    observer.require(len({row['source_cell_id'] for row in prior}) == 4999
        and all(lookup.get(row['source_cell_id']) == row for row in prior),
        'Terminal-animal selection changed the preserved 4,999-row prefix')
    return target


def configuration(port=8077, terminal_animal=False):
    root = physical(__file__).parent
    workspace = root.parents[1]
    allowed_roots = {workspace/OBSERVER_REL}
    if terminal_animal:
        allowed_roots.add(workspace/TERMINAL_OBSERVER_REL)
    require(root in allowed_roots and workspace.name == 'ajounice_base5_capture',
            'Run the observer from its separate approved borrowed-host deployment')
    observer, scope = load_components(root)
    require(scope['workspace'] == workspace and scope['root'] == workspace/COMPLETION_REL,
            'Completion observer deployment escaped its approved workspace')
    config = observer.configuration(port=port, borrowed=True)
    doc = scope['previous']
    observer.require(config['ctx']['workspace'] == workspace
        and all(config['doc'].get(key) == doc.get(key) for key in IDENTITY),
        'Completion observer must retain the same frozen source/runtime/UI')
    new, old, seed = map(observer.physical, (scope['output'], scope['baseline'], scope['seed']))
    observer.require(all(p.parent == workspace/'collections' and p.is_dir() for p in (new, old, seed))
        and len({new, old, seed}) == 3, 'Prepared independent completion outputs required')
    manifest = observer.physical(doc['manifest'])
    storage = storage_ancestry(observer, workspace, old, seed, doc['manifest_sha256'])
    if terminal_animal:
        storage = sorted(set(storage) | {new})
        new, old = terminal_animal_output(observer, scope), new
    state = observer.physical(root/(('state-terminal-animal-port' if terminal_animal else 'state-port')+str(port)))
    observer.require(not any(observer.inside(state,p) or observer.inside(p,state)
        for p in (new, old, seed, manifest.parent)), 'Observer state must be separate from raw/source')
    config.update(profile=scope['profile'], doc=dict(doc, output=str(new), old_output=str(old)),
        root=new, old=old, seed=seed, manifest=manifest, state=state, history=state/'history.jsonl')
    config['storage_ancestors'] = storage
    config['terminal_animal'] = terminal_animal
    return observer, config


def namespace_command(observer, config):
    command = observer.namespace_command(config)
    for root in config['storage_ancestors']:
        if root != config['old']:
            command += ['--storage-root', str(root)]
    return command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--port', type=int, default=8077)
    parser.add_argument('--run', action='store_true', help='Start only the read-only web observer')
    parser.add_argument('--terminal-animal', action='store_true',
        help='Observe only the certified fixed terminal-animal 5K output')
    args = parser.parse_args(argv)
    observer, config = configuration(args.port, terminal_animal=args.terminal_animal)
    observer.state_directory(config)
    observer.check_port(config['port'])
    command = namespace_command(observer, config)
    if not args.run:
        print(json.dumps(dict(event='borrowed_completion_dashboard_plan',
            capture_root=str(config['root']), profile=str(config['profile']),
            url='http://127.0.0.1:'+str(config['port']), history_file=str(config['history']),
            collection_read_only=True, writable_state_only=str(config['state']),
            storage_ancestors=list(map(str,config['storage_ancestors'])), files_written=False,
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
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        print(json.dumps(dict(event='borrowed_completion_dashboard_blocked', error=str(error),
            existing_processes_untouched=True, capture_controls=False), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
