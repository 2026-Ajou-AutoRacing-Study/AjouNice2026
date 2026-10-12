"""Reusable authoring YAMLs and immutable, timestamped execution inputs."""
import hashlib
import json
import os
import shutil
from datetime import datetime
from pathlib import Path

import yaml

from .config import load_campaign, resolve_path
from .runtime import atomic_json, now
from .models import resolve_contract, verify_contract


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def is_snapshot(path):
    path = Path(path)
    return path.name == 'config_snapshot.yaml'


def prepare_execution(source, *, check_install=True):
    """Copy small inputs only. Never rename/rewrite the user's YAML or JSONs."""
    source = resolve_path(source)
    source_bytes = source.read_bytes()
    cfg = load_campaign(source, check_install=check_install)
    if source.read_bytes() != source_bytes:
        raise ValueError('사본 준비 중 YAML이 변경됐습니다. 다시 시작하세요')
    if is_snapshot(cfg['path']):
        raise ValueError('실행 사본은 편집용 YAML이 아닙니다. 원본 YAML로 새 실험을 시작하세요')
    document = yaml.safe_load(source_bytes)
    root = resolve_path(document.get('output_root', 'planner_logs/validation'))
    root.mkdir(parents=True, exist_ok=True)
    label = cfg['name']
    stem = label[:43] + '_' + datetime.now().strftime('%y%m%d_%H%M%S')
    index = 1
    while True:
        identifier = stem if index == 1 else stem[:54] + '_' + str(index)
        directory = root / identifier
        try:
            directory.mkdir()
            break
        except FileExistsError:
            index += 1
    originals, files = {}, {}

    def copy_input(path, relative):
        path = Path(path)
        data = path.read_bytes()
        destination = directory / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
        files[relative] = hashlib.sha256(data).hexdigest()
        originals[str(path)] = files[relative]
        return str(destination)

    try:
        copy_input(cfg['path'], 'source_config.yaml')
        if (directory / 'source_config.yaml').read_bytes() != source_bytes:
            raise ValueError('사본 준비 중 YAML이 변경됐습니다. 다시 시작하세요')
        for case in {case.source: case for case in cfg['cases']}.values():
            copy_input(case.source, 'scenarios/' + case.source.name)
            if files['scenarios/' + case.source.name] != case.source_sha256:
                raise ValueError('사본 준비 중 시나리오가 변경됐습니다. 다시 시작하세요')
        document['campaign_name'] = identifier
        document['output_root'] = str(root)
        document['scenarios']['directory'] = str(directory / 'scenarios')
        document['scenarios']['glob'] = '*.json'
        document['scenarios'].pop('include', None)
        for key in ('sensor_json', 'ego_network_json'):
            document['network'][key] = copy_input(cfg['network'][key], 'network/' + key + '.json')
        document['recording']['topic_profile'] = copy_input(
            resolve_path(document['recording']['topic_profile']), 'topic_profile.json')
        model = resolve_contract(cfg['stack'], cfg['network']['sensor_json'], resolve_path('.'),
                                 require_assets=check_install)
        for relative in model.get('config_files_sha256', {}):
            copy_input(resolve_path(relative), 'model_configs/' + relative)
        if model.get('runtime_config'):
            copy_input(model['runtime_config'], 'model_configs/bridge.yaml')
        atomic_json(directory / 'model_contract.json', model)
        files['model_contract.json'] = sha256(directory / 'model_contract.json')
        # Calibration/map/model binaries are intentionally referenced, not copied.
        path = directory / 'config_snapshot.yaml'
        path.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding='utf-8')
        files[path.name] = sha256(path)
        if any(sha256(path) != digest for path, digest in originals.items()):
            raise ValueError('사본 준비 중 원본 입력이 변경됐습니다. 다시 시작하세요')
        atomic_json(directory / 'execution.json', {
            'schema_version': 1, 'execution_id': identifier, 'experiment_name': label,
            'total': len(cfg['cases']),
            'created_at': now(), 'source_yaml': str(cfg['path']), 'snapshot': path.name,
            'files': files, 'original_hashes': originals,
            'note': '편집용 YAML과 독립된 실행 사본. 재개는 이 폴더의 입력만 사용합니다.'})
        return load_execution(directory, check_install=check_install)
    except BaseException:
        shutil.rmtree(directory)
        raise


def load_execution(directory, *, check_install=True):
    directory = resolve_path(directory)
    if directory.is_file():
        directory = directory.parent
    manifest_path = directory / 'execution.json'
    if not manifest_path.is_file():
        if (directory / 'config_snapshot.yaml').exists():
            raise ValueError('실험 사본 목록 execution.json이 사라졌습니다. 재개할 수 없습니다')
        # Read-only compatibility for old fixed-name outputs, without moving them.
        state = json.loads((directory / 'campaign_state.json').read_text(encoding='utf-8'))
        if (not isinstance(state, dict) or not isinstance(state.get('config_path'), str)
                or not isinstance(state.get('config_sha256'), str)):
            raise ValueError('이전 실험 state 형식이 잘못되었습니다. 결과는 보존합니다')
        cfg = load_campaign(state['config_path'], check_install=check_install)
        if cfg['output'] != directory or sha256(cfg['path']) != state.get('config_sha256'):
            raise ValueError('이전 실험의 원본 YAML이 변경됐습니다. 기존 입력을 복원해야 재개할 수 있습니다')
        return cfg
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if (not isinstance(manifest, dict) or manifest.get('schema_version') != 1 or manifest.get('snapshot') != 'config_snapshot.yaml'
            or not isinstance(manifest.get('files'), dict) or not manifest['files']):
        raise ValueError('실험 사본 목록 형식이 잘못됐습니다')
    required = {'config_snapshot.yaml', 'source_config.yaml', 'network/sensor_json.json',
                'network/ego_network_json.json', 'topic_profile.json'}
    if not required <= manifest['files'].keys():
        raise ValueError('실험 사본 목록에 필수 입력이 없습니다')
    for relative, digest in manifest['files'].items():
        if not isinstance(relative, str) or not isinstance(digest, str):
            raise ValueError('실험 사본 hash 형식이 잘못됐습니다')
        path = (directory / relative).resolve()
        if directory not in path.parents or not path.is_file() or sha256(path) != digest:
            raise ValueError('실험 사본이 변경되거나 사라졌습니다: {}'.format(relative))
    cfg = load_campaign(directory / manifest['snapshot'], check_install=check_install)
    if 'model_contract.json' in manifest['files']:
        model = json.loads((directory / 'model_contract.json').read_text())
        if check_install:
            verify_contract(model, cfg['stack'], cfg['network']['sensor_json'], resolve_path('.'))
        cfg['model_contract'] = model
    if cfg['output'] != directory or cfg['name'] != manifest.get('execution_id'):
        raise ValueError('실험 사본의 결과 경로가 기록과 다릅니다')
    expected_scenes = {relative for relative in manifest['files'] if relative.startswith('scenarios/')}
    if {'scenarios/' + case.source.name for case in cfg['cases']} != expected_scenes:
        raise ValueError('실험 사본의 시나리오 목록이 변경됐습니다')
    cfg['execution'] = manifest
    return cfg


def history(root):
    """Lightweight local history; tolerate damaged records without hiding them."""
    root = resolve_path(root)
    rows = []
    if not root.is_dir():
        return rows
    for directory in root.iterdir():
        if not directory.is_dir():
            continue
        manifest_path = directory / 'execution.json'
        state_path = directory / 'campaign_state.json'
        if not manifest_path.exists() and not state_path.exists():
            continue
        try:
            manifest = json.loads(manifest_path.read_text(encoding='utf-8')) if manifest_path.exists() else {}
            state = json.loads(state_path.read_text(encoding='utf-8')) if state_path.exists() else {}
            results = state.get('results', {})
            counts = {key: sum(row.get('status') == key for row in results.values())
                      for key in ('PASS', 'FAIL', 'ERROR')}
            total = state.get('total', manifest.get('total', sum(
                key.startswith('scenarios/') for key in manifest.get('files', {}))))
            if total and counts['PASS'] + counts['FAIL'] == total:
                status = 'DONE'
            elif state.get('run_status') == 'RUNNING':
                # A stale current-phase record after power loss is not a live run.
                try:
                    pid = int(state.get('runner_pid', -1))
                    if pid <= 0:
                        raise ProcessLookupError()
                    os.kill(pid, 0)
                    status = 'RUNNING'
                except (OSError, TypeError, ValueError):
                    status = 'INTERRUPTED'
            else:
                status = 'INTERRUPTED' if state else 'READY'
            rows.append({'directory': str(directory), 'name': manifest.get('experiment_name', state.get('campaign', directory.name)),
                         'created_at': manifest.get('created_at', state.get('created_at', '')),
                         'total': total, 'completed': counts['PASS'] + counts['FAIL'], 'counts': counts,
                         'status': status, 'legacy': not bool(manifest)})
        except (OSError, ValueError, AttributeError, TypeError):
            rows.append({'directory': str(directory), 'name': directory.name, 'created_at': '',
                         'total': 0, 'completed': 0, 'counts': {}, 'status': 'DAMAGED', 'legacy': True})
    return sorted(rows, key=lambda row: row['created_at'], reverse=True)


def local_time(value):
    try:
        return datetime.fromisoformat(value).astimezone().strftime('%Y-%m-%d %H:%M:%S')
    except (ValueError, TypeError):
        return value
