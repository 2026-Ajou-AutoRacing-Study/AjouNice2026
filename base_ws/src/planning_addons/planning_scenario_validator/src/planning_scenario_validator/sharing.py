"""Build credential-filtered evidence bundles; verify remote bytes, not copy exit alone."""
import hashlib
import json
import os
import re
import shutil
from pathlib import Path
import yaml
from .runtime import atomic_json
from .recording_options import sharing_warnings

SECRET = re.compile(r'(password|passwd|secret|token|api_key|access_key|private_key|authorization|credential|cookie)', re.I)
ALLOWED = {'.json', '.jsonl', '.yaml', '.yml', '.csv', '.log', '.py', '.png', '.jpg', '.bag', '.mp4'}


def redact(value):
    if isinstance(value, dict):
        return {k: '[REDACTED]' if SECRET.search(str(k)) else redact(v) for k, v in value.items()}
    if isinstance(value, list):
        return [redact(v) for v in value]
    if isinstance(value, str):
        value = re.sub(r'(?i)(Bearer\s+)[^\s"\']+', r'\1[REDACTED]', value)
        value = re.sub(r'''(?i)((?:password|passwd|client_secret|access_token|refresh_token|api_key|authorization)["']?\s*[=:]\s*)("[^"]*"|'[^']*'|[^\s,;]+)''', r'\1[REDACTED]', value)
        value = re.sub(r'(https?://)[^/@\s]+:[^/@\s]+@', r'\1[REDACTED]@', value)
        value = re.sub(r'''(?i)((?:token|access_key(?:_id)?|private_key|cookie)\s*[=:]\s*)("[^"]*"|'[^']*'|[^\s,;]+)''', r'\1[REDACTED]', value)
        value = re.sub(r'(?i)(--(?:password|secret|token|api-key|access-token)\s+)\S+', r'\1[REDACTED]', value)
        value = re.sub(r'-----BEGIN [^-]*PRIVATE KEY-----.*?-----END [^-]*PRIVATE KEY-----',
                       '[REDACTED PRIVATE KEY]', value, flags=re.S)
    return value


def safe_copy(source, target):
    source, target = Path(source), Path(target)
    if source.is_symlink() or not source.is_file() or source.suffix not in ALLOWED:
        return False
    if SECRET.search(source.name) or source.name.startswith('.') or source.name == 'rclone.conf':
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.suffix in ('.bag', '.mp4', '.png', '.jpg'):
        # Hard links avoid duplicating multi-GB bags. Originals are never edited.
        if target.exists():
            target.unlink()
        try:
            os.link(source, target)
        except OSError:
            shutil.copy2(source, target)
    else:
        text = source.read_text(encoding='utf-8', errors='replace')
        if source.suffix == '.json':
            text = json.dumps(redact(json.loads(text)), ensure_ascii=False, indent=2)
        elif source.suffix in ('.yaml', '.yml'):
            text = yaml.safe_dump(redact(yaml.safe_load(text)), allow_unicode=True, sort_keys=False)
        elif source.suffix == '.jsonl':
            text = '\n'.join(json.dumps(redact(json.loads(line)), ensure_ascii=False)
                             for line in text.splitlines() if line.strip())
        else:
            text = redact(text)
        target.write_text(text, encoding='utf-8')
    return True


def hashes(path):
    sha, md5 = hashlib.sha256(), hashlib.md5()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            sha.update(chunk)
            md5.update(chunk)
    return {'sha256': sha.hexdigest(), 'md5': md5.hexdigest()}


def build_bundle(cfg, folder):
    folder = Path(folder)
    root = cfg['output'] / 'share_payload'
    target = root / folder.parent.name / folder.name
    if target.exists():
        shutil.rmtree(target)
    target.mkdir(parents=True)
    profile = cfg['upload'].get('profile', 'basic')
    for path in folder.rglob('*'):
        if any(part.startswith('.') for part in path.relative_to(folder).parts):
            continue
        if path.suffix == '.bag' and profile not in ('bag', 'full'):
            continue
        if path.suffix == '.mp4' and profile not in ('video', 'full'):
            continue
        safe_copy(path, target / path.relative_to(folder))
    for name in ('model_runtime.json', 'model_launch.json', 'model_process_receipt.json',
                 'execution.json', 'campaign_config.yaml', 'campaign_state.json', 'summary.csv'):
        safe_copy(cfg['output'] / name, target / 'execution' / name)
    for path in (cfg['output'] / 'stack_logs').glob('*'):
        safe_copy(path, target / 'stack_logs' / path.name)
    # Preserve actual calibration and config sources, not just local paths/hashes.
    sources = [cfg.get('network', {}).get(key) for key in ('sensor_json', 'ego_network_json')]
    model = cfg.get('model_contract') or {}
    sources += [model.get(key) for key in ('model_config', 'runtime_config')]
    sources += list(model.get('config_files_sha256', {}))
    from .config import ROOT
    for source in filter(None, sources):
        path = Path(source)
        if not path.is_absolute():
            path = ROOT / path
        try:
            relative = path.relative_to(ROOT)
        except ValueError:
            relative = Path(path.name)
        safe_copy(path, target / 'configs' / relative)
    warnings = sharing_warnings(cfg.get('recording', {}), cfg['upload'])
    audit_path = folder / 'recording_audit.json'
    audit = json.loads(audit_path.read_text()) if audit_path.is_file() else {}
    metadata_missing = []
    if audit.get('complete_for_analysis'):
        if not (target / 'stack_logs/commands.json').is_file():
            metadata_missing.append('stack_logs/commands.json')
        if model.get('preset') not in (None, 'od_gt'):
            receipt = target / 'actual_model_receipt.json'
            if not receipt.is_file() or json.loads(receipt.read_text()).get('status') != 'VERIFIED':
                metadata_missing.append('verified actual_model_receipt.json')
        if not (target / 'configs').is_dir():
            metadata_missing.append('actual configs')
    if metadata_missing:
        warnings.append('분석 재현 정보 누락: ' + ', '.join(metadata_missing))
    manifest = {'schema_version': 1, 'profile': profile, 'warnings': warnings,
                'recording_status': audit.get('status', 'UNKNOWN'),
                'analysis_available_locally': audit.get('complete_for_analysis', False),
                'analysis_available_in_share': audit.get('complete_for_analysis', False)
                                               and profile in ('bag', 'full') and not metadata_missing,
                'credentials': 'known credential keys/headers redacted; credential files excluded',
                'files': {str(p.relative_to(target)): {'size': p.stat().st_size, **hashes(p)}
                          for p in sorted(target.rglob('*')) if p.is_file()}}
    atomic_json(target / 'share_manifest.json', manifest)
    # The manifest cannot recursively hash itself; remote verification also checks it.
    for name in ('summary.csv', 'campaign_config.yaml', 'campaign_state.json'):
        safe_copy(cfg['output'] / name, root / name)
    return root, target, manifest


def verify_listing(folder, listing, *, files=None):
    """Require size AND a matching content hash for every selected shared file."""
    remote = {entry['Path']: entry for entry in listing if not entry.get('IsDir')}
    failures = []
    for path in sorted(Path(folder).rglob('*') if files is None else files):
        if not path.is_file():
            continue
        name = str(path.relative_to(folder))
        entry = remote.get(name, {})
        remote_hashes = {key.lower(): value.lower() for key, value in entry.get('Hashes', {}).items()}
        local = hashes(path)
        match = any(remote_hashes.get(key) == value for key, value in local.items())
        if entry.get('Size') != path.stat().st_size or not match:
            failures.append(name)
    return failures
