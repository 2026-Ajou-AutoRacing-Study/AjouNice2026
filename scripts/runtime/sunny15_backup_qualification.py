"""Reuse proven E24 backup mechanics without claiming a new payload upload.

Only local evidence and a read-only remote executable SHA probe are used.
Renewable OAuth credentials are never copied into the receipt or to Vast.
"""
import configparser
import hashlib
import json
from pathlib import Path
import shlex
import time

import base5_vast_oneclick as transport

ROOT = Path(__file__).resolve().parents[2]
KIND = 'reused_full_backup_qualification'
E24_SHA = '84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746'
E24_CONTRACT = 'ecddcbad9c2a5a8347a134c0af848fa46a6bf97261b39b605379825075713ca9'
HELPERS = tuple('perception/detection3d/bevfusion/tools/' + name for name in (
    'backup_base5_checkpoints.py', 'base5_checkpoint_parts.py', 'backup_base5_vast_access_only.py'))


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024**2), b''):
            digest.update(chunk)
    return digest.hexdigest()


def identity(c):
    return {key: c[key] for key in ('host', 'port', 'drive_remote', 'drive_root', 'destination', 'backup_script')}


def credential_profile(c):
    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(Path.home() / '.config/rclone/rclone.conf')
    # Profile/root/app identity, not a claim to know the Google account subject.
    return dict(remote=c['drive_remote'], root=c['drive_root'],
                client_id_sha256=hashlib.sha256(cfg[c['drive_remote']]['client_id'].encode()).hexdigest())


def prior_full_evidence(log, manifest):
    frozen = json.loads(Path(manifest).read_text())
    if frozen['contract_sha256'] != E24_CONTRACT:
        raise ValueError('Expected the completed E24 training contract')
    helpers = {name: frozen['identity']['source_hashes'][name] for name in HELPERS}
    found_readback = found_report = None
    for number, line in enumerate(Path(log).read_text().splitlines(), 1):
        try:
            entry = json.loads(line)
        except ValueError:
            continue
        if not isinstance(entry, dict):
            continue
        if (entry.get('event') == 'local_readback_verified' and entry.get('verified') is True
                and entry.get('sha256') == E24_SHA):
            found_readback = dict(line=number, sha256=entry['sha256'], bytes=entry['bytes'])
        status = entry.get('status', {})
        record = status.get('uploaded', {}).get('epoch_24.pth', {})
        if (entry.get('valid') is True and record.get('sha256') == E24_SHA
                and record.get('resume_capable') is True and record.get('contract_sha256') == E24_CONTRACT
                and record.get('readback_location') == 'local_credential_owner'
                and not status.get('errors') and entry.get('pending_checkpoints') == []):
            found_report = dict(line=number, record=record, destination=status['destination'],
                                credential_scope=entry.get('credential_scope'))
    if not found_readback or not found_report or found_readback['bytes'] != found_report['record']['bytes']:
        raise ValueError('Actual full E24 upload/readback evidence missing')
    return dict(full_readback=found_readback, completed_report=found_report, frozen_helpers=helpers)


def make_receipt(c, output):
    output = Path(output)
    if output.exists():
        raise FileExistsError('Preserve qualification receipts')
    base = ROOT / 'artifacts/evaluation/bevfusion'
    paths = dict(prior_log=base / 'base5_oneclick_261011/drive_backup.jsonl',
                 prior_manifest=base / 'base5_full_initializer_261011/candidate/run_manifest.json',
                 current_probe=base / 'sunny15_training_readiness_261018/drive_small_probe.json',
                 current_probe_config=base / 'sunny15_training_readiness_261018/cosine.json')
    old = prior_full_evidence(paths['prior_log'], paths['prior_manifest'])
    local_helpers = {name: sha(ROOT / name) for name in HELPERS}
    if local_helpers != old['frozen_helpers']:
        raise ValueError('Local backup mechanics differ from the proven E24 implementation')
    probe = json.loads(paths['current_probe'].read_text())
    probe_config = json.loads(paths['current_probe_config'].read_text())
    if probe.get('write_readback') is not True or probe.get('training_started') is not False:
        raise ValueError('Current destination write/readback is not proven')
    if identity(c) != identity(probe_config):
        raise ValueError('Current destination/probe endpoint differs')
    for key in ('host', 'port', 'drive_remote', 'drive_root', 'backup_script'):
        if c[key] != transport.DEFAULTS[key]:
            raise ValueError('Original E24 backend differs')
    remote_paths = [c['backup_script'], str(Path(c['backup_script']).with_name('base5_checkpoint_parts.py'))]
    code = ('import hashlib,json,pathlib; paths=' + repr(remote_paths) +
            ';print(json.dumps({p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() for p in paths}))')
    result = transport.ssh(c, 'python -c ' + shlex.quote(code), capture_output=True, text=True, timeout=20)
    remote_helpers = json.loads(result.stdout.strip().splitlines()[-1])
    expected = dict(zip(remote_paths, (local_helpers[HELPERS[0]], local_helpers[HELPERS[1]])))
    if remote_helpers != expected:
        raise ValueError('Actual remote uploader/parts differ from proven E24 code')
    receipt = dict(schema='sunny15_backup_qualification_reuse_v1', qualification_kind=KIND,
        qualified_at_unix=time.time(), new_checkpoint_uploaded=False,
        first_new_epoch_backup_requires_actual_verification=True,
        scope='Reused E24 full payload mechanics plus current new-destination small write/readback; not new payload or throughput proof.',
        transport_identity=identity(c), credential_profile=credential_profile(c),
        local_helpers=local_helpers, remote_helpers=remote_helpers, prior_full_evidence=old,
        local_evidence={name:dict(path=str(path), sha256=sha(path)) for name, path in paths.items()})
    validate(receipt, c, local=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        stream.write(json.dumps(receipt, indent=2) + '\n')
    return receipt


def validate(receipt, c, *, local=False):
    if (receipt.get('qualification_kind') != KIND or receipt.get('new_checkpoint_uploaded') is not False
            or receipt.get('first_new_epoch_backup_requires_actual_verification') is not True):
        raise ValueError('Require explicit reused qualification, never a small probe or new-payload claim')
    if receipt.get('transport_identity') != identity(c):
        raise ValueError('Reused backend/destination identity differs')
    old = receipt['prior_full_evidence']
    record = old.get('completed_report', {}).get('record', {})
    readback = old.get('full_readback', {})
    if (record.get('sha256') != E24_SHA or readback.get('sha256') != E24_SHA
            or record.get('contract_sha256') != E24_CONTRACT or record.get('resume_capable') is not True
            or record.get('bytes', 0) <= 0 or record['bytes'] != readback.get('bytes')):
        raise ValueError('Prior full-checkpoint readback proof differs')
    if receipt.get('local_helpers') != old.get('frozen_helpers') or set(receipt.get('local_helpers', {})) != set(HELPERS):
        raise ValueError('Prior/current helper hashes differ')
    expected_remote = {c['backup_script']:receipt['local_helpers'][HELPERS[0]],
        str(Path(c['backup_script']).with_name('base5_checkpoint_parts.py')):receipt['local_helpers'][HELPERS[1]]}
    if receipt.get('remote_helpers') != expected_remote:
        raise ValueError('Recorded remote helper hashes differ')
    if local:
        if receipt['credential_profile'] != credential_profile(c):
            raise ValueError('Local credential app/profile/root changed')
        for name, checksum in receipt['local_helpers'].items():
            if sha(ROOT / name) != checksum:
                raise ValueError('Local helper changed after qualification')
        for item in receipt['local_evidence'].values():
            if sha(item['path']) != item['sha256']:
                raise ValueError('Local qualification evidence changed')
        evidence = receipt['local_evidence']
        if prior_full_evidence(evidence['prior_log']['path'], evidence['prior_manifest']['path']) != old:
            raise ValueError('Prior full evidence no longer matches')
        probe = json.loads(Path(evidence['current_probe']['path']).read_text())
        probe_config = json.loads(Path(evidence['current_probe_config']['path']).read_text())
        if probe.get('write_readback') is not True or identity(probe_config) != identity(c):
            raise ValueError('Current destination small probe differs')
    else:
        for path, checksum in receipt['remote_helpers'].items():
            if sha(path) != checksum:
                raise ValueError('Remote helper changed after qualification')
    return receipt
