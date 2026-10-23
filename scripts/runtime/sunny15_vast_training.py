#!/usr/bin/env python3
"""Sunny15 A/B launch gate. Default is read-only status, never training.

Use a new, explicit JSON configuration per candidate. Renewable Drive tokens
stay local; the existing restricted loopback relay handles W&B and backup.
"""
import argparse
import contextlib
import configparser
import datetime
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
import urllib.parse
import urllib.request

import base5_vast_oneclick as transport
import sunny15_backup_qualification as backup_qualification

ROOT = Path(__file__).resolve().parents[2]
GROUP = 'base5-sunny15-ab-261018'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024**2), b''):
            digest.update(chunk)
    return digest.hexdigest()


def emit(event, **fields):
    transport.emit(event, **fields)


def load_config(path):
    config = json.loads(Path(path).read_text())
    for key in ('source', 'data', 'work', 'initializer', 'remote_script', 'remote_config', 'backup_script'):
        if not Path(config[key]).is_absolute():
            raise ValueError('Configuration paths must be absolute')
    if config['variant'] not in ('cosine', 'restarts'):
        raise ValueError('Unknown candidate')
    if config.get('dataset_environment') != 'sunny15' or config.get('expected_scenes') != 5000:
        raise ValueError('Require explicit new Sunny15 5K data contract')
    if Path(config['work']).name != config['variant']:
        raise ValueError('Separate work directory required for each candidate')
    return config


def command(c, *, resume=False, run=False):
    result = ['python', str(Path(c['source']) / 'scripts/runtime/prepare_sunny15_ab.sh')]
    # The tracked helper is bash; use bash so the qualified Python is selected
    # by the same script for prepare, fresh start and resume.
    result[0] = 'bash'
    result += [c['variant'], c['data'], str(Path(c['work']).parent)]
    if resume:
        result += ['--resume', 'auto']
    if run:
        result.append('--run')
    return result


def remote_env(c):
    env = transport.remote_env(c)
    env['PYTHONPATH'] += ':' + c['source'] + '/docker/bevfusion'
    env['BASE5_INITIALIZER'] = c['initializer']
    return env


def validate_environment_provenance(root, manifest, expected_count=5000):
    """Use real portable inventory/ledger/native metadata, not folder names."""
    root = Path(root)
    inventory = manifest.get('files', {})

    def checked_json_file(relative):
        relative = Path(relative)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe provenance path')
        path = root / relative
        entry = inventory.get(str(relative))
        if not entry or not path.is_file() or sha(path) != entry.get('sha256'):
            raise ValueError('Portable provenance inventory SHA differs')
        return path

    ledger = checked_json_file('provenance/original_selected_ledger.jsonl')
    if sha(ledger) != manifest.get('source_ledger_sha256'):
        raise ValueError('Portable selected-ledger SHA differs')
    rows = [json.loads(line) for line in ledger.read_text().splitlines() if line]
    provenance = manifest.get('provenance', {})
    if (len(rows) != expected_count or len(provenance) != expected_count or
            len({row['id'] for row in rows}) != expected_count or
            {row['id'] for row in rows} != set(provenance) or
            any(row.get('status') != 'accepted_candidate' for row in rows)):
        raise ValueError('Expected unique accepted Sunny15 selections and matching provenance')
    for row in rows:
        token = row['id']
        if Path(token).name != token or token in ('.', '..'):
            raise ValueError('Unsafe capture token')
        if provenance[token].get('original_capture_root') != row['capture_root']:
            raise ValueError('Selected ledger and portable original-root provenance differ')
        metadata = json.loads(checked_json_file(Path('data') / token / 'metadata.json').read_text())
        if metadata.get('fresh_environment_provenance', {}).get('environment') != 'sunny15':
            raise ValueError('Native metadata is not the new Sunny15 environment')


def data_gate(c):
    expected = c.get('dataset_manifest_sha256')
    if not expected or len(expected) != 64 or any(x not in '0123456789abcdef' for x in expected):
        raise ValueError('New Sunny15 data SHA not bound; old data cannot start this run')
    manifest_path = Path(c['data']) / 'manifest.json'
    if not manifest_path.is_file() or sha(manifest_path) != expected:
        raise ValueError('New Sunny15 data missing or manifest SHA differs')
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('scenes') != c['expected_scenes']:
        raise ValueError('Expected all 5000 accepted scenes')
    validate_environment_provenance(c['data'], manifest, c['expected_scenes'])
    return manifest


def prepared_manifest(c, resume=False):
    data_gate(c)
    work = Path(c['work'])
    manifest = json.loads((work / 'run_manifest.json').read_text())
    identity = manifest['identity']
    expected = dict(epochs=8, batch=4, workers=4, accumulate=1, lr=1e-4,
                    seed=261018, val_enabled=True, profile='512x1056',
                    training_label_policy='animal_to_pedestrian_v1', wandb_group=GROUP)
    if any(identity.get(k) != v for k, v in expected.items()):
        raise ValueError('Candidate settings differ from qualified eight-epoch plan')
    if list(identity.get('classes', ())) != ['vehicle', 'pedestrian', 'obstacle']:
        raise ValueError('Expected merged three-class labels')
    if identity.get('model', {}).get('compile_camera') is not False:
        raise ValueError('Training compile must remain OFF')
    if identity['dataset_manifest_sha256'] != c['dataset_manifest_sha256']:
        raise ValueError('Prepared run refers to different data')
    schedules = identity['param_schedulers']
    expected_type = 'MoraiWarmupCosineRestarts' if c['variant'] == 'restarts' else 'LinearLR'
    if schedules[0]['type'] != expected_type:
        raise ValueError('Prepared schedule differs')
    if c['variant'] == 'restarts' and (len(schedules) != 1 or
            schedules[0]['cycle_steps'] != identity['iterations_per_epoch'] * 4 or
            schedules[0]['warmup_steps'] != 500):
        raise ValueError('Restart cycle/warmup differs')
    for name, checksum in identity['source_hashes'].items():
        if sha(Path(c['source']) / name) != checksum:
            raise ValueError('Frozen detector source changed')
    if sha(c['initializer']) != identity['initial_checkpoint_sha256']:
        raise ValueError('Initializer changed')
    if sha(Path(c['source']) / c['config']) != identity['config_sha256']:
        raise ValueError('Model configuration changed')
    for name, checksum in c['support_hashes'].items():
        if sha(Path(c['source']) / name) != checksum:
            raise ValueError('Launch/diagnostic helper changed')
    started = (work / 'fresh_start.json').exists() or any(work.glob('*.pth')) or (work / 'last_checkpoint').exists()
    if resume:
        pointer = work / 'last_checkpoint'
        if not pointer.is_file():
            raise ValueError('No completed checkpoint for resume')
        checkpoint = Path(pointer.read_text().strip()).resolve(strict=True)
        if checkpoint.parent != work.resolve():
            raise ValueError('Resume checkpoint outside this candidate')
    elif started:
        raise ValueError('Run already started; explicit --run --resume required')
    return manifest


def qualify_environment(c):
    report = json.loads(Path(c['qualification_report']).read_text())
    proof = report.get('resume_payload_proof', {})
    required = ('model', 'optimizer_and_scaler', 'schedulers', 'python_rng', 'numpy_rng',
                'torch_rng', 'cuda_rng', 'iteration', 'no_compiler_key_prefix')
    if not report.get('valid') or not all(proof.get(name) is True for name in required):
        raise ValueError('GPU/resume qualification missing or failed')
    if report.get('lr_schedule') != ('warmup-restarts' if c['variant'] == 'restarts' else 'cosine'):
        raise ValueError('Qualification uses another schedule')
    gpu = transport.checked(['nvidia-smi', '--query-gpu=memory.total,memory.used,utilization.gpu',
                             '--format=csv,noheader,nounits'], capture_output=True, text=True).stdout.strip().splitlines()
    if len(gpu) != 1:
        raise ValueError('Require one GPU')
    total, used, util = map(lambda x: int(x.strip()), gpu[0].split(','))
    if total < 48000 or used > 1024 or util > 5:
        raise ValueError('Require idle 48GB GPU; no concurrent candidate on this card')
    if shutil.disk_usage('/workspace').free < 20 * 1024**3:
        raise ValueError('Require at least 20GiB free after data upload')
    return total


def backup_gate(c):
    receipt = Path(c['backup_qualification_report'])
    if not receipt.is_file():
        reused = Path(c.get('backup_reuse_receipt_remote', '/nonexistent'))
        if not reused.is_file() or sha(reused) != c.get('backup_reuse_receipt_sha256'):
            raise ValueError('Full-checkpoint backup or explicit reused qualification missing')
        result = backup_qualification.validate(json.loads(reused.read_text()), c, local=False)
        emit('sunny15_backup_qualification_reused', new_checkpoint_uploaded=False,
             first_new_epoch_backup_requires_actual_verification=True)
        return result
    backup = json.loads(receipt.read_text())
    status = backup.get('status', {})
    uploaded = status.get('uploaded', {})
    full = [record for record in uploaded.values() if record.get('resume_capable') is True
            and record.get('bytes', 0) > 0 and len(record.get('sha256', '')) == 64]
    if (not backup.get('valid') or not full or status.get('errors') or
            status.get('destination') != c['destination']):
        raise ValueError('Backup receipt does not prove a full resumable checkpoint at this destination')
    return backup


def remote_check(c, resume=False):
    data_gate(c)
    manifest = prepared_manifest(c, resume)
    total = qualify_environment(c)
    transport.checked(command(c, resume=resume), cwd=c['source'], env=remote_env(c), timeout=180)
    if not shutil.which('tmux') or not shutil.which('rclone'):
        raise ValueError('tmux/rclone missing')
    env = remote_env(c)
    os.environ.update({k: env[k] for k in ('HTTPS_PROXY', 'HTTP_PROXY', 'NO_PROXY')})
    import wandb
    run = wandb.Api(timeout=20).run(c['entity'] + '/' + c['project'] + '/' + c['probe_id'])
    if run.summary.get('qualification/online_upload') != 1:
        raise ValueError('W&B connectivity/authentication probe failed')
    backup_gate(c)
    emit('sunny15_remote_preflight_pass', contract=manifest['contract_sha256'], gpu_mib=total,
         variant=c['variant'], training_started=False)
    return manifest


def remote_status(c):
    work = Path(c['work'])
    result = dict(variant=c['variant'], training_started=(work / 'fresh_start.json').exists(), ready=False)
    for name in ('oneclick_status.json', 'backup_status.json'):
        path = work / name
        if path.is_file():
            result[name] = json.loads(path.read_text())
    try:
        data_gate(c)
        prepared_manifest(c, resume=result['training_started'])
        result['data_and_contract_ready'] = True
    except (ValueError, OSError, KeyError) as error:
        result['data_and_contract_ready'] = False
        result['blocker'] = str(error)
    # Status is not network/full-payload/Drive qualification. Never imply it is.
    result['requires_online_preflight'] = True
    emit('sunny15_status', **result)


def remote_start(c, resume=False):
    work = Path(c['work']); work.mkdir(parents=True, exist_ok=True)
    with (work.parent / '.gpu-launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for session in c['all_train_sessions']:
            if subprocess.run(['tmux', 'has-session', '-t', session], capture_output=True).returncode == 0:
                raise ValueError('Another candidate is running on this GPU')
        remote_check(c, resume)
        cmd = ['python', c['remote_script'], '--config', c['remote_config'], '--remote-worker']
        if resume:
            cmd.append('--resume')
        transport.checked(['tmux', 'new-session', '-d', '-s', c['train_session'], shlex.join(cmd)])
        emit('sunny15_training_started', session=c['train_session'])


def remote_worker(c, resume=False):
    work = Path(c['work'])
    with (work.parent / '.gpu-training.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        log = work / ('console_resume_' + str(int(time.time())) + '.log' if resume else 'console.log')
        error_type = None
        try:
            prepared_manifest(c, resume)
            transport.save(work / 'oneclick_status.json', dict(state='running', time=time.time(), log=str(log)))
            with log.open('a') as stream:
                code = subprocess.run(command(c, resume=resume, run=True), cwd=c['source'], env=remote_env(c),
                                      stdout=stream, stderr=subprocess.STDOUT).returncode
        except Exception as error:
            code, error_type = 1, type(error).__name__
        transport.save(work / 'oneclick_status.json', dict(state='finished' if code == 0 else 'failed',
                       time=time.time(), log=str(log), exit_code=code, error_type=error_type))
        return code


def backup_once(c):
    helper_path = ROOT / 'perception/detection3d/bevfusion/tools/backup_base5_vast_access_only.py'
    spec = importlib.util.spec_from_file_location('sunny15_access_only_backup', helper_path)
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    proxy = 'http://127.0.0.1:' + str(c['backup_relay_remote_port'])
    helper.REMOTE = ('import os\nos.environ.update(HTTPS_PROXY=%r, HTTP_PROXY=%r, '
                     'NO_PROXY="localhost,127.0.0.1")\n' % (proxy, proxy)) + helper.REMOTE
    args = transport.backup_arguments(c, Path(c['local_logs']))
    previous = sys.argv
    try:
        sys.argv = [str(helper_path)] + args
        try:
            helper.main()
        except SystemExit as error:
            return error.code or 0
    finally:
        sys.argv = previous


def backup_worker(c):
    logs = Path(c['local_logs']); logs.mkdir(parents=True, exist_ok=True)
    with (logs / '.backup.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        while True:
            try:
                with transport.logging_relay(c) as tunnels:
                    transport.save(logs / 'relay_ready.json', dict(ready=True, time=time.time()))
                    while tunnels.poll() is None:
                        code = backup_once(c)
                        state = transport.remote_status(c)
                        transport.save(logs / 'status.json', dict(state, checked_at=time.time()))
                        if code == 0 and state.get('state') in ('finished', 'failed') and not state['pending_checkpoints'] and not state['backup_errors']:
                            return 0
                        time.sleep(120)
                    raise ConnectionError('Logging/backup relay closed')
            except Exception as error:
                emit('sunny15_backup_retry', error_type=type(error).__name__)
                time.sleep(10)


def check_drive(c):
    """Scoped bounded write/readback; access token exists only in remote RAM."""
    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(Path.home() / '.config/rclone/rclone.conf')
    account = cfg[c['drive_remote']]
    token = json.loads(account['token'])
    body = urllib.parse.urlencode(dict(client_id=account['client_id'], client_secret=account['client_secret'],
        refresh_token=token['refresh_token'], grant_type='refresh_token')).encode()
    with urllib.request.urlopen('https://oauth2.googleapis.com/token', data=body, timeout=20) as response:
        fresh = json.load(response)
    expiry = datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(seconds=fresh['expires_in'])
    payload = dict(token=dict(access_token=fresh['access_token'], token_type='Bearer', expiry=expiry.isoformat()),
                   client_id=account['client_id'], root=c['drive_root'], destination=c['destination'],
                   proxy='http://127.0.0.1:' + str(c['backup_relay_remote_port']), variant=c['variant'])
    code = r'''import ctypes,json,os,subprocess,sys
p=json.load(sys.stdin)
fd=ctypes.CDLL(None).memfd_create(b'sunny15_drive_probe',0)
if fd<0: raise RuntimeError('memfd unavailable')
try:
 os.write(fd,('[backup]\ntype = drive\nroot_folder_id = '+p['root']+'\nclient_id = '+p['client_id']+'\ntoken = '+json.dumps(p['token'])+'\n').encode())
 env=dict(os.environ,RCLONE_CONFIG=f'/proc/{os.getpid()}/fd/{fd}',HTTPS_PROXY=p['proxy'],HTTP_PROXY=p['proxy'],NO_PROXY='localhost,127.0.0.1')
 target=p['destination']+'/_preflight_'+p['variant']+'/receipt.json'
 receipt=json.dumps({'purpose':'sunny15_online_preflight','training_started':False,'variant':p['variant']}).encode()
 opts=['--retries','1','--low-level-retries','1','--contimeout','5s','--timeout','15s']
 r=subprocess.run(['/usr/bin/rclone','rcat',target]+opts,input=receipt,env=env,capture_output=True,timeout=25)
 if r.returncode==0:
  r=subprocess.run(['/usr/bin/rclone','cat',target]+opts,env=env,capture_output=True,timeout=25)
  if r.returncode==0 and r.stdout!=receipt: raise ValueError('Readback differs')
 print(json.dumps({'event':'sunny15_drive_probe','write_readback':r.returncode==0,'training_started':False}),flush=True)
 sys.exit(0 if r.returncode==0 else 1)
finally: os.close(fd)
'''
    result = transport.ssh(c, 'python -c ' + shlex.quote(code), input=json.dumps(payload),
                           text=True, capture_output=True, timeout=60)
    print(result.stdout, end='')


def local_action(c, action, resume=False):
    remote_command = ['python', c['remote_script'], '--config', c['remote_config'], '--remote-' + action]
    if resume:
        remote_command.append('--resume')
    if action in ('status', 'prepare'):
        return transport.ssh(c, shlex.join(remote_command), timeout=240)
    if c.get('backup_reuse_receipt_sha256'):
        reused = Path(c['backup_reuse_receipt_local'])
        if sha(reused) != c['backup_reuse_receipt_sha256']:
            raise ValueError('Local reused-backup receipt changed')
        backup_qualification.validate(json.loads(reused.read_text()), c, local=True)
    existing = subprocess.run(['tmux', 'has-session', '-t', c['backup_session']], capture_output=True).returncode == 0
    with contextlib.nullcontext() if existing else transport.logging_relay(c):
        preflight = ['python', c['remote_script'], '--config', c['remote_config'], '--remote-check']
        if resume:
            preflight.append('--resume')
        manifest = transport.ssh(c, shlex.join(preflight),
                                 capture_output=True, text=True, timeout=240)
        print(manifest.stdout, end='')
        # The legacy helper is transport-only here: no E24 manifest/runner is used.
        check_drive(c)
    if action == 'check':
        emit('sunny15_ready_without_training', training_started=False, local_pc_must_stay_on=True)
        return
    if not existing:
        logs = Path(c['local_logs']); logs.mkdir(parents=True, exist_ok=True)
        worker = [sys.executable, str(Path(__file__).resolve()), '--config', c['local_config'], '--backup-worker']
        started = time.time()
        transport.checked(['tmux', 'new-session', '-d', '-s', c['backup_session'],
                           shlex.join(worker) + ' >> ' + shlex.quote(str(logs / 'watcher.log')) + ' 2>&1'])
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            path = logs / 'relay_ready.json'
            if path.is_file() and json.loads(path.read_text()).get('time', 0) >= started:
                break
            if subprocess.run(['tmux', 'has-session', '-t', c['backup_session']], capture_output=True).returncode:
                raise ValueError('Backup watcher exited before relay became ready')
            time.sleep(.5)
        else:
            raise TimeoutError('Backup relay did not become ready; training not started')
    transport.checked(['tmux', 'has-session', '-t', c['backup_session']], capture_output=True)
    transport.ssh(c, shlex.join(remote_command), timeout=240)


def initialize_configs(folder):
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    support = ('scripts/runtime/prepare_sunny15_ab.sh', 'scripts/runtime/base5_distance_diagnostics.py')
    reused = folder / 'reused_full_backup_qualification.json'
    for variant, ports in (('cosine', (19012, 19011, 19013)), ('restarts', (19022, 19021, 19023))):
        path = folder / (variant + '.json')
        if path.exists():
            raise FileExistsError('Preserve existing candidate configuration')
        c = dict(transport.DEFAULTS)
        c.update(source='/workspace/ajounice_sunny15_training_source_261018_v2',
            data='/workspace/base5_sunny15_jpeg90_pedestrian_merged_261018',
            work='/workspace/sunny15_ab_261018/' + variant, variant=variant,
            config='perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py',
            remote_script='/workspace/sunny15_runtime_261018_v5/sunny15_vast_training.py',
            remote_config='/workspace/sunny15_runtime_261018_v5/' + variant + '.json',
            dataset_manifest_sha256=None, dataset_environment='sunny15', expected_scenes=5000,
            qualification_report='/workspace/sunny15_env_qualification_261018_v1/' + variant + '/report.json',
            backup_qualification_report='/workspace/sunny15_backup_qualification_261018_v1/external_backup_report.json',
            backup_reuse_receipt_local=str(reused),
            backup_reuse_receipt_remote='/workspace/sunny15_runtime_261018_v5/reused_full_backup_qualification.json',
            backup_reuse_receipt_sha256=sha(reused) if reused.is_file() else None,
            all_train_sessions=['sunny15-cosine-training', 'sunny15-restarts-training'],
            train_session='sunny15-' + variant + '-training', backup_session='sunny15-' + variant + '-backup',
            destination='backup:model/bevfusion/base5/sunny15_ab_261018/checkpoint_backups',
            relay_local_port=ports[0], relay_remote_port=ports[1], backup_relay_remote_port=ports[2],
            local_config=str(path), local_logs=str(folder / variant),
            support_hashes={name: sha(ROOT / name) for name in support})
        c.pop('contract', None)
        transport.save(path, c)
    emit('sunny15_configs_created', directory=str(folder), data_bound=False, training_started=False)


def deploy(c):
    """Only new launcher files/config, not the old run or qualified source."""
    import base64
    files = {name: base64.b64encode(Path(__file__).with_name(name).read_bytes()).decode()
             for name in ('sunny15_vast_training.py', 'sunny15_backup_qualification.py',
                          'base5_vast_oneclick.py', 'base5_wandb_relay.py')}
    if c.get('backup_reuse_receipt_sha256'):
        receipt = Path(c['backup_reuse_receipt_local'])
        if sha(receipt) != c['backup_reuse_receipt_sha256']:
            raise ValueError('Reused qualification receipt changed')
        files[Path(c['backup_reuse_receipt_remote']).name] = base64.b64encode(receipt.read_bytes()).decode()
    payload = dict(files=files, config=c)
    code = r'''import base64,hashlib,json,pathlib,sys
p=json.load(sys.stdin); c=p['config']; folder=pathlib.Path(c['remote_script']).parent
folder.mkdir(parents=True,exist_ok=True)
for name,encoded in p['files'].items():
 content=base64.b64decode(encoded); target=folder/name
 if target.exists() and target.read_bytes()!=content: raise FileExistsError('Use a new runtime directory for changed launchers')
 if not target.exists(): target.write_bytes(content)
target=pathlib.Path(c['remote_config'])
if target.exists() and json.loads(target.read_text())!=c: raise FileExistsError('Preserve existing launcher config; use explicit data binding')
if not target.exists(): target.write_text(json.dumps(c,indent=2)+'\n')
print(json.dumps({'event':'sunny15_launcher_deployed','files_sha256':{name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in p['files']},'training_started':False}))
'''
    result = transport.ssh(c, 'python -c ' + shlex.quote(code), input=json.dumps(payload),
                           text=True, capture_output=True, timeout=45)
    print(result.stdout, end='')


def bind_data(c, local_manifest):
    path = Path(local_manifest).resolve(strict=True)
    manifest = json.loads(path.read_text())
    if manifest.get('scenes') != 5000 or manifest.get('image_policy') != dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'):
        raise ValueError('Bind only a verified new all-JPEG 5K portable manifest')
    validate_environment_provenance(path.parent, manifest)
    checksum = sha(path)
    if c.get('dataset_manifest_sha256') not in (None, checksum):
        raise ValueError('Existing data binding differs; do not reuse candidate')
    c = dict(c, dataset_manifest_sha256=checksum)
    code = r'''import json,pathlib,sys
c=json.load(sys.stdin); path=pathlib.Path(c['remote_config']); previous=json.loads(path.read_text())
work=pathlib.Path(c['work'])
if (work/'fresh_start.json').exists() or any(work.glob('*.pth')): raise ValueError('Cannot change started run')
if {k:v for k,v in previous.items() if k!='dataset_manifest_sha256'}!={k:v for k,v in c.items() if k!='dataset_manifest_sha256'}: raise ValueError('Only data binding may change')
if previous.get('dataset_manifest_sha256') not in (None,c['dataset_manifest_sha256']): raise ValueError('Existing data binding differs')
path.write_text(json.dumps(c,indent=2)+'\n')
print(json.dumps({'event':'sunny15_data_bound','sha256':c['dataset_manifest_sha256'],'training_started':False}))
'''
    result = transport.ssh(c, 'python -c ' + shlex.quote(code), input=json.dumps(c),
                           text=True, capture_output=True, timeout=25)
    transport.save(c['local_config'], c)
    print(result.stdout, end='')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    modes = parser.add_mutually_exclusive_group()
    for name in ('status', 'prepare', 'check', 'run', 'remote-status', 'remote-prepare',
                 'remote-check', 'remote-start', 'remote-worker', 'backup-worker', 'deploy'):
        modes.add_argument('--' + name, action='store_true')
    modes.add_argument('--init-config-dir', type=Path)
    modes.add_argument('--bind-local-manifest', type=Path)
    modes.add_argument('--qualify-backup-reuse', type=Path,
                       help='Create an explicit reuse receipt using old full evidence and read-only remote helper SHA')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    try:
        if args.init_config_dir:
            initialize_configs(args.init_config_dir)
            return 0
        if not args.config:
            parser.error('--config required except for --init-config-dir')
        c = load_config(args.config)
        c['local_config'] = str(args.config.resolve())
        if args.resume and not any((args.run, args.check, args.remote_check, args.remote_start, args.remote_worker)):
            raise ValueError('Resume requires explicit run/check mode')
        if args.qualify_backup_reuse:
            backup_qualification.make_receipt(c, args.qualify_backup_reuse)
            emit('sunny15_reused_backup_qualification_created', new_checkpoint_uploaded=False,
                 first_new_epoch_backup_requires_actual_verification=True)
        elif args.deploy:
            deploy(c)
        elif args.bind_local_manifest:
            bind_data(c, args.bind_local_manifest)
        elif args.remote_status:
            remote_status(c)
        elif args.remote_prepare:
            data_gate(c)
            transport.checked(command(c), cwd=c['source'], env=remote_env(c), timeout=180)
            prepared_manifest(c)
        elif args.remote_check:
            remote_check(c, args.resume)
        elif args.remote_start:
            remote_start(c, args.resume)
        elif args.remote_worker:
            return remote_worker(c, args.resume)
        elif args.backup_worker:
            return backup_worker(c)
        else:
            local_action(c, 'start' if args.run else 'check' if args.check else 'prepare' if args.prepare else 'status', args.resume)
        return 0
    except Exception as error:
        # Do not print arbitrary OAuth, W&B or SSH exception messages.
        emit('sunny15_launcher_blocked', error_type=type(error).__name__)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
