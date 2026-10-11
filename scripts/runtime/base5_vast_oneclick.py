#!/usr/bin/env python3
"""One local command starts qualified remote training and access-only backups.

The frozen detector source is not edited. Drive renewable credentials stay on
the local PC; only W&B's separately authenticated remote login is used there.
No training is started by --check or by deploying this script.
"""
import argparse
import configparser
import contextlib
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
import threading
import time
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
DEFAULTS = dict(
    host='root@175.155.64.164', port=19463,
    source='/workspace/base5_training_pedestrian_merged_source_261011',
    data='/workspace/base5_sunny13_jpeg90_pedestrian_merged_261011',
    work='/workspace/base5_training_sunny13_pedestrian_merged_v2_55272391_261011',
    initializer='/workspace/base5_nuscenes_camera_lidar_xy_261011.pth',
    config='perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_pedestrian_merged.py',
    contract='486aee660511fcd04b393e7ac0acd49ea77c45bded857141a4c81024ca665843',
    remote_script='/workspace/base5_oneclick_pedestrian_merged_261011/base5_vast_oneclick.py',
    entity='AjouNice2026', project='morai-3d-object-detection',
    probe_id='base5-health-online-probe-261011',
    train_session='base5-sunny13-training', backup_session='base5-sunny13-backup',
    drive_remote='ajou_drive_private', drive_root='12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP',
    destination='backup:model/bevfusion/base5/checkpoint_backups',
    # The uploader is independently repaired; old frozen training source and
    # its checkpoint contract must not be edited just to fix backup mechanics.
    backup_script='/workspace/base5_backup_repair_261011/backup_base5_checkpoints.py',
    relay_local_port=18992, relay_remote_port=18991, backup_relay_remote_port=18993,
)


def emit(event, **values):
    print(json.dumps(dict(event=event, **values), ensure_ascii=False), flush=True)


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + '.partial')
    pending.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    pending.replace(path)


def checked(command, **kwargs):
    return subprocess.run(command, check=True, timeout=kwargs.pop('timeout', 120), **kwargs)


def ssh(c, command, **kwargs):
    return checked(['ssh', '-p', str(c['port']), '-o', 'BatchMode=yes',
                    '-o', 'ConnectTimeout=10', c['host'], command], **kwargs)


def train_command(c, resume=False):
    command = [sys.executable, '-u', str(Path(c['source']) /
               'perception/detection3d/bevfusion/tools/run_base5_direct_training.py'),
               '--data', c['data'], '--config', str(Path(c['source']) / c['config']),
               '--profile', '512x1056', '--epochs', '24', '--batch', '4', '--workers', '4',
               '--lr', '1e-4', '--warmup-updates', '500', '--init-checkpoint', c['initializer'],
               '--work-dir', c['work'], '--wandb']
    if resume:
        command += ['--resume', 'auto']
    return command


def remote_env(c):
    env = dict(os.environ)
    env.update(PYTHONPATH=c['source'] + ':' + c['source'] + '/third_party/mmdetection3d',
               WANDB_ENTITY=c['entity'], WANDB_MODE='online', WANDB_DISABLE_CODE='true',
               WANDB_SILENT='true', WANDB_DIR=c['work'], OMP_NUM_THREADS='4',
               MKL_NUM_THREADS='4', OPENBLAS_NUM_THREADS='1', CUDA_VISIBLE_DEVICES='0')
    env.update(HTTPS_PROXY='http://127.0.0.1:' + str(c['relay_remote_port']),
               HTTP_PROXY='http://127.0.0.1:' + str(c['relay_remote_port']),
               NO_PROXY='localhost,127.0.0.1')
    return env


def validate_manifest(c, resume=False):
    work = Path(c['work'])
    manifest = json.loads((work / 'run_manifest.json').read_text())
    if manifest['contract_sha256'] != c['contract']:
        raise ValueError('Prepared training contract differs; do not start')
    identity = manifest['identity']
    expected = dict(epochs=24, batch=4, workers=4, accumulate=1,
                    train_count=4255, val_enabled=True, lr=1e-4)
    if any(identity.get(key) != value for key, value in expected.items()):
        raise ValueError('Training settings differ from qualification')
    if (identity.get('training_label_policy') != 'animal_to_pedestrian_v1' or
            list(identity.get('classes', ())) != ['vehicle', 'pedestrian', 'obstacle']):
        raise ValueError('Expected merged three-class Base5; never resume the old four-class run')
    source = Path(c['source'])
    for name, expected_hash in identity['source_hashes'].items():
        if hashlib.sha256((source / name).read_bytes()).hexdigest() != expected_hash:
            raise ValueError('Frozen source changed: ' + name)
    if hashlib.sha256(Path(c['initializer']).read_bytes()).hexdigest() != identity['initial_checkpoint_sha256']:
        raise ValueError('Initializer checksum differs')
    if hashlib.sha256((source / c['config']).read_bytes()).hexdigest() != identity['config_sha256']:
        raise ValueError('Configuration checksum differs')
    if hashlib.sha256((Path(c['data']) / 'manifest.json').read_bytes()).hexdigest() != identity['dataset_manifest_sha256']:
        raise ValueError('Dataset manifest checksum differs')
    started = (work / 'fresh_start.json').exists() or (work / 'last_checkpoint').exists() or any(work.glob('*.pth'))
    if resume:
        if not (work / 'last_checkpoint').is_file():
            raise ValueError('No completed checkpoint to resume')
    elif started:
        raise ValueError('Run already started; use explicit --resume, never overwrite')
    return manifest


def check_remote(c, resume=False):
    emit('checking_frozen_contract')
    validate_manifest(c, resume)
    if shutil.disk_usage(c['work']).free < 15 * 1024**3:
        raise ValueError('Require at least 15 GiB disk headroom')
    if not shutil.which('tmux') or not shutil.which('rclone'):
        raise ValueError('Remote tmux/rclone missing')
    gpu = checked(['nvidia-smi', '--query-gpu=memory.total,memory.used,utilization.gpu',
                   '--format=csv,noheader,nounits'], capture_output=True, text=True).stdout.strip()
    rows = gpu.splitlines()
    if len(rows) != 1:
        raise ValueError('Expected one GPU')
    total, used, util = (int(v.strip()) for v in rows[0].split(','))
    if total < 48000 or used > 512 or util > 5:
        raise ValueError('Require idle 48GB GPU before cold compiler preparation')
    # Read-only authentication/network check of the separately uploaded probe.
    emit('checking_wandb_online_probe')
    os.environ.update({k: v for k, v in remote_env(c).items()
                       if k in ('HTTPS_PROXY', 'HTTP_PROXY', 'NO_PROXY')})
    import wandb
    if tuple(int(x) for x in wandb.__version__.split('.')[:3]) < (0, 22, 3):
        raise ValueError('W&B SDK >= 0.22.3 required for current API keys')
    run = wandb.Api(timeout=20).run(c['entity'] + '/' + c['project'] + '/' + c['probe_id'])
    if run.summary.get('qualification/online_upload') != 1:
        raise ValueError('W&B online upload probe not qualified')
    emit('checking_resolved_training_configuration')
    checked(train_command(c, resume), cwd=c['source'], env=remote_env(c), timeout=180)
    emit('remote_preflight_pass', contract=c['contract'], gpu_mib=total,
         wandb_url='https://wandb.ai/' + c['entity'] + '/' + c['project'] + '/runs/' +
         Path(c['work']).name + '-' + c['contract'][:12])


def start_remote(c, resume=False):
    work = Path(c['work'])
    with (work / '.oneclick-launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if subprocess.run(['tmux', 'has-session', '-t', c['train_session']],
                          capture_output=True).returncode == 0:
            raise ValueError('Training tmux already exists; do not start another job')
        check_remote(c, resume)
        job = [sys.executable, str(Path(__file__).resolve()), '--remote-worker']
        if resume:
            job.append('--resume')
        save(work / 'oneclick_status.json', dict(state='launching', time=time.time(), resume=resume))
        checked(['tmux', 'new-session', '-d', '-s', c['train_session'], shlex.join(job)])
        emit('training_tmux_started', session=c['train_session'])


def remote_worker(c, resume=False):
    work = Path(c['work'])
    with (work / '.oneclick-training.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        log = work / ('console_resume_' + str(int(time.time())) + '.log' if resume else 'console.log')
        error_type = None
        try:
            validate_manifest(c, resume)
            save(work / 'oneclick_status.json', dict(state='running', time=time.time(), log=str(log)))
            # File-backed output survives SSH disconnection; W&B is the dashboard.
            with log.open('a') as stream:
                child = subprocess.Popen(train_command(c, resume) + ['--run'], cwd=c['source'],
                                         env=remote_env(c), stdout=stream, stderr=subprocess.STDOUT)
                code = child.wait()
        except Exception as error:
            code, error_type = 1, type(error).__name__
        save(work / 'oneclick_status.json', dict(state='finished' if code == 0 else 'failed',
                                                exit_code=code, time=time.time(), log=str(log),
                                                error_type=error_type))
        emit('training_finished', exit_code=code, log=str(log))
        return code


def check_drive(c):
    emit('checking_drive_connectivity')
    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(Path.home() / '.config/rclone/rclone.conf')
    account = cfg[c['drive_remote']]
    token = json.loads(account['token'])
    body = urllib.parse.urlencode(dict(client_id=account['client_id'],
        client_secret=account['client_secret'], refresh_token=token['refresh_token'],
        grant_type='refresh_token')).encode()
    with urllib.request.urlopen('https://oauth2.googleapis.com/token', data=body, timeout=20) as response:
        fresh = json.load(response)
    expiry = datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(seconds=fresh['expires_in'])
    payload = dict(token=dict(access_token=fresh['access_token'], token_type='Bearer', expiry=expiry.isoformat()),
                   client_id=account['client_id'], root=c['drive_root'], contract=c['contract'],
                   proxy='http://127.0.0.1:' + str(c['relay_remote_port']))
    code = """import ctypes,json,os,subprocess,sys
p=json.load(sys.stdin)
fd=ctypes.CDLL(None).memfd_create(b'base5_drive_preflight',0)
if fd<0: raise RuntimeError('memfd unavailable')
os.write(fd,('[backup]\\ntype = drive\\nroot_folder_id = '+p['root']+'\\nclient_id = '+p['client_id']+'\\ntoken = '+json.dumps(p['token'])+'\\n').encode())
env=dict(os.environ,RCLONE_CONFIG=f'/proc/{os.getpid()}/fd/{fd}',HTTPS_PROXY=p['proxy'],HTTP_PROXY=p['proxy'],NO_PROXY='localhost,127.0.0.1')
r=subprocess.run(['/usr/bin/rclone','lsf','backup:model/bevfusion/base5/checkpoint_backups','--max-depth','1','--retries','1','--low-level-retries','1','--contimeout','5s','--timeout','15s'],env=env,capture_output=True,timeout=30)
if r.returncode==0:
 receipt=json.dumps({'contract':p['contract'],'purpose':'oneclick_preflight','training_started':False}).encode()
 target='backup:model/bevfusion/base5/checkpoint_backups/_oneclick_probe_261011/receipt.json'
 opts=['--retries','1','--low-level-retries','1','--contimeout','5s','--timeout','15s']
 r=subprocess.run(['/usr/bin/rclone','rcat',target]+opts,input=receipt,env=env,capture_output=True,timeout=30)
 if r.returncode==0:
  r=subprocess.run(['/usr/bin/rclone','cat',target]+opts,env=env,capture_output=True,timeout=30)
  if r.returncode==0 and r.stdout!=receipt: raise ValueError('Drive probe readback differs')
os.close(fd)
reason=next((s for s in ['directory not found','invalid_grant','403','timeout','lookup','error loading config','illegal seek','invalid_client'] if s.lower() in r.stderr.decode().lower()),'unclassified')
print(json.dumps({'drive_preflight_pass':r.returncode==0,'error_class':None if r.returncode==0 else reason}))
sys.exit(0 if r.returncode==0 else 1)
"""
    try:
        result = ssh(c, 'python -c ' + shlex.quote(code), input=json.dumps(payload),
                     capture_output=True, text=True, timeout=110)
    except subprocess.CalledProcessError as error:
        # The remote prints only classified status, never rclone/auth stderr.
        if error.stdout:
            print(error.stdout, end='', flush=True)
        raise
    emit('drive_preflight_pass', write_readback=True,
         credentials='local renewable / remote RAM access-only')


def remote_status(c):
    code = """import json,pathlib
w=pathlib.Path(%r)
s=json.loads((w/'oneclick_status.json').read_text()) if (w/'oneclick_status.json').is_file() else {'state':'waiting'}
b=json.loads((w/'backup_status.json').read_text()) if (w/'backup_status.json').is_file() else {}
s['backup_errors']=b.get('errors',[])
s['pending_checkpoints']=[p.name for p in w.glob('*.pth') if b.get('uploaded',{}).get(p.name,{}).get('fingerprint') != [p.stat().st_size,p.stat().st_mtime_ns]]
s['uploaded_count']=len(b.get('uploaded',{}))
print(json.dumps(s))
""" % c['work']
    result = ssh(c, 'python -c ' + shlex.quote(code), capture_output=True, text=True, timeout=30)
    return json.loads(result.stdout)


@contextlib.contextmanager
def logging_relay(c):
    from base5_wandb_relay import server
    proxy = server(c['relay_local_port'])
    thread = threading.Thread(target=proxy.serve_forever, daemon=True)
    thread.start()
    tunnels = []
    try:
        ports = tuple(dict.fromkeys((c['relay_remote_port'],
                      c.get('backup_relay_remote_port', c['relay_remote_port']))))
        for port in ports:
            # Independent SSH TCP connections: bulk checkpoint readback must
            # not head-of-line block small W&B authentication/log requests.
            tunnels.append(subprocess.Popen(['ssh', '-N', '-T', '-p', str(c['port']),
                '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=15',
                '-o', 'ServerAliveCountMax=3', '-R',
                '127.0.0.1:' + str(port) + ':127.0.0.1:' + str(c['relay_local_port']),
                c['host']], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        # Verify actual bind address: sshd GatewayPorts must not expose it.
        probe = """import pathlib,time
ports=%r
for port in ports:
 for attempt in range(20):
  listeners=[]
  for name in ('tcp','tcp6'):
   for line in pathlib.Path('/proc/net/'+name).read_text().splitlines()[1:]:
    fields=line.split()
    address,p=fields[1].split(':')
    if int(p,16)==port and fields[3]=='0A': listeners.append(address)
  if listeners:
   assert all(a in ('0100007F','00000000000000000000000001000000') for a in listeners), 'Relay must bind loopback only'
   break
  time.sleep(.1)
 else: raise RuntimeError('SSH relay listener unavailable')
""" % (ports,)
        ssh(c, 'python -c ' + shlex.quote(probe), capture_output=True, timeout=20)
        if any(tunnel.poll() is not None for tunnel in tunnels):
            raise ValueError('W&B logging SSH relay could not start')
        yield RelayTunnels(tunnels)
    finally:
        for tunnel in tunnels:
            tunnel.terminate()
            try:
                tunnel.wait(timeout=5)
            except subprocess.TimeoutExpired:
                tunnel.kill(); tunnel.wait()
        proxy.shutdown()
        proxy.server_close()


class RelayTunnels:
    def __init__(self, tunnels):
        self.tunnels = tunnels

    def poll(self):
        for tunnel in self.tunnels:
            code = tunnel.poll()
            if code is not None:
                return code
        return None


def backup_worker(c):
    # Reconnect logging transport independently; never restart/stop GPU training.
    while True:
        try:
            with logging_relay(c) as tunnel:
                return backup_poll_worker(c, tunnel)
        except Exception as error:
            emit('logging_relay_retry', error_type=type(error).__name__)
            time.sleep(10)


def backup_arguments(c, logs):
    return ['--host', c['host'], '--port', str(c['port']), '--work-dir', c['work'],
            '--destination', c['destination'], '--root-folder-id', c['drive_root'],
            '--rclone-remote', c['drive_remote'], '--rclone-binary', '/usr/bin/rclone',
            '--backup-script', c.get('backup_script') or c['source'] + '/perception/detection3d/bevfusion/tools/backup_base5_checkpoints.py',
            '--part-mib', '32', '--transfers', '16', '--timeout', '1200',
            '--watch', '--max-polls', '1', '--local-readback', '--log', str(logs / 'drive_backup.jsonl')]


def backup_once(c):
    # Reuse the qualified uploader unchanged. Only its remote subprocess network
    # environment is routed through the authenticated loopback SSH tunnel.
    path = ROOT / 'perception/detection3d/bevfusion/tools/backup_base5_vast_access_only.py'
    spec = importlib.util.spec_from_file_location('access_only_backup', path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    proxy = 'http://127.0.0.1:' + str(c.get('backup_relay_remote_port', c['relay_remote_port']))
    helper.REMOTE = ('import os\nos.environ.update(HTTPS_PROXY=%r, HTTP_PROXY=%r, '
                     'NO_PROXY="localhost,127.0.0.1")\n' % (proxy, proxy)) + helper.REMOTE
    logs = ROOT / 'artifacts/evaluation/bevfusion/base5_oneclick_261011'
    sys.argv = [str(path)] + backup_arguments(c, logs)
    helper.main()


def backup_poll_worker(c, tunnel=None):
    logs = ROOT / 'artifacts/evaluation/bevfusion/base5_oneclick_261011'
    logs.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, '-u', str(Path(__file__).resolve()), '--backup-once']
    with (logs / '.backup.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        while True:
            if tunnel is not None and tunnel.poll() is not None:
                raise ConnectionError('W&B SSH relay disconnected')
            try:
                code = subprocess.run(command, timeout=1350).returncode
                status = remote_status(c)
                save(logs / 'status.json', dict(status, checked_at=time.time(), backup_poll_exit=code))
                emit('backup_watch_status', **status)
                if status.get('state') in ('finished', 'failed') and code == 0 and \
                        not status['backup_errors'] and not status['pending_checkpoints']:
                    emit('backup_watch_finished', training_state=status['state'],
                         uploaded_count=status['uploaded_count'])
                    return 0
            except Exception as error:
                emit('backup_retry', error_type=type(error).__name__)
            time.sleep(120)


def start_local(c, resume=False, check=False):
    if not shutil.which('tmux'):
        raise ValueError('Local tmux required')
    command = ['python', c['remote_script'], '--remote-check' if check else '--remote-start']
    if resume:
        command.append('--resume')
    backup_exists = subprocess.run(['tmux', 'has-session', '-t', c['backup_session']],
                                   capture_output=True).returncode == 0
    # An existing backup worker owns the same relay ports; don't bind twice on resume.
    relay = contextlib.nullcontext() if backup_exists else logging_relay(c)
    with relay:
        # Validate BEFORE starting any background job or GPU training.
        preflight = ['python', c['remote_script'], '--remote-check']
        if resume:
            preflight.append('--resume')
        ssh(c, shlex.join(preflight), timeout=240)
        check_drive(c)
    if check:
        save(ROOT / 'artifacts/evaluation/bevfusion/base5_oneclick_261011/preflight.json',
             dict(checked_at=time.time(), contract=c['contract'], remote_preflight=True,
                  wandb_online=True, drive_write_readback=True, training_started=False,
                  local_pc_must_stay_on=True))
        emit('ready_without_training', local_pc_must_stay_on=True)
        return
    logs = ROOT / 'artifacts/evaluation/bevfusion/base5_oneclick_261011'
    logs.mkdir(parents=True, exist_ok=True)
    if not backup_exists:
        worker = shlex.join([sys.executable, '-u', str(Path(__file__).resolve()), '--backup-worker'])
        checked(['tmux', 'new-session', '-d', '-s', c['backup_session'],
                 worker + ' >> ' + shlex.quote(str(logs / 'watcher.log')) + ' 2>&1'])
    time.sleep(1)
    checked(['tmux', 'has-session', '-t', c['backup_session']], capture_output=True)
    ssh(c, shlex.join(command), timeout=240)
    emit('oneclick_started', remote_training_session=c['train_session'],
         local_backup_session=c['backup_session'], local_pc_must_stay_on=True,
         wandb_url='https://wandb.ai/' + c['entity'] + '/' + c['project'] + '/runs/' +
         Path(c['work']).name + '-' + c['contract'][:12])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check', action='store_true', help='Preflight only; no training or resident watcher')
    modes.add_argument('--remote-check', action='store_true')
    modes.add_argument('--remote-start', action='store_true')
    modes.add_argument('--remote-worker', action='store_true')
    modes.add_argument('--backup-worker', action='store_true')
    modes.add_argument('--backup-once', action='store_true')
    parser.add_argument('--resume', action='store_true', help='Explicit same-contract epoch checkpoint resume')
    args = parser.parse_args()
    try:
        if args.remote_check:
            check_remote(DEFAULTS, args.resume)
        elif args.remote_start:
            start_remote(DEFAULTS, args.resume)
        elif args.remote_worker:
            return remote_worker(DEFAULTS, args.resume)
        elif args.backup_worker:
            return backup_worker(DEFAULTS)
        elif args.backup_once:
            return backup_once(DEFAULTS)
        else:
            start_local(DEFAULTS, args.resume, args.check)
        return 0
    except Exception as error:
        # OAuth/W&B/SSH exception messages may contain credentials or URLs.
        # Detailed safe progress is logged separately; never dump auth payloads.
        emit('oneclick_failed', error_type=type(error).__name__)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
