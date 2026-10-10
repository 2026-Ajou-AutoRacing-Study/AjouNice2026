#!/usr/bin/env python3
"""Local credential refresh; bounded remote full-checkpoint Drive qualification.

Only a short-lived access token crosses SSH stdin. Refresh tokens and client
secrets stay local; the remote config is anonymous RAM, not an on-disk file.
Watch mode refreshes locally for each bounded remote poll; the local watcher
must remain alive for unattended backups. No renewable credential is remote.
"""
import argparse
import configparser
import datetime
import json
from pathlib import Path
import shlex
import subprocess
import urllib.parse
import urllib.request


REMOTE = r'''
import ctypes,json,os,signal,subprocess,sys,time
from pathlib import Path
p=json.load(sys.stdin)
work=Path(p['work_dir']).resolve(strict=True)
stage=work if p['watch'] else work.parent/'external_backup_probe'
if not p['watch']:
    if stage.exists(): raise FileExistsError('Use a fresh qualification stage')
    stage.mkdir()
checkpoints=sorted(work.glob('epoch_*.pth'),key=lambda f:int(f.stem.split('_')[1]))
if not checkpoints:
    if p['watch']:
        print(json.dumps(dict(event='waiting_for_completed_checkpoint')),flush=True);sys.exit(0)
    raise ValueError('No completed epoch checkpoint')
latest=checkpoints[-1]
if not p['watch']:
    for source in [latest]+[work/n for n in ('run_manifest.json','resolved_train.py','resolved_resume.py') if (work/n).is_file()]:
        os.link(source,stage/source.name)
token=json.dumps(dict(access_token=p['access_token'],token_type='Bearer',expiry=p['expiry']))
cfg='[backup]\ntype = drive\nroot_folder_id = '+p['root_folder_id']+'\nclient_id = '+p['client_id']+'\ntoken = '+token+'\n'
fd=ctypes.CDLL(None).memfd_create(b'checkpoint_access_only',0)
if fd<0: raise RuntimeError('Memory-only authentication unavailable')
os.write(fd,cfg.encode())
env=dict(os.environ,RCLONE_CONFIG=f'/proc/{os.getpid()}/fd/{fd}')
env['PATH']=str(Path(p['rclone_binary']).parent)+':'+env['PATH']
# Do not spend minutes retrying a large checkpoint when the host cannot reach
# Drive at all. This read-only preflight is deliberately bounded and secret-free.
try:
    preflight=subprocess.run([p['rclone_binary'],'about','backup:','--json',
        '--retries','1','--low-level-retries','1','--contimeout','3s','--timeout','10s'],
        env=env,capture_output=True,text=True,timeout=15)
except subprocess.TimeoutExpired:
    print(json.dumps(dict(event='connectivity_failed',reason='bounded_preflight_timeout')),flush=True)
    sys.exit(1)
if preflight.returncode:
    reason='DNS_lookup_timeout' if 'lookup' in preflight.stderr and 'timeout' in preflight.stderr else 'Drive_preflight_failed'
    print(json.dumps(dict(event='connectivity_failed',reason=reason)),flush=True)
    sys.exit(1)
command=['python',p['backup_script'],'--work-dir',str(stage),'--destination',p['destination'],'--once']
print(json.dumps(dict(event='backup_started',checkpoint=latest.name,bytes=latest.stat().st_size,
    destination=p['destination'],timeout_seconds=p['timeout'])),flush=True)
started=time.monotonic()
process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,
    env=env,start_new_session=True)
while process.poll() is None and time.monotonic()-started<p['timeout']:
    time.sleep(15)
    print(json.dumps(dict(event='backup_progress',elapsed_seconds=round(time.monotonic()-started,1))),flush=True)
if process.poll() is None:
    os.killpg(process.pid,signal.SIGTERM)
    try: process.wait(timeout=10)
    except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL);process.wait()
stdout=process.stdout.read()
if process.returncode!=0:
    failed_status=json.loads((stage/'backup_status.json').read_text()) if (stage/'backup_status.json').exists() else {}
    print(json.dumps(dict(event='backup_failed',returncode=process.returncode,
        errors=failed_status.get('errors',[]),
        elapsed_seconds=time.monotonic()-started)),flush=True)
    sys.exit(1)
status=json.loads((stage/'backup_status.json').read_text())
if status.get('errors') or not status.get('uploaded',{}):
    raise ValueError('Incomplete external backup')
# Same watcher state must not re-upload the same immutable checkpoint.
again=subprocess.run(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=60)
if again.returncode!=0: raise ValueError('Idempotent backup recheck failed')
report=dict(valid=True,elapsed_seconds=time.monotonic()-started,stage=str(stage),
    status=status,duplicate_recheck_passed=True,
    credential_scope='short-lived access-only config in anonymous RAM; no refresh token or client secret on remote')
report_path=work/'external_backup_report.json' if p['watch'] else work.parent/'external_backup_report.json'
report_path.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
os.close(fd)
'''


def poll(a):
    cfg = configparser.ConfigParser(interpolation=None)
    cfg.read(Path.home()/'.config/rclone/rclone.conf')
    c = cfg[a.rclone_remote]
    token = json.loads(c['token'])
    body = urllib.parse.urlencode(dict(client_id=c['client_id'],client_secret=c['client_secret'],
        refresh_token=token['refresh_token'],grant_type='refresh_token')).encode()
    try:
        with urllib.request.urlopen('https://oauth2.googleapis.com/token', data=body, timeout=20) as response:
            fresh = json.load(response)
    except Exception as error:
        raise RuntimeError('Token refresh failed: '+type(error).__name__) from None
    expiry = datetime.datetime.now(datetime.timezone.utc)+datetime.timedelta(seconds=fresh['expires_in'])
    payload = dict(access_token=fresh['access_token'],client_id=c['client_id'],expiry=expiry.isoformat(),
        work_dir=a.work_dir,destination=a.destination,root_folder_id=a.root_folder_id,timeout=a.timeout,watch=a.watch,
        rclone_binary=a.rclone_binary,
        backup_script=a.backup_script)
    a.log.parent.mkdir(parents=True,exist_ok=True)
    child = subprocess.Popen(['ssh','-p',str(a.port),'-o','BatchMode=yes','-o','ConnectTimeout=10',
        a.host,'python -c '+shlex.quote(REMOTE)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,text=True)
    child.stdin.write(json.dumps(payload));child.stdin.close()
    with a.log.open('a' if a.watch else 'w') as log:
        for line in child.stdout:
            safe = line.replace(fresh['access_token'],'[REDACTED]')
            log.write(safe);log.flush();print(safe,end='',flush=True)
    return child.wait()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', required=True)
    p.add_argument('--port', type=int, required=True)
    p.add_argument('--work-dir', required=True)
    p.add_argument('--destination', required=True, help='backup:path within the authorized Drive root')
    p.add_argument('--root-folder-id', required=True)
    p.add_argument('--rclone-remote', default='ajou_drive_private')
    p.add_argument('--timeout', type=int, default=1200)
    p.add_argument('--log', type=Path, required=True)
    p.add_argument('--rclone-binary', default='/tmp/ajou_rclone_probe_tool/rclone-v1.75.2-linux-amd64/rclone',
                   help='Absolute rclone path on the remote instance')
    p.add_argument('--backup-script', default='/root/AjouNice2026/perception/detection3d/bevfusion/tools/backup_base5_checkpoints.py',
                   help='Absolute backup script path on the remote instance; override after migration')
    p.add_argument('--watch', action='store_true', help='Keep a local watcher alive; refresh access per poll')
    p.add_argument('--interval-sec', type=int, default=120)
    p.add_argument('--max-polls', type=int, default=0, help='0 is continuous; positive values bound watcher qualification')
    a = p.parse_args()
    if not a.destination.startswith('backup:') or not 60<=a.timeout<=1800:
        p.error('Require scoped backup remote and a bounded deadline')
    if a.interval_sec < 60 or a.max_polls < 0:
        p.error('Require at least 60 seconds between polls and nonnegative budget')
    if not all(Path(path).is_absolute() for path in (a.rclone_binary, a.backup_script)):
        p.error('Remote executable paths must be absolute')
    import time
    count=0
    while True:
        try:
            code=poll(a)
        except Exception as error:
            # OAuth/SSH exceptions can contain request details; report types only.
            print(json.dumps(dict(event='backup_poll_error',error=type(error).__name__)),flush=True)
            code=1
        count+=1
        if not a.watch or (a.max_polls and count>=a.max_polls):
            raise SystemExit(code)
        if code:
            print(json.dumps(dict(event='backup_retry_scheduled',interval_seconds=a.interval_sec)),flush=True)
        time.sleep(a.interval_sec)


if __name__ == '__main__':
    main()
