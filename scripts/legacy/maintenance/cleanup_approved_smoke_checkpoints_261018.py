#!/usr/bin/env python3
"""Delete only the twelve explicitly approved short-resume test checkpoints.

Default is read-only. Production/model initialization weights, native captures,
reports, configurations, contracts and logs are never selected by this helper.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import time


ROOT = Path(__file__).absolute().parents[2]
EXPECTED_ROOT = Path('/home/luv/AjouNice2026')
REPORT = ROOT/'artifacts/cleanup/approved_storage_261018/smoke_checkpoints/report.json'
BASE = ROOT/'artifacts/evaluation/bevfusion'
TARGETS = tuple(
    BASE/'base5_training_runtime_261011'/profile/'training'/f'epoch_{epoch}.pth'
    for profile in ('compiled_runtime_resume_smoke', 'runtime_resume_smoke')
    for epoch in (1, 2)
) + tuple(
    BASE/profile/'gpu_resume_smoke/training'/f'epoch_{epoch}.pth'
    for profile in ('base5_upright_targets_261011', 'base5_heatmap_targets_261011',
                    'base5_input_pipeline_261011')
    for epoch in (1, 2)
) + tuple(
    ROOT/'artifacts/detection3d/bevfusion/base5_direct_resume_smoke_261010_v2/training'/f'epoch_{epoch}.pth'
    for epoch in (1, 2)
)
IMAGE = 'python:3.10-slim-bookworm'


def require(value, message):
    if not value:
        raise RuntimeError(message)


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def checked_path(path):
    path = Path(path).absolute()
    require('..' not in path.parts, 'Parent traversal is forbidden')
    for ancestor in (path, *path.parents):
        require(not ancestor.is_symlink(), 'Symlink in cleanup scope: '+str(ancestor))
        if ancestor == ROOT:
            break
        if ancestor.exists():
            require(not os.path.ismount(ancestor), 'Mount boundary in cleanup scope: '+str(ancestor))
    return path


def signature(path):
    path = checked_path(path)
    value = path.lstat()
    require(stat.S_ISREG(value.st_mode) and value.st_uid == value.st_gid == 0,
            'Expected the original Docker-root-owned regular checkpoint: '+str(path))
    require(value.st_dev == ROOT.stat().st_dev and value.st_nlink == 1,
            'Checkpoint device/link count differs from approved unique files: '+str(path))
    return dict(device=value.st_dev, inode=value.st_ino, size=value.st_size,
                blocks=value.st_blocks, links=value.st_nlink, mtime_ns=value.st_mtime_ns,
                ctime_ns=value.st_ctime_ns, uid=value.st_uid)


def release_paths():
    paths = sorted((ROOT/'artifacts/evaluation').rglob('release.json'))
    required = [ROOT/'artifacts/evaluation/base5_unit_release_261017/release_v158a/release.json',
        ROOT/'artifacts/evaluation/base5_sunny15_261017/finalization_v1/release_v159_sunny15/release.json']
    require(all(path in paths for path in required), 'Both current frozen releases must be present')
    return paths


def verify_release_pins():
    """Inspect all nested path strings/keys, not only top-level runtime pins."""
    pins = []
    target_text = {str(path) for path in TARGETS}
    target_parents = {str(path.parent) for path in TARGETS}
    for path in release_paths():
        checked_path(path)
        document = json.loads(path.read_text())
        stack = [document]
        while stack:
            value = stack.pop()
            if isinstance(value, dict):
                stack.extend(value.keys())
                stack.extend(value.values())
            elif isinstance(value, list):
                stack.extend(value)
            elif isinstance(value, str):
                require(value not in target_text and value not in target_parents,
                        'Frozen release still refers to approved checkpoint: '+value)
        pins.append(dict(path=str(path), sha256=sha(path)))
    return pins


def verify_idle(signatures):
    """Check all visible same-user process argv and open inode references."""
    wanted = {(row['device'], row['inode']) for row in signatures.values()}
    text = {str(path) for path in TARGETS}
    parents = {str(path.parent) for path in TARGETS}
    inspected, skipped_other_uids, inaccessible_login_helpers = 0, 0, []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            if process.stat().st_uid != os.getuid():
                skipped_other_uids += 1
                continue
            argv = [os.fsdecode(item) for item in (process/'cmdline').read_bytes().split(b'\0') if item]
            values = {arg.split('=', 1)[-1] for arg in argv}
            require(not (values & (text | parents)), 'A process names the checkpoint/training directory: '+process.name)
            try:
                descriptors = list((process/'fd').iterdir())
            except PermissionError:
                name = (process/'comm').read_text().strip()
                # Observed non-dumpable desktop/security daemons, not model
                # workers. Keep this limitation explicit alongside lsof and
                # refuse every unknown or Python/ROS/training exception.
                known = (name == '(sd-pam)' and argv == ['(sd-pam)']) or bool(argv and
                    argv[0] in ('/usr/bin/gnome-keyring-daemon','/usr/bin/ssh-agent','/usr/bin/sunshine'))
                require(known, 'Cannot inspect potentially active process '+process.name)
                inaccessible_login_helpers.append(dict(pid=int(process.name),comm=name,argv=argv))
                continue
            for descriptor in descriptors:
                try:
                    entry = descriptor.stat()
                except (FileNotFoundError, ProcessLookupError):
                    continue
                require((entry.st_dev, entry.st_ino) not in wanted,
                        'An approved checkpoint is open in process '+process.name)
            inspected += 1
        except (FileNotFoundError, ProcessLookupError):
            continue
    opened = subprocess.run(['lsof', '-nP', '-F', 'pfn', '--', *map(str,TARGETS)],
        text=True, capture_output=True, check=False)
    require(opened.returncode == 1 and not opened.stdout.strip(), 'lsof found an open checkpoint or failed')
    containers = subprocess.run(['docker','ps','--quiet'],text=True,capture_output=True,check=True)
    require(not containers.stdout.strip(), 'A Docker container is active; do not assume root-owned checkpoints idle')
    return dict(same_user_processes_inspected=inspected, other_uid_processes_not_inspected=skipped_other_uids,
        same_user_open_inode_references=0, inaccessible_login_helpers=inaccessible_login_helpers,
        lsof_returncode=opened.returncode,
        lsof_stderr=opened.stderr, running_docker_containers=0, process_signals_sent=0)


CONTAINER_CODE = r'''
import hashlib,json,os,stat,sys
from pathlib import Path
payload=json.load(sys.stdin)
rows=payload['files']
def require(value,message):
    if not value:raise RuntimeError(message)
def check(row):
    path=Path(row['container_path'])
    require(path.parent in {Path('/approved'+str(i)) for i in range(6)}
        and path.name in ('epoch_1.pth','epoch_2.pth'), 'Exact approved mount/name required')
    require(not path.is_symlink() and not path.parent.is_symlink(), 'Symlink checkpoint/mount')
    value=path.lstat()
    actual=dict(device=value.st_dev,inode=value.st_ino,size=value.st_size,
        blocks=value.st_blocks,links=value.st_nlink,mtime_ns=value.st_mtime_ns,
        ctime_ns=value.st_ctime_ns,uid=value.st_uid)
    require(stat.S_ISREG(value.st_mode) and value.st_uid==value.st_gid==0
        and actual==row['original_stat'],'Checkpoint identity changed inside scoped mount')
    digest=hashlib.sha256()
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    with os.fdopen(fd,'rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    require(digest.hexdigest()==row['original_sha256'],'Checkpoint SHA changed inside scoped mount')
    require(path.lstat()==value,'Checkpoint changed while hashing')
    return path
removed=[]
try:
    require(len(rows)==12 and len({r['container_path'] for r in rows})==12,'Exactly twelve files required')
    require(all(sum(Path(r['container_path']).parent==Path('/approved'+str(i)) for r in rows)==2
        for i in range(6)),'Exactly two checkpoint names per approved mount required')
    for row in rows:check(row)
    for row in rows:
        path=check(row)
        path.unlink()
        require(not path.exists(),'Unlink did not remove checkpoint')
        removed.append(row['path'])
        print(json.dumps(dict(event='approved_checkpoint_removed',path=row['path'])),flush=True)
    print(json.dumps(dict(event='complete',removed=removed)),flush=True)
except BaseException as error:
    print(json.dumps(dict(event='error',removed=removed,error_type=type(error).__name__,error=str(error))),flush=True)
    raise SystemExit(1)
'''


def delete_scoped(rows):
    """No GPU/network or broad writable tree; only six exact training folders."""
    image = subprocess.run(['docker','image','inspect','--format','{{.Id}}',IMAGE],
        text=True,capture_output=True,check=True).stdout.strip()
    require(image.startswith('sha256:') and len(image) == 71, 'Existing cached image identity required')
    parents = list(dict.fromkeys(Path(row['path']).parent for row in rows))
    require(len(parents) == 6, 'Exactly six approved training folder mounts required')
    command = ['docker','run','--rm','--pull=never','--runtime=runc','--network=none',
        '--read-only','--cap-drop=ALL','--security-opt=no-new-privileges:true','--user=0:0',
        '--pids-limit=32','--memory=256m','--cpus=1','--log-driver=none','-i']
    for index,parent in enumerate(parents):
        checked_path(parent)
        require(parent.stat().st_uid == parent.stat().st_gid == 0, 'Original training directory ownership changed')
        command.extend(['--mount','type=bind,src='+str(parent)+',dst=/approved'+str(index)])
    command.extend([image,'python','-B','-c',CONTAINER_CODE])
    payload = dict(files=[dict(row,container_path='/approved'+str(parents.index(Path(row['path']).parent))
        +'/'+Path(row['path']).name) for row in rows])
    result = subprocess.run(command,input=json.dumps(payload),text=True,capture_output=True,check=False)
    events = [json.loads(line) for line in result.stdout.splitlines() if line.strip()]
    return dict(image=IMAGE,image_id=image,returncode=result.returncode,events=events,
        stderr=result.stderr,writable_mounts=[str(parent) for parent in parents],
        host_repository_writable=False,host_home_writable=False,gpu_devices_mounted=False,
        network='none',capabilities='ALL dropped',privilege_escalation=False)


def write_report(document):
    checked_path(REPORT)
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    checked_path(REPORT.parent)
    require(REPORT.parent.stat().st_uid == os.getuid(), 'Cleanup report directory owner differs')
    temporary = REPORT.with_name('.report.'+str(os.getpid())+'.json')
    with temporary.open('x') as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, REPORT)


def run(execute=False):
    require(ROOT == EXPECTED_ROOT, 'This approval only applies to the original local repository')
    require(len(TARGETS) == len(set(TARGETS)) == 12, 'Exactly twelve explicit checkpoints required')
    require(not REPORT.exists(), 'Existing cleanup evidence must not be overwritten')
    release_pins = verify_release_pins()
    signatures = {path:signature(path) for path in TARGETS}
    rows = [dict(path=str(path), original_stat=signatures[path], original_sha256=sha(path),
                 allocated_bytes=signatures[path]['blocks']*512, deleted=False) for path in TARGETS]
    require(all(signature(path) == signatures[path] for path in TARGETS), 'Checkpoint changed during SHA calculation')
    process_proof = verify_idle(signatures)
    document = dict(schema='base5_explicit_12_smoke_checkpoint_cleanup_v1',
        created_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        script=str(Path(__file__).absolute()), script_sha256=sha(__file__),
        authorization='User explicitly approved short training/resume test checkpoint removal; exact twelve files only',
        executed=execute, status='VALIDATED' if execute else 'READ_ONLY_PLAN',
        selected_count=12, selected_original_bytes=sum(row['original_stat']['size'] for row in rows),
        expected_reclaimed_bytes=sum(row['allocated_bytes'] for row in rows),
        releases_rechecked=release_pins, frozen_release_checkpoint_references=0,
        process_proof=process_proof, deleted_count=0, files=rows,
        reports_configs_contracts_logs_preserved=True,
        production_e9_e24_e25_and_initializers_selected=False,
        native_data_selected=False, remote_access=False, gpu_ros_morai_operations=False)
    if not execute:
        return document
    write_report(document)
    try:
        require(all(sha(pin['path']) == pin['sha256'] for pin in release_pins),
                'Frozen release changed before scoped deletion')
        require(all(signature(path) == signatures[path] for path in TARGETS),
                'Approved checkpoint identity changed before scoped deletion')
        document['immediate_process_proof'] = verify_idle(signatures)
        document['scoped_cpu_container'] = delete_scoped(rows)
        removed = {event['path'] for event in document['scoped_cpu_container']['events']
            if event.get('event') == 'approved_checkpoint_removed'}
        for row in rows:
            path = Path(row['path'])
            row['deleted'] = row['path'] in removed and not path.exists() and not path.is_symlink()
            if row['deleted']:
                row['deleted_at_unix'] = time.time()
        document['deleted_count'] = sum(row['deleted'] for row in rows)
        write_report(document)
        require(document['scoped_cpu_container']['returncode'] == 0 and len(removed) == 12
            and document['deleted_count'] == 12, 'Scoped cleanup did not finish all exact approved files')
        require(all(not path.exists() for path in TARGETS), 'Not all exact approved checkpoints were removed')
        require(all(sha(pin['path']) == pin['sha256'] for pin in release_pins),
                'Frozen release changed after cleanup')
        document.update(status='COMPLETE', removed_allocated_bytes=sum(row['allocated_bytes'] for row in rows),
            completed_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    except BaseException as error:
        document.update(status='STOPPED_ERROR', error_type=type(error).__name__, error=str(error))
        write_report(document)
        raise
    write_report(document)
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true')
    arguments = parser.parse_args()
    result = run(arguments.run)
    print(json.dumps(dict(status=result['status'], selected_count=result['selected_count'],
        deleted_count=result['deleted_count'], expected_reclaimed_bytes=result['expected_reclaimed_bytes'],
        report=str(REPORT) if arguments.run else None), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
