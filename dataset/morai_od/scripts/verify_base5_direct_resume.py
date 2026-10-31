#!/usr/bin/env python3
"""Prove an already-complete optimized capture resumes without new MR work.

Incomplete sessions are refused before launching a child. The child runs the
same optimized collector arguments behind ROS-import/socket denial guards.
Only status.updated_at may change; native RAW, ledger, EventCmd, timing and
source/runtime/session pins must remain byte/inode/mtime/size identical.
"""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys

from base5_capture_safety import prepare_session,sha256,status_row

REPO=Path(__file__).resolve().parents[3]
SCRIPT=Path(__file__).with_name('run_base5_capture_optimized.py')
SOURCE=SCRIPT.with_name('run_base5_capture.py')
SENSOR=REPO/'configs/sensors/Sensor_competition_local.json'
NETWORK=REPO/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json'
GUARD_MARKER='BASE5_DIRECT_RESUME_OFFLINE_GUARD '
GUARD=r'''
import builtins,json,runpy,sys
denied={'ros_imports':0,'sockets':0}
original_import=builtins.__import__
def guarded_import(name,*args,**kwargs):
    if name.split('.')[0] in ('rospy','morai_msgs'):
        denied['ros_imports']+=1
        raise RuntimeError('Completed-resume verifier forbids live ROS imports')
    return original_import(name,*args,**kwargs)
def audit(event,args):
    if event=='socket.__new__':
        denied['sockets']+=1
        raise RuntimeError('Completed-resume verifier forbids network sockets')
builtins.__import__=guarded_import
sys.addaudithook(audit)
script=sys.argv[1];sys.argv=sys.argv[1:]
sys.path.insert(0,str(__import__('pathlib').Path(script).parent))
try:runpy.run_path(script,run_name='__main__')
finally:print('BASE5_DIRECT_RESUME_OFFLINE_GUARD '+json.dumps(denied,sort_keys=True),flush=True)
'''


def read(path):return json.loads(Path(path).read_text())


def stamp(path):
    path=Path(path)
    if path.is_symlink() or not path.is_file():raise ValueError('Regular immutable source file required: '+str(path))
    a=path.stat();checksum=sha256(path);b=path.stat()
    state=lambda value:(value.st_dev,value.st_ino,value.st_size,value.st_mtime_ns,value.st_ctime_ns)
    if state(a)!=state(b):raise ValueError('Source changed during snapshot: '+str(path))
    return dict(sha256=checksum,bytes=b.st_size,inode=b.st_ino,device=b.st_dev,
                mtime_ns=b.st_mtime_ns,ctime_ns=b.st_ctime_ns)


def snapshot(output,sources):
    output=Path(output)
    paths=set(Path(p).resolve() for p in sources)
    for path in output.rglob('*'):
        if path.is_symlink():raise ValueError('Capture output contains a symlink')
        if path.is_file() and path!=output/'status.json':paths.add(path.resolve())
    return {str(p):stamp(p) for p in sorted(paths)}


def state(manifest,output,limit,profile,network):
    """Pure admission checks; never repair a torn ledger or create a contract."""
    if type(limit) is not int or not 1<=limit<=64:raise ValueError('Finite direct pilot limit 1 to 64 required')
    if profile not in ('baseline','static-ready'):raise ValueError('Unknown optimized setup profile')
    manifest,output,network=Path(manifest).resolve(strict=True),Path(output).resolve(strict=True),Path(network).resolve(strict=True)
    for name in ('ledger.jsonl','session_contract.json','status.json','shutdown_safety.json',
                 'event_cmd_requests.jsonl','timing.json','.runner.lock'):
        if not (output/name).is_file():raise ValueError('Completed direct capture lacks '+name)
    status=read(output/'status.json')
    if status.get('stage')!='complete' or status.get('current') is not None or status.get('error'):
        raise ValueError('Only an already-complete direct session may be verified')
    if read(output/'shutdown_safety.json').get('park_verified') is not True:
        raise ValueError('Completed direct session lacks final verified Park')
    data=(output/'ledger.jsonl').read_bytes()
    if not data or not data.endswith(b'\n'):raise ValueError('Refuse missing/torn/unterminated completion ledger')
    rows=[json.loads(line) for line in data.splitlines()]
    latest={row['id']:row for row in rows}
    deck=read(manifest);selected=deck['scenes'][:limit]
    ids=[entry['id'] for entry in selected]
    if not ids or len(set(ids))!=len(ids) or set(latest)!=set(ids):
        raise ValueError('Completed ledger and selected manifest IDs differ')
    if any(row.get('status') not in ('accepted_candidate','review') for row in latest.values()):
        raise ValueError('Pending/failed direct rows must not enter a verifier child')
    accepted=sum(row['status']=='accepted_candidate' for row in latest.values())
    if (status.get('requested'),status.get('finished'),status.get('accepted_candidate'),
            status.get('review'),status.get('failed'))!=(len(ids),len(ids),accepted,len(ids)-accepted,0):
        raise ValueError('Completed status counts disagree with actual latest ledger')
    expected_results={row['id']:status_row(row) for row in latest.values()}
    actual_results={row['id']:row for row in status.get('results',[])}
    if len(actual_results)!=len(status.get('results',[])) or actual_results!=expected_results:
        raise ValueError('Completed status results differ from actual latest ledger')
    sources={manifest,network,SENSOR,SCRIPT,SOURCE}
    for entry in selected:
        row=latest[entry['id']]
        scene,meta=Path(entry['scenario']).resolve(strict=True),Path(entry['metadata']).resolve(strict=True)
        capture=Path(row['capture_root']).resolve(strict=True)
        capture.relative_to(output)
        if (sha256(scene)!=entry['scenario_sha256'] or row.get('scenario_sha256')!=entry['scenario_sha256']
                or sha256(capture/'scenario.json')!=entry['scenario_sha256']):
            raise ValueError('Selected source/captured scenario checksum differs')
        sources.update((scene,meta))
    contract=read(output/'session_contract.json')
    for name in contract['safety_code_sha256']:sources.add(SCRIPT.with_name(name))
    from generate_dynamic_object_capture_set import DEFAULT_MGEO
    for name in contract.get('ego_support_map_sha256',{}):sources.add(DEFAULT_MGEO/name)
    # Prove another collector is not currently appending to this same output.
    with (output/'.runner.lock').open('r+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        fcntl.flock(lock,fcntl.LOCK_UN)
    options=dict(shared_ros_batch=False,selected_scene_ids=ids,optimized_adapter=dict(profile=profile,
        adapter_sha256=sha256(SCRIPT),source_sha256=sha256(SOURCE),
        policy='static conditional setup delay only; native gates'))
    return dict(manifest=manifest,output=output,network=network,status=status,sources=sources,
                rows=rows,latest=latest,runner_options=options)


def verify(manifest,output,receipt,limit,profile='static-ready',network=NETWORK,timeout=120.,run=None):
    receipt=Path(receipt).resolve()
    if receipt.exists():raise FileExistsError('Use a fresh verification receipt directory')
    before_state=state(manifest,output,limit,profile,network)
    output=before_state['output']
    if receipt==output or output in receipt.parents:raise ValueError('Receipt must remain outside immutable capture output')
    before=snapshot(output,before_state['sources'])
    # Existing contracts only: prepare_session is called after strict newline /
    # completion checks, so its historical torn-ledger repair cannot run.
    _,completed=prepare_session(output,before_state['manifest'],before_state['network'],SENSOR,
                                before_state['runner_options'])
    if set(completed)!=set(before_state['latest']):raise ValueError('Native session verifier disagrees with completed ledger')
    if before!=snapshot(output,before_state['sources']):raise ValueError('Native preflight modified protected files')
    logical=[str(SCRIPT),'--manifest',str(before_state['manifest']),'--output',str(output),
             '--limit',str(limit),'--setup-profile',profile,'--network-template',str(before_state['network'])]
    command=[sys.executable,'-c',GUARD]+logical
    receipt.mkdir(parents=True)
    (receipt/'before.json').write_text(json.dumps(before,indent=2)+'\n')
    result=(run or subprocess.run)(command,cwd=str(REPO),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                  text=True,timeout=float(timeout),check=False)
    (receipt/'child.log').write_text(result.stdout)
    after=snapshot(output,before_state['sources'])
    (receipt/'after.json').write_text(json.dumps(after,indent=2)+'\n')
    errors=[]
    if result.returncode:errors.append('Completed collector child exited nonzero')
    if before!=after:errors.append('Protected RAW/ledger/EventCmd/timing/source/runtime files changed')
    guards=[json.loads(line[len(GUARD_MARKER):]) for line in result.stdout.splitlines() if line.startswith(GUARD_MARKER)]
    if guards!=[dict(ros_imports=0,sockets=0)]:errors.append('Offline child guard missing or a live path was attempted')
    try:
        after_state=state(manifest,output,limit,profile,network)
        plain=lambda value:{key:item for key,item in value.items() if key!='updated_at'}
        if plain(after_state['status'])!=plain(before_state['status']):errors.append('Unexpected status fields changed')
    except (ValueError,KeyError,OSError) as error:errors.append('Post-resume completion differs: '+str(error))
    result_report=dict(policy='direct-complete-resume-read-only-v155',passed=not errors,errors=errors,
        actual_new_captures=0 if before==after else None,selected=len(before_state['latest']),
        logical_optimized_command=[sys.executable]+logical,guarded_child_command=command,
        child_returncode=result.returncode,protected_files=len(before),status_only_allowed_change='updated_at',
        native_control='ROS imports and sockets denied in child',guard_results=guards)
    (receipt/'report.json').write_text(json.dumps(result_report,indent=2)+'\n')
    if errors:raise ValueError('; '.join(errors))
    return result_report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--receipt',type=Path,required=True)
    parser.add_argument('--limit',type=int,required=True)
    parser.add_argument('--setup-profile',choices=('baseline','static-ready'),default='static-ready')
    parser.add_argument('--network-template',type=Path,default=NETWORK)
    parser.add_argument('--timeout-sec',type=float,default=120.)
    args=parser.parse_args()
    if not 1<=args.timeout_sec<=600:parser.error('Finite timeout 1 to 600 seconds required')
    result=verify(args.manifest,args.output,args.receipt,args.limit,args.setup_profile,args.network_template,args.timeout_sec)
    print(json.dumps({key:result[key] for key in ('passed','actual_new_captures','selected','protected_files')}))


if __name__=='__main__':main()
