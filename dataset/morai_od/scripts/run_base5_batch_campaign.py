#!/usr/bin/env python3
"""Bounded shared-ROS pilot with safe single-cell quota recovery; never reuse networks."""
import argparse
import copy
import fcntl
import json
import os
from pathlib import Path
import subprocess
import signal
import sys
import time
import run_base5_capture_campaign as campaign
from base5_capture_safety import atomic_json, load_ledger, prepare_session, sha256, status_row,is_collision_failure,validate_collision_failure


def inspect_batch(root, manifest):
    """Require both boundary and final Park; a safe quality review is not a transport failure."""
    shutdown=json.loads((root/'shutdown_safety.json').read_text())
    if not shutdown.get('park_verified'):raise ValueError('Batch final Park unverified')
    session=json.loads((root/'session_contract.json').read_text())
    repo=Path(__file__).resolve().parents[3]
    rows,_=prepare_session(root,manifest,
        repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
        repo/'configs/sensors/Sensor_competition_local.json',session.get('runner_options'))
    for row in rows:
        if not row.get('between_scene_park',{}).get('park_verified'):
            raise ValueError('Batch boundary Park unverified')
        if is_collision_failure(row):validate_collision_failure(row)
        elif row['status']=='failed':campaign.loop.validate_miss_evidence(row,manifest)
        elif row['status'] not in ('accepted_candidate','review'):
            raise ValueError('Batch capture failure; automatic continuation forbidden')
        reasons=row.get('audit',{}).get('review_reasons',[])
        if row['status']=='accepted_candidate':
            if reasons:raise ValueError('Accepted row still carries review reasons')
            metadata=json.loads((Path(row['capture_root'])/'metadata.json').read_text())
            if metadata.get('calibration_only'):raise ValueError('Calibration cannot fill quota')
    return rows


def stop_child(child,root):
    """Request graceful shutdown first; never orphan a capture on parent Ctrl-C."""
    (root/'STOP').touch()
    try:child.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:os.killpg(child.pid,signal.SIGTERM)
        except ProcessLookupError:child.wait();return
        try:child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            child.wait()


def wait_child(child,root,stop,heartbeat,timeout):
    deadline=time.monotonic()+timeout
    try:
        while child.poll() is None:
            heartbeat(root)
            if stop.exists() or time.monotonic()>deadline:
                raise RuntimeError('STOP/deadline: inspect child safety before starting a new campaign')
            time.sleep(.5)
        return child.returncode
    except BaseException:
        stop_child(child,root)
        raise


def run_child(manifest, root, stop, heartbeat):
    root.mkdir(parents=True,exist_ok=True)
    command=[sys.executable,str(Path(__file__).with_name('run_base5_capture.py')),
        '--manifest',str(manifest),'--output',str(root),'--shared-ros-batch',
        '--limit',str(len(json.loads(manifest.read_text())['scenes']))]
    with (root.parent/'batch.log').open('a') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        return wait_child(child,root,stop,heartbeat,
            100*len(json.loads(manifest.read_text())['scenes']))


def recover(source,index,output,stop,heartbeat=lambda root:None):
    """Use an independently supervised recovery process, including parent Ctrl-C."""
    output.mkdir(parents=True,exist_ok=True)
    command=[sys.executable,str(Path(__file__).with_name('run_base5_capture_campaign.py')),
        '--manifest',str(source),'--indices',str(index),'--output',str(output),'--resume']
    seed=output.parent/(output.name+'_seed.json')
    if seed.exists():command.extend(['--seed-capture',str(seed)])
    with (output.parent/(output.name+'.log')).open('a') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        return wait_child(child,output,stop,heartbeat,800)


def execute(source, indices, output, chunk_size=4, resume=False, stop_after_chunks=0,bulk=False):
    if not indices or len(indices)>(10000 if bulk else 64) or len(set(indices))!=len(indices):
        raise ValueError('Unique 1–64 pilot cells; up to10K only with explicit bulk approval')
    if not 1<=chunk_size<=16:raise ValueError('Chunk must contain 1–16 cells')
    if output.exists() and not resume:raise FileExistsError('Existing campaign requires --resume')
    expected=campaign.contract(source,indices,2)
    expected.update(batch_driver_sha256=sha256(__file__),chunk_size=chunk_size,bulk=bulk)
    output.mkdir(parents=True,exist_ok=True)
    lock=(output/'.batch.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    original=json.loads(source.read_text());selected={};deficits={};started=time.monotonic()
    contract_path=output/'batch_contract.json';stop=output/'STOP'
    if contract_path.exists() and json.loads(contract_path.read_text())!=expected:
        lock.close();raise ValueError('Batch campaign contract mismatch')
    atomic_json(contract_path,expected)
    status_path=output/'status.json'
    prior_status=json.loads(status_path.read_text()) if status_path.exists() else {}
    if resume and prior_status.get('stage')=='halted':
        lock.close();raise ValueError('Halted campaign requires manual safety recovery and new output')
    completed_stages=('complete','complete_with_deficit')
    verify_only=resume and prior_status.get('stage') in completed_stages
    prior_elapsed=prior_status.get('elapsed_sec',0) if resume else 0
    last_checkpoint=prior_status.get('checkpoint_finished',-1) if resume else -1
    def save(stage,error=None,child=None):
        nonlocal last_checkpoint
        # A completed resume is a verification pass. Do not publish its partial
        # reconstruction to concurrent converters/dashboard readers.
        if verify_only and stage=='running':return
        finished=len(selected)+len(deficits)
        snapshot=(not bulk or stage!='running' or
                  (child is None and (last_checkpoint<0 or finished-last_checkpoint>=64)))
        elapsed=prior_elapsed if verify_only else prior_elapsed+time.monotonic()-started
        if snapshot:
            atomic_json(output/'quota.json',dict(target=len(indices),accepted=len(selected),
                deficit=len(deficits),pending=len(indices)-len(selected)-len(deficits),cells=list(deficits.values())))
            atomic_json(output/'deficit_manifest.json',campaign.subset_deck(original,
                [original['scenes'][i] for i in indices if original['scenes'][i]['id'] in deficits]))
        if snapshot and (not verify_only or stage in completed_stages):
            temp=output/'.selected.tmp'
            with temp.open('w') as handle:
                for i in indices:
                    row=selected.get(original['scenes'][i]['id'])
                    if row:handle.write(json.dumps(row,ensure_ascii=False)+'\n')
                handle.flush();os.fsync(handle.fileno())
            temp.replace(output/'selected_ledger.jsonl')
            last_checkpoint=finished
        bins={}
        for row in selected.values():
            key=json.dumps([row.get('family'),row.get('distance_bounds'),row.get('role')])
            bins[key]=bins.get(key,0)+1
        atomic_json(output/'status.json',dict(stage=stage,error=error,requested=len(indices),
            finished=len(selected)+len(deficits),accepted_candidate=len(selected),review=len(deficits),
            failed=int(stage=='halted'),updated_at=time.time(),
            elapsed_sec=elapsed,
            eta_sec=elapsed/finished*(len(indices)-finished) if finished and not verify_only else 0,
            current=str(child) if child else None,
            checkpoint_finished=last_checkpoint,checkpoint_only=bulk,
            accepted_bins=bins if bulk else None,
            results=[] if bulk else [status_row(row) for row in selected.values()]))
    try:
        for ordinal,start in enumerate(range(0,len(indices),chunk_size)):
            chunk_indices=indices[start:start+chunk_size]
            folder=output/f'chunk_{ordinal:04d}';folder.mkdir(exist_ok=True)
            manifest=folder/'manifest.json';root=folder/'capture'
            if not manifest.exists():atomic_json(manifest,campaign.subset_deck(original,[original['scenes'][i] for i in chunk_indices]))
            from base5_motion_ground import moving_actors
            needs_ground=any(moving_actors(json.loads(Path(original['scenes'][i]['metadata']).read_text()))
                for i in chunk_indices if original['scenes'][i].get('metadata'))
            if needs_ground:
                # Animated cells require calibration-first isolated recovery,
                # never a raw unverified constant-Z shared batch.
                for index in chunk_indices:
                    cell=original['scenes'][index];recovery=folder/'recovery'/cell['id']
                    if verify_only and (not (recovery/'status.json').exists() or
                            json.loads((recovery/'status.json').read_text())['stage'] not in completed_stages):
                        raise ValueError('Completed ground preparation evidence missing; no recapture')
                    if stop.exists():save('stopped');return 0
                    if recover(source,index,recovery,stop,lambda child:save('running',child=child)):raise RuntimeError('Ground-prepared isolated capture halted; no next load')
                    accepted=load_ledger(recovery/'selected_ledger.jsonl')
                    if accepted:selected[cell['id']]=accepted[0]
                    else:deficits[cell['id']]=dict(id=cell['id'],index=index,reasons=json.loads((recovery/'quota.json').read_text())['cells'])
                    save('running')
                if (output/'PAUSE').exists() or (stop_after_chunks and ordinal+1>=stop_after_chunks):
                    save('paused');return 0
                continue
            while True:
                # No unsafe child may be resumed, even if the final Park later succeeds.
                rows=inspect_batch(root,manifest) if (root/'session_contract.json').exists() else []
                by_id={r['id']:r for r in rows}
                for index in chunk_indices:
                    cell=original['scenes'][index];row=by_id.get(cell['id'])
                    if not row:continue
                    if is_collision_failure(row):
                        validate_collision_failure(row)
                        deficits[cell['id']]=dict(id=cell['id'],index=index,reasons=['physical_collision'],evidence=row)
                    elif row['status']=='accepted_candidate':
                        chosen=copy.deepcopy(row);chosen['source_cell_id']=cell['id'];selected[cell['id']]=chosen
                    else:
                        recovery=folder/'recovery'/cell['id']
                        if verify_only and (not (recovery/'status.json').exists() or
                                json.loads((recovery/'status.json').read_text())['stage'] not in completed_stages):
                            raise ValueError('Completed recovery evidence missing; verification cannot recapture')
                        if stop.exists():save('stopped');return 0
                        seed=recovery.parent/(recovery.name+'_seed.json')
                        seed.parent.mkdir(parents=True,exist_ok=True)
                        if not seed.exists():atomic_json(seed,dict(capture_root=str(root.resolve()),manifest=str(manifest.resolve()),id=cell['id']))
                        code=recover(source,index,recovery,stop,lambda child:save('running',child=child))
                        if code:raise RuntimeError('Single-cell recovery halted; no next load')
                        accepted=load_ledger(recovery/'selected_ledger.jsonl')
                        if stop.exists():save('stopped');return 0
                        if accepted:selected[cell['id']]=accepted[0]
                        else:deficits[cell['id']]=dict(id=cell['id'],index=index,reasons=json.loads((recovery/'quota.json').read_text())['cells'])
                save('running')
                if any(r['status']=='failed' or is_collision_failure(r) for r in rows):
                    # Do not resume the failed six-second source batch: its raw
                    # session correctly does not mark failed rows as completed.
                    # Finish the remaining cells through isolated bounded loops.
                    for index in chunk_indices:
                        cell=original['scenes'][index]
                        if cell['id'] in by_id:continue
                        if stop.exists():save('stopped');return 0
                        recovery=folder/'recovery'/cell['id']
                        if verify_only and (not (recovery/'status.json').exists() or
                                json.loads((recovery/'status.json').read_text())['stage'] not in completed_stages):
                            raise ValueError('Completed tail evidence missing; no recapture')
                        code=recover(source,index,recovery,stop,lambda child:save('running',child=child))
                        if code:raise RuntimeError('Post-miss isolated capture halted; no next load')
                        accepted=load_ledger(recovery/'selected_ledger.jsonl')
                        if accepted:selected[cell['id']]=accepted[0]
                        else:deficits[cell['id']]=dict(id=cell['id'],index=index,reasons=json.loads((recovery/'quota.json').read_text())['cells'])
                        save('running')
                    break
                if len(rows)==len(chunk_indices):break
                if stop.exists():save('stopped');return 0
                before=len(rows)
                if verify_only:raise ValueError('Completed batch evidence missing; verification cannot recapture')
                code=run_child(manifest,root,stop,lambda child:save('running',child=child))
                after=inspect_batch(root,manifest)
                if len(after)<=before:raise RuntimeError('Batch made no progress')
                if code and after[-1]['status'] not in ('review','failed'):
                    raise RuntimeError('Batch failed outside a verified review/target-miss boundary')
                # Reviewed raw is preserved in its original safe session; recovery
                # verifies that evidence rather than repeating the initial capture.
            if (output/'PAUSE').exists() or (stop_after_chunks and ordinal+1>=stop_after_chunks):
                save('paused');return 0
        save('complete_with_deficit' if deficits else 'complete');return 0
    except KeyboardInterrupt:
        save('halted','Parent interrupted; child STOP issued; inspect Park before recovery');return 130
    except Exception as exc:
        save('halted',str(exc));return 1
    finally:lock.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--limit',type=int,default=32);parser.add_argument('--indices')
    parser.add_argument('--chunk-size',type=int,default=4);parser.add_argument('--resume',action='store_true')
    parser.add_argument('--stop-after-chunks',type=int,default=0)
    parser.add_argument('--approve-bulk-capture',action='store_true',help='Explicit authorization above64 cells; child batches remain bounded')
    args=parser.parse_args();deck=json.loads(args.manifest.read_text())
    indices=[int(i) for i in args.indices.split(',')] if args.indices else list(range(min(args.limit,len(deck['scenes']))))
    raise SystemExit(execute(args.manifest,indices,args.output,args.chunk_size,args.resume,args.stop_after_chunks,args.approve_bulk_capture))
