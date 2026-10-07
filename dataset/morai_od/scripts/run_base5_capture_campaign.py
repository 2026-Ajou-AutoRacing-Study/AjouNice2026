#!/usr/bin/env python3
"""Durable quota campaign around bounded Base5 quality recovery (no gate waiver)."""
import argparse
import copy
import fcntl
import json
import os
from pathlib import Path
import time
import run_base5_quality_loop as loop
from base5_capture_safety import atomic_json, sha256, prepare_session


def subset_deck(original, scenes):
    """Copy only retained rows, not the full 10K scene array for each cell."""
    deck=copy.deepcopy({k:v for k,v in original.items() if k!='scenes'})
    deck.update(scenes=copy.deepcopy(scenes),count=len(scenes))
    return deck


def contract(source, indices, rounds):
    data=loop.gen.geo.load_json(source)
    scripts=Path(__file__).parent
    names=['run_base5_capture_campaign.py','run_base5_quality_loop.py',
           'generate_base5_independent.py','prepare_base5_ground_retry.py',
           'prepare_base5_empty_ground_probe.py','prepare_base5_probed_retry.py',
           'run_base5_capture.py','base5_capture_safety.py','base5_capture_audit.py',
           'base5_ground_support.py','base5_ground_probe.py','base5_capture_timing.py','base5_event_cmd.py']
    names.extend(['prepare_base5_setup_budget_pilot.py','base5_motion_ground.py','base5_static_height.py',
        'base5_static_ground_reference.py','prepare_base5_static_height_reference.py','base5_population_budget.py'])
    sources=[]
    for i in indices:
        row=data['scenes'][i]
        if sha256(Path(row['scenario']))!=row['scenario_sha256']:
            raise ValueError('Source scenario checksum mismatch')
        sources.append([row['id'],row['scenario_sha256'],sha256(Path(row['metadata']))])
    repo=scripts.parents[2]
    return dict(schema=1,manifest=sha256(source),indices=indices,rounds=rounds,
                network=sha256(repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json'),
                sensor=sha256(repo/'configs/sensors/Sensor_competition_local.json'),
                source_files=sources,code={n:sha256(scripts/n) for n in names})


def validate_selected(report):
    """Re-run the existing raw/provenance/session integrity validator, offline."""
    for row in report['selected']:
        if row['status']!='accepted_candidate' or row.get('audit',{}).get('review_reasons'):
            raise ValueError('Reviewed capture cannot fill quota')
        if loop.gen.geo.load_json(Path(row['capture_root'])/'metadata.json').get('calibration_only'):
            raise ValueError('Calibration cannot fill quota')
        root=Path(row['capture_root'])
        if loop.gen.geo.load_json(root/'metadata.json').get('static_ground_reaudit'):
            from base5_static_ground_reference import validate_reaudit_output
            verified=validate_reaudit_output(root)
            if verified['audit']!=row['audit']:raise RuntimeError('Selected re-audit row changed')
    for attempt in report['attempts']:
        if 'capture_root' not in attempt:continue
        root=Path(attempt['capture_root'])
        if attempt['kind']=='static_rigid_raw_reaudit':
            # Hardlinked QA views have no ROS session/independent shutdown file.
            # Their source and nearby capture attempts are validated separately.
            from base5_static_ground_reference import validate_reaudit_output
            validate_reaudit_output(root)
            continue
        safety=loop.gen.geo.load_json(root/'shutdown_safety.json')
        if not safety.get('park_verified'):raise ValueError('Unsafe child cannot be resumed')
        session=loop.gen.geo.load_json(root/'session_contract.json')
        repo=Path(__file__).resolve().parents[3]
        network=repo/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json'
        sensor=repo/'configs/sensors/Sensor_competition_local.json'
        # Child manifest is retained beside its capture directory.
        candidates=list(root.parent.rglob('manifest.json'))
        manifest=next((p for p in candidates if sha256(p)==session['manifest_sha256']),None)
        if manifest is None:raise ValueError('Child manifest missing')
        prepare_session(root,manifest,network,sensor,session.get('runner_options'))


def execute(source, indices, output, resume=False, rounds=2, stop_after=0, bulk=False,seed_capture=None):
    if not indices or len(indices)!=len(set(indices)):raise ValueError('Unique nonempty indices required')
    if len(indices)>64 and not bulk:raise ValueError('More than 64 cells requires --approve-bulk-capture')
    if not 1<=rounds<=3:raise ValueError('Quota rounds must be 1–3')
    if seed_capture is not None and len(indices)!=1:raise ValueError('Seed campaign must be single-cell')
    if output.exists() and not resume:raise FileExistsError('Existing campaign requires --resume')
    output.mkdir(parents=True,exist_ok=True)
    lock=(output/'.campaign.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    expected=contract(source,indices,rounds);cp=output/'campaign_contract.json'
    if seed_capture is not None:expected['seed_sha256']=sha256(seed_capture)
    if cp.exists() and loop.gen.geo.load_json(cp)!=expected:
        lock.close();raise ValueError('Campaign resume contract mismatch; use new output')
    atomic_json(cp,expected)
    original=loop.gen.geo.load_json(source);selected=[];deficits=[];finished=0;start=time.time()
    status_path=output/'status.json'
    prior_status=loop.gen.geo.load_json(status_path) if status_path.exists() else {}
    completed_stages=('complete','complete_with_deficit')
    verify_only=resume and prior_status.get('stage') in completed_stages
    def progress(cell,kind,root):
        atomic_json(output/'status.json',dict(stage='running',current=cell+' / '+kind,
            requested=len(indices),finished=finished,accepted_candidate=len(selected),review=len(deficits),
            failed=0,updated_at=time.time(),elapsed_sec=time.time()-start,
            eta_sec=((time.time()-start)/finished*(len(indices)-finished)) if finished else 0,
            results=[dict(family=r.get('family'),role=r.get('role'),distance_bounds=r.get('distance_bounds'),status='accepted_candidate') for r in selected],
            child_status=str(root/'status.json')))
    def save(stage,error=None):
        if verify_only and stage=='running':return
        # Cell reports are authoritative. Canonical ledger is rebuilt on resume,
        # with one selection per original quota cell, never calibration frames.
        if not verify_only or stage in completed_stages:
            temp=output/'.selected.tmp'
            with temp.open('w') as f:
                for row in selected:f.write(json.dumps(row,ensure_ascii=False)+'\n')
                f.flush();os.fsync(f.fileno())
            temp.replace(output/'selected_ledger.jsonl')
        atomic_json(output/'quota.json',dict(target=len(indices),accepted=len(selected),
                    deficit=len(deficits),pending=len(indices)-finished,cells=deficits))
        deck=subset_deck(original,[original['scenes'][d['index']] for d in deficits])
        atomic_json(output/'deficit_manifest.json',deck)
        atomic_json(output/'status.json',dict(stage=stage,error=error,requested=len(indices),
                    finished=finished,accepted_candidate=len(selected),review=len(deficits),
                    failed=int(stage=='halted'),
                    elapsed_sec=prior_status['elapsed_sec'] if verify_only else time.time()-start,
                    updated_at=time.time(),
                    eta_sec=0,results=[]))
    try:
        save('running')
        for index in indices:
            if (output/'STOP').exists():save('stopped');return 0
            row=original['scenes'][index];cell=output/row['id'];cell.mkdir(exist_ok=True)
            deck=subset_deck(original,[row])
            manifest=cell/'manifest.json'
            if not manifest.exists():atomic_json(manifest,deck)
            outcome=None
            for ordinal in range(rounds):
                target=cell/f'round_{ordinal}';report=target/'report.json'
                if ordinal:
                    replacement=cell/f'replacement_{ordinal}'/'manifest.json'
                    if not replacement.exists():loop.replacement(manifest,replacement.parent)
                    manifest=replacement
                if report.exists() and loop.gen.geo.load_json(report)['stage']=='complete':
                    outcome=loop.gen.geo.load_json(report);validate_selected(outcome)
                else:
                    if verify_only:raise ValueError('Completed cell evidence missing; verification cannot recapture')
                    # STOP is also watched within the running bounded loop.
                    options=dict(resume=target.exists(),stop_path=output/'STOP',progress=progress)
                    if ordinal==0 and seed_capture is not None:options['seed_capture']=seed_capture
                    code=loop.execute(manifest,[0],target,**options)
                    outcome=loop.gen.geo.load_json(report)
                    if code:raise RuntimeError('Quality child halted; no next cell: '+str(target))
                    validate_selected(outcome)
                if outcome['selected']:
                    accepted=copy.deepcopy(outcome['selected'][0]);accepted['source_cell_id']=row['id']
                    selected.append(accepted);break
                if (output/'STOP').exists():save('stopped');return 0
            else:
                deficits.append(dict(index=index,id=row['id'],family=row.get('family'),
                    role=row.get('role'),distance_bounds=row.get('distance_bounds'),
                    subfamily=row.get('subfamily'),reasons=outcome['unresolved']))
            finished+=1;save('running')
            if stop_after and finished>=stop_after:save('paused');return 0
        save('complete_with_deficit' if deficits else 'complete');return 0
    except Exception as exc:
        save('halted',str(exc));return 1
    finally:lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--indices');p.add_argument('--limit',type=int,default=64)
    p.add_argument('--resume',action='store_true');p.add_argument('--approve-bulk-capture',action='store_true')
    p.add_argument('--quota-rounds',type=int,default=2);p.add_argument('--stop-after-cells',type=int,default=0)
    p.add_argument('--seed-capture',type=Path)
    a=p.parse_args();m=loop.gen.geo.load_json(a.manifest)
    ids=[int(i) for i in a.indices.split(',')] if a.indices else list(range(min(a.limit,len(m['scenes']))))
    raise SystemExit(execute(a.manifest,ids,a.output,a.resume,a.quota_rounds,a.stop_after_cells,a.approve_bulk_capture,a.seed_capture))
