#!/usr/bin/env python3
"""Fresh, ranked, one-shot supplements to a verified frozen Base5 collection.

Never changes the original runtime/journal/raw. Projection is a proposal score,
not a visibility certificate; only the unchanged live auditor can fill quota.
"""
import argparse
import copy
import fcntl
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import generate_base5_independent as gen
import run_base5_batch_campaign as batch
from base5_campaign_replenishment import cell_contract, check_placement
from base5_capture_safety import atomic_json, load_ledger, prepare_session, sha256
from base5_region_contract import foreground_angular_coverage
from dynamic_capture_metadata import normalized_sensor_calibration

ROOT=Path(__file__).resolve().parents[3]
SENSOR=ROOT/'configs/sensors/Sensor_competition_local.json'
NETWORK=ROOT/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json'


def read(path):return json.loads(Path(path).read_text())


def view(meta):
    a=meta['runtime_actors'][0]
    return (a['longitudinal_m'],a['lateral_m'],
        ((a.get('placement_yaw_deg',meta['ego']['yaw'])-meta['ego']['yaw']+180)%360)-180)


def geometry_score(meta,calibration):
    """Unoccluded camera rectangle / VLP16 beam proxy; never edits native GT."""
    actor=meta['runtime_actors'][0];ego=meta['ego']
    size=np.asarray(actor['estimated_size']);x,y=actor['longitudinal_m'],actor['lateral_m']
    z=actor['position']['z']-ego['z']+size[2]/2
    yaw=math.radians(view(meta)[2]);c,s=math.cos(yaw),math.sin(yaw)
    rot=np.asarray([[c,-s,0],[s,c,0],[0,0,1]])
    corners=np.asarray(list(itertools.product([-1,1],repeat=3)))*size/2
    corners=corners@rot.T+np.asarray([x,y,z])
    best=0.
    for camera in calibration['cameras'].values():
        optical=(np.c_[corners,np.ones(8)]@np.asarray(camera['ego_to_sensor']).T)[:,:3]
        optical=optical[optical[:,2]>.1]
        if len(optical)<4:continue
        image=optical@np.asarray(camera['intrinsic']).T;image=image[:,:2]/image[:,2:3]
        width,height=camera['resolution_px'];scale=896./width;top=max(0,height-320/scale)
        lo=np.maximum(image.min(axis=0),[0,top]);hi=np.minimum(image.max(axis=0),[width,height])
        extent=np.maximum(0,hi-lo)*scale
        best=max(best,float(min(extent)),0.)
    transform=np.asarray(calibration['lidar']['ego_to_lidar'])
    lidar=(np.c_[corners,np.ones(8)]@transform.T)[:,:3]
    elevations=np.degrees(np.arctan2(lidar[:,2],np.linalg.norm(lidar[:,:2],axis=1)))
    beams=int(sum(elevations.min()<=v<=elevations.max() for v in range(-15,16,2)))
    # Footprints alone cannot prove blockage in height. This is a soft penalty.
    boxes=[gen.rectangle(a['geometric_center_estimate'],a['estimated_size'],
        a.get('placement_yaw_deg',ego['yaw'])) for a in meta['runtime_actors']]
    occlusion=max((foreground_angular_coverage((ego['x'],ego['y']),boxes[0],b) for b in boxes[1:]),default=0.)
    return dict(camera_min_extent_proxy_px=best,lidar_beam_proxy=beams,
        angular_occlusion_proxy=occlusion,
        score=(min(best,20)/4+min(beams,4)*2)*(1-.8*occlusion),
        limitation='Estimated parent height/rectangles, no rendered mask, road occlusion or actual ray test')


def novelty(meta,history):
    current=view(meta)
    if not history:return 1.
    def separation(old):
        v=view(old);distance=math.hypot(current[0]-v[0],current[1]-v[1])
        yaw=abs((current[2]-v[2]+180)%360-180)
        scale=max(1.,min(10.,meta['runtime_actors'][0]['planned_center_distance_m']*.1))
        return min(2.,distance/scale+yaw/30.)
    return min(map(separation,history))


def choose(entry,config,ctx,calibration,history,ordinal,candidates=16):
    original=read(entry['metadata']);identity=cell_contract(entry);ranked=[];tried=0
    # Non-overlapping search blocks; never restart at original retry+1.
    first=max([original.get('retry',0)]+[m.get('retry',0) for m in history])+1+(ordinal-1)*4096
    for retry in range(first,first+512):
        tried+=1
        try:scene,meta=gen.build(original['spec'],config,ctx,retry)
        except ValueError:continue
        if meta['split']!=identity['split']:continue
        if meta['runtime_actors'][0]['data_id']!=identity['primary']:raise ValueError('Primary changed')
        # Population-budget sampling can revise row.actors. Do not let that
        # become a post-write contract error or silently change the frozen cell.
        if meta['spec']!=identity['spec']:continue
        fresh=novelty(meta,history)
        if fresh<.15:continue  # Reject essentially the same failed relative view.
        proxy=geometry_score(meta,calibration)
        ranked.append((proxy['score']+min(1.,fresh),retry,scene,meta,proxy,fresh))
        if len(ranked)>=candidates:break
    if not ranked:raise ValueError('No distinct same-contract candidate within bounded search')
    _,retry,scene,meta,proxy,fresh=max(ranked,key=lambda r:(r[0],-r[1]))
    return scene,meta,dict(policy='failure-aware-ranked-one-shot-v134',retry=retry,
        proposals_tried=tried,valid_candidates=len(ranked),proxy=proxy,relative_view_novelty=fresh,
        history_views=len(history),identity_contract=identity,
        decision='Rank only; unchanged live gates decide acceptance')


def prepare(entry,deck,ctx,calibration,history,ordinal,folder,candidates=16):
    if folder.exists():raise FileExistsError('Never overwrite a candidate or its raw')
    scene,meta,proof=choose(entry,deck['config'],ctx,calibration,history,ordinal,candidates)
    old=read(entry['metadata'])
    for key in ('scene_group_id','environment_split_contract'):
        if key in old:meta[key]=copy.deepcopy(old[key])
    meta['scene_group_id']=cell_contract(entry)['group']
    meta['failure_aware_replenishment']=proof
    folder.mkdir(parents=True)
    sid=entry['id']+'_fa'+str(ordinal).zfill(2)
    sp=folder/(sid+'.json');mp=folder/(sid+'_metadata.json')
    atomic_json(sp,scene);atomic_json(mp,meta)
    row=copy.deepcopy(entry);row.update(id=sid,source_cell_id=entry['id'],scenario=str(sp.resolve()),
        metadata=str(mp.resolve()),scenario_sha256=sha256(sp),motion=meta['runtime_actors'][0]['motion'])
    manifest=folder/'manifest.json';atomic_json(manifest,batch.campaign.subset_deck(deck,[row]))
    check_placement(entry,manifest,deck['config']);return manifest


def history_for(baseline,entry):
    history=[read(entry['metadata'])]
    paths=list((baseline/'replenishment').glob('round_*/'+entry['id']+'/**/captures/*/metadata.json'))
    paths+=list(baseline.glob('chunk_*/recovery/'+entry['id']+'/**/captures/*/metadata.json'))
    for path in paths:
        meta=read(path)
        if meta.get('runtime_actors') and meta.get('spec')==history[0]['spec']:history.append(meta)
    return history


def verify_attempt(attempt,entry,deck):
    manifest=Path(attempt['manifest']);root=Path(attempt['capture'])
    if sha256(manifest)!=attempt['manifest_sha256']:raise ValueError('Candidate manifest changed')
    check_placement(entry,manifest,deck['config'])
    session=read(root/'session_contract.json')
    rows,_=prepare_session(root,manifest,NETWORK,SENSOR,session.get('runner_options'))
    if not read(root/'shutdown_safety.json').get('park_verified'):raise ValueError('Final Park unverified')
    if len(rows)!=1:raise ValueError('One-shot runner must produce exactly one row')
    row=rows[0]
    if row['status']=='accepted_candidate':
        if row['audit']['review_reasons'] or read(Path(row['capture_root'])/'audit.json')!=row['audit']:
            raise ValueError('Selected audit mismatch')
        meta=read(Path(row['capture_root'])/'metadata.json')
        expected=cell_contract(entry)
        if meta['spec']!=expected['spec'] or meta['split']!=expected['split'] or meta['scene_group_id']!=expected['group']:
            raise ValueError('Selected metadata changed identity')
        if meta.get('calibration_only') or meta['runtime_actors'][0]['data_id']!=expected['primary']:
            raise ValueError('Invalid selected primary')
    elif row['status']=='failed':
        from base5_capture_safety import is_collision_failure,is_stationary_failure,validate_collision_failure,validate_stationary_failure
        if is_collision_failure(row):validate_collision_failure(row)
        elif is_stationary_failure(row):validate_stationary_failure(row)
        else:raise ValueError('Unsafe failure cannot be skipped')
    return row


def execute(source,baseline,output,resume=False,run=False,stop_after=0,max_attempts=2,candidates=16,seed_supplement=None):
    if not 1<=max_attempts<=3 or not 1<=candidates<=64:raise ValueError('Bounded attempts/candidates required')
    if output.exists() and not resume:raise FileExistsError('Existing supplement requires --resume')
    if not (baseline/'PAUSE').exists():raise ValueError('Pause original campaign first')
    if read(baseline/'status.json')['stage']!='paused':raise ValueError('Original must be safely paused')
    output.mkdir(parents=True,exist_ok=True)
    lock=(output/'.supplement.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    # Mutual exclusion with original campaign, held for the entire supplement.
    original_lock=(baseline/'.batch.lock').open('a');fcntl.flock(original_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    state_path=output/'state.json';deck=read(source);lookup={r['id']:r for r in deck['scenes']}
    contract=dict(schema=1,source_sha256=sha256(source),baseline=str(baseline.resolve()),
        baseline_hashes={name:sha256(baseline/name) for name in
            ('selected_ledger.jsonl','quota.json','replenishment/state.json')},
        code_sha256=sha256(__file__),capture_contract=batch.campaign.contract(source,list(range(len(deck['scenes']))),2),
        max_attempts=max_attempts,candidates=candidates)
    if seed_supplement is not None:
        contract['import_supplement']=dict(path=str(seed_supplement.resolve()),
            state_sha256=sha256(seed_supplement/'state.json'),ledger_sha256=sha256(seed_supplement/'selected_ledger.jsonl'))
    try:
        if state_path.exists():
            state=read(state_path)
            if state['contract']!=contract:raise ValueError('Supplement resume contract changed')
            if state['stage']=='halted':raise ValueError('Halted supplement requires safety diagnosis')
        else:
            # Frozen original validators, including imported raw and completed replenishment.
            verified={};indices=list(range(len(deck['scenes'])))
            code=batch.execute(source,indices,baseline,read(baseline/'batch_contract.json')['chunk_size'],
                True,bulk=True,_verification=verified,_lock_held=True)
            if code or not verified:raise ValueError('Original main evidence verification failed')
            journal=read(baseline/'replenishment/state.json')
            from base5_campaign_replenishment import result,merged
            for a in journal['attempts']:
                if not a.get('result'):raise ValueError('Original pending child remains')
                if result(Path(a['manifest']),Path(a['child']),lookup[a['id']])!=a['result']:
                    raise ValueError('Original supplement evidence changed')
            selected,remaining=merged(journal,deck,indices)
            if selected!=load_ledger(baseline/'selected_ledger.jsonl'):raise ValueError('Original merged ledger mismatch')
            state=dict(contract=contract,stage='prepared',baseline_selected=selected,
                deficits=list(remaining),attempts=[],unavailable={},elapsed_sec=0.)
            if seed_supplement is not None:
                legacy=read(seed_supplement/'state.json');old_contract=legacy['contract']
                for key in ('source_sha256','baseline','baseline_hashes','capture_contract','max_attempts','candidates'):
                    if old_contract[key]!=contract[key]:raise ValueError('Imported supplement source/capture/quota changed')
                if legacy['baseline_selected']!=selected or legacy['deficits']!=list(remaining):
                    raise ValueError('Imported supplement baseline changed')
                if any(a.get('result') is None for a in legacy['attempts']):
                    raise ValueError('Cannot import a pending live attempt')
                # Explicit import validates raw with the unchanged capture
                # runtime. It does not rewrite a halted journal or bypass its
                # old driver hash. Failed/unattempted geometry is not reused.
                reconstructed=copy.deepcopy(selected);used={r['source_cell_id'] for r in selected}
                for a in legacy['attempts']:
                    if verify_attempt(a,lookup[a['id']],deck)!=a['result']:raise ValueError('Imported raw evidence changed')
                    row=a['result']
                    if row['status']=='accepted_candidate':
                        if a['id'] in used:raise ValueError('Imported cell selected twice')
                        chosen=copy.deepcopy(row);chosen.update(source_cell_id=a['id'],scene_group_id=cell_contract(lookup[a['id']])['group'],
                            failure_aware_supplement=dict(manifest=a['manifest'],manifest_sha256=a['manifest_sha256'],baseline=str(baseline)))
                        reconstructed.append(chosen);used.add(a['id'])
                order={r['id']:i for i,r in enumerate(deck['scenes'])};reconstructed.sort(key=lambda r:order[r['source_cell_id']])
                if reconstructed!=load_ledger(seed_supplement/'selected_ledger.jsonl'):raise ValueError('Imported merged ledger mismatch')
                state.update(attempts=copy.deepcopy(legacy['attempts']),unavailable=copy.deepcopy(legacy['unavailable']),
                    elapsed_sec=legacy['elapsed_sec'],imported_driver_sha256=old_contract['code_sha256'])
            atomic_json(state_path,state)
        for a in state['attempts']:
            if a.get('result') is not None and verify_attempt(a,lookup[a['id']],deck)!=a['result']:
                raise ValueError('Completed attempt evidence changed')
        started=time.monotonic();prior=state['elapsed_sec']
        def save(stage,error=None):
            state.update(stage=stage,error=error,elapsed_sec=prior+time.monotonic()-started if run else prior)
            atomic_json(state_path,state)
            selected=copy.deepcopy(state['baseline_selected']);used={r['source_cell_id'] for r in selected}
            for a in state['attempts']:
                row=a.get('result')
                if not row or row['status']!='accepted_candidate':continue
                if a['id'] in used:raise ValueError('Cell selected twice')
                chosen=copy.deepcopy(row);chosen.update(source_cell_id=a['id'],scene_group_id=cell_contract(lookup[a['id']])['group'],
                    failure_aware_supplement=dict(manifest=a['manifest'],manifest_sha256=a['manifest_sha256'],baseline=str(baseline)))
                selected.append(chosen);used.add(a['id'])
            order={r['id']:i for i,r in enumerate(deck['scenes'])};selected.sort(key=lambda r:order[r['source_cell_id']])
            body=''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected)
            ledger=output/'selected_ledger.jsonl'
            if not ledger.exists() or ledger.read_text()!=body:
                temp=output/'.selected.tmp';temp.write_text(body);temp.replace(ledger)
            remaining=[i for i in state['deficits'] if i not in used]
            atomic_json(output/'quota.json',dict(target=len(deck['scenes']),accepted=len(selected),deficit=len(remaining),
                cells=[dict(id=i,index=order[i],reason=state['unavailable'].get(i,'bounded_live_review')) for i in remaining]))
            atomic_json(output/'deficit_manifest.json',batch.campaign.subset_deck(deck,[lookup[i] for i in remaining]))
            atomic_json(output/'status.json',dict(stage=stage,error=error,mode='failure_aware_replenishment',requested=len(deck['scenes']),
                finished=len(deck['scenes']),accepted_candidate=len(selected),review=len(remaining),failed=int(stage=='halted'),
                initial_accepted=len(state['baseline_selected']),replenished=len(selected)-len(state['baseline_selected']),
                remaining_deficit=len(remaining),elapsed_sec=read(baseline/'status.json')['elapsed_sec']+state['elapsed_sec'],
                replenishment_elapsed_sec=state['elapsed_sec'],eta_sec=0,updated_at=time.time(),current=state.get('current')))
            return used
        used=save(state['stage'])
        if not run:return 0
        ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO);calibration=normalized_sensor_calibration(SENSOR)
        processed=0
        for ident in state['deficits']:
            if ident in used or ident in state['unavailable']:continue
            if sum(a['id']==ident and a.get('result') is not None for a in state['attempts'])>=max_attempts:continue
            source_meta=read(lookup[ident]['metadata'])
            if any(v['motion'] not in ('static','standing') for v in source_meta['runtime_actors']):
                state['unavailable'][ident]='moving_requires_dedicated_ground_preparation';save('running');continue
            for ordinal in range(1,max_attempts+1):
                a=next((a for a in state['attempts'] if a['id']==ident and a['ordinal']==ordinal),None)
                if a and a.get('result') is not None:continue
                if (output/'PAUSE').exists() or (output/'STOP').exists():save('paused');return 0
                state['current']=ident;save('running')
                if a is None:
                    history=history_for(baseline,lookup[ident])
                    for prev in state['attempts']:
                        if prev['id']==ident:history.append(read(read(prev['manifest'])['scenes'][0]['metadata']))
                    folder=output/'cells'/ident/f'candidate_{ordinal}'
                    try:
                        manifest=(folder/'placement/manifest.json')
                        if not manifest.exists():manifest=prepare(lookup[ident],deck,ctx,calibration,history,ordinal,folder/'placement',candidates)
                        check_placement(lookup[ident],manifest,deck['config'])
                    except ValueError as exc:
                        if str(exc)!='No distinct same-contract candidate within bounded search':raise
                        state['unavailable'][ident]=str(exc);save('running');break
                    # Reject exact old raw and old candidate payloads, not just scenario names.
                    newsha=read(manifest)['scenes'][0]['scenario_sha256']
                    oldshas={lookup[ident]['scenario_sha256']}
                    for p in (baseline/'replenishment').glob('round_*/'+ident+'/**/scenario.json'):oldshas.add(sha256(p))
                    for prev in state['attempts']:
                        if prev['id']==ident:oldshas.add(read(prev['manifest'])['scenes'][0]['scenario_sha256'])
                    if newsha in oldshas:raise ValueError('Candidate repeats a known payload')
                    a=dict(id=ident,ordinal=ordinal,manifest=str(manifest),manifest_sha256=sha256(manifest),
                        capture=str(folder/'capture'),result=None)
                    state['attempts'].append(a);save('running')
                meta=read(read(a['manifest'])['scenes'][0]['metadata'])
                if any(v['motion'] not in ('static','standing') for v in meta['runtime_actors']):
                    # Dynamic targets still require the established terrain preparation.
                    raise ValueError('Moving supplement requires dedicated prepared quality campaign; no direct raw bypass')
                log=Path(a['capture']).parent/'runner.log'
                capture_root=Path(a['capture'])
                if (capture_root/'shutdown_safety.json').exists() and load_ledger(capture_root/'ledger.jsonl'):
                    code=0  # Crash after completed raw but before commit: verify, never replay.
                else:
                    with log.open('a') as handle:
                        child=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).with_name('run_base5_capture.py')),
                            '--manifest',a['manifest'],'--output',a['capture'],'--limit','1'],stdout=handle,stderr=subprocess.STDOUT)
                        deadline=time.monotonic()+120
                        try:
                            while True:
                                try:code=child.wait(timeout=1);break
                                except subprocess.TimeoutExpired:
                                    progress=read(output/'status.json')
                                    progress.update(updated_at=time.time(),current=str(capture_root),
                                        child_status=str(capture_root/'status.json'))
                                    atomic_json(output/'status.json',progress)
                                    if time.monotonic()>deadline:raise RuntimeError('One-shot child timeout; no next load')
                        except BaseException:
                            # Ask the established runner to brake/Park, then wait.
                            atomic_json(capture_root/'STOP',dict(reason='supplement parent stopping'))
                            try:child.wait(timeout=30)
                            except subprocess.TimeoutExpired:pass  # Halt; never start another MR command.
                            raise
                # The raw runner may return 1 for a witnessed safe collision/stationary miss.
                row=verify_attempt(a,lookup[ident],deck)
                if code and row['status']!='failed':raise RuntimeError('Runner failed without safe per-scene proof')
                a['result']=row;used=save('running')
                print(json.dumps(dict(cell=ident,attempt=ordinal,status=row['status'],
                    reasons=row.get('audit',{}).get('review_reasons',[]))),flush=True)
                if ident in used:break
            processed+=1
            if stop_after and processed>=stop_after:state['current']=None;save('paused');return 0
        state['current']=None;save('complete' if len(used)==len(deck['scenes']) else 'complete_with_deficit');return 0
    except Exception as exc:
        if 'save' in locals():save('halted',str(exc))
        else:atomic_json(output/'preflight_error.json',dict(error=str(exc),time=time.time()))
        print(str(exc),file=sys.stderr);return 1
    finally:original_lock.close();lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('manifest','baseline','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--resume',action='store_true');p.add_argument('--run',action='store_true')
    p.add_argument('--stop-after-cells',type=int,default=0)
    p.add_argument('--max-attempts',type=int,default=2);p.add_argument('--candidates',type=int,default=16)
    p.add_argument('--import-supplement',type=Path,help='Explicitly validate/import completed attempts into a NEW output; never force resume')
    a=p.parse_args();raise SystemExit(execute(a.manifest,a.baseline,a.output,a.resume,a.run,a.stop_after_cells,a.max_attempts,a.candidates,a.import_supplement))
