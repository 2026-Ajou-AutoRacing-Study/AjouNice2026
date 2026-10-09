#!/usr/bin/env python3
"""Bounded, reason-aware completion; original ledgers and capture gates stay frozen."""
import argparse
from collections import Counter
import copy
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import time

import run_base5_failure_aware_replenishment as fa
from run_base5_failure_aware_replenishment_fast import prewarm

DEFAULT_ROOT=Path.home()/'morai_cap/base5_v2_261008/final_appearance_5k_v132_release'


def reasons(value):
    if isinstance(value,str):return {value}
    if isinstance(value,list):return set().union(*(reasons(v) for v in value)) if value else set()
    if isinstance(value,dict):
        return set().union(*(reasons(value[k]) for k in ('reason','reasons','review_reasons') if k in value))
    return set()


def strategy(history_rows,moving):
    found=set()
    for row in history_rows:found.update(reasons(row.get('audit',{}).get('review_reasons',row.get('reasons',[]))))
    if moving:return 'prepared_motion',found
    if 'missing_runtime_actor' in found:return 'spawn_clearance',found
    if any('ground' in v for v in found):return 'terrain_and_visibility',found
    if 'stationary_vehicle_moved_from_placement' in found:return 'supported_vehicle_pose',found
    if 'population_visible_actor_deficit' in found:return 'all_actor_visibility',found
    return 'primary_visibility',found


def proposal_score(meta,calibration,kind):
    """Rank all parents, not just the primary; proxies never approve raw."""
    proxies=[];penalty=0.
    actors=meta['runtime_actors']
    for i,actor in enumerate(actors):
        view=dict(meta,runtime_actors=[actor]+actors[:i]+actors[i+1:])
        p=fa.geometry_score(view,calibration);proxies.append(p)
        ground=actor.get('map_ground_initializer',{})
        span=ground.get('footprint',{}).get('map_height_span_m')
        if span is not None:penalty+=min(4.,max(0.,span-.10)*8)
        if kind in ('terrain_and_visibility','supported_vehicle_pose') and ground and not ground.get('usable'):
            penalty+=4.
    predicted=sum((p['lidar_beam_proxy']>0 or p['camera_min_extent_proxy_px']>=4)
                  and p['angular_occlusion_proxy']<.85 for p in proxies)
    required=meta.get('population_budget',{}).get('required_visible_actors',1)
    primary=proxies[0]
    if kind=='spawn_clearance':
        boxes=[fa.gen.rectangle(a['geometric_center_estimate'],a['estimated_size'],
            a.get('placement_yaw_deg',meta['ego']['yaw'])) for a in actors]
        for i,box in enumerate(boxes):
            for other in boxes[i+1:]:penalty+=20*box.intersection(other).area/max(.01,min(box.area,other.area))
    score=primary['score']+min(primary['camera_min_extent_proxy_px'],80)/10
    score-=20*max(0,required-predicted)
    score+=sum(min(p['score'],8) for p in proxies[1:])/max(1,len(proxies)-1)
    score-=penalty
    return dict(score=float(score),predicted_visible_parents=int(predicted),required_visible_parents=required,
        primary=primary,parents=proxies,terrain_penalty=penalty,
        limitation='Geometry-only ranking; no rendered visibility, collision or measured-ground certification')


def choose(entry,config,ctx,calibration,history,ordinal,candidates=32,kind='primary_visibility',heartbeat=None):
    original=fa.read(entry['metadata']);identity=fa.cell_contract(entry)
    first=max([original.get('retry',0)]+[m.get('retry',0) for m in history])+1+ordinal*8192
    ranked=[];tried=0
    for retry in range(first,first+1024):
        tried+=1
        if heartbeat and (retry-first)%32==0:heartbeat()
        try:scene,meta=fa.gen.build(original['spec'],config,ctx,retry)
        except ValueError:continue
        if meta['split']!=identity['split'] or meta['spec']!=identity['spec']:continue
        if meta['runtime_actors'][0]['data_id']!=identity['primary']:raise ValueError('Primary changed')
        fresh=fa.novelty(meta,history)
        if fresh<.25:continue
        if kind=='prepared_motion':
            from prepare_base5_empty_ground_probe import target_probe_pose
            try:
                for actor in meta['runtime_actors']:
                    target=actor.get('motion_path',{}).get('target_position',actor['position'])
                    target_probe_pose(ctx['links'],dict(actor,position=target),maximum_road_distance=8.)
            except ValueError:continue
        proxy=proposal_score(meta,calibration,kind)
        ranked.append((proxy['score']+min(fresh,1.),retry,scene,meta,proxy,fresh))
        if len(ranked)>=candidates:break
    if not ranked:raise ValueError('No distinct same-contract candidate within bounded search')
    _,retry,scene,meta,proxy,fresh=max(ranked,key=lambda item:(item[0],-item[1]))
    return scene,meta,dict(policy='reason-aware-all-parent-completion-v136',strategy=kind,retry=retry,
        proposals_tried=tried,valid_candidates=len(ranked),proxy=proxy,
        relative_view_novelty=fresh,history_views=len(history),identity_contract=identity,
        decision='Only unchanged live native-GT/ground/motion/collision/pose gates approve the candidate')


def prepare(entry,deck,ctx,calibration,history,ordinal,folder,kind,heartbeat=None):
    original_choose=fa.choose
    def adaptive(*args,**kwargs):return choose(*args,**kwargs,kind=kind,heartbeat=heartbeat)
    fa.choose=adaptive
    try:return fa.prepare(entry,deck,ctx,calibration,history,ordinal,folder,32)
    finally:fa.choose=original_choose


def verify_dynamic(attempt,entry,deck):
    fa.batch.verify_recovery(Path(attempt['manifest']),0,Path(attempt['capture']))
    rows=fa.load_ledger(Path(attempt['capture'])/'selected_ledger.jsonl')
    if rows:
        row=rows[0];meta=fa.read(Path(row['capture_root'])/'metadata.json');identity=fa.cell_contract(entry)
        if (meta['spec']!=identity['spec'] or meta['split']!=identity['split'] or
                meta.get('scene_group_id')!=identity['group'] or
                meta['runtime_actors'][0]['data_id']!=identity['primary']):
            return dict(status='review',reasons=['replacement_identity_changed'],observed_selected=row)
        return row
    return dict(status='review',reasons=fa.read(Path(attempt['capture'])/'quota.json')['cells'])


def verify(attempt,entry,deck):
    if fa.sha256(attempt['manifest'])!=attempt['manifest_sha256']:raise ValueError('Candidate manifest changed')
    fa.check_placement(entry,attempt['manifest'],deck['config'])
    return verify_dynamic(attempt,entry,deck) if attempt['moving'] else fa.verify_attempt(attempt,entry,deck)


def run_child(attempt,heartbeat):
    root=Path(attempt['capture']);manifest=Path(attempt['manifest'])
    if attempt['moving']:
        if (root/'status.json').exists() and fa.read(root/'status.json')['stage'] in ('complete','complete_with_deficit'):
            return 0
        if root.exists():raise RuntimeError('Interrupted dynamic preparation needs manual safety inspection')
        command=[sys.executable,str(Path(__file__).with_name('run_base5_capture_campaign.py')),
            '--manifest',str(manifest),'--output',str(root),'--limit','1','--quota-rounds','2']
        timeout=800
    else:
        if (root/'shutdown_safety.json').exists() and fa.load_ledger(root/'ledger.jsonl'):return 0
        if (root/'session_contract.json').exists():raise RuntimeError('Interrupted raw session needs safety inspection')
        command=['/usr/bin/python3',str(Path(__file__).with_name('run_base5_capture.py')),
            '--manifest',str(manifest),'--output',str(root),'--limit','1']
        timeout=120
    with (root.parent/'runner.log').open('a') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+timeout
        try:
            while child.poll() is None:
                heartbeat(root)
                if time.monotonic()>deadline:raise RuntimeError('Child timeout; no next load')
                time.sleep(1)
            return child.returncode
        except BaseException:
            fa.atomic_json(root/'STOP',dict(reason='Adaptive parent stopping'))
            try:child.wait(timeout=30)
            except subprocess.TimeoutExpired:pass
            raise


def publish(output,state,deck,elapsed=None):
    selected=copy.deepcopy(state['baseline_selected']);used={r['source_cell_id'] for r in selected}
    for attempt in state['attempts']:
        row=attempt.get('result')
        if not row or row['status']!='accepted_candidate':continue
        if attempt['id'] in used:raise ValueError('Duplicate selected quota cell')
        row=copy.deepcopy(row);row.update(source_cell_id=attempt['id'],scene_group_id=fa.cell_contract(state['lookup'][attempt['id']])['group'],
            adaptive_completion=dict(manifest=attempt['manifest'],strategy=attempt['strategy'],round=attempt['round']))
        selected.append(row);used.add(attempt['id'])
    order={r['id']:i for i,r in enumerate(deck['scenes'])};selected.sort(key=lambda r:order[r['source_cell_id']])
    body=''.join(json.dumps(row,ensure_ascii=False)+'\n' for row in selected)
    ledger=output/'selected_ledger.jsonl'
    if not ledger.exists() or ledger.read_text()!=body:
        tmp=output/'.selection.tmp';tmp.write_text(body);tmp.replace(ledger)
    remaining=[i for i in state['deficits'] if i not in used]
    fa.atomic_json(output/'quota.json',dict(target=len(deck['scenes']),accepted=len(selected),deficit=len(remaining),pending=0,
        cells=[dict(id=i,reason=state['unavailable'].get(i,'adaptive_pending_or_review')) for i in remaining]))
    fa.atomic_json(output/'deficit_manifest.json',fa.batch.campaign.subset_deck(deck,[state['lookup'][i] for i in remaining]))
    bins=Counter(json.dumps([r.get('family'),r.get('distance_bounds'),r.get('role')]) for r in selected)
    elapsed=state['elapsed_sec'] if elapsed is None else elapsed
    fa.atomic_json(output/'status.json',dict(stage=state['stage'],error=state.get('error'),mode='failure_aware_replenishment',
        adaptive_completion=True,requested=len(deck['scenes']),finished=len(deck['scenes']),accepted_candidate=len(selected),
        review=len(remaining),failed=int(state['stage']=='halted'),initial_accepted=len(state['baseline_selected']),
        replenished=len(selected)-len(state['baseline_selected']),remaining_deficit=len(remaining),
        elapsed_sec=state['baseline_elapsed_sec']+elapsed,replenishment_elapsed_sec=elapsed,eta_sec=0,
        updated_at=time.time(),current=state.get('current'),accepted_bins=dict(bins),
        adaptive_round=state.get('round'),adaptive_round_cap=state['contract']['rounds'],
        adaptive_attempts=sum(a.get('result') is not None for a in state['attempts'])))
    return used


def execute(source,baseline,seed,output,run=False,resume=False,rounds=3,time_budget=1200,stop_after=0,only_cells=()):
    if not 1<=rounds<=3 or not 30<=time_budget<=3600:raise ValueError('Bounded round/time budget required')
    if output.exists() and not resume:raise FileExistsError('Use --resume for existing adaptive output')
    output.mkdir(parents=True,exist_ok=True);locks=[];state=None
    started=time.monotonic();prior=0.;deadline=started+time_budget
    try:
        for path in (output/'.adaptive.lock',seed/'.supplement.lock'):
            lock=path.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(lock)
        seed_state=fa.read(seed/'state.json');seed_status=fa.read(seed/'status.json')
        if seed_status['stage'] not in ('paused','complete_with_deficit','complete'):raise ValueError('Finish/pause seed supplement first')
        if any(a.get('result') is None for a in seed_state['attempts']):raise ValueError('Pending seed child')
        deck=fa.read(source);lookup={e['id']:e for e in deck['scenes']}
        contract=dict(schema=1,source_sha256=fa.sha256(source),code_sha256=fa.sha256(__file__),
            seed=str(seed.resolve()),seed_hashes={n:fa.sha256(seed/n) for n in ('state.json','selected_ledger.jsonl','quota.json')},
            baseline=str(baseline.resolve()),rounds=rounds,only_cells=list(only_cells),
            capture_contract=fa.batch.campaign.contract(source,list(range(len(deck['scenes']))),2),
            policy='Explicit additional campaign, prior attempts retained; fixed quota; no gate waivers')
        sp=output/'state.json'
        if sp.exists():
            state=fa.read(sp)
            if state['contract']!=contract:raise ValueError('Adaptive resume contract changed')
            if state['stage']=='halted':raise ValueError('Halted adaptive campaign requires safety inspection')
            prior=state['elapsed_sec']
        else:
            # Existing importer revalidates the entire main and old supplement
            # without changing them. Its journal remains separate from new rounds.
            verification=output/'verified_import'
            code=fa.execute(source,baseline,verification,resume=verification.exists(),seed_supplement=seed)
            if code:raise ValueError('Frozen source import verification failed')
            selected=fa.load_ledger(verification/'selected_ledger.jsonl')
            if selected!=fa.load_ledger(seed/'selected_ledger.jsonl'):raise ValueError('Imported selection changed')
            state=dict(contract=contract,stage='prepared',baseline_selected=selected,baseline_elapsed_sec=seed_status['elapsed_sec'],
                deficits=[r['id'] for r in fa.read(seed/'quota.json')['cells']],lookup=lookup,attempts=[],
                source_failure_reasons={d['id']:d.get('reasons',[]) for d in
                    (fa.read(baseline/'replenishment/state.json')['baseline']['deficits']
                     if (baseline/'replenishment/state.json').exists() else [])},
                unavailable={},elapsed_sec=0.,current=None)
            if set(only_cells)-set(state['deficits']):raise ValueError('Requested cell is not a deficit')
            fa.atomic_json(sp,state)
        lock=(baseline/'.batch.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(lock)
        for a in state['attempts']:
            if a.get('result') is not None and verify(a,lookup[a['id']],deck)!=a['result']:
                raise ValueError('Completed adaptive evidence changed')
        def save(stage,error=None):
            state.update(stage=stage,error=error,elapsed_sec=prior+time.monotonic()-started if run else prior)
            fa.atomic_json(sp,state);return publish(output,state,deck)
        def heartbeat(root=None):
            if root:state['current']=str(root)
            status=fa.read(output/'status.json');status.update(updated_at=time.time(),current=state['current'],
                elapsed_sec=state['baseline_elapsed_sec']+prior+time.monotonic()-started)
            fa.atomic_json(output/'status.json',status)
        used=publish(output,state,deck)
        if not run or state['stage'] in ('complete','complete_with_deficit'):return 0
        ctx=prewarm(fa.gen.context(fa.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,fa.gen.geo.DEFAULT_MGEO),deck['config'])
        calibration=fa.normalized_sensor_calibration(fa.SENSOR);processed=set()
        prior_counts=Counter(a['id'] for a in seed_state['attempts'])
        old_sha={a['result'].get('scenario_sha256') for a in seed_state['attempts'] if a.get('result')}
        history_cache={}
        for round_no in range(1,rounds+1):
            state['round']=round_no
            for ident in state['deficits']:
                if ident in used or ident in state['unavailable'] or (only_cells and ident not in only_cells):continue
                a=next((a for a in state['attempts'] if a['id']==ident and a['round']==round_no),None)
                if a and a.get('result') is not None:continue
                if time.monotonic()>=deadline or (output/'PAUSE').exists() or (output/'STOP').exists():
                    state['current']=None;save('paused');return 0
                state['current']=ident;save('running')
                old=[v['result'] for v in seed_state['attempts'] if v['id']==ident and v.get('result')]
                old += [v['result'] for v in state['attempts'] if v['id']==ident and v.get('result')]
                old.append(dict(reasons=state['source_failure_reasons'].get(ident,[])))
                original=fa.read(lookup[ident]['metadata'])
                moving=any(v['motion'] not in ('static','standing') for v in original['runtime_actors'])
                kind,found=strategy(old,moving)
                if a is None:
                    if ident not in history_cache:
                        history_cache[ident]=fa.history_for(baseline,lookup[ident])+[
                            fa.read(fa.read(v['manifest'])['scenes'][0]['metadata']) for v in seed_state['attempts'] if v['id']==ident]
                    history=history_cache[ident]+[fa.read(fa.read(v['manifest'])['scenes'][0]['metadata'])
                        for v in state['attempts'] if v['id']==ident]
                    folder=output/'cells'/ident/f'round_{round_no}'
                    manifest=folder/'placement/manifest.json'
                    try:
                        if not manifest.exists():manifest=prepare(lookup[ident],deck,ctx,calibration,history,
                            prior_counts[ident]+round_no,manifest.parent,kind,heartbeat)
                    except ValueError as exc:
                        if str(exc)!='No distinct same-contract candidate within bounded search':raise
                        state['unavailable'][ident]=str(exc);save('running');continue
                    newsha=fa.read(manifest)['scenes'][0]['scenario_sha256']
                    known={lookup[ident]['scenario_sha256'],*old_sha}
                    known.update(fa.read(v['manifest'])['scenes'][0]['scenario_sha256'] for v in state['attempts'])
                    known.update(fa.sha256(p) for p in (baseline/'replenishment').glob('round_*/'+ident+'/**/scenario.json'))
                    if newsha in known:raise ValueError('New candidate repeats previous payload')
                    a=dict(id=ident,round=round_no,strategy=kind,previous_reasons=sorted(found),moving=moving,
                        manifest=str(manifest),manifest_sha256=fa.sha256(manifest),capture=str(folder/'capture'),result=None)
                    state['attempts'].append(a);save('running')
                code=run_child(a,heartbeat);row=verify(a,lookup[ident],deck)
                if code and row['status']!='failed':raise RuntimeError('Child failed without safe per-scene proof')
                a['result']=row;used=save('running');processed.add(ident)
                print(json.dumps(dict(cell=ident,round=round_no,strategy=kind,status=row['status'],
                    reasons=sorted(reasons(row.get('audit',{}).get('review_reasons',row.get('reasons',[]))))),ensure_ascii=False),flush=True)
                if stop_after and len(processed)>=stop_after:
                    state['current']=None;save('paused');return 0
        state['current']=None;save('complete' if len(used)==len(deck['scenes']) else 'complete_with_deficit');return 0
    except BaseException as exc:
        if state is not None and 'save' in locals():save('halted',str(exc))
        else:fa.atomic_json(output/'preflight_error.json',dict(error=str(exc),time=time.time()))
        if isinstance(exc,KeyboardInterrupt):return 130
        if not isinstance(exc,Exception):raise
        print(str(exc),file=sys.stderr);return 1
    finally:
        for lock in reversed(locks):lock.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    p.add_argument('--output',type=Path)
    p.add_argument('--run',action='store_true');p.add_argument('--resume',action='store_true')
    p.add_argument('--rounds',type=int,default=3);p.add_argument('--time-budget-sec',type=int,default=1200)
    p.add_argument('--stop-after-cells',type=int,default=0);p.add_argument('--cells',help='Comma-separated deficit IDs for a bounded pilot')
    a=p.parse_args()
    return execute(a.root/'environment_suite/sunny13/manifest.json',
        a.root/'collection_5k_final_v132_sunny13_recovered_v133',
        a.root/'collection_5k_sunny13_supplement_v135',
        a.output or a.root/'collection_5k_sunny13_adaptive_v137',a.run,a.resume,
        a.rounds,a.time_budget_sec,a.stop_after_cells,tuple(a.cells.split(',')) if a.cells else ())


if __name__=='__main__':raise SystemExit(main())
