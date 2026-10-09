#!/usr/bin/env python3
"""Finite same-family fresh-geometry completion for a terminal fresh batch.

Preserves the terminal parent and its successful rows. This is a new selection,
not an assertion that the original exact asset/range/count quotas were met.
"""
import argparse
from collections import Counter
import copy
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import base5_spatial_diversity as spatial
import run_base5_flexible_completion as flex
import run_base5_adaptive_completion as adaptive
from run_base5_motion_contract_repair import motion_matches
from base5_capture_safety import atomic_json, load_ledger, sha256

POLICY='fresh-environment-finite-family-completion-v141'
TERMINAL=('complete','complete_with_deficit')
POOLS=('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')


def read(path):return json.loads(Path(path).read_text())


def pool_map(ctx):return {v['data_id']:pool for pool in POOLS for v in ctx['pools'][pool]}


def floor_donors(original,donors,counts,floor):
    """Flexible replacement cannot silently remove a rare primary's floor."""
    if not original['runtime_actors']:return donors
    primary=original['runtime_actors'][0]['data_id']
    if counts[primary]>=floor:return donors
    same=[d for d in donors if d['runtime_actors'] and d['runtime_actors'][0]['data_id']==primary]
    # Source spec is a valid proposal template, never evidence of sensor success.
    return same or [original]


def compatible(source,candidate,pools):
    return flex.compatible(source,candidate,pools) and motion_matches(source,candidate)


def verify_parent(manifest,parent):
    status=read(parent/'status.json')
    if status['stage'] not in TERMINAL:raise ValueError('Finish the fresh batch before completion')
    if not (parent/'fresh_runtime_contract.json').exists():raise ValueError('Fresh batch parent required')
    contract=read(parent/'batch_contract.json');proof={}
    # Native reconstruction checks recorded session adapters and every captured
    # input/audit/Park proof. _verification cannot launch missing children.
    code=adaptive.fa.batch.execute(manifest,contract['indices'],parent,contract['chunk_size'],
        True,bulk=contract['bulk'],_verification=proof,_lock_held=True)
    if code or 'selected' not in proof:raise ValueError('Frozen parent evidence failed')
    selected=list(proof['selected'].values());order={e['id']:i for i,e in enumerate(read(manifest)['scenes'])}
    selected.sort(key=lambda r:order[r['source_cell_id']])
    if selected!=load_ledger(parent/'selected_ledger.jsonl'):raise ValueError('Parent selection differs from native proof')
    misses=set(proof['deficits'])
    if misses!={r['id'] for r in read(parent/'quota.json')['cells']}:raise ValueError('Parent deficit evidence differs')
    return selected,sorted(misses,key=order.get),status['elapsed_sec']


def merged(state):
    selected=copy.deepcopy(state['baseline_selected']);used={r['source_cell_id'] for r in selected}
    for a in state['attempts']:
        row=a.get('result')
        if not row or row['status']!='accepted_candidate':continue
        if a['id'] in used:raise ValueError('Quota slot selected twice')
        row=copy.deepcopy(row);row.update(source_cell_id=a['id'],scene_group_id=a['group'],
            flexible_completion=dict(policy=POLICY,manifest=a['manifest'],original_cell_id=a['id'],delta=a['delta']))
        selected.append(row);used.add(a['id'])
    order={v:i for i,v in enumerate(state['source_order'])};selected.sort(key=lambda r:order[r['source_cell_id']])
    return selected,[i for i in state['deficits'] if i not in used]


def prepare(entry,original,donors,deck,ctx,cal,ordinal,folder,known,pools):
    if folder.exists():raise FileExistsError('Never replace an existing proposal')
    with spatial.installed():
        if original['runtime_actors']:
            eligible=[d for d in donors if compatible(original,d,pools)]
            scene,meta=flex.choose(original,eligible,deck,ctx,cal,ordinal,entry['id'],known,pools)
            if not compatible(original,meta,pools):raise ValueError('Proposed primary gait or mover minima changed')
        else:
            spec=copy.deepcopy(original['spec']);spec['seed']=spatial.seed_for(deck['config']['environment_tag'],entry['id'],ordinal)
            for retry in range(512):
                try:scene,meta=spatial.build(spec,deck['config'],ctx,retry)
                except ValueError:continue
                digest=hashlib.sha256(json.dumps(scene,ensure_ascii=False,indent=2).encode()).hexdigest()
                if not meta['runtime_actors'] and meta['split']==original['split'] and digest not in known:break
            else:raise ValueError('No distinct background within bounded search')
    # Before raw capture, retain the SOURCE fresh geometry group, not the donor
    # group and not an old Sunny13 group invented from a logical slot number.
    group=original['scene_group_id'];meta['scene_group_id']=group
    meta['environment_split_contract']=dict(environment=deck['config']['environment_tag'],
        scene_group_id=group,split=original['split'],policy=POLICY,
        limitation='Same parent split; altered geometry/asset/range/count is explicit, not original exact quota')
    meta['fresh_completion_source']=dict(source_cell_id=entry['id'],source_scenario_sha256=entry['scenario_sha256'],ordinal=ordinal)
    sid=entry['id']+f'_fc{ordinal:02}';folder.mkdir(parents=True)
    sp=folder/(sid+'.json');mp=folder/(sid+'_metadata.json');atomic_json(sp,scene);atomic_json(mp,meta)
    if sha256(sp) in known:raise ValueError('Known payload must never be captured again')
    candidate=copy.deepcopy(entry);candidate.update(id=sid,source_cell_id=sid,scenario=str(sp.resolve()),
        metadata=str(mp.resolve()),scenario_sha256=sha256(sp),scene_group_id=group,
        role=meta['spec']['role'],distance_bounds=meta['spec']['distance_bounds'])
    manifest=folder/'manifest.json';atomic_json(manifest,adaptive.fa.batch.campaign.subset_deck(deck,[candidate]))
    delta=flex.candidate_delta(original,meta) if original['runtime_actors'] else dict(background_only=True)
    return dict(id=entry['id'],ordinal=ordinal,group=group,entry=candidate,delta=delta,
        manifest=str(manifest),manifest_sha256=sha256(manifest),moving=bool(flex.movers(meta)),
        capture=str(folder.parent/'capture'))


def verify(a,deck,source,pools):
    if a['moving']:row=adaptive.verify(a,a['entry'],deck)
    else:
        if sha256(a['manifest'])!=a['manifest_sha256']:raise ValueError('Proposal manifest changed')
        adaptive.fa.check_placement(a['entry'],Path(a['manifest']),deck['config'])
        rows=adaptive.fa.batch.inspect_batch(Path(a['capture']),Path(a['manifest']))
        if len(rows)!=1:raise ValueError('Completion expects exactly one raw row')
        row=rows[0]
        if row['status']=='accepted_candidate':
            actual=read(Path(row['capture_root'])/'metadata.json');expected=read(a['entry']['metadata'])
            for key in ('spec','split','scene_group_id'):
                if actual[key]!=expected[key]:raise ValueError('Captured proposal identity changed')
            pa=lambda m:m['runtime_actors'][0]['data_id'] if m['runtime_actors'] else None
            if pa(actual)!=pa(expected):raise ValueError('Captured primary changed')
    if row['status']=='accepted_candidate':
        meta=read(Path(row['capture_root'])/'metadata.json')
        if source['runtime_actors']:
            if not compatible(source,meta,pools):raise ValueError('Captured family/motion/split contract changed')
        elif meta['runtime_actors'] or meta['split']!=source['split']:raise ValueError('Background replacement changed context')
    return row


def run_child(a,output,heartbeat):
    root=Path(a['capture']);manifest=Path(a['manifest']);script='run_base5_capture_campaign_spatial.py' if a['moving'] else 'run_base5_capture_optimized.py'
    if root.exists():
        # Native completed verification can be retried, but never reload a
        # partial or unsafe child just because the parent was resumed.
        safe=(read(root/'status.json')['stage'] in TERMINAL) if a['moving'] and (root/'status.json').exists() else (root/'shutdown_safety.json').exists() and (root/'ledger.jsonl').exists()
        if safe:return 0
        raise RuntimeError('Interrupted child needs safety inspection; no automatic reload')
    command=[sys.executable,str(Path(__file__).with_name(script)),'--manifest',str(manifest),'--output',str(root),'--limit','1']
    if not a['moving']:command.extend(['--shared-ros-batch','--setup-profile','static-ready'])
    with (root.parent/'runner.log').open('a') as log:
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        return adaptive.fa.batch.wait_child(proc,root,output/'STOP',heartbeat,800 if a['moving'] else 120)


def execute(a):
    if not 1<=a.attempts<=5 or not 30<=a.time_budget_sec<=3600:raise ValueError('Finite attempt/time limits required')
    if a.output.exists() and not a.resume:raise FileExistsError('Explicit resume or new output required')
    if a.output.resolve()==a.baseline.resolve():raise ValueError('Never write into parent')
    a.output.mkdir(parents=True,exist_ok=True);started=time.monotonic();state=None;locks=[];prior=0.
    try:
        for path in (a.output/'.completion.lock',a.baseline/'.batch.lock'):
            lock=path.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(lock)
        deck=read(a.manifest)
        if not deck['config'].get('fresh_geometry_policy'):raise ValueError('Fresh environment manifest required')
        baseline,deficits,elapsed=verify_parent(a.manifest,a.baseline)
        contract=dict(policy=POLICY,source_sha256=sha256(a.manifest),baseline=str(a.baseline.resolve()),
            parent_hashes={n:sha256(a.baseline/n) for n in ('selected_ledger.jsonl','quota.json','batch_contract.json','fresh_runtime_contract.json')},
            code_hashes={name:sha256(Path(__file__).with_name(name)) for name in
                (Path(__file__).name,'base5_spatial_diversity.py','run_base5_flexible_completion.py','run_base5_adaptive_completion.py','run_base5_capture_optimized.py','run_base5_capture_campaign_spatial.py')},
            capture_contract=adaptive.fa.batch.campaign.contract(a.manifest,list(range(len(deck['scenes']))),2),attempts=a.attempts)
        sp=a.output/'state.json'
        if sp.exists():
            state=read(sp)
            if state['contract']!=contract:raise ValueError('Completion resume contract changed')
            if state['stage']=='halted':raise ValueError('Inspect safety before any halted completion recovery')
            prior=state['elapsed_sec']
        else:
            state=dict(contract=contract,stage='prepared',baseline_selected=baseline,deficits=deficits,
                baseline_elapsed_sec=elapsed,source_order=[e['id'] for e in deck['scenes']],attempts=[],unavailable={},elapsed_sec=0.,current=None)
        lookup={e['id']:e for e in deck['scenes']}
        def save(stage,error=None):
            state.update(stage=stage,error=error,current=state['current'] if stage=='running' else None,
                elapsed_sec=prior+time.monotonic()-started if a.run else prior)
            atomic_json(sp,state);rows,missing=merged(state)
            body=''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows);ledger=a.output/'selected_ledger.jsonl'
            if not ledger.exists() or ledger.read_text()!=body:
                tmp=a.output/'.selected.tmp';tmp.write_text(body);tmp.replace(ledger)
            atomic_json(a.output/'quota.json',dict(target=len(deck['scenes']),accepted=len(rows),deficit=len(missing),pending=0,
                cells=[dict(id=i,reason=state['unavailable'].get(i,'bounded_family_completion_review')) for i in missing]))
            atomic_json(a.output/'status.json',dict(stage=stage,error=error,mode='failure_aware_replenishment',flexible_completion=True,
                requested=len(deck['scenes']),finished=len(deck['scenes']),accepted_candidate=len(rows),review=len(missing),
                replenished=len(rows)-len(baseline),remaining_deficit=len(missing),initial_accepted=len(baseline),
                elapsed_sec=elapsed+state['elapsed_sec'],replenishment_elapsed_sec=state['elapsed_sec'],eta_sec=0.,updated_at=time.time(),
                current=state['current'],attempt_cap=a.attempts,attempts=len(state['attempts']),
                accepted_bins=dict(Counter(json.dumps([r.get('family'),r.get('distance_bounds'),r.get('role')]) for r in rows))))
            adopted=[]
            for row in rows:
                ident=row['source_cell_id'];entry=copy.deepcopy(lookup[ident]);entry.update(scenario=row['scenario'],scenario_sha256=row['scenario_sha256'],
                    metadata=str(Path(row['capture_root'])/'metadata.json'))
                meta=read(entry['metadata']);entry.update(role=meta['spec']['role'],distance_bounds=meta['spec']['distance_bounds'],scene_group_id=meta['scene_group_id'])
                adopted.append(entry)
            atomic_json(a.output/'adopted_manifest.json',adaptive.fa.batch.campaign.subset_deck(deck,adopted))
            atomic_json(a.output/'replacement_mapping.json',dict(policy=POLICY,raw_preserved=True,exact_original_quota_preserved=False,
                replacements=[dict(id=v['id'],ordinal=v['ordinal'],delta=v['delta'],status=(v.get('result') or {}).get('status')) for v in state['attempts']]))
            return missing
        # Completed resume is strictly read-only, including journal/elapsed.
        if state['stage'] in TERMINAL:
            rows,_=merged(state)
            if rows!=load_ledger(a.output/'selected_ledger.jsonl'):raise ValueError('Final completion selection changed')
            ctx=spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,spatial.gen.geo.DEFAULT_MGEO)
            pools=pool_map(ctx)
            for v in state['attempts']:
                if v.get('result') is None:raise ValueError('Terminal output has unresolved child')
                if verify(v,deck,read(lookup[v['id']]['metadata']),pools)!=v['result']:raise ValueError('Completed raw evidence changed')
            return 0
        missing=save(state['stage'])
        if not a.run:return 0
        ctx=adaptive.prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,spatial.gen.geo.DEFAULT_MGEO),deck['config'])
        pools=pool_map(ctx)
        originals={i:read(lookup[i]['metadata']) for i in deficits};donors=[read(Path(r['capture_root'])/'metadata.json') for r in baseline]
        primary_counts=Counter(d['runtime_actors'][0]['data_id'] for d in donors if d['runtime_actors'])
        floor=deck['config'].get('coverage_policy',{}).get('minimum_accepted_visible_primary_per_asset',4)
        cal=adaptive.fa.normalized_sensor_calibration(adaptive.fa.SENSOR)
        known={e['scenario_sha256'] for e in deck['scenes']}|{r['scenario_sha256'] for r in baseline}|{v['entry']['scenario_sha256'] for v in state['attempts']}
        for v in state['attempts']:
            if v.get('result') is not None and verify(v,deck,originals[v['id']],pools)!=v['result']:raise ValueError('Past completion evidence changed')
            if (v.get('result') or {}).get('status')=='accepted_candidate':
                m=read(Path(v['result']['capture_root'])/'metadata.json')
                if m['runtime_actors']:primary_counts[m['runtime_actors'][0]['data_id']]+=1
        def heartbeat(root):
            # Do not rebuild the full 5K ledger/adopted manifest every 0.5s.
            status=read(a.output/'status.json');status.update(updated_at=time.time(),current=str(root),
                elapsed_sec=elapsed+prior+time.monotonic()-started)
            atomic_json(a.output/'status.json',status)
        for ordinal in range(1,a.attempts+1):
            for ident in deficits:
                if ident not in missing or ident in state['unavailable']:continue
                v=next((v for v in state['attempts'] if v['id']==ident and v['ordinal']==ordinal),None)
                if v and v.get('result') is not None:continue
                if time.monotonic()-started>=a.time_budget_sec or any((a.output/n).exists() for n in ('STOP','PAUSE')):
                    save('paused');return 0
                state['current']=ident;save('running')
                if v is None:
                    options=floor_donors(originals[ident],donors,primary_counts,floor)
                    try:v=prepare(lookup[ident],originals[ident],options,deck,ctx,cal,ordinal,a.output/'cells'/ident/f'attempt_{ordinal}'/'placement',known,pools)
                    except ValueError as exc:
                        state['unavailable'][ident]=str(exc);missing=save('running');continue
                    state['attempts'].append(v);known.add(v['entry']['scenario_sha256']);save('running')
                if run_child(v,a.output,heartbeat):raise RuntimeError('Completion child halted; no next load')
                v['result']=verify(v,deck,originals[ident],pools)
                if v['result']['status']=='accepted_candidate':
                    m=read(Path(v['result']['capture_root'])/'metadata.json')
                    if m['runtime_actors']:primary_counts[m['runtime_actors'][0]['data_id']]+=1
                missing=save('running')
                print(dict(cell=ident,attempt=ordinal,status=v['result']['status'],deficit=len(missing)),flush=True)
        save('complete_with_deficit' if missing else 'complete');return 0
    except BaseException as exc:
        if state is not None and 'save' in locals():save('halted',str(exc))
        else:atomic_json(a.output/'preflight_error.json',dict(error=str(exc)))
        if isinstance(exc,KeyboardInterrupt):return 130
        if not isinstance(exc,Exception):raise
        print(str(exc),flush=True);return 1
    finally:
        for lock in reversed(locks):lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--run',action='store_true');p.add_argument('--resume',action='store_true')
    p.add_argument('--attempts',type=int,default=3);p.add_argument('--time-budget-sec',type=int,default=1800)
    raise SystemExit(execute(p.parse_args()))
