#!/usr/bin/env python3
"""Repair lost animated appearance in a frozen adaptive selection, without rewriting it."""
import argparse
from collections import Counter
import copy
import fcntl
import importlib.util
import json
from pathlib import Path
import time

import run_base5_adaptive_completion as ac


def motion_signature(meta):
    actors=meta['runtime_actors']
    return dict(primary=actors[0]['motion'] if actors else None,
        movers=dict(Counter(a['motion'] for a in actors if a['motion'] not in ('static','standing'))))


def motion_matches(source,actual):
    expected=motion_signature(source)
    observed=motion_signature(actual)
    # Random secondary identities may ADD movers. Do not forbid this legitimate
    # diversity, but never lose the intended moving modes or change primary gait.
    return not expected['movers'] or (observed['primary']==expected['primary'] and
        all(observed['movers'].get(mode,0)>=count for mode,count in expected['movers'].items()))


def partition(rows,lookup):
    kept=[];excluded=[]
    for row in rows:
        ident=row['source_cell_id'];source=ac.fa.read(lookup[ident]['metadata'])
        actual=ac.fa.read(Path(row['capture_root'])/'metadata.json')
        if motion_matches(source,actual):kept.append(copy.deepcopy(row))
        else:excluded.append(dict(id=ident,original_selected=row,
            expected=motion_signature(source),actual=motion_signature(actual),
            reason='actual_motion_contract_changed'))
    return kept,excluded


def prepare(entry,deck,ctx,calibration,history,ordinal,folder,heartbeat):
    source=ac.fa.read(entry['metadata']);original_build=ac.fa.gen.build
    def filtered_build(*args,**kwargs):
        scene,meta=original_build(*args,**kwargs)
        if not motion_matches(source,meta):raise ValueError('Animated appearance signature changed')
        return scene,meta
    ac.fa.gen.build=filtered_build
    try:return ac.prepare(entry,deck,ctx,calibration,history,ordinal,folder,'prepared_motion',heartbeat)
    finally:ac.fa.gen.build=original_build


def verify(attempt,entry,deck):
    row=ac.verify(attempt,entry,deck)
    if row['status']=='accepted_candidate':
        source=ac.fa.read(entry['metadata']);actual=ac.fa.read(Path(row['capture_root'])/'metadata.json')
        if not motion_matches(source,actual):
            return dict(status='review',reasons=['actual_motion_contract_changed'],observed_selected=row)
    return row


def execute(parent,output,run=False,resume=False,max_attempts=3):
    if not 1<=max_attempts<=3:raise ValueError('Bounded repair attempts required')
    if output.exists() and not resume:raise FileExistsError('Existing repair requires --resume')
    output.mkdir(parents=True,exist_ok=True);locks=[];state=None;started=time.monotonic()
    try:
        parent_state=ac.fa.read(parent/'state.json');parent_status=ac.fa.read(parent/'status.json')
        if parent_status['stage'] not in ('complete','complete_with_deficit'):raise ValueError('Parent must be terminal')
        if parent_state['contract']['code_sha256']!=ac.fa.sha256(ac.__file__):raise ValueError('Frozen parent driver changed')
        for path in (output/'.repair.lock',parent/'.adaptive.lock',
                     Path(parent_state['contract']['seed'])/'.supplement.lock',
                     Path(parent_state['contract']['baseline'])/'.batch.lock'):
            lock=path.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(lock)
        source=parent.parent/'environment_suite/sunny13/manifest.json';deck=ac.fa.read(source)
        if ac.fa.sha256(source)!=parent_state['contract']['source_sha256']:raise ValueError('Source changed')
        if not all(ac.fa.sha256(Path(parent_state['contract']['seed'])/n)==v
                   for n,v in parent_state['contract']['seed_hashes'].items()):raise ValueError('Frozen seed changed')
        if ac.fa.batch.campaign.contract(source,list(range(len(deck['scenes']))),2)!=parent_state['contract']['capture_contract']:
            raise ValueError('Frozen capture runtime changed')
        lookup={e['id']:e for e in deck['scenes']}
        previous=parent.parent/'collection_5k_sunny13_motion_contract_v138'
        previous_state=None;previous_contract=None
        if previous.exists():
            lock=(previous/'.repair.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(lock)
            previous_state=ac.fa.read(previous/'state.json')
            if previous_state['stage'] not in ('paused','complete','complete_with_deficit') or any(not a.get('result') for a in previous_state['attempts']):
                raise ValueError('Previous repair must finish safely')
            frozen=previous/'frozen_repair_driver.py'
            if ac.fa.sha256(frozen)!=previous_state['contract']['driver_sha256']:raise ValueError('Previous repair snapshot changed')
            if previous_state['contract']['parent_hashes']!={n:ac.fa.sha256(parent/n) for n in ('state.json','selected_ledger.jsonl','quota.json')}:
                raise ValueError('Previous repair parent changed')
            spec=importlib.util.spec_from_file_location('base5_frozen_motion_repair',frozen)
            legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
            for attempt in previous_state['attempts']:
                if legacy.verify(attempt,lookup[attempt['id']],deck)!=attempt['result']:raise ValueError('Previous repair evidence changed')
            previous_contract=dict(path=str(previous),driver_sha256=ac.fa.sha256(frozen),
                hashes={n:ac.fa.sha256(previous/n) for n in ('state.json','selected_ledger.jsonl','quota.json')})
        contract=dict(schema=1,parent=str(parent.resolve()),
            parent_hashes={n:ac.fa.sha256(parent/n) for n in ('state.json','selected_ledger.jsonl','quota.json')},
            source_sha256=ac.fa.sha256(source),driver_sha256=ac.fa.sha256(__file__),max_attempts=max_attempts,
            capture_contract=parent_state['contract']['capture_contract'],
            previous_repair=previous_contract,
            policy='Retain primary gait and at least source moving-mode counts; added secondary movers allowed; preserve parent raw')
        sp=output/'state.json'
        if sp.exists():
            state=ac.fa.read(sp)
            if state['contract']!=contract:raise ValueError('Repair resume contract changed')
            if state['stage']=='halted':raise ValueError('Halted repair requires safety inspection')
        else:
            for a in parent_state['attempts']:
                if not a.get('result') or ac.verify(a,lookup[a['id']],deck)!=a['result']:
                    raise ValueError('Parent attempt evidence changed')
            parent_rows=ac.fa.load_ledger(parent/'selected_ledger.jsonl')
            indexed={r['source_cell_id']:r for r in parent_rows}
            expected=copy.deepcopy(parent_state['baseline_selected'])
            for a in parent_state['attempts']:
                if a['result']['status']!='accepted_candidate':continue
                row=copy.deepcopy(a['result']);row.update(source_cell_id=a['id'],
                    scene_group_id=ac.fa.cell_contract(lookup[a['id']])['group'],
                    adaptive_completion=dict(manifest=a['manifest'],strategy=a['strategy'],round=a['round']))
                expected.append(row)
            if len(indexed)!=len(parent_rows) or len(expected)!=len(parent_rows) or any(indexed.get(r['source_cell_id'])!=r for r in expected):
                raise ValueError('Parent canonical selection does not match evidence')
            kept,excluded=partition(parent_rows,lookup)
            latest={a['id']:a['result'] for a in parent_state['attempts']}
            original_reasons={c['id']:sorted(ac.reasons(latest[c['id']].get('audit',{}).get('review_reasons',latest[c['id']].get('reasons',[]))))
                for c in ac.fa.read(parent/'quota.json')['cells'] if c['id'] in latest}
            state=dict(contract=contract,stage='prepared',baseline_selected=kept,excluded=excluded,
                attempts=[],elapsed_sec=0.,parent_elapsed_sec=parent_status['elapsed_sec'],current=None,
                original_deficit_reasons=original_reasons)
            ac.fa.atomic_json(sp,state)
        prior=state['elapsed_sec']
        for a in state['attempts']:
            if a.get('result') and verify(a,lookup[a['id']],deck)!=a['result']:
                raise ValueError('Repair attempt evidence changed')
        def save(stage,error=None):
            state.update(stage=stage,error=error,elapsed_sec=prior+time.monotonic()-started if run else prior)
            ac.fa.atomic_json(sp,state)
            selected=copy.deepcopy(state['baseline_selected']);used={r['source_cell_id'] for r in selected}
            for a in state['attempts']:
                if not a.get('result') or a['result']['status']!='accepted_candidate':continue
                if a['id'] in used:raise ValueError('Duplicate repair selection')
                row=copy.deepcopy(a['result']);row.update(source_cell_id=a['id'],motion_contract_repair=True)
                selected.append(row);used.add(a['id'])
            order={e['id']:i for i,e in enumerate(deck['scenes'])};selected.sort(key=lambda r:order[r['source_cell_id']])
            body=''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected)
            ledger=output/'selected_ledger.jsonl'
            if not ledger.exists() or ledger.read_text()!=body:
                tmp=output/'.selected.tmp';tmp.write_text(body);tmp.replace(ledger)
            missing=[e for e in deck['scenes'] if e['id'] not in used]
            ac.fa.atomic_json(output/'quota.json',dict(target=len(deck['scenes']),accepted=len(selected),
                deficit=len(missing),pending=0,cells=[dict(id=e['id'],reasons=state['original_deficit_reasons'].get(e['id'],['actual_motion_contract_changed'])) for e in missing]))
            ac.fa.atomic_json(output/'deficit_manifest.json',ac.fa.batch.campaign.subset_deck(deck,missing))
            ac.fa.atomic_json(output/'exclusions.json',dict(parent=str(parent),cells=state['excluded'],raw_preserved=True))
            bins=Counter(json.dumps([r.get('family'),r.get('distance_bounds'),r.get('role')]) for r in selected)
            ac.fa.atomic_json(output/'status.json',dict(stage=stage,error=error,mode='failure_aware_replenishment',
                requested=len(deck['scenes']),finished=len(deck['scenes']),accepted_candidate=len(selected),review=len(missing),
                remaining_deficit=len(missing),initial_accepted=len(state['baseline_selected']),
                replenished=len(selected)-len(state['baseline_selected']),elapsed_sec=state['parent_elapsed_sec']+state['elapsed_sec'],
                replenishment_elapsed_sec=state['elapsed_sec'],updated_at=time.time(),current=state['current'],eta_sec=0,
                accepted_bins=dict(bins),motion_contract_repair=True,repair_attempts=len(state['attempts']),repair_attempt_cap=max_attempts))
            return used
        terminal=state['stage'] in ('complete','complete_with_deficit')
        if terminal:
            # Read-only verification must not restart the repair or accrue time.
            return 0
        used=save(state['stage'])
        if not run:return 0
        ctx=ac.prewarm(ac.fa.gen.context(ac.fa.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,ac.fa.gen.geo.DEFAULT_MGEO),deck['config'])
        calibration=ac.fa.normalized_sensor_calibration(ac.fa.SENSOR)
        def heartbeat(root=None):
            if root:state['current']=str(root)
            status=ac.fa.read(output/'status.json');status.update(updated_at=time.time(),current=state['current'])
            ac.fa.atomic_json(output/'status.json',status)
        for exclusion in state['excluded']:
            ident=exclusion['id']
            history=ac.fa.history_for(Path(parent_state['contract']['baseline']),lookup[ident])
            history += [ac.fa.read(ac.fa.read(a['manifest'])['scenes'][0]['metadata']) for a in parent_state['attempts'] if a['id']==ident]
            if previous_state:
                history += [ac.fa.read(ac.fa.read(a['manifest'])['scenes'][0]['metadata']) for a in previous_state['attempts'] if a['id']==ident]
            history += [ac.fa.read(ac.fa.read(a['manifest'])['scenes'][0]['metadata']) for a in state['attempts'] if a['id']==ident]
            for ordinal in range(1,max_attempts+1):
                if ident in used:break
                a=next((a for a in state['attempts'] if a['id']==ident and a['ordinal']==ordinal),None)
                if a and a.get('result'):continue
                if (output/'PAUSE').exists() or (output/'STOP').exists():save('paused');return 0
                state['current']=ident;save('running')
                if a is None:
                    folder=output/'cells'/ident/f'candidate_{ordinal}'
                    manifest=prepare(lookup[ident],deck,ctx,calibration,history,100+ordinal,folder/'placement',heartbeat)
                    a=dict(id=ident,ordinal=ordinal,manifest=str(manifest),manifest_sha256=ac.fa.sha256(manifest),
                        moving=True,capture=str(folder/'capture'),result=None)
                    state['attempts'].append(a);save('running')
                code=ac.run_child(a,heartbeat);row=verify(a,lookup[ident],deck)
                if code:raise RuntimeError('Repair child unsafe/incomplete; no next load')
                a['result']=row;used=save('running')
                history.append(ac.fa.read(ac.fa.read(a['manifest'])['scenes'][0]['metadata']))
                print(json.dumps(dict(cell=ident,attempt=ordinal,status=row['status'],reasons=row.get('reasons',[]))),flush=True)
        state['current']=None
        save('complete' if len(used)==len(deck['scenes']) else 'complete_with_deficit');return 0
    except BaseException as exc:
        if state is not None and 'save' in locals():save('halted',str(exc))
        else:ac.fa.atomic_json(output/'preflight_error.json',dict(error=str(exc)))
        if isinstance(exc,KeyboardInterrupt):return 130
        if not isinstance(exc,Exception):raise
        print(str(exc));return 1
    finally:
        for lock in reversed(locks):lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=ac.DEFAULT_ROOT)
    p.add_argument('--resume',action='store_true');p.add_argument('--run',action='store_true')
    a=p.parse_args();raise SystemExit(execute(a.root/'collection_5k_sunny13_adaptive_v137',
        a.root/'collection_5k_sunny13_motion_contract_v139',a.run,a.resume))
