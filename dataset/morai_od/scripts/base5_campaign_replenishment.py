"""Post-completion quota replenishment with durable, bounded, per-cell evidence.

The original batch remains immutable. This journal is authoritative for merged
selection; publication is idempotent after a crash between atomic file renames.
"""
import copy
import fcntl
import json
import os
from pathlib import Path
import time

import run_base5_batch_campaign as batch
from base5_capture_safety import atomic_json, load_ledger, sha256, status_row


COMPLETED=('complete','complete_with_deficit')


def read(path):
    return json.loads(Path(path).read_text())


def cell_contract(entry):
    meta=read(entry['metadata'])
    actors=meta.get('runtime_actors',[])
    return dict(spec=meta['spec'],split=meta['split'],
        primary=actors[0]['data_id'] if actors else None,
        group=meta.get('scene_group_id',entry.get('scene_group_id',entry['id'])))


def check_placement(source_entry,manifest,source_config):
    deck=read(manifest)
    if deck['config']!=source_config or len(deck['scenes'])!=1:
        raise ValueError('Replenishment changed environment/config or cell count')
    expected=cell_contract(source_entry);entry=deck['scenes'][0];actual=cell_contract(entry)
    if actual!=expected or entry.get('source_cell_id')!=source_entry['id']:
        raise ValueError('Replenishment changed asset/spec/split/group identity')
    if sha256(entry['scenario'])!=entry['scenario_sha256']:
        raise ValueError('Replenishment scenario changed')


def prepare(source_entry,source_deck,previous,folder):
    """Fresh geometry, never re-run a known collision's original payload."""
    manifest=folder/'placement/manifest.json'
    if not manifest.exists():
        seed=folder/'seed_manifest.json'
        if not seed.exists():
            deck=read(previous) if previous else batch.campaign.subset_deck(source_deck,[source_entry])
            entry=deck['scenes'][0]
            entry['source_cell_id']=source_entry['id']
            atomic_json(seed,deck)
        batch.campaign.loop.replacement(seed,manifest.parent)
        # Set the group before any capture. Nested quality replacements retain
        # it too. Never modify already captured metadata or raw annotations.
        deck=read(manifest);entry=deck['scenes'][0];meta=read(entry['metadata'])
        meta['scene_group_id']=cell_contract(source_entry)['group']
        atomic_json(Path(entry['metadata']),meta)
    check_placement(source_entry,manifest,source_deck['config'])
    return manifest


def result(manifest,child,source_entry):
    batch.verify_recovery(manifest,0,child)
    entry=read(manifest)['scenes'][0]
    rows=load_ledger(child/'selected_ledger.jsonl')
    next_source=child/entry['id']/'replacement_1/manifest.json'
    next_source=next_source if next_source.exists() else manifest
    if rows:
        row=copy.deepcopy(rows[0]);meta=read(Path(row['capture_root'])/'metadata.json')
        expected=cell_contract(source_entry)
        actors=meta.get('runtime_actors',[])
        actual=dict(spec=meta['spec'],split=meta['split'],
            primary=actors[0]['data_id'] if actors else None,
            group=meta.get('scene_group_id',expected['group']))
        if actual!=expected:raise ValueError('Selected replacement changed cell identity')
        row['source_cell_id']=source_entry['id']
        row['scene_group_id']=expected['group']
        row['replenishment']=dict(manifest=str(manifest.resolve()),
            manifest_sha256=sha256(manifest),campaign=str(child.resolve()),
            original_scenario_sha256=source_entry['scenario_sha256'])
    else:row=None
    return dict(selected=row,reasons=read(child/'quota.json')['cells'],
        next_source=str(next_source.resolve()),next_source_sha256=sha256(next_source))


def merged(state,source_deck,indices):
    selected={r['source_cell_id']:copy.deepcopy(r) for r in state['baseline']['selected']}
    deficits={r['id']:copy.deepcopy(r) for r in state['baseline']['deficits']}
    for attempt in state['attempts']:
        if attempt.get('result') is None:continue
        ident=attempt['id'];row=attempt['result']['selected']
        if row:
            if ident in selected:raise ValueError('Replenishment selected the same cell twice')
            selected[ident]=copy.deepcopy(row);deficits.pop(ident)
        else:deficits[ident]['replenishment_reasons']=attempt['result']['reasons']
    ordered=[selected[source_deck['scenes'][i]['id']] for i in indices if source_deck['scenes'][i]['id'] in selected]
    if len(ordered)+len(deficits)!=len(indices):raise ValueError('Quota reconstruction mismatch')
    return ordered,deficits


def publish(output,state,source_deck,indices):
    selected,deficits=merged(state,source_deck,indices)
    ledger=output/'selected_ledger.jsonl'
    body=''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected)
    # Do not replace a correct ledger during verification (mtime/inode stable).
    if not ledger.exists() or ledger.read_text()!=body:
        temp=output/'.replenished_selected.tmp'
        with temp.open('w') as handle:
            handle.write(body);handle.flush();os.fsync(handle.fileno())
        temp.replace(ledger)
    atomic_json(output/'quota.json',dict(target=len(indices),accepted=len(selected),
        deficit=len(deficits),pending=0,cells=list(deficits.values())))
    atomic_json(output/'deficit_manifest.json',batch.campaign.subset_deck(source_deck,
        [source_deck['scenes'][i] for i in indices if source_deck['scenes'][i]['id'] in deficits]))
    initial=len(state['baseline']['selected']);filled=len(selected)-initial
    finished_attempts=sum(a.get('result') is not None for a in state['attempts'])
    elapsed=state['baseline']['elapsed_sec']+state['elapsed_sec']
    eta=(state['elapsed_sec']/finished_attempts*len(deficits)) if finished_attempts and state['stage']=='replenishing' else 0
    bins={}
    for row in selected:
        key=json.dumps([row.get('family'),row.get('distance_bounds'),row.get('role')])
        bins[key]=bins.get(key,0)+1
    atomic_json(output/'status.json',dict(stage=state['stage'],mode='replenishment',
        error=state.get('error'),requested=len(indices),finished=len(indices),
        accepted_candidate=len(selected),review=len(deficits),failed=int(state['stage']=='halted'),
        initial_accepted=initial,replenished=filled,remaining_deficit=len(deficits),
        replenish_round=state.get('round',0),replenish_round_cap=state['round_cap'],
        replenishment_attempts=finished_attempts,updated_at=time.time(),elapsed_sec=elapsed,
        replenishment_elapsed_sec=state['elapsed_sec'],eta_sec=eta,current=state.get('current'),
        checkpoint_finished=len(indices),checkpoint_only=True,accepted_bins=bins,
        results=[status_row(r) for r in selected] if len(indices)<=64 else []))


def execute(source,indices,output,chunk_size,resume,bulk,replenish,rounds,stop_after=0):
    if not resume:raise ValueError('Replenishment campaign requires --resume')
    if not 1<=rounds<=3:raise ValueError('Replenishment total round cap must be 1–3')
    if not output.exists():raise ValueError('Complete initial collection before --replenish')
    lock=(output/'.batch.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    state_path=output/'replenishment/state.json';state=None;start=time.monotonic();prior=0;validated=False
    deck=read(source);status=read(output/'status.json');source_sha=sha256(source)
    def save(stage,error=None,current=None):
        state.update(stage=stage,error=error,current=str(current) if current else None,
            elapsed_sec=prior+time.monotonic()-start)
        atomic_json(state_path,state)
        publish(output,state,deck,indices)
    try:
        if status.get('stage')=='halted':raise ValueError('Unsafe halted collection is not replenishable')
        if state_path.exists():
            state=read(state_path);prior=state['elapsed_sec']
            if state['source_sha256']!=source_sha or state['indices']!=indices:
                raise ValueError('Replenishment source/indices changed')
            if state['driver_sha256']!=sha256(__file__):raise ValueError('Replenishment runtime changed')
            if state['stage']=='halted':raise ValueError('Halted replenishment requires manual safety recovery')
            # Heartbeats are lightweight and can survive a power loss newer
            # than the last cell commit. Never count offline waiting time.
            prior=max(prior,status.get('replenishment_elapsed_sec',prior))
        elif status['stage'] not in COMPLETED:
            raise ValueError('Finish initial campaign with ordinary --resume first')
        # Rebuild the original completion evidence read-only, under this lock.
        base={}
        code=batch.execute(source,indices,output,chunk_size,True,bulk=bulk,
            _verification=base,_lock_held=True)
        if code or not base:raise ValueError('Initial completion evidence failed verification')
        ordered=[base['selected'][deck['scenes'][i]['id']] for i in indices if deck['scenes'][i]['id'] in base['selected']]
        deficits=list(base['deficits'].values())
        if state is None:
            if load_ledger(output/'selected_ledger.jsonl')!=ordered or read(output/'quota.json')['deficit']!=len(deficits):
                raise ValueError('Initial selection/quota does not match capture evidence')
            state=dict(schema=1,source_sha256=source_sha,indices=indices,
                driver_sha256=sha256(__file__),round_cap=rounds,stage=status['stage'],
                elapsed_sec=0,round=0,attempts=[],
                baseline=dict(selected=ordered,deficits=deficits,elapsed_sec=status['elapsed_sec']))
            prior=0
            state_path.parent.mkdir(exist_ok=True)
            atomic_json(state_path,state)
        elif state['baseline']['selected']!=ordered or state['baseline']['deficits']!=deficits:
            raise ValueError('Original completion evidence changed')
        # Validate completed journal entries before repairing canonical snapshots.
        # Pending children resume in place, but successful ones are never run again.
        for attempt in state['attempts']:
            entry=deck['scenes'][attempt['index']]
            if attempt['id']!=entry['id'] or not 1<=attempt['round']<=3:
                raise ValueError('Attempt does not refer to its original quota cell')
            if sum(a['id']==attempt['id'] and a['round']==attempt['round'] for a in state['attempts'])!=1:
                raise ValueError('Duplicate attempt identity')
            manifest=Path(attempt['manifest'])
            if sha256(manifest)!=attempt['manifest_sha256']:raise ValueError('Attempt manifest changed')
            if attempt.get('kind')=='placement_unavailable':
                if attempt['result']!=dict(selected=None,reasons=['same_cell_placement_unavailable'],
                        next_source=str(manifest.resolve()),next_source_sha256=sha256(manifest)):
                    raise ValueError('Placement failure evidence changed')
                continue
            check_placement(entry,manifest,deck['config'])
            if attempt.get('result') is not None:
                if result(manifest,Path(attempt['child']),entry)!=attempt['result']:
                    raise ValueError('Completed replenishment evidence changed')
        merged(state,deck,indices)
        validated=True
        publish(output,state,deck,indices)
        if not replenish:return 0  # Ordinary --resume is strictly non-capturing.
        _,remaining=merged(state,deck,indices)
        if not remaining:return 0
        if rounds<state['round_cap']:raise ValueError('Do not lower the cumulative round cap')
        if all(sum(a['id']==ident and a.get('result') is not None for a in state['attempts'])>=rounds for ident in remaining):
            return 0  # Exhausted command is also a verification, not a new wave.
        state['round_cap']=rounds
        processed=0
        for ordinal in range(1,rounds+1):
            _,remaining=merged(state,deck,indices)
            for index in indices:
                entry=deck['scenes'][index];ident=entry['id']
                if ident not in remaining:continue
                attempt=next((a for a in state['attempts'] if a['id']==ident and a['round']==ordinal),None)
                if attempt and attempt.get('result') is not None:continue
                if (output/'STOP').exists() or (output/'PAUSE').exists():save('paused');return 0
                folder=state_path.parent/f'round_{ordinal:02d}'/ident
                folder.mkdir(parents=True,exist_ok=True)
                state['round']=ordinal
                if attempt is None:
                    history=[a for a in state['attempts'] if a['id']==ident and a.get('result') is not None]
                    previous=Path(history[-1]['result']['next_source']) if history else None
                    if history and sha256(previous)!=history[-1]['result']['next_source_sha256']:
                        raise ValueError('Previous failed placement changed')
                    try:manifest=prepare(entry,deck,previous,folder)
                    except ValueError as exc:
                        if str(exc)!='Same-cell/split replacement unavailable':raise
                        # No live operation took place. A bounded generation
                        # miss fills neither quota nor raw, but need not halt MR.
                        seed=folder/'seed_manifest.json'
                        attempt=dict(id=ident,index=index,round=ordinal,kind='placement_unavailable',
                            manifest=str(seed.resolve()),manifest_sha256=sha256(seed),child=None,
                            result=dict(selected=None,reasons=['same_cell_placement_unavailable'],
                                next_source=str(seed.resolve()),next_source_sha256=sha256(seed)))
                        state['attempts'].append(attempt);save('replenishing');processed+=1
                        if (output/'PAUSE').exists() or (stop_after and processed>=stop_after):
                            save('paused');return 0
                        continue
                    attempt=dict(id=ident,index=index,round=ordinal,manifest=str(manifest.resolve()),
                        manifest_sha256=sha256(manifest),child=str((folder/'campaign').resolve()),result=None)
                    state['attempts'].append(attempt)
                    save('replenishing')  # Durable pending identity before live work.
                manifest=Path(attempt['manifest']);child=Path(attempt['child'])
                def heartbeat(root):
                    # A 5K ledger can be tens of MiB. Do not serialize/fsync it
                    # (or its baseline journal) every half second during capture.
                    progress=read(output/'status.json')
                    progress.update(stage='replenishing',current=str(root),updated_at=time.time(),
                        replenishment_elapsed_sec=prior+time.monotonic()-start,
                        elapsed_sec=state['baseline']['elapsed_sec']+prior+time.monotonic()-start)
                    atomic_json(output/'status.json',progress)
                if batch.recover(manifest,0,child,output/'STOP',heartbeat):
                    raise RuntimeError('Replenishment child halted; no next load')
                attempt['result']=result(manifest,child,entry)
                save('replenishing');processed+=1
                if (output/'PAUSE').exists() or (stop_after and processed>=stop_after):
                    save('paused');return 0
        _,remaining=merged(state,deck,indices)
        save('complete_with_deficit' if remaining else 'complete')
        return 0
    except BaseException as exc:
        # Preflight contract errors must not rewrite a valid historical ledger.
        # After initialization, evidence/safety errors are fatal, never a miss.
        if state is not None and validated:
            save('halted',str(exc))
        else:atomic_json(output/'replenishment_preflight_error.json',dict(error=str(exc),time=time.time()))
        if isinstance(exc,KeyboardInterrupt):return 130
        if isinstance(exc,(SystemExit,GeneratorExit)):raise
        return 1
    finally:lock.close()
