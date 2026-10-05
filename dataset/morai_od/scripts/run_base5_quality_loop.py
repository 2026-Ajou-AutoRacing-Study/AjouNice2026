#!/usr/bin/env python3
"""Bounded Base5 quality recovery: capture -> Z/probe/replacement -> recheck."""
import argparse
import copy
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import time
import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256
from prepare_base5_ground_retry import prepare as height_retry
from prepare_base5_empty_ground_probe import prepare as empty_probe
from prepare_base5_probed_retry import prepare as probed_retry


def replacement(manifest,output):
    m=gen.geo.load_json(manifest);old=m['scenes'][0];meta=gen.geo.load_json(Path(old['metadata']))
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    for retry in range(meta['retry']+1,meta['retry']+513):
        try:scene,new=gen.build(meta['spec'],m['config'],ctx,retry)
        except ValueError:continue
        if new['split']!=meta['split']:continue
        if new['runtime_actors'][0]['data_id']!=meta['runtime_actors'][0]['data_id']:raise ValueError('Replacement changed primary asset')
        break
    else:raise ValueError('Same-cell/split replacement unavailable')
    output.mkdir();sid=old['id']+'_replace';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
    new['replacement']=dict(source_cell_id=old.get('source_cell_id',old['id']),source_scenario_sha256=old['scenario_sha256'],
        policy='same primary asset, family, mission, motion request, role, distance cell and split; no GT rewriting')
    atomic_json(sp,scene);atomic_json(mp,new)
    old.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),motion=new['runtime_actors'][0]['motion'])
    old['source_cell_id']=new['replacement']['source_cell_id']
    atomic_json(output/'manifest.json',m);return output/'manifest.json'


def execute(source,indices,output):
    if output.exists():raise FileExistsError('New quality-loop output required; child captures retain resume support')
    if not 1<=len(indices)<=4 or len(set(indices))!=len(indices):raise ValueError('Pilot limited to 1–4 cells')
    output.mkdir(parents=True);lock=(output/'.quality.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    source_data=gen.geo.load_json(source);attempts=[];selected=[];unresolved=[]
    runner=Path(__file__).with_name('run_base5_capture.py')
    def save(stage,error=None):
        # Preserve usable selections even when a later child fails. Unsafe
        # children never enter this list or the conversion handoff.
        target=output/'selected_ledger.jsonl';temp=output/'.selected_ledger.tmp'
        temp.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected));temp.replace(target)
        atomic_json(output/'report.json',dict(stage=stage,error=error,source_manifest_sha256=sha256(source),
            attempts=attempts,selected=selected,unresolved=unresolved,bulk_capture=False))
        atomic_json(output/'status.json',dict(stage=stage,error=error,current=attempts[-1]['cell'] if attempts else None,
            requested=len(indices),finished=len(selected)+len(unresolved),accepted_candidate=len(selected),review=len(unresolved),
            failed=int(stage=='halted'),updated_at=time.time(),eta_sec=0,results=[]))
    def capture(manifest,root,cell,kind):
        if (output/'STOP').exists():raise RuntimeError('Quality-loop STOP requested')
        root.mkdir(parents=True,exist_ok=True)
        cmd=[sys.executable,str(runner),'--manifest',str(manifest),'--output',str(root),'--limit','1']
        with (root.parent/(root.name+'.log')).open('w') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+100.
            while child.poll() is None:
                if (output/'STOP').exists() or time.monotonic()>deadline:
                    (root/'STOP').touch();child.terminate()
                    try:child.wait(timeout=15.)
                    except subprocess.TimeoutExpired:raise RuntimeError('Child remains alive after interrupt; no further loads')
                    raise RuntimeError('Child interrupted; check shutdown evidence before any resume')
                time.sleep(.1)
        if child.returncode:
            safety=gen.geo.load_json(root/'shutdown_safety.json') if (root/'shutdown_safety.json').exists() else {}
            attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status='failed',
                reasons=['child_failed_or_exit_unsafe'],shutdown=safety))
            raise RuntimeError('Capture child failed; see '+str(root)+'.log')
        safety=gen.geo.load_json(root/'shutdown_safety.json')
        if not safety['park_verified']:raise RuntimeError('Exit Park unverified; no further loads')
        rows=[json.loads(l) for l in (root/'ledger.jsonl').read_text().splitlines() if l]
        row=rows[-1];attempts.append(dict(cell=cell,kind=kind,capture_root=str(root),status=row['status'],
            reasons=row.get('audit',{}).get('review_reasons',[]),shutdown=safety));save('running')
        if row['status']=='failed':raise RuntimeError('Scene failed; no automatic continuation')
        return row
    try:
        save('running')
        for index in indices:
            original=copy.deepcopy(source_data['scenes'][index]);cell=original['id'];base=output/cell;base.mkdir()
            m=copy.deepcopy(source_data);m.update(scenes=[original],count=1)
            manifest=base/'manifest.json';atomic_json(manifest,m)
            result=capture(manifest,base/'initial',cell,'initial')
            for ordinal in range(2):
                if result['status']=='accepted_candidate':break
                reasons=set(result['audit']['review_reasons'])
                if 'physical_collision_during_capture_window' in reasons:raise RuntimeError('Physical contact; stop quality loop')
                current=Path(attempts[-1]['capture_root'])
                if ordinal==0 and reasons=={'actor_annotation_above_ground'}:
                    next_manifest=height_retry(manifest,current,base/'height_deck')
                    kind='input_z_retry'
                elif ordinal==0 and reasons and reasons<={'actor_ground_plane_unverified','actor_annotation_deep_below_ground'}:
                    probes=empty_probe(manifest,[0],base/'terrain_deck')
                    capture(probes,base/'terrain',cell,'empty_terrain')
                    next_manifest=probed_retry(manifest,current,base/'terrain',base/'probed_deck')
                    kind='independent_ground_retry'
                else:
                    next_manifest=replacement(manifest,base/f'replacement_deck_{ordinal}')
                    kind='same_cell_replacement'
                if not gen.geo.load_json(next_manifest)['scenes']:break
                manifest=next_manifest
                result=capture(manifest,base/f'retry_{ordinal}',cell,kind)
            if result['status']=='accepted_candidate':
                result['source_cell_id']=cell;selected.append(result)
            else:unresolved.append(dict(cell=cell,reasons=result['audit']['review_reasons']))
            save('running')
        save('complete');return 0
    except Exception as exc:save('halted',str(exc));print(str(exc),file=sys.stderr);return 1
    finally:lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--indices',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(execute(a.manifest,[int(i) for i in a.indices.split(',')],a.output))
