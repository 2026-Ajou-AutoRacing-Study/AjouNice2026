#!/usr/bin/env python3
"""Bounded same-cell visibility replenishment; raw attempts are never overwritten."""
import argparse
import copy
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import time

import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256,status_row


def eligible(row):
    return row['status']=='review' and set(row.get('audit',{}).get('review_reasons',[]))=={'primary_no_sensor_evidence'}


def prepare(source,ledger,output,attempts=3):
    if not 1<=attempts<=4:raise ValueError('At most four attempts per cell')
    if output.exists():raise FileExistsError('Replenishment plan overwrite refused')
    source_data=json.loads(source.read_text());by_id={r['id']:r for r in source_data['scenes']}
    latest={r['id']:r for r in map(json.loads,ledger.read_text().splitlines())}
    reviews=[r for r in latest.values() if eligible(r)]
    if not reviews or len(reviews)>16 or len(reviews)*attempts>64:
        raise ValueError('Require 1–16 visibility-only cells and <=64 planned attempts')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    rows=[];output.mkdir(parents=True)
    for previous in reviews:
        if previous['id'] not in by_id:raise ValueError('Review cell absent from source manifest')
        capture=Path(previous['capture_root']);meta=json.loads((capture/'metadata.json').read_text())
        if sha256(capture/'scenario.json')!=previous['scenario_sha256']:raise ValueError('Source capture checksum mismatch')
        primary=meta['runtime_actors'][0]
        for ordinal in range(attempts):
            first=meta['retry']+1+ordinal*128
            for retry in range(first,first+128):
                try:scene,new=gen.build(meta['spec'],source_data['config'],ctx,retry)
                except ValueError:continue
                if new['split']!=meta['split']:continue
                if new['runtime_actors'][0]['data_id']!=primary['data_id']:raise ValueError('Primary asset changed')
                break
            else:raise ValueError('Same-cell/split replenishment unfillable')
            name=previous['id']+'_r'+str(ordinal+1).zfill(2)
            new['replacement']=dict(source_cell_id=previous['id'],ordinal=ordinal+1,
                source_capture_root=str(capture),source_scenario_sha256=previous['scenario_sha256'],
                primary_data_id=primary['data_id'],split=meta['split'],
                policy='same asset/family/mission/distance/direction/motion/split; no GT tolerance relaxation')
            sp=output/(name+'.json');mp=output/(name+'_metadata.json')
            atomic_json(sp,scene);atomic_json(mp,new)
            row=copy.deepcopy(by_id[previous['id']]);row.update(id=name,source_cell_id=previous['id'],
                scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
                split=new['split'],motion=new['runtime_actors'][0]['motion'])
            rows.append(row)
    manifest=copy.deepcopy(source_data);manifest.update(scenes=rows,count=len(rows),start=0,
        status='BOUNDED_VISIBILITY_REPLENISHMENT_ONLY; no bulk capture',
        replenishment_provenance=dict(manifest_sha256=sha256(source),ledger_sha256=sha256(ledger),
            cells=len(reviews),maximum_attempts_per_cell=attempts,generator_sha256=sha256(Path(gen.__file__))))
    atomic_json(output/'manifest.json',manifest)
    return output/'manifest.json'


def execute(manifest_path,output):
    with (output/'.replenishment.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Another replenishment process owns this output')
        return _execute(manifest_path,output)


def _execute(manifest_path,output):
    prior=output/'report.json'
    if prior.exists() and json.loads(prior.read_text())['stage']=='halted':
        raise RuntimeError('Halted safety session cannot automatically resume; check MR first')
    manifest=json.loads(manifest_path.read_text());rows=manifest['scenes']
    if not rows or len(rows)>64:raise ValueError('Bounded pilot only')
    cells=list(dict.fromkeys(r['source_cell_id'] for r in rows));selected=[];reports=[]
    runner=Path(__file__).with_name('run_base5_capture.py');started=time.time()
    def save(stage,error=None):
        accepted={r['source_cell_id'] for r in selected}
        exhausted={c for c in cells if sum(r['source_cell_id']==c for r in reports)==sum(r['source_cell_id']==c for r in rows)}-accepted
        atomic_json(output/'report.json',dict(stage=stage,selected=selected,attempts=reports,
            unresolved_cells=[c for c in cells if c not in accepted],error=error))
        atomic_json(output/'status.json',dict(stage=stage,error=error,current=reports[-1]['id'] if reports else None,
            updated_at=time.time(),requested=len(cells),finished=len(accepted|exhausted),
            accepted_candidate=len(accepted),review=len(exhausted),failed=int(stage=='halted'),eta_sec=0,
            elapsed_sec=time.time()-started,results=[status_row(r) for r in selected]))
    save('running')
    for cell in cells:
        for index,row in enumerate(rows):
            if row['source_cell_id']!=cell:continue
            if (output/'STOP').exists():save('stopped','STOP file requested');return 0
            root=output/'attempts'/row['id']
            cmd=[sys.executable,str(runner),'--manifest',str(manifest_path),'--output',str(root),'--indices',str(index),'--limit','1']
            try:run=subprocess.run(cmd,capture_output=True,text=True,timeout=90)
            except subprocess.TimeoutExpired:
                save('halted','Bounded child timeout; do not continue');return 1
            if run.returncode:
                reports.append(dict(id=row['id'],source_cell_id=cell,status='failed',stderr=run.stderr,stdout=run.stdout))
                save('halted','Child failed, including exit Park gate; no further loads');return 1
            shutdown=json.loads((root/'shutdown_safety.json').read_text())
            if not shutdown['park_verified']:save('halted','Park not verified');return 1
            results=list(map(json.loads,(root/'ledger.jsonl').read_text().splitlines()))
            result=results[-1]
            if result['id']!=row['id']:raise ValueError('Child result ID mismatch')
            result['source_cell_id']=cell
            reports.append(dict(id=row['id'],source_cell_id=cell,status=result['status'],
                review_reasons=result.get('audit',{}).get('review_reasons',[]),shutdown=shutdown))
            if result['status']=='accepted_candidate':selected.append(result)
            save('running');print(json.dumps(reports[-1]),flush=True)
            if result['status']=='accepted_candidate':break
    # Only one accepted measurement per original cell. All other raw stays in attempts/.
    target=output/'selected_ledger.jsonl'
    body=''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected)
    if not target.exists() or target.read_text()!=body:target.write_text(body)
    save('complete');return 0


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-manifest',type=Path,required=True);p.add_argument('--source-ledger',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--attempts',type=int,default=3)
    p.add_argument('--run',action='store_true',help='Capture only bounded planned attempts; no bulk flag')
    a=p.parse_args();manifest=a.output/'manifest.json'
    if not manifest.exists():manifest=prepare(a.source_manifest,a.source_ledger,a.output,a.attempts)
    else:
        contract=json.loads(manifest.read_text())['replenishment_provenance']
        if contract['manifest_sha256']!=sha256(a.source_manifest) or contract['ledger_sha256']!=sha256(a.source_ledger):
            raise ValueError('Resume source provenance changed')
        if contract['maximum_attempts_per_cell']!=a.attempts or contract['generator_sha256']!=sha256(Path(gen.__file__)):
            raise ValueError('Resume generator/attempt contract changed')
    print(str(manifest),flush=True)
    if a.run:raise SystemExit(execute(manifest,a.output))
