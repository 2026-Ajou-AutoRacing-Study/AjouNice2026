"""Explicit historical-runtime validation; import references, never copy/relabel raw."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def verify(receipt,source,indices):
    data=json.loads(Path(receipt).read_text())
    if data['schema']!=1 or digest(source)!=data['manifest_sha256']:
        raise ValueError('Imported campaign source mismatch')
    deck=json.loads(Path(source).read_text())
    allowed={deck['scenes'][i]['id'] for i in indices}
    ids=[r['source_cell_id'] for r in data['selected']]+[r['id'] for r in data['deficits']]
    if len(ids)!=len(set(ids)) or not set(ids)<=allowed:
        raise ValueError('Imported campaign cells overlap or fall outside request')
    for path,value in data['hashes'].items():
        if digest(path)!=value:raise ValueError('Imported evidence changed: '+path)
    for path,value in data['sensor_stats'].items():
        s=Path(path).stat()
        if [s.st_size,s.st_mtime_ns]!=value:raise ValueError('Imported raw changed: '+path)
    return copy.deepcopy(data['selected']),copy.deepcopy(data['deficits']),data['elapsed_sec']


def export(source,old,output,runtime,archive):
    # Run the ORIGINAL integrity validators with their exact saved code/config.
    # The current capture runtime never receives a historical hash bypass.
    sys.path.insert(0,str(runtime/'dataset/morai_od/scripts'))
    import run_base5_batch_campaign as batch
    from base5_capture_safety import prepare_session,atomic_json
    deck=json.loads(source.read_text());contract=json.loads((old/'batch_contract.json').read_text())
    expected=batch.campaign.contract(source,contract['indices'],2)
    expected.update(batch_driver_sha256=digest(runtime/'dataset/morai_od/scripts/run_base5_batch_campaign.py'),
        chunk_size=contract['chunk_size'],bulk=contract['bulk'],
        replenishment_driver_sha256=digest(runtime/'dataset/morai_od/scripts/base5_campaign_replenishment.py'))
    if expected!=contract:raise ValueError('Historical runtime/source does not match original batch contract')
    status=json.loads((old/'status.json').read_text())
    selected={r['source_cell_id']:r for r in batch.load_ledger(old/'selected_ledger.jsonl')}
    deficits={r['id']:r for r in json.loads((old/'quota.json').read_text())['cells']}
    lookup={r['id']:i for i,r in enumerate(deck['scenes'])}
    hashes={};stats={}
    def evidence(path):hashes[str(path.resolve())]=digest(path)
    for path in (source,archive,old/'batch_contract.json',old/'status.json',old/'selected_ledger.jsonl',old/'quota.json'):
        evidence(path)
    for folder in sorted(old.glob('chunk_*')):
        root=folder/'capture';manifest=folder/'manifest.json'
        if not (root/'session_contract.json').exists():continue
        session=json.loads((root/'session_contract.json').read_text())
        rows,_=prepare_session(root,manifest,
            runtime/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
            runtime/'configs/sensors/Sensor_competition_local.json',session.get('runner_options'))
        if not json.loads((root/'shutdown_safety.json').read_text()).get('park_verified'):
            raise ValueError('Historical child final Park unverified')
        for path in (manifest,root/'session_contract.json',root/'ledger.jsonl',root/'shutdown_safety.json'):evidence(path)
        for row in rows:
            ident=row['id']
            if not row.get('between_scene_park',{}).get('park_verified'):
                raise ValueError('Historical boundary Park unverified')
            if row['status']=='failed' and row.get('failure_kind')=='ego_stationary_support':
                p=Path(row['failure_evidence']);proof=json.loads(p.read_text())
                if digest(p)!=row['failure_evidence_sha256'] or proof.get('scenario_sha256')!=row['scenario_sha256'] or not proof.get('no_capture_requested'):
                    raise ValueError('Historical stationary failure proof mismatch')
                if proof['reason'] not in ('ego_pre_capture_not_stationary','ego_post_park_not_stationary','stationary_ego_drift_exceeded'):
                    raise ValueError('Unknown historical Ego failure')
                evidence(p);deficits[ident]=dict(id=ident,index=lookup[ident],reasons=['ego_stationary_support'],evidence=row,
                    policy='historical rejected scene; never accepted or automatically replayed')
                continue
            if batch.is_collision_failure(row):batch.validate_collision_failure(row)
            elif row['status']=='failed':batch.campaign.loop.validate_miss_evidence(row,manifest)
            recovery=folder/'recovery'/ident
            if (recovery/'status.json').exists():
                batch.verify_recovery(source,lookup[ident],recovery)
                for path in recovery.rglob('*'):
                    if path.is_file() and path.suffix in ('.json','.jsonl'):evidence(path)
                recovered=batch.load_ledger(recovery/'selected_ledger.jsonl')
                if recovered:selected[ident]=recovered[0];deficits.pop(ident,None)
                else:deficits[ident]=dict(id=ident,index=lookup[ident],reasons=json.loads((recovery/'quota.json').read_text())['cells'])
            elif row['status']=='accepted_candidate':
                if row['audit']['review_reasons']:raise ValueError('Historical accepted capture has review reasons')
                chosen=copy.deepcopy(row);chosen['source_cell_id']=ident;selected[ident]=chosen;deficits.pop(ident,None)
            elif ident not in selected:
                deficits[ident]=dict(id=ident,index=lookup[ident],reasons=row.get('audit',{}).get('review_reasons',[row.get('error')]))
    for row in selected.values():
        root=Path(row['capture_root'])
        if row['status']!='accepted_candidate' or row['audit']['review_reasons'] or json.loads((root/'audit.json').read_text())!=row['audit']:
            raise ValueError('Selected historical audit mismatch')
        if json.loads((root/'metadata.json').read_text()).get('calibration_only'):raise ValueError('Historical calibration cannot fill quota')
        if digest(root/'scenario.json')!=row['scenario_sha256']:raise ValueError('Historical selected scenario changed')
        for path in root.rglob('*'):
            if not path.is_file():continue
            if path.suffix=='.json':evidence(path)
            else:
                s=path.stat();stats[str(path.resolve())]=[s.st_size,s.st_mtime_ns]
    for row in deck['scenes']:
        evidence(Path(row['scenario']));evidence(Path(row['metadata']))
    receipt=dict(schema=1,manifest_sha256=digest(source),historical_runtime_archive=str(archive.resolve()),
        historical_runtime_sha256=digest(archive),original_output=str(old.resolve()),
        selected=list(selected.values()),deficits=list(deficits.values()),elapsed_sec=status['elapsed_sec'],
        hashes=hashes,sensor_stats=stats,
        raw_integrity='Historical session completeness + unchanged sizes/mtimes; no full raw checksum scrub')
    output.mkdir(parents=True,exist_ok=False);atomic_json(output/'imported_prefix.json',receipt)
    verify(output/'imported_prefix.json',source,contract['indices'])
    print(json.dumps(dict(imported=len(selected),deficits=len(deficits),pending=len(contract['indices'])-len(selected)-len(deficits),output=str(output))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','old-output','output','historical-runtime','runtime-archive'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--worker',action='store_true');a=p.parse_args()
    if a.worker:export(a.manifest,a.old_output,a.output,a.historical_runtime,a.runtime_archive)
    else:
        command=[sys.executable,str(Path(__file__).resolve()),*sys.argv[1:],'--worker']
        raise SystemExit(subprocess.call(command))
