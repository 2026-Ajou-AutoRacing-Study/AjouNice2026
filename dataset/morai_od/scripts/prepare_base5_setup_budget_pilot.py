#!/usr/bin/env python3
"""Bounded matched natural-motion setup-budget deck; canonical 10K stays untouched."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256


def prepare(source,indices,output,budget=6.,include_static=False,approve_bulk=False):
    if output.exists():raise FileExistsError('New budget pilot output required')
    maximum=10000 if approve_bulk and include_static else (64 if include_static else 8)
    if not 1<=len(indices)<=maximum or len(set(indices))!=len(indices):raise ValueError('Bounded unique pilot cells required')
    if not 6<=budget<=9:raise ValueError('Pilot setup budget must be 6–9 seconds')
    original=gen.geo.load_json(source)
    deck=copy.deepcopy({k:v for k,v in original.items() if k!='scenes'})
    deck['config']['capture']['natural_setup_budget_sec']=budget
    context=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    pending=[]
    dynamic_count=0
    for index in indices:
        entry=copy.deepcopy(original['scenes'][index]);old=gen.geo.load_json(Path(entry['metadata']))
        if sha256(Path(entry['scenario']))!=entry['scenario_sha256']:raise ValueError('Source scenario changed')
        if not any(a.get('motion_path',{}).get('policy')=='natural-target-v1' for a in old['runtime_actors']):
            if not include_static:raise ValueError('Budget pilot requires natural motion')
            pending.append((entry,gen.geo.load_json(Path(entry['scenario'])),old));continue
        dynamic_count+=1
        if dynamic_count>8 and not approve_bulk:raise ValueError('Mixed pilot limited to eight natural-motion cells')
        scene,new=gen.build(old['spec'],deck['config'],context,old['retry'])
        if any(new[k]!=old[k] for k in ('spec','ego','split')):raise ValueError('Budget changed quota or Ego')
        if len(new['runtime_actors'])!=len(old['runtime_actors']):raise ValueError('Budget changed actors')
        for before,after in zip(old['runtime_actors'],new['runtime_actors']):
            keys=('data_id','unique_id','motion','geometric_center_estimate','estimated_size')
            if any(before[k]!=after[k] for k in keys):raise ValueError('Budget changed target geometry or appearance speed')
            first=before.get('motion_path');second=after.get('motion_path')
            if first is not None:
                if any(first[k]!=second[k] for k in ('policy','target_position','nominal_speed_mps','speed_field_value')):
                    raise ValueError('Budget changed motion target/speed')
        new['setup_budget_pilot']=dict(source_metadata_sha256=sha256(Path(entry['metadata'])),
            source_scenario_sha256=entry['scenario_sha256'],budget_sec=budget,
            policy='same spec/retry, ego, target geometry and appearance speed; departure-only budget candidate')
        pending.append((entry,scene,new))
    output.mkdir(parents=True)
    entries=[]
    for entry,scene,meta in pending:
        path=output/(entry['id']+'.json');mp=output/(entry['id']+'_metadata.json')
        atomic_json(path,scene);atomic_json(mp,meta)
        entry.update(scenario=str(path.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(path))
        entries.append(entry)
    deck.update(count=len(entries),scenes=entries,status='SETUP_BUDGET_DECK; LIVE_PENDING' if approve_bulk else 'SETUP_BUDGET_PILOT; LIVE_PENDING',
        setup_budget_pilot=dict(source_manifest_sha256=sha256(source),indices=indices,budget_sec=budget))
    deck['provenance']['config_sha256']=hashlib.sha256(json.dumps(deck['config'],sort_keys=True).encode()).hexdigest()
    atomic_json(output/'manifest.json',deck)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    select=p.add_mutually_exclusive_group(required=True)
    select.add_argument('--indices');select.add_argument('--all-scenes',action='store_true')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--budget-sec',type=float,default=6)
    p.add_argument('--include-static',action='store_true',help='Allow bounded mixed pilot up to64 cells, with at most8 natural-motion cells')
    p.add_argument('--approve-bulk-prepare',action='store_true',help='Explicit offline-only deck preparation up to10K; does not authorize capture')
    a=p.parse_args()
    indices=list(range(len(gen.geo.load_json(a.manifest)['scenes']))) if a.all_scenes else [int(i) for i in a.indices.split(',')]
    print(prepare(a.manifest,indices,a.output,a.budget_sec,a.include_static,a.approve_bulk_prepare))
