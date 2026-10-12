#!/usr/bin/env python3
"""Verify source preservation and explicit population deltas in offline decks."""
import argparse
from collections import Counter
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
from base5_obstacle_support_policy import geometry_summary,verify_protected_near
from base5_region_contract import category


def audit(source,candidate):
    old=json.loads(source.read_text());new=json.loads(candidate.read_text())
    original={e['id']:e for e in old['scenes']}
    counts=Counter();before=Counter();after=Counter();extras=Counter();errors=[]
    protected_payloads=0
    for e in new['scenes']:
        m=json.loads(Path(e['metadata']).read_text())
        pair=m['obstacle_support_pair'];ref=original[pair['source_id']]
        r=json.loads(Path(ref['metadata']).read_text())
        counts[category(m['spec'])+(' mission' if m['spec']['mission'] else ' generic')]+=1
        before.update(geometry_summary(r));after.update(geometry_summary(m))
        if sha256(e['scenario'])!=e['scenario_sha256']:errors.append([e['id'],'scenario checksum'])
        if m['split']!=r['split'] or m.get('scene_group_id')!=r.get('scene_group_id'):
            errors.append([e['id'],'split/group identity'])
        for key in ('family','mission','role','distance_bounds','subfamily','motion'):
            if m['spec'].get(key)!=r['spec'].get(key):errors.append([e['id'],'primary spec '+key])
        aa=m['runtime_actors'];bb=r['runtime_actors']
        ids=lambda values:[a['data_id'] for a in values]
        if ids(aa[:len(bb)])!=ids(bb) or len(aa)-len(bb) not in (0,1):
            errors.append([e['id'],'original asset list/count lost'])
        if len(aa)>8:errors.append([e['id'],'actor cap'])
        if len(aa)>len(bb):extras[aa[-1]['name']]+=1
        plan=m['spec'].get('secondary_obstacle_support')
        if plan:
            for i,a in enumerate(aa):
                try:verify_protected_near(m['spec'],i,a['planned_center_distance_m'])
                except ValueError as exc:errors.append([e['id'],str(exc)])
                if i in plan['ordinals'] and not a.get('secondary_obstacle_support'):
                    errors.append([e['id'],'support proof absent'])
        else:
            if e['scenario_sha256']!=ref['scenario_sha256']:errors.append([e['id'],'protected payload changed'])
            protected_payloads+=1
        if e.get('scene_group_id')!=m.get('scene_group_id'):errors.append([e['id'],'entry/metadata group mismatch'])
        if any(a['motion']!=b['motion'] for a,b in zip(aa,bb)):errors.append([e['id'],'actual motion changed'])
    if len(new['scenes'])==len(old['scenes']) and set(e['id'] for e in new['scenes'])!=set(original):
        errors.append(['deck','full logical slot identity'])
    frozen=json.loads(Path(new['config']['frozen_specs_path']).read_text())
    if sha256(new['config']['frozen_specs_path'])!=new['config']['frozen_specs_sha256']:
        errors.append(['deck','frozen specs hash'])
    if frozen!=[json.loads(Path(e['metadata']).read_text())['spec'] for e in new['scenes']]:
        errors.append(['deck','frozen specs do not match generated metadata'])
    return dict(source_sha256=sha256(source),candidate_sha256=sha256(candidate),frames=len(new['scenes']),
        primary_and_original_asset_list_preserved=not errors,protected_payloads_exactly_preserved=protected_payloads,
        categories=dict(counts),before=dict(before),after=dict(after),extra_assets=dict(extras),errors=errors,
        live_qualified=False,bulk_started=False,
        limitation='Offline geometry/identity/budget support, not native visibility or learned accuracy')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=audit(a.source,a.candidate);atomic_json(a.output,r)
    print(json.dumps({k:v for k,v in r.items() if k!='extra_assets'},ensure_ascii=False,indent=2))
    if r['errors']:raise SystemExit(1)
