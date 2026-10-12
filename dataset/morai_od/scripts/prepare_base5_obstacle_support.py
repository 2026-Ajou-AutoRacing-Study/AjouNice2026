#!/usr/bin/env python3
"""Build bounded paired pilots or offline-only full decks, never collect 5K.

Baseline payloads are preserved. Candidate attempts freeze primary, complete
asset list/count, split, role and mission contracts. UI remains Sunny13 for
qualification regardless of the seed environment of the input geometry.
"""
import argparse
from collections import Counter,defaultdict
import copy
import hashlib
import json
from pathlib import Path
import time

import base5_obstacle_support_policy as policy
import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json,sha256
from run_base5_failure_aware_replenishment_fast import prewarm


def read(path):return json.loads(Path(path).read_text())


def select(deck, count):
    buckets=defaultdict(list)
    for e in deck['scenes']:
        m=read(e['metadata']);r=m['spec']
        if not policy.eligible(r):continue
        targets=[a for a in m['runtime_actors'][1:] if a.get('population_pool','').startswith('obstacle')
                 and a['source_list']=='objectList' and a['motion'] in ('static','standing')]
        if not targets:continue
        if any(a['motion'] not in ('static','standing') for a in m['runtime_actors']):continue
        key=(r['family'],r['population_layout'],r['population_mode'],m['split'],
             min(3,len(targets)),max(a['population_cost'] for a in targets)>=6)
        buckets[key].append((hashlib.sha256(('support-pilot:'+e['id']).encode()).hexdigest(),e,m))
    # Interleave families first; a short pilot must not exhaust only the first
    # alphabetic family before reaching pedestrian-related generic scenes.
    by_family=defaultdict(list)
    for key,values in sorted(buckets.items()):by_family[key[0]].append(sorted(values))
    ordered=[]
    while any(by_family.values()):
        for family in sorted(by_family):
            if by_family[family]:ordered.append(by_family[family].pop(0))
    result=[]
    while len(result)<count:
        changed=False
        for values in ordered:
            if values and len(result)<count:
                _,e,m=values.pop(0);result.append((e,m));changed=True
        if not changed:raise ValueError('Insufficient eligible support scenes')
    return result


def prepare(source,output,variant,count=32,full=False):
    if output.exists():raise FileExistsError('Never overwrite pilot or frozen deck')
    if variant not in ('baseline',*policy.VARIANTS):raise ValueError('Unknown variant')
    snapshot_paths=[Path(__file__),Path(policy.__file__),Path(spatial.__file__),Path(spatial.gen.__file__),
                    Path(spatial.gen.population.__file__),Path(__file__).with_name('base5_obstacle_rich_floor.py')]
    snapshot={str(p):sha256(p) for p in snapshot_paths}
    deck=read(source);config=copy.deepcopy(deck['config'])
    selected=[(e,read(e['metadata'])) for e in deck['scenes']] if full else select(deck,count)
    config['environment_tag']='qualification_sunny13' if not full else deck['config']['environment_tag']
    if not full:config['collection_environment']=dict(weather='sunny',hour=13,policy='Sunny13 qualification only; not the input seed environment training data')
    config['obstacle_support_policy']=dict(policy=policy.POLICY,variant=variant,
        source_manifest_sha256=sha256(source),full_offline=full,bulk_started=False)
    if variant=='richfloor':
        from base5_obstacle_rich_floor import POLICY as RICH_POLICY
        config['obstacle_support_policy']['policy']=RICH_POLICY
    ctx=None
    if variant!='baseline':
        ctx=prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,spatial.gen.geo.DEFAULT_MGEO),config)
        # Budget the same measured static geometry that gen.build will use;
        # conservative fallback sizes can assign a different cost to new props.
        from base5_placement_geometry import configure
        representative=next((m['spec'] for _,m in selected if policy.eligible(m['spec'])),None)
        if representative is not None:ctx=configure(ctx,config,representative)
    extras=[]
    if variant=='richfloor':
        extras=sorted([a for a in ctx['pools']['obstacle'] if a['source_list']=='objectList'
                       and a.get('semantic_class')=='obstacle' and
                       spatial.gen.population.cost(spatial.gen.geometry(a,ctx)[0])<=6],key=lambda a:a['data_id'])
        if not extras:raise ValueError('No bounded physical obstacle assets')
    output.mkdir(parents=True);entries=[];errors=Counter();deltas=[];actual_specs=[];started=time.monotonic()
    extra_cursor=0
    for ordinal,(e,old) in enumerate(selected):
        if variant=='richfloor':
            from base5_obstacle_rich_floor import attach as rich_attach
            asset=extras[extra_cursor%len(extras)]
            row=rich_attach(old['spec'],old['runtime_actors'],asset,
                            spatial.gen.population.cost(spatial.gen.geometry(asset,ctx)[0]))
            extra_cursor+=int(bool(row.get('population_support_delta')))
        else:row=policy.attach(old['spec'],old['runtime_actors'],variant) if variant!='baseline' else copy.deepcopy(old['spec'])
        applicable=bool(row.get('secondary_obstacle_support'))
        if applicable:row['secondary_obstacle_support']['original_generation_attempt']=old['retry']
        if applicable:
            for attempt in range(old['retry'],old['retry']+4096):
                try:
                    scene,m=spatial.build(row,config,ctx,attempt)
                    if m['split']!=old['split']:raise ValueError('Support proposal changed split')
                    actual=[a['data_id'] for a in m['runtime_actors']]
                    expected=row.get('frozen_population_data_ids',[a['data_id'] for a in old['runtime_actors']])
                    if actual != expected:
                        raise ValueError('Support proposal changed asset/count')
                    break
                except ValueError as exc:errors[str(exc)]+=1
            else:
                atomic_json(output/'failure.json',dict(id=e['id'],completed=ordinal,reasons=dict(errors)))
                raise ValueError('Bounded support proposal exhausted '+e['id'])
        else:scene,m=read(e['scenario']),copy.deepcopy(old)
        for key in ('scene_group_id','environment_split_contract','logical_quota_slot_id',
                    'fresh_environment_provenance'):
            if key in old:m[key]=copy.deepcopy(old[key])
        m['obstacle_support_pair']=dict(source_id=e['id'],source_payload_sha256=e['scenario_sha256'],
            variant=variant,changed=applicable,original_retry=old['retry'],actual_retry=m['retry'],
            ego_xy_delta_m=((m['ego']['x']-old['ego']['x'])**2+(m['ego']['y']-old['ego']['y'])**2)**.5)
        # Full deck keeps frozen logical/group identity. Diagnostic pilot IDs are
        # separate and are never imported into the canonical Sunny13 ledger.
        ident=e['id'] if full else f'support_{variant}_{ordinal:03}_{e["id"]}'
        sp=output/'scenarios'/(ident+'.json');mp=output/'scenarios'/(ident+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,m)
        entry=copy.deepcopy(e);entry.update(id=ident,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
                                            scene_group_id=m.get('scene_group_id',e.get('scene_group_id',e['id'])))
        entries.append(entry)
        actual_specs.append(m['spec'])
        deltas.append(dict(id=e['id'],before=policy.geometry_summary(old),after=policy.geometry_summary(m),
                           pair=m['obstacle_support_pair']))
        if (ordinal+1)%100==0:print(variant,ordinal+1,round(time.monotonic()-started,1),flush=True)
    atomic_json(output/'frozen_specs.json',actual_specs)
    config['frozen_specs_path']=str((output/'frozen_specs.json').resolve())
    config['frozen_specs_sha256']=sha256(output/'frozen_specs.json')
    result=copy.deepcopy(deck);result.update(config=config,scenes=entries,count=len(entries),
        status='OFFLINE_ONLY_SUPPORT_CANDIDATE; not bulk collection; live qualification pending')
    if any(sha256(p)!=digest for p,digest in snapshot.items()):
        raise ValueError('Generator edited during preparation; output remains diagnostic only')
    result['provenance']['obstacle_support_source_hashes']=snapshot
    atomic_json(output/'manifest.json',result)
    atomic_json(output/'registration.json',dict(source=str(source),source_sha256=sha256(source),
        policy_sha256=sha256(policy.__file__),generator_sha256=sha256(spatial.gen.__file__),
        variant=variant,frames=len(entries),elapsed_sec=time.monotonic()-started,
        rejections=dict(errors),deltas=deltas,bulk_started=False,full_offline=full,source_hashes=snapshot))
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--variant',choices=('baseline',*policy.VARIANTS),required=True)
    p.add_argument('--count',type=int,default=32);p.add_argument('--full-offline',action='store_true')
    a=p.parse_args()
    if not a.full_offline and not 1<=a.count<=64:p.error('Pilot 1–64 only')
    print(prepare(a.manifest,a.output,a.variant,a.count,a.full_offline))
