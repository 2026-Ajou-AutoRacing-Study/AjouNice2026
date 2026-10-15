#!/usr/bin/env python3
"""Generate fresh geometry for one future environment from frozen quota/primary specs.

Does not copy payloads, apply UI weather, capture bulk, or mutate old Sunny13.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
import time

import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json, sha256
from run_base5_failure_aware_replenishment_fast import prewarm
from run_base5_motion_contract_repair import motion_matches

MAX_OFFLINE_PROPOSALS = 4096


def bounded_candidate(spec, config, ctx, reference, split, reasons, maximum=MAX_OFFLINE_PROPOSALS):
    """Finite CPU proposals; never relax geometry/quota or perform MR captures.

    Near rings are measured from the rear-axle Ego origin, not its front bumper.
    Long riders in 3–5m rings have a small collision-free subset. 512 draws can
    miss it even though the unchanged footprint checks permit valid geometry.
    More OFFLINE proposals are cheap; do not turn this into 4096 live retries.
    """
    local=Counter()
    for retry in range(maximum):
        try:
            scene,meta=spatial.build(spec,config,ctx,retry)
            if meta['split']!=split:raise ValueError('Original logical-slot split changed')
            primary=lambda m:m['runtime_actors'][0]['data_id'] if m['runtime_actors'] else None
            if primary(meta)!=primary(reference):raise ValueError('Original primary assignment changed')
            moving=lambda m:any(a['motion'] not in ('static','standing') for a in m['runtime_actors'])
            if moving(reference)!=moving(meta) or not motion_matches(reference,meta):
                raise ValueError('Actual animated motion contract changed')
            return scene,meta,retry,dict(local)
        except ValueError as exc:
            local[str(exc)]+=1;reasons[str(exc)]+=1
    raise ValueError('Offline proposal budget exhausted: '+json.dumps(dict(local),sort_keys=True))


def generate(source,output,environment,seed=261010,limit=5000,allow_new_sunny13=False):
    if not 1<=limit<=5000:raise ValueError('Generate 1–5000 slots per environment')
    if environment not in ('sunny11','sunny15','foggy11','foggy13','foggy15') and not (environment=='sunny13' and allow_new_sunny13):
        raise ValueError('Only the remaining five environments; Sunny13 stays preserved')
    if output.exists():raise FileExistsError('Fresh environment directory required')
    original=json.loads(source.read_text());source_hash=sha256(source);config=copy.deepcopy(original['config'])
    if config.get('obstacle_front_support_policy'):
        raise ValueError('Generate each fresh environment from the v150 source first, then run '
            'prepare_base5_obstacle_front_support.py on that complete environment deck. '
            'Do not reseed a frozen v151 allocation or silently remove its plan.')
    generator_hash=sha256(__file__);adapter_hash=sha256(spatial.__file__)
    config['dataset_id']='base5_fresh_v141_'+environment;config['environment_tag']=environment
    config['collection_environment']=dict(weather=environment[:-2],hour=int(environment[-2:]),
        policy='Manual UI confirmation required; generating geometry does not set weather/time')
    config['fresh_geometry_policy']=dict(policy=spatial.POLICY,seed=seed,source_manifest_sha256=source_hash)
    if environment=='sunny13':
        config['new_sunny13_diagnostic_plan']=dict(approved_offline_only=True,original_sunny13_preserved=True,
            purpose='Independent Sunny13 preview requested for generator qualification; not a second 5K capture')
    ctx=prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,spatial.gen.geo.DEFAULT_MGEO),config)
    intervals=spatial.allowed_intervals(ctx,config);items=[];references={}
    for entry in original['scenes']:
        meta=json.loads(Path(entry['metadata']).read_text());spec=copy.deepcopy(meta['spec'])
        spec['seed']=spatial.seed_for(environment,entry['id'],seed)
        items.append((entry['id'],spec,meta['split']));references[entry['id']]=(entry,meta)
    spatial.assign_plans(items,intervals,ctx)
    output.mkdir(parents=True)
    atomic_json(output/'frozen_specs.json',[spec for _,spec,_ in items])
    config['frozen_specs_path']=str((output/'frozen_specs.json').resolve())
    config['frozen_specs_sha256']=sha256(output/'frozen_specs.json')
    entries=[];failures=Counter();proposal_counts=[];start=time.monotonic()
    for slot,spec,split in items[:limit]:
        old,reference=references[slot]
        try:
            scene,meta,retry,local=bounded_candidate(spec,config,ctx,reference,split,failures)
        except ValueError as exc:
            atomic_json(output/'failure.json',dict(slot=slot,reasons=dict(failures),completed=len(entries),
                isolated_failure=str(exc),maximum_offline_proposals=MAX_OFFLINE_PROPOSALS))
            raise ValueError('Unfillable fresh-environment slot '+slot)
        proposal_counts.append(retry+1)
        meta['offline_generation_proposals']=dict(accepted_attempt=retry,proposals=retry+1,
            rejected_reasons=local,maximum=MAX_OFFLINE_PROPOSALS,live_capture_retries=False,
            geometry_or_quota_relaxed=False)
        ident=environment+'_'+slot
        meta['logical_quota_slot_id']=slot;meta['scene_group_id']='fresh_v141:'+ident
        meta['environment_split_contract']=dict(policy='physical-link split + unique geometry identity',
            original_slot_split=split,geometry_is_weather_pair=False)
        meta['fresh_environment_provenance']=dict(source_manifest_sha256=source_hash,
            source_payload_sha256=old['scenario_sha256'],source_slot_id=slot,
            environment=environment,geometry_seed=spec['seed'],generator_adapter_sha256=adapter_hash)
        sp=output/'scenarios'/(ident+'.json');mp=output/'scenarios'/(ident+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        entry=copy.deepcopy(old);entry.update(id=ident,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=sha256(sp),split=split,logical_quota_slot_id=slot)
        if entry['scenario_sha256']==old['scenario_sha256']:raise ValueError('Fresh geometry copied original payload')
        entries.append(entry)
        if len(entries)%100==0:print(environment,len(entries),f'{time.monotonic()-start:.1f}s',flush=True)
    doc=copy.deepcopy(original);doc.update(dataset_id=config['dataset_id'],config=config,count=len(entries),scenes=entries,
        status='FRESH_GEOMETRY_OFFLINE; live qualification and UI weather confirmation pending')
    if sha256(__file__)!=generator_hash or sha256(spatial.__file__)!=adapter_hash:
        raise ValueError('Generator edited during generation; outputs remain diagnostic only')
    doc['provenance'].update(fresh_source_manifest_sha256=source_hash,
        fresh_generator_sha256=generator_hash,spatial_adapter_sha256=adapter_hash)
    atomic_json(output/'manifest.json',doc)
    bases=Counter();finals=Counter();source_bases=set();modes=Counter();splits=Counter()
    for e in entries:
        m=json.loads(Path(e['metadata']).read_text());key=tuple(round(m['base'][k],3) for k in ('x','y'))
        bases[key]+=1;finals[tuple(round(m['ego'][k],3) for k in ('x','y'))]+=1
        ref=references[e['logical_quota_slot_id']][1];source_bases.add(tuple(round(ref['base'][k],3) for k in ('x','y')))
        modes['animated' if any(a['motion'] not in ('static','standing') for a in m['runtime_actors']) else 'static']+=1
        splits[m['split']]+=1
    atomic_json(output/'generation_audit.json',dict(source_sha256=source_hash,environment=environment,
        frames=len(entries),elapsed_sec=time.monotonic()-start,rejected_proposals=dict(failures),
        offline_proposal_counts=dict(maximum=max(proposal_counts),total=sum(proposal_counts),
            slots_over_512=sum(n>512 for n in proposal_counts),limit_per_slot=MAX_OFFLINE_PROPOSALS),
        split=dict(splits),actual_motion=dict(modes),unique_base_xy_3dp=len(bases),unique_final_xy_3dp=len(finals),
        source_base_overlap_3dp=len(set(bases)&source_bases),
        continuous_generic_slots=sum('spatial_sampling_plan' in s for _,s,_ in items[:limit]),
        bulk_started=False,live_qualified=False,
        limitations=['Saved initializer/map screening only; native ground, stationary, collision and visibility QA still mandatory.',
            'Equal logical slots are not paired weather scenes. Link split is not fully disjoint 100m sensor views.']))
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--environment',required=True)
    p.add_argument('--seed',type=int,default=261010);p.add_argument('--limit',type=int,default=5000)
    p.add_argument('--allow-new-sunny13-plan',action='store_true',help='Explicit new-output offline preview; preserve completed Sunny13')
    a=p.parse_args();print(generate(a.manifest,a.output,a.environment,a.seed,a.limit,a.allow_new_sunny13_plan))
