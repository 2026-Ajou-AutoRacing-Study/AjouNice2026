#!/usr/bin/env python3
"""Fresh 5K: 250 roundabout geometry cells plus parked static NPC state; no live capture."""
import argparse
import copy
import json
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256
from base5_static_vehicle import park,POLICY as STATIC_POLICY


def revise(source,output,plan_path,reference_path):
    if output.exists():raise FileExistsError('Fresh release output required')
    original=gen.geo.load_json(source);config=copy.deepcopy(original['config'])
    plan=gen.geo.load_json(plan_path)
    config['mission_roundabout_observation']=plan
    config['dataset_id']='base5_independent_5k_sunny13_roundabout_v119'
    config['capture']['static_vehicle_policy']=STATIC_POLICY
    # Persist evidence hashes; never learn offsets from a locked test bag.
    reference=gen.geo.load_json(reference_path)
    if any('evaluation_locked' in r['path'] for r in reference['bags']):
        raise ValueError('Locked bag in generation references')
    plan['development_reference']=dict(path=str(reference_path.resolve()),sha256=sha256(reference_path))
    rows=gen.specs(config);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    gen.assign_crossing_directions(rows,ctx,config)
    by_index={r['index']:r for r in rows}
    output.mkdir(parents=True);scenes_dir=output/'scenarios_5k';scenes_dir.mkdir()
    entries=[];changed=[];controls_changed=[];reasons_all=Counter();attempts=Counter()
    for index,entry in enumerate(original['scenes']):
        if sha256(Path(entry['scenario']))!=entry['scenario_sha256']:
            raise ValueError('Source payload changed')
        if not (entry['family']=='roundabout' and entry['mission']):
            prior_scene=gen.geo.load_json(Path(entry['scenario']))
            prior_meta=gen.geo.load_json(Path(entry['metadata']))
            if not prior_scene['vehicleList']:
                entries.append(copy.deepcopy(entry));continue
            scene=copy.deepcopy(prior_scene);meta=copy.deepcopy(prior_meta)
            for actor in scene['vehicleList']:park(actor)
            for actor in meta['runtime_actors']:
                if actor['source_list']=='vehicleList':
                    if actor['motion'] not in ('static','standing'):raise ValueError('Refuse to park a moving NPC')
                    actor['stationary_vehicle_policy']=STATIC_POLICY
            # Geometry is not regenerated: identifiers, poses, paths, actors,
            # quotas, source metadata and splits remain byte-equivalent values.
            expected=copy.deepcopy(prior_scene)
            for actor in expected['vehicleList']:park(actor)
            if expected!=scene:raise ValueError('Control migration changed geometric placement')
            sp=scenes_dir/(entry['id']+'.json');mp=scenes_dir/(entry['id']+'_metadata.json')
            atomic_json(sp,scene);atomic_json(mp,meta)
            new=copy.deepcopy(entry);new.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp))
            entries.append(new);controls_changed.append(entry['id']);continue
        old_meta=gen.geo.load_json(Path(entry['metadata']))
        row=by_index[old_meta['spec']['index']]
        for attempt in range(512):
            try:scene,meta=gen.build(row,config,ctx,attempt);break
            except ValueError as exc:reasons_all[str(exc)]+=1
        else:
            atomic_json(output/'failed_cell.json',dict(row=row,reasons=dict(reasons_all)))
            raise ValueError('Unfillable mission '+entry['id'])
        meta['roundabout_observation']['development_reference']=plan['development_reference']
        sp=scenes_dir/(entry['id']+'.json');mp=scenes_dir/(entry['id']+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        new=copy.deepcopy(entry)
        new.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
            split=meta['split'],distance_bounds=row['distance_bounds'],role=row['role'],motion='static')
        entries.append(new);attempts[str(attempt)]+=1
        changed.append(dict(id=entry['id'],original_payload_sha256=entry['scenario_sha256'],
            new_payload_sha256=new['scenario_sha256'],phase=row['roundabout_phase']))
    if len(changed)!=250 or len(entries)!=5000:raise ValueError('Expected exact 250/5000 release')
    deck=copy.deepcopy(original)
    deck.update(dataset_id=config['dataset_id'],config=config,scenes=entries,status='OFFLINE_GENERATED_ROUNDABOUT_V119; live pending')
    deck['provenance'].update(generator_sha256=sha256(gen.__file__),
        config_sha256=gen.hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
        roundabout_mission_sha256=sha256(gen.roundabout_mission.__file__),
        roundabout_source_manifest_sha256=sha256(source),
        roundabout_plan_sha256=sha256(plan_path),roundabout_development_reference_sha256=sha256(reference_path))
    atomic_json(output/'config.json',config)
    atomic_json(scenes_dir/'manifest.json',deck)
    atomic_json(output/'revision_report.json',dict(changed_geometry_scenes=250,preserved_geometry_scenes=4750,
        stationary_vehicle_control_only_updated=len(controls_changed),control_only_updated_ids=controls_changed,
        byte_preserved_other_scenes=4750-len(controls_changed),
        phases=dict(Counter(r['phase'] for r in changed)),generation_attempts=dict(attempts),
        generation_rejection_reasons=dict(reasons_all),changes=changed,
        source_manifest=str(source.resolve()),source_manifest_sha256=sha256(source),
        old_raw_and_outputs_preserved=True,bulk_capture_started=False))
    pilot=[];used=set()
    candidates=[(e,gen.geo.load_json(Path(e['metadata']))) for e in entries if e['family']=='roundabout' and e['mission']]
    for phase in plan['phase_counts']:
        for density in plan['density_weights']:
            match=next((e for e,m in candidates if e['id'] not in used and m['spec']['roundabout_phase']==phase
                        and m['spec']['roundabout_density']==density),None)
            if match is None:raise ValueError('Pilot phase/density unavailable')
            pilot.append(match);used.add(match['id'])
    for condition in (lambda m: m['runtime_actors'][0]['estimated_size'][0]>10.,
                      lambda m:len(m['runtime_actors'])==3 and m['spec']['roundabout_phase']=='circulation',
                      lambda m:m['spec'].get('roundabout_observation_state')=='waiting'):
        match=next(e for e,m in candidates if e['id'] not in used and condition(m))
        pilot.append(match);used.add(match['id'])
    pilot_deck=copy.deepcopy({k:v for k,v in deck.items() if k!='scenes'})
    pilot_deck.update(scenes=pilot,count=len(pilot),status='ROUNDABOUT_STRATIFIED_PILOT_PENDING_LIVE')
    atomic_json(output/'pilot_manifest.json',pilot_deck)
    return scenes_dir/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plan',type=Path,default=gen.geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_roundabout_mission_v118.json')
    p.add_argument('--development-reference',type=Path,required=True)
    a=p.parse_args();print(revise(a.source_manifest,a.output,a.plan,a.development_reference))
