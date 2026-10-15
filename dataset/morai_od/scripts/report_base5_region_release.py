#!/usr/bin/env python3
"""Planned parent exposure vs native visible components; never conflate the two."""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_region_contract import category, validate, validate_quotas


def pool_group(actor):
    key=actor['population_pool']
    return 'obstacle' if key.startswith('obstacle') else key


def report(manifest, source=None, ledger=None):
    deck=gen.geo.load_json(manifest);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    metas=[];groups=Counter();primary=Counter();asset=Counter();actor_scenes=Counter();ranges=Counter();roles=Counter()
    ids=set();geometry=[];planned={};moving=Counter();multiplicity=Counter();shadow_preserved=0
    old={e['id']:e for e in gen.geo.load_json(source)['scenes']} if source else {}
    for e in deck['scenes']:
        if e['id'] in ids:raise ValueError('Duplicate scene group')
        ids.add(e['id'])
        if sha256(Path(e['scenario']))!=e['scenario_sha256']:raise ValueError('Changed source payload')
        m=gen.geo.load_json(Path(e['metadata']));metas.append(m);r=m['spec'];cat=category(r)
        proof=validate(gen,ctx,m,deck['config']);geometry.append(proof)
        if m['scene_group_id']!=e['scene_group_id']:raise ValueError('Scene group mismatch')
        population=m['runtime_actors'];multiplicity[len(population)]+=1
        if population:primary[population[0]['name']]+=1
        present=set()
        for a in population:
            group=pool_group(a);groups[group]+=1;asset[a['name']]+=1;present.add(group)
            ranges[str((cat,group,r['mission'],a['actual_ego_role'],
                        '0-10' if a['planned_center_distance_m']<10 else '10-30' if a['planned_center_distance_m']<30 else
                        '30-60' if a['planned_center_distance_m']<60 else '60-100'))]+=1
        for group in present:actor_scenes[group]+=1
        roles[str((cat,r['mission'],r.get('role')))]+=1
        moving['moving' if any(a['motion'] not in ('static','standing') for a in population) else 'static']+=1
        planned[e['id']]=dict(pool=pool_group(population[0]) if population else None,category=cat)
        if source and cat=='shadow':
            if old[e['id']]['scenario_sha256']!=e['scenario_sha256']:raise ValueError('Shadow geometry changed')
            shadow_preserved+=1
    quotas=validate_quotas(metas)
    sample={};gt=Counter();visible=Counter();sampled=Counter();seen=set();accepted=0;contact=0
    if ledger:
        for line in ledger.read_text().splitlines():
            row=json.loads(line);source_id=row.get('source_cell_id',row['id'])
            if source_id in seen:raise ValueError('Repeated selected source cell')
            seen.add(source_id)
            if source_id not in planned:raise ValueError('Pilot source missing')
            if row['status']!='accepted_candidate':continue
            root=Path(row['capture_root']);m=gen.geo.load_json(root/'metadata.json');a=gen.geo.load_json(root/'audit.json')
            if a['review_reasons']:raise ValueError('Reviewed capture selected')
            accepted+=1
            pool_by_uid={x['unique_id']:pool_group(x) for x in m['runtime_actors']}
            for x in m['runtime_actors']:sampled[pool_group(x)]+=1
            visible_uids=set()
            for c in a['components']:
                if c['positive_lidar1_camera0'] and all(lo<=v<hi for v,lo,hi in zip(c['center_lidar_m'],[-100,-30,-3],[100,30,2])):
                    gt[c['training_class']]+=1;visible_uids.add(c['parent_uid'])
            for uid in visible_uids:visible[pool_by_uid[uid]]+=1
        sample=dict(accepted=accepted,visible_parent_counts=dict(visible),placed_parent_counts=dict(sampled),
            native_positive_components=dict(gt),visibility_fraction_by_pool={k:visible[k]/n for k,n in sampled.items()},
            warning='Preregistered hard-case pilot is not a random sample; do not treat its visibility fraction as a 5K guarantee.')
    previous_groups=Counter()
    if source:
        for e in old.values():
            for a in gen.geo.load_json(Path(e['metadata']))['runtime_actors']:previous_groups[pool_group(a)]+=1
    return dict(status='OFFLINE_REGIONAL_PASS',manifest=str(manifest.resolve()),manifest_sha256=sha256(manifest),
        target_frames=5000,total_environment_frames=30000,independent_geometry_scenes=5000,
        quotas=quotas,planned_capture_motion=dict(moving),planned_actor_multiplicity=dict(multiplicity),
        planned_groups=[dict(group=k,planned_parents_5k=n,planned_parents_30k=n*6,
            containing_scenes_5k=actor_scenes[k],source_parents_5k=previous_groups.get(k),
            change_from_source=n-previous_groups[k] if source else None) for k,n in sorted(groups.items())],
        planned_total_parents_5k=sum(groups.values()),planned_total_parents_30k=sum(groups.values())*6,
        planned_asset_exposures=[dict(name=k,primary_5k=primary[k],all_parents_5k=asset[k],all_parents_30k=asset[k]*6) for k in sorted(asset)],
        primary_assets=len(primary),primary_floor=min(primary.values()),shadow_geometry_preserved=shadow_preserved,
        planned_distance_direction_histogram=dict(ranges),primary_roles=dict(roles),pilot=sample,
        limitations=['Parent counts are exact planned exposures, not confirmed native GT counts.',
            'Partial visibility can remove parents; composite assets can produce several GT components per parent.',
            'Six environments retain the same 5K geometry and split; no independent-data or model-quality claim.',
            'Regional rules concern new actor placement, not inference filtering or GT removal.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--source-manifest',type=Path);p.add_argument('--ledger',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=report(a.manifest,a.source_manifest,a.ledger);atomic_json(a.output,r)
    print(json.dumps({k:v for k,v in r.items() if k not in ('planned_asset_exposures','planned_distance_direction_histogram','primary_roles')},ensure_ascii=False,indent=2))
