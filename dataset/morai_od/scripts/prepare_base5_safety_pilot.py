#!/usr/bin/env python3
"""Regenerate a bounded v15 coverage subset using updated placement references."""
import argparse
import copy
import json
from pathlib import Path
import generate_base5_independent as gen
from select_base5_avoidance_pilot import select as avoidance_select


def prepare(source,output,motion_policy=None,retry_offset=0,extra_runtime_coverage=False,placement_attempts=128,setup_budget_sec=None):
    if not 1<=placement_attempts<=2048:raise ValueError('Bounded offline placement attempts required')
    manifest=gen.geo.load_json(source)
    if motion_policy:manifest['config']['capture']['dynamic_path_policy']=motion_policy
    if setup_budget_sec is not None:
        if manifest['config']['capture'].get('dynamic_path_policy')!='one-way-slow-v1' or not 4<=setup_budget_sec<=30:
            raise ValueError('Setup budget is only for bounded one-way probes')
        manifest['config']['capture']['one_way_setup_budget_sec']=setup_budget_sec
    if output.exists():raise FileExistsError('Pilot overwrite refused')
    ids=avoidance_select(manifest)
    predicates=[lambda r:r['family']=='highway' and r['role']=='front' and r['distance_bounds'][0]>=45,
        lambda r:r['family']=='highway' and r['role']=='rear' and r['distance_bounds'][0]>=80,
        lambda r:r['family']=='roundabout',lambda r:r['family']=='background']
    for subtype in ('person','animal','stroller','pet_person'):
        predicates.append(lambda r,s=subtype:r.get('subfamily')==s and r['motion'] not in ('static','standing'))
    for pred in predicates:
        candidates=[i for i,r in enumerate(manifest['scenes']) if pred(r) and i not in ids]
        if not candidates:raise ValueError('Missing safety pilot coverage')
        ids.append(min(candidates))
    if extra_runtime_coverage:
        checks=[('AI police',lambda r,m:m['runtime_actors'] and m['runtime_actors'][0]['name']=='police1'),
            ('moving rider',lambda r,m:r['subfamily']=='rider' and r['motion'] not in ('static','standing')),
            ('front near stroller',lambda r,m:r['subfamily']=='stroller' and r['role']=='front' and r['distance_bounds']==[3,5] and r['motion'] not in ('static','standing'))]
        for label,check in checks:
            for index,row in enumerate(manifest['scenes']):
                if index not in ids and row['family']=='pedestrian_related' and check(row,gen.geo.load_json(Path(row['metadata']))):
                    ids.append(index);break
            else:raise ValueError('Missing runtime coverage: '+label)
    output.mkdir(parents=True);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO);entries=[]
    for index in ids:
        old=manifest['scenes'][index];meta=gen.geo.load_json(Path(old['metadata']))
        first=meta['retry']+retry_offset
        for attempt in range(first,first+placement_attempts):
            try:scene,newmeta=gen.build(meta['spec'],manifest['config'],ctx,attempt);break
            except ValueError:
                if attempt==first+placement_attempts-1:raise
        name=old['id'];sp=output/(name+'.json');mp=output/(name+'_metadata.json')
        newmeta['safety_pilot_source']=dict(manifest=str(source.resolve()),index=index,
            scenario_sha256=old['scenario_sha256'],placement_reference_updated=True)
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,newmeta)
        entry=copy.deepcopy(old);entry.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp),
            motion=newmeta['runtime_actors'][0]['motion'] if newmeta['runtime_actors'] else old['motion']);entries.append(entry)
    manifest.update(scenes=entries,count=len(entries),start=0,status='SAFETY_PILOT_ONLY; no bulk collection',
        safety_pilot_provenance=dict(source_manifest_sha256=gen.geo.sha256(source),source_indices=ids,
            geometry_references=ctx['geometry_sources'],placement_attempts=placement_attempts,generator_sha256=gen.geo.sha256(Path(gen.__file__))))
    gen.geo.write_json(output/'manifest.json',manifest);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--motion-policy',choices=['bounded-shuttle-v1','one-way-slow-v1']);p.add_argument('--retry-offset',type=int,default=0)
    p.add_argument('--extra-runtime-coverage',action='store_true');p.add_argument('--placement-attempts',type=int,default=128)
    p.add_argument('--one-way-setup-budget-sec',type=float);a=p.parse_args()
    print(prepare(a.manifest,a.output,a.motion_policy,a.retry_offset,a.extra_runtime_coverage,a.placement_attempts,a.one_way_setup_budget_sec))
