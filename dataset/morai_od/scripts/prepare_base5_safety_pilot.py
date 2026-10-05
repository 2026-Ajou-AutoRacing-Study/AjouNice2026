#!/usr/bin/env python3
"""Regenerate a bounded v15 coverage subset using updated placement references."""
import argparse
import copy
import json
from pathlib import Path
import generate_base5_independent as gen
from select_base5_avoidance_pilot import select as avoidance_select


def prepare(source,output):
    manifest=gen.geo.load_json(source)
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
    output.mkdir(parents=True);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO);entries=[]
    for index in ids:
        old=manifest['scenes'][index];meta=gen.geo.load_json(Path(old['metadata']))
        for attempt in range(meta['retry'],meta['retry']+128):
            try:scene,newmeta=gen.build(meta['spec'],manifest['config'],ctx,attempt);break
            except ValueError:
                if attempt==meta['retry']+127:raise
        name=old['id'];sp=output/(name+'.json');mp=output/(name+'_metadata.json')
        newmeta['safety_pilot_source']=dict(manifest=str(source.resolve()),index=index,
            scenario_sha256=old['scenario_sha256'],placement_reference_updated=True)
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,newmeta)
        entry=copy.deepcopy(old);entry.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp));entries.append(entry)
    manifest.update(scenes=entries,count=len(entries),start=0,status='SAFETY_PILOT_ONLY; no bulk collection',
        safety_pilot_provenance=dict(source_manifest_sha256=gen.geo.sha256(source),source_indices=ids,
            geometry_references=ctx['geometry_sources'],generator_sha256=gen.geo.sha256(Path(gen.__file__))))
    gen.geo.write_json(output/'manifest.json',manifest);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(prepare(a.manifest,a.output))
