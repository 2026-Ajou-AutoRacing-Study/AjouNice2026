#!/usr/bin/env python3
"""Small near/mixed deck built by the ordinary generator, not a separate placer."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen


def prepare(output):
    if output.exists():raise FileExistsError('Pilot overwrite refused')
    config=gen.geo.load_json(gen.CONFIG)
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    rows=gen.specs(config);entries=[];output.mkdir(parents=True)
    cases=[('person','Woman2'),('animal','Dog1'),('animal','Dog2'),('animal','Deer1'),
           ('pet_person','Ped_Pet1'),('stroller','BabyStroller2'),('stroller','BabyStroller3'),
           ('rider','Bike3'),('rider','Scooter1')]
    for index,(group,name) in enumerate(cases):
        row=copy.deepcopy(next(r for r in rows if r['subfamily']==group and not r['mission']))
        row.update(index=28000+index,asset_index=next(i for i,a in enumerate(ctx['pools'][group]) if a['name']==name),
            motion='walking',actors=2,role=('left','right','front')[index%3],distance_bounds=[5,10] if index%2 else [3,5])
        for retry in range(1024):
            try:
                scene,meta=gen.build(row,config,ctx,retry)
                if meta['runtime_actors'][1]['motion'] not in ('static','standing'):continue
                break
            except ValueError:continue
        else:raise ValueError('Unfillable pilot '+name)
        sid=f'b5appearance28_{index:02d}';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        meta['appearance_integration_pilot']=dict(target_asset=name,generated_by='ordinary gen.build',
            limitation='Nine deliberately chosen near mixed scenes, not exhaustive catalog coverage')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        entries.append(dict(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp),
            split=meta['split'],family=row['family'],mission=row['mission'],distance_bounds=row['distance_bounds'],
            role=row['role'],motion=meta['runtime_actors'][0]['motion'],subfamily=group))
    manifest=dict(schema_version=2,config=config,dataset_id=config['dataset_id'],count=len(entries),start=0,
        status='SMALL_APPEARANCE_INTEGRATION_PILOT_ONLY',scenes=entries,
        provenance=dict(generator_sha256=gen.geo.sha256(Path(gen.__file__)),appearance_policy_sha256=gen.geo.sha256(gen.APPEARANCE_POLICY)))
    gen.geo.write_json(output/'manifest.json',manifest)
    return output/'manifest.json'


def resample(source,indices,output):
    if output.exists():raise FileExistsError('Pilot overwrite refused')
    if not 1<=len(indices)<=4 or len(set(indices))!=len(indices):raise ValueError('Retry limited to 1–4 unique rows')
    manifest=gen.geo.load_json(source);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    output.mkdir(parents=True);entries=[]
    for index in indices:
        old=manifest['scenes'][index];original=gen.geo.load_json(Path(old['metadata']))
        for retry in range(original['retry']+1,original['retry']+1025):
            try:
                scene,meta=gen.build(original['spec'],manifest['config'],ctx,retry)
                if meta['runtime_actors'][1]['motion'] not in ('static','standing'):continue
                break
            except ValueError:continue
        else:raise ValueError('Unfillable retry '+old['id'])
        sid=old['id']+'_resample';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        meta['appearance_resample_source']=dict(manifest_sha256=gen.geo.sha256(source),source_id=old['id'],
            previous_retry=original['retry'],note='Same asset/distance/direction quota; new placement, not ground correction')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        new=copy.deepcopy(old);new.update(id=sid,source_cell_id=old['id'],scenario=str(sp.resolve()),
            metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp));entries.append(new)
    manifest.update(scenes=entries,count=len(entries),status='BOUNDED_GENERAL_PLACEMENT_RETRY')
    gen.geo.write_json(output/'manifest.json',manifest)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--manifest',type=Path);p.add_argument('--indices',help='1–4 original row indices to resample once')
    a=p.parse_args()
    if bool(a.manifest)!=bool(a.indices):p.error('--manifest and --indices must be provided together')
    print(resample(a.manifest,[int(i) for i in a.indices.split(',')],a.output) if a.manifest else prepare(a.output))
