#!/usr/bin/env python3
"""Bounded 48-cell readiness deck; preserves source specs, never emits a 10K deck."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen


def choose(rows, metadata):
    picked=[]
    def take(label,predicate,count=1):
        candidates=[i for i,r in enumerate(rows) if i not in picked and predicate(r,metadata(i))]
        if len(candidates)<count:raise ValueError('Missing coverage: '+label)
        picked.extend(candidates[:count])
    for role in ('front','rear'):
        for lo in (6,10,30,45,60,80):
            take('highway '+role+str(lo),lambda r,m,role=role,lo=lo:
                r['family']=='highway' and r['role']==role and r['distance_bounds'][0]==lo)
    take('roundabout',lambda r,m:r['family']=='roundabout',4)
    for family in ('other_vehicle','obstacle'):
        for phase,role in [('approach','front'),('entry','front'),('alongside','left'),('alongside','right')]:
            take(family+phase+role,lambda r,m,family=family,phase=phase,role=role:
                r['family']==family and m['spec'].get('avoidance_phase')==phase and r['role']==role)
    # Eight additional obstacle cells: close/front/rear/side plus long front.
    for role,lo in [('front',3),('front',10),('front',30),('front',60),
                    ('rear',3),('rear',10),('left',5),('right',5)]:
        take('obstacle '+role+str(lo),lambda r,m,role=role,lo=lo:
            r['family']=='obstacle' and r['role']==role and r['distance_bounds'][0]==lo)
    for motion in ('standing','walking','running'):
        take('person '+motion,lambda r,m,motion=motion:r['subfamily']=='person' and r['motion']==motion
            and (motion=='standing' or r['distance_bounds'][0]>=15))
    for name in ('Dog1','Dog2','Deer1'):
        take(name,lambda r,m,name=name:r['subfamily']=='animal' and m['runtime_actors'][0]['name']==name)
    take('dummy',lambda r,m:r['subfamily']=='dummy')
    for lo in (3,15):
        take('pet'+str(lo),lambda r,m,lo=lo:r['subfamily']=='pet_person' and r['distance_bounds'][0]==lo)
    for role,lo in [('front',3),('left',3),('front',15)]:
        take('stroller'+role+str(lo),lambda r,m,role=role,lo=lo:r['subfamily']=='stroller'
            and r['role']==role and r['distance_bounds'][0]==lo)
    for lo in (5,15):
        take('rider'+str(lo),lambda r,m,lo=lo:r['subfamily']=='rider' and r['distance_bounds'][0]==lo)
    take('background',lambda r,m:r['family']=='background',2)
    if len(picked)!=48:raise AssertionError(len(picked))
    return picked


def prepare(source,output,retry_indices=None,retry_offset=0):
    if output.exists():raise FileExistsError('Pilot overwrite refused')
    manifest=gen.geo.load_json(source);cache={}
    def metadata(i):
        if i not in cache:cache[i]=gen.geo.load_json(Path(manifest['scenes'][i]['metadata']))
        return cache[i]
    if retry_indices is not None:
        if not 1<=len(retry_indices)<=4 or len(set(retry_indices))!=len(retry_indices):
            raise ValueError('Diagnostic retry limited to 1–4 unique cells')
        if any(i<0 or i>=len(manifest['scenes']) for i in retry_indices):raise ValueError('Retry index out of range')
        if not 1<=retry_offset<=16:raise ValueError('Diagnostic retry offset required')
        ids=retry_indices
    else:ids=choose(manifest['scenes'],metadata)
    config=manifest['config'];current=gen.geo.load_json(gen.CONFIG)
    # Historical one-way appearance probes lack the target-position contract
    # required by terrain preparation. Never downgrade a natural-target source.
    config['capture']['dynamic_path_policy']=current['capture']['dynamic_path_policy']
    config['capture']['asset_appearance_policy']=current['capture']['asset_appearance_policy']
    config['gt_policy']=current['gt_policy']
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    output.mkdir(parents=True);entries=[]
    for index in ids:
        old=manifest['scenes'][index];meta=metadata(index)
        first=meta['retry']+retry_offset
        for retry in range(first,first+1024):
            try:scene,newmeta=gen.build(meta['spec'],config,ctx,retry);break
            except ValueError:
                if retry==first+1023:raise
        name=old['id']+(f'_retry{retry_offset:02d}' if retry_indices is not None else '')
        sp=output/(name+'.json');mp=output/(name+'_metadata.json')
        newmeta['readiness_pilot_source']=dict(index=index,manifest_sha256=gen.geo.sha256(source))
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,newmeta)
        row=copy.deepcopy(old);row.update(id=name,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=gen.geo.sha256(sp),motion=newmeta['runtime_actors'][0]['motion'] if newmeta['runtime_actors'] else 'static')
        if retry_indices is not None:row['source_cell_id']=old.get('source_cell_id',old['id'])
        entries.append(row)
    manifest.update(scenes=entries,count=len(entries),start=0,status='READINESS_PILOT_ONLY',
        readiness_pilot_provenance=dict(source_indices=ids,source_sha256=gen.geo.sha256(source),
            generator_sha256=gen.geo.sha256(Path(gen.__file__)),placement_attempts=1024,
            limitation='Coverage sample, not exhaustive catalog or production accuracy evidence'))
    gen.geo.write_json(output/'manifest.json',manifest)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--retry-indices',help='1–4 source row indices for explicit diagnostic replacement')
    p.add_argument('--retry-offset',type=int,default=0)
    a=p.parse_args();indices=[int(i) for i in a.retry_indices.split(',')] if a.retry_indices else None
    print(prepare(a.manifest,a.output,indices,a.retry_offset))
