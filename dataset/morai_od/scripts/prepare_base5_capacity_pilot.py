#!/usr/bin/env python3
"""Prepare a family-stratified 32-cell capture-cost sample without loading MR."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen
from select_base5_pilot import select

COUNTS = dict(highway=10, roundabout=2, other_vehicle=1,
              obstacle=9, pedestrian_related=8, background=2)


def prepare(source, output,regenerate_current=False):
    if output.exists():
        raise FileExistsError('New pilot output required')
    data = gen.geo.load_json(source)
    indices = []
    for family, count in COUNTS.items():
        candidates = [(i, r) for i, r in enumerate(data['scenes']) if r['family'] == family]
        chosen = select(dict(scenes=[r for _, r in candidates]), count)
        if len(chosen) != count:
            raise ValueError('Insufficient family candidates: ' + family)
        indices.extend(candidates[i][0] for i in chosen)
    indices.sort()
    pilot = copy.deepcopy(data)
    rows=[copy.deepcopy(data['scenes'][i]) for i in indices]
    payloads=[]
    if regenerate_current:
        ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
        current=gen.geo.load_json(gen.CONFIG)
        pilot['config']['capture'].update(dynamic_path_policy=current['capture']['dynamic_path_policy'],
            asset_appearance_policy=current['capture']['asset_appearance_policy'])
        pilot['config']['gt_policy']=current['gt_policy']
        for index,row in zip(indices,rows):
            old=gen.geo.load_json(Path(row['metadata']))
            for retry in range(old['retry'],old['retry']+512):
                try:scene,meta=gen.build(old['spec'],pilot['config'],ctx,retry)
                except ValueError:continue
                if meta['split']!=old['split']:continue
                if old['runtime_actors'] and meta['runtime_actors'][0]['data_id']!=old['runtime_actors'][0]['data_id']:
                    raise ValueError('Capacity regeneration changed primary asset')
                break
            else:raise ValueError('Capacity regeneration unavailable in same quota/split')
            meta['capacity_regeneration']=dict(source_index=index,source_scenario_sha256=row['scenario_sha256'],
                source_manifest_sha256=gen.geo.sha256(source),policy='current generator; same spec/primary/split')
            sp=output/(row['id']+'.json');mp=output/(row['id']+'_metadata.json')
            row.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),
                motion=meta['runtime_actors'][0]['motion'] if meta['runtime_actors'] else 'static')
            payloads.append((row,sp,mp,scene,meta))
    pilot.update(scenes=rows, count=len(indices),
        status='32_CELL_CAPACITY_PILOT; LIVE_PENDING',
        capacity_sample=dict(source_manifest=str(source.resolve()), source_sha256=gen.geo.sha256(source),
            source_indices=indices, counts=COUNTS,regenerated_current=regenerate_current,
            limitation='Rounded family quotas with coverage-biased selection; not unbiased 10K throughput estimator'))
    output.mkdir(parents=True)
    for row,sp,mp,scene,meta in payloads:
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        row['scenario_sha256']=gen.geo.sha256(sp)
    gen.geo.write_json(output / 'manifest.json', pilot)
    return output / 'manifest.json'


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--regenerate-current',action='store_true',help='Rebuild only the 32 quota cells using current generator')
    a = p.parse_args()
    print(prepare(a.manifest, a.output,a.regenerate_current))
