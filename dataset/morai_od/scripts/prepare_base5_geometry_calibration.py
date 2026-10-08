#!/usr/bin/env python3
"""Two safe static appearances per unresolved identity; bounded diagnostic only."""
import argparse
import copy
import json
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256
from base5_static_yaw import FINAL_POLICY


def prepare(source,reference,output):
    if output.exists():raise FileExistsError('Fresh calibration output required')
    deck=gen.geo.load_json(source);ref=gen.geo.load_json(reference)
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    by_name={a['name']:(pool,a) for pool in ('static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')
             for a in ctx['pools'][pool]}
    output.mkdir(parents=True);entries=[];errors=Counter()
    config=copy.deepcopy(deck['config']);config.pop('frozen_specs_path',None);config.pop('frozen_specs_sha256',None)
    config.pop('static_placement_geometry',None)
    template=next(gen.geo.load_json(Path(e['metadata']))['spec'] for e in deck['scenes']
                  if e['family']=='obstacle' and not e.get('mission')
                  and not gen.geo.load_json(Path(e['metadata']))['spec'].get('shadow_phase'))
    for j,name in enumerate(ref['unresolved']):
        pool,asset=by_name[name]
        for k in range(2):
            row=copy.deepcopy(template)
            for field in ('avoidance_phase','avoidance_origin','avoidance_side','avoidance_gap_bounds_m',
                          'static_yaw_policy','required_secondary_pool','population_budget_bounds'):
                row.pop(field,None)
            row.update(seed=261008130+j*101+k,mission=False,family='obstacle',topology='route_general',
                actors=1,motion='static',role='front',distance_bounds=[20.,35.],distance_bin='geometry-calibration',
                yaw_limit=3,population_mode='generic_single',population_layout='distributed',
                primary_pool_override=pool,primary_identity=asset['data_id'],
                static_yaw_policy=dict(policy=FINAL_POLICY,scope='generic',eligible_data_ids=[asset['data_id']],
                    obj_vehicle_data_ids=[a['data_id'] for a in ctx['pools']['static_vehicle']],primary_mode='uniform',
                    primary_sector=(k*3+6)%12))
            for retry in range(512):
                try:scene,meta=gen.build(row,config,ctx,retry);break
                except ValueError as exc:errors[str(exc)]+=1
            else:raise ValueError('Unfillable calibration '+name)
            sid=f'b5geom130_{j:02d}_{k}'
            sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
            atomic_json(sp,scene);atomic_json(mp,meta)
            entries.append(dict(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
                family=row['family'],mission=False,role='front',distance_bounds=row['distance_bounds'],split=meta['split'],
                subfamily=row.get('subfamily'),motion='static'))
    deck.update(config=config,scenes=entries,count=len(entries),status='BOUNDED_GEOMETRY_DIAGNOSTIC_NOT_BULK')
    atomic_json(output/'manifest.json',deck)
    return dict(scenes=len(entries),assets=len(ref['unresolved']),rejections=dict(errors))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(json.dumps(prepare(a.source,a.reference,a.output),ensure_ascii=False))
