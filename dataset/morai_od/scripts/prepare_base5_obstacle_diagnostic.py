#!/usr/bin/env python3
"""Single-asset range/yaw diagnostics, separate from the training quotas."""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path

import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json,sha256
from run_base5_failure_aware_replenishment_fast import prewarm

ASSETS=('CargoBox','PE_Drum','WoodBox','TrafficBarrel','Steel_Barricade',
        'concrete-barrier','Bollard04','Scooter_01')


def prepare(source,output):
    if output.exists():raise FileExistsError('New diagnostic directory required')
    deck=json.loads(source.read_text());config=copy.deepcopy(deck['config'])
    base=next(json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']
              if e['family']=='obstacle' and not e['mission'] and e['role']=='front'
              and e['distance_bounds']==[3,10])
    ctx=prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,spatial.gen.geo.DEFAULT_MGEO),config)
    assets={a['name']:a for a in ctx['pools']['obstacle']}
    if any(n not in assets for n in ASSETS):raise ValueError('Diagnostic asset unavailable')
    entries=[];errors=Counter()
    for name in ASSETS:
        for distance in (10,20,25,35):
            for yaw_sector in (6,7,9):
                row=copy.deepcopy(base['spec'])
                for key in list(row):
                    if ('avoidance' in key or key.startswith('original_') or key in
                            ('required_secondary_pool','frozen_population_data_ids','population_budget_bounds','secondary_obstacle_support')):
                        row.pop(key)
                row.update(actors=1,population_mode='generic_single',population_max_actors=1,
                           population_layout='single',role='front',motion='static',primary_pool_override='obstacle',
                           primary_identity=assets[name]['data_id'],asset_index=0,
                           distance_bounds=[distance-1.5,distance+1.5],distance_bin='diagnostic_range',
                           static_yaw_policy=dict(row['static_yaw_policy'],primary_mode='uniform',primary_sector=yaw_sector))
                for attempt in range(4096):
                    try:
                        scene,meta=spatial.build(row,config,ctx,attempt)
                        if meta['split']!=base['split']:raise ValueError('Diagnostic changed physical split')
                        break
                    except ValueError as exc:errors[str(exc)]+=1
                else:raise ValueError('Unfillable diagnostic '+name+str(distance))
                ident=f'diag_{name}_{distance:02}_sector{yaw_sector}'
                meta['obstacle_diagnostic']=dict(asset=name,range_center_m=distance,
                    requested_road_yaw_sector=yaw_sector,actual_road_yaw_deg=meta['runtime_actors'][0]['static_yaw_policy']['road_relative_yaw_deg'],
                    actual_anchor_xy=[meta['ego']['x'],meta['ego']['y']],not_training=True)
                sp=output/'scenarios'/(ident+'.json');mp=output/'scenarios'/(ident+'_metadata.json')
                atomic_json(sp,scene);atomic_json(mp,meta)
                entries.append(dict(id=ident,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
                    scenario_sha256=sha256(sp),split=meta['split'],family='obstacle',mission=False,
                    distance_bounds=row['distance_bounds'],role='front',motion='static',subfamily=None))
    config['environment_tag']='qualification_sunny13'
    config['collection_environment']=dict(weather='sunny',hour=13,policy='Sunny13 diagnostic only')
    result=copy.deepcopy(deck);result.update(config=config,scenes=entries,count=len(entries),status='DIAGNOSTIC_ONLY; not training')
    atomic_json(output/'manifest.json',result)
    atomic_json(output/'registration.json',dict(source_sha256=sha256(source),script_sha256=sha256(__file__),
        frames=len(entries),assets=ASSETS,rejections=dict(errors),bulk_started=False,
        purpose='16-channel support at 10/20/25/35m with three continuous yaw sectors; not detector AP'))
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.manifest,a.output))
