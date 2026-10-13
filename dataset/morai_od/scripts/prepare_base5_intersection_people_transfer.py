#!/usr/bin/env python3
"""Test—not assume—whether Woman2's editor path height transfers to other people."""
import argparse
import copy
from pathlib import Path
import shutil

import generate_base5_independent as gen
import prepare_base5_intersection_ground_pilot as pilot
from base5_capture_safety import atomic_json,sha256
from base5_intersection_ground import load_lookup


def prepare(lookup_file,prepared_file,output):
    if output.exists():raise FileExistsError('New transfer diagnostic output required')
    lookup=load_lookup(lookup_file)
    prepared=gen.geo.load_json(prepared_file)
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    people=[a for a in ctx['pools']['person'] if a['source_list']=='pedestrianList' and a['name']!='Woman2']
    output.mkdir(parents=True);data=copy.deepcopy(lookup);data['profiles']={};data['status']='PERSON_HEIGHT_TRANSFER_PENDING_LIVE'
    data.pop('live_evidence',None);rows=[]
    for crossing_row in (r for r in prepared['scenes'] if r['asset']=='Woman2'):
        ref=crossing_row['crosswalk']['crosswalk_id']+'/Woman2'
        for asset in people:
            profile=copy.deepcopy(lookup['profiles'][ref])
            profile.update(asset=asset['name'],data_id=asset['data_id'],contact_verified=False,
                           transfer_reference=ref,transfer_verified=False,
                           limitation='Shared pedestrian waypoint-height candidate; this individual asset still requires live ground/Z/visibility checks')
            key=crossing_row['crosswalk']['crosswalk_id']+'/'+asset['name'];data['profiles'][key]=profile
            row=copy.deepcopy(crossing_row)
            row.update(number=100+len(rows),asset=asset['name'],data_id=asset['data_id'],source_list=asset['source_list'])
            rows.append(row)
    files={a['source_file'] for a in people}
    for name in files:
        src=gen.geo.DEFAULT_RUNTIME_ASSET_DIR/name;dst=output/name;shutil.copy2(src,dst)
        data['sources'].append(dict(path=str(dst.resolve()),sha256=sha256(dst),original_path=str(src)))
    atomic_json(output/'height_lookup_pending.json',data)
    prepared.update(scenes=rows,count=len(rows));atomic_json(output/'transfer_prepared.json',prepared)
    return pilot.prepare(output/'height_lookup_pending.json',output/'transfer_prepared.json',output/'pilot')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--lookup',type=Path,required=True)
    p.add_argument('--prepared-manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(prepare(a.lookup,a.prepared_manifest,a.output))
