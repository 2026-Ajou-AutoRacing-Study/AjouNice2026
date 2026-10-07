#!/usr/bin/env python3
"""Copy a bounded static deck with mandatory ground evidence for every actor."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_motion_ground import moving_actors


def prepare(source, indices, output):
    if output.exists():raise FileExistsError('New validation output required')
    if not 1<=len(indices)<=8 or len(set(indices))!=len(indices):
        raise ValueError('Static ground validation limited to 1–8 distinct cells')
    deck=gen.geo.load_json(source);rows=[];payloads=[]
    for i in indices:
        old=deck['scenes'][i]
        if sha256(Path(old['scenario']))!=old['scenario_sha256']:raise ValueError('Source checksum mismatch')
        meta=gen.geo.load_json(Path(old['metadata']))
        if moving_actors(meta):raise ValueError('Static validation cannot erase moving quota')
        meta['ground_support_check']=dict(enabled=True,max_range_m=20.,maximum_gap_m=.15,
            required_uids=[a['unique_id'] for a in meta['runtime_actors']],policy='all-actors-observed-ground-v1')
        meta['static_ground_validation']=dict(source_manifest_sha256=sha256(source),
            source_metadata_sha256=sha256(Path(old['metadata'])),
            limitation='Observed annotation/terrain proxy, not mesh contact; sparse terrain remains review')
        row=copy.deepcopy(old);sid=old['id']+'_groundcheck'
        row.update(id=sid,source_cell_id=old.get('source_cell_id',old['id']),
            scenario=str((output/(sid+'.json')).resolve()),metadata=str((output/(sid+'_metadata.json')).resolve()))
        rows.append(row);payloads.append((row,gen.geo.load_json(Path(old['scenario'])),meta))
    output.mkdir(parents=True)
    for row,scene,meta in payloads:
        atomic_json(Path(row['scenario']),scene);atomic_json(Path(row['metadata']),meta)
        row['scenario_sha256']=sha256(Path(row['scenario']))
    deck.update(scenes=rows,count=len(rows),status='STATIC_ALL_ACTOR_GROUND_PENDING_LIVE')
    atomic_json(output/'manifest.json',deck)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--indices',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(prepare(a.manifest,[int(i) for i in a.indices.split(',')],a.output))
