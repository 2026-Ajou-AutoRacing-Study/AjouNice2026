#!/usr/bin/env python3
"""Order immutable stationary cells before animated cells; retain the complete quota."""
import argparse
import copy
from pathlib import Path
from base5_capture_safety import atomic_json,sha256
from base5_motion_ground import moving_actors
import generate_base5_independent as gen


def prepare(source,output):
    if output.exists():raise FileExistsError('New ordered manifest directory required')
    data=gen.geo.load_json(source);stationary=[];animated=[]
    ids=[row['id'] for row in data['scenes']]
    if len(ids)!=len(set(ids)):raise ValueError('Duplicate source cell')
    for row in data['scenes']:
        if sha256(Path(row['scenario']))!=row['scenario_sha256']:raise ValueError('Source payload checksum mismatch')
        meta=gen.geo.load_json(Path(row['metadata']))
        (animated if moving_actors(meta) else stationary).append(copy.deepcopy(row))
    deck=copy.deepcopy(data);deck.update(scenes=stationary+animated,count=len(ids),
        status='ORDERED_ONLY; ALL_QUOTA_RETAINED; LIVE_PENDING',
        collection_order=dict(policy='stationary-first; animated-last; stable within each group',
            source_manifest=str(source.resolve()),source_manifest_sha256=sha256(source),
            script_sha256=sha256(__file__),stationary=len(stationary),animated=len(animated),
            limitation='Ordering only; does not certify terrain, visibility, motion, collisions or throughput'))
    output.mkdir(parents=True);atomic_json(output/'manifest.json',deck)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(prepare(a.manifest,a.output))
