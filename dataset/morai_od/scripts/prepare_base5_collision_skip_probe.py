#!/usr/bin/env python3
"""Two stationary scenes for explicitly labelled CollisionData fault injection."""
import argparse
import copy
from pathlib import Path
from base5_capture_safety import atomic_json,sha256
import generate_base5_independent as gen

def prepare(source,output):
    if output.exists():raise FileExistsError('New probe output required')
    original=gen.geo.load_json(source);output.mkdir(parents=True)
    rows=[]
    for index in (0,0):
        old=original['scenes'][index];scene=gen.geo.load_json(Path(old['scenario']))
        meta=gen.geo.load_json(Path(old['metadata']))
        contact=not rows
        meta.update(calibration_only=contact,collision_skip_diagnostic=True)
        if contact:meta['diagnostic_exception']='INJECTED_TEST_CONTACT; not proof of a physical collision'
        sid=old['id']+('_contactprobe' if contact else '_normalprobe');sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        rows.append(dict(old,id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp)))
    deck=copy.deepcopy(original);deck.update(scenes=rows,count=2,status='DIAGNOSTIC_ONLY_NOT_TRAINING')
    atomic_json(output/'manifest.json',deck);return output/'manifest.json'

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.manifest,a.output))
