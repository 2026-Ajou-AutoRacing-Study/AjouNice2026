#!/usr/bin/env python3
"""Extend preregistered yaw pilot with distinct person clearance-floor assets."""
import argparse
import copy
import json
from pathlib import Path
from base5_capture_safety import atomic_json,sha256


def prepare(manifest,pilot,output):
    if output.exists():raise FileExistsError('Fresh pilot manifest required')
    deck=json.loads(manifest.read_text());p=json.loads(pilot.read_text());used={e['id'] for e in p['scenes']}
    names=set();tags=[]
    for e in p['scenes']:
        m=json.loads(Path(e['metadata']).read_text())
        if m['spec'].get('character_spawn_gap_policy'):names.add(m['runtime_actors'][0]['name'])
    selected=[]
    for e in deck['scenes']:
        if e['id'] in used:continue
        m=json.loads(Path(e['metadata']).read_text())
        if not m['spec'].get('character_spawn_gap_policy'):continue
        name=m['runtime_actors'][0]['name']
        if name in names:continue
        selected.append(e);names.add(name);tags.append(dict(id=e['id'],tag='person_gap_floor_'+name))
        if len(selected)==4:break
    if len(selected)!=4:raise ValueError('Four additional distinct clearance-floor people required')
    p['scenes']+=copy.deepcopy(selected);p['count']=len(p['scenes'])
    p['preregistered_clearance_extension']=dict(source_manifest_sha256=sha256(manifest),
        original_pilot_sha256=sha256(pilot),additional_strata=tags,selected_before_live_results=True)
    atomic_json(output,p);return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--pilot',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(prepare(a.manifest,a.pilot,a.output))
