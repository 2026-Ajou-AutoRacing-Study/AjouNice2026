#!/usr/bin/env python3
"""Freeze original payloads for the exact support-pilot slot list."""
import argparse
import copy
import json
from pathlib import Path
from base5_capture_safety import atomic_json,sha256


def build(source,pilot,output):
    if output.exists():raise FileExistsError('New paired baseline output required')
    old=json.loads(source.read_text());selected=json.loads(pilot.read_text())
    index={e['id']:e for e in old['scenes']};entries=[]
    for e in selected['scenes']:
        baseline=copy.deepcopy(index[e['id']]);m=json.loads(Path(baseline['metadata']).read_text())
        m['obstacle_support_pair']=dict(source_id=e['id'],variant='baseline',changed=False,
            source_payload_sha256=baseline['scenario_sha256'],ego_xy_delta_m=0.)
        path=output/'metadata'/(e['id']+'.json');atomic_json(path,m);baseline['metadata']=str(path.resolve())
        entries.append(baseline)
    deck=copy.deepcopy(old);deck.update(scenes=entries,count=len(entries),status='MATCHED_BASELINE_PILOT_ONLY')
    atomic_json(output/'manifest.json',deck)
    atomic_json(output/'registration.json',dict(source_sha256=sha256(source),pilot_sha256=sha256(pilot),
        frames=len(entries),same_source_slots=True,bulk_started=False))
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','pilot','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();print(build(a.source,a.pilot,a.output))
