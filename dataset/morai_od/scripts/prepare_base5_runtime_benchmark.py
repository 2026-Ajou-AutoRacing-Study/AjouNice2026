#!/usr/bin/env python3
"""Freeze deterministic, pre-outcome timing samples from a completed 5K deck."""
import argparse
from collections import defaultdict
import copy
import hashlib
import json
from pathlib import Path
from base5_capture_safety import atomic_json, sha256


def prepare(source,output,static_count=64,moving_count=8):
    if output.exists(): raise FileExistsError('Fresh benchmark registration required')
    deck = json.loads(source.read_text()); groups = defaultdict(list); movers = defaultdict(list)
    for row in deck['scenes']:
        m = json.loads(Path(row['metadata']).read_text()); spec=m['spec']
        moving = any(a['motion'] not in ('static','standing') for a in m['runtime_actors'])
        rank = hashlib.sha256(('runtime-v141:'+row['id']).encode()).hexdigest()
        if moving:
            movers['shadow' if spec.get('shadow_moving') else 'intersection'].append((rank,row));continue
        actors=m['runtime_actors']; cost=max((a.get('population_cost',0) for a in actors),default=0)
        key=(spec['family'],bool(spec['mission']),len(actors)>1,cost>=6,
             bool(spec.get('distance_bounds') and spec['distance_bounds'][0]>=30))
        groups[key].append((rank,row))
    def choose(buckets,n):
        ordered = [sorted(v,key=lambda t:t[0]) for _,v in sorted(buckets.items())]
        result=[]
        while len(result)<n:
            progressed=False
            for values in ordered:
                if values and len(result)<n:result.append(values.pop(0)[1]);progressed=True
            if not progressed:raise ValueError('Not enough registered scenes')
        return result
    static=choose(groups,static_count);moving=choose(movers,moving_count)
    for name,rows in [('static64',static),('animated8',moving)]:
        doc=copy.deepcopy(deck);doc['scenes']=rows;doc['count']=len(rows)
        atomic_json(output/name/'manifest.json',doc)
    atomic_json(output/'registration.json',dict(source_sha256=sha256(source),script_sha256=sha256(__file__),
        policy='Stratified round-robin, SHA ordering fixed before new runtime outcomes; no score selection',
        static=[dict(id=r['id'],sha256=r['scenario_sha256']) for r in static],
        animated=[dict(id=r['id'],sha256=r['scenario_sha256']) for r in moving],
        comparison='same payloads, Sunny13 UI and sensor; chunk4 baseline vs chunk4 delay vs chunk16 combined; repeated reverse order'))
    return output


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.manifest,a.output))
