#!/usr/bin/env python3
"""Compare paired placement captures, not learned detection AP."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path

import numpy as np
from base5_capture_safety import atomic_json,sha256


def summarize(doc):
    rows=[c for f in doc['captures'] for c in f['components'] if c['class']=='obstacle']
    result={}
    for name,lo,hi in [('near20',0,20),('mid20_30',20,30),('far30',30,110)]:
        rr=[c for c in rows if lo<=c['range']<hi]
        kept=[c for c in rr if c['kept512']]
        result[name]=dict(native=len(rr),kept512=len(kept),lidar5=sum(c['points']>=5 for c in kept),
            lidar20=sum(c['points']>=20 for c in kept),camera_only=sum(c['points']==0 for c in kept),
            points_p10_p50_p90=np.percentile([c['points'] for c in kept],[10,50,90]).tolist() if kept else None)
    result['placed']=sum(f['placed'] for f in doc['captures'])
    result['valid_by_class']=dict(Counter(c['class'] for f in doc['captures'] for c in f['components'] if c['kept512']))
    result['outcomes']=doc['outcomes']
    result['accepted_rich_obstacle_gt']=sum(c['points']>=20 for f in doc['captures'] if f['status']=='accepted_candidate'
                                            for c in f['components'] if c['class']=='obstacle' and c['kept512'])
    return result


def compare(paths):
    docs={p.parent.name:json.loads(p.read_text()) for p in paths}
    summary={k:summarize(v) for k,v in docs.items()}
    pairs=defaultdict(dict)
    for name,doc in docs.items():
        for f in doc['captures']:
            key=(f.get('paired_source') or {}).get('source_id')
            if key:pairs[key][name]=f
    common=set.intersection(*(set(k for k,v in pairs.items() if name in v) for name in docs)) if docs else set()
    paired={name:summarize(dict(captures=[pairs[k][name] for k in sorted(common)],
                               outcomes={'accepted_candidate':len(common)})) for name in docs}
    return dict(inputs={str(p):sha256(p) for p in paths},summary=summary,
        common_pair_scenes=len(common),matched_identity_summary=paired,
        paired_scene_counts=dict(Counter(len(v) for v in pairs.values())),
        paired_scene_ids=list(pairs),
        limitation='Native sensor support/placement comparison only. No post-training AP, temporal stability or weather generalization claim.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reports',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=compare(a.reports);atomic_json(a.output,result)
    print(json.dumps(result['summary'],ensure_ascii=False,indent=2))
