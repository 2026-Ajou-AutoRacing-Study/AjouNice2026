#!/usr/bin/env python3
"""Catalog correspondence for private ROS replay; never compound FP/FN."""
import argparse
import gzip
import json
from pathlib import Path
from collections import Counter

from audit_tracking_continuity import TRACK,evaluate_gt
from analyze_live_perception_candidates import tracker_frames


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--outputs',type=Path,required=True)
    p.add_argument('--predictions',type=Path,required=True)
    p.add_argument('--case',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Fresh evidence required')
    rows=[r for r in json.load(gzip.open(a.predictions,'rt'))['rows'] if r['case']==a.case]
    if not rows:raise ValueError('Missing explicit case')
    _,gt=tracker_frames(rows)
    outputs=json.loads(a.outputs.read_text());report={}
    for mode,frames in outputs.items():
        small=[sum(o['class_name']=='pedestrian' and o['size'][2]<1.1 for o in f['objects']) for f in frames]
        tall=[sum(o['class_name']=='pedestrian' and o['size'][2]>=1.1 for o in f['objects']) for f in frames]
        row=dict(frames=len(frames),small_present_frames=sum(n>0 for n in small),
                 small_duplicate_frames=sum(n>1 for n in small),tall_present_frames=sum(n>0 for n in tall),
                 tall_duplicate_frames=sum(n>1 for n in tall))
        if all(r.get('actor_geometry_resolved',True) for r in rows):
            case={**gt,'frames':{TRACK:frames}}
            row['catalog_gt_gate2m']=evaluate_gt(case,2.)[0]
        else:row['gt_note']='Compound native component identity unavailable; height strata are diagnostics only.'
        report[mode]=row
    report['scope']='Actual installed ROS nodes, same frozen inputs and private clock. Catalog geometry, not visibility-qualified AP or live planner safety.'
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
