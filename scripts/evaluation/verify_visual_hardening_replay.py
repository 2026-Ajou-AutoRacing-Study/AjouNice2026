#!/usr/bin/env python3
"""Recheck final identity helper against saved GPU predictions; no model calls.

This checks restrictive confidence/pairwise hardening does not change the already
visually reviewed accepted identities. CNN/vehicle behavior is not rerun here.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'base_ws/src/perception/traffic_light_detection'
sys.path.insert(0,str(PACKAGE/'scripts'))
from traffic_light_visual_continuation import VisualContinuation
from replay_signal_bag_projection_samples import selection_boxes


def run(source,output):
    data=json.loads(source.read_text());input_path=Path(data['source_inputs'])
    inputs=json.loads(input_path.read_text());packets={r['source_stamp_ns']:r['map_packet'] for r in inputs['records']}
    tracker=VisualContinuation();checks=[];normal=0
    for row in data['records']:
        item=row['outcomes']['current'];decision=item['decision'];selection=item['selection']
        source_ns=row['source_stamp_ns'];now=row['receipt_stamp_ns'];group=row['group_id']
        boxes=selection_boxes(row['detector_predictions'].get('960',[]),1280,720)
        if (decision['near_rescue'].get('attempted') or
                decision.get('visual_continuation',{}).get('retry640_attempted')):
            boxes+=selection_boxes(row['detector_predictions'].get('640',[]),1280,720)
        if selection['status']=='matched' and selection['reason']=='map_head_association':
            tracker.remember(boxes[selection['box_index']],selection,group,source_ns,now);normal+=1
            continue
        packet=packets[source_ns]
        primary=dict(status='missing',reason=(packet['reason'] if packet['status']=='missing'
                                             else 'no_unambiguous_route_detection'),rejected=[])
        selected,current=tracker.select(boxes,packet,primary,group,source_ns,now,.3)
        previous=decision.get('visual_continuation',{})
        if previous.get('status')=='matched':
            ok=(selected is not None and current.get('status')=='matched'
                and current.get('head_id')==previous['head_id']
                and list(selected[4][:4])==item['classifier']['detector_bbox_xyxy'])
            checks.append(dict(t=row['t'],section=row['section'],head_id=previous['head_id'],unchanged=ok))
            if not ok:raise ValueError('Previously reviewed acceptance changed at '+str(row['t']))
    output=output.resolve();output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh report required')
    output.mkdir(parents=True)
    helper=PACKAGE/'scripts/traffic_light_visual_continuation.py'
    report=dict(source_replay=str(source),helper_sha256=hashlib.sha256(helper.read_bytes()).hexdigest(),
        normal_anchors=normal,accepted_identities_checked=len(checks),all_unchanged=all(r['unchanged'] for r in checks),
        records=checks,limitations=['Saved GPU predictions only; no new inference/native GT/closed-loop claim.',
        'Positive acceptance identity and actual detector crop equivalence; adversarial cases are unit-tested separately.'])
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key not in ('records','limitations')}))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.source,a.output)
