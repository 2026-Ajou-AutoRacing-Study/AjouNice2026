#!/usr/bin/env python3
"""Summarize controlled ROS policy A/B and provenance-separated recorded delivery."""
import argparse
import gzip
import json
from pathlib import Path

from analyze_base5_ros_tracking import compare
from compare_base5_common30 import digest


def compact(row):
    overall=row['overall']
    return dict(gt=overall['gt'],tp1m=overall['tp'],tp2m=overall['center2m']['tp'],
        unmatched2m=row['unmatched_persistence']['unmatched_gate2m'],
        unmatched_episodes_ge1s=row['unmatched_persistence']['episodes_ge_1s'],
        by_range={k:dict(gt=v['gt'],matched2m=v['matched_gt_counts']['2.0'],
            pred=v['pred_in_band'],unmatched2m=v['unmatched_predictions_by_center_gate']['2.0'])
            for k,v in row['by_range'].items()})


def summary(root):
    inventory=json.loads((root/'inventory_v1.json').read_text())
    delivery=json.loads((root/'recorded_delivery_v1.json').read_text())
    result=dict(scope='Bag-based tracker review, not closed-loop planner certification or new model accuracy.',
        inventory={k:v for k,v in inventory.items() if k!='rows'},
        recorded_delivery=delivery,experiments={},source_sha256={})
    for name,file in [('promotion_ros_v2','promotion_metrics_v1.json'),('birth_guard_ros_v1','birth_guard_metrics_v1.json')]:
        receipt=json.loads((root/name/'receipt.json').read_text())
        if (receipt['checkpoint_sha256']!='84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746'
                or receipt['comparison_kind']!='tracker_policy' or receipt['candidate_coast_times'] is not None
                or receipt['reference_thresholds']!=[.4,.1,.1]
                or receipt['reference_thresholds']!=receipt['candidate_thresholds']
                or len(set(receipt['input_sha256'].values()))!=1
                or receipt['binary_sha256']!='7dda240b5e3331ea2b3322eb6e2403c2e8e1616d8dd665b95d31c7baf80ca3d1'):
            raise ValueError('Policy comparison changed its frozen source/options/weights')
        metric=json.loads((root/file).read_text())['tracking'][name]
        with gzip.open(root/name/'messages.json.gz','rt') as f:messages=json.load(f)
        cases={}
        for case in metric['off']['cases']:
            control,_=compare(messages[case+'/off/policy'],messages[case+'/off_repeat/policy'],.5)
            all_stats=control['by_class']['all']
            if (control['reference_only_timestamps'] or control['candidate_only_timestamps']
                    or all_stats['added'] or all_stats['removed']
                    or (all_stats['distributions']['center_xy_m']['maximum'] or 0)!=0):
                raise ValueError('Independent repeat is not identical; do not attribute differences to policy')
            cases[case]={mode:{c:compact(r) for c,r in metric[mode]['cases'][case].items()} for mode in ('off','on')}
        result['experiments'][name]=dict(receipt=receipt,reference_repeat_identical=True,cases=cases)
        for p in (root/file,root/name/'receipt.json',root/name/'messages.json.gz'):
            result['source_sha256'][str(p)]=digest(p)
    for p in (root/'inventory_v1.json',root/'recorded_delivery_v1.json'):result['source_sha256'][str(p)]=digest(p)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError('Fresh summary required')
    value=summary(a.root);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    print('Saved',a.output)
