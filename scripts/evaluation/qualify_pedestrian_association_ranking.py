#!/usr/bin/env python3
"""Frozen-input qualification of the deployed pedestrian rank switch.

No simulator or production parameters are mutated. OFF is compared to a
pre-edit core binary; compound outputs are shape diagnostics, not FP/FN GT.
"""
import argparse
import gzip
import json
from pathlib import Path
import sys
import yaml

from audit_tracking_continuity import DET, read_case
from audit_tracking_continuity_events import build_original, core_summary, load, replay, sha
from audit_tracking_continuity_saved_outputs import nonped_states_equal
from analyze_live_perception_candidates import tracker_frames, shape_summary, motion_summary

ROOT = Path(__file__).resolve().parents[2]


def corrected_predictions(prediction, gt_inputs=None):
    with gzip.open(prediction, 'rt') as stream:
        payload = json.load(stream)
    rows = payload['rows']
    if not rows or len({r['token'] for r in rows}) != len(rows):
        raise ValueError('Nonempty unique frozen tokens required')
    if gt_inputs:
        lookup = {r['token']: r for r in json.loads(gt_inputs.read_text())['rows']}
        if set(lookup) != {r['token'] for r in rows}:
            raise ValueError('GT correction changes cohort')
        for row in rows:
            source = lookup[row['token']]
            for key in ('stamp', 'point_sha256', 'image_sha256'):
                if row[key] != source[key]:
                    raise ValueError('Correction changes sensor inputs')
            row.update({k:v for k,v in source.items() if k.startswith('gt_') or k in
                ('ego', 'parent_uids', 'actor_geometry_resolved', 'heading_correction_deg')})
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--frozen-legacy-core',type=Path,required=True)
    p.add_argument('--live-prediction',type=Path,action='append',default=[])
    p.add_argument('--gt-inputs',type=Path,action='append',default=[])
    p.add_argument('--bag',type=Path,action='append',default=[])
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Fresh output required')
    if len(a.gt_inputs) not in (0,len(a.live_prediction)):
        raise ValueError('One GT input for each frozen live prediction required')
    a.output.mkdir(parents=True)
    params=yaml.safe_load((ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())['configure']
    binaries={k:build_original(a.output/('core_'+k),params,policy)[0] for k,policy in
        [('off',None),('on',dict(gaussian_xy_ranking='pedestrian'))]}
    cases={}
    for index,prediction in enumerate(a.live_prediction):
        rows=corrected_predictions(prediction,a.gt_inputs[index] if a.gt_inputs else None)
        for case in sorted({r['case'] for r in rows}):
            selected=[r for r in rows if r['case']==case]
            frames,gt=tracker_frames(selected)
            cases[f'live{index}_{case}']=(frames,gt if all(r.get('actor_geometry_resolved',True) for r in selected) else None,
                                         dict(prediction=str(prediction),sha256=sha(prediction)))
    for index,bag in enumerate(a.bag):
        _,frames,excluded=load(bag)
        gt=read_case(bag,assisted=True)
        cases[f'bag{index}_{bag.stem}']=(frames[DET],gt,dict(bag=str(bag),sha256=sha(bag),excluded_tf=len(excluded)))
    if not cases:raise ValueError('At least one explicit frozen input case required')
    report=dict(cases={},frozen_legacy_core_sha256=sha(a.frozen_legacy_core),
        scope='Same fixed detections, actual C++ core OFF/ON. No new AP, learned pipeline latency, or closed-loop certificate.')
    for name,(frames,gt,provenance) in cases.items():
        outputs={k:replay(binary,frames) for k,binary in binaries.items()}
        legacy=replay(a.frozen_legacy_core,frames)
        rollback_exact=legacy==outputs['off']
        nonped_exact=nonped_states_equal(outputs['off'],outputs['on'])
        row=dict(provenance=provenance,frames=len(frames),rollback_exact=rollback_exact,
                 nonpedestrian_states_exact_ignoring_id_counter=nonped_exact,results={})
        for mode,output in outputs.items():
            row['results'][mode]=core_summary(output,gt)
            row['results'][mode]['shape_diagnostic']=shape_summary(output)
            if gt is not None:row['results'][mode]['motion_diagnostic']=motion_summary(output,gt)
            with gzip.open(a.output/(name+'_'+mode+'.json.gz'),'wt') as stream:json.dump(output,stream)
        with gzip.open(a.output/(name+'_frames.json.gz'),'wt') as stream:json.dump(frames,stream)
        report['cases'][name]=row
        (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(case=name,frames=len(frames),rollback_exact=rollback_exact,nonped_exact=nonped_exact,
                              ids={m:r['confirmed_unique_ids'] for m,r in row['results'].items()})),flush=True)
        if not rollback_exact or not nonped_exact:
            raise RuntimeError('Rollback or non-pedestrian regression; do not promote')


if __name__=='__main__':main()
