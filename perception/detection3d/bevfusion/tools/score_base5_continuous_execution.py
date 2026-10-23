#!/usr/bin/env python3
"""CPU paired canonical XY AP against frozen bag catalog, not native GT.

All selected frames and zero-point catalog GT remain. Recorded catalog GT is
incomplete for background actors, so absolute AP is conditional, not a full
scene accuracy certificate. Never use GT to prune predictions.
"""
import argparse
import gzip
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'perception/detection3d/bevfusion/scripts'))
import score_base5_execution_pair as S


def rows_from(context,payload):
    if [r['token'] for r in payload['frames']]!=[r['token'] for r in context['rows']]:
        raise ValueError('Exact selected frame order required, including empty frames')
    rows=[]
    for source,prediction in zip(context['rows'],payload['frames']):
        if prediction['stamp']!=source['stamp'] or prediction['case']!=source['case']:
            raise ValueError('Original recorded frame identity differs')
        gt=np.asarray([r['bbox_3d'] for r in source['instances']],float).reshape(-1,7)
        gt[:,2]-=gt[:,5]/2  # Original catalog is geometric; decoder is bottom.
        detections=prediction['detections']
        rows.append(dict(token=source['token'],
            gt_boxes=gt,gt_labels=np.asarray([r['bbox_label_3d'] for r in source['instances']],int),
            gt_points=np.asarray([r['num_lidar_pts'] for r in source['instances']],int),
            pred_boxes=np.asarray([[*r['center'],*r['size'],r['yaw']] for r in detections],float).reshape(-1,7),
            pred_labels=np.asarray([r['label'] for r in detections],int),
            pred_scores=np.asarray([r['score'] for r in detections],float)))
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('context','run','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--reference-mode',default='camera_trt')
    p.add_argument('--candidate-mode',default='camera_trt_sdpa_position')
    p.add_argument('--restore-mode',default='camera_trt_restore')
    a=p.parse_args()
    if a.output.exists():p.error('Fresh AP output required')
    modes=(a.reference_mode,a.candidate_mode,a.restore_mode)
    if len(set(modes))!=3 or any(not m.replace('_','').isalnum() for m in modes):
        p.error('Three distinct simple modes required')
    files=[Path(__file__),Path(S.__file__),S.METRIC_PATH,a.context,a.run/'report.json',a.run/'COMPLETE.json']
    files.extend(a.run/(mode+'_raw_predictions.json.gz') for mode in modes)
    pins={str(path.resolve()):S.sha(path) for path in files}
    context=json.loads(a.context.read_text());report=json.loads((a.run/'report.json').read_text())
    complete=json.loads((a.run/'COMPLETE.json').read_text())
    if not (complete['complete'] and report['complete'] and complete['source_and_inputs_unchanged'] and
            complete['report_sha256']==S.sha(a.run/'report.json') and report['checkpoint_sha256']==S.E12_SHA256):
        raise ValueError('Completed frozen E12 comparison required')
    if S.sha(a.context) not in report['input_sha256'].values():raise ValueError('Context not pinned by inference')
    scores={};identities={}
    for mode in modes:
        payload=json.load(gzip.open(a.run/(mode+'_raw_predictions.json.gz'),'rt'))
        if payload['checkpoint_sha256']!=S.E12_SHA256 or payload['mode']!=mode:
            raise ValueError('Raw mode/checkpoint differs')
        identities[mode]=payload['input_manifest_sha256']
        rows=rows_from(context,payload)
        scores[mode]=dict(whole_ROI=S.ap(S.filtered(rows)),
            ranges={('%g_%gm'%band):S.ap(S.filtered(rows,band)) for band in S.BANDS},
            per_case={case:S.ap(S.filtered([row for source,row in zip(context['rows'],rows) if source['case']==case]))
                      for case in context['cases']})
    if len(set(identities.values()))!=1:raise ValueError('Input manifest changed between modes')
    ref=scores[modes[0]]
    changes={mode:dict(whole_ROI=S.delta(ref['whole_ROI'],scores[mode]['whole_ROI']),
        ranges={band:S.delta(ref['ranges'][band],scores[mode]['ranges'][band]) for band in ref['ranges']},
        per_case={case:S.delta(ref['per_case'][case],scores[mode]['per_case'][case]) for case in ref['per_case']})
        for mode in modes[1:]}
    if pins!={path:S.sha(path) for path in pins}:raise RuntimeError('AP source or input changed')
    result=dict(schema='base5_continuous_execution_catalog_AP_v1',complete=True,
        reference_mode=modes[0],scores=scores,delta_against_reference=changes,
        source_and_input_sha256=pins,GT_scope=context['GT'],frames=len(context['rows']),
        zero_point_GT_preserved=True,empty_frames_preserved=True,GT_prediction_pruning=False,
        evaluation=dict(ROI=S.ROI,AP_XY_gates_m=[.5,1.,2.,4.],
            predictions='Raw <=200 decoder outputs, no ROS floor/top100/NMS/class cutoffs',
            original_GT='Catalog geometry-center converted once to decoder bottom-center; never new native annotation',
            absent_GT='null AP and actual prediction count, not zero/perfect'),
        limitation='Incomplete catalog/bounded interpolation, known recorded development scenes; absolute AP is conditional. No GT IDF1 or competition safety claim.',
        production_changed=False)
    S.write_new(a.output,result)
    print(json.dumps(dict(complete=True,frames=result['frames'],output=str(a.output),delta=changes[modes[1]]['whole_ROI'])),flush=True)


if __name__=='__main__':main()
