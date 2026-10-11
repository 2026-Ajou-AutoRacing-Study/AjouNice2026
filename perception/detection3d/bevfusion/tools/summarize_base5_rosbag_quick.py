#!/usr/bin/env python3
"""Compare common-frame detection pilots, never infer tracking/dropout duration."""
import argparse
import gzip
import json
from pathlib import Path
import sys
from compare_base5_common30 import evaluate, validate_payload, CLASSES

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'base_ws/src/perception/bevfusion_ros/src'))
from bevfusion_ros.rotated_bev_nms import class_aware_rotated_bev_nms


def bridge_nms(row):
    """Mirror server floor/top100 and bridge NMS; keep score curves for AP.

    Actual class thresholds are applied by evaluate's fixed-score P/R. A box
    below that class threshold cannot suppress a higher-scoring eligible box.
    """
    indices=sorted(range(len(row['pred_scores'])),key=lambda i:-row['pred_scores'][i])
    indices=[i for i in indices if row['pred_scores'][i]>=.1][:100]
    detections=[dict(center=row['pred_boxes'][i][:3],size=row['pred_boxes'][i][3:6],
                     yaw=row['pred_boxes'][i][6],score=row['pred_scores'][i],
                     class_name=CLASSES[row['pred_labels'][i]],index=i) for i in indices]
    kept=[d['index'] for d in class_aware_rotated_bev_nms(detections,.5)]
    result=dict(row)
    for key in ('pred_boxes','pred_labels','pred_scores'):
        result[key]=[row[key][i] for i in kept]
    return result


def summarize(inputs, predictions, output):
    if output.exists():raise ValueError('Fresh summary output required')
    manifest=json.loads((inputs/'manifest.json').read_text())
    result=dict(scope='Same development bag LiDAR tokens; radial LiDAR30 intersected with common E25 box',
        frames=manifest['frames'],cases=manifest['cases'],models={},
        postprocessing='Primary: server score floor0.1/top100 then production class-aware rotated BEV NMS IoU0.5. AP keeps score curves; fixed P/R uses V/P/O0.4/0.15/0.1.',
        limitations=['Subsampled single-frame detection; no tracker or live ROS timing certification.',
                     'Status/camera synchronization and interpolated catalog GT are approximate, not native annotations.',
                     'All supported nearby GT kept; no LiDAR-point visibility deletion.',
                     'Previously evaluated development bags, not an untouched held-out test.'])
    identity=None
    for path in predictions:
        with gzip.open(path,'rt') as f:payload=json.load(f)
        validate_payload(payload)
        if payload['frames']!=manifest['frames']:
            raise ValueError('Prediction frame count differs from input package')
        if identity is not None and identity!=payload['gt_identity']:
            raise ValueError('Different scenes/GT across models')
        identity=payload['gt_identity']
        model={}
        for case,details in manifest['cases'].items():
            rows=[r for r in payload['rows'] if r['token'].startswith(case+'_')]
            if len(rows)!=details['selected']:raise ValueError('Missing case inference frames')
            evaluated=evaluate([bridge_nms(r) for r in rows])
            raw_evaluated=evaluate(rows)
            target=details['mission'] if details['mission'] in ('pedestrian','obstacle') else 'vehicle'
            metrics=evaluated['metrics']; score={'vehicle':.4,'pedestrian':.15,'obstacle':.1}[target]
            model[case]=dict(class_name=target,frames=len(rows),gt=metrics['center/'+target+'_GT'],
                center_AP=metrics['center/'+target+'_mean_AP'],
                operating_point=evaluated['support_recall'][f'{target}/score{score:g}_1m'],
                raw_decoder_AP=raw_evaluated['metrics']['center/'+target+'_mean_AP'],
                all_classes_metrics=metrics,scope='radial30')
        name=payload['name']
        if name in result['models']:raise ValueError('Duplicate model name')
        result['models'][name]=dict(receipt={k:v for k,v in payload.items() if k!='rows'},cases=model,
                                   pooled=evaluate([bridge_nms(r) for r in payload['rows']]))
    result['gt_identity']=identity
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for name,model in result['models'].items():
        print(name,{case:round(row['center_AP']*100,2) for case,row in model['cases'].items()})
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--predictions',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();summarize(a.inputs,a.predictions,a.output)
