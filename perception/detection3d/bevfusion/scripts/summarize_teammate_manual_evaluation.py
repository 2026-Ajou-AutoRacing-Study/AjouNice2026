#!/usr/bin/env python3
"""Summarize completed manual-bag evaluations without averaging quantiles."""
import argparse
import json
from pathlib import Path
from statistics import mean

ROOT = Path(__file__).resolve().parents[4]


def extract(case, suffix=''):
    contract = json.loads((case / 'case_contract.json').read_text())
    detection = json.loads((case / f'summary{suffix}.json').read_text())
    pipeline = json.loads((case / f'pipeline/summary{suffix}.json').read_text())
    stable = json.loads((case / f'pipeline/summary_cv_stable{suffix}.json').read_text())
    cls = pipeline['evaluation']['class']
    quality = detection['metrics']['overall']['classes'][cls]
    track = pipeline['tracking']
    calibration = pipeline['evaluation']['gt_reference_calibration']
    row = dict(model=contract['model'], bag=case.name, mission=contract['mission'],
        role=contract['role'], center_ap=quality['mean_ap'],
        tracking_f1=track['f1'], id_switches=track['id_switches'],
        center_p50_m=track['bbox_center_absolute_distance_m'].get('p50'),
        speed_mae_mps=track['absolute_speed_error_mps'].get('mean'),
        speed_p50_mps=track['absolute_speed_error_mps'].get('p50'),
        speed_p95_mps=track['absolute_speed_error_mps'].get('p95'),
        ave_mps=track['velocity_l2_error_mps'].get('mean'),
        ave_p50_mps=track['velocity_l2_error_mps'].get('p50'),
        ave_p95_mps=track['velocity_l2_error_mps'].get('p95'),
        gt_segments=calibration['input_segment_count'],
        supported_gt_segments=calibration['supported_segment_count'],
        resolved_frame_policy=detection.get('resolved_frame_policy'),
        coverage=detection['runtime_coverage'],
        detector_speed_mae_mps=quality['tp_quality_2m']['speed_error_mps'].get('mean'),
        detector_speed_p50_mps=quality['tp_quality_2m']['speed_error_mps'].get('p50'),
        detector_speed_p95_mps=quality['tp_quality_2m']['speed_error_mps'].get('p95'),
        detector_longitudinal_mae_mps=quality['tp_quality_2m']['longitudinal_velocity_error_mps'].get('mean'),
        detector_ave_mps=quality['tp_quality_2m']['velocity_l2_error_mps'].get('mean'))
    for h in ['1','2','3']:
        metrics=pipeline['prediction']['multimodal']['horizons'][h]
        row[f'fde{h}_m']=metrics['top1_fde_m'].get('mean')
        row[f'fde{h}_p50_m']=metrics['top1_fde_m'].get('p50')
        row[f'fde{h}_p95_m']=metrics['top1_fde_m'].get('p95')
        row[f'prediction{h}_coverage']=pipeline['prediction']['fde_m'][h].get('coverage')
        row[f'cv_stable_fde{h}_m']=stable['prediction']['multimodal']['horizons'][h]['top1_fde_m'].get('mean')
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT/'artifacts/evaluation/bevfusion/teammate_manual_261004')
    parser.add_argument('--resolved',action='store_true',help='Use conservative GT-resolution frame subset')
    parser.add_argument('--suffix',help='Custom summary suffix (e.g. _status_size_v2)')
    args=parser.parse_args()
    rows=[]
    suffix=args.suffix if args.suffix is not None else ('_resolved_frames' if args.resolved else '')
    for contract in sorted(args.root.glob('*/*/case_contract.json')):
        case=contract.parent
        if (case/f'pipeline/summary_cv_stable{suffix}.json').is_file():
            rows.append(extract(case,suffix))
    groups={}
    metrics=['center_ap','tracking_f1','speed_mae_mps','ave_mps','fde1_m','fde2_m','fde3_m',
             'cv_stable_fde3_m','prediction3_coverage','detector_speed_mae_mps','detector_longitudinal_mae_mps','detector_ave_mps']
    for model in ['e25','p7']:
        for mission in ['highway','roundabout','pedestrian','obstacle','highway_locked']:
            members=[r for r in rows if r['model']==model and
                (r['mission']==mission or mission=='highway_locked' and
                 r['mission']=='highway' and r['role']=='evaluation_locked')]
            if not members:continue
            groups[f'{model}/{mission}']=dict(bags=len(members),
                id_switches=sum(r['id_switches'] for r in members),
                **{metric:mean([r[metric] for r in members if r[metric] is not None])
                   if any(r[metric] is not None for r in members) else None for metric in metrics})
    result=dict(aggregation='Unweighted bag macro means; quantiles reported per bag only. '
        'IDs summed. No pooled AP or pooled error quantiles claimed.',rows=rows,groups=groups)
    (args.root/f'aggregate{suffix}.json').write_text(json.dumps(result,indent=2)+'\n')
    lines=['| Model / bag | Center AP % | Track F1 % | IDs | Center p50 m | Speed MAE m/s | 2D AVE m/s | FDE@1/2/3s m |',
           '| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |']
    def fmt(v):return '—' if v is None else f'{v:.3f}'
    for r in rows:
        lines.append(f"| {r['model']} {r['bag']} | {r['center_ap']*100:.2f} | {r['tracking_f1']*100:.2f} | {r['id_switches']} | "
            + ' | '.join(fmt(r[k]) for k in ['center_p50_m','speed_mae_mps','ave_mps'])+' | '
            + ' / '.join(fmt(r[f'fde{h}_m']) for h in ['1','2','3'])+' |')
    (args.root/f'tables{suffix}.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    print(json.dumps(groups,indent=2))


if __name__=='__main__':
    main()
