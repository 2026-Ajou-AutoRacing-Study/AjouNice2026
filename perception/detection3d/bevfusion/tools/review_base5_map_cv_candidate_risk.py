#!/usr/bin/env python3
"""Group-level and rare-regression review; aggregate gates are not deployment gates."""
import argparse
from collections import defaultdict
import gzip
import json
from pathlib import Path

import numpy as np

from audit_base5_predicted_path_selection import stats
from experiment_base5_map_cv_candidates import ROOT, digest, history_fit, history_rank, load_dev, prepare_network
from probe_base5_map_lane_history import future_point


def summarize(result,candidate='route_history'):
    groups=defaultdict(list); deltas={}; events={}
    for row in result['rows']:
        if row['errors'][candidate]['minK']!=row['errors']['baseline']['minK']:
            raise ValueError('Reranking changed available geometric GT coverage')
        groups[(tuple(row['gt_key']),row['horizon'])].append(row)
    for horizon in (1,2,3):
        selected=[r for r in result['rows'] if r['horizon']==horizon]
        values=[r['errors'][candidate]['top1']-r['errors']['baseline']['top1'] for r in selected]
        deltas[str(horizon)]=dict(all_observations=stats(values),
            changed_only=stats([v for v in values if abs(v)>1.e-8]),
            extra_error_gt5m=sum(v>5 for v in values),improvement_gt5m=sum(v<-5 for v in values))
        episodes=[]
        for (key,h),rows in groups.items():
            if h!=horizon:continue
            active=[]
            for row in sorted(rows,key=lambda r:r['stamp']):
                bad=row['errors'][candidate]['top1']-row['errors']['baseline']['top1']>5
                if active and (not bad or row['stamp']-active[-1]['stamp']>.25 or row['id']!=active[-1]['id']):
                    episodes.append(dict(gt_key=list(key),count=len(active),start=active[0]['stamp'],end=active[-1]['stamp']))
                    active=[]
                if bad:active.append(row)
            if active:episodes.append(dict(gt_key=list(key),count=len(active),start=active[0]['stamp'],end=active[-1]['stamp']))
        events[str(horizon)]=episodes
    actors=[]
    for (key,horizon),rows in groups.items():
        if horizon!=3 or len(rows)<10:continue
        differences=[r['errors'][candidate]['top1']-r['errors']['baseline']['top1'] for r in rows]
        actors.append(dict(gt_key=list(key),n=len(rows),mean_change=float(np.mean(differences)),
                           delta=stats(differences)))
    return dict(deltas=deltas,severe_episodes=events,actors3s=actors,
                actor_groups_improve_gt01=sum(a['mean_change']<-.1 for a in actors),
                actor_groups_worsen_gt01=sum(a['mean_change']>.1 for a in actors),
                minK_exact=True)


def render(network,frames,gt,case,row,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    history=[]; obj=None
    for frame in frames:
        if frame['stamp']>row['stamp']+1.e-6:break
        found=next((o for o in frame['objects'] if o['id']==row['id'] and o['class_name']=='vehicle'),None)
        if found is None:continue
        stamp=frame['stamp']
        if history and (stamp<=history[-1][0] or stamp-history[-1][0]>.15):history=[]
        history=[p for p in history if stamp-p[0]<=.5]+[(stamp,*found['center'][:2])]
        if abs(stamp-row['stamp'])<1.e-6:obj=found
    if obj is None:raise ValueError('Review state missing')
    fit=history_fit(history,row['stamp']);x,y,z=obj['center']
    paths=network.predict(x,y,z,obj['yaw'],obj['body_velocity'][0],tuple(i/10 for i in range(1,31)))
    ranked=history_rank(network,paths,fit,obj)
    series=gt[1][tuple(row['gt_key'])]
    actual=[future_point(series,row['stamp']+i/10) for i in range(31)]
    actual=[p for p in actual if p is not None]
    for name,index in (('baseline',0),('route_history',0)):
        chosen=paths[index] if name=='baseline' else ranked[index]
        future=future_point(series,row['stamp']+3)
        error=float(np.linalg.norm(np.asarray(chosen.poses[-1][:2])-np.asarray(future[:2])))
        if abs(error-row['errors'][name]['top1'])>1.e-6:raise ValueError('Review replay differs from frozen metrics')
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for p in paths:
        axes[0].plot([v[0]-x for v in p.poses],[v[1]-y for v in p.poses],color='0.8',lw=1)
    for label,p,color in (('Baseline Top-1',paths[0],'tab:orange'),('History Top-1',ranked[0],'tab:green')):
        axes[0].plot([x0-x for x0,_,_,_ in p.poses],[y0-y for _,y0,_,_ in p.poses],label=label,color=color,lw=2)
    axes[0].plot([p[0]-x for p in actual],[p[1]-y for p in actual],label='Catalog GT future',color='tab:blue',lw=2)
    axes[0].scatter([p[1]-x for p in history],[p[2]-y for p in history],label='Past track states',color='black',s=15)
    axes[0].scatter([0],[0],marker='*',s=100,color='black')
    axes[0].set_aspect('equal',adjustable='datalim');axes[0].legend(fontsize=8)
    axes[0].set_xlabel('Relative map X (m)');axes[0].set_ylabel('Relative map Y (m)');axes[0].grid(alpha=.3)
    relevant=[r for r in render.all_rows if r['gt_key']==row['gt_key'] and r['horizon']==3]
    relevant.sort(key=lambda r:r['stamp']);origin=relevant[0]['stamp']
    for name,color in (('baseline','tab:orange'),('route_history','tab:green')):
        axes[1].plot([r['stamp']-origin for r in relevant],[r['errors'][name]['top1'] for r in relevant],label=name,color=color)
    axes[1].axvline(row['stamp']-origin,color='black',ls='--',lw=1)
    axes[1].set_xlabel('Time within scored GT segment (s)');axes[1].set_ylabel('3s forecast position error (m)')
    axes[1].legend();axes[1].grid(alpha=.3)
    change=row['errors']['route_history']['top1']-row['errors']['baseline']['top1']
    fig.suptitle(f'{case}: extra 3s error {change:+.2f}m; future GT used for evaluation only')
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(output,dpi=150);plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps(dict(row=row,history=history,
        baseline_route=paths[0].lanelet_ids,candidate_route=ranked[0].lanelet_ids,
        baseline_confidence=paths[0].confidence,candidate_confidence=ranked[0].confidence),indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--development',type=Path,required=True)
    parser.add_argument('--regression',type=Path,required=True)
    parser.add_argument('--pack',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Fresh review required')
    args.output.mkdir(parents=True)
    dev=json.loads((args.development/'development.json').read_text());reg=json.loads(args.regression.read_text())
    if digest(args.pack)!=reg['pack_sha256']:raise ValueError('Regression pack receipt mismatch')
    reports={**dev['results'],**reg['results']}
    summary={k:summarize(v) for k,v in reports.items()}
    receipt=dict(source_sha256=digest(__file__),summary=summary,production_enabled=False,
                 decision='Aggregate screen passed, but rare large new Top-1 errors prevent deployment. '
                          'No post-regression tuning in this experiment.')
    (args.output/'risk.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    for case,s in summary.items():
        print(case,'3s severe worsening',s['deltas']['3']['extra_error_gt5m'],
              'actors improved/worsened',s['actor_groups_improve_gt01'],s['actor_groups_worsen_gt01'],flush=True)
    import yaml
    from perception_object_pipeline.map_based_prediction import LaneletNetwork
    alignment=yaml.safe_load((ROOT/'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
    network=LaneletNetwork.load(ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm',ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    prepare_network(network)
    cases,gt=load_dev(ROOT/'artifacts/evaluation/bevfusion/base5_production_tracker_review_261013/promotion_ros_v2/messages.json.gz',
                      ROOT/'artifacts/evaluation/bevfusion/base5_far_bag_retention_261012/inputs_v1',ROOT/'configs/sensors/Sensor_4.json',alignment)
    for direction in ('worse','better'):
        candidates=[(case,row) for case in dev['results'] for row in dev['results'][case]['rows'] if row['horizon']==3]
        case,row=(max if direction=='worse' else min)(candidates,key=lambda x:x[1]['errors']['route_history']['top1']-x[1]['errors']['baseline']['top1'])
        render.all_rows=dev['results'][case]['rows']
        render(network,cases[case],gt,case,row,args.output/f'development_{direction}.png')


if __name__=='__main__':main()
