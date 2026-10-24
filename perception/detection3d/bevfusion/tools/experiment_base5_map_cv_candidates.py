#!/usr/bin/env python3
"""Preregistered CPU-only Map-CV experiments. Never changes production paths."""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
from dataclasses import replace
import gzip
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from audit_base5_predicted_path_selection import stats
from probe_base5_map_lane_history import lanelet_coordinates, future_point, prepare_gt

ROOT = Path(__file__).resolve().parents[4]
VARIANTS = ('baseline', 'route_history', 'convergence3', 'lateral_hermite3')
SPEC = dict(variants=VARIANTS, history_sec=.5, history_min_span=.3,
            history_max_gap=.15, history_min_points=4, fit_rms_limit_m=.4,
            lateral_velocity_cap_mps=2., convergence_sec=3., history_blend=.5,
            gate_mean_ratio=1.03, gate_p95_ratio=1.05, minimum_eval_observations=30,
            rule='Require no mean/p95 regression beyond the fixed gates per case/horizon. '
                 'Select on H03/R01 only; freeze at most one candidate before reserved bags. '
                 'No further tuning after reserved metrics. Known development data, not untouched test.')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def history_fit(history, stamp):
    recent = [p for p in history if 0 <= stamp-p[0] <= SPEC['history_sec']]
    if len(recent)<SPEC['history_min_points'] or recent[-1][0]-recent[0][0]<SPEC['history_min_span']-1e-6:
        return None
    if any(b[0]-a[0]>SPEC['history_max_gap'] for a,b in zip(recent,recent[1:])):
        return None
    t=np.asarray([p[0]-stamp for p in recent]); xy=np.asarray([p[1:3] for p in recent])
    design=np.column_stack((np.ones_like(t),t))
    coefficients=np.linalg.lstsq(design,xy,rcond=None)[0]
    rms=float(np.sqrt(np.mean(np.sum((design@coefficients-xy)**2,axis=1))))
    if rms>SPEC['fit_rms_limit_m']:
        return None
    return dict(position=coefficients[0],velocity=coefficients[1],rms=rms,recent=recent)


def history_rank(network, trajectories, fit, obj=None):
    """Past start-lane fit only. Identical start-lane siblings get equal evidence.

    One upstream segment is allowed to avoid penalizing normal lanelet progress.
    No previous favorite ID and no future GT is used.
    """
    from perception_object_pipeline.map_based_prediction import project_point_to_polyline
    if fit is None or not trajectories:
        return trajectories
    predecessors=network._probe_predecessors
    current_scores={c.lanelet_id:c.score for c in network.lane_candidates(
        obj['center'][0],obj['center'][1],obj['yaw'])}
    evidence={}
    for trajectory in trajectories:
        start=trajectory.lanelet_ids[0]
        if start in evidence:
            continue
        lines=[network.records[i].centerline for i in (start,*predecessors.get(start,()))]
        residuals=[]; weights=[]
        last=fit['recent'][-1][0]
        for stamp,x,y in fit['recent']:
            residuals.append(min(project_point_to_polyline(line,x,y).distance for line in lines)**2)
            weights.append(math.exp((stamp-last)/.25))
        error=float(np.average(residuals,weights=weights))
        evidence[start]=max(1.e-12,math.exp(-.5*error/network.config.lane_distance_sigma_m**2))
    # Modify start-lane evidence only. Keep conditional sibling branch ratios.
    scores=[p.confidence*(evidence[p.lanelet_ids[0]]/
            max(current_scores[p.lanelet_ids[0]],1.e-12))**SPEC['history_blend'] for p in trajectories]
    total=sum(scores)
    return tuple(sorted((replace(p,confidence=s/total) for p,s in zip(trajectories,scores)),
                        key=lambda p:p.confidence,reverse=True))


def lateral_paths(network, trajectories, obj, fit):
    from perception_object_pipeline.map_based_prediction import project_point_to_polyline, sample_polyline, tangent_yaw
    if fit is None:
        return trajectories
    x,y,z=obj['center']; speed=obj['body_velocity'][0]; result=[]; duration=SPEC['convergence_sec']
    for trajectory in trajectories:
        route=network._stitch_route(trajectory.lanelet_ids)
        origin=project_point_to_polyline(route,x,y)
        c,s=math.cos(origin.yaw),math.sin(origin.yaw)
        longitudinal=(x-origin.x)*c+(y-origin.y)*s
        lateral=-(x-origin.x)*s+(y-origin.y)*c
        vx,vy=fit['velocity']
        lateral_speed=float(np.clip(-vx*s+vy*c,-SPEC['lateral_velocity_cap_mps'],SPEC['lateral_velocity_cap_mps']))
        poses=[]
        for i in range(1,31):
            h=i/10; u=min(1.,h/duration)
            d=(2*u**3-3*u**2+1)*lateral+(u**3-2*u**2+u)*duration*lateral_speed
            residual=max(0.,1-h/duration)
            center=sample_polyline(route,origin.arc_length+speed*h)
            yaw=tangent_yaw(route,origin.arc_length+speed*h)
            poses.append((center[0]-math.sin(yaw)*d+math.cos(yaw)*longitudinal*residual,
                          center[1]+math.cos(yaw)*d+math.sin(yaw)*longitudinal*residual,
                          center[2]+(z-origin.z)*residual,yaw))
        result.append(replace(trajectory,poses=tuple(poses)))
    return tuple(result)


def prepare_network(network):
    predecessors=defaultdict(list)
    for ident,record in network.records.items():
        for successor in record.following_ids:
            predecessors[successor].append(ident)
    network._probe_predecessors={k:tuple(sorted(v)) for k,v in predecessors.items()}


def evaluate(network, frames, gt, case, variants):
    from analyze_base5_runtime_effects import match
    from perception_object_pipeline.map_based_prediction import LaneletNetwork
    slow=LaneletNetwork(tuple(network.records.values()),replace(network.config,lateral_convergence_time_sec=3.),
                        map_sha256=network.map_sha256)
    histories={}; counts=Counter(); rows=[]; cost=defaultdict(list); speed_cohorts=Counter()
    for frame in frames:
        stamp=frame['stamp']; vehicles=[o for o in frame['objects'] if o['class_name']=='vehicle']
        observations=gt[0].get((case,round(stamp,6)),[])
        pairs,_,_=match(vehicles,observations,2.)
        assignment={vehicles[i]['id']:observations[j]['key'] for i,j in pairs}
        for obj in vehicles:
            counts['vehicle_observations']+=1
            ident=obj['id']; x,y,z=obj['center']; speed=obj['body_velocity'][0]
            old=histories.get(ident,[])
            if old and (stamp<=old[-1][0] or stamp-old[-1][0]>.15):old=[]
            history=[p for p in old if stamp-p[0]<=.5]+[(stamp,x,y)]
            histories[ident]=history
            fit=history_fit(history,stamp)
            counts['history_fit_supported' if fit else 'history_fit_fallback']+=1
            begin=time.perf_counter()
            baseline=network.predict(x,y,z,obj['yaw'],speed,tuple(i/10 for i in range(1,31)))
            cost['baseline'].append((time.perf_counter()-begin)*1000)
            if not baseline:
                counts['map_fallback_observations']+=1
                continue
            outputs={'baseline':baseline}
            for variant in variants:
                if variant=='baseline':continue
                begin=time.perf_counter()
                if variant=='route_history':outputs[variant]=history_rank(network,baseline,fit,obj)
                elif variant=='convergence3':
                    outputs[variant]=slow.predict(x,y,z,obj['yaw'],speed,tuple(i/10 for i in range(1,31)))
                elif variant=='lateral_hermite3':outputs[variant]=lateral_paths(network,baseline,obj,fit)
                else:raise ValueError(variant)
                cost[variant].append((time.perf_counter()-begin)*1000)
                if not outputs[variant]:raise ValueError('Candidate removed baseline map coverage')
                if {p.lanelet_ids for p in outputs[variant]}!={p.lanelet_ids for p in baseline}:
                    counts[variant+'/route_set_changed']+=1
                if outputs[variant][0].lanelet_ids!=baseline[0].lanelet_ids:
                    counts[variant+'/top1_route_changed']+=1
            if ident not in assignment:continue
            series=gt[1][assignment[ident]]
            for horizon in (1,2,3):
                future=future_point(series,stamp+horizon)
                if future is None:continue
                errors={}
                for variant,paths in outputs.items():
                    values=[math.hypot(p.poses[horizon*10-1][0]-future[0],p.poses[horizon*10-1][1]-future[1]) for p in paths]
                    errors[variant]=dict(top1=values[0],minK=min(values))
                rows.append(dict(stamp=stamp,id=ident,gt_key=list(assignment[ident]),horizon=horizon,
                                 speed=abs(speed),history_fit=fit is not None,errors=errors))
    metrics={}
    for variant in variants:
        metrics[variant]={str(h):{m:stats([r['errors'][variant][m] for r in rows if r['horizon']==h])
                                  for m in ('top1','minK')} for h in (1,2,3)}
    effects={variant:{str(h):dict(improved_gt05=sum(r['errors']['baseline']['top1']-r['errors'][variant]['top1']>.5 for r in rows if r['horizon']==h),
                worsened_gt05=sum(r['errors'][variant]['top1']-r['errors']['baseline']['top1']>.5 for r in rows if r['horizon']==h))
                for h in (1,2,3)} for variant in variants if variant!='baseline'}
    return dict(counts=dict(counts),metrics=metrics,effects=effects,
                incremental_cost_ms={k:stats(v) for k,v in cost.items()},rows=rows)


def gates(results, candidate):
    checks={}
    for case,result in results.items():
        for horizon in ('1','2','3'):
            old=result['metrics']['baseline'][horizon]['top1']; new=result['metrics'][candidate][horizon]['top1']
            if old['n']<SPEC['minimum_eval_observations']:
                checks[case+'/'+horizon+'/supported']=False
                continue
            checks[case+'/'+horizon+'/mean']=new['mean']<=old['mean']*SPEC['gate_mean_ratio']
            checks[case+'/'+horizon+'/p95']=new['p95']<=old['p95']*SPEC['gate_p95_ratio']
            old_min=result['metrics']['baseline'][horizon].get('minK')
            new_min=result['metrics'][candidate][horizon].get('minK')
            if old_min and new_min:
                checks[case+'/'+horizon+'/minK_mean']=new_min['mean']<=old_min['mean']*SPEC['gate_mean_ratio']
                checks[case+'/'+horizon+'/minK_p95']=new_min['p95']<=old_min['p95']*SPEC['gate_p95_ratio']
    return dict(pass_all=bool(checks) and all(checks.values()),checks=checks)


def load_dev(messages, inputs, sensor, alignment):
    data=json.load(gzip.open(messages,'rt')); gt=prepare_gt(inputs,sensor,alignment); cases={}
    for case in ('H03_162301','R01_163034'):
        frames=[]
        for original in data[case+'/off/policy']:
            frame=dict(original,objects=[])
            for obj in original['objects']:
                point,yaw=lanelet_coordinates(obj['center'],obj['yaw'],alignment)
                a=alignment['rpy'][2];vx,vy=obj['velocity']
                frame['objects'].append(dict(obj,center=point,yaw=yaw,velocity=[math.cos(a)*vx-math.sin(a)*vy,math.sin(a)*vx+math.cos(a)*vy]))
            frames.append(frame)
        cases[case]=frames
    return cases,gt


def main():
    import yaml
    from perception_object_pipeline.map_based_prediction import LaneletNetwork
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--messages',type=Path,required=True)
    parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Fresh directory required')
    args.output.mkdir(parents=True)
    frozen=dict(spec=SPEC,source_sha256=digest(__file__),input_sha256=digest(args.messages),
                gt_sha256=digest(args.inputs/'infos_val.pkl'),production_changed=False,
                reserved_bags=[
                    'rosbags/eval/localization/260915_field_inspection/planning_rviz_0915_sensor_local_1.bag',
                    'rosbags/eval/localization/260915_field_inspection/planning_rviz_0915_sensor_local_2.bag',
                    'rosbags/eval/localization/260915_field_inspection/planning_rviz_0915_sensor_local_3.bag',
                    'rosbags/eval/localization/local_mr_stability/260917_134456_full_route_mr_sigma0p10.bag'])
    (args.output/'preregistered.json').write_text(json.dumps(frozen,indent=2)+'\n')
    alignment=yaml.safe_load((ROOT/'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
    network=LaneletNetwork.load(ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm',ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    prepare_network(network)
    cases,gt=load_dev(args.messages,args.inputs,ROOT/'configs/sensors/legacy/Sensor_4.json',alignment)
    results={}
    for case,frames in cases.items():
        results[case]=evaluate(network,frames,gt,case,VARIANTS)
        print(case,{v:{h:round(m['top1']['mean'],3) for h,m in results[case]['metrics'][v].items()} for v in VARIANTS},flush=True)
    acceptance={v:gates(results,v) for v in VARIANTS if v!='baseline'}
    eligible=[v for v in acceptance if acceptance[v]['pass_all']]
    # Rank eligible candidates by case-balanced 2/3-sec relative mean, not pooled repeated frames.
    selected=min(eligible,key=lambda v:sum(results[c]['metrics'][v][h]['top1']['mean']/results[c]['metrics']['baseline'][h]['top1']['mean']
                    for c in results for h in ('2','3')),default=None)
    report=dict(frozen=frozen,map_sha256=network.map_sha256,results=results,acceptance=acceptance,
                selected_for_reserved=selected,production_enabled=False,
                caveat='Conditional currently GT-matched/future-supported observations, not detection recall. '
                       'Known dev bags; tracked-state history may include coasting and is not independent sensor evidence. '
                       'CPU incremental research cost is not real ROS latency. No future GT enters candidate functions.')
    (args.output/'development.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Development gate:',{v:a['pass_all'] for v,a in acceptance.items()},'Selected:',selected,flush=True)


if __name__=='__main__':main()
