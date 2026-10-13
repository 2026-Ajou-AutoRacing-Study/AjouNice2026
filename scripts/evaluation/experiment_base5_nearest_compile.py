#!/usr/bin/env python3
"""E24 compiler numerical control with deterministic nearest-positive hint.

Authored representative or explicitly frozen complete inputs; never actual
ROS/full-stack or target4090 timing proof. Whole score-sorted payloads retain
the common saved JSON schema. Query cells distinguish numerical drift from
reassignment, including weak/raw proposals instead of discarding their tails.
"""

import argparse
from collections import defaultdict
import gzip
import json
import math
from pathlib import Path
import time

import numpy as np
from experiment_base5_proposal_peaks import CAMERAS, bootstrap_source, digest, uniform_indices
from analyze_base5_proposal_peaks import distribution, paired, prepared, summarize, THRESHOLDS, CLASSES
from compare_base5_common30 import gt_identity, validate_payload
from audit_base5_offline_regression import angle_error_pi
from summarize_base5_rosbag_quick import bridge_nms


def scoring_rows(rows):
    """Keep reserved gt_* fields arrays without rewriting historical exports.

    The first continuous artifact included a scalar provenance tag under a
    reserved prefix. That metadata is not a GT observation. Strip only this
    exact known scalar on a shallow analysis copy, preserving all box values,
    input lineage and the raw saved JSON bytes.
    """
    result = []
    for row in rows:
        copy = dict(row)
        if 'gt_center_convention' in copy:
            if not isinstance(copy['gt_center_convention'],str):
                raise ValueError('Unexpected center convention metadata type')
            copy['box_center_convention'] = copy.pop('gt_center_convention')
        result.append(copy)
    return result


def complete_cpu_report(report, buffers):
    for key,values in buffers.items():
        report.setdefault('latency',{})[key] = distribution([r['inference_ms'] for r in values])
    repeats = min(report['repeat_counts'].values())
    for repeat in range(repeats):
        key = str(repeat)
        left,right = buffers['eager_'+key],buffers['compile_'+key]
        report['paired'][key] = paired(scoring_rows(left),scoring_rows(right))
        report['query_drift'][key] = query_difference(left,right)
        report.setdefault('final_detector_drift',{})[key] = final_difference(left,right)
    report['operating_counts'] = {profile: {case:summarize(scoring_rows([r for r in rows if r['case']==case]),True)
                                 for case in sorted({r['case'] for r in rows})}
                                 for profile,rows in buffers.items()}
    return report


def select_cohort(rows):
    selected = []
    by_case = defaultdict(list)
    for row in rows:
        if row['source'] == 'dev':
            by_case[row['case']].append(row)
    for values in by_case.values():
        selected += [values[i] for i in uniform_indices(len(values), 3)]
    native = [r for r in rows if r['source'] == 'native']
    predicates = (
        lambda r: not r['gt_labels'],
        lambda r: r['token'] == 'b5v2_02330',
        lambda r: any(a == 'Steel_Barricade' and np.linalg.norm(b[:2]) < 20 and n >= 20
                      for a,b,n in zip(r['gt_assets'],r['gt_boxes'],r['gt_points'])),
        lambda r: any(l == 0 and 50 <= np.linalg.norm(b[:2]) < 80 for l,b in zip(r['gt_labels'],r['gt_boxes'])),
        lambda r: any(l == 0 and 80 <= np.linalg.norm(b[:2]) < 100 for l,b in zip(r['gt_labels'],r['gt_boxes'])),
        lambda r: any(a.startswith('Man') and np.linalg.norm(b[:2]) < 10 and n >= 20
                      for a,b,n in zip(r['gt_assets'],r['gt_boxes'],r['gt_points'])),
        lambda r: 'BabyStroller3' in r['gt_assets'],
        lambda r: any(a.startswith('Deer') or a.startswith('Dog') for a in r['gt_assets']),
    )
    taken = set()
    for predicate in predicates:
        candidate = next((r for r in native if r['token'] not in taken and predicate(r)), None)
        if candidate is None:
            raise ValueError('Authored representative cohort stratum not found')
        selected.append(candidate)
        taken.add(candidate['token'])
    if len(selected) != 20 or len({r['token'] for r in selected}) != 20:
        raise ValueError('Twenty unique authored frames required')
    return selected


def query_difference(left, right):
    values, crossings = defaultdict(list), defaultdict(int)
    changed_keys = defaultdict(int)
    for a,b in zip(left,right):
        def keyed(row):
            result = {tuple(v['key']): v for v in row['queries']}
            if len(result) != len(row['queries']):
                raise ValueError('Non-unique initial query identity')
            return result
        ka,kb = keyed(a),keyed(b)
        for key in set(ka)-set(kb):
            changed_keys[CLASSES[key[0]]+'/query_removed'] += 1
        for key in set(kb)-set(ka):
            changed_keys[CLASSES[key[0]]+'/query_added'] += 1
        for key in set(ka)&set(kb):
            x,y = ka[key],kb[key]
            label = key[0]
            name = CLASSES[label]
            pa,pb = np.asarray(x['box']),np.asarray(y['box'])
            if (x['score'] >= THRESHOLDS[label]) != (y['score'] >= THRESHOLDS[label]):
                crossings[name+'/threshold_crossing'] += 1
            # Per-query raw distribution includes weak/unfiltered queries.
            for prefix in ('raw', 'operating_either'):
                if prefix == 'operating_either' and max(x['score'],y['score']) < THRESHOLDS[label]:
                    continue
                vals = dict(center_m=np.linalg.norm(pa[:2]-pb[:2]),
                            z_m=abs(pa[2]-pb[2]),length_m=abs(pa[3]-pb[3]),width_m=abs(pa[4]-pb[4]),
                            height_m=abs(pa[5]-pb[5]), axis_yaw_deg=math.degrees(angle_error_pi(pa[6],pb[6])),
                            score=abs(x['score']-y['score']))
                for metric,value in vals.items():
                    values[f'{prefix}/{name}/{metric}'].append(float(value))
    return dict(query_set_changes=dict(changed_keys), threshold_crossings=dict(crossings),
                distributions={k:distribution(v) for k,v in values.items()})


def final_difference(left, right):
    stats, counts = defaultdict(list), defaultdict(int)
    examples, geometry_examples = [], []
    for a,b in zip(left,right):
        aa,bb = bridge_nms(a),bridge_nms(b)
        def eligible(r):
            return [i for i,(l,s) in enumerate(zip(r['pred_labels'],r['pred_scores'])) if s>=THRESHOLDS[l]]
        remaining = set(eligible(bb))
        for i in eligible(aa):
            label = aa['pred_labels'][i]
            name = CLASSES[label]
            own = sorted(j for j in remaining if bb['pred_labels'][j]==label)
            distances = [np.linalg.norm(np.asarray(aa['pred_boxes'][i][:2])-np.asarray(bb['pred_boxes'][j][:2])) for j in own]
            if own and min(distances)<1.:
                j = own[int(np.argmin(distances))]
                remaining.remove(j)
                counts[name+'/matched'] += 1
                counts[name+'/multiple_candidate_within1m'] += int(sum(v<1. for v in distances)>1)
                x,y = np.asarray(aa['pred_boxes'][i]),np.asarray(bb['pred_boxes'][j])
                deltas = dict(center_m=np.linalg.norm(x[:2]-y[:2]),z_m=abs(x[2]-y[2]),
                    length_m=abs(x[3]-y[3]),width_m=abs(x[4]-y[4]),height_m=abs(x[5]-y[5]),
                    axis_yaw_deg=math.degrees(angle_error_pi(x[6],y[6])),
                    score=abs(aa['pred_scores'][i]-bb['pred_scores'][j]))
                for metric,value in deltas.items():
                    stats[name+'/'+metric].append(float(value))
                same_gt = [(g,np.linalg.norm(np.asarray(box[:2])-x[:2])) for g,(box,l) in
                           enumerate(zip(a.get('gt_boxes',[]),a.get('gt_labels',[]))) if l==label]
                closest = min(same_gt,key=lambda pair:pair[1]) if same_gt else None
                geometry_examples.append(dict(token=a['token'],cls=name,range_m=float(np.linalg.norm(x[:2])),
                    baseline_box=x.tolist(),candidate_box=y.tolist(),baseline_score=aa['pred_scores'][i],candidate_score=bb['pred_scores'][j],
                    closest_authored_same_class_gt=None if closest is None else dict(asset=a['gt_assets'][closest[0]],center_distance_m=float(closest[1])),
                    **{k:float(v) for k,v in deltas.items()}))
            else:
                counts[name+'/baseline_only'] += 1
                if len(examples)<100:
                    examples.append(dict(token=a['token'],cls=name,kind='baseline_only',box=aa['pred_boxes'][i],score=aa['pred_scores'][i]))
        for j in sorted(remaining):
            name = CLASSES[bb['pred_labels'][j]]
            counts[name+'/candidate_only'] += 1
            if len(examples)<100:
                examples.append(dict(token=b['token'],cls=name,kind='candidate_only',box=bb['pred_boxes'][j],score=bb['pred_scores'][j]))
    largest = {metric:{name:sorted([row for row in geometry_examples if row['cls']==name],key=lambda row:-row[metric])[:5]
                      for name in CLASSES} for metric in ('axis_yaw_deg','center_m')}
    return dict(counts=dict(counts),distributions={k:distribution(v) for k,v in stats.items()},examples=examples,
                largest_geometry_changes=largest,
                association='Same-class score-ordered nearest unmatched center<1m; ambiguous close boxes flagged. This is not GT-identity correspondence.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','output','sensor','config','checkpoint'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--full-cohort',action='store_true')
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--eager-repeats',type=int)
    p.add_argument('--compile-repeats',type=int)
    p.add_argument('--eager-camera-numerics',action='store_true',
                   help='Research comparator: leave camera LayerNorm/GELU eager while compiling other camera/neck operations')
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    bootstrap_source()
    import cv2
    import torch
    from app.bevfusion_model import BevFusionModel
    from app.inference_optimization import InferenceOptimization
    from app.sensor4_calibration import load_sensor4_calibration
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True)
    manifest = json.loads(args.inputs.read_text())
    rows = manifest['rows'] if args.full_cohort else select_cohort(manifest['rows'])
    mode_repeats = dict(eager=args.repeats if args.eager_repeats is None else args.eager_repeats,
                       compile=args.repeats if args.compile_repeats is None else args.compile_repeats)
    if min(mode_repeats.values())<1:
        raise ValueError('Positive repeat count required')
    (args.output/'cohort.json').write_text(json.dumps(dict(selection=f'All {len(rows)} frozen inputs' if args.full_cohort else 'Authored GT strata, not output-based selection',rows=rows),indent=2)+'\n')
    cal = load_sensor4_calibration(args.sensor,['camera_'+c for c in CAMERAS])
    model = BevFusionModel(str(args.config),str(args.checkpoint),cal,use_amp=True,
        preprocess_workers=4,warmup_iterations=2,base5_runtime=True,parallel_camera=True,
        point_intensity_source='ros255')
    model.model.view_transform.depth_raster_policy = 'nearest_positive'
    model.model.view_transform.depth_raster_backend = 'sorted'
    head = model.model.bbox_head
    tensors = {}
    def positions(_module,_args,kwargs):
        tensors['positions'] = kwargs['query_pos'].detach().clone()
    def output(module,_args,value):
        result = value[0][0]
        n = module.num_proposals
        tensors['labels'] = module.query_labels.detach().clone()
        for key in ('heatmap','query_heatmap_score','center','height','dim','rot'):
            tensors[key] = result[key][...,-n:].detach().clone()
    hooks = [head.decoder[0].register_forward_pre_hook(positions,with_kwargs=True),head.register_forward_hook(output)]

    def run(row,profile,sequence):
        if digest(row['point_path']) != row['point_sha256']:
            raise ValueError('Frozen points changed')
        images = {key:Path(path).read_bytes() for key,path in row['images'].items()}
        if any(digest(row['images'][key]) != value for key,value in row['image_sha256'].items()):
            raise ValueError('Frozen JPEG changed')
        points = np.fromfile(row['point_path'],'<f4').reshape(-1,4)
        model.preprocessor.point_intensity_source = 'native' if row['source']=='native' else 'ros255'
        det,prep,latency = model.infer(points,images,sequence,0.,200,timestamp_sec=row['stamp'])
        labels = tensors['labels']
        one_hot = torch.nn.functional.one_hot(labels,head.num_classes).permute(0,2,1)
        scores = tensors['heatmap'].sigmoid()*tensors['query_heatmap_score']*one_hot
        raw = head.bbox_coder.decode(scores,tensors['rot'].clone(),tensors['dim'].clone(),
            tensors['center'].clone(),tensors['height'].clone(),None,filter=False)[0]
        query_positions = tensors['positions'].float().cpu().numpy()[0]
        indices = np.rint(query_positions-.5).astype(int)
        if not np.allclose(query_positions,indices+.5,rtol=0,atol=1e-6):
            raise ValueError('Initial query positions are not canonical cell centers')
        query_labels = labels.cpu().tolist()[0]
        qs,qb = raw['scores'].float().cpu().tolist(),raw['bboxes'].float().cpu().tolist()
        qr = tensors['rot'].float().cpu().numpy()[0].T.tolist()
        return dict(row, profile=profile,pred_boxes=[d['center']+d['size']+[d['yaw']] for d in det],
            pred_labels=[d['label'] for d in det],pred_scores=[d['score'] for d in det],query_scores=[],
            queries=[dict(key=[l,*index],score=s,box=b,rot_vector=r) for l,index,s,b,r in zip(query_labels,indices.tolist(),qs,qb,qr)],
            preprocess_ms=prep,inference_ms=latency)

    try:
        buffers = {}
        report = dict(scope=__doc__,cohort=len(rows),classes=CLASSES,classes_thresholds=THRESHOLDS,
                      nearest_depth_hint=True,compile_parts=['camera','neck'],compile_warmup_excluded=True,
                      gpu=torch.cuda.get_device_name(),warmup={},parity={},paired={},query_drift={},
                      repeat_counts=mode_repeats,source_lineage=manifest.get('source_lineage'),
                      source_input_sha256=manifest.get('input_sha256'),sensor_sha256=digest(args.sensor),
                      eager_camera_numerics=args.eager_camera_numerics,
                      cudnn_benchmark=torch.backends.cudnn.benchmark,
                      cudnn_deterministic=torch.backends.cudnn.deterministic,
                      source_sha256={str(path):digest(path) for path in [Path(__file__),
                          Path('perception/detection3d/bevfusion/depth_raster.py'),
                          Path('perception/detection3d/bevfusion/plugin/morai_depth_raster.py'),
                          Path('docker/bevfusion/app/bevfusion_model.py'),
                          Path('docker/bevfusion/app/inference_optimization.py'),
                          Path('third_party/mmdetection3d/projects/BEVFusion/bevfusion/depth_lss.py')]})
        for mode in ('eager','compile'):
            if mode == 'compile':
                model.inference_optimization.disable()
                model.inference_optimization = InferenceOptimization(model.model,('camera','neck'),parallel_camera=True,
                    eager_camera_numerics=args.eager_camera_numerics)
            start = time.perf_counter()
            warm = run(rows[0],mode,0)
            report['warmup'][mode] = dict(wall_seconds=time.perf_counter()-start,inference_ms=warm['inference_ms'])
            print(json.dumps(dict(mode=mode,event='warmup_done',**report['warmup'][mode])),flush=True)
            for repeat in range(mode_repeats[mode]):
                key = f'{mode}_{repeat}'
                values = []
                for i,row in enumerate(rows):
                    values.append(run(row,key,i))
                    if (i+1)%100==0:
                        print(json.dumps(dict(profile=key,frames=i+1,total=len(rows))),flush=True)
                buffers[key] = values
                payload = dict(name='E24_nearest_'+key,checkpoint_sha256=digest(args.checkpoint),
                    config_sha256=digest(args.config),input_sha256=digest(args.inputs),classes=CLASSES,
                    frames=len(values),gt_identity=gt_identity(values),rows=values,
                    sensor_sha256=digest(args.sensor),source_lineage=manifest.get('source_lineage'),
                    source_input_sha256=manifest.get('input_sha256'),source_sha256=report['source_sha256'],
                    inference_precision='AMP/TF32 enabled; one identical learned model; nearest-positive sorted hint',
                    inference_optimization=model.inference_optimization.report,
                    nearest_depth_hint=dict(policy='nearest_positive',backend='sorted'),
                    replay_scope=manifest.get('replay_scope','Four dev snippets are sparse, native samples static; JSON compatible but no continuous tracker certification.'))
                validate_payload(payload)
                with gzip.open(args.output/(key+'.json.gz'),'wt') as stream:
                    json.dump(payload,stream,allow_nan=False)
                print(json.dumps(dict(profile=key,event='cohort_done',frames=len(values))),flush=True)
            reference = buffers[mode+'_0']
            report['parity'][mode] = {str(i):dict(raw_bitwise_frames=sum(all(a[k]==b[k] for k in ('pred_boxes','pred_labels','pred_scores'))
                        for a,b in zip(reference,buffers[mode+'_'+str(i)])),frames=len(rows)) for i in range(1,mode_repeats[mode])}
            (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        complete_cpu_report(report,buffers)
        (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False,
                                  default=lambda x:int(x) if isinstance(x,np.integer) else bool(x))+'\n')
    finally:
        for hook in hooks:
            hook.remove()
        model.preprocessor.close()


if __name__=='__main__':
    main()
