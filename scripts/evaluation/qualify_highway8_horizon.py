#!/usr/bin/env python3
"""Offline 3/3 vs append/native 3/8 coverage, preservation and CPU audit.

No ROS master, node initialization, publisher, detector or simulator is used.
The E12 frozen core pack reconstructs ROS UUID/covariance fields; this is not a
recorded-message replay, a full-stack latency test, or a planner safety result.
"""
import argparse
from collections import Counter
from copy import deepcopy
import gzip
import hashlib
import importlib.util
from io import BytesIO
import json
import math
import os
from pathlib import Path
import platform
import resource
import sys
import time

import yaml

from analyze_e12_map_horizon import (
    ROOT, CORE_PATH, NODE_PATH, E12_SHA, CANONICAL, current_assignment,
    digest, future_reference, load, path_errors, prepare_case,
)
from audit_vehicle_map_prediction import distribution
from qualify_e12_mixed_horizon import PACK_SHA, build_message, clone_network, synthetic_uuid

SOURCE = ROOT/'base_ws/src/perception_object_pipeline/src'
SELECTOR_PATH = SOURCE/'perception_object_pipeline/mixed_horizon.py'
DEFAULT_PACK = ROOT/'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1/tracker_v1/replay_pack.json.gz'
OUTPUT_ROOT = ROOT/'artifacts/evaluation/highway8_261024_v1'


def wire(message):
    stream = BytesIO()
    message.serialize(stream)
    return stream.getvalue()


def compare_prefix(baseline, extended, baseline_count=31, extended_count=81):
    """Only suffix removal is permitted; every other serialized byte must match."""
    if len(baseline.objects) != len(extended.objects):
        raise ValueError('Object count changed')
    trimmed = deepcopy(extended)
    count = 0
    for old, new in zip(baseline.objects, trimmed.objects):
        a, b = old.kinematics.predicted_paths, new.kinematics.predicted_paths
        if len(a) != len(b) or not a:
            raise ValueError('Top-K count changed or empty object paths')
        if any(len(path.path) != baseline_count for path in a):
            raise ValueError('Incomplete baseline path')
        lengths = {len(path.path) for path in b}
        if len(lengths) != 1 or not lengths.issubset({baseline_count, extended_count}):
            raise ValueError('Partial extension or padded/truncated output')
        count += lengths == {extended_count}
        for path in b:
            path.path = path.path[:baseline_count]
    first, second = wire(baseline), wire(trimmed)
    if first != second:
        raise ValueError('First3/current state/UUID/order/Top-K/confidence changed')
    return int(count), first, second


def _pose_values(pose):
    return (pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,
            pose.orientation.y,pose.orientation.z,pose.orientation.w)


def compare_native(baseline, output, selected_ids, moving_ids=(), tangent_speeds=None):
    """Preserve non-path fields/all short objects, but measure native route changes."""
    if len(baseline.objects)!=len(output.objects):
        raise ValueError('Native object count changed')
    old_common,new_common=deepcopy(baseline),deepcopy(output)
    trimmed=deepcopy(output)
    stats=Counter();differences=[]
    selected=set(selected_ids);moving=set(moving_ids)
    tangent_speeds={} if tangent_speeds is None else tangent_speeds
    if not set(tangent_speeds).issubset(selected):
        raise ValueError('Tangent object was not selected for highway horizon')
    for index,(old,new) in enumerate(zip(baseline.objects,output.objects)):
        key=bytes(new.object_id.uuid)
        if key!=bytes(old.object_id.uuid):
            raise ValueError('Native UUID/order changed')
        a,b=old.kinematics.predicted_paths,new.kinematics.predicted_paths
        if not a or not b or any(len(path.path)!=31 for path in a):
            raise ValueError('Empty native path or incomplete baseline')
        expected=81 if key in selected else 31
        if any(len(path.path)!=expected for path in b):
            raise ValueError('Selected highway object did not retain full8s coverage')
        if key not in selected and wire(old)!=wire(new):
            raise ValueError('Non-highway object output changed')
        old_common.objects[index].kinematics.predicted_paths=[]
        new_common.objects[index].kinematics.predicted_paths=[]
        for path in trimmed.objects[index].kinematics.predicted_paths:
            path.path=path.path[:31]
        if key not in selected:
            stats['nonhighway_objects_exact']+=1
            continue
        stats['highway_objects']+=1
        if key in tangent_speeds:
            if wire(old)!=wire(trimmed.objects[index]):
                raise ValueError('Tangent fallback changed Map3 prefix/Top-K/confidence')
            stats['tangent_objects_prefix_exact']+=1
            speed=tangent_speeds[key]
            for path in b:
                terminal=_pose_values(path.path[30])
                qx,qy,qz,qw=terminal[3:]
                yaw=math.atan2(2.*(qw*qz+qx*qy),1.-2.*(qy*qy+qz*qz))
                for offset,pose in enumerate(path.path[31:],1):
                    values=_pose_values(pose)
                    expected=(terminal[0]+speed*.1*offset*math.cos(yaw),
                              terminal[1]+speed*.1*offset*math.sin(yaw),terminal[2])
                    if (any(abs(x-y)>1.e-6 for x,y in zip(values[:3],expected))
                            or values[3:]!=terminal[3:]):
                        raise ValueError('Tangent suffix is discontinuous or changes terminal yaw/z')
                    stats['tangent_tail_intervals_valid']+=1
        stats['path_count_changed']+=len(a)!=len(b)
        stats['ranked_confidences_changed']+=[p.confidence for p in a]!=[p.confidence for p in b]
        for path in b:
            step=float(path.time_step.sec)+float(path.time_step.nanosec)*1.e-9
            if not math.isclose(step,.1,abs_tol=1.e-9):
                raise ValueError('Native time step changed')
            if not math.isfinite(path.confidence) or not 0.<=path.confidence<=1.:
                raise ValueError('Invalid native path confidence')
            points=[_pose_values(pose) for pose in path.path]
            if any(not all(math.isfinite(v) for v in pose) for pose in points):
                raise ValueError('Nonfinite native path')
            if any(abs(sum(v*v for v in pose[3:])-1.)>1.e-6 for pose in points):
                raise ValueError('Invalid native quaternion')
            if points[0]!=_pose_values(a[0].path[0]):
                raise ValueError('Native current path pose changed')
            if key in moving and any(math.dist(first[:3],second[:3])<=1.e-7
                                     for first,second in zip(points[-11:-1],points[-10:])):
                raise ValueError('Native moving path contains repeated/clamped final samples')
        old_points=[_pose_values(p) for p in a[0].path]
        new_points=[_pose_values(p) for p in b[0].path[:31]]
        distances=[math.dist(x[:3],y[:3]) for x,y in zip(old_points,new_points)]
        differences.append(dict(top1_prefix_mean_xyz_m=sum(distances)/len(distances),
                                top1_prefix_max_xyz_m=max(distances)))
        stats['top1_prefix_changed']+=old_points!=new_points
        matched_rank=next((i for i,path in enumerate(a)
                           if [_pose_values(p) for p in path.path]==new_points),None)
        stats['native_top1_unmatched_baseline_prefix']+=matched_rank is None
        stats['native_top1_matches_later_baseline_rank']+=matched_rank is not None and matched_rank!=0
    if wire(old_common)!=wire(new_common):
        raise ValueError('Native changed header/current state/shape/classification fields')
    return len(selected),wire(baseline),wire(trimmed),dict(stats),differences


def summarize_gt(rows):
    return dict(origins=len(rows),**{key:distribution([row[key] for row in rows]) for key in
                ('top1_ADE_m','top1_FDE_m','minK_ADE_m','minK_FDE_m')})


def matched_gt_comparison(native_rows,append_rows):
    """Compare only identical current assignment origins; report unique cohorts too."""
    def key(row):
        return json.dumps([row['case'],row['stamp'],row['id'],row['gt_key']],sort_keys=True)
    native={key(row):row for row in native_rows}
    append={key(row):row for row in append_rows}
    if len(native)!=len(native_rows) or len(append)!=len(append_rows):
        raise ValueError('Duplicate GT origin key')
    common=set(native)&set(append)
    return dict(common_origins=len(common),native_only_origins=len(set(native)-set(append)),
                append_only_origins=len(set(append)-set(native)),
                native_common=summarize_gt([native[k] for k in sorted(common)]),
                append_common=summarize_gt([append[k] for k in sorted(common)]),
                native_only=summarize_gt([native[k] for k in sorted(set(native)-set(append))]))


def diagnose_route_failure(network, args, kwargs):
    """Read-only explanatory copy of conservative continuation checks, not policy."""
    x, y, _z, _yaw, speed, base, extended, trajectories = args
    allowed = kwargs.get('allowed_lanelet_ids')
    allowed = None if allowed is None else frozenset(allowed)
    maximum = max(1, int(network.config.maximum_route_depth))
    for trajectory in trajectories:
        ids = tuple(trajectory.lanelet_ids)
        if not ids or any(i not in network.records for i in ids):
            return dict(reason='invalid_baseline_route', route=list(ids))
        if allowed is not None and any(i not in allowed for i in ids):
            return dict(reason='baseline_leaves_highway', route=list(ids))
        geometry, projection = network._route_geometry(ids)
        start = projection.project(x, y)
        if start is None:
            return dict(reason='baseline_projection_missing', route=list(ids))
        if geometry.length < start.arc_length + speed*base[-1]:
            return dict(reason='baseline_already_clamped', route=list(ids))
        required = start.arc_length + speed*extended[-1]
        occurrences = Counter(ids)
        while geometry.length < required:
            info = dict(route=list(ids), last_lanelet=ids[-1], speed_mps=speed,
                        route_remaining_m=geometry.length-start.arc_length,
                        requested_distance_m=speed*extended[-1])
            if len(ids) >= maximum:
                return dict(info, reason='maximum_route_depth')
            following = tuple(dict.fromkeys(network.records[ids[-1]].following_ids))
            if not following:
                return dict(info, reason='route_ended')
            if len(following) != 1:
                return dict(info, reason='ambiguous_successor', followers=list(following))
            next_id = following[0]
            if next_id not in network.records:
                return dict(info, reason='missing_successor', successor=next_id)
            if occurrences[next_id] >= 2:
                return dict(info, reason='loop_limit', successor=next_id)
            if allowed is not None and next_id not in allowed:
                return dict(info, reason='successor_leaves_highway', successor=next_id)
            ids += (next_id,)
            occurrences[next_id] += 1
            geometry, _ = network._route_geometry(ids)
    return dict(reason='other_contract_rejection')


def qualify_case(name, case, transform, loaded, node, limit=0, warmup=30,mode='append'):
    from perception_object_pipeline.mixed_horizon import MixedHorizonSelector
    from perception_object_pipeline.kinematic_prediction import build_prediction_horizons

    baseline_net, mixed_net = clone_network(loaded), clone_network(loaded)
    base, longer = build_prediction_horizons(.1, 3.), build_prediction_horizons(.1, 8.)
    selector = MixedHorizonSelector(mixed_net,**(
        {'current_lanelet_only':True,'entry_confirm_sec':0.0} if mode=='native' else {}))
    extension = node.MixedHorizonExtension(mixed_net, base, longer, selector,
                                         **({'mode':'native'} if mode=='native' else {}))
    decisions={};native_kinds={}
    decide=selector.decide
    def traced_decide(track_id,*args,**kwargs):
        decision=decide(track_id,*args,**kwargs)
        decisions[bytes(track_id)]=decision
        return decision
    selector.decide=traced_decide
    extend=extension.extend
    def traced_object_extend(tracked,*args,**kwargs):
        names=('native_map_objects','native_short_objects','native_kinematic_objects')
        before=[getattr(extension,name,0) for name in names]
        result=extend(tracked,*args,**kwargs)
        changed=[kind for name,old,kind in zip(names,before,('map','short','kinematic'))
                 if getattr(extension,name,0)>old]
        if len(changed)>1:
            raise ValueError('Native object assigned multiple output kinds')
        if changed:
            native_kinds[bytes(tracked.object_id.uuid)]=changed[0]
        return result
    extension.extend=traced_object_extend
    attempts = []
    original_extend = mixed_net.extend_trajectories

    def traced_extend(*args, **kwargs):
        result = original_extend(*args, **kwargs)
        attempts.append((args, kwargs, result))
        return result

    mixed_net.extend_trajectories = traced_extend
    frames, trajectories = prepare_case(case, transform)
    pairs = list(zip(case['frames'], frames))
    if limit:
        pairs = pairs[:limit]
    counts, reasons, pose_lengths, failures, gt_exclusions = [Counter() for _ in range(5)]
    rows, gt_rows, failure_rows = [], [], []
    native_counts=Counter();native_prefix_differences=[]
    timing = dict(baseline=[], mixed=[])
    wire_sizes = dict(baseline=[], mixed=[], additional=[])
    baseline_hash, prefix_hash, input_hash = [hashlib.sha256() for _ in range(3)]
    for index, (raw, frame) in enumerate(pairs):
        message = build_message(name, index, raw, frame, transform)
        before = wire(message)
        input_hash.update(before)
        values, elapsed = {}, {}
        attempts.clear()
        decisions.clear()
        native_kinds.clear()
        for variant in (('baseline', 'mixed') if index % 2 == 0 else ('mixed', 'baseline')):
            started = time.perf_counter()
            values[variant] = node.make_map_based_predicted_objects(
                message, base, baseline_net if variant == 'baseline' else mixed_net,
                mixed_extension=None if variant == 'baseline' else extension)
            elapsed[variant] = (time.perf_counter()-started)*1000.
        if wire(message) != before:
            raise ValueError('Input mutated')
        baseline, mapped, fallback, paths = values['baseline']
        output, mapped2, fallback2, paths2 = values['mixed']
        if mode=='append' and (mapped, fallback, paths) != (mapped2, fallback2, paths2):
            raise ValueError('Mapped/fallback/Top-K counts changed')
        changes,diffs={},[]
        if mode=='append':
            extended_objects, first, trimmed = compare_prefix(baseline, output)
        else:
            selected_ids={key for key,kind in native_kinds.items() if kind=='map'}
            moving_ids={bytes(obj.object_id.uuid) for obj in message.objects
                        if abs(obj.kinematics.twist_with_covariance.twist.linear.x)>=.5}
            tangent_speeds={bytes(obj.object_id.uuid):obj.kinematics.twist_with_covariance.twist.linear.x
                            for obj in message.objects
                            if native_kinds.get(bytes(obj.object_id.uuid))=='tangent'}
            extended_objects,first,trimmed,changes,diffs=compare_native(
                baseline,output,selected_ids,moving_ids,tangent_speeds)
            if set(native_kinds)!={key for key,value in decisions.items() if value.extend}:
                raise ValueError('Native selected/output-kind membership differs')
            native_counts.update(changes);native_prefix_differences.extend(diffs)
        if extended_objects != extension.extended_objects:
            raise ValueError('Diagnostic/output extension count differs')
        baseline_hash.update(first)
        prefix_hash.update(trimmed)
        encoded = wire(output)
        wire_sizes['baseline'].append(len(first))
        wire_sizes['mixed'].append(len(encoded))
        wire_sizes['additional'].append(len(encoded)-len(first))
        reasons.update(extension.reasons)
        counts.update(dict(frames=1, objects=len(message.objects), mapped_objects=mapped,
                           kinematic_fallback_objects=fallback, paths=paths,
                           extended_objects=extended_objects, prefix_exact_frames=int(first==trimmed),
                           extension_errors=extension.extension_errors,
                           route_extension_attempts=len(attempts)))
        if mode=='native':
            counts.update(dict(native_map_objects=extension.native_map_objects,
                               native_short_objects=extension.native_short_objects,
                               native_tangent_objects=getattr(extension,'native_tangent_objects',0),
                               native_kinematic_objects=extension.native_kinematic_objects,
                               native_topk_paths=paths2,baseline_topk_paths=paths))
        for obj in output.objects:
            for path in obj.kinematics.predicted_paths:
                pose_lengths[len(path.path)] += 1
        for args, kwargs, result in attempts:
            if len(result) == len(args[-1]) and all(len(p.poses) == len(longer) for p in result):
                counts['successful_route_extension_attempts'] += 1
                continue
            diagnostic = diagnose_route_failure(mixed_net, args, kwargs)
            failures[diagnostic['reason']] += 1
            failure_rows.append(dict(case=name, stamp=frame['stamp'], **diagnostic))
        if index >= warmup and message.objects:
            for key in timing:
                timing[key].append(elapsed[key])
        rows.append(dict(case=name, stamp=frame['stamp'], input_objects=len(message.objects),
                         extended_objects=extended_objects, reasons=dict(extension.reasons),
                         timing_ms=elapsed, output_bytes=len(encoded), baseline_bytes=len(first),
                         native_output_changes=changes,native_prefix_differences=diffs))
        if not raw['scoring_eligible']:
            continue
        current_pairs = current_assignment(frame['objects'], frame['gt'])
        output_by_id = {bytes(obj.object_id.uuid): obj for obj in output.objects}
        for oi, gi, current_error in current_pairs:
            obj, truth = frame['objects'][oi], frame['gt'][gi]
            predicted = output_by_id[bytes(synthetic_uuid(name, obj['id']))]
            outputs = predicted.kinematics.predicted_paths
            if any(len(path.path) != 81 for path in outputs):
                gt_exclusions['not_extended'] += 1
                continue
            series = trajectories.get(truth['key'])
            if series is None:
                gt_exclusions['no_gt_trajectory'] += 1
                continue
            reference, reason = future_reference(series, frame['stamp'], 8)
            if reason:
                gt_exclusions[reason] += 1
                continue
            paths_xy = [dict(xy=[[p.position.x,p.position.y] for p in path.path[1:]]) for path in outputs]
            gt_rows.append(dict(case=name,stamp=frame['stamp'],id=obj['id'],gt_key=truth['key'],
                                native_kind=native_kinds.get(bytes(synthetic_uuid(name,obj['id'])),
                                                            'append' if mode=='append' else 'unselected'),
                                current_xy_error_m=current_error,**path_errors(paths_xy,reference,8)))
    summary = dict(mode=mode,counts=dict(counts), selector_reasons=dict(reasons),
                   extension_failure_reasons=dict(failures), output_path_pose_counts=dict(pose_lengths),
                   prefix_exact=baseline_hash.hexdigest()==prefix_hash.hexdigest(),
                   baseline_wire_sha256=baseline_hash.hexdigest(),trimmed_wire_sha256=prefix_hash.hexdigest(),
                   input_wire_sha256=input_hash.hexdigest(),
                   nonhighway_and_current_fields_exact=True,
                   native_output_changes=dict(native_counts),
                   native_top1_prefix_differences={key:distribution([r[key] for r in native_prefix_differences])
                       for key in ('top1_prefix_mean_xyz_m','top1_prefix_max_xyz_m')},
                   timing_ms={key:distribution(values) for key,values in timing.items()},
                   serialized_bytes={key:distribution(values) for key,values in wire_sizes.items()},
                   eight_second_gt=dict(origins=len(gt_rows),exclusions=dict(gt_exclusions),
                       by_native_kind={kind:summarize_gt([r for r in gt_rows if r['native_kind']==kind])
                           for kind in sorted({r['native_kind'] for r in gt_rows})},
                       **{key:distribution([r[key] for r in gt_rows]) for key in
                          ('top1_ADE_m','top1_FDE_m','minK_ADE_m','minK_FDE_m')}))
    return summary, rows, gt_rows, failure_rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pack',type=Path,default=DEFAULT_PACK)
    parser.add_argument('--parameters',type=Path,default=ROOT/'base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml')
    parser.add_argument('--map-file',type=Path,default=ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm')
    parser.add_argument('--projector-file',type=Path,default=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    parser.add_argument('--transform',type=Path,default=ROOT/'configs/tf/morai_lanelet_static_tf.yaml')
    parser.add_argument('--case',action='append',default=[])
    parser.add_argument('--max-frames-per-case',type=int,default=0)
    parser.add_argument('--mode',choices=('append','native'),default='append')
    parser.add_argument('--compare-append-report',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    args.output=args.output or OUTPUT_ROOT/('frozen_e12_all14' if args.mode=='append' else 'native8_all14')
    args.output.resolve().relative_to(OUTPUT_ROOT.resolve())
    if args.output.exists():
        raise FileExistsError('Fresh evidence output required')
    if args.max_frames_per_case<0:
        raise ValueError('Frame limit must be nonnegative')
    pack=load(args.pack)
    if (digest(args.pack)!=PACK_SHA or pack.get('schema')!='e12_postprocessing_core_pack_v1'
            or not pack.get('complete') or any(c['provenance']['recorded_model']['checkpoint_sha256']!=E12_SHA
                                               for c in pack['cases'].values())):
        raise ValueError('Expected exact frozen completed E12 pack')
    names=args.case or list(pack['cases'])
    if len(set(names))!=len(names) or any(name not in pack['cases'] for name in names):
        raise ValueError('Unknown or duplicate case selection')
    sys.path.insert(0,str(SOURCE))
    import perception_object_pipeline.map_based_prediction as core
    if Path(core.__file__).resolve()!=CORE_PATH.resolve():
        raise ValueError('Source core import required')
    spec=importlib.util.spec_from_file_location('highway8_source_node',NODE_PATH)
    node=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    source_paths=[CORE_PATH,NODE_PATH,SELECTOR_PATH,Path(__file__),
                  Path(__file__).with_name('qualify_e12_mixed_horizon.py'),
                  Path(__file__).with_name('analyze_e12_map_horizon.py'),
                  Path(__file__).with_name('audit_vehicle_map_prediction.py'),
                  SOURCE/'perception_object_pipeline/kinematic_prediction.py']
    input_paths=[args.pack,args.parameters,args.map_file,args.projector_file,args.transform]
    hashes={str(path):digest(path) for path in source_paths+input_paths}
    params=yaml.safe_load(args.parameters.read_text())
    config=core.MapPredictionConfig(**{k:v for k,v in params.items() if k in core.MapPredictionConfig.__dataclass_fields__})
    loaded=core.LaneletNetwork.load(args.map_file,args.projector_file,config)
    transform=yaml.safe_load(args.transform.read_text())['map_to_morai_map']
    report=dict(schema='highway8_frozen_e12_qualification_v2',mode=args.mode,source_and_input_sha256=hashes,
                cases={},production_changed_by_this_tool=False,baseline_horizon_s=3,highway_horizon_s=8,
                native_policy=(dict(entry_confirm_sec=0.,current_lanelet_only=True,
                    selected_stationary_paths_may_repeat=True,
                    moving_tail_validation='abs(body_vx)>=0.5m/s requires nonrepeated final10 intervals')
                    if args.mode=='native' else None),
                protocol=dict(input='Frozen E12 confirmed numeric core state reconstructed to ROS; synthetic UUID/default covariance, production yaw/UNKNOWN policy',
                    comparison=('Same TrackedObjects; actual public conversion; input/current fields immutable. '
                                'append preserves serialized prefix31; native allows highway paths/ranks/confidence to change '
                                'while every non-selected object remains wire-identical. Selected objects contain81 finite poses; '
                                'moving vehicles must not have repeated/clamped tails, stationary paths may repeat.'),
                    timing='CPU Python conversion/map math with alternating order; first30/empty frames excluded. Thin passive decision/extension tracing included; not live ROS latency.',
                    memory='Exact serialized payload cost; process ru_maxrss is whole-process high-water, not per-variant memory delta.',
                    gt='Current XY2m one-to-one association fixed before any future lookup; contiguous eligible native GT only; oracle min-K is evaluation only.'),
                host=dict(platform=platform.platform(),cpu_count=os.cpu_count(),load_average=os.getloadavg()),
                limitations=['No simulator/planner/closed-loop test; no claim that 8s motion is accurate or collision-safe.',
                    ('Append route ambiguity preserves3s; native re-predicts branches. If native Map8 is unavailable, '
                     'valid Map3 candidates stay byte-identical; absent Map3 uses ordinary CTRV3. '
                     'Neither terminal-tangent extrapolation nor CTRV8 is used. '
                     'Native selection uses current-road eligibility immediately (entry confirmation0s); append retains0.5s confirmation. '
                     'Not every visible vehicle is a selected highway object.'),
                    'Historical start-acceleration origins remain included in diagnostic GT where supported.'])
    all_rows,all_gt,all_failures=[],[],[]
    for name in names:
        summary,rows,gt,failures=qualify_case(name,pack['cases'][name],transform,loaded,node,args.max_frames_per_case,mode=args.mode)
        report['cases'][name]=summary
        all_rows.extend(rows);all_gt.extend(gt);all_failures.extend(failures)
        print(json.dumps(dict(case=name,**summary['counts'],failure_reasons=summary['extension_failure_reasons']),sort_keys=True),flush=True)
    for path,expected in hashes.items():
        if digest(path)!=expected:
            raise ValueError('Source/input changed during qualification: '+path)
    report['aggregate']=dict(counts=dict(sum((Counter(c['counts']) for c in report['cases'].values()),Counter())),
        selector_reasons=dict(sum((Counter(c['selector_reasons']) for c in report['cases'].values()),Counter())),
        extension_failure_reasons=dict(sum((Counter(c['extension_failure_reasons']) for c in report['cases'].values()),Counter())),
        output_path_pose_counts=dict(sum((Counter(c['output_path_pose_counts']) for c in report['cases'].values()),Counter())),
        prefix_exact=all(c['prefix_exact'] for c in report['cases'].values()),
        nonhighway_and_current_fields_exact=all(c['nonhighway_and_current_fields_exact'] for c in report['cases'].values()),
        native_output_changes=dict(sum((Counter(c['native_output_changes']) for c in report['cases'].values()),Counter())),
        native_top1_prefix_differences={key:distribution([item[key] for row in all_rows
            for item in row['native_prefix_differences']]) for key in
                ('top1_prefix_mean_xyz_m','top1_prefix_max_xyz_m')},
        timing_ms={key:distribution([r['timing_ms'][key] for name in names
            for r in [row for row in all_rows if row['case']==name][30:] if r['input_objects']]) for key in ('baseline','mixed')},
        serialized_bytes={key:distribution([r['baseline_bytes'] if key=='baseline' else r['output_bytes']
            if key=='mixed' else r['output_bytes']-r['baseline_bytes'] for r in all_rows]) for key in ('baseline','mixed','additional')},
        eight_second_gt=dict(origins=len(all_gt),
            by_native_kind={kind:summarize_gt([r for r in all_gt if r['native_kind']==kind])
                for kind in sorted({r['native_kind'] for r in all_gt})},
            **{key:distribution([r[key] for r in all_gt]) for key in
            ('top1_ADE_m','top1_FDE_m','minK_ADE_m','minK_FDE_m')}),
        process_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    if args.compare_append_report:
        reference=json.loads(args.compare_append_report.read_text())
        old_hashes=reference['source_and_input_sha256']
        old_maps=[value for path,value in old_hashes.items() if path.endswith('.osm')]
        old_pack_hash=next((value for path,value in old_hashes.items()
                           if Path(path).resolve()==args.pack.resolve()),None)
        if (reference.get('mode','append')!='append' or old_maps!=[digest(args.map_file)]
                or old_pack_hash!=digest(args.pack)):
            raise ValueError('Append reference must use identical map and frozen input pack')
        append_rows=load(args.compare_append_report.parent/'eight_second_gt.json.gz')
        append_rows=[row for row in append_rows if row['case'] in names]
        report['matched_append_comparison']=dict(
            reference_report=str(args.compare_append_report),reference_report_sha256=digest(args.compare_append_report),
            **matched_gt_comparison(all_gt,append_rows))
    args.output.mkdir(parents=True)
    for name,value in [('frames.json.gz',all_rows),('eight_second_gt.json.gz',all_gt),('route_failures.json.gz',all_failures)]:
        with gzip.open(args.output/name,'wt',encoding='utf-8') as handle:
            json.dump(value,handle,ensure_ascii=False,allow_nan=False)
    (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (args.output/'COMPLETE.json').write_text(json.dumps(dict(complete=True,
        files_sha256={name:digest(args.output/name) for name in
            ('report.json','frames.json.gz','eight_second_gt.json.gz','route_failures.json.gz')},
        report_sha256=digest(args.output/'report.json')),indent=2)+'\n')
    print(json.dumps(report['aggregate'],indent=2,ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
