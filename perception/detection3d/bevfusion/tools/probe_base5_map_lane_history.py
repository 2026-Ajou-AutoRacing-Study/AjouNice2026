#!/usr/bin/env python3
"""Current-map replay of frozen ROS track states; lane history is diagnostic only."""
import argparse
from bisect import bisect_left
from collections import Counter
import copy
from dataclasses import replace
import gzip
import hashlib
import json
import math
from pathlib import Path
import pickle

from audit_base5_predicted_path_selection import selected_indices, stats


def lanelet_coordinates(center, yaw, transform):
    if transform is None:
        return tuple(center), yaw
    if transform['parent'] != 'map' or transform['child'] != 'morai_map':
        raise ValueError('Expected map parent / morai_map child')
    roll, pitch, angle = transform['rpy']
    if roll != 0 or pitch != 0:
        raise ValueError('Only audited planar rigid TF supported')
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = center; tx, ty, tz = transform['xyz']
    return (c*x - s*y + tx, s*x + c*y + ty, z + tz), yaw + angle


def history_choice(candidates, previous, stamp, records, ratio=1.25, max_age=.5):
    """Retain a near-tied compatible start lane, not a future fork/route choice.

    Never creates a candidate, crosses the map yaw/distance gates, or removes
    a route. This is NOT installed in the predictor.
    """
    if not candidates:
        return None
    best = candidates[0]
    if previous is None or not 0 < stamp - previous[0] <= max_age:
        return best.lanelet_id
    record = records.get(previous[1])
    allowed = {previous[1]} | (set(record.following_ids) if record else set())
    compatible = [c for c in candidates if c.lanelet_id in allowed]
    if compatible and best.score <= compatible[0].score * ratio:
        return compatible[0].lanelet_id
    return best.lanelet_id


def future_point(series, stamp, max_gap=.15):
    times = [r[0] for r in series]
    i = bisect_left(times, stamp)
    if i < len(series) and abs(series[i][0] - stamp) < 1.e-6:
        return series[i][1]
    if i == 0 or i == len(series) or times[i] - times[i-1] > max_gap:
        return None
    alpha = (stamp - times[i-1]) / (times[i] - times[i-1])
    return tuple((1-alpha)*a + alpha*b for a,b in zip(series[i-1][1],series[i][1]))


def history_weighted_network(network, candidates, chosen):
    """Isolated soft start-lane prior. All current lane/route candidates survive.

    Reuses current route enumeration/clustering; capped output can still change.
    No candidate is invented and no future branch is directly suppressed.
    """
    temporary = copy.copy(network)
    weighted = tuple(sorted(
        (replace(c,score=c.score*1.25) if c.lanelet_id==chosen else c for c in candidates),
        key=lambda c:c.score,reverse=True))
    temporary.lane_candidates = lambda x,y,yaw: weighted
    return temporary


def prepare_gt(inputs, sensor, transform):
    from replay_base5_compiled_tracking import ego_trajectory, world_detections, load_sensor4_calibration
    manifest = json.loads((inputs/'manifest.json').read_text())
    source = pickle.loads((inputs/'infos_val.pkl').read_bytes())['data_list']
    extrinsic = load_sensor4_calibration(sensor,
        ['camera_front','camera_front_left','camera_front_right','camera_back'])['base_from_lidar']
    ego = {k:ego_trajectory(Path(v['raw_bag'])) for k,v in manifest['cases'].items()
           if k.startswith(('H03','R01'))}
    by_frame = {}; trajectories = {}
    for row in source:
        case = row['scene_id']
        if case not in ego or not row['scoring_eligible']:
            continue
        observations = []
        for instance in row['instances']:
            if instance['bbox_label_3d'] != 0:
                continue
            box=instance['bbox_3d']; key=(case,instance['track_key'])
            d=world_detections([dict(center=box[:3],yaw=box[6])],row['timestamp'],ego[case],extrinsic)[0]
            point,_=lanelet_coordinates(d['center'],d['yaw'],transform)
            observations.append(dict(center=point,key=key))
            trajectories.setdefault(key,[]).append((row['timestamp'],point))
        by_frame[(case,round(row['timestamp'],6))]=observations
    return by_frame,trajectories


def run(network, frames, transform=None, gt=None, case=None):
    from analyze_base5_runtime_effects import match
    counts = Counter(); path_counts = Counter(); margins = []; examples = []; branch_separation = []
    fde = {str(h):dict(top1=[], oracle_minK=[], cartesian_cv=[], history_top1=[],
                     history_oracle_minK=[]) for h in (1,2,3)}
    baseline_history = {}; guarded_history = {}; trace = []
    for frame in frames:
        if frame['frame_id'] != 'map':
            raise ValueError('Require frozen map-frame actual ROS output')
        stamp = frame['stamp']
        assignments = {}
        if gt is not None:
            observations = gt[0].get((case,round(stamp,6)),[])
            vehicles = [o for o in frame['objects'] if o['class_name']=='vehicle']
            mapped = [dict(center=lanelet_coordinates(o['center'],o['yaw'],transform)[0]) for o in vehicles]
            pairs,_,_ = match(mapped,observations,2.)
            assignments={vehicles[i]['id']:observations[j]['key'] for i,j in pairs}
        for obj in frame['objects']:
            if obj['class_name'] != 'vehicle':
                continue
            counts['vehicle_observations'] += 1
            (x, y, z), yaw = lanelet_coordinates(obj['center'], obj['yaw'], transform)
            ident = obj['id']
            speed = obj['body_velocity'][0]
            if speed < network.config.minimum_speed_mps:
                counts['low_speed_fallback'] += 1
                baseline_history.pop(ident, None); guarded_history.pop(ident, None)
                continue
            candidates = network.lane_candidates(x, y, yaw)
            old_guarded = guarded_history.get(ident)
            guarded = history_choice(candidates, old_guarded, stamp, network.records)
            trajectories = network.predict(x, y, z, yaw, speed,
                                           tuple(i / 10 for i in range(1, 31)))
            # Only a continuous, ambiguous start-lane conflict changes weights.
            biased = trajectories
            if candidates and guarded != candidates[0].lanelet_id:
                biased = history_weighted_network(network,candidates,guarded).predict(
                    x,y,z,yaw,speed,tuple(i/10 for i in range(1,31)))
            if {p.lanelet_ids for p in trajectories} != {p.lanelet_ids for p in biased}:
                counts['history_published_route_set_changed'] += 1
            if trajectories and biased:
                if trajectories[0].lanelet_ids != biased[0].lanelet_ids:
                    counts['history_top1_route_changed'] += 1
                if len(selected_indices([p.confidence for p in trajectories],'roundabout_threshold')) != len(
                        selected_indices([p.confidence for p in biased],'roundabout_threshold')):
                    counts['history_roundabout_eligible_count_changed'] += 1
            path_counts[len(trajectories)] += 1
            if not trajectories:
                counts['no_complete_map_path_fallback'] += 1
            else:
                counts['map_path_observations'] += 1
                confidences = [p.confidence for p in trajectories]
                if len(trajectories) > 1:
                    counts['multiple_map_paths'] += 1
                    margins.append(confidences[0] - confidences[1])
                    if trajectories[0].lanelet_ids[0] == trajectories[1].lanelet_ids[0]:
                        counts['top2_same_start_route_branch'] += 1
                    else:
                        counts['top2_different_start_lane_ambiguity'] += 1
                    a, b = trajectories[0].poses[-1], trajectories[1].poses[-1]
                    branch_separation.append(math.hypot(a[0] - b[0], a[1] - b[1]))
                if len(selected_indices(confidences, 'lc_max_ties')) > 1:
                    counts['lc_exact_top_ties'] += 1
                if len(selected_indices(confidences, 'roundabout_threshold')) > 1:
                    counts['roundabout_multiple_eligible'] += 1
                if len(examples) < 5 and len(trajectories) > 1:
                    examples.append(dict(stamp=stamp, id=ident,
                        confidences=confidences, routes=[list(p.lanelet_ids) for p in trajectories]))
                if ident in assignments:
                    series=gt[1][assignments[ident]]
                    for horizon in (1,2,3):
                        future=future_point(series,stamp+horizon)
                        if future is None:
                            continue
                        errors=[math.hypot(p.poses[horizon*10-1][0]-future[0],
                                           p.poses[horizon*10-1][1]-future[1]) for p in trajectories]
                        fde[str(horizon)]['top1'].append(errors[0])
                        fde[str(horizon)]['oracle_minK'].append(min(errors))
                        biased_errors=[math.hypot(p.poses[horizon*10-1][0]-future[0],
                                                  p.poses[horizon*10-1][1]-future[1]) for p in biased]
                        fde[str(horizon)]['history_top1'].append(biased_errors[0])
                        fde[str(horizon)]['history_oracle_minK'].append(min(biased_errors))
                        vx,vy=obj['velocity']
                        if transform:
                            a=transform['rpy'][2];vx,vy=math.cos(a)*vx-math.sin(a)*vy,math.sin(a)*vx+math.cos(a)*vy
                        fde[str(horizon)]['cartesian_cv'].append(math.hypot(x+vx*horizon-future[0],y+vy*horizon-future[1]))
            if not candidates:
                counts['no_lane_candidate'] += 1
                baseline_history.pop(ident, None); guarded_history.pop(ident, None)
                continue
            baseline = candidates[0].lanelet_id
            if guarded != baseline:
                counts['history_changed_primary_start'] += 1
            for name, selected, history in (('baseline', baseline, baseline_history),
                                            ('guarded', guarded, guarded_history)):
                old = history.get(ident)
                if old and 0 < stamp - old[0] <= .5:
                    counts[name + '_continuous_pairs'] += 1
                    if selected != old[1]:
                        counts[name + '_start_lane_changes'] += 1
                        if selected not in network.records[old[1]].following_ids:
                            counts[name + '_non_successor_changes'] += 1
                history[ident] = (stamp, selected)
            trace.append(dict(stamp=stamp, id=ident, center=obj['center'],
                              baseline_start=baseline, guarded_start=guarded,
                              candidates=[dict(lane=c.lanelet_id, score=c.score,
                                               distance=c.projection.distance) for c in candidates]))
    return dict(counts=dict(counts), map_path_count=dict(path_counts),
                top1_top2_confidence_margin=stats(margins),
                top2_separation_at3_m=stats(branch_separation),
                future_gt_fde_m={h:{mode:stats(values) for mode,values in modes.items()} for h,modes in fde.items()},
                oracle_better_by05m={h:sum(a-b>.5 for a,b in zip(modes['top1'],modes['oracle_minK'])) for h,modes in fde.items()},
                history_improved_by05m={h:sum(a-b>.5 for a,b in zip(modes['top1'],modes['history_top1'])) for h,modes in fde.items()},
                history_worsened_by05m={h:sum(b-a>.5 for a,b in zip(modes['top1'],modes['history_top1'])) for h,modes in fde.items()},
                examples=examples, trace=trace)


def main():
    from perception_object_pipeline.map_based_prediction import LaneletNetwork
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--messages', type=Path, required=True)
    parser.add_argument('--map-file', type=Path, required=True)
    parser.add_argument('--projector-file', type=Path, required=True)
    parser.add_argument('--source-coordinate-frame', choices=('map', 'morai_map'), required=True,
                        help='Physical coordinate contract, not the replay header spelling')
    parser.add_argument('--alignment-file', type=Path)
    parser.add_argument('--gt-inputs', type=Path)
    parser.add_argument('--sensor-file', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh report required')
    network = LaneletNetwork.load(args.map_file, args.projector_file)
    transform = None
    if args.source_coordinate_frame == 'morai_map':
        if args.alignment_file is None:
            raise ValueError('MORAI-world fixture needs explicit map alignment')
        import yaml
        transform = yaml.safe_load(args.alignment_file.read_text())['map_to_morai_map']
    elif args.alignment_file is not None:
        raise ValueError('Already map coordinates must not be transformed twice')
    with gzip.open(args.messages, 'rt') as handle:
        data = json.load(handle)
    gt = None
    if args.gt_inputs:
        if not args.sensor_file or args.source_coordinate_frame != 'morai_map':
            raise ValueError('This frozen GT adapter requires sensor file and MORAI fixture contract')
        gt=prepare_gt(args.gt_inputs,args.sensor_file,transform)
    rows = {}
    for key in ('O02_164012/off/policy', 'P01_163434/off/policy',
                'H03_162301/off/policy', 'R01_163034/off/policy'):
        rows[key] = run(network, data[key], transform, gt, key.split('/')[0])
        print(key, rows[key]['counts'], flush=True)
    report = dict(input_sha256=hashlib.sha256(args.messages.read_bytes()).hexdigest(),
                  map_sha256=network.map_sha256,
                  predictor_source_sha256=hashlib.sha256((Path(__file__).resolve().parents[4]/
                      'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/map_based_prediction.py').read_bytes()).hexdigest(),
                  projector_sha256=hashlib.sha256(args.projector_file.read_bytes()).hexdigest(),
                  sensor_sha256=hashlib.sha256(args.sensor_file.read_bytes()).hexdigest() if args.sensor_file else None,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), rows=rows,
                  source_coordinate_frame=args.source_coordinate_frame,
                  alignment=transform,
                  alignment_sha256=hashlib.sha256(args.alignment_file.read_bytes()).hexdigest()
                      if args.alignment_file else None,
                  gt_infos_sha256=hashlib.sha256((args.gt_inputs/'infos_val.pkl').read_bytes()).hexdigest()
                      if args.gt_inputs else None,
                  production_changed=False,
                  limitation='Frozen actual E24 ROS tracker states + current map core. No planner, '
                             'no ground-truth maneuver labels. Optional FDE uses catalog/interpolated '
                             'GT only on current 2m-matched and future-supported observations; '
                             'misses/fallbacks are not folded into its conditional mean. Oracle minK '
                             'uses future GT in evaluation only and is not a deployable choice. '
                             'Non-successor changes may be real '
                             'lane changes or overlapping map segments, not confirmed errors. '
                             'Guarded start selection and a separate 1.25x soft-prior copy are '
                             'research-only. Production publisher is unchanged. Known dev bags '
                             'cannot certify maneuver intent or closed-loop safety.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
