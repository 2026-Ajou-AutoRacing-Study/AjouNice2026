#!/usr/bin/env python3
"""Read-only recorded ROI-cap counterfactual; never starts ROS/GPU/MORAI.

Existing accepted projections and explicit uncertainty-only rejections are
retested against finite X90 / candidate Y caps. This is NOT object detection,
head-identity GT, classifier accuracy, policy permission or a driving PASS.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
TOPIC = '/perception/traffic_light_recognition/map_rois'
HEADS = {201351: {'C1256W000077'}, 201365: {'C1256W000091'},
         201290: {'C1256W000016', 'C1256W000018'}, 201326: {'C1256W000052'}}
CAPS = (180, 200, 210, 240)
SELECTOR = ROOT / 'base_ws/src/perception/traffic_light_detection/scripts/traffic_light_map_selector.py'


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b''):
            value.update(block)
    return value.hexdigest()


def distribution(values):
    values = sorted(x for x in values if isinstance(x, (int, float))
                    and not isinstance(x, bool) and math.isfinite(x))
    if not values:
        return {'n': 0}
    return dict(n=len(values), min=values[0], mean=statistics.mean(values),
                p50=statistics.median(values), p95=values[math.ceil(.95 * len(values))-1],
                max=values[-1])


def padding_inside(values, y_cap):
    return (isinstance(values, (list, tuple)) and len(values) == 2
            and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                    and math.isfinite(x) and x >= 0 for x in values)
            and values[0] <= 90 and values[1] <= y_cap)


def current_eligible(packet):
    group = packet.get('group_id')
    return packet.get('status') == 'ready' and group in HEADS and any(
        row.get('head_id') in HEADS[group] and row.get('eligible') is True
        for row in packet.get('rois', []))


def eligible_at_cap(packet, y_cap):
    group = packet.get('group_id')
    if packet.get('status') != 'ready' or group not in HEADS:
        return False
    accepted = any(row.get('head_id') in HEADS[group] and row.get('eligible') is True
                   and padding_inside(row.get('padding_xy_px'), y_cap)
                   for row in packet.get('rois', []))
    rescued = any(row.get('head_id') in HEADS[group]
                  and row.get('reason') == 'projection_uncertainty_exceeds_bound'
                  and padding_inside(row.get('required_padding_xy_px'), y_cap)
                  for row in packet.get('rejected_heads', []))
    return accepted or rescued


def summarize_packets(packets, stall_start_sec=None):
    counts = defaultdict(Counter)
    values = defaultdict(lambda: defaultdict(list))
    contracts = Counter()
    active, missing_runs = {}, defaultdict(list)
    previous_t = None

    def close(cap, end):
        run = active.pop(cap, None)
        if run is not None and not run['eligible']:
            run.update(end_sec=end, duration_sec=max(0., end-run['start_sec']))
            missing_runs[cap].append(run)

    for elapsed, packet in packets:
        if previous_t is not None and elapsed < previous_t:
            raise ValueError('ROI record timestamps moved backwards')
        previous_t = elapsed
        group = packet.get('group_id')
        # Close at a route transition; never extend A/B/C outages to bag EOF.
        if group not in HEADS:
            for cap in CAPS:
                close(cap, elapsed)
            continue
        config = packet.get('selector_config', {})
        contracts[json.dumps(config, sort_keys=True)] += 1
        if config.get('max_padding_px') != 90:
            raise ValueError('This audit requires the recorded X90 contract')
        before = bool(current_eligible(packet))
        count = counts[group]
        count['packets'] += 1
        count['recorded_eligible_packets'] += before
        for cap in CAPS:
            after = eligible_at_cap(packet, cap)
            count['eligible_y' + str(cap)] += after
            count['newly_eligible_y' + str(cap)] += after and not before
            count['recorded_eligible_lost_y' + str(cap)] += before and not after
            run = active.get(cap)
            if run is None or run['group_id'] != group or run['eligible'] != after:
                close(cap, elapsed)
                active[cap] = dict(group_id=group, eligible=after, start_sec=elapsed, packets=1)
            else:
                run['packets'] += 1
        if packet.get('status') != 'ready':
            count['packet_missing:' + str(packet.get('reason'))] += 1
        for rejected in packet.get('rejected_heads', []):
            if rejected.get('head_id') not in HEADS[group]:
                continue
            reason = str(rejected.get('reason'))
            count['head_reject:' + reason] += 1
            values[group]['depth_m:' + reason].append(rejected.get('depth_m'))
            padding = rejected.get('required_padding_xy_px')
            if reason == 'projection_uncertainty_exceeds_bound' and isinstance(padding, list) and len(padding) == 2:
                values[group]['required_x_px'].append(padding[0])
                values[group]['required_y_px'].append(padding[1])
                count['uncertainty_head_x_over90'] += padding[0] > 90
                for key in ('position_sigma_xy_m', 'position_sigma_z_m', 'yaw_sigma_radians'):
                    values[group][key].append(packet.get(key))
    if previous_t is not None:
        for cap in CAPS:
            close(cap, previous_t)
    runs = {}
    for cap in CAPS:
        per_group = {}
        for group in HEADS:
            group_runs = [row for row in missing_runs[cap] if row['group_id'] == group]
            per_group[str(group)] = dict(
                longest_recorded_missing_span_sec=max((r['duration_sec'] for r in group_runs), default=0.),
                longest_runs=sorted(group_runs, key=lambda r: r['duration_sec'], reverse=True)[:10])
        if stall_start_sec is not None:
            clipped = [dict(row, window_start_sec=max(stall_start_sec, row['start_sec']),
                            window_missing_span_sec=max(0., row['end_sec']-max(stall_start_sec, row['start_sec'])))
                       for row in missing_runs[cap] if row['group_id'] == 201326 and row['end_sec'] > stall_start_sec]
            per_group['D_stall_window'] = dict(start_sec=stall_start_sec,
                longest_recorded_missing_span_sec=max((r['window_missing_span_sec'] for r in clipped), default=0.),
                longest_runs=sorted(clipped, key=lambda r: r['window_missing_span_sec'], reverse=True)[:10])
        runs[str(cap)] = per_group
    return dict(packet_counts={str(g): dict(c) for g, c in counts.items()},
                rejection_distributions={str(g): {key: distribution(v) for key, v in row.items()}
                                        for g, row in values.items()},
                recorded_selector_contracts=[dict(config=json.loads(key), packets=n) for key, n in contracts.items()],
                missing_spans_by_y_cap=runs)


def development_replay(directory):
    """Exact old development replay; no fresh holdout or new inference claim."""
    sys.path[:0] = [str(ROOT / 'scripts/evaluation'), str(SELECTOR.parent)]
    from traffic_light_map_selector import load_geometry, load_camera, project_heads, associate_boxes, selector_config
    from evaluate_traffic_light_pipeline import development_camera_transform, manual_identity
    curated = directory / 'housing_detector_development_curated_v1.json'
    baseline = directory / 'pipeline_baseline640_development348_v1.json'
    geometry = ROOT / 'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json'
    sensor = ROOT / 'configs/sensors/Sensor_competition_local.json'
    inputs = [curated, baseline, geometry, sensor, SELECTOR,
              ROOT / 'scripts/evaluation/evaluate_traffic_light_pipeline.py']
    fingerprints = {str(p.resolve()): digest(p) for p in inputs}
    geom, cam = load_geometry(geometry), load_camera(sensor)
    annotations = {row['image_sha256']: row for row in json.loads(curated.read_text())['records']}
    predictions = json.loads(baseline.read_text())['records']
    gid_by_head = {row['id']: row.get('group_id') for row in geom['heads']}

    def box(values):
        x1, y1, x2, y2, confidence = values
        return ((x1+x2)/2560, (y1+y2)/1440, (x2-x1)/1280, (y2-y1)/720,
                (x1, y1, x2, y2, confidence))

    results = {}
    for cap in CAPS:
        count = Counter()
        # This historical X90 comparison must not inherit newer runtime bounds.
        cfg = selector_config({'max_vertical_padding_px': cap, 'max_padding_px': 90.,
                               'max_yaw_sigma_degrees': 3.})
        for row in predictions:
            rec = annotations[row['image_sha256']]
            group = row['group_regulatory_id']
            transform = development_camera_transform(rec['ego'], cam)
            rois, _ = project_heads(geom, cam, transform, group, position_sigma_m=.027136410963869192,
                                   yaw_sigma_rad=.02905, vertical_sigma_m=1., **cfg.projection_kwargs())
            selected, _ = associate_boxes([box(x) for x in row['detections']], rois, group, config=cfg)
            count['frames'] += 1
            count['selected'] += selected is not None
            if selected is not None:
                identity = manual_identity(list(selected[4][:4]), rec['boxes'])
                count['identity_known'] += identity.get('known', False)
                if identity.get('known'):
                    same = gid_by_head.get(identity.get('native_id')) == group
                    count['correct_mission'] += same
                    count['wrong_mission'] += not same
            for annotation in rec['boxes']:
                if gid_by_head.get(annotation['native_id']) != group:
                    count['actual_other_head_challenges'] += 1
                    count['actual_other_head_false_matches'] += associate_boxes(
                        [box(annotation['xyxy']+[.9])], rois, group, config=cfg)[0] is not None
            for roi in rois:
                if roi.get('eligible'):
                    a, b, x, y = roi['bbox_xyxy']
                    cx, cy = (a+x)/2, (b+y)/2+cam.fy*2.8/roi['depth_m']
                    width, height = (x-a)*.55, (y-b)*2.8
                    count['synthetic_pedestrian_challenges'] += 1
                    count['synthetic_pedestrian_false_matches'] += associate_boxes(
                        [box([cx-width/2, cy-height/2, cx+width/2, cy+height/2, .9])],
                        rois, group, config=cfg)[0] is not None
        results[str(cap)] = dict(count)
    if any(digest(path) != value for path, value in fingerprints.items()):
        raise RuntimeError('Development source/input changed during audit; rerun with a fresh output')
    return dict(input_sha256=fingerprints, results=results,
                noise_assumption=dict(sigma_xy_m=.027136410963869192, sigma_z_m=1., yaw_sigma_rad=.02905),
                scope='previously_observed_development_348_saved_baseline_boxes_not_new_inference_or_unseen_test',
                identity_reference='assistant visual housing QC, not independent human GT')


def audit(bag_path, stall_start_sec=None, development_root=None):
    import rosbag
    before = bag_path.stat()
    fingerprint = digest(bag_path)
    selector_sha = digest(SELECTOR)
    with rosbag.Bag(str(bag_path)) as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        packets = ((stamp.to_sec()-start, json.loads(message.data))
                   for _, message, stamp in bag.read_messages(topics=[TOPIC]))
        result = summarize_packets(packets, stall_start_sec)
    result.update(schema='traffic_light_roi_cap_audit_v1', bag=str(bag_path), bag_sha256=fingerprint,
                  bag_size_bytes=before.st_size, bag_duration_sec=end-start,
                  audit_source_sha256=digest(Path(__file__)), selector_source_sha256=selector_sha,
                  topic=TOPIC, x_bound_px=90, y_caps_px=list(CAPS),
                  limitations=['Recorded ROI eligibility counterfactual only; not actual detector/selection/CNN/GO.',
                               'No native lamp state or independent pixel/head-identity GT for the live bag.',
                               'Rejected-head counts are per physical head; eligibility counts are per packet (ANY allowed head).',
                               'Counterfactual preserves preceding geometry/FOV/facing/pose guards and X90.',
                               'Timestamp spans describe recorded packet state, not guaranteed continuous transport coverage.',
                               'No source, sensor, GT, model, ROS parameter or process is modified.'])
    if development_root is not None:
        result['old_development_replay'] = development_replay(development_root)
    after = bag_path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns) or digest(SELECTOR) != selector_sha:
        raise RuntimeError('Bag/selector source changed during audit; rerun with a fresh output')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stall-start-sec', type=float)
    parser.add_argument('--development-root', type=Path)
    args = parser.parse_args()
    if args.stall_start_sec is not None and (not math.isfinite(args.stall_start_sec) or args.stall_start_sec < 0):
        parser.error('stall-start-sec must be finite and nonnegative')
    target = args.output.resolve()
    target.relative_to(ROOT / 'artifacts/evaluation')
    if target.exists():
        parser.error('Use a fresh output; evidence is never overwritten')
    result = audit(args.bag.resolve(strict=True), args.stall_start_sec,
                   args.development_root.resolve(strict=True) if args.development_root else None)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(target), packet_counts=result['packet_counts'],
                          old_development_replay=bool(result.get('old_development_replay')))))


if __name__ == '__main__':
    main()
