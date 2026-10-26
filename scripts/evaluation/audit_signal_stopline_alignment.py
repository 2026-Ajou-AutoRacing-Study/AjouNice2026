#!/usr/bin/env python3
"""Compare stable stopped housing medians across explicit closed-bag reports.

Only original detector observations 2s after stopping to 0.5s before departure
count. A permissive pass, a short re-stop or missing observations cannot pass.
No ROS graph mutation, map edits, native control or new inference.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

GROUPS = dict(A=201351, B=201365, C=201290, D=201326)
HEADS = dict(A='C1256W000077', B='C1256W000091', C='C1256W000016', D='C1256W000052')


def stable_window_summary(frames, window, head_id=None):
    start = window['start_stamp_ns']+2_000_000_000
    end = window['end_stamp_ns']-500_000_000
    selected = [f for f in frames if start <= f['stamp'] <= end
                and (head_id is None or f['head_id'] == head_id)]
    if window['duration_sec'] < 3. or len(selected) < 5:
        return None
    values = [f['top'] for f in selected]
    return dict(start_ns=start, end_ns=end, samples=len(values),
        median_top_px=statistics.median(values), minimum_top_px=min(values),
        maximum_top_px=max(values), heads=dict(Counter(f['head_id'] for f in selected)))


def alignment_verdict(groups, maximum_spread_px=5.):
    medians = [g.get('median_top_px') for g in groups]
    if (len(groups) != 4 or {g['mission'] for g in groups} != set(GROUPS)
            or any(v is None or not math.isfinite(v) for v in medians)):
        return dict(status='insufficient_stable_stop_evidence', spread_px=None)
    bounds = [g.get('between_stop_median_range_px') or [g['median_top_px']]*2 for g in groups]
    spread = max(v for pair in bounds for v in pair)-min(v for pair in bounds for v in pair)
    return dict(status='aligned' if spread <= maximum_spread_px else 'adjustment_needed',
                spread_px=spread, required_maximum_spread_px=maximum_spread_px)


def same_group_geometry(source, target, mission):
    """Reuse a measurement only when this mission's full line/identity is exact.

    Other missions may have changed; this is NOT a same-whole-map drive claim.
    """
    def group(document):
        rows = [g for g in document['groups'] if g['group_id'] == GROUPS[mission]]
        if len(rows) != 1:
            raise ValueError('Require one explicit mission geometry')
        return rows[0]
    return source['frame'] == target['frame'] and group(source) == group(target)


def analyze(path, mission):
    import rosbag
    report = json.loads(path.read_text())
    if report['mission'] != mission or report['group_id'] != GROUPS[mission]:
        raise ValueError('Incorrect mission report')
    if report['front_bumper_offset_m'] != 3.845:
        raise ValueError('Vehicle reference point mismatch')
    bag_path = Path(report['bag'])
    if bag_path.suffix != '.bag':
        raise ValueError('Only closed bags qualify')
    before = bag_path.stat()
    frames = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, _ in bag.read_messages(topics=['/traffic_light_node/debug_classifier']):
            d = json.loads(msg.data)
            box = d.get('detector_bbox_xyxy')
            if d.get('group_id') == GROUPS[mission] and isinstance(box, list) and len(box) == 4:
                if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in box):
                    continue
                frames.append(dict(stamp=d['source_stamp_ns'], top=box[1],
                    head_id=d.get('head_id', d.get('selected_head_id', 'unrecorded'))))
    windows = [stable_window_summary(frames, w, HEADS[mission]) for w in report['stationary_windows']]
    windows = [w for w in windows if w is not None]
    # Do not choose the one flattering stop if repeated stable stops disagree.
    medians = [w['median_top_px'] for w in windows]
    after = bag_path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError('Input bag changed during audit')
    return dict(mission=mission, compared_physical_head=HEADS[mission], stable_windows=windows,
        median_top_px=statistics.median(medians) if medians else None,
        between_stop_median_range_px=[min(medians), max(medians)] if medians else None,
        report=str(path.resolve()), report_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        bag=str(bag_path), report_observed_approach_min_top_px=(
            report.get('approach_detector_top_margin_px') or {}).get('min'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', action='append', required=True, metavar='A=measurements.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--reference-geometry', type=Path,
                   help='Explicit target geometry for reusing unchanged mission lines across candidates')
    p.add_argument('--source-geometry', action='append', type=Path, default=[],
                   help='Original hashed geometry documents, needed for cross-candidate reuse')
    args = p.parse_args()
    paths = {}
    for entry in args.report:
        mission, path = entry.split('=', 1)
        if mission not in GROUPS or mission in paths:
            p.error('Unique A/B/C/D reports required')
        paths[mission] = Path(path)
    if set(paths) != set(GROUPS) or args.output.exists():
        p.error('Require all four reports and a fresh output')
    source_hashes = {m: json.loads(path.read_text())['geometry_sha256'] for m,path in paths.items()}
    reuse = None
    if args.reference_geometry is not None:
        geometries = {hashlib.sha256(path.read_bytes()).hexdigest(): json.loads(path.read_text())
                      for path in args.source_geometry+[args.reference_geometry]}
        target = json.loads(args.reference_geometry.read_text())
        for m, digest in source_hashes.items():
            if digest not in geometries or not same_group_geometry(geometries[digest], target, m):
                p.error('Mission line differs or original hashed geometry missing: '+m)
        reuse = dict(reference_geometry=str(args.reference_geometry.resolve()),
            reference_sha256=hashlib.sha256(args.reference_geometry.read_bytes()).hexdigest(),
            report_geometry_sha256=source_hashes, exact_mission_geometry_verified=True,
            limitation='Different candidate runs, not a same-map full-course regression')
    elif len(set(source_hashes.values())) != 1:
        p.error('All reports need the same geometry or explicit per-mission equality proof')
    groups = [analyze(paths[m], m) for m in GROUPS]
    result = dict(schema_version=1,
        scope='Stable observed housing geometry only; not all-frame visibility or uninterrupted permission',
        criteria=dict(minimum_stop_sec=3., discard_initial_sec=2., discard_final_sec=.5,
                      minimum_observed_frames=5, maximum_between_mission_spread_px=5.),
        groups=groups, alignment=alignment_verdict(groups), runtime_default_changed=False,
        unchanged_mission_reuse=reuse)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
