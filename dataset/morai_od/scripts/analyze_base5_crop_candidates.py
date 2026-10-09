#!/usr/bin/env python3
"""CPU-only, read-only raw instance-mask comparison; never import model/ROS code."""
import argparse
import ast
from collections import defaultdict
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image

from report_base5_camera_crop import CAMERAS, CLASS_NAMES, configure_plot, observations

PROFILES = [
    dict(id='bottom320', name='320×896 아래 정렬', h=320, w=896, side_start=None),
    dict(id='shift320', name='320×896 측면 위로 이동', h=320, w=896, side_start=.35),
    dict(id='bottom384', name='384×896 아래 정렬', h=384, w=896, side_start=None),
    dict(id='shift384', name='384×896 측면 위로 이동', h=384, w=896, side_start=.28),
    dict(id='bottom448', name='448×896 아래 정렬', h=448, w=896, side_start=None),
    dict(id='shift448', name='448×896 측면 위로 이동', h=448, w=896, side_start=.20),
    dict(id='bottom384wide', name='384×1056 아래 정렬', h=384, w=1056, side_start=None),
    dict(id='shift384wide', name='384×1056 측면 위로 이동', h=384, w=1056, side_start=.33),
    dict(id='shift448wide', name='448×1056 측면 위로 이동', h=448, w=1056, side_start=.25),
    dict(id='bottom448w1056', name='448×1056 아래 정렬', h=448, w=1056, side_start=None),
    dict(id='bottom480w1056', name='480×1056 아래 정렬', h=480, w=1056, side_start=None),
    dict(id='bottom512w1056', name='512×1056 아래 정렬', h=512, w=1056, side_start=None),
    dict(id='bottom448wide', name='448×1248 아래 정렬', h=448, w=1248, side_start=None),
]
SIDE_CAMERAS = ('CAMERA_2', 'CAMERA_3')
DISTANCES = [(0, 5), (5, 10), (10, 30), (30, 60), (60, 100), (100, math.inf)]
ANALYSIS_ROI = (-100, -30, -3, 100, 30, 2)


def inside_analysis_roi(center):
    """Mirror the candidate info converter's lower-inclusive center ROI contract."""
    return all(ANALYSIS_ROI[i] <= center[i] < ANALYSIS_ROI[i + 3] for i in range(3))


def geometry(camera, profile):
    """Match the current transform's int resize and ideal-scale augmentation matrix."""
    _, width, height = CAMERAS[camera]
    base = profile['w'] / 1280
    scale = base * 1280 / width
    new_width, new_height = int(width * scale), int(height * scale)
    top = new_height - profile['h']
    if camera in SIDE_CAMERAS and profile['side_start'] is not None:
        top = int(profile['side_start'] * new_height)
    left = max(0, new_width - profile['w']) // 2
    if top < 0 or top + profile['h'] > new_height or left + profile['w'] > new_width:
        raise ValueError('Invalid crop for ' + camera + '/' + profile['id'])
    matrix = np.array([[scale, 0., -left], [0., scale, -top], [0., 0., 1.]])
    return dict(scale=scale, resize_wh=[new_width, new_height],
                crop_xyxy=[left, top, left + profile['w'], top + profile['h']],
                native_window_xyxy=[left / scale, top / scale,
                    (left + profile['w']) / scale, (top + profile['h']) / scale],
                augmentation_matrix=matrix.tolist())


def native_row_interval(height, scale, top, final_height):
    """Original pixel centers inside the continuous inverse crop window, [lo,hi)."""
    lo = max(0, min(height, math.ceil(top / scale - .5)))
    hi = max(lo, min(height, math.ceil((top + final_height) / scale - .5)))
    return lo, hi


def mask_statistics(mask, prefix, rect, geo):
    """Native foreground retention plus nearest-mask resized pixel support proxy."""
    left, top, right, bottom = geo['crop_xyxy']
    total = int(prefix[-1])
    lo, hi = native_row_interval(mask.shape[0], geo['scale'], top, bottom - top)
    if left != 0 or right != geo['resize_wh'][0]:
        raise ValueError('Row histogram requires full-width crops')
    kept = int(prefix[hi] - prefix[lo])
    resized = cv2.resize(mask, tuple(geo['resize_wh']), interpolation=cv2.INTER_NEAREST)
    model_pixels = int(resized[top:bottom, left:right].sum())
    x1, y1, x2, y2 = rect
    native_width, native_height = mask.shape[1], mask.shape[0]
    # Clamp annotation rectangles to the original sensor before testing extra crop loss.
    x1, x2 = max(0, x1), min(native_width, x2)
    y1, y2 = max(0, y1), min(native_height, y2)
    scale = geo['scale']
    extent = [max(0., min(right, x2 * scale) - max(left, x1 * scale)),
              max(0., min(bottom, y2 * scale) - max(top, y1 * scale))]
    lost_top = int(prefix[lo])
    lost_bottom = int(prefix[-1] - prefix[hi])
    return dict(native_kept=kept, native_fraction=kept / total,
        lost_top=lost_top, lost_bottom=lost_bottom, model_mask_pixels_nearest=model_pixels,
        model_bbox_extent_px=extent, candidate_camera_gate=model_pixels >= 16 and min(extent) >= 4)


def observation_key(scene, camera, evidence):
    return (scene, camera, evidence['camera_instance_id'], evidence['annotation_class'],
            tuple(evidence['native_rect']))


def measure(entries, rows, max_groups=0):
    grouped = defaultdict(list)
    for index, row in enumerate(rows):
        grouped[(row['capture_root'], row['camera'])].append(index)
    metrics, sweeps = {}, defaultdict(list)
    geometry_table = {c: {p['id']: geometry(c, p) for p in PROFILES} for c in CAMERAS}
    started = time.monotonic()
    groups = sorted(grouped.items())
    if max_groups:
        groups = groups[:max_groups]
    for number, ((root, camera), indices) in enumerate(groups, 1):
        folder = Path(root) / camera
        instance_files = list(folder.glob('*_Instance.png'))
        annotation_files = list(folder.glob('*_BBox2D.txt'))
        if len(instance_files) != 1 or len(annotation_files) != 1:
            raise ValueError('Expected one completed camera capture: ' + str(folder))
        with Image.open(instance_files[0]) as image:
            pixels = np.asarray(image.convert('RGBA'))
        _, width, height = CAMERAS[camera]
        if pixels.shape[:2] != (height, width):
            raise ValueError('Sensor dimensions changed: ' + str(folder))
        annotations = {}
        for line in annotation_files[0].read_text().splitlines():
            fields = line.split(maxsplit=6)
            if len(fields) == 7:
                key = (int(fields[5]), fields[0], tuple(float(x) for x in fields[1:5]))
                annotations[key] = ast.literal_eval(fields[6])
        for index in indices:
            row = rows[index]
            key = (row['camera_instance_id'], row['raw_class'], tuple(row['native_rect']))
            if key not in annotations:
                raise ValueError('Cannot resolve camera annotation: ' + str(key))
            colour = np.asarray(annotations[key], dtype=np.uint8)
            mask = (pixels == colour).all(axis=2).astype(np.uint8)
            prefix = np.r_[0, np.cumsum(mask.sum(axis=1), dtype=np.int64)]
            if int(prefix[-1]) != row['native_pixels']:
                raise ValueError('Raw instance and saved audit differ: ' + str(folder))
            metrics[index] = {p['id']: mask_statistics(mask, prefix, row['native_rect'],
                               geometry_table[camera][p['id']]) for p in PROFILES}
            if camera in SIDE_CAMERAS and row['native_pixels'] >= 16:
                sweep = []
                for start in np.linspace(0, 1 - 320 / 672, 53):
                    top = int(start * 672)
                    lo, hi = native_row_interval(height, 1.4, top, 320)
                    sweep.append(float(prefix[hi] - prefix[lo]) / prefix[-1])
                sweeps[index] = sweep
        if number == 16 or number % 100 == 0 or number == len(groups):
            elapsed = time.monotonic() - started
            print(json.dumps(dict(camera_files=number, total_files=len(groups),
                elapsed_sec=round(elapsed, 2), eta_sec=round(elapsed / number * (len(groups) - number), 1))), flush=True)
    return metrics, dict(sweeps), geometry_table, len(groups), time.monotonic() - started


def summarize(indices, metrics, profile):
    values = [metrics[i][profile] for i in indices if i in metrics]
    n = len(values)
    if not n:
        return dict(n=0)
    fraction = np.asarray([r['native_fraction'] for r in values])
    return dict(n=n, full_pct=float((fraction == 1).mean() * 100),
        partial_pct=float(((fraction > 0) & (fraction < 1)).mean() * 100),
        removed_pct=float((fraction == 0).mean() * 100),
        kept_at_least_90pct=float((fraction >= .9).mean() * 100),
        kept_at_least_50pct=float((fraction >= .5).mean() * 100),
        mean_native_retention_pct=float(fraction.mean() * 100),
        candidate_camera_gate_pct=100 * sum(r['candidate_camera_gate'] for r in values) / n,
        model_pixels_p10_p50_p90=np.percentile([r['model_mask_pixels_nearest'] for r in values], [10, 50, 90]).tolist(),
        model_bbox_height_px_p10_p50_p90=np.percentile([r['model_bbox_extent_px'][1] for r in values], [10, 50, 90]).tolist(),
        top_loss_observations=sum(r['lost_top'] > 0 for r in values),
        bottom_loss_observations=sum(r['lost_bottom'] > 0 for r in values))


def component_unions(entries, rows, metrics):
    keys = {(r['scene'], r['camera'], r['camera_instance_id'], r['raw_class'],
             tuple(r['native_rect'])): i for i, r in enumerate(rows)}
    components = []
    ambiguous = 0
    for entry in entries:
        if entry['status'] != 'accepted_candidate':
            continue
        for component in entry['audit']['components']:
            if component.get('camera_component_ambiguous', False):
                ambiguous += 1
                continue
            indices = []
            for camera in component['cameras']:
                key = observation_key(entry['id'], camera['camera'], camera)
                if key in keys:
                    indices.append(keys[key])
            # Avoid partial camera groups in a small diagnostic run.
            if not indices or any(i not in metrics for i in indices):
                continue
            candidates = {}
            for p in PROFILES:
                candidates[p['id']] = dict(
                    any_native_pixel=any(metrics[i][p['id']]['native_kept'] > 0 for i in indices),
                    camera_gate=any(metrics[i][p['id']]['candidate_camera_gate'] for i in indices))
            components.append(dict(scene=entry['id'], component_id=component['component_id'],
                asset=component['asset'], training_class=component['training_class'],
                lidar_points=component['lidar_points'],
                inside_candidate_roi=inside_analysis_roi(component['center_lidar_m']),
                saved_positive_lidar1_camera0=component['positive_lidar1_camera0'],
                candidates=candidates))
    totals = {}
    for p in PROFILES:
        candidates = [r['candidates'][p['id']] for r in components]
        zero = [r for r in components if r['lidar_points'] == 0]
        totals[p['id']] = dict(camera_native_visible_components=len(components),
            any_camera_native_pixel=sum(c['any_native_pixel'] for c in candidates),
            any_camera_gate=sum(c['camera_gate'] for c in candidates),
            zero_lidar_components=len(zero),
            zero_lidar_any_camera_gate=sum(r['candidates'][p['id']]['camera_gate'] for r in zero),
            zero_lidar_gate_gained_vs_bottom320=sum(
                not r['candidates']['bottom320']['camera_gate'] and r['candidates'][p['id']]['camera_gate'] for r in zero),
            zero_lidar_gate_lost_vs_bottom320=sum(
                r['candidates']['bottom320']['camera_gate'] and not r['candidates'][p['id']]['camera_gate'] for r in zero))
        roi_zero = [r for r in zero if r['inside_candidate_roi']]
        totals[p['id']]['within_candidate_roi_zero_lidar'] = dict(
            n=len(roi_zero),
            camera_gate=sum(r['candidates'][p['id']]['camera_gate'] for r in roi_zero),
            gained_vs_bottom320=sum(not r['candidates']['bottom320']['camera_gate'] and
                                   r['candidates'][p['id']]['camera_gate'] for r in roi_zero),
            lost_vs_bottom320=sum(r['candidates']['bottom320']['camera_gate'] and
                                 not r['candidates'][p['id']]['camera_gate'] for r in roi_zero))
    return totals, ambiguous, components


def draw(rows, metrics, sweeps, summaries, output):
    plt = configure_plot()
    profile_names = [p['name'] for p in PROFILES]
    xs = np.arange(len(PROFILES))
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    for ax, camera in zip(axes.flat, CAMERAS):
        ax.plot(xs, [summaries[camera][p['id']]['kept_at_least_90pct'] for p in PROFILES],
                marker='o', color='#16a34a', label='객체 픽셀 90% 이상 유지')
        ax.plot(xs, [summaries[camera][p['id']]['removed_pct'] for p in PROFILES],
                marker='x', color='#dc2626', label='객체 픽셀 전부 사라짐')
        ax.set_title(CAMERAS[camera][0] + f"  |  관측 {summaries[camera]['bottom320']['n']}개")
        ax.set_xticks(xs)
        ax.set_xticklabels(profile_names, rotation=35, ha='right', fontsize=8)
        ax.set_ylabel('관측 비율 (%)')
        ax.set_ylim(-2, 102)
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('실제 instance 픽셀로 비교한 crop 후보', fontsize=17)
    fig.text(.5, .01, '원본 객체 픽셀 ≥16인 카메라별 관측. 모든 비율은 원본에 보이던 객체 픽셀 기준이며 AP가 아니다.', ha='center')
    fig.tight_layout(rect=[0, .04, 1, .95])
    fig.savefig(output / 'candidate_pixel_retention.png', dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    starts = np.linspace(0, 1 - 320 / 672, 53) * 100
    for ax, camera in zip(axes, SIDE_CAMERAS):
        for label, select, color in [
            ('전체 관측', lambda r: True, '#2563eb'),
            ('근접 중심거리 <5m', lambda r: r['range_m'] < 5, '#d97706'),
            ('차량 중심거리 ≥60m', lambda r: r['range_m'] >= 60 and r['training_class'] == 'vehicle', '#9333ea')]:
            indices = [i for i in sweeps if rows[i]['camera'] == camera and select(rows[i])]
            if indices:
                values = np.asarray([sweeps[i] for i in indices])
                ax.plot(starts, (values >= .9).mean(axis=0) * 100,
                        label=f'{label} ({len(indices)}개)', color=color)
        ax.axvline(35, color='#374151', linestyle='--', label='위로 이동 후보 35%')
        ax.set_title(CAMERAS[camera][0])
        ax.set_xlabel('원본 위쪽 crop 시작 높이 (%)')
        ax.set_ylabel('원본 객체 픽셀 90% 이상 유지 비율 (%)')
        ax.set_ylim(0, 102)
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('320×896 고정 크기에서 crop 위치 이동의 장단점', fontsize=15)
    fig.tight_layout(rect=[0, 0, 1, .92])
    fig.savefig(output / 'side_crop_position_sweep.png', dpi=145)
    plt.close(fig)


def draw_examples(details, output):
    """Plot real RGB inputs with analytical target outlines, without altering raw files."""
    plt = configure_plot()
    from matplotlib.patches import Rectangle
    selected = []
    for cls, lo, hi, label in [('obstacle', 0, 5, '근접 장애물'),
                              ('vehicle', 0, 5, '근접 차량'),
                              ('vehicle', 60, 100, '원거리 차량')]:
        candidates = [r for r in details if r['camera'] in SIDE_CAMERAS and
                      r['training_class'] == cls and lo <= r['range_m'] < hi and
                      r['native_pixels'] >= 16]
        if cls == 'vehicle':
            ordinary = [r for r in candidates if r['asset'][:1].isdigit()]
            candidates = ordinary or candidates
        if lo == 0:
            candidates.sort(key=lambda r: r['candidates']['bottom320']['native_fraction'] -
                            r['candidates']['shift320']['native_fraction'], reverse=True)
        else:
            candidates.sort(key=lambda r: r['native_pixels'], reverse=True)
        if candidates:
            selected.append((label, candidates[0]))
    profile_ids = ['bottom320', 'shift320', 'bottom384', 'bottom448']
    profiles = {p['id']: p for p in PROFILES}
    fig, axes = plt.subplots(len(selected), 5, figsize=(17, 3.9 * len(selected)), squeeze=False)
    for n, (label, row) in enumerate(selected):
        path = next((Path(row['capture_root']) / row['camera']).glob('*_Intensity.png'))
        with Image.open(path) as im:
            rgb = np.asarray(im.convert('RGB'))
        rect = row['native_rect']
        for col in range(5):
            ax = axes[n, col]
            if col == 0:
                image = rgb
                box = rect
                title = '원본'
            else:
                profile = profiles[profile_ids[col - 1]]
                geo = geometry(row['camera'], profile)
                l, t, r, b = geo['crop_xyxy']
                image = cv2.resize(rgb, tuple(geo['resize_wh']), interpolation=cv2.INTER_CUBIC)[t:b, l:r]
                box = [rect[0] * geo['scale'] - l, rect[1] * geo['scale'] - t,
                       rect[2] * geo['scale'] - l, rect[3] * geo['scale'] - t]
                title = profile['name'] + '\n보이던 객체 픽셀 ' + str(round(
                    100 * row['candidates'][profile['id']]['native_fraction'], 1)) + '% 유지'
            ax.imshow(image)
            ax.add_patch(Rectangle((box[0], box[1]), box[2]-box[0], box[3]-box[1],
                                   fill=False, edgecolor='#ec4899', linewidth=1.5))
            ax.set_xlim(0, image.shape[1])
            ax.set_ylim(image.shape[0], 0)
            ax.set_title(title, fontsize=9)
            ax.set_xticks([])
            ax.set_yticks([])
            if col == 0:
                ax.set_ylabel(label + '\n' + row['asset'] + '\n' +
                              str(round(row['range_m'], 2)) + 'm', fontsize=9)
    fig.suptitle('원본 RGB와 crop 후보 비교  |  분홍 윤곽은 비교 대상의 원본 2D annotation', fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, .95])
    fig.savefig(output / 'rgb_crop_examples.png', dpi=150)
    plt.close(fig)
    return [dict(label=label, scene=r['scene'], asset=r['asset'], camera=r['camera'],
                 range_m=r['range_m']) for label, r in selected]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ledger', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-camera-files', type=int, default=0,
                        help='0 measures the full snapshot; positive values are a wiring pilot only')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    if args.max_camera_files < 0:
        parser.error('max-camera-files must be nonnegative')
    if args.output.exists() and any(args.output.iterdir()):
        parser.error('Use a new output directory to preserve previous evidence')
    if args.ledger.parent.resolve() in [args.output.resolve(), *args.output.resolve().parents]:
        parser.error('Analysis output must be outside the capture campaign')
    payload = args.ledger.read_bytes()
    snapshot_at = datetime.now(timezone(timedelta(hours=9))).isoformat()
    # A concurrently appending ledger may end in an unfinished line; do not interpret it.
    lines = payload.splitlines()
    tail_ignored = bool(payload and not payload.endswith(b'\n'))
    if tail_ignored:
        lines = lines[:-1]
    latest = {}
    for line in lines:
        if line.strip():
            entry = json.loads(line)
            latest[entry.get('source_cell_id', entry['id'])] = entry
    entries = list(latest.values())
    if any(not isinstance(e['audit'], dict) for e in entries if e['status'] == 'accepted_candidate'):
        raise ValueError('This analysis expects embedded immutable capture audits')
    rows, conflicts = observations(entries)
    metrics, sweeps, geometries, camera_files, elapsed = measure(entries, rows, args.max_camera_files)
    accepted = {e['id']: e for e in entries}
    summaries, subgroups = {}, []
    for camera in CAMERAS:
        indices = [i for i in metrics if rows[i]['camera'] == camera and rows[i]['native_pixels'] >= 16]
        if not indices:
            continue
        summaries[camera] = {p['id']: summarize(indices, metrics, p['id']) for p in PROFILES}
        for cls in CLASS_NAMES:
            for lo, hi in DISTANCES:
                subset = [i for i in indices if rows[i]['training_class'] == cls and lo <= rows[i]['range_m'] < hi]
                if subset:
                    subgroups.append(dict(camera=camera, training_class=cls, distance_m=[lo, None if math.isinf(hi) else hi],
                        profiles={p['id']: summarize(subset, metrics, p['id']) for p in PROFILES}))
    unions, ambiguous, components = component_unions(entries, rows, metrics)
    recommendation_examples = []
    for i in metrics:
        r, m = rows[i], metrics[i]
        if r['native_pixels'] >= 64 and r['camera'] in SIDE_CAMERAS:
            if m['shift320']['native_fraction'] + .1 < m['bottom320']['native_fraction']:
                recommendation_examples.append(dict(**r, family=accepted[r['scene']]['family'],
                    candidates={k: m[k] for k in ('bottom320', 'shift320', 'shift384', 'bottom448')}))
    recommendation_examples.sort(key=lambda r: r['native_pixels'], reverse=True)
    report = dict(created_at_kst=datetime.now(timezone(timedelta(hours=9))).isoformat(),
        snapshot_at_kst=snapshot_at,
        source_ledger=str(args.ledger.resolve()), source_sha256=hashlib.sha256(payload).hexdigest(),
        source_bytes=len(payload), incomplete_tail_ignored=tail_ignored,
        accepted_scenes=sum(e['status'] == 'accepted_candidate' for e in entries),
        camera_observations=len(rows), measured_observations=len(metrics),
        camera_files_read=camera_files, cpu_measurement_elapsed_sec=elapsed,
        full_snapshot=args.max_camera_files == 0,
        observed_assets=len({rows[i]['asset'] for i in metrics}),
        profiles=[dict(**p, pixel_budget_ratio=p['h'] * p['w'] / (320 * 896)) for p in PROFILES],
        candidate_roi=ANALYSIS_ROI,
        geometry=geometries, per_camera=summaries, by_camera_class_distance=subgroups,
        component_unions=unions, ambiguous_components_excluded_from_unions=ambiguous,
        semantic_conflicts=conflicts, examples_of_bottom_loss_after_shift=recommendation_examples[:12],
        limitations=[
            'Geometry and foreground retention, not trained AP or online performance.',
            'Metrics refer to already visible native foreground; original sensor clipping cannot be recovered.',
            'Native foreground uses pixel centers in the inverse ideal-scale window; integer resize rounding is explicit.',
            'Nearest-neighbor model masks are support proxies, not RGB cubic interpolation or a changed GT policy.',
            'Per-camera aggregation uses native >=16 pixels; unions include all originally camera-visible unambiguous components.',
            'Same-parent same-class ambiguous 3D components are excluded from union accounting, not from collection or labels.',
            'Center-distance bins are not BBox boundary clearances.',
            'Sunny13 snapshot only; not all 5K, all environments, or all future road pitches.',
            'Shared output shape is required; per-camera shifted crop profiles are not implemented runtime settings.'])
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'source_ledger_snapshot.jsonl').write_bytes(payload)
    (args.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    details = [dict(**rows[i], family=accepted[rows[i]['scene']]['family'],
                    role=accepted[rows[i]['scene']]['role'], candidates=metrics[i]) for i in metrics]
    (args.output / 'observations.json').write_text(json.dumps(details, ensure_ascii=False, allow_nan=False) + '\n')
    (args.output / 'component_unions.json').write_text(json.dumps(components, ensure_ascii=False, allow_nan=False) + '\n')
    if len(summaries) == 4:
        draw(rows, metrics, sweeps, summaries, args.output)
    draw_examples(details, args.output)
    print(json.dumps(dict(output=str(args.output.resolve()), accepted_scenes=report['accepted_scenes'],
        camera_observations=len(rows), measured_observations=len(metrics),
        elapsed_sec=round(elapsed, 2), full_snapshot=report['full_snapshot']), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
