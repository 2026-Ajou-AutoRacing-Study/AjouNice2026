#!/usr/bin/env python3
"""Read saved camera evidence only; never change collection or training labels."""
import argparse
import ast
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path

import numpy as np

CAMERAS = {
    'CAMERA_1': ('전방', 1280, 720),
    'CAMERA_2': ('좌전방', 640, 480),
    'CAMERA_3': ('우전방', 640, 480),
    'CAMERA_4': ('후방', 1280, 720),
}
COLORS = {'vehicle': '#2563eb', 'pedestrian': '#d97706',
          'obstacle': '#16a34a', 'animal': '#9333ea'}
CLASS_NAMES = {'vehicle': '차량', 'pedestrian': '사람',
               'obstacle': '장애물', 'animal': '동물'}
CANDIDATES = [(320, 896), (384, 1056), (448, 1248), (384, 896), (448, 896)]


def crop_start(camera, final_h=320, final_w=896):
    """Continuous native-image cutoff; exact resize rounding differs <=1 px."""
    _, width, height = CAMERAS[camera]
    return max(0., 1. - final_h / ((final_w / width) * height))


def height_state(top, bottom, cutoff):
    return window_state(top, bottom, cutoff, 1.)


def window_state(top, bottom, start, end):
    if bottom <= start or top >= end:
        return 'removed'
    if top < start or bottom > end:
        return 'partial'
    return 'full'


def observations(entries):
    """Deduplicate 2D observations repeated across ambiguous 3D components."""
    seen = {}; conflicts = 0
    for entry in entries:
        if entry['status'] != 'accepted_candidate':
            continue
        audit = entry['audit']
        if not isinstance(audit, dict):
            audit = json.loads(Path(audit).read_text())
        for component in audit.get('components', []):
            for evidence in component.get('cameras', []):
                camera = evidence['camera']
                if camera not in CAMERAS or evidence['native_instance_pixels'] <= 0:
                    continue
                rect = tuple(evidence['native_rect'])
                key = (entry['id'], camera, evidence['camera_instance_id'],
                       evidence['annotation_class'], rect)
                _, width, height = CAMERAS[camera]
                top = float(np.clip(rect[1] / height, 0., 1.))
                bottom = float(np.clip(rect[3] / height, 0., 1.))
                if bottom <= top or rect[2] <= rect[0]:
                    continue
                value = dict(scene=entry['id'], capture_root=entry['capture_root'],
                    camera=camera, asset=component['asset'],
                    training_class=component['training_class'],
                    camera_instance_id=evidence['camera_instance_id'],
                    raw_class=evidence['annotation_class'], native_rect=rect,
                    top=top, bottom=bottom, center=(top + bottom) / 2,
                    native_pixels=evidence['native_instance_pixels'],
                    kept_native_pixels=evidence['native_bottom_crop_instance_pixels'],
                    model_pixels=evidence['crop_320x896_instance_pixels'],
                    range_m=component.get('range_ego_xy_m', component['range_xy_m']),
                    lidar_points=component['lidar_points'],
                    component_ambiguous=component.get('camera_component_ambiguous', False))
                if key in seen:
                    previous = seen[key]
                    if previous['training_class'] != value['training_class']:
                        conflicts += 1
                    previous['component_ambiguous'] |= value['component_ambiguous']
                    # Same-class components cannot supply one reliable per-camera LiDAR count.
                    previous['lidar_points'] = None
                else:
                    seen[key] = value
    return list(seen.values()), conflicts


def rate(rows, cutoff, end=1.):
    counts = Counter(window_state(r['top'], r['bottom'], cutoff, end) for r in rows)
    n = len(rows)
    return dict(n=n, **{k: counts[k] for k in ('full', 'partial', 'removed')},
                percentages={k: 100 * counts[k] / n if n else None
                             for k in ('full', 'partial', 'removed')})


def pixel_rate(rows):
    counts = Counter()
    fractions = []
    for row in rows:
        native, kept = row['native_pixels'], row['kept_native_pixels']
        if not 0 <= kept <= native:
            raise ValueError('Invalid native instance pixel retention')
        counts['removed' if kept == 0 else 'partial' if kept < native else 'full'] += 1
        fractions.append(kept / native)
    return dict(n=len(rows), **{k: counts[k] for k in ('full', 'partial', 'removed')},
                percentages={k: 100 * counts[k] / len(rows) if rows else None
                             for k in ('full', 'partial', 'removed')},
                mean_instance_pixel_retention=float(np.mean(fractions)) if fractions else None)


def summarize(rows):
    result = {}
    for camera in CAMERAS:
        selected = [r for r in rows if r['camera'] == camera]
        strong = [r for r in selected if r['native_pixels'] >= 16]
        exact = pixel_rate(selected)
        result[camera] = dict(name=CAMERAS[camera][0], observations=len(selected),
            current_cutoff_pct=100 * crop_start(camera),
            current_bbox=rate(selected, crop_start(camera)),
            current_exact_native_pixel_counts={k: exact[k] for k in ('full', 'partial', 'removed')},
            current_exact_native_pixels=exact,
            current_exact_native_pixels_at_least_16=pixel_rate(strong),
            mean_instance_pixel_retention=exact['mean_instance_pixel_retention'],
            substantial_native_pixel_observations=len(strong),
            candidates=[dict(image_hw=[h, w], cutoff_pct=100 * crop_start(camera, h, w),
                all_bbox=rate(selected, crop_start(camera, h, w)),
                native_pixels_at_least_16_bbox=rate(strong, crop_start(camera, h, w)))
                for h, w in CANDIDATES])
        if camera in ('CAMERA_2', 'CAMERA_3'):
            start, end = .35, .35 + 320 / 672
            result[camera]['shifted_320x896_candidate'] = dict(
                window_pct=[100 * start, 100 * end],
                substantial_bbox=rate(strong, start, end),
                distance_bins=[dict(distance_m=[lo, hi], bbox=rate(
                    [r for r in strong if lo <= r['range_m'] < hi], start, end))
                    for lo, hi in [(0, 10), (10, 30), (30, 60), (60, 101)]])
    return result


def configure_plot():
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt, font_manager
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        prop = font_manager.FontProperties(fname=str(font))
        plt.rcParams['font.family'] = prop.get_name()
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['font.size'] = 10
    return plt


def draw(rows, output):
    plt = configure_plot()
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), sharey=True)
    heights = np.linspace(0, 1, 201)
    for ax, camera in zip(axes.flat, CAMERAS):
        selected = [r for r in rows if r['camera'] == camera and r['native_pixels'] >= 16]
        for cls, color in COLORS.items():
            group = [r for r in selected if r['training_class'] == cls]
            if not group:
                continue
            occupancy = np.mean([[r['top'] <= h <= r['bottom'] for h in heights]
                                 for r in group], axis=0) * 100
            ax.plot(occupancy, heights * 100, color=color,
                    label=f'{CLASS_NAMES[cls]} 관측 {len(group)}개')
        ax.axhspan(0, crop_start(camera) * 100, color='#fee2e2', alpha=.6)
        ax.axhline(crop_start(camera) * 100, color='#dc2626', linewidth=2,
                   label=f'현재 320×896 시작선 {100 * crop_start(camera):.1f}%')
        ax.axhline(crop_start(camera, 448, 896) * 100, color='#0891b2', linestyle='--',
                   label=f'세로 확대 448×896 시작선 {100 * crop_start(camera,448,896):.1f}%')
        ax.set_title(f'{CAMERAS[camera][0]}  |  원본 픽셀 ≥16인 관측 {len(selected)}개')
        ax.set_ylim(100, 0)
        ax.set_xlabel('해당 높이에 2D 박스가 걸치는 관측 비율 (%)')
        ax.set_ylabel('원본 이미지 높이 (%)  위 0 → 아래 100')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc='upper right')
    fig.suptitle('카메라별 객체 높이 분포와 crop 시작선', fontsize=17)
    fig.text(.5, .02, '빨간 영역은 현재 잘리는 위쪽이다. 곡선은 2D 박스 분포이며 실제 픽셀 밀도가 아니다.',
             ha='center', fontsize=10)
    fig.tight_layout(rect=[0, .05, 1, .95])
    fig.savefig(output / 'camera_vertical_distribution.png', dpi=145)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    cutoffs = np.linspace(0, .7, 141)
    for ax, camera in zip(axes.flat, CAMERAS):
        selected = [r for r in rows if r['camera'] == camera and r['native_pixels'] >= 16]
        rates = [rate(selected, cut)['percentages'] for cut in cutoffs]
        for key, name, color in [('full', '박스 높이 전부 유지', '#16a34a'),
                                 ('partial', '박스 높이 일부 잘림', '#d97706'),
                                 ('removed', '박스 높이 전부 잘림', '#dc2626')]:
            ax.plot(cutoffs * 100, [r[key] for r in rates], label=name, color=color, lw=2)
        ax.axvline(crop_start(camera) * 100, color='#374151', linestyle='--', label='현재 320×896')
        ax.axvline(crop_start(camera, 448, 896) * 100, color='#0891b2', linestyle=':', label='세로 확대 448×896')
        ax.set_title(f'{CAMERAS[camera][0]}  |  관측 {len(selected)}개')
        ax.set_xlabel('위쪽에서 잘라내는 원본 높이 (%)')
        ax.set_ylabel('관측 비율 (%)')
        ax.set_ylim(0, 101)
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('어디서 자를 때 객체의 일부 또는 전부가 잘리는가', fontsize=17)
    fig.text(.5, .02, '원본 객체 픽셀 ≥16인 관측만 표시. 대체 시작선은 2D 박스 기반 추정이며 실제 픽셀 검증 전이다.',
             ha='center', fontsize=10)
    fig.tight_layout(rect=[0, .05, 1, .95])
    fig.savefig(output / 'crop_boundary_tradeoff.png', dpi=145)
    plt.close(fig)

    asset_counts = Counter(r['asset'] for r in rows)
    assets = [asset for asset, _ in asset_counts.most_common()]
    groups = defaultdict(list)
    for r in rows:
        groups[(r['asset'], r['camera'])].append(r)
    for page_start in range(0, len(assets), 24):
        page = assets[page_start:page_start + 24]
        fig, axes = plt.subplots(1, 4, figsize=(18, max(6, len(page) * .36)), sharey=True)
        for ax, camera in zip(axes, CAMERAS):
            for index, asset in enumerate(page):
                group = groups.get((asset, camera), [])
                if not group:
                    continue
                left = np.percentile([r['top'] for r in group], 10) * 100
                right = np.percentile([r['bottom'] for r in group], 90) * 100
                center = np.median([r['center'] for r in group]) * 100
                color = COLORS[group[0]['training_class']]
                ax.plot([left, right], [index, index], color=color, lw=3, alpha=.6)
                ax.scatter([center], [index], color=color, s=18)
                ax.text(100.5, index, str(len(group)), fontsize=7, va='center')
            ax.axvspan(0, crop_start(camera) * 100, color='#fee2e2', alpha=.55)
            ax.axvline(crop_start(camera) * 100, color='#dc2626', lw=1.5)
            ax.set_title(CAMERAS[camera][0])
            ax.set_xlim(0, 110)
            ax.set_ylim(len(page) - .4, -.7)
            ax.set_xticks([0, 25, 50, 75, 100])
            ax.set_xlabel('원본 높이 (%)  위 0 → 아래 100')
            ax.set_yticks(range(len(page)))
            ax.set_yticklabels(page, fontsize=8)
            ax.grid(axis='x', alpha=.2)
        fig.suptitle(f'자산별 높이 위치  {page_start + 1}~{page_start + len(page)} / {len(assets)}종', fontsize=15)
        fig.text(.5, .015, '선: 여러 관측의 상단 p10~하단 p90  ·  점: 중심 중앙값  ·  오른쪽 숫자: 관측 수\n'
                 '한 객체의 실제 높이가 아니라 관측 위치의 분포다. 빨간 선 왼쪽은 현재 잘리는 영역이다.', ha='center')
        fig.tight_layout(rect=[0, .07, 1, .94])
        fig.savefig(output / f'asset_height_page_{page_start // 24 + 1:02d}.png', dpi=130)
        plt.close(fig)


def draw_examples(rows, output):
    """Read four existing captures only, to check the bounding-box interpretation."""
    from PIL import Image
    from matplotlib.patches import Rectangle
    plt = configure_plot()
    def damaged(r):
        return r['native_pixels'] >= 100 and 0 < r['kept_native_pixels'] < .6 * r['native_pixels']
    selectors = [
        lambda r: r['camera'] == 'CAMERA_1' and r['training_class'] == 'vehicle',
        lambda r: r['camera'] == 'CAMERA_3' and r['training_class'] == 'vehicle',
        lambda r: r['camera'] == 'CAMERA_2' and r['training_class'] == 'pedestrian'
            and r['asset'].startswith(('Man', 'Woman', 'worker', 'police')),
        lambda r: r['camera'] in ('CAMERA_2', 'CAMERA_3') and r['training_class'] == 'obstacle',
    ]
    selected = []
    for selector in selectors:
        choices = [r for r in rows if damaged(r) and selector(r)]
        if choices:
            selected.append(max(choices, key=lambda r: r['native_pixels']))
    if not selected:
        return []
    fig, axes = plt.subplots(len(selected), 2, figsize=(15, 4 * len(selected)), squeeze=False)
    results = []
    for row, axes_row in zip(selected, axes):
        folder = Path(row['capture_root']) / row['camera']
        native_path = next(folder.glob('*_Intensity.png'))
        with Image.open(native_path) as image:
            rgb = np.asarray(image.convert('RGB'))
        with Image.open(next(folder.glob('*_Instance.png'))) as image:
            instance = np.asarray(image.convert('RGBA'))
        colour = None
        for line in next(folder.glob('*_BBox2D.txt')).read_text().splitlines():
            fields = line.split(maxsplit=6)
            if len(fields) == 7 and int(fields[5]) == row['camera_instance_id'] and fields[0] == row['raw_class']:
                if np.allclose([float(x) for x in fields[1:5]], row['native_rect']):
                    colour = ast.literal_eval(fields[6])
                    break
        if colour is None:
            raise ValueError('Cannot locate example instance colour')
        mask = (instance == np.asarray(colour)).all(axis=2)
        height, width = mask.shape
        cut = int(height - 320 / (896 / width))
        start, end = ((.35, .35 + 320 / 672) if row['camera'] in ('CAMERA_2', 'CAMERA_3')
                      else (crop_start(row['camera'], 448, 896), 1.))
        lo, hi = int(start * height), min(height, int(end * height))
        total = int(mask.sum())
        if total != row['native_pixels']:
            raise ValueError('Saved evidence and example mask disagree')
        current_kept = int(mask[cut:].sum())
        alternative_kept = int(mask[lo:hi].sum())
        record = dict(asset=row['asset'], camera=row['camera'], scene=row['scene'],
                      original=str(native_path), native_pixels=total,
                      current_kept_pixels=current_kept, alternative_kept_pixels=alternative_kept,
                      alternative_window_native_rows=[lo, hi], range_m=row['range_m'])
        results.append(record)
        for ax in axes_row:
            ax.imshow(rgb)
            ax.axis('off')
            x1, y1, x2, y2 = row['native_rect']
            ax.add_patch(Rectangle((x1, y1), x2 - x1, y2 - y1, fill=False, color='#fde047', lw=1.5))
        current, alternative = axes_row
        current.axhspan(0, cut, color='red', alpha=.16)
        current.axhline(cut, color='#ff3030', lw=2)
        current.set_title(f"{row['asset']} / {CAMERAS[row['camera']][0]} / {row['range_m']:.1f}m\n"
                          f'현재 crop: 객체 픽셀 {100 * current_kept / total:.1f}% 유지')
        if lo:
            alternative.axhspan(0, lo, color='red', alpha=.16)
        if hi < height:
            alternative.axhspan(hi, height, color='red', alpha=.16)
        alternative.axhline(lo, color='#00e5ff', lw=2)
        if hi < height:
            alternative.axhline(hi, color='#00e5ff', lw=2)
        label = '같은 320×896, crop 위로 이동' if hi < height else '448×896, 세로 범위 확대'
        alternative.set_title(f'{label}\n객체 픽셀 {100 * alternative_kept / total:.1f}% 유지')
    fig.suptitle('일부 잘림의 실제 사례  |  붉은 영역은 제외, 노란 사각형은 해당 객체 annotation', fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, .97])
    fig.savefig(output / 'partial_crop_examples.png', dpi=120)
    plt.close(fig)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ledger', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--examples', action='store_true', help='Read at most four raw camera examples')
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise ValueError('Use a new output; never overwrite an earlier analysis snapshot')
    payload = args.ledger.read_bytes()
    latest = {}
    for line in payload.splitlines():
        if line.strip():
            entry = json.loads(line)
            latest[entry.get('source_cell_id', entry['id'])] = entry
    entries = list(latest.values())
    rows, conflicts = observations(entries)
    args.output.mkdir(parents=True, exist_ok=True)
    by_asset = defaultdict(list)
    for row in rows:
        by_asset[(row['asset'], row['camera'])].append(row)
    asset_stats = []
    for (asset, camera), group in sorted(by_asset.items()):
        asset_stats.append(dict(asset=asset, camera=camera, n=len(group),
            top_p10_pct=float(np.percentile([r['top'] for r in group], 10) * 100),
            center_p50_pct=float(np.median([r['center'] for r in group]) * 100),
            bottom_p90_pct=float(np.percentile([r['bottom'] for r in group], 90) * 100),
            current_bbox=rate(group, crop_start(camera))))
    report = dict(created_at_kst=datetime.now(timezone(timedelta(hours=9))).isoformat(),
        source_ledger=str(args.ledger.resolve()), source_sha256=hashlib.sha256(payload).hexdigest(),
        source_bytes=len(payload), cells=len(entries),
        accepted_scenes=sum(r['status'] == 'accepted_candidate' for r in entries),
        camera_observations=len(rows), observed_assets=len({r['asset'] for r in rows}),
        semantic_conflicts=conflicts, cameras=summarize(rows), asset_statistics=asset_stats,
        examples=sorted([r for r in rows if r['native_pixels'] >= 64 and
            0 < r['kept_native_pixels'] < .6 * r['native_pixels']],
            key=lambda r: r['native_pixels'], reverse=True)[:16],
        limitations=[
            'Snapshot of accepted captures; not full 5K or trained detector performance.',
            'Per-camera observations, not distinct 3D GT count; deduplicated shared component evidence.',
            '2D BBox height envelopes are not foreground pixel density or original-sensor clipping tests.',
            'Current native pixel retention uses saved instance evidence; alternative cutoffs use BBoxes only.',
            'Crop formula is continuous; actual resize/crop rounding can differ by about one pixel.',
            'Native >=16 pixels is an analysis cohort, not a changed training or collection threshold.',
            'Already unseen or missing camera annotations are absent from this camera-visible cohort.',
            'Low-count asset quantiles and ambiguous 3D component identities need caution.',
            'Collector, native annotations, labels, and production configs are never modified.'])
    if args.examples:
        report['verified_image_examples'] = draw_examples(rows, args.output)
    (args.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    with (args.output / 'camera_observations.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['scene'])
        writer.writeheader()
        writer.writerows(rows)
    draw(rows, args.output)
    print(json.dumps({k: report[k] for k in ('created_at_kst', 'accepted_scenes', 'camera_observations', 'observed_assets')}, ensure_ascii=False))
    for camera, stats in report['cameras'].items():
        print(camera, json.dumps({k: stats[k] for k in ('observations', 'current_cutoff_pct', 'current_bbox', 'current_exact_native_pixel_counts')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
