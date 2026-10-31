#!/usr/bin/env python3
"""Offline, read-only measurement of actual lamp observations before stop lines.

No ROS, native RPC or driving. Missing detections are NOT physical FOV exits.
Use image source stamps, recorded front-axle proxies, and bracketed native GET.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/evaluation/traffic_light_robustness_261025'
RUNS = ('production_full_natural_sunny13_261026_v1',
        'production_full_natural_foggy13_261026_v2')


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def positive_observation(decision):
    # A held permission is not a new camera observation. Conversely, a fresh
    # raw lamp remains evidence even if a later policy rejects that lamp.
    return (decision.get('selection_status') == 'matched'
            and decision.get('classifier_known') is True
            and bool(decision.get('observed_tuples')))


def interpolate_pose(poses, stamps, stamp):
    i = bisect_left(stamps, stamp)
    if i < len(stamps) and stamps[i] == stamp:
        return np.array(poses[i]['front_point_map_xyz_m'][:2]), 0.
    if not 0 < i < len(stamps):
        raise ValueError('Pose extrapolation forbidden')
    gap = (stamps[i]-stamps[i-1])/1e9
    if gap > .15:
        raise ValueError('Pose interpolation gap exceeds 150ms')
    f = (stamp-stamps[i-1])/(stamps[i]-stamps[i-1])
    a, b = [np.array(poses[j]['front_point_map_xyz_m'][:2]) for j in (i-1, i)]
    return a+(b-a)*f, gap


def load_run(name, groups):
    folder = BASE / name / 'observer'
    poses, frames, selections, native = [], [], {}, []
    for line in (folder/'observations.jsonl').open():
        row = json.loads(line)
        if row.get('event') == 'ego_front_GT_proxy' and row.get('pose_time_verified'):
            poses.append(row)
        elif row.get('event') == 'native_read_only_sample':
            native.append(row)
        elif row.get('topic') in ('/traffic_light_node/debug_runtime', '/traffic_light_node/debug_selection'):
            data = row['message'].get('parsed_json') or json.loads(row['message']['data'])
            if data.get('source_stamp_ns'):
                if row['topic'].endswith('debug_runtime'):
                    data['_observer_receipt_monotonic'] = row['receipt_monotonic']
                    frames.append(data)
                else:
                    selections[data['source_stamp_ns']] = data
    poses.sort(key=lambda r: r['source_stamp_ns'])
    stamps = [p['source_stamp_ns'] for p in poses]
    crossings = json.loads((folder/'crossings.json').read_text())['crossings']
    results = []
    for cross in crossings:
        group = groups[cross['group_id']]
        t = cross['estimated_source_stamp_ns']
        history = [f for f in frames if (f.get('frame_decision') or {}).get('group_id') == cross['group_id']
                   and t-60e9 < f['source_stamp_ns'] <= t]
        eligible = [f for f in history if positive_observation(f['frame_decision'])]
        last = max(eligible, key=lambda f: f['source_stamp_ns']) if eligible else None
        if last is None:
            raise ValueError('No last observation: '+group['mission'])
        lt = last['source_stamp_ns']
        delivered = [f for f in eligible if f['_observer_receipt_monotonic'] <= cross['estimated_source_monotonic']]
        last_delivered = max(delivered, key=lambda f: f['source_stamp_ns']) if delivered else None
        xy, interp_gap = interpolate_pose(poses, stamps, lt)
        path = [xy.tolist()] + [p['front_point_map_xyz_m'][:2] for p in poses if lt < p['source_stamp_ns'] < t]
        path.append(cross['point_xy_m'])
        distance = sum(float(np.linalg.norm(np.subtract(b, a))) for a, b in zip(path, path[1:]))
        after = [f for f in history if f['source_stamp_ns'] > lt]
        reasons = Counter(selections.get(f['source_stamp_ns'], {}).get('reason', 'unrecorded') for f in after)
        sample_rows = [r for r in native if r['native_id'] in group['native_ids']
                       and lt <= r['request_ros_ns'] and r['response_ros_ns'] <= t]
        actual_colors = Counter(str(r.get('color')) if r.get('valid') else 'unknown' for r in sample_rows)
        yellow = [f for f in history if positive_observation(f['frame_decision'])
                  and any(v[0] == 'yellow' for v in f['frame_decision']['observed_tuples'])]
        # This counts observations, not independent vehicle encounters.
        yellow_locations = []
        for f in yellow:
            yxy, _ = interpolate_pose(poses, stamps, f['source_stamp_ns'])
            yellow_locations.append(dict(source_stamp_ns=f['source_stamp_ns'], xy=yxy.tolist(),
                                         seconds_before_crossing=(t-f['source_stamp_ns'])/1e9))
        cxy = np.array(cross['point_xy_m'])
        approach = [p['front_point_map_xyz_m'][:2] for p in poses
                    if t-12e9 < p['source_stamp_ns'] <= t and np.linalg.norm(np.subtract(p['front_point_map_xyz_m'][:2], cxy)) < 35]
        result = dict(run=name, mission=group['mission'], group_id=cross['group_id'],
                      last_observation_source_stamp_ns=lt, crossing_source_stamp_ns=t,
                      last_observation_xy_m=xy.tolist(), crossing_xy_m=cxy.tolist(),
                      last_observed_tuples=last['frame_decision']['observed_tuples'],
                      latest_observation_report_received_before_crossing=last_delivered['source_stamp_ns'] if last_delivered else None,
                      latest_report_source_age_at_crossing_sec=(t-last_delivered['source_stamp_ns'])/1e9 if last_delivered else None,
                      no_new_positive_observation_sec=(t-lt)/1e9,
                      traveled_to_crossing_m=distance, pose_interpolation_bracket_sec=interp_gap,
                      missing_or_unknown_frame_count=len(after), selection_reasons_after_last=dict(reasons),
                      native_colors_inside_gap=dict(actual_colors),
                      crossing_native=cross['native_bracket_evaluation']['0.25'],
                      raw_yellow_observations=yellow_locations,
                      last_selection=selections.get(lt), gap_path_xy_m=path,
                      approach_xy_m=approach, jpeg=cross['crossing_jpeg_bracket']['before']['path'])
        results.append(result)
    return results, poses, dict(observations_sha256=sha(folder/'observations.jsonl'),
                               crossings_sha256=sha(folder/'crossings.json'))


def map_roads(groups):
    from pyproj import Transformer
    import yaml
    map_path = ROOT/'kcity_map/lanelet_map/lanelet2_map_v87.osm'
    origin = yaml.safe_load((ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml').read_text())['map_origin']
    project = Transformer.from_crs('EPSG:4326', 'EPSG:32652', always_xy=True)
    ox, oy = project.transform(origin['longitude'], origin['latitude'])
    tree = ET.parse(map_path).getroot()
    nodes = {n.get('id'): np.subtract(project.transform(float(n.get('lon')), float(n.get('lat'))), (ox, oy))
             for n in tree.findall('node')}
    ways = {int(w.get('id')): [nodes[n.get('ref')] for n in w.findall('nd')] for w in tree.findall('way')}
    # Independently verify the plotted map and frozen observer geometry align.
    for g in groups.values():
        for line in g['stop_lines']:
            if not np.allclose(ways[line['way_id']], line['points_xy_m'], atol=.001, rtol=0):
                raise ValueError('Map/projector differs from recorded stopline')
    ids = {int(m.get('ref')) for r in tree.findall('relation')
           if any(t.get('k') == 'type' and t.get('v') == 'lanelet' for t in r.findall('tag'))
           for m in r.findall('member') if m.get('type') == 'way' and m.get('role') in ('left', 'right')}
    return [np.array(ways[i]) for i in ids], sha(map_path)


def plots(rows, poses_by_run, groups, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib import font_manager
    fonts = [f for f in font_manager.fontManager.ttflist if f.name == 'Noto Sans CJK JP']
    if fonts:
        plt.rcParams['font.family'] = fonts[0].name
    plt.rcParams['axes.unicode_minus'] = False
    roads, map_sha = map_roads(groups)
    geometry = json.loads((ROOT/'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json').read_text())
    # Coordinates use recorded map<-morai transform, never a guessed offset.
    from observe_traffic_light_compliance import rotate
    tf = poses_by_run[RUNS[0]][0]['TF']
    head_xy = {h['id']: np.add(rotate(h['point'], tf['quaternion_xyzw']), tf['translation'])[:2]
               for h in geometry['heads']}
    colors = ['#087ac1', '#e57716']
    labels = ['Sunny13', 'Foggy13']
    fig, axes = plt.subplots(2, 2, figsize=(15, 13))
    for ax, g in zip(axes.flat, groups.values()):
        ax.add_collection(LineCollection(roads, colors='#c4cbd1', linewidths=1, zorder=0))
        stop = np.array(g['stop_line_xy'])
        center = stop.mean(axis=0)
        ax.plot(*stop.T, color='crimson', lw=5, label='정지선', zorder=5)
        for hi, identifier in enumerate(g['native_ids']):
            hp = head_xy[identifier]
            ax.scatter(*hp, color='#7f32a8', marker='s', s=60, zorder=6,
                       label='해당 신호등 위치' if hi == 0 else None)
            ax.annotate('신호등'+(' (두 번째)' if hi else ''), hp, xytext=(5, 5),
                        textcoords='offset points', fontsize=9, color='#7f32a8')
        texts = []
        for index, name in enumerate(RUNS):
            r = next(r for r in rows if r['run'] == name and r['mission'] == g['mission'])
            path = np.array(r['approach_xy_m'])
            ax.plot(*path.T, color=colors[index], lw=1.7, alpha=.65)
            gap = np.array(r['gap_path_xy_m'])
            ax.plot(*gap.T, color=colors[index], lw=5, alpha=.8, label=labels[index]+' 마지막 인식→선')
            ax.scatter(*r['last_observation_xy_m'], color=colors[index], s=85,
                       marker='o' if index == 0 else '^', edgecolors='white', zorder=7)
            ax.scatter(*r['crossing_xy_m'], color='black', marker='x', s=65, zorder=8)
            texts.append(f"{labels[index]}: {r['traveled_to_crossing_m']:.2f}m / {r['no_new_positive_observation_sec']:.2f}초")
        ax.set_title(g['mission']+(' · 좌회전' if g['mission'] == 'D' else ' · 직진'), fontsize=18)
        ax.text(.03, .97, '\n'.join(texts)+'\n두 통과 모두 실제 허가 신호', transform=ax.transAxes,
                va='top', fontsize=11, bbox=dict(facecolor='white', alpha=.9, edgecolor='none'))
        ax.set_xlim(center[0]-15, center[0]+15)
        ax.set_ylim(center[1]-21, max(center[1]+14,
                    max(head_xy[i][1] for i in g['native_ids'])+4))
        ax.set_aspect('equal'); ax.grid(alpha=.18)
        ax.set_xlabel('지도 X (m)'); ax.set_ylabel('지도 Y (m)')
        ax.legend(loc='lower left', fontsize=9)
    fig.suptitle('A/B/C/D · 마지막 실제 색 인식부터 앞차축 정지선 통과까지', fontsize=20)
    fig.text(.5, .025, '점/삼각형 = 마지막 새 영상의 색 인식 · 검정 × = 앞차축 통과\n'
             '굵은 선은 관측 공백이며, 고정된 사각지대나 황색→적색 전환 위치가 아님',
             ha='center', fontsize=12)
    fig.tight_layout(rect=(0, .065, 1, .95))
    fig.savefig(out/'abcd_observation_gap_map.png', dpi=160)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 10))
    ax.add_collection(LineCollection(roads, colors='#c4cbd1', linewidths=.7))
    path = np.array([p['front_point_map_xyz_m'][:2] for p in poses_by_run[RUNS[0]]])
    ax.plot(*path.T, color='#087ac1', lw=1, alpha=.6)
    for g in groups.values():
        stop = np.array(g['stop_line_xy']);center=stop.mean(axis=0)
        ax.plot(*stop.T, color='crimson', lw=3)
        ax.annotate(g['mission'], center, xytext=(12, 5), textcoords='offset points', fontsize=20, color='crimson')
    ax.set_xlim(-15, 175);ax.set_ylim(255, 680);ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.set_title('시험 경로와 네 미션 정지선 · v87 지도', fontsize=16)
    ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)')
    fig.tight_layout();fig.savefig(out/'abcd_overview.png', dpi=130);plt.close(fig)
    # Original images, not synthetic renders, preserve visible field evidence.
    fig, axes = plt.subplots(2, 4, figsize=(20, 7))
    for ax, r in zip(axes.flat, rows):
        ax.imshow(plt.imread(r['jpeg']));ax.axis('off')
        ax.set_title(('Sunny13' if 'sunny' in r['run'] else 'Foggy13')+' / '+r['mission'])
    fig.suptitle('앞차축 통과 직전 원본 카메라 · 네 미션 신호가 아직 화면 안에 있음', fontsize=17)
    fig.tight_layout(rect=(0, 0, 1, .94));fig.savefig(out/'crossing_original_contact_sheet.png', dpi=150);plt.close(fig)
    return map_sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    groups_path = BASE/'production_trials_abcd_261026_v4/groups.json'
    groups = {g['group_id']: g for g in json.loads(groups_path.read_text())['groups']}
    rows, poses, inputs = [], {}, {}
    for name in RUNS:
        result, poses[name], inputs[name] = load_run(name, groups)
        rows.extend(result)
    map_sha = plots(rows, poses, groups, args.output)
    document = dict(scope='Two historical natural full runs, not current OFF-policy certification',
                    physical_FOV_exit_measured=False,
                    visual_review='Original crossing-before JPEGs show mission lamps in image for all eight crossings; D is near top edge. No universal FOV boundary inferred.',
                    caveat='Last classified lamp is not last physical visibility. Native timing is sampled, not official judge.',
                    front_axle_offset_m=3., groups_sha256=sha(groups_path), map_sha256=map_sha,
                    inputs=inputs, results=rows)
    (args.output/'measurements.json').write_text(json.dumps(document, ensure_ascii=False, indent=2)+'\n')
    for r in rows:
        print(r['mission'], 'sunny' if 'sunny' in r['run'] else 'foggy',
              round(r['traveled_to_crossing_m'], 3), round(r['no_new_positive_observation_sec'], 3),
              r['last_observed_tuples'], r['selection_reasons_after_last'],
              r['native_colors_inside_gap'], 'yellow_frames', len(r['raw_yellow_observations']))


if __name__ == '__main__':
    main()
