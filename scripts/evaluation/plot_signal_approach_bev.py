#!/usr/bin/env python3
"""Recorded BEV approach and illustrative horizontal camera cones, offline only.

Map, native GET, camera streams and live ROS are never mutated. Cone membership
is horizontal geometry only, not occlusion, full housing, or detector eligibility.
First known positive raw classification is NOT the first physical visibility.
"""
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path

import numpy as np

from analyze_signal_last_observation_gap import (
    BASE, ROOT, RUNS, interpolate_pose, load_run, map_roads, positive_observation, sha)
from observe_traffic_light_compliance import rotate


def camera_xy(front_xy, heading_degrees, camera_forward, camera_left):
    yaw = math.radians(heading_degrees)
    return np.array(front_xy)+np.array([
        (camera_forward-3.)*math.cos(yaw)-camera_left*math.sin(yaw),
        (camera_forward-3.)*math.sin(yaw)+camera_left*math.cos(yaw)])


def horizontal_angle(camera, heading, head):
    bearing = math.degrees(math.atan2(head[1]-camera[1], head[0]-camera[0]))
    return (bearing-heading+180.) % 360.-180.


def vehicle_cone(ax, xy, yaw, color, number, cone_radius, fov=90.):
    from matplotlib.patches import Polygon, Wedge
    xy = np.array(xy); theta = math.radians(yaw)
    forward = np.array([math.cos(theta), math.sin(theta)])
    left = np.array([-forward[1], forward[0]])
    # Diagram icon, deliberately not a calibrated physical footprint.
    center = xy-1.2*forward
    corners = [center+a*forward+b*left for a, b in [(2.2, 1.), (2.2, -1.), (-2.2, -1.), (-2.2, 1.)]]
    ax.add_patch(Wedge(xy, cone_radius, yaw-fov/2, yaw+fov/2, facecolor=color,
                      edgecolor=color, alpha=.09, linewidth=1, zorder=2))
    ax.add_patch(Polygon(corners, facecolor=color, edgecolor='white', zorder=6))
    ax.annotate('', xy=xy+3*forward, xytext=xy-forward,
                arrowprops=dict(arrowstyle='->', color='white', lw=1.7), zorder=7)
    ax.annotate(str(number), center, xytext=(-10, -1), textcoords='offset points',
                color='black', fontsize=15, weight='bold', zorder=8)


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args();args.output.mkdir(parents=True, exist_ok=False)
    plt.rcParams['font.family'] = 'Noto Sans CJK JP'
    plt.rcParams['axes.unicode_minus'] = False
    gp = BASE/'production_trials_abcd_261026_v4/groups.json'
    groups = {g['group_id']: g for g in json.loads(gp.read_text())['groups']}
    rows, poses, inputs = load_run(RUNS[0], groups)
    stamps = [p['source_stamp_ns'] for p in poses]
    first = {}
    for line in (BASE/RUNS[0]/'observer/observations.jsonl').open():
        r = json.loads(line)
        if r.get('topic') != '/traffic_light_node/debug_runtime':
            continue
        d = r['message'].get('parsed_json') or json.loads(r['message']['data'])
        f = d.get('frame_decision') or {}; gid = f.get('group_id')
        if gid in groups and gid not in first and positive_observation(f):
            t = d['source_stamp_ns'];xy, gap = interpolate_pose(poses, stamps, t)
            i = min(range(len(poses)), key=lambda i: abs(stamps[i]-t))
            first[gid] = dict(source_stamp_ns=t, xy=xy.tolist(),
                              heading=poses[i]['corrected_native_heading_degrees'],
                              raw=f['observed_tuples'], head=f['head_id'],
                              pose_bracket_sec=gap, heading_nearest_age_sec=abs(stamps[i]-t)/1e9)
    sensor_path = ROOT/'configs/sensors/Sensor_competition_local.json'
    sensor = json.loads(sensor_path.read_text())
    camera = next(c for c in sensor['cameraList'] if c['m_SensorUniqueID'] == 1)
    cx, cy = float(camera['pos']['x']), float(camera['pos']['y'])
    fov = float(camera['cc']['cameraFOV'])
    camera_yaw = float(camera['rot']['yaw'])
    geometry_path = ROOT/'base_ws/src/perception/traffic_light_detection/config/traffic_light_map_geometry.json'
    geometry = json.loads(geometry_path.read_text())
    tf = poses[0]['TF']
    heads = {h['id']: np.add(rotate(h['point'], tf['quaternion_xyzw']), tf['translation'])[:2]
             for h in geometry['heads']}
    roads, map_sha = map_roads(groups)
    raw_names = {201351: '적색', 201365: '직진 녹색', 201290: '적색', 201326: '직진 녹색 (좌회전 비허가)'}
    fig, axes = plt.subplots(2, 2, figsize=(15, 14))
    for ax, (gid, group) in zip(axes.flat, groups.items()):
        row = next(r for r in rows if r['group_id'] == gid)
        observed = first[gid]; stop = np.array(group['stop_line_xy']); point = np.array(row['crossing_xy_m'])
        path = np.array([p['front_point_map_xyz_m'][:2] for p in poses
                         if observed['source_stamp_ns']-3e9 < p['source_stamp_ns'] < row['crossing_source_stamp_ns']])
        ax.add_collection(LineCollection(roads, colors='#c0c8cf', linewidths=.8))
        ax.plot(*path.T, color='#487dab', lw=2, zorder=3)
        ax.plot(*stop.T, color='black', lw=5, zorder=5)
        for identifier in group['native_ids']:
            ax.scatter(*heads[identifier], marker='s', color='#8b35b4', s=70, zorder=8)
        cam = camera_xy(observed['xy'], observed['heading'], cx, cy)
        radius = float(np.linalg.norm(heads[observed['head']]-cam))+5
        vehicle_cone(ax, cam, observed['heading']+camera_yaw, '#168b62', 1, radius, fov)
        ax.plot([cam[0], heads[observed['head']][0]], [cam[1], heads[observed['head']][1]],
                color='#168b62', ls='--', lw=1.5)
        distance = float(np.linalg.norm(np.array(observed['xy'])-point))
        observed['distance_to_crossing_m'] = distance
        ax.text(.025, .975, f"첫 색 인식: 정지선 약 {distance:.0f}m 전\n실제 첫 색: {raw_names[gid]}",
                transform=ax.transAxes, va='top', fontsize=12,
                bbox=dict(facecolor='white', edgecolor='none', alpha=.94))
        ax.set_title(group['mission']+(' · 우회전 중 D 신호 획득 → 다음 교차로 좌회전' if gid == 201326 else ' · 직진 접근'), fontsize=14)
        bounds = np.vstack([path, stop]+[np.array([heads[i]]) for i in group['native_ids']])
        ax.set_xlim(bounds[:, 0].min()-10, bounds[:, 0].max()+14)
        ax.set_ylim(bounds[:, 1].min()-8, bounds[:, 1].max()+10)
        ax.set_aspect('equal');ax.grid(alpha=.18)
        ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)')
    fig.suptitle('실제 Sunny13 주행 · 경로 / 첫 색 인식 위치 / 전방 시야', fontsize=20)
    fig.text(.5, .035, '보라색 ■ = 미션 신호등 · 검정 = 정지선 · 녹색 차량 = 실제 첫 색 인식 위치\n'
             '부채꼴은 수평 90° 설명용: 가림·수직 시야·검출 성공을 보증하지 않음\n'
             '첫 검출 위치 ≠ 최초 물리 가시 경계 · 처음 보이는 색은 도착 시각에 따라 달라짐', ha='center', fontsize=12)
    fig.tight_layout(rect=(0, .09, 1, .95));fig.savefig(args.output/'abcd_first_observation_bev.png', dpi=160);plt.close(fig)

    # D: record positions before turn, at first classification, after turn.
    gid = 201326; observed = first[gid]; hp = heads['C1256W000052']
    fig = plt.figure(figsize=(16, 10));ax = fig.add_axes([.055, .13, .60, .78]);text_ax = fig.add_axes([.69, .12, .29, .79]);text_ax.axis('off')
    ax.add_collection(LineCollection(roads, colors='#c0c8cf', linewidths=1))
    t0 = observed['source_stamp_ns']; path = np.array([p['front_point_map_xyz_m'][:2] for p in poses if t0-4e9 < p['source_stamp_ns'] < t0+4e9])
    ax.plot(*path.T, color='#487dab', lw=3, zorder=3)
    stop = np.array(groups[gid]['stop_line_xy']);ax.plot(*stop.T, color='black', lw=5)
    ax.scatter(*hp, s=150, color='#8b35b4', marker='s', zorder=8)
    ax.annotate('D 미션 신호등\n(이 교차로에서 좌회전)', hp, xytext=(5, 12), textcoords='offset points', fontsize=12, color='#8b35b4')
    snapshots = []
    for n, dt, color in [(1, -2, '#d36235'), (2, 0, '#d49a13'), (3, 2, '#168b62')]:
        t = t0+dt*1000000000;xy, _ = interpolate_pose(poses, stamps, t)
        p = poses[min(range(len(poses)), key=lambda i: abs(stamps[i]-t))]
        yaw = p['corrected_native_heading_degrees'];cam = camera_xy(xy, yaw, cx, cy)
        angle = horizontal_angle(cam, yaw+camera_yaw, hp)
        vehicle_cone(ax, cam, yaw+camera_yaw, color, n, 60, fov)
        ax.plot([cam[0], hp[0]], [cam[1], hp[1]], color=color, ls='--', lw=1.6, alpha=.8)
        snapshots.append(dict(number=n, seconds_from_first_actual_classification=dt,
                              front_axle_xy=xy.tolist(), camera_xy=cam.tolist(),
                              corrected_heading_degrees=yaw,
                              signal_horizontal_angle_degrees=angle,
                              horizontal_center_in_FOV=abs(angle) <= fov/2))
    ax.set_xlim(58, 134);ax.set_ylim(609, 665);ax.set_aspect('equal');ax.grid(alpha=.18)
    ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)')
    ax.set_title('D 접근: 북쪽으로 진행 → 우회전 → 동쪽으로 D 정지선 접근', fontsize=15)
    lines = ['① 우회전 전', f"신호 방향: 전방축에서 {abs(snapshots[0]['signal_horizontal_angle_degrees']):.0f}°",
             '수평 시야 ±45° 바깥', '', '② 우회전 중 · 실제 첫 인식',
             f"신호 방향: 약 {abs(snapshots[1]['signal_horizontal_angle_degrees']):.0f}° · 오른쪽 끝",
             '정지선 약 35m 전', '실제 첫 색은 직진 녹색', '→ 좌회전 허가를 본 것은 아님', '',
             '③ 우회전 후', f"신호 방향: 약 {abs(snapshots[2]['signal_horizontal_angle_degrees']):.0f}° · 정면 근처",
             '신호를 계속 추적할 기회', '', '도착 시각이 다른 경우 [가정]',
             '②에서 이미 황색일 수 있음', '그 전에 허가를 못 봤다면', '황색 시작 시점을 알 수 없음']
    text_ax.text(0, 1, '\n'.join(lines), va='top', fontsize=13, linespacing=1.55)
    fig.suptitle('D의 첫 관측이 이미 황색일 수 있다는 뜻', fontsize=22)
    fig.text(.5, .04, '차량 위치·방향·경로는 실제 기록 / 황색 첫 관측은 가능성을 설명한 가정이며 실제 촬영 사례가 아님\n'
             '부채꼴 반경은 설명용 / 수평 시야 그림은 가림·수직 시야·YOLO/CNN 성능을 보증하지 않음', ha='center', fontsize=12)
    fig.savefig(args.output/'d_turn_first_observation_bev.png', dpi=160);plt.close(fig)
    receipt = dict(run=RUNS[0], inputs=inputs, map_sha256=map_sha, groups_sha256=sha(gp),
                   sensor_sha256=sha(sensor_path), geometry_sha256=sha(geometry_path),
                   front_axle_offset_m=3., horizontal_FOV_degrees=fov, first_observations=first,
                   D_snapshots=snapshots, yellow_first_seen='hypothetical, not observed in this plot',
                   caveats=['horizontal only; not occlusion/vertical housing eligibility',
                            'first classification is not first physical visibility boundary',
                            'icons illustrative, not calibrated vehicle footprint'])
    (args.output/'plot_inputs_and_measurements.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(snapshots, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
