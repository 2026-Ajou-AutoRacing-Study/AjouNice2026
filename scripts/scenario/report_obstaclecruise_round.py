#!/usr/bin/env python3
"""Joint evidence report; temporal and spatial traces, never a mission-pass assertion."""
import argparse
import html
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag


def traces(path):
    actual, target, position, controls = [], [], [], []
    odom = None
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=[
                '/Ego_topic', '/localization/kinematic_state', '/planning/trajectory', '/ctrl_cmd']):
            t = stamp.to_sec()-start
            if topic == '/Ego_topic':
                actual.append((t, abs(msg.velocity.x)*3.6))
            elif topic == '/localization/kinematic_state':
                odom = (t, msg.pose.pose.position)
                position.append((t, odom[1].x, odom[1].y))
            elif topic == '/planning/trajectory' and odom and msg.points and t-odom[0] < .3:
                point = min(msg.points, key=lambda p:
                    (p.pose.position.x-odom[1].x)**2+(p.pose.position.y-odom[1].y)**2)
                target.append((t, point.longitudinal_velocity_mps*3.6))
            elif topic == '/ctrl_cmd':
                controls.append((t, msg.accel, msg.brake))
    return actual, target, position, controls, start


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder', type=Path)
    a = p.parse_args()
    experiment = json.loads((a.folder/'round.json').read_text())
    measured_diagnostic = bool(experiment.get('lc_measured_motion_diagnostic_only', False))
    continuous_bounded = bool(experiment.get('lc_continuous_bounded_output', False))
    pre_adjacency_deferral = bool(experiment.get('lc_defer_pre_adjacency_normal', False))
    adaptive_positive = bool(experiment.get('lc_adaptive_positive_grid', False))
    future_mandatory = bool(experiment.get('lc_future_mandatory_anticipation', False))
    anchor_invariant = bool(experiment.get('lc_anchor_invariant_curvature', False))
    post_end_tail = bool(experiment.get('lc_post_end_bounded_tail', False))
    full_envelope = bool(experiment.get('lc_full_envelope_preference', False))
    gt_velocity = bool(experiment.get('od_gt_velocity_observation', False))
    physical_endpoint = bool(experiment.get('lc_physical_endpoint_consistency', False))
    world_candidate = bool(experiment.get('lc_world_candidate', False))
    exact_frenet_endpoint = bool(experiment.get('lc_exact_frenet_endpoint', False))
    final_ego_poll = bool(experiment.get('bpp_final_ego_poll', False))
    reachable_admission = bool(experiment.get('lc_reachable_admission', False))
    rss_positive_grid = bool(experiment.get('lc_rss_positive_grid', False))
    certified_reference = bool(experiment.get('lc_certified_target_only_speed_reference', False))
    preferred_defer = bool(experiment.get('lc_defer_unhelpful_preferred_request', False))
    gt_velocity_verified = gt_velocity and all(
        (a.folder/('prepare%03d'%number)/'velocity_input_gate.json').exists()
        for number in range(1,4))
    runs = sorted(a.folder.glob('official*/evaluation.json'))
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    space, sax = plt.subplots(figsize=(7, 10))
    rows, issues, manifests = [], [], []
    for index, file in enumerate(runs):
        data = json.loads(file.read_text())
        run = file.parent
        manifests.append(json.loads((run/'runtime_manifest.json').read_text()))
        actual, target, position, controls, bag_start = traces(run/'planning.bag')
        ax = axes[index]
        if actual:
            x, y = zip(*actual); ax.plot(x, y, label='ego speed')
        if target:
            x, y = zip(*target); ax.plot(x, y, label='nearest planned speed', alpha=.75)
        ax.axhline(60,color='gray',linestyle=':',alpha=.5,label='60 km/h reference (not a hard floor)')
        intervention=data.get('intervention') or {}
        if intervention.get('stamp') is not None and actual:
            cut=intervention['stamp']-bag_start
            ax.axvline(cut,color='black',linestyle='--',label='harness stop intervention')
            ax.axvspan(cut,actual[-1][0],color='gray',alpha=.13)
        ax.set_title(run.name); ax.set_ylabel('km/h'); ax.grid(alpha=.25); ax.legend(loc='upper left')
        if controls:
            other = ax.twinx()
            x, throttle, brake = zip(*controls)
            other.plot(x, brake, color='red', alpha=.3, label='brake')
            other.set_ylim(0, 1.05); other.set_ylabel('brake command')
        if position:
            _, x, y = zip(*position); sax.plot(x, y, label=run.name, linewidth=1)
        gaps = data.get('nearby_gt_footprint_checks', {}).values()
        gap = min((g['min_gap_m'] for g in gaps), default=None)
        row = dict(run=run.name, reached=data.get('evaluation_waypoint_reached'),
            termination=(run/'termination.txt').read_text().strip(),
            seconds=data.get('driving_time_to_evaluation_waypoint_sec'),
            max_kph=3.6*data.get('max_speed_mps', 0), min_approx_gap_m=gap,
            overlap_ids=data.get('geometric_overlap_ids'),
            invalid=data.get('invalid_counts'), warnings=data.get('warning_counts'))
        rows.append(row)
        issues.append(dict(run=run.name, termination=(run/'termination.txt').read_text().strip(),
            hard_deceleration_events=data.get('hard_deceleration_events'),
            validation_extrema=data.get('validation_extrema'),
            optimizer_failure_events=data.get('optimizer_failure_events'),
            note='Reported factors may be unapproved candidates; extrema can include rear prefix.'))
    axes[-1].set_xlabel('seconds since each bag start (not matched traffic)')
    fig.tight_layout(); fig.savefig(a.folder/'speed_profiles.svg'); plt.close(fig)
    sax.set_aspect('equal'); sax.set_xlabel('map x [m]'); sax.set_ylabel('map y [m]')
    sax.grid(alpha=.25)
    if rows: sax.legend()
    space.tight_layout(); space.savefig(a.folder/'ego_paths.svg'); plt.close(space)
    frozen = bool(manifests) and all(m == manifests[0] for m in manifests)
    summary = dict(runs=len(rows), required_runs=3, runtime_manifest_equal=frozen,
                   complete_round=len(rows)==3 and frozen, mission_complete=False,
                   lc_measured_motion_diagnostic_only=measured_diagnostic,
                   lc_continuous_bounded_output=continuous_bounded,
                   lc_defer_pre_adjacency_normal=pre_adjacency_deferral,
                   lc_adaptive_positive_grid=adaptive_positive,
                   lc_future_mandatory_anticipation=future_mandatory,
                   lc_anchor_invariant_curvature=anchor_invariant,
                   lc_post_end_bounded_tail=post_end_tail,
                   lc_full_envelope_preference=full_envelope,
                   od_gt_velocity_observation=gt_velocity,
                   od_gt_velocity_observation_verified=gt_velocity_verified,
                   lc_physical_endpoint_consistency=physical_endpoint,
                   lc_world_candidate=world_candidate,
                   lc_exact_frenet_endpoint=exact_frenet_endpoint,
                   bpp_final_ego_poll=final_ego_poll,
                   lc_reachable_admission=reachable_admission,
                   lc_rss_positive_grid=rss_positive_grid,
                   lc_certified_target_only_speed_reference=certified_reference,
                   lc_defer_unhelpful_preferred_request=preferred_defer,
                   lc_path_hazard_preferred_request=bool(experiment.get('lc_path_hazard_preferred_request', False)),
                   cruise_reset_on_lead_change=bool(experiment.get('cruise_reset_on_lead_change', False)),
                   cruise_precise_future_admission=bool(experiment.get('cruise_precise_future_admission', False)),
                   results=rows, issues=issues)
    (a.folder/'round_summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    table = '<tr><th>run</th><th>endpoint</th><th>termination</th><th>seconds</th><th>max km/h</th><th>approx min gap m</th><th>overlap IDs</th><th>invalid</th><th>warnings</th></tr>'
    for row in rows:
        table += '<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in row.values())+'</tr>'
    (a.folder/'index.html').write_text('<!doctype html><meta charset="utf-8">'
        '<title>'+a.folder.name+' three-run evidence</title>'
        '<style>body{font:16px sans-serif;margin:30px}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #aaa}img{max-width:100%}pre{white-space:pre-wrap}</style>'
        '<h1>'+a.folder.name+' — 동일 설정 3런 종합</h1>'
        + ('<p><strong>실험 조건:</strong> 현재 가속도를 유지하는 미래 예측은 진단용입니다. '
           '현재 상태와 계획 궤적의 충돌 검사는 유지합니다. 기존 이중 예측 통과 회차와 검사 조건이 다르며, '
           '이 예측에서 발생한 위험 경고도 따로 검토해야 합니다.</p>' if measured_diagnostic else '') +
        ('<p>정상 LC에서는 매 주기 실측 초기 상태와 최신 상한으로 bounded 속도 후보를 생성·검증해 출력합니다. '
         '기존 가속 프로필로 즉시 복귀하던 전환을 제거하는 실험이며, 속도 후보 간 전환과 하위 플래너 응답은 별도 확인합니다.</p>'
         if continuous_bounded else '') +
        ('<p>지도·경로·잔여 합류 공간이 확인된 진입 구간에서만, 옆 차선이 시작되기 전 정상 LC 예약을 미룹니다. '
         '현재 차선 주행은 기존 플래너가 담당하고, 인접 구간에서 후보를 다시 검사합니다. 승인 중 LC·외부 요청·필수 합류 여유가 부족한 경우는 변경하지 않습니다.</p>'
         if pre_adjacency_deferral else '') +
        ('<p>속도 상한과 실측 초기 상태로 생성 가능한 작은 양수가속 후보를 제한된 추가 탐색으로 검사합니다. 기존 충돌·RSS·경계 검사는 유지하며, 전역 최적해 보장은 아닙니다.</p>'
         if adaptive_positive else '') +
        ('<p>현재 합법적인 인접 차선에서 미래 필수 선호 차선으로 이어지는 경로가 확인될 때만 조기 복귀 후보를 고려합니다. 경로 변경·강제 승인은 하지 않고 후보 안전검사를 다시 수행합니다.</p>'
         if future_mandatory else '') +
        ('<p>기존 경로 선분 위에 ego 참조점을 추가할 때 원래 기하의 곡률 상한을 보존합니다. 도로·정지·충돌 제약은 유지하며, 형상 변경은 거부합니다.</p>'
         if anchor_invariant else '') +
        ('<p>물리적 LC 끝을 통과했으나 native 완료 판정 전인 짧은 구간은 현재 차로 추종 구간으로 속도 후보를 검사합니다. LC 끝·완료 상태를 옮기지 않으며 정지·충돌 검사는 유지합니다.</p>'
         if post_end_tail else '') +
        ('<p>안전검사를 통과한 LC 후보끼리 전체 속도 상한에 도달 가능한 형상을 우선 비교합니다. 후보 형상·속도·충돌검사는 변경하지 않는 제한된 점수 실험이며 전역 최적해가 아닙니다.</p>'
         if full_envelope else '') +
        ('<p>GT 속도 관측 옵션 요청='+str(gt_velocity)+'; 3런 출발 전 실제 메시지 검증='+str(gt_velocity_verified)+'. 검증이 없는 회차는 GT 속도 입력 효과로 해석하지 않습니다. 일반 detector/BEVFusion 성능 검증도 아닙니다.</p>'
         if gt_velocity else '') +
        ('<p>LC 실행 형상의 zero-shift 완료점을 정확히 확인한 경우에만 종료 메타데이터와 일치시킵니다. 형상·속도·충돌 제약은 변경하지 않습니다.</p>'
         if physical_endpoint else '') +
        ('<p>기존 NORMAL LC 형상 생성이 거부된 경우에만 world-coordinate 추가 후보를 검사하는 실험입니다. '
         '원래 탐색 시간 한도·정지/곡률 상한·native/전체 객체 RSS·실측 도달가능성·최종 출력 검사를 모두 유지합니다. '
         '옵션 요청 사실은 실제 후보 채택이나 전체 주행 안전의 증명이 아닙니다.</p>'
         if world_candidate else '') +
        ('<p>native Frenet 궤적의 정확한 종료 시각 표본을 포함하는 실험입니다. 기존 필터·순위·메타데이터 처리와 형상·속도·전체 객체 안전검사를 유지합니다. 옵션 활성화는 실제 후보 채택이나 안전 보장이 아닙니다.</p>'
         if exact_frenet_endpoint else '') +
        ('<p>BPP가 다른 입력을 읽은 뒤 최신 측위·가속도를 다시 nonblocking 취득합니다. 객체·측위 시각이나 기존 시간 허용치는 변경하지 않습니다.</p>'
         if final_ego_poll else '') +
        ('<p>LC 승인 전에 현재 속도·가속도로 생성 가능한 제한된 속도 후보가 전체 형상 상한과 기존 모든 객체 안전검사를 통과하는지 추가 검사합니다. 기존 안전검사를 대체하지 않고 실제 실행 중에도 다시 검사합니다. 모든 가능한 궤적의 완전 탐색을 의미하지 않습니다.</p>'
         if reachable_admission else '') +
        ('<p>양의 가속 후보 사이의 빈 구간을 보완합니다. 기존 기본 후보를 유지하고 전체 후보 수를 제한하며, 모든 추가 후보는 동일한 충돌·RSS·경계 검사를 통과해야 합니다.</p>'
         if rss_positive_grid else '') +
        ('<p>동일한 실제 경로점·차량 자세로 차체 전체가 목표 차선 내부에 있고 원차선과 겹치지 않는 점만 인증합니다. 그 점에서 끝난 원차선·upstream의 참조 범위 오류만 목표 차선 속도로 처리하며, 정지점·잘못된 참조·목표 차선 제한·곡률·충돌 검사는 그대로입니다.</p>'
         if certified_reference else '') +
        ('<p>검증된 현재 차로의 합법적 연속 경로와 충분한 여유가 있을 때, 느린 목표 앞차 뒤로 들어가는 초기 NORMAL 선호 LC 요청만 보류합니다. 승인된 LC·필수 복귀·경로·속도·기존 안전검사는 변경하지 않습니다.</p>'
         if preferred_defer else '') +
        '<p>세 런의 교통은 동일하지 않습니다. 시간 차이만으로 수정 효과를 단정하지 않습니다. '
        '전체 미션 완료 판정이 아니며 실패·중단도 표본에 포함합니다. 횡가속 극대는 뒤쪽 잔존점일 수 있습니다. '
        '속도 그래프의 회색 영역은 하네스 정지 개입 이후이며 자율주행 감속과 구분합니다. 60km/h 선은 비교용이지 강제 하한이 아닙니다.</p>'
        '<p>완료 표본 '+str(len(rows))+'/3; runtime manifest equal='+str(frozen)+'</p>'
        '<table>'+table+'</table><h2>실속도·계획속도·제동 시계열</h2><img src="speed_profiles.svg">'
        '<h2>측위 기반 실제 주행 경로 — 계획 회피 후보와 구분</h2><img src="ego_paths.svg">'
        '<h2>모든 감속·검증·최적화 문제 후보</h2><pre>'+html.escape(json.dumps(issues,indent=2,ensure_ascii=False))+'</pre>')
    links = ''.join('<li><a href="'+d.name+'/index.html">'+d.name+'</a>' +
                    (' · <a href="'+d.name+'/review.html">종합 판정</a>'
                     if (d/'review.html').exists() else '') + '</li>'
                    for d in sorted(a.folder.parent.glob('round*')) if (d/'index.html').exists())
    (a.folder.parent/'index.html').write_text('<!doctype html><meta charset="utf-8">'
        '<h1>OD_GT 튜닝 — 최대100회차 × 3런</h1><p>기존14런은 별도 기준 자료. '
        '각 회차 설정 동결, 실패도 보존. 근거 없는 미션 완료 판정 없음.</p>'
        '<p><a href="overview.html">전체 현황·주요 원인·수정 전후 진입점</a> · '
        '<a href="candidates/report/index.html">상세 문제·수정 후보 비교</a></p><ul>'+links+'</ul>')
    print('ROUND REPORT', a.folder, 'samples', len(rows), 'frozen', frozen, flush=True)


if __name__ == '__main__':
    main()
