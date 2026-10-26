#!/usr/bin/env python3
"""동적 장애물 구간 보행자 시나리오 생성기 (AjouNice2026).

음영구간 생성기(gen_shadow_pedestrian_scenarios.py)와 같은 구조.
시드가 같으면 누가 돌려도 바이트 단위로 동일한 시나리오가 나온다.

    python3 scripts/scenario/gen_dynamic_pedestrian_scenarios.py
    python3 scripts/scenario/gen_dynamic_pedestrian_scenarios.py --count 300 --seed 20261027

기하는 전부 실측값이다. 일요일 캠페인
(dynamic_obstacle_starter_261025_193827) 의 planbag에서
/localization/kinematic_state 와 /map/lanelet_map 을 읽어 확정했다.
"""
import argparse
import csv
import json
import math
import os
import random
import shutil

ROOT = os.environ.get('AJOUNICE_ROOT', os.path.expanduser('~/AjouNice2026'))
MIS = os.path.join(
    ROOT, 'base_ws/src/planning_addons/planning_scenario_validator/config/missions')
MISSION = os.path.join(MIS, 'dynamic_obstacle')
SRC = os.path.join(MISSION, 'base/scenarios')
OUT = os.path.join(MISSION, 'workspace/scenarios')
WS = os.path.join(MISSION, 'workspace')
PED2_SRC = os.path.join(MIS, 'gps_shadow/base/scenarios/hid_soa_normal_09.json')

# ── 주행 구간 (실측: ego는 y 464에서 대기 후 y 584까지 주행)
EGO_X = 71.25                 # ego 차로 중심 (lanelet 중심선 피크)
EGO_START = (71.73, 464.0)    # 스폰 대기 지점
Y0, Y1 = 480.0, 580.0         # 보행자 배치 범위 (가속 완료 ~ 감속 시작)
NCELL = 15
GROUND = 28.43                # 지도 중심선 z (base_link 28.67이 아님. 0.24 m 차이)

# ── 횡방향 (lat = -(x - EGO_X), 진행방향 +y)
#    노견 -6.5 | A -3.85 | EGO 0 | C +3.85 | D +7.65 | 노견 +10.3   폭 15.35 m
SN, LANE_A, EGO, LANE_C, LANE_D, SF = -6.5, -3.85, 0.0, 3.85, 7.65, 10.3
HALF_LANE = 1.925
EDGE_F = HALF_LANE + 0.3      # ego 차로 밖 0.3 m — pass/stall 경계 실험

# ── 측정 속도 프로파일 (4 m 구간 평균, 정지시간 포함한 실효속도)
#    y 511 정지선과 y 539 교차로에서 떨어진다. 상수 V_EGO를 쓰지 않는 이유.
VPROF = [
    (464, 0.01), (468, 3.26), (472, 6.04), (476, 7.58), (480, 8.68),
    (484, 9.37), (488, 9.81), (492, 10.02), (496, 10.03), (500, 9.77),
    (504, 8.65), (508, 7.00), (512, 4.98), (516, 5.44), (520, 7.18),
    (524, 7.88), (528, 2.32), (532, 5.38), (536, 7.24), (540, 8.34),
    (544, 9.25), (548, 9.93), (552, 10.30), (556, 10.50), (560, 10.18),
    (564, 9.36), (568, 8.24), (572, 6.02), (576, 1.99), (580, 2.65),
    (584, 0.47),
]
VFLOOR = 1.5                  # 적분 발산 방지

# ── 보행자 속도: MORAI speed 필드는 m/s가 아니라 약 x0.16
SPEEDS = {5: 0.80, 10: 1.53, 13: 2.00, 19: 3.00}
# ── 종방향 이동 길이 (종방향/차선내 패턴에만 적용)
LONS = {'s': 14.0, 'm': 40.0, 'l': 70.0}
MIN_GAP = 15.0                # 2명일 때 종방향 보행구간 최소 간격

ASSETS = [
    (50100001, 'Man1'), (50100002, 'Man2'), (50100003, 'Man3'),
    (50100004, 'Woman1'), (50100005, 'Woman2'), (50100006, 'Woman3'),
    (50100007, 'Man4'), (50100008, 'Man5'), (50100009, 'Man6'),
    (50100010, 'Man7'), (50100011, 'Man8'), (50100012, 'Woman4'),
    (50100013, 'Woman5'), (50100014, 'Woman6'), (50100015, 'Woman7'),
    (50100016, 'Woman8'), (50100055, 'police_f_1'), (50100058, 'police_m_2'),
    (50100074, 'worker_f_1'), (50100077, 'worker_m_1'),
    (50200001, 'Boy1'), (50200002, 'Girl1'), (50200003, 'Boy2'), (50200004, 'Girl2'),
]

# 횡단 계열: 종방향 길이 축이 없다 (수직 횡단)
CROSS = ('X_fn', 'X_nf', 'H_fb', 'H_nb', 'E_fa', 'E_nc')
PATTERNS = CROSS + ('D_fwd', 'D_bwd', 'L_ego', 'L_A', 'L_CD')
ROT = {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"}
CELL = (Y1 - Y0) / NCELL


def unity(x, y, z):
    return {"x": -x, "y": z, "z": -y,
            "_x": "%.3f" % -x, "_y": "%.3f" % z, "_z": "%.3f" % -y}


def world(x, y, z):
    return {"x": x, "y": y, "z": z,
            "_x": "%.3f" % x, "_y": "%.3f" % y, "_z": "%.3f" % z}


def at(along, lat):
    """구간 좌표 (진행거리, 횡오프셋) -> map (x, y)."""
    return (EGO_X - lat, Y0 + along)


def vat(y):
    """측정 프로파일 선형보간."""
    if y <= VPROF[0][0]:
        return max(VPROF[0][1], VFLOOR)
    if y >= VPROF[-1][0]:
        return max(VPROF[-1][1], VFLOOR)
    for i in range(len(VPROF) - 1):
        y0, v0 = VPROF[i]
        y1, v1 = VPROF[i + 1]
        if y0 <= y <= y1:
            t = (y - y0) / (y1 - y0)
            return max(v0 + (v1 - v0) * t, VFLOOR)
    return VFLOOR


_TGRID = None


def tgrid():
    """y -> 누적 주행시간 (y=464 기준). 0.5 m 적분."""
    global _TGRID
    if _TGRID is None:
        g, t, y = [], 0.0, 464.0
        while y <= 586.0:
            g.append((y, t))
            t += 0.5 / vat(y + 0.25)
            y += 0.5
        _TGRID = g
    return _TGRID


def y_before(y_target, dt):
    """y_target 도달 dt초 전 ego의 y."""
    g = tgrid()
    tt = None
    for y, t in g:
        if y >= y_target:
            tt = t
            break
    if tt is None:
        tt = g[-1][1]
    want = tt - dt
    if want <= g[0][1]:
        return g[0][0]
    prev = g[0][0]
    for y, t in g:
        if t >= want:
            return prev
        prev = y
    return g[-1][0]


def endpoints(cell, pat, lon_key):
    """패턴별 시작·끝 (진행거리, 횡오프셋)."""
    c = (cell + 0.5) * CELL
    if pat in CROSS:
        lo = hi = c
    else:
        half = LONS[lon_key] / 2.0
        lo, hi = c - half, c + half
        if lo < 1.0:
            lo, hi = 1.0, 1.0 + LONS[lon_key]
        if hi > (Y1 - Y0) - 1.0:
            hi = (Y1 - Y0) - 1.0
            lo = hi - LONS[lon_key]
        lo = max(lo, 0.5)
    if pat == 'X_fn':
        return (lo, SF), (hi, SN)
    if pat == 'X_nf':
        return (lo, SN), (hi, SF)
    if pat == 'H_fb':
        return (lo, SF), (hi, EGO)
    if pat == 'H_nb':
        return (lo, SN), (hi, EGO)
    if pat == 'E_fa':
        return (lo, SF), (hi, EDGE_F)
    if pat == 'E_nc':
        return (lo, SN), (hi, LANE_C)
    if pat == 'D_fwd':
        return (lo, EGO), (hi, EGO)
    if pat == 'D_bwd':
        return (hi, EGO), (lo, EGO)
    if pat == 'L_ego':
        return (lo, -HALF_LANE + 0.4), (hi, HALF_LANE - 0.4)
    if pat == 'L_A':
        return (lo, LANE_A), (hi, LANE_A)
    return (lo, LANE_C), (hi, LANE_D)


def active_distance(cell, pat, spd_key, lon_key):
    """보행자가 ego 차로를 점유하는 순간에 ego가 도달하도록 역산.

    측정 속도 프로파일로 적분하므로 정지선 감속 구간도 반영된다.
    """
    v = SPEEDS[spd_key]
    (a0, l0), (a1, l1) = endpoints(cell, pat, lon_key)
    sx, sy = at(a0, l0)
    if pat in CROSS:
        if pat == 'E_fa':
            reach = abs(l1 - l0)                 # 끝점까지
        else:
            reach = abs(l0 - EGO)                # ego 차로 중심까지
    else:
        reach = math.hypot(*(at(a1, l1)[i] - at(a0, l0)[i] for i in (0, 1))) / 2.0
    tw = reach / v
    yc = Y0 + (a0 + a1) / 2.0
    ys = y_before(yc, tw)
    ad = math.hypot(sx - EGO_X, sy - ys)
    # 스폰에서 보행자까지보다 크면 로드 즉시 켜져 대기 50초 동안 다 걸어가 버린다
    room = math.hypot(sx - EGO_START[0], sy - EGO_START[1]) - 8.0
    return round(min(max(ad, 8.0), max(room, 8.0)), 1)


def span(cell, pat, lon_key):
    (a0, _), (a1, _) = endpoints(cell, pat, lon_key)
    return (min(a0, a1), max(a0, a1))


def build_plan(count, rng):
    full = []
    for pat in PATTERNS:
        lons = ('x',) if pat in CROSS else tuple(LONS)
        for cell in range(NCELL):
            for spd in SPEEDS:
                for lon in lons:
                    full.append((cell, pat, spd, lon))
    must = []
    for pat in PATTERNS:
        for sec in range(3):
            for spd in SPEEDS:
                pool = [c for c in full
                        if c[1] == pat and c[2] == spd and sec * 5 <= c[0] < sec * 5 + 5]
                must.append(rng.choice(pool))
    # 패턴별 라운드로빈으로 채운다. 그냥 섞으면 종길이 축이 있는 패턴이
    # 조합 수가 3배라 횡단 계열이 묻힌다.
    taken = set(must)
    pools = {}
    for pat in PATTERNS:
        pool = [c for c in full if c[1] == pat and c not in taken]
        rng.shuffle(pool)
        pools[pat] = pool
    plan = list(must)
    while len(plan) < count:
        added = False
        for pat in PATTERNS:
            if len(plan) >= count:
                break
            if pools[pat]:
                plan.append(pools[pat].pop())
                added = True
        if not added:
            break
    rng.shuffle(plan)
    return plan


def place(ped, cell, pat, spd, lon, asset):
    (a0, l0), (a1, l1) = endpoints(cell, pat, lon)
    s, e = at(a0, l0), at(a1, l1)
    walk = math.hypot(e[0] - s[0], e[1] - s[1])
    ped['DataID'] = asset[0]
    ped.update(speed=float(spd),
               activeDistance=active_distance(cell, pat, spd, lon),
               active=True, loop=False, isWaypointMode=True)
    # movingDistance 의미가 불확실하다. 템플릿 기본 5.0이 보행거리를 제한할
    # 가능성이 있어 넉넉하게 덮어쓴다.
    if 'movingDistance' in ped:
        ped['movingDistance'] = round(walk + 5.0, 1)
    ped['pos'] = world(s[0], s[1], GROUND)
    ped['initPos'] = unity(s[0], s[1], GROUND)
    ped['standardPos'] = unity(s[0], s[1], GROUND)
    path = {"pathName": "PedPath - %d" % ped['UNIQUEID'],
            "waypointData": [{"pos": unity(s[0], s[1], GROUND), "rot": dict(ROT)},
                             {"pos": unity(e[0], e[1], GROUND), "rot": dict(ROT)}]}
    return path, walk


def load_templates():
    """1명은 미션 자체 파일, 2명은 음영구간 파일의 보행자 배열만 이식."""
    one = json.load(open(os.path.join(SRC, 'do_baseline_01.json')))
    two = json.loads(json.dumps(one))
    if os.path.exists(PED2_SRC):
        donor = json.load(open(PED2_SRC))
        peds = donor.get('pedestrianList') or []
        if len(peds) >= 2:
            two['pedestrianList'] = json.loads(json.dumps(peds[:2]))
    ids = [p.get('UNIQUEID') for p in (two.get('pedestrianList') or [])]
    if len(set(ids)) < len(ids):
        raise SystemExit('2명 템플릿의 UNIQUEID가 중복입니다: %s' % ids)
    return one, two


def write_configs(timeout):
    """캠페인 YAML 생성.

    stack / localization_safety 는 음영구간 base에서 그대로 들어온다.
    필드를 새로 만들어내지 않는다 (스키마 추측 금지).
    detector 는 od_gt 고정 — 이 환경의 GTX 1060(sm_61)에서 BEVFusion이
    안 돌고, 이 과제는 플래닝의 정지 판단을 보는 것이라 인지는 변수에서 뺀다.
    """
    try:
        import yaml
    except ImportError:
        print('[skip] PyYAML 없음 — 캠페인 YAML은 수동으로 만드세요')
        return
    SH = os.path.join(MIS, 'gps_shadow/base')
    legacy = os.path.join(WS, 'dp_set.yaml')      # 구버전 이름 정리
    if os.path.exists(legacy):
        os.remove(legacy)
        print('  remove  dp_set.yaml (구버전)')
    variants = (
        ('dp_set_gt.yaml', 'example.yaml', 'localization_gt_pose.yaml',
         'dynped_gt_pose'),
        ('dp_set_base.yaml', 'example.yaml', 'localization_baseline.yaml',
         'dynped_baseline'),
        ('dp_set_lan.yaml', 'example_lan.yaml', 'localization_gt_pose.yaml',
         'dynped_gt_pose_lan'),
    )
    for out, base, donor, cname in variants:
        p = os.path.join(MISSION, 'base', base)
        dp = os.path.join(SH, donor)
        if not os.path.exists(p):
            print('  [skip] %s — base %s 없음' % (out, base))
            continue
        if not os.path.exists(dp):
            print('  [skip] %s — 공여 설정 %s 없음' % (out, donor))
            continue
        d = yaml.safe_load(open(p))
        sd = yaml.safe_load(open(dp)) or {}
        # 캠페인 이름이 설정과 어긋나면 결과 디렉터리를 찾을 때 헷갈린다
        d['campaign_name'] = cname
        d['stack'] = dict(sd.get('stack') or d.get('stack') or {})
        d['stack']['detector'] = 'od_gt'
        safety = dict(((sd.get('validation') or {}).get('localization_safety')) or {})
        safety.update(enabled=True, stop_on_collision=False)
        d.setdefault('validation', {})['localization_safety'] = safety
        d['scenarios'].pop('include', None)
        d['scenarios']['directory'] = os.path.relpath(OUT, ROOT)
        d['scenarios']['glob'] = 'dq*.json'
        d['scenarios']['repeat'] = 1
        d['default_case']['timeout_sec'] = timeout
        # goal은 base 값을 그대로 쓴다 (70.991, 585.981). 임의 단축 금지.
        v = d.setdefault('recording', {}).setdefault('video', {})
        v.update(enabled=True, display=':1', fps=10, window_layout='split')
        yaml.safe_dump(d, open(os.path.join(WS, out), 'w'),
                       allow_unicode=True, sort_keys=False)
        print('  config  %-16s pose=%-13s detector=%s'
              % (out, d['stack'].get('pose_source'), d['stack'].get('detector')))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', type=int, default=300)
    ap.add_argument('--seed', type=int, default=20261027)
    ap.add_argument('--two-ratio', type=float, default=0.35)
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        if f.startswith('dq') or f == '_manifest.csv':
            os.remove(os.path.join(OUT, f))

    tmpl1, tmpl2 = load_templates()
    plan = build_plan(args.count, rng)
    two = set(rng.sample(range(len(plan)), int(len(plan) * args.two_ratio)))

    rows = []
    for idx, (cell, pat, spd, lon) in enumerate(plan, 1):
        lo1, hi1 = span(cell, pat, lon)
        second = None
        if (idx - 1) in two:
            for _ in range(80):
                c2 = rng.randrange(NCELL)
                p2 = rng.choice(PATTERNS)
                l2 = 'x' if p2 in CROSS else rng.choice(list(LONS))
                s2 = rng.choice(list(SPEEDS))
                lo2, hi2 = span(c2, p2, l2)
                if max(lo1, lo2) - min(hi1, hi2) >= MIN_GAP:
                    second = (c2, p2, s2, l2)
                    break
        n = 2 if second else 1
        d = json.loads(json.dumps(tmpl2 if n == 2 else tmpl1))
        d['vehicleList'] = []
        d['pedestrianList'] = d['pedestrianList'][:n]
        a1 = ASSETS[rng.randrange(len(ASSETS))]
        w1, walk1 = place(d['pedestrianList'][0], cell, pat, spd, lon, a1)
        paths = [w1]
        if n == 2:
            c2, p2, s2, l2 = second
            a2 = ASSETS[rng.randrange(len(ASSETS))]
            w2, _ = place(d['pedestrianList'][1], c2, p2, s2, l2, a2)
            paths.append(w2)
            second = (c2 + 1, p2, s2, l2, a2[1])
        d['waypointDataList'] = paths
        nm = 'dq%03d_%s_c%02d_v%02d_%s%s' % (
            idx, pat, cell + 1, spd, lon, '_x2' if n == 2 else '')
        json.dump(d, open(os.path.join(OUT, nm + '.json'), 'w'), ensure_ascii=False)
        rows.append([nm, n, cell + 1, pat, spd, round(SPEEDS[spd], 2), lon,
                     d['pedestrianList'][0]['activeDistance'], a1[1], round(walk1, 1),
                     *(second if second else ('', '', '', '', ''))])

    # do_baseline_01 은 쓰지 않는다. 그 파일의 보행자는 실제 주행 경로에서
    # 600 m 벗어난 (-56.74, -11.61) 에 있어 장애물 역할을 하지 못한다.
    for i, src in enumerate(['do_baseline_00']):
        p = os.path.join(SRC, src + '.json')
        if os.path.exists(p):
            shutil.copy(p, os.path.join(OUT, 'dq_base%d.json' % i))
            rows.append(['dq_base%d' % i, 0, '', '', '', '', '', '', '', '',
                         '', '', '', '', ''])

    with open(os.path.join(OUT, '_manifest.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['name', 'ped', 'cell', 'pattern', 'speed_field', 'speed_mps', 'lon',
                    'active_distance', 'asset', 'walk_m',
                    'cell2', 'pattern2', 'speed2', 'lon2', 'asset2'])
        w.writerows(rows)

    write_configs(args.timeout)

    n2 = sum(1 for r in rows if r[1] == 2)
    ads = [r[7] for r in rows if r[1]]
    print()
    print('시드 %d · 시나리오 %d개 (2명 %d개) + baseline %d = %d개'
          % (args.seed, len(plan), n2, len(rows) - len(plan), len(rows)))
    print('칸 %d개 x %.2f m · 구간 y %.0f~%.0f · 패턴 %d · 속도 %s · 종길이 %s'
          % (NCELL, CELL, Y0, Y1, len(PATTERNS), list(SPEEDS), list(LONS)))
    print('ego 차로 x %.2f · 도로폭 15.35 m · 지면 z %.2f · 자산 %d종'
          % (EGO_X, GROUND, len(ASSETS)))
    print('activeDistance %.1f~%.1f m (측정 속도 프로파일 역산)'
          % (min(ads), max(ads)))
    print('goal은 base YAML 값 유지 (70.991, 585.981)')
    print('manifest: %s' % os.path.join(OUT, '_manifest.csv'))


if __name__ == '__main__':
    main()
