#!/usr/bin/env python3
"""동적 장애물 구간 — 정지선 앞 돌발 보행자 시나리오 생성기 (AjouNice2026).

음영구간처럼 도로를 균등 분할하지 않는다. 이 구간의 핵심은 "신호등 정지선
앞에서 보행자가 갑자기 튀어나오는" 상황이므로, 위치는 정지선 기준 상대
거리 7종으로 두고 나머지 예산을 방향·돌발성·속도·길이에 쓴다.

    python3 scripts/scenario/gen_dynamic_pedestrian_scenarios.py
    python3 scripts/scenario/gen_dynamic_pedestrian_scenarios.py --count 300

시드가 같으면 누가 돌려도 바이트 단위로 동일한 시나리오가 나온다.

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
EGO_START = (71.73, 464.0)    # 실제 스폰·대기 지점
EGO_Z = 28.726
GROUND = 28.70                # 탐침 pb_ego 에서 생성·보행이 확인된 값
Y_MIN, Y_MAX = 478.0, 580.0   # 배치 허용 범위

# ── 신호등 정지선 (실측: features/stop_lines, x 69.67->73.76 = ego 차로 폭)
STOPLINE = 510.6
# 정지선 기준 상대 거리. ego가 감속·정지·재출발하는 구간이라
# 같은 보행자라도 거동이 완전히 달라진다.
OFFS = (-30.0, -18.0, -8.0, -3.0, 4.0, 15.0, 28.0)

# ── 횡방향 (lat = -(x - EGO_X), 진행방향 +y)
#    노견 -6.5 | A -3.85 | EGO 0 | C +3.85 | D +7.65 | 노견 +10.3   폭 15.35 m
SN, LANE_A, EGO, LANE_C, LANE_D, SF = -6.5, -3.85, 0.0, 3.85, 7.65, 10.3
HALF_LANE = 1.925

# ── 등장 방향. 꼭 횡단보도일 필요 없음: 비스듬한 진입과 차로 사이 등장 포함
DIRS = {
    'Rperp': (SN, 0.0),       # 우측 노견에서 수직
    'Rdiag': (SN, 0.7),       # 우측에서 비스듬히 (진행방향으로 기울어짐)
    'Lperp': (SF, 0.0),       # 좌측 노견에서 수직
    'Ldiag': (SF, 0.7),       # 좌측에서 비스듬히
    'Mid': (LANE_C, 0.0),     # 차로 사이에서 등장 (정차 차량 뒤 모사)
}
# ── 보행 길이
LENS = ('enter', 'cross', 'along')
ALONG_LEN = 30.0              # 차로 안으로 들어와 종방향으로 걷는 거리

# ── 돌발성: activeDistance 배수. 클수록 일찍 출발해 여유롭고,
#    작을수록 늦게 튀어나와 제동거리를 시험한다.
URG = {'early': 1.8, 'ontime': 1.0, 'late': 0.55}

# ── 보행자 속도: MORAI speed 필드는 m/s가 아니라 약 x0.16
SPEEDS = {5: 0.80, 10: 1.53, 13: 2.00, 19: 3.00}
MIN_GAP = 15.0                # 2명일 때 종방향 최소 간격

# ── 측정 속도 프로파일 (4 m 구간 평균 = 그 구간 통과시간의 역수)
VPROF = [
    (464, 0.01), (468, 3.26), (472, 6.04), (476, 7.58), (480, 8.68),
    (484, 9.37), (488, 9.81), (492, 10.02), (496, 10.03), (500, 9.77),
    (504, 8.65), (508, 7.00), (512, 4.98), (516, 5.44), (520, 7.18),
    (524, 7.88), (528, 2.32), (532, 5.38), (536, 7.24), (540, 8.34),
    (544, 9.25), (548, 9.93), (552, 10.30), (556, 10.50), (560, 10.18),
    (564, 9.36), (568, 8.24), (572, 6.02), (576, 1.99), (580, 2.65),
    (584, 0.47),
]
VFLOOR = 1.5

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
ROT = {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"}


def unity(x, y, z):
    return {"x": -x, "y": z, "z": -y,
            "_x": "%.3f" % -x, "_y": "%.3f" % z, "_z": "%.3f" % -y}


def world(x, y, z):
    return {"x": x, "y": y, "z": z,
            "_x": "%.3f" % x, "_y": "%.3f" % y, "_z": "%.3f" % z}


def xy(y, lat):
    return (EGO_X - lat, y)


def vat(y):
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


_TG = None


def tgrid():
    global _TG
    if _TG is None:
        g, t, y = [], 0.0, 464.0
        while y <= 586.0:
            g.append((y, t))
            t += 0.5 / vat(y + 0.25)
            y += 0.5
        _TG = g
    return _TG


def y_before(y_target, dt):
    """y_target 도달 dt초 전 ego의 y."""
    g = tgrid()
    tt = g[-1][1]
    for y, t in g:
        if y >= y_target:
            tt = t
            break
    want = tt - dt
    if want <= g[0][1]:
        return g[0][0]
    prev = g[0][0]
    for y, t in g:
        if t >= want:
            return prev
        prev = y
    return g[-1][0]


def legs(off, dirn, length):
    """보행 경로 꼭짓점 [(y, lat), ...]. 2~3점."""
    lat0, diag = DIRS[dirn]
    y0 = STOPLINE + off
    if length == 'cross':
        lat1 = SF if lat0 < 0 else SN
    else:
        lat1 = EGO
    dy = abs(lat1 - lat0) * diag
    pts = [(y0, lat0), (y0 + dy, lat1)]
    if length == 'along':
        pts.append((y0 + dy + ALONG_LEN, EGO))
    lo = min(p[0] for p in pts)
    hi = max(p[0] for p in pts)
    shift = 0.0
    if lo < Y_MIN:
        shift = Y_MIN - lo
    elif hi > Y_MAX:
        shift = Y_MAX - hi
    return [(y + shift, lat) for y, lat in pts]


def walk_len(pts):
    tot = 0.0
    for a, b in zip(pts, pts[1:]):
        pa, pb = xy(*a), xy(*b)
        tot += math.hypot(pb[0] - pa[0], pb[1] - pa[1])
    return tot


def span(off, dirn, length):
    pts = legs(off, dirn, length)
    return (min(p[0] for p in pts), max(p[0] for p in pts))


def active_distance(off, dirn, length, spd_key, urg):
    """보행자가 ego 차로를 점유하는 순간에 ego가 도달하도록 역산한 뒤,
    돌발성 배수를 적용한다."""
    v = SPEEDS[spd_key]
    pts = legs(off, dirn, length)
    (y0, lat0) = pts[0]
    # ego 차로에 처음 닿기까지의 보행 거리
    reach = abs(lat0) * math.hypot(1.0, DIRS[dirn][1])
    t_walk = reach / v
    frac = abs(lat0) / max(abs(pts[1][1] - lat0), 1e-6)
    y_enter = y0 + (pts[1][0] - y0) * min(frac, 1.0)
    ys = y_before(y_enter, t_walk)
    sx, sy = xy(y0, lat0)
    ad = math.hypot(sx - EGO_X, sy - ys) * URG[urg]
    # 스폰 거리보다 크면 로드 즉시 켜져 대기 50초 동안 다 걸어가 버린다
    room = math.hypot(sx - EGO_START[0], sy - EGO_START[1]) - 8.0
    return round(min(max(ad, 6.0), max(room, 6.0)), 1)


def build_plan(count, rng):
    full = [(o, d, l, s, u)
            for o in OFFS for d in DIRS for l in LENS
            for s in SPEEDS for u in URG]
    # 보장 집합: 위치 x 돌발성 x 속도 = 7 x 3 x 4 = 84
    must = []
    for o in OFFS:
        for u in URG:
            for s in SPEEDS:
                pool = [c for c in full if c[0] == o and c[4] == u and c[3] == s]
                must.append(rng.choice(pool))
    taken = set(must)
    # 방향 x 길이로 라운드로빈 채움
    pools = {}
    for d in DIRS:
        for l in LENS:
            pool = [c for c in full if c[1] == d and c[2] == l and c not in taken]
            rng.shuffle(pool)
            pools[(d, l)] = pool
    keys = sorted(pools)
    plan = list(must)
    while len(plan) < count:
        added = False
        for k in keys:
            if len(plan) >= count:
                break
            if pools[k]:
                plan.append(pools[k].pop())
                added = True
        if not added:
            break
    rng.shuffle(plan)
    return plan


def place(ped, off, dirn, length, spd, urg, asset):
    pts = legs(off, dirn, length)
    wl = walk_len(pts)
    ped['DataID'] = asset[0]
    ped.update(speed=float(spd),
               activeDistance=active_distance(off, dirn, length, spd, urg),
               active=True, loop=False, isWaypointMode=True)
    # movingDistance 기본 5.0 이 보행거리를 제한할 가능성이 있어 넉넉히 덮어쓴다
    if 'movingDistance' in ped:
        ped['movingDistance'] = round(wl + 5.0, 1)
    s = xy(*pts[0])
    ped['pos'] = world(s[0], s[1], GROUND)
    ped['initPos'] = unity(s[0], s[1], GROUND)
    ped['standardPos'] = unity(s[0], s[1], GROUND)
    wps = []
    for y, lat in pts:
        px, py = xy(y, lat)
        wps.append({"pos": unity(px, py, GROUND), "rot": dict(ROT)})
    path = {"pathName": "PedPath - %d" % ped['UNIQUEID'], "waypointData": wps}
    return path, wl


def load_templates():
    """1명은 미션 자체 파일, 2명은 음영구간 파일의 보행자 배열만 이식."""
    one = json.load(open(os.path.join(SRC, 'do_baseline_01.json')))
    # do_baseline_01 의 ego 스폰 (-59.89, -80.53) 은 실제 주행 시작점에서
    # 600 m 벗어난 낡은 값이다. 교정하지 않으면 MORAI가 보행자를 아예
    # 생성하지 않는다 (탐침 pb_both 0샘플 vs pb_ego 739샘플로 확인).
    ip = one['egoVehicle']['initPosition']
    ip['pos'].update(x=EGO_START[0], y=EGO_START[1], z=EGO_Z,
                     _x='%.3f' % EGO_START[0], _y='%.3f' % EGO_START[1],
                     _z='%.3f' % EGO_Z)
    two = json.loads(json.dumps(one))
    if os.path.exists(PED2_SRC):
        peds = (json.load(open(PED2_SRC)).get('pedestrianList') or [])
        if len(peds) >= 2:
            two['pedestrianList'] = json.loads(json.dumps(peds[:2]))
    ids = [p.get('UNIQUEID') for p in (two.get('pedestrianList') or [])]
    if len(set(ids)) < len(ids):
        raise SystemExit('2명 템플릿의 UNIQUEID 중복: %s' % ids)
    return one, two


def write_configs(timeout):
    """캠페인 YAML 생성.

    일요일에 실제로 통과한 starter 설정(stack/network/타임아웃)을 쓰려면
    source_config.yaml 을 기준으로 삼는 게 가장 안전하다. 없으면 미션의
    base YAML에 음영구간 stack을 얹는다.
    """
    try:
        import yaml
    except ImportError:
        print('[skip] PyYAML 없음 — 캠페인 YAML은 수동으로 만드세요')
        return
    # 1순위는 레포에 커밋된 검증본. planner_logs 쪽은 생성 PC에만 있으므로
    # 다른 컴퓨터에서도 돌아가려면 base/starter_od_gt.yaml 이 있어야 한다.
    cands = [os.path.join(MISSION, 'base/starter_od_gt.yaml'),
             os.path.join(ROOT, 'planner_logs/validation',
                          'dynamic_obstacle_starter_261025_193827/source_config.yaml')]
    known = next((p for p in cands if os.path.exists(p)), None)
    rel = os.path.relpath(OUT, ROOT)
    if known:
        print('  base 설정: %s' % os.path.relpath(known, ROOT))
        base = yaml.safe_load(open(known))
        for out, cname, resume in (('dp_run.yaml', 'dynped_run', True),
                                   ('dp_set_gt.yaml', 'dynped_gt', True)):
            d = yaml.safe_load(yaml.safe_dump(base))
            d['campaign_name'] = cname
            d['scenarios'] = {'directory': rel, 'glob': 'dq*.json', 'repeat': 1}
            d['resume'] = resume
            d['continue_on_system_error'] = True
            d['default_case']['timeout_sec'] = timeout
            d.setdefault('validation', {}).setdefault(
                'localization_safety', {})['enabled'] = True
            d['validation']['localization_safety']['stop_on_collision'] = False
            d.setdefault('recording', {}).setdefault('video', {}).update(
                enabled=True, display=':1', fps=10, window_layout='split')
            yaml.safe_dump(d, open(os.path.join(WS, out), 'w'),
                           allow_unicode=True, sort_keys=False)
            print('  config  %-16s stack=%s' % (out, d.get('stack')))
    else:
        print('  [경고] 검증된 starter 설정을 못 찾았습니다. 아래 중 하나가 필요합니다:')
        for p in cands:
            print('     %s' % os.path.relpath(p, ROOT))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', type=int, default=300)
    ap.add_argument('--seed', type=int, default=20261027)
    ap.add_argument('--two-ratio', type=float, default=0.30)
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
    for idx, (off, dirn, length, spd, urg) in enumerate(plan, 1):
        lo1, hi1 = span(off, dirn, length)
        second = None
        if (idx - 1) in two:
            for _ in range(80):
                c2 = (rng.choice(OFFS), rng.choice(list(DIRS)),
                      rng.choice(LENS), rng.choice(list(SPEEDS)),
                      rng.choice(list(URG)))
                lo2, hi2 = span(c2[0], c2[1], c2[2])
                if max(lo1, lo2) - min(hi1, hi2) >= MIN_GAP:
                    second = c2
                    break
        n = 2 if second else 1
        d = json.loads(json.dumps(tmpl2 if n == 2 else tmpl1))
        d['vehicleList'] = []
        d['pedestrianList'] = d['pedestrianList'][:n]
        a1 = ASSETS[rng.randrange(len(ASSETS))]
        w1, wl1 = place(d['pedestrianList'][0], off, dirn, length, spd, urg, a1)
        paths = [w1]
        if n == 2:
            a2 = ASSETS[rng.randrange(len(ASSETS))]
            w2, _ = place(d['pedestrianList'][1], second[0], second[1],
                          second[2], second[3], second[4], a2)
            paths.append(w2)
        d['waypointDataList'] = paths
        nm = 'dq%03d_%s_%s_p%s_%s_v%02d%s' % (
            idx, dirn, length, ('m%02d' % -off) if off < 0 else ('p%02d' % off),
            urg[0].upper(), spd, '_x2' if n == 2 else '')
        json.dump(d, open(os.path.join(OUT, nm + '.json'), 'w'), ensure_ascii=False)
        rows.append([nm, n, '%+.0f' % off, round(STOPLINE + off, 1), dirn, length,
                     urg, spd, round(SPEEDS[spd], 2),
                     d['pedestrianList'][0]['activeDistance'], a1[1], round(wl1, 1),
                     *(['%+.0f' % second[0], second[1], second[2], second[3],
                        second[4]] if second else ['', '', '', '', ''])])

    p = os.path.join(SRC, 'do_baseline_00.json')
    if os.path.exists(p):
        shutil.copy(p, os.path.join(OUT, 'dq_base0.json'))
        rows.append(['dq_base0', 0] + [''] * 15)

    with open(os.path.join(OUT, '_manifest.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['name', 'ped', 'offset_m', 'y', 'dir', 'len', 'urgency',
                    'speed_field', 'speed_mps', 'active_distance', 'asset', 'walk_m',
                    'offset2', 'dir2', 'len2', 'speed2', 'urgency2'])
        w.writerows(rows)

    write_configs(args.timeout)

    n2 = sum(1 for r in rows if r[1] == 2)
    ads = [r[9] for r in rows if r[1]]
    print()
    print('시드 %d · 시나리오 %d개 (2명 %d개) + baseline %d = %d개'
          % (args.seed, len(plan), n2, len(rows) - len(plan), len(rows)))
    print('정지선 y %.1f 기준 위치 %s' % (STOPLINE, [int(o) for o in OFFS]))
    print('방향 %s · 길이 %s · 돌발성 %s · 속도 %s'
          % (list(DIRS), list(LENS), list(URG), list(SPEEDS)))
    print('보장집합 위치x돌발x속도 = %d · 전체조합 %d'
          % (len(OFFS) * len(URG) * len(SPEEDS),
             len(OFFS) * len(DIRS) * len(LENS) * len(SPEEDS) * len(URG)))
    print('ego 스폰 (%.2f, %.1f) 교정 · 지면 z %.2f · 자산 %d종'
          % (EGO_START[0], EGO_START[1], GROUND, len(ASSETS)))
    print('activeDistance %.1f~%.1f m' % (min(ads), max(ads)))
    print('manifest: %s' % os.path.join(OUT, '_manifest.csv'))


if __name__ == '__main__':
    main()
