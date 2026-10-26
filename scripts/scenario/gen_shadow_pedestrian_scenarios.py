#!/usr/bin/env python3
"""GPS 음영 터널 보행자 시나리오 생성기 (AjouNice2026).

시드가 같으면 누가 돌려도 바이트 단위로 동일한 시나리오가 나온다.
    python3 scripts/scenario/gen_shadow_pedestrian_scenarios.py
    python3 scripts/scenario/gen_shadow_pedestrian_scenarios.py --count 300 --seed 20261027
"""
import argparse
import csv
import json
import math
import os
import random
import shutil

ROOT = os.environ.get('AJOUNICE_ROOT', os.path.expanduser('~/AjouNice2026'))
MISSION = os.path.join(
    ROOT, 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow')
SRC = os.path.join(MISSION, 'base/scenarios')
OUT = os.path.join(MISSION, 'workspace/scenarios')
WS = os.path.join(MISSION, 'workspace')

# ── 음영 회랑 기하 (경로 × shadedAreaList 교집합에서 측정)
ENTRY = (27.08, -489.33)
EXIT = (-58.25, -535.82)
NCELL = 15
# ── 터널 횡단면 (Ego 차로 중심 기준, +n = 옆 차로 쪽)
FAR, NEAR, LANEB = 5.2, -1.8, 3.45
HALF_LANE = 1.73
GROUND = [28.34, 28.45, 28.42]          # 구간별 지면 고도
SPAWN = (53.07, -466.53)
V_EGO = 14.0                            # 터널 접근 실측 12~16 m/s
MIN_GAP = 15.0                          # 보행자 2명일 때 종방향 보행구간 최소 간격

# ── 보행자 속도: MORAI speed 필드는 m/s가 아니라 약 ×0.16
SPEEDS = {5: 0.80, 10: 1.53, 13: 2.00, 19: 3.00}
# ── 종방향 이동 길이 (L/D 패턴에만 적용)
LONS = {'s': 14.0, 'm': 40.0, 'l': 70.0}
# ── 자산: 카탈로그의 person 계열 enabled만
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
CROSS = ('C_BA', 'C_AB', 'C_ABh')
PATTERNS = ('C_BA', 'C_AB', 'C_ABh', 'D_fwd', 'D_bwd', 'L_A', 'L_B')
ROT = {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"}

DX, DY = EXIT[0] - ENTRY[0], EXIT[1] - ENTRY[1]
CORR = math.hypot(DX, DY)
UX, UY = DX / CORR, DY / CORR
NX, NY = -UY, UX
CELL = CORR / NCELL


def unity(x, y, z):
    return {"x": -x, "y": z, "z": -y,
            "_x": "%.3f" % -x, "_y": "%.3f" % z, "_z": "%.3f" % -y}


def world(x, y, z):
    return {"x": x, "y": y, "z": z,
            "_x": "%.3f" % x, "_y": "%.3f" % y, "_z": "%.3f" % z}


def at(along, lat):
    return (ENTRY[0] + UX * along + NX * lat, ENTRY[1] + UY * along + NY * lat)


def endpoints(cell, pat, lon_key):
    """패턴별 시작·끝 (along, lat)."""
    c = (cell + 0.5) * CELL
    if pat in CROSS:
        lo = hi = c
    else:
        half = LONS[lon_key] / 2.0
        lo, hi = c - half, c + half
        if lo < 2.0:
            lo, hi = 2.0, 2.0 + LONS[lon_key]
        if hi > CORR - 2.0:
            hi, lo = CORR - 2.0, CORR - 2.0 - LONS[lon_key]
        lo = max(lo, 1.0)
    if pat == 'C_BA':
        return (lo, FAR), (hi, NEAR)
    if pat in ('C_AB', 'C_ABh'):
        return (lo, NEAR), (hi, FAR)
    if pat == 'D_fwd':
        return (lo, FAR), (hi, NEAR)
    if pat == 'D_bwd':
        return (hi, FAR), (lo, NEAR)
    if pat == 'L_A':
        return (lo, 0.0), (hi, 0.0)
    return (lo, LANEB), (hi, LANEB)


def active_distance(cell, pat, spd_key):
    """보행자가 Ego 차로를 점유하는 한가운데에 Ego가 도달하도록 역산."""
    v = SPEEDS[spd_key]
    if pat == 'C_ABh':
        ad = 11.0
    elif pat in ('C_AB',):
        ad = V_EGO * ((abs(NEAR) + HALF_LANE) / 2.0) / v
    elif pat in ('C_BA', 'D_fwd', 'D_bwd'):
        ad = V_EGO * (FAR / v)
    else:
        ad = 40.0
    # 스폰에서 보행자까지보다 크면 로드 즉시 켜져 대기 중에 다 걸어가 버린다
    s, _ = endpoints(cell, pat, 's')
    px, py = at(s[0], s[1])
    room = math.hypot(px - SPAWN[0], py - SPAWN[1]) - 10.0
    return round(min(max(ad, 8.0), max(room, 8.0)), 1)


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
    rest = [c for c in full if c not in must]
    rng.shuffle(rest)
    plan = must + rest[:max(0, count - len(must))]
    rng.shuffle(plan)
    return plan


def place(ped, cell, pat, spd, lon, asset):
    (a0, l0), (a1, l1) = endpoints(cell, pat, lon)
    gz = GROUND[min(cell // 5, 2)]
    s, e = at(a0, l0), at(a1, l1)
    ped['DataID'] = asset[0]
    ped.update(speed=float(spd), activeDistance=active_distance(cell, pat, spd),
               active=True, loop=False, isWaypointMode=True)
    ped['pos'] = world(s[0], s[1], gz)
    ped['initPos'] = unity(s[0], s[1], gz)
    ped['standardPos'] = unity(s[0], s[1], gz)
    path = {"pathName": "PedPath - %d" % ped['UNIQUEID'],
            "waypointData": [{"pos": unity(s[0], s[1], gz), "rot": dict(ROT)},
                             {"pos": unity(e[0], e[1], gz), "rot": dict(ROT)}]}
    return path, s, e, math.hypot(e[0] - s[0], e[1] - s[1])


def span(cell, pat, lon):
    """보행자가 점유하는 종방향 구간 [lo, hi]."""
    (a0, _), (a1, _) = endpoints(cell, pat, lon)
    return (min(a0, a1), max(a0, a1))


def write_configs(timeout):
    try:
        import yaml
    except ImportError:
        print('[skip] PyYAML 없음 — 캠페인 YAML은 수동으로 만드세요')
        return
    for base, out in (('localization_gt_pose.yaml', 'sp_set_gt.yaml'),
                      ('localization_baseline.yaml', 'sp_set_base.yaml')):
        p = os.path.join(MISSION, 'base', base)
        if not os.path.exists(p):
            continue
        d = yaml.safe_load(open(p))
        d['scenarios'].pop('include', None)
        d['scenarios']['directory'] = os.path.relpath(OUT, ROOT)
        d['scenarios']['glob'] = 'sq*.json'
        d['scenarios']['repeat'] = 1
        d['default_case']['timeout_sec'] = timeout
        # goal은 base YAML 값을 그대로 쓴다. 임의 좌표로 단축하면 ROUTE_FAILURE.
        # 충돌은 의도한 결과이므로 캠페인을 멈추지 않는다.
        s = d.setdefault('validation', {}).setdefault('localization_safety', {})
        s['enabled'] = True
        s['stop_on_collision'] = False
        v = d.setdefault('recording', {}).setdefault('video', {})
        v.update(enabled=True, display=':1', fps=10, window_layout='split')
        yaml.safe_dump(d, open(os.path.join(WS, out), 'w'),
                       allow_unicode=True, sort_keys=False)
        print('  config  %s' % out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--count', type=int, default=300)
    ap.add_argument('--seed', type=int, default=20261027)
    ap.add_argument('--two-ratio', type=float, default=0.35)
    ap.add_argument('--timeout', type=float, default=180.0)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        if f.startswith('sq') or f.startswith('sp_') or f == '_manifest.csv':
            os.remove(os.path.join(OUT, f))

    # 1명/2명 템플릿을 따로 쓴다. 한 항목을 복제하면 MORAI가 별개 객체로 보지 않는다.
    tmpl1 = json.load(open(os.path.join(SRC, 'hid_soa_normal_10.json')))
    tmpl2 = json.load(open(os.path.join(SRC, 'hid_soa_normal_09.json')))
    plan = build_plan(args.count, rng)
    two = set(rng.sample(range(len(plan)), int(len(plan) * args.two_ratio)))

    rows = []
    for idx, (cell, pat, spd, lon) in enumerate(plan, 1):
        want_two = (idx - 1) in two
        lo1, hi1 = span(cell, pat, lon)
        second = None
        if want_two:
            # 앵커가 아니라 실제 보행구간이 MIN_GAP 이상 떨어져야 한다.
            for _ in range(80):
                c2 = rng.randrange(NCELL)
                p2 = rng.choice(PATTERNS)
                l2 = 'x' if p2 in CROSS else rng.choice(list(LONS))
                s2k = rng.choice(list(SPEEDS))
                lo2, hi2 = span(c2, p2, l2)
                if max(lo1, lo2) - min(hi1, hi2) >= MIN_GAP:
                    second = (c2, p2, s2k, l2)
                    break
        n = 2 if second else 1
        d = json.loads(json.dumps(tmpl2 if n == 2 else tmpl1))
        d['vehicleList'] = []
        d['pedestrianList'] = d['pedestrianList'][:n]
        a1 = ASSETS[rng.randrange(len(ASSETS))]
        w1, s1, e1, d1 = place(d['pedestrianList'][0], cell, pat, spd, lon, a1)
        paths = [w1]
        if n == 2:
            c2, p2, s2k, l2 = second
            a2 = ASSETS[rng.randrange(len(ASSETS))]
            w2, _, _, _ = place(d['pedestrianList'][1], c2, p2, s2k, l2, a2)
            paths.append(w2)
            second = (c2 + 1, p2, s2k, l2, a2[1])
        d['waypointDataList'] = paths
        nm = 'sq%03d_%s_c%02d_v%02d_%s%s' % (idx, pat, cell + 1, spd, lon, '_x2' if n == 2 else '')
        json.dump(d, open(os.path.join(OUT, nm + '.json'), 'w'), ensure_ascii=False)
        rows.append([nm, n, cell + 1, pat, spd, round(SPEEDS[spd], 2), lon,
                     d['pedestrianList'][0]['activeDistance'], a1[1], round(d1, 1),
                     *(second if second else ('', '', '', '', ''))])

    for i, src in enumerate(['hid_soa_baseline_00', 'hid_soa_baseline_01', 'hid_soa_normal_11']):
        p = os.path.join(SRC, src + '.json')
        if os.path.exists(p):
            shutil.copy(p, os.path.join(OUT, 'sq_base%d.json' % i))
            rows.append(['sq_base%d' % i, 0, '', '', '', '', '', '', '', '', '', '', '', '', ''])

    with open(os.path.join(OUT, '_manifest.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['name', 'ped', 'cell', 'pattern', 'speed_field', 'speed_mps', 'lon',
                    'active_distance', 'asset', 'walk_m',
                    'cell2', 'pattern2', 'speed2', 'lon2', 'asset2'])
        w.writerows(rows)

    write_configs(args.timeout)

    n2 = sum(1 for r in rows if r[1] == 2)
    print()
    print('시드 %d · 시나리오 %d개 (2명 %d개) + baseline 3 = %d개'
          % (args.seed, len(plan), n2, len(rows)))
    print('칸 %d개 × %.2f m · 회랑 %.1f m · 패턴 %d · 속도 %s · 종길이 %s'
          % (NCELL, CELL, CORR, len(PATTERNS), list(SPEEDS), list(LONS)))
    print('자산 %d종 (카탈로그 person/enabled) · 2명 최소 종간격 %.0f m'
          % (len(ASSETS), MIN_GAP))
    print('goal은 base YAML 값 유지 (임의 단축 시 ROUTE_FAILURE)')
    print('manifest: %s' % os.path.join(OUT, '_manifest.csv'))


if __name__ == '__main__':
    main()
