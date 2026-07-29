#!/usr/bin/env python3
"""
find_roundabout_and_filter.py

로터리 중심 좌표를 좌표값을 직접 안 외우고 자동으로 찾는다:
    "yaw_rate가 큰(=실제로 도는) 지점들"의 중심 = 로터리 중심일 가능성이 높음.
    (직선/하이웨이 구간에서는 yaw_rate가 거의 0이라 안 잡힘)

그다음, 각 객체(unique_id)의 전체 궤적 중 단 한 순간이라도 로터리 중심에서
반경 R(m) 이내로 들어온 적이 있으면 "로터리 관련 차량"으로 유지하고,
평생 한 번도 안 들어온 적이 있으면(=순수 하이웨이/직선 구간만 다님)
"하이웨이쪽 차량"으로 간주해서 제외 대상 목록에 올린다.

이 스크립트는 필터링 대상 unique_id 목록만 출력한다. 실제 평가 스크립트에서
이 목록을 --exclude-ids 로 넘겨서 제외하면 된다.

사용법:
    python3 find_roundabout_and_filter.py <bag_path> --radius 60
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

MORAI_HEADING_IN_DEGREES = True
TURN_THRESHOLD_RAD_PER_SEC = math.radians(5.0)  # 이 이상이면 "도는 중"으로 판단


def _heading_to_rad(heading):
    return math.radians(heading) if MORAI_HEADING_IN_DEGREES else heading


def _extract_object_series(bag_path, topic="/Object_topic"):
    series = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            for obj in msg.npc_list:
                series[obj.unique_id].append(
                    {"t": stamp, "x": obj.position.x, "y": obj.position.y, "heading": _heading_to_rad(obj.heading)}
                )
    for uid in series:
        series[uid].sort(key=lambda r: r["t"])
    return series


def find_roundabout_center(series):
    """yaw_rate가 큰 지점들의 중심 좌표를 계산."""
    turn_xs, turn_ys = [], []
    for uid, records in series.items():
        for i in range(1, len(records)):
            prev, cur = records[i - 1], records[i]
            dt = cur["t"] - prev["t"]
            if dt <= 0:
                continue
            dtheta = cur["heading"] - prev["heading"]
            dtheta = (dtheta + math.pi) % (2 * math.pi) - math.pi
            omega = dtheta / dt
            if abs(omega) > TURN_THRESHOLD_RAD_PER_SEC:
                turn_xs.append(cur["x"])
                turn_ys.append(cur["y"])

    if not turn_xs:
        return None, None, 0

    return np.mean(turn_xs), np.mean(turn_ys), len(turn_xs)


def filter_highway_only_ids(series, center_x, center_y, radius):
    """반경 안에 한 번도 안 들어온 unique_id 목록(=하이웨이쪽 차량으로 간주) 반환."""
    highway_only_ids = []
    roundabout_ids = []
    for uid, records in series.items():
        ever_near = any(
            math.hypot(r["x"] - center_x, r["y"] - center_y) <= radius for r in records
        )
        if ever_near:
            roundabout_ids.append(uid)
        else:
            highway_only_ids.append(uid)
    return roundabout_ids, highway_only_ids


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path")
    parser.add_argument("--radius", type=float, default=60.0, help="로터리 중심 반경(m), 기본 60m")
    args = parser.parse_args()

    series = _extract_object_series(args.bag_path)
    print(f"전체 unique_id 수: {len(series)}")

    cx, cy, n_turn_points = find_roundabout_center(series)
    if cx is None:
        print("회전 구간(yaw_rate 큰 지점)을 하나도 못 찾았습니다. 이 bag엔 로터리가 없을 수 있습니다.")
        return

    print(f"자동 탐지된 로터리 중심: x={cx:.1f}, y={cy:.1f} (회전 샘플 {n_turn_points}개 기준)")

    roundabout_ids, highway_only_ids = filter_highway_only_ids(series, cx, cy, args.radius)

    print(f"\n반경 {args.radius}m 이내로 한 번이라도 들어온 적 있는 차량(로터리 관련): {len(roundabout_ids)}대")
    print(f"반경 밖에서만 계속 다닌 차량(하이웨이쪽, 제외 대상): {len(highway_only_ids)}대")
    print(f"\n제외할 unique_id 목록:")
    print(",".join(str(i) for i in sorted(highway_only_ids)))


if __name__ == "__main__":
    main()
