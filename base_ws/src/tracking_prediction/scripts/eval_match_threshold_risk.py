#!/usr/bin/env python3
"""
eval_match_threshold_risk.py

eval_vehicle_spacing.py는 "같은 순간 두 차 간 거리"만 봤는데, 실제
prediction_node.py의 위험은 그게 아니라 "직전 프레임(t-1)의 위치들 중에서
가장 가까운 것"을 현재 프레임(t) 차량과 매칭하는 로직(_estimate_yaw_rate,
MATCH_DIST_THRESHOLD=3.0m) 자체가 엉뚱한 차와 매칭되는지다. 이 스크립트는
그 로직을 그대로 재현해서, 실제로 몇 %가 잘못 매칭되는지 측정한다.

정답(true identity)은 unique_id + 텔레포트 분리로 만든다(다른 스크립트와
동일 기준). prediction_node.py는 이 정답을 모르는 채 "직전 프레임에서
가장 가까운 위치"만으로 추측하므로, 두 방식이 다르면 그게 곧 오매칭이다.

두 가지 rate로 측정한다:
    - native: bag에 실제 기록된 그대로의 프레임 간격(dt)
      (지금 tracking_node.py는 /Object_topic을 다운샘플 없이 그대로 쓰므로,
       "지금 당장의" 위험을 보여줌)
    - downsampled: 가정한 실제 OD rate(기본 10Hz)로 다운샘플한 뒤 측정
      (PointPainting 등 실제 검출기로 교체했을 때의 위험을 보여줌 - dt가
       커질수록 프레임 간 이동거리가 커져서 오매칭 위험이 올라갈 수 있음)

출력 지표(둘 다 "매칭이 실제로 이루어진 경우"만 집계):
    - id_swap_rate: 매칭된 직전 위치가 진짜 자기 자신이 아닌 다른 차인 비율
      (이게 실제 오매칭/오추정 위험)
    - own_not_nearest_rate: 자기 자신의 직전 위치가 프레임 내에 존재했음에도
      그게 최근접이 아니었던 비율 (오매칭의 근본 원인)
    - no_match_rate: 3.0m 이내에 아무 후보도 없어서 omega=0으로 대체된 비율
      (오매칭은 아니지만 정보 손실)

사용법:
    python3 eval_match_threshold_risk.py <bag_path> --label roundabout
    python3 eval_match_threshold_risk.py <bag_path> --label roundabout --od-hz 10
"""

import argparse
import math
from collections import defaultdict

import numpy as np
import rosbag

TELEPORT_THRESHOLD_M = 15.0
MATCH_DIST_THRESHOLD = 3.0  # prediction_node.py와 동일 값


def _read_raw_frames(bag_path, topic="/Object_topic"):
    """frames: [{"t": stamp, "objs": [record_dict, ...]}, ...] (시간순).
    record_dict는 per_uid에도 같은 객체(참조)로 들어가서 나중에 true_id를
    한 번만 채우면 양쪽에 다 반영된다."""
    frames = []
    per_uid = defaultdict(list)
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            objs = []
            for o in msg.npc_list:
                rec = {"uid": o.unique_id, "x": o.position.x, "y": o.position.y}
                objs.append(rec)
                per_uid[o.unique_id].append(rec)
            frames.append({"t": stamp, "objs": objs})
    return frames, per_uid


def _assign_true_ids(per_uid):
    """uid별로 시간순 레코드에 텔레포트 분리 기준 true_id를 부여 (in-place)."""
    for uid, records in per_uid.items():
        if not records:
            continue
        seg_idx = 0
        records[0]["true_id"] = f"{uid}_{seg_idx}"
        for i in range(1, len(records)):
            dist = math.hypot(
                records[i]["x"] - records[i - 1]["x"], records[i]["y"] - records[i - 1]["y"]
            )
            if dist > TELEPORT_THRESHOLD_M:
                seg_idx += 1
            records[i]["true_id"] = f"{uid}_{seg_idx}"


def _downsample_frames(frames, target_hz):
    """target_hz 간격으로 프레임을 골라낸다 (실제 OD가 이 rate로 온다고 가정)."""
    if not frames:
        return frames
    period = 1.0 / target_hz
    out = [frames[0]]
    last_t = frames[0]["t"]
    for f in frames[1:]:
        if f["t"] - last_t >= period:
            out.append(f)
            last_t = f["t"]
    return out


def _nearest_prev(cur_x, cur_y, prev_objs, threshold=MATCH_DIST_THRESHOLD):
    best_true_id, best_dist = None, threshold
    for p in prev_objs:
        d = math.hypot(cur_x - p["x"], cur_y - p["y"])
        if d < best_dist:
            best_dist, best_true_id = d, p["true_id"]
    return best_true_id


def analyze(frames):
    n_matched = 0
    n_id_swap = 0
    n_own_not_nearest = 0
    n_no_match = 0
    n_total_candidates = 0  # 매칭 시도 대상(직전 프레임이 존재하는 경우)

    for i in range(1, len(frames)):
        prev_objs = frames[i - 1]["objs"]
        prev_true_ids = {p["true_id"] for p in prev_objs}
        cur_objs = frames[i]["objs"]

        for cur in cur_objs:
            n_total_candidates += 1
            matched_true_id = _nearest_prev(cur["x"], cur["y"], prev_objs)

            if matched_true_id is None:
                n_no_match += 1
                continue

            n_matched += 1
            if matched_true_id != cur["true_id"]:
                n_id_swap += 1

            # own_not_nearest: 자기 자신의 직전 위치가 존재했는데도 그걸 못 찾은 경우
            if cur["true_id"] in prev_true_ids and matched_true_id != cur["true_id"]:
                n_own_not_nearest += 1

    return {
        "n_total_candidates": n_total_candidates,
        "n_matched": n_matched,
        "n_id_swap": n_id_swap,
        "n_own_not_nearest": n_own_not_nearest,
        "n_no_match": n_no_match,
    }


def print_report(label, stats):
    n = stats["n_total_candidates"]
    print(f"\n[{label}]")
    print(f"  전체 대상: {n}")
    if n == 0:
        print("  (표본 없음)")
        return
    print(f"  매칭 성공: {stats['n_matched']} ({100*stats['n_matched']/n:.2f}%)")
    print(f"  매칭 실패(3.0m 이내 후보 없음, omega=0 대체): {stats['n_no_match']} "
          f"({100*stats['n_no_match']/n:.2f}%)")
    if stats["n_matched"] > 0:
        print(f"  -> ID 스왑률 (매칭 성공 중 엉뚱한 차와 매칭): "
              f"{100*stats['n_id_swap']/stats['n_matched']:.2f}%")
    print(f"  -> 자기 직전 위치가 있었는데도 최근접이 아니었던 비율(전체 대비): "
          f"{100*stats['n_own_not_nearest']/n:.2f}%")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag_path")
    parser.add_argument("--label", default="result")
    parser.add_argument("--od-hz", type=float, default=10.0,
                         help="다운샘플 시 가정할 실제 OD rate (기본 10Hz)")
    args = parser.parse_args()

    frames, per_uid = _read_raw_frames(args.bag_path)
    _assign_true_ids(per_uid)

    print(f"총 프레임 수(native): {len(frames)}")
    if len(frames) >= 2:
        dts = [frames[i]["t"] - frames[i - 1]["t"] for i in range(1, len(frames))]
        print(f"native 평균 dt: {np.mean(dts)*1000:.1f}ms (~{1.0/np.mean(dts):.1f}Hz)")

    native_stats = analyze(frames)
    print_report("native rate (지금 tracking_node.py가 실제로 쓰는 rate)", native_stats)

    ds_frames = _downsample_frames(frames, args.od_hz)
    print(f"\n다운샘플 후 프레임 수({args.od_hz}Hz 가정): {len(ds_frames)}")
    ds_stats = analyze(ds_frames)
    print_report(f"{args.od_hz}Hz OD 가정 시 (실제 검출기로 교체했을 때 위험)", ds_stats)


if __name__ == "__main__":
    main()
