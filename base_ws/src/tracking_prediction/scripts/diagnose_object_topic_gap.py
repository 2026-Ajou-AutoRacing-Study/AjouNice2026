#!/usr/bin/env python3
"""
diagnose_object_topic_gap.py

extrapolation_node.py 확인 중 발견된 /tracking/objects의 ~2.3초 gap이
EKF 트래커/브리지 문제인지, 아니면 bag에 녹화된 /Object_topic 자체에
이미 그 gap이 있는지 확인한다. bag 파일을 직접 읽어서(ROS 노드 실행 없이)
/Object_topic의 메시지 timestamp 사이 간격을 전부 계산하고, 가장 큰 gap
top 5와 그 시점에 npc_list가 비어있었는지도 같이 보여준다.

사용법:
    python3 diagnose_object_topic_gap.py <bag_path> [--topic /Object_topic]
"""

import argparse

import rosbag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("--topic", default="/Object_topic")
    args = parser.parse_args()

    stamps = []
    npc_counts = []
    with rosbag.Bag(args.bag_path, "r") as bag:
        info = bag.get_type_and_topic_info()
        if args.topic not in info.topics:
            print(f"[에러] {args.topic} 토픽이 이 bag에 없습니다. 존재하는 토픽:")
            for t in info.topics:
                print(f"    {t}")
            return
        n_msgs = info.topics[args.topic].message_count
        print(f"[{args.topic}] 총 메시지 수: {n_msgs}")

        for _, msg, t in bag.read_messages(topics=[args.topic]):
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else t.to_sec()
            stamps.append(stamp)
            npc_counts.append(len(msg.npc_list))

    if len(stamps) < 2:
        print("메시지가 너무 적습니다.")
        return

    gaps = [(stamps[i] - stamps[i - 1], i) for i in range(1, len(stamps))]
    gaps.sort(reverse=True)

    print(f"\n총 구간: {stamps[0]:.3f}s ~ {stamps[-1]:.3f}s (길이 {stamps[-1] - stamps[0]:.3f}s)")
    print(f"평균 발행 간격: {(stamps[-1] - stamps[0]) / (len(stamps) - 1):.4f}s "
          f"(~{(len(stamps) - 1) / (stamps[-1] - stamps[0]):.1f}Hz)")

    print("\n가장 큰 gap top 5:")
    print(f"{'gap(s)':>10} | {'직전 t':>12} | {'직후 t':>12} | {'직전 npc수':>10} | {'직후 npc수':>10}")
    for gap, i in gaps[:5]:
        print(
            f"{gap:>10.3f} | {stamps[i - 1]:>12.3f} | {stamps[i]:>12.3f} | "
            f"{npc_counts[i - 1]:>10d} | {npc_counts[i]:>10d}"
        )

    n_empty = sum(1 for c in npc_counts if c == 0)
    print(f"\nnpc_list가 0개(빈 배열)로 온 메시지 수: {n_empty} / {len(npc_counts)}")


if __name__ == "__main__":
    main()
