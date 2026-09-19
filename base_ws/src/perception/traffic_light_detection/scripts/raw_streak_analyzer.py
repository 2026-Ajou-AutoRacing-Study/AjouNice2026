#!/usr/bin/env python3
"""
raw_streak_analyzer.py

traffic_light_node v14가 새로 발행하는 ~debug_raw_label(스무딩 전 raw 라벨) 토픽을
구독해서, 같은 라벨이 몇 "프레임" 연속으로 유지되는지 실측하는 도구.
스무딩 파라미터(window_size, majority_count) 재조정의 직접적인 근거 데이터를 만듦.

사용법:
  python3 raw_streak_analyzer.py [csv_경로] [초]
"""
import csv
import sys
import time

import rospy
from std_msgs.msg import String


class RawStreakAnalyzer:
    def __init__(self, csv_path, duration_sec):
        self.csv_path = csv_path
        self.duration_sec = duration_sec
        self.start_time = time.time()

        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["elapsed_sec", "raw_label"])

        self._prev_label = None
        self._streak_len = 0
        self.streak_lengths = []  # 프레임 수 단위
        self.n_frames = 0

        rospy.Subscriber("/traffic_light_node/debug_raw_label", String, self.callback, queue_size=50)
        rospy.loginfo(f"[raw_streak_analyzer] 구독 시작, {duration_sec}초 동안 기록")

    def callback(self, msg):
        elapsed = time.time() - self.start_time
        if self.duration_sec and elapsed > self.duration_sec:
            self.finish()
            rospy.signal_shutdown("측정 종료")
            return

        label = msg.data
        self.n_frames += 1
        self.writer.writerow([f"{elapsed:.3f}", label])

        if label == self._prev_label:
            self._streak_len += 1
        else:
            if self._prev_label is not None:
                self.streak_lengths.append((self._prev_label, self._streak_len))
            self._prev_label = label
            self._streak_len = 1

    def finish(self):
        if self._prev_label is not None:
            self.streak_lengths.append((self._prev_label, self._streak_len))
        self.csv_file.close()

        print()
        print("=" * 60)
        print("=== raw_streak_analyzer 요약 ===")
        print(f"총 프레임 수: {self.n_frames}")
        print(f"총 streak(연속 구간) 개수: {len(self.streak_lengths)}")
        print()

        # 프레임 길이별 분포 (1프레임짜리가 얼마나 되는지가 핵심)
        from collections import Counter
        len_counter = Counter(l for _lbl, l in self.streak_lengths)
        print("=== streak 길이(프레임 수)별 분포 ===")
        for length in sorted(len_counter.keys())[:20]:
            count = len_counter[length]
            print(f"  {length}프레임 지속: {count}번")

        print()
        n_1frame = len_counter.get(1, 0)
        n_2frame = len_counter.get(2, 0)
        total = len(self.streak_lengths)
        print(f"1프레임짜리(가장 순간적인 노이즈): {n_1frame}개 / 전체 {total}개 ({100*n_1frame/total:.0f}%)" if total else "데이터 없음")
        print(f"2프레임 이하: {n_1frame+n_2frame}개 / 전체 {total}개 ({100*(n_1frame+n_2frame)/total:.0f}%)" if total else "")
        print()
        print("=== 가장 길게 지속된 streak 상위 10개 (라벨, 프레임수) ===")
        for lbl, length in sorted(self.streak_lengths, key=lambda x: -x[1])[:10]:
            print(f"  {lbl}: {length}프레임")
        print("=" * 60)
        print()
        print("판단 가이드: 1~2프레임짜리가 대다수면 지금 window=5,majority=3도 충분.")
        print("3프레임 이상 지속되는 '틀린' streak가 많다면 window를 늘리는 게 필요할 수 있음")
        print("(단, 이 스크립트는 '틀렸는지'는 판단 안 함 - 사람이 라벨 내용 보고 판단 필요)")


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "raw_streak_log.csv"
    duration_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    rospy.init_node("raw_streak_analyzer", anonymous=True)
    analyzer = RawStreakAnalyzer(csv_path, duration_sec)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if not analyzer.csv_file.closed:
            analyzer.finish()


if __name__ == "__main__":
    main()
