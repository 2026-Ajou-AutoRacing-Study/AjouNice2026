#!/usr/bin/env python3
import csv
import sys
import time

import rospy
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroupArray

COLOR_NAME = {
    TrafficLightElement.RED: "red",
    TrafficLightElement.AMBER: "yellow",
    TrafficLightElement.GREEN: "green",
    TrafficLightElement.UNKNOWN: "unknown",
}
SHAPE_NAME = {
    TrafficLightElement.CIRCLE: "circle",
    TrafficLightElement.LEFT_ARROW: "arrow_left",
    TrafficLightElement.RIGHT_ARROW: "arrow_right",
}


def label_for(elements):
    if not elements:
        return "off"
    parts = []
    for e in elements:
        color = COLOR_NAME.get(e.color, "?")
        shape = SHAPE_NAME.get(e.shape, "?")
        parts.append(f"{color}_{shape}")
    return "+".join(sorted(parts))


def has_impossible_combo(elements):
    has_red_circle = any(
        e.color == TrafficLightElement.RED and e.shape == TrafficLightElement.CIRCLE for e in elements
    )
    has_green_circle = any(
        e.color == TrafficLightElement.GREEN and e.shape == TrafficLightElement.CIRCLE for e in elements
    )
    return has_red_circle and has_green_circle


class LiveVerificationLogger:
    def __init__(self, csv_path, duration_sec):
        self.csv_path = csv_path
        self.duration_sec = duration_sec
        self.start_time = time.time()

        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["wall_time", "elapsed_sec", "gid", "label"])

        self.n_messages = 0
        self.n_impossible_final = 0
        self.impossible_examples = []

        self._current_label = None
        self._current_gid = None
        self._current_streak_start = None
        self.label_streaks = []

        self.sub = rospy.Subscriber(
            "/perception/traffic_light_recognition/traffic_signals",
            TrafficLightGroupArray,
            self.callback,
            queue_size=10,
        )
        rospy.loginfo(f"[live_verification_logger] 구독 시작, {duration_sec if duration_sec else '무제한'}초 동안 기록")

    def callback(self, msg):
        now = time.time()
        elapsed = now - self.start_time

        if self.duration_sec and elapsed > self.duration_sec:
            self.finish()
            rospy.signal_shutdown("측정 시간 종료")
            return

        for group in msg.traffic_light_groups:
            gid = group.traffic_light_group_id
            elements = group.elements
            label = label_for(elements)

            self.n_messages += 1
            self.csv_writer.writerow([now, f"{elapsed:.3f}", gid, label])

            if has_impossible_combo(elements):
                self.n_impossible_final += 1
                if len(self.impossible_examples) < 10:
                    self.impossible_examples.append((elapsed, gid, label))
                rospy.logerr(
                    f"[live_verification_logger] !!경고!! 최종 발행 데이터에 불가능한 조합 발견 "
                    f"(누적 {self.n_impossible_final}회) t={elapsed:.1f}s gid={gid} label={label}"
                )

            key = (gid, label)
            if key != (self._current_gid, self._current_label):
                if self._current_label is not None:
                    streak_dur = now - self._current_streak_start
                    self.label_streaks.append((self._current_gid, self._current_label, streak_dur))
                self._current_gid, self._current_label = gid, label
                self._current_streak_start = now

    def finish(self):
        if self._current_label is not None:
            streak_dur = time.time() - self._current_streak_start
            self.label_streaks.append((self._current_gid, self._current_label, streak_dur))

        self.csv_file.close()

        print()
        print("=" * 60)
        print("=== live_verification_logger 요약 ===")
        print(f"총 메시지 수: {self.n_messages}")
        print(f"CSV 저장 경로: {self.csv_path}")
        print()
        print(f"[체크 1] 최종 발행 데이터에서 불가능한 조합 발견 횟수: {self.n_impossible_final}")
        if self.n_impossible_final > 0:
            print("  -> 안전장치(스무딩+보정)가 완전하지 않음. 예시:")
            for elapsed, gid, label in self.impossible_examples:
                print(f"     t={elapsed:.1f}s gid={gid} label={label}")
        else:
            print("  -> 정상: 스무딩+보정을 거친 뒤에는 불가능한 조합이 한 번도 안 샜음")
        print()
        print(f"[체크 2] 라벨 지속시간(streak) 분포 (스무딩 창 ~0.2~0.25초 기준):")
        long_streaks = [s for s in self.label_streaks if s[2] > 1.0]
        short_streaks = [s for s in self.label_streaks if s[2] <= 1.0]
        print(f"  총 streak 개수: {len(self.label_streaks)}")
        print(f"  1초 이하로 짧게 끝난 것: {len(short_streaks)}개 (노이즈성 가능성)")
        print(f"  1초 넘게 지속된 것: {len(long_streaks)}개 (아래 목록 - MORAI 화면과 대조 필요)")
        for gid, label, dur in sorted(long_streaks, key=lambda x: -x[2])[:20]:
            print(f"     gid={gid} label={label} 지속시간={dur:.1f}초")
        print("=" * 60)


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "verification_log.csv"
    duration_sec = float(sys.argv[2]) if len(sys.argv) > 2 else None

    rospy.init_node("live_verification_logger", anonymous=True)
    logger = LiveVerificationLogger(csv_path, duration_sec)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if not logger.csv_file.closed:
            logger.finish()


if __name__ == "__main__":
    main()
