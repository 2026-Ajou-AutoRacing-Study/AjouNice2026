#!/usr/bin/env python3
"""
measure_recognition_distance.py (v2 - 거리별 스크린샷 캡처 추가)

차량이 정지선에 접근하는 동안 "정지선까지 거리" + "인식 성공 여부"를 CSV로 기록하고,
동시에 지정한 거리(체크포인트)를 지날 때마다 디버그 이미지(박스+라벨)를 자동 저장.

사용법:
  1) 차량을 정지선에 정확히 세운 뒤, 그 위치를 기준점(원점)으로 캡처해서 STOP_X, STOP_Y에 입력
     (rostopic echo /Ego_topic -n 1 로 position.x/y 확인)
  2) 차량을 신호등이 안 보일 만큼 멀리 이동 (60~80m 이상 권장)
  3) 이 스크립트 실행 후, 차량을 일정한 저속으로 정지선까지(살짝 지나칠 때까지) 전진
  4) Ctrl+C로 종료하면 CSV 저장 + 인식 끊기는 지점 요약 + 거리별 캡처 이미지들 출력

예시:
  python3 measure_recognition_distance.py \
    --stop_x -16.9269 --stop_y 98.7547 \
    --out result_section1.csv \
    --capture_dir captures_section1 \
    --checkpoints 70,60,56,50,40,30,20,15,10,8,6,5.8,5,3

토픽:
  구독: /Ego_topic (morai_msgs/EgoVehicleStatus) - position.x/y 사용
  구독: /perception/traffic_light_recognition/traffic_signals (autoware_perception_msgs/TrafficLightGroupArray)
  구독: /traffic_light_node/debug_image/compressed (sensor_msgs/CompressedImage) - 캡처용
"""
import argparse
import csv
import math
import os
import time

import rospy
from morai_msgs.msg import EgoVehicleStatus
from autoware_perception_msgs.msg import TrafficLightGroupArray
from sensor_msgs.msg import CompressedImage


class DistanceLogger:
    def __init__(self, stop_x, stop_y, out_csv, capture_dir, checkpoints):
        self.stop_x = stop_x
        self.stop_y = stop_y
        self.out_csv = out_csv
        self.capture_dir = capture_dir
        # 먼 거리 -> 가까운 거리 순으로 정렬, 각 체크포인트 한 번씩만 캡처
        self.checkpoints = sorted(checkpoints, reverse=True)
        self.captured = set()

        os.makedirs(self.capture_dir, exist_ok=True)

        self.latest_recognized = False
        self.latest_num_elements = 0
        self.latest_image_bytes = None

        self.rows = []

        self.sub_ego = rospy.Subscriber(
            "/Ego_topic", EgoVehicleStatus, self.ego_callback, queue_size=1
        )
        self.sub_signal = rospy.Subscriber(
            "/perception/traffic_light_recognition/traffic_signals",
            TrafficLightGroupArray,
            self.signal_callback,
            queue_size=1,
        )
        self.sub_image = rospy.Subscriber(
            "/traffic_light_node/debug_image/compressed",
            CompressedImage,
            self.image_callback,
            queue_size=1,
        )

        self._t0 = time.time()
        rospy.loginfo(
            f"[measure] 기준점(정지선) = ({stop_x:.2f}, {stop_y:.2f}) / 출력: {out_csv}"
        )
        rospy.loginfo(f"[measure] 캡처 체크포인트(m): {self.checkpoints}")
        rospy.loginfo("[measure] 차량을 이제 서서히 정지선 쪽으로 전진시키세요. Ctrl+C로 종료 및 저장.")

    def signal_callback(self, msg):
        total_elements = sum(len(g.elements) for g in msg.traffic_light_groups)
        self.latest_recognized = total_elements > 0
        self.latest_num_elements = total_elements

    def image_callback(self, msg):
        self.latest_image_bytes = msg.data

    def _maybe_capture(self, dist):
        # 아직 캡처 안 한 체크포인트 중, 현재 거리가 그 체크포인트 이하로 내려온 첫 순간에 캡처
        for cp in self.checkpoints:
            if cp in self.captured:
                continue
            if dist <= cp:
                self._save_capture(cp, dist)
                self.captured.add(cp)

    def _save_capture(self, checkpoint_m, actual_dist):
        if self.latest_image_bytes is None:
            rospy.logwarn(f"[measure] {checkpoint_m}m 체크포인트: 아직 디버그 이미지 없음, 캡처 스킵")
            return
        tag = "REC" if self.latest_recognized else "NO"
        fname = f"{checkpoint_m:05.1f}m_actual{actual_dist:.2f}m_{tag}.jpg"
        fpath = os.path.join(self.capture_dir, fname)
        with open(fpath, "wb") as f:
            f.write(self.latest_image_bytes)
        rospy.loginfo(f"[measure] 캡처 저장: {fpath} (인식={tag})")

    def ego_callback(self, msg):
        ex, ey = msg.position.x, msg.position.y
        dist = math.hypot(ex - self.stop_x, ey - self.stop_y)
        t = time.time() - self._t0
        self.rows.append(
            (round(t, 3), round(dist, 3), int(self.latest_recognized),
             self.latest_num_elements, round(ex, 3), round(ey, 3))
        )
        self._maybe_capture(dist)

    def save_and_summarize(self):
        if not self.rows:
            rospy.logwarn("[measure] 기록된 데이터가 없습니다.")
            return

        with open(self.out_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_sec", "dist_to_stopline_m", "recognized", "num_elements", "ego_x", "ego_y"])
            writer.writerows(self.rows)

        rows_by_time = sorted(self.rows, key=lambda r: r[0])
        first_recognized = next((r for r in rows_by_time if r[2] == 1), None)
        last_recognized = None
        for r in rows_by_time:
            if r[2] == 1:
                last_recognized = r

        print("\n===== 요약 =====")
        if first_recognized:
            print(f"원거리 첫 인식 지점: {first_recognized[1]:.2f} m")
        else:
            print("전체 구간에서 인식 성공 기록 없음")
        if last_recognized:
            print(f"근거리 마지막 인식 지점: {last_recognized[1]:.2f} m")

        print(f"\nCSV 저장 완료: {self.out_csv} (총 {len(self.rows)}행)")
        print(f"캡처 이미지 저장 폴더: {self.capture_dir} (체크포인트 {len(self.captured)}/{len(self.checkpoints)}개 캡처됨)")
        missed = [c for c in self.checkpoints if c not in self.captured]
        if missed:
            print(f"주의: 못 지나간(캡처 안 된) 체크포인트: {missed} - 주행 범위가 부족했을 수 있음")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stop_x", type=float, required=True)
    parser.add_argument("--stop_y", type=float, required=True)
    parser.add_argument("--out", type=str, default="traffic_light_recognition_distance.csv")
    parser.add_argument("--capture_dir", type=str, default="captures")
    parser.add_argument(
        "--checkpoints", type=str,
        default="70,60,56,50,40,30,20,15,10,8,6,5.8,5,3",
        help="쉼표로 구분된 캡처할 거리(m) 목록"
    )
    args, _ = parser.parse_known_args()

    checkpoints = [float(x) for x in args.checkpoints.split(",")]

    rospy.init_node("measure_recognition_distance")
    logger = DistanceLogger(args.stop_x, args.stop_y, args.out, args.capture_dir, checkpoints)
    rospy.on_shutdown(logger.save_and_summarize)
    rospy.spin()


if __name__ == "__main__":
    main()
