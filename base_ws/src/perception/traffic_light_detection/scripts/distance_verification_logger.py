#!/usr/bin/env python3
"""
distance_verification_logger.py

접근 거리별로 신호등 판정이 언제부터 안정적으로 정확해지는지 측정하는 로거.
ego 위치와 신호등 전구 좌표(오늘 GO_OUT 계산에 쓰는 것과 동일한 좌표)로 거리를
계산해서, gid별로 "라벨이 안정화되는 거리"를 실측함.

사용법 (다른 노드 다 띄운 상태에서):
  python3 distance_verification_logger.py <lanelet2_map_file> <projector_file> [csv경로] [시간초]
"""
import csv
import sys
import time
import xml.etree.ElementTree as ET

import rospy
from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroupArray

try:
    from lanelet2 import core, io, projection
    LANELET2_AVAILABLE = True
except ImportError:
    LANELET2_AVAILABLE = False

import yaml
import math

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


def load_bulb_positions(map_file, projector_file):
    """각 신호등 group_id의 전구 위치(x,y)를 미리 로드."""
    with open(projector_file, "r") as f:
        config = yaml.safe_load(f)
    origin = io.Origin(config["map_origin"]["latitude"], config["map_origin"]["longitude"])
    projector = projection.UtmProjector(origin, True, False)

    tree = ET.parse(map_file)
    root = tree.getroot()
    nodes = {nd.get("id"): (float(nd.get("lat")), float(nd.get("lon"))) for nd in root.findall("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}

    positions = {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("subtype") != "traffic_light":
            continue
        rid = rel.get("id")
        way_ids = [m.get("ref") for m in rel.findall("member") if m.get("role") == "refers"]
        pts = []
        for wid in way_ids:
            for nref in ways.get(wid, []):
                if nref in nodes:
                    pts.append(nodes[nref])
        if not pts:
            continue
        lat = sum(p[0] for p in pts) / len(pts)
        lon = sum(p[1] for p in pts) / len(pts)
        gps = core.GPSPoint(lat, lon, 0.0)
        p = projector.forward(gps)
        positions[int(rid)] = (p.x, p.y)
    return positions


class DistanceVerificationLogger:
    def __init__(self, bulb_positions, csv_path, duration_sec):
        self.bulb_positions = bulb_positions
        self.csv_path = csv_path
        self.duration_sec = duration_sec
        self.start_time = time.time()
        self.ego_xy = None

        self.csv_file = open(csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["elapsed_sec", "gid", "distance_m", "label"])

        self.n_rows = 0
        # gid별로: 마지막으로 "정상적인"(무효조합 아닌) 라벨이 계속 유지되기 시작한 거리
        self.stabilized_distance = {}  # gid -> 첫 안정화 거리
        self._prev_label = {}

        rospy.Subscriber("/localization/kinematic_state", Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(
            "/perception/traffic_light_recognition/traffic_signals",
            TrafficLightGroupArray, self._signal_cb, queue_size=10,
        )
        rospy.loginfo(f"[distance_verification_logger] 구독 시작, {duration_sec}초 동안 기록")

    def _odom_cb(self, msg):
        self.ego_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _signal_cb(self, msg):
        elapsed = time.time() - self.start_time
        if self.duration_sec and elapsed > self.duration_sec:
            self.finish()
            rospy.signal_shutdown("측정 종료")
            return
        if self.ego_xy is None:
            return

        ex, ey = self.ego_xy
        for group in msg.traffic_light_groups:
            gid = group.traffic_light_group_id
            label = label_for(group.elements)

            if gid in self.bulb_positions:
                bx, by = self.bulb_positions[gid]
                dist = math.hypot(ex - bx, ey - by)
            else:
                dist = -1.0

            self.writer.writerow([f"{elapsed:.3f}", gid, f"{dist:.1f}", label])
            self.n_rows += 1

    def finish(self):
        self.csv_file.close()
        print()
        print("=" * 60)
        print(f"총 {self.n_rows}행 기록, CSV: {self.csv_path}")
        print("=" * 60)
        print("이제 CSV를 열어서, gid별로 거리(distance_m)가 줄어들면서")
        print("label이 언제부터 안정적으로 정상 패턴을 유지하는지 확인하세요.")
        print("(정지선까지 남은 거리와 비교해서, 차량 제동에 충분한 거리인지 판단)")


def main():
    if len(sys.argv) < 3:
        print("사용법: python3 distance_verification_logger.py <map_file> <projector_file> [csv] [초]")
        sys.exit(1)
    map_file, projector_file = sys.argv[1], sys.argv[2]
    csv_path = sys.argv[3] if len(sys.argv) > 3 else "distance_log.csv"
    duration_sec = float(sys.argv[4]) if len(sys.argv) > 4 else 180.0

    if not LANELET2_AVAILABLE:
        print("lanelet2 python bindings 없음")
        sys.exit(1)

    rospy.init_node("distance_verification_logger", anonymous=True)
    bulb_positions = load_bulb_positions(map_file, projector_file)
    rospy.loginfo(f"[distance_verification_logger] 전구 위치 {len(bulb_positions)}개 로드")

    logger = DistanceVerificationLogger(bulb_positions, csv_path, duration_sec)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if not logger.csv_file.closed:
            logger.finish()


if __name__ == "__main__":
    main()
