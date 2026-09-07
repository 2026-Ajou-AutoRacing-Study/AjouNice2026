#!/usr/bin/env python3
"""Publish a completed temporal-dataset Ego coverage audit to RViz."""

from __future__ import annotations

import argparse
import csv
import hashlib
from collections import defaultdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_AUDIT = REPO_ROOT / "artifacts/datasets/morai_temporal_base2_10k_audit"
DEFAULT_ROUTE = REPO_ROOT / "configs/global_path/2026_molit_comp_global_path.txt"


def color_for(name: str):
    digest = hashlib.sha256(name.encode("utf-8")).digest()
    # Keep colors bright enough on RViz's dark background.
    return tuple(0.35 + 0.60 * channel / 255.0 for channel in digest[:3])


def load_rows(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def load_route(path: Path):
    points = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) >= 3:
                points.append(tuple(float(value) for value in fields[:3]))
    return points


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-dir", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument("--frame-id", default="morai_map")
    parser.add_argument("--topic", default="/dataset/base2_10k/ego_coverage")
    parser.add_argument("--point-size", type=float, default=0.75)
    args = parser.parse_args()

    import rospy
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    rows = load_rows(args.audit_dir / "ego_trajectory_points.csv")
    route = load_route(args.route)
    by_family = defaultdict(list)
    for row in rows:
        by_family[row["family"]].append(row)

    rospy.init_node("temporal_dataset_coverage_publisher", anonymous=False)
    publisher = rospy.Publisher(args.topic, MarkerArray, queue_size=1, latch=True)
    markers = MarkerArray()
    stamp = rospy.Time.now()

    route_marker = Marker()
    route_marker.header.frame_id = args.frame_id
    route_marker.header.stamp = stamp
    route_marker.ns = "competition_route"
    route_marker.id = 0
    route_marker.type = Marker.LINE_STRIP
    route_marker.action = Marker.ADD
    route_marker.pose.orientation.w = 1.0
    route_marker.scale.x = 0.25
    route_marker.color = ColorRGBA(1.0, 1.0, 1.0, 0.85)
    route_marker.points = [Point(x=x, y=y, z=z + 0.15) for x, y, z in route]
    markers.markers.append(route_marker)

    for marker_id, family in enumerate(sorted(by_family), start=1):
        marker = Marker()
        marker.header.frame_id = args.frame_id
        marker.header.stamp = stamp
        marker.ns = "clip_ego_" + family
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = args.point_size
        marker.scale.y = args.point_size
        red, green, blue = color_for(family)
        marker.color = ColorRGBA(red, green, blue, 0.85)
        marker.points = [Point(x=float(row["x_m"]), y=float(row["y_m"]), z=29.2)
                         for row in by_family[family]]
        markers.markers.append(marker)

    publisher.publish(markers)
    rospy.loginfo("Published %d coverage markers (%d Ego samples) to %s",
                  len(markers.markers), len(rows), args.topic)
    rospy.spin()


if __name__ == "__main__":
    main()
