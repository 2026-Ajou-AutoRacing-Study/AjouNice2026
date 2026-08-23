#!/usr/bin/env python3
"""Publish generated Base3 scenario Ego and actor positions to RViz."""

import argparse
import csv
import hashlib
from collections import defaultdict
from pathlib import Path


def color_for(name):
    digest = hashlib.sha256(name.encode("utf-8")).digest()
    return tuple(0.35 + 0.60 * value / 255.0 for value in digest[:3])


def load_csv(path):
    with path.open("r", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def load_route(path):
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) >= 3:
                rows.append(tuple(float(value) for value in fields[:3]))
    return rows


def load_generated_route(path):
    return [
        (float(row["x_m"]), float(row["y_m"]), float(row["z_m"]))
        for row in load_csv(path)
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-dir", type=Path, required=True)
    parser.add_argument(
        "--route",
        type=Path,
        help="Optional route override. Defaults to audit-dir/generated_route.csv.",
    )
    parser.add_argument("--frame-id", default="morai_map")
    parser.add_argument("--topic", default="/dataset/base3/generated_coverage")
    args = parser.parse_args()

    import rospy
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    scenes = load_csv(args.audit_dir / "generated_scenes.csv")
    actors = load_csv(args.audit_dir / "generated_actors.csv")
    generated_route_path = args.audit_dir / "generated_route.csv"
    if args.route is not None:
        route = load_route(args.route)
    elif generated_route_path.is_file():
        route = load_generated_route(generated_route_path)
    else:
        route = load_route(
            Path("configs/global_path/2026_molit_comp_global_path.txt")
        )
    grouped_ego = defaultdict(list)
    grouped_actor = defaultdict(list)
    for row in scenes:
        grouped_ego[row["family"]].append(row)
    for row in actors:
        grouped_actor[row["class"]].append(row)

    rospy.init_node("base3_generated_coverage_publisher", anonymous=False)
    publisher = rospy.Publisher(args.topic, MarkerArray, queue_size=1, latch=True)
    result = MarkerArray()
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
    result.markers.append(route_marker)

    marker_id = 1
    for family in sorted(grouped_ego):
        marker = Marker()
        marker.header.frame_id = args.frame_id
        marker.header.stamp = stamp
        marker.ns = "ego_" + family
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.85
        marker.color = ColorRGBA(*color_for(family), 0.92)
        marker.points = [
            Point(x=float(row["ego_x_m"]), y=float(row["ego_y_m"]), z=float(row["ego_z_m"]) + 0.5)
            for row in grouped_ego[family]
        ]
        result.markers.append(marker)

    class_colors = {
        "vehicle": (0.15, 0.55, 1.0),
        "pedestrian": (1.0, 0.25, 0.25),
        "obstacle": (1.0, 0.65, 0.1),
    }
    for class_name in sorted(grouped_actor):
        marker = Marker()
        marker.header.frame_id = args.frame_id
        marker.header.stamp = stamp
        marker.ns = "actor_" + class_name
        marker.id = marker_id
        marker_id += 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.55
        marker.color = ColorRGBA(*class_colors.get(class_name, (0.8, 0.8, 0.8)), 0.9)
        marker.points = [
            Point(x=float(row["x_m"]), y=float(row["y_m"]), z=float(row["z_m"]) + 0.8)
            for row in grouped_actor[class_name]
        ]
        result.markers.append(marker)

    publisher.publish(result)
    rospy.loginfo(
        "Published %d Base3 coverage markers (%d clips, %d actors)",
        len(result.markers), len(scenes), len(actors),
    )
    rospy.spin()


if __name__ == "__main__":
    main()
