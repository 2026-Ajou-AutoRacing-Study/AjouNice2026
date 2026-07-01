#!/usr/bin/env python3
#
# Copyright 2026 AjouNice2026 contributors.
#
# Licensed under the Apache License, Version 2.0.
#
# Publish MORAI MGeo JSON geometry as RViz markers in the MORAI local map frame.
# The static map -> morai_map TF is intentionally kept outside this node so map
# alignment can be inspected by overlaying these markers with the Lanelet map.

import ast
import hashlib
import json
import math
import os
from collections import Counter

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, low, high):
    return max(low, min(high, value))


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_hash_table(value):
    if isinstance(value, dict):
        return value
    if not value:
        return {}
    try:
        parsed = ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


def color(r, g, b, a, alpha_scale=1.0):
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = clamp(a * alpha_scale, 0.0, 1.0)
    return c


def point_from_xyz(x, y, z):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def point_from_list(values, z_offset=0.0):
    return point_from_xyz(values[0], values[1], values[2] + z_offset)


def marker_base(frame_id, namespace, marker_id, marker_type, scale, marker_color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time(0)
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color = marker_color
    marker.lifetime = rospy.Duration(0)
    return marker


def text_marker(frame_id, namespace, marker_id, point, text, size, marker_color):
    marker = marker_base(frame_id, namespace, marker_id, Marker.TEXT_VIEW_FACING, size, marker_color)
    marker.pose.position = point
    marker.text = text
    return marker


def arrow_marker(frame_id, namespace, marker_id, start, end, shaft_width, head_width, head_length, marker_color):
    marker = marker_base(frame_id, namespace, marker_id, Marker.ARROW, shaft_width, marker_color)
    marker.scale.x = shaft_width
    marker.scale.y = head_width
    marker.scale.z = head_length
    marker.points.append(start)
    marker.points.append(end)
    return marker


def append_line_segments(marker, points):
    if len(points) < 2:
        return 0
    for i in range(len(points) - 1):
        marker.points.append(points[i])
        marker.points.append(points[i + 1])
    return len(points) - 1


def link_midpoint(link):
    points = link.get("points", [])
    if not points:
        return None
    return points[len(points) // 2]


def endpoint_direction(points, distance):
    if len(points) < 2:
        return None
    end = points[-1]
    for prev in reversed(points[:-1]):
        dx = end[0] - prev[0]
        dy = end[1] - prev[1]
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            scale = distance / norm
            return [end[0] - dx * scale, end[1] - dy * scale, end[2]]
    return None


def append_width_edges(marker, points, width_start, width_end, z_offset):
    if len(points) < 2:
        return 0
    count = 0
    width_start = float(width_start)
    width_end = float(width_end)
    denom = max(1, len(points) - 1)
    for i in range(len(points) - 1):
        a = points[i]
        b = points[i + 1]
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            continue
        nx = -dy / norm
        ny = dx / norm
        t0 = i / denom
        t1 = (i + 1) / denom
        w0 = width_start + (width_end - width_start) * t0
        w1 = width_start + (width_end - width_start) * t1
        left_a = point_from_xyz(a[0] + nx * w0 * 0.5, a[1] + ny * w0 * 0.5, a[2] + z_offset)
        left_b = point_from_xyz(b[0] + nx * w1 * 0.5, b[1] + ny * w1 * 0.5, b[2] + z_offset)
        right_a = point_from_xyz(a[0] - nx * w0 * 0.5, a[1] - ny * w0 * 0.5, a[2] + z_offset)
        right_b = point_from_xyz(b[0] - nx * w1 * 0.5, b[1] - ny * w1 * 0.5, b[2] + z_offset)
        marker.points.extend([left_a, left_b, right_a, right_b])
        count += 2
    return count


def counter_summary(counter, max_items=8):
    if not counter:
        return "none"
    return ", ".join("%s:%s" % (k, v) for k, v in counter.most_common(max_items))


def field_coverage(items):
    present = Counter()
    non_null = Counter()
    truthy = Counter()
    for item in items:
        for key, value in item.items():
            present[key] += 1
            if value is not None:
                non_null[key] += 1
            if value:
                truthy[key] += 1
    return present, non_null, truthy


def make_mgeo_markers(global_info, nodes, links, params, paths):
    alpha = clamp(float(params["alpha"]), 0.0, 1.0)
    frame_id = params["frame_id"]
    z_offset = float(params["z_offset"])
    line_width = float(params["line_width"])
    width_line_width = float(params["width_line_width"])
    node_scale = float(params["node_scale"])
    stop_node_scale = float(params["stop_node_scale"])
    signal_node_scale = float(params["signal_node_scale"])

    node_by_id = {node["idx"]: node for node in nodes}
    link_by_id = {link["idx"]: link for link in links}

    markers = []
    stats = {
        "nodes": len(nodes),
        "links": len(links),
        "link_points": 0,
        "link_segments": 0,
        "width_edge_segments": 0,
        "stop_nodes": 0,
        "traffic_light_nodes": 0,
        "lane_change_left": 0,
        "lane_change_right": 0,
        "signal_links": 0,
        "dangling_from_nodes": 0,
        "dangling_to_nodes": 0,
        "dangling_lane_change_targets": 0,
        "hash_mismatches": [],
    }

    link_types = Counter()
    road_types = Counter()
    road_ids = Counter()
    ego_lanes = Counter()
    related_signals = Counter()
    geometry_methods = Counter()
    node_types = Counter()

    for node in nodes:
        node_types[node.get("node_type")] += 1
        if node.get("on_stop_line"):
            stats["stop_nodes"] += 1
        if node.get("traffic_light_id"):
            stats["traffic_light_nodes"] += 1

    centerlines = marker_base(
        frame_id,
        "mgeo_link_centerlines",
        0,
        Marker.LINE_LIST,
        line_width,
        color(1.0, 0.0, 0.85, 0.95, alpha),
    )
    width_edges = marker_base(
        frame_id,
        "mgeo_width_edges",
        1,
        Marker.LINE_LIST,
        width_line_width,
        color(0.0, 0.45, 1.0, 0.55, alpha),
    )
    all_nodes = marker_base(
        frame_id,
        "mgeo_nodes",
        2,
        Marker.SPHERE_LIST,
        node_scale,
        color(0.0, 0.25, 1.0, 0.45, alpha),
    )
    stop_nodes = marker_base(
        frame_id,
        "mgeo_stop_nodes",
        3,
        Marker.SPHERE_LIST,
        stop_node_scale,
        color(1.0, 0.05, 0.15, 0.95, alpha),
    )
    signal_nodes = marker_base(
        frame_id,
        "mgeo_signal_nodes",
        4,
        Marker.CUBE_LIST,
        signal_node_scale,
        color(0.65, 0.15, 1.0, 0.90, alpha),
    )
    lane_change_left = marker_base(
        frame_id,
        "mgeo_lane_change_left",
        5,
        Marker.LINE_LIST,
        line_width * 1.6,
        color(0.0, 1.0, 0.55, 0.85, alpha),
    )
    lane_change_right = marker_base(
        frame_id,
        "mgeo_lane_change_right",
        6,
        Marker.LINE_LIST,
        line_width * 1.6,
        color(1.0, 0.25, 0.45, 0.85, alpha),
    )

    for node in nodes:
        point = point_from_list(node["point"], z_offset)
        if params["show_nodes"]:
            all_nodes.points.append(point)
        if params["show_stop_nodes"] and node.get("on_stop_line"):
            stop_nodes.points.append(point)
        if params["show_signal_nodes"] and node.get("traffic_light_id"):
            signal_nodes.points.append(point)

    signal_colors = {
        "straight": color(0.0, 0.95, 1.0, 0.90, alpha),
        "left": color(0.35, 1.0, 0.10, 0.90, alpha),
        "left_unprotected": color(0.55, 1.0, 0.25, 0.90, alpha),
        "right": color(1.0, 0.55, 0.0, 0.90, alpha),
        "right_unprotected": color(1.0, 0.35, 0.75, 0.90, alpha),
        "uturn_normal": color(1.0, 0.10, 0.10, 0.90, alpha),
    }
    signal_markers = {}

    for link in links:
        points = link.get("points", [])
        stats["link_points"] += len(points)
        link_types[link.get("link_type")] += 1
        road_types[link.get("road_type")] += 1
        road_ids[link.get("road_id")] += 1
        ego_lanes[link.get("ego_lane")] += 1
        if link.get("related_signal"):
            related_signals[link.get("related_signal")] += 1
        for geometry in link.get("geometry", []):
            geometry_methods[geometry.get("method")] += 1
        if link.get("from_node_idx") not in node_by_id:
            stats["dangling_from_nodes"] += 1
        if link.get("to_node_idx") not in node_by_id:
            stats["dangling_to_nodes"] += 1

        marker_points = [point_from_list(point, z_offset) for point in points]
        stats["link_segments"] += append_line_segments(centerlines, marker_points)
        if params["show_width_edges"] and points:
            stats["width_edge_segments"] += append_width_edges(
                width_edges,
                points,
                link.get("width_start", 0.0),
                link.get("width_end", 0.0),
                z_offset + 0.04,
            )

        midpoint = link_midpoint(link)
        if midpoint is None:
            continue
        source = point_from_list(midpoint, z_offset + 0.08)
        if params["show_lane_changes"]:
            left_id = link.get("left_lane_change_dst_link_idx")
            if left_id:
                target_link = link_by_id.get(left_id)
                target_midpoint = link_midpoint(target_link) if target_link else None
                if target_midpoint is not None:
                    lane_change_left.points.extend([source, point_from_list(target_midpoint, z_offset + 0.08)])
                    stats["lane_change_left"] += 1
                else:
                    stats["dangling_lane_change_targets"] += 1
            right_id = link.get("right_lane_change_dst_link_idx")
            if right_id:
                target_link = link_by_id.get(right_id)
                target_midpoint = link_midpoint(target_link) if target_link else None
                if target_midpoint is not None:
                    lane_change_right.points.extend([source, point_from_list(target_midpoint, z_offset + 0.08)])
                    stats["lane_change_right"] += 1
                else:
                    stats["dangling_lane_change_targets"] += 1

        signal = link.get("related_signal")
        if params["show_signal_dirs"] and signal:
            start_raw = endpoint_direction(points, float(params["signal_arrow_length"]))
            if start_raw is not None:
                if signal not in signal_markers:
                    marker_id = 100 + len(signal_markers)
                    signal_markers[signal] = []
                marker_id = 100 + len(signal_markers) * 1000 + len(signal_markers[signal])
                signal_markers[signal].append(
                    arrow_marker(
                        frame_id,
                        "mgeo_signal_%s" % signal,
                        marker_id,
                        point_from_list(start_raw, z_offset + 0.16),
                        point_from_list(points[-1], z_offset + 0.16),
                        line_width * 1.8,
                        line_width * 4.0,
                        line_width * 6.0,
                        signal_colors.get(signal, color(0.0, 0.95, 1.0, 0.90, alpha)),
                    )
                )
                stats["signal_links"] += 1

    if centerlines.points:
        markers.append(centerlines)
    if params["show_width_edges"] and width_edges.points:
        markers.append(width_edges)
    if params["show_nodes"] and all_nodes.points:
        markers.append(all_nodes)
    if params["show_stop_nodes"] and stop_nodes.points:
        markers.append(stop_nodes)
    if params["show_signal_nodes"] and signal_nodes.points:
        markers.append(signal_nodes)
    if params["show_lane_changes"] and lane_change_left.points:
        markers.append(lane_change_left)
    if params["show_lane_changes"] and lane_change_right.points:
        markers.append(lane_change_right)
    if params["show_signal_dirs"]:
        for marker_list in signal_markers.values():
            markers.extend(marker_list)

    if params["show_origin"]:
        origin = point_from_xyz(0.0, 0.0, z_offset + 0.25)
        markers.append(
            marker_base(
                frame_id,
                "mgeo_origin",
                20,
                Marker.SPHERE,
                float(params["origin_scale"]),
                color(1.0, 1.0, 1.0, 0.95, alpha),
            )
        )
        markers[-1].pose.position = origin
        markers.append(
            arrow_marker(
                frame_id,
                "mgeo_origin_axis_x",
                21,
                origin,
                point_from_xyz(float(params["origin_axis_length"]), 0.0, z_offset + 0.25),
                line_width * 2.0,
                line_width * 5.0,
                line_width * 8.0,
                color(1.0, 0.0, 0.0, 0.90, alpha),
            )
        )
        markers.append(
            arrow_marker(
                frame_id,
                "mgeo_origin_axis_y",
                22,
                origin,
                point_from_xyz(0.0, float(params["origin_axis_length"]), z_offset + 0.25),
                line_width * 2.0,
                line_width * 5.0,
                line_width * 8.0,
                color(0.0, 1.0, 0.0, 0.90, alpha),
            )
        )

    expected_hashes = parse_hash_table(global_info.get("mgeo_file_hash"))
    for key, path in (
        ("node_set.json", paths["node_file"]),
        ("link_set.json", paths["link_file"]),
    ):
        expected = expected_hashes.get(key)
        if expected:
            actual = sha256_file(path)
            if actual != expected:
                stats["hash_mismatches"].append("%s expected=%s actual=%s" % (key, expected[:12], actual[:12]))

    node_present, node_non_null, node_truthy = field_coverage(nodes)
    link_present, link_non_null, link_truthy = field_coverage(links)
    stats.update(
        {
            "markers": len(markers),
            "global_keys": len(global_info),
            "hash_entries": len(expected_hashes),
            "node_types": node_types,
            "link_types": link_types,
            "road_types": road_types,
            "road_ids": road_ids,
            "ego_lanes": ego_lanes,
            "related_signals": related_signals,
            "geometry_methods": geometry_methods,
            "node_fields": sorted(node_present.keys()),
            "link_fields": sorted(link_present.keys()),
            "node_field_non_null": node_non_null,
            "node_field_truthy": node_truthy,
            "link_field_non_null": link_non_null,
            "link_field_truthy": link_truthy,
        }
    )

    if params["show_metadata_text"]:
        text = (
            "MGeo JSON\\n"
            "saved: %s\\n"
            "global: %s\\n"
            "local_origin: %s\\n"
            "workspace_origin: %s\\n"
            "nodes=%d links=%d points=%d\\n"
            "node_types: %s\\n"
            "link_types: %s\\n"
            "signals: %s\\n"
            "hash_mismatch: %s"
            % (
                global_info.get("saved_utc_time", ""),
                global_info.get("global_coordinate_system", ""),
                global_info.get("local_origin_in_global", ""),
                global_info.get("workspace_origin", ""),
                stats["nodes"],
                stats["links"],
                stats["link_points"],
                counter_summary(node_types, 4),
                counter_summary(link_types, 6),
                counter_summary(related_signals, 6),
                "; ".join(stats["hash_mismatches"]) if stats["hash_mismatches"] else "none",
            )
        )
        markers.append(
            text_marker(
                frame_id,
                "mgeo_metadata",
                30,
                point_from_xyz(0.0, 0.0, z_offset + float(params["metadata_text_z"])),
                text,
                float(params["metadata_text_size"]),
                color(1.0, 1.0, 1.0, 0.90, alpha),
            )
        )
        stats["markers"] = len(markers)

    marker_array = MarkerArray()
    marker_array.markers = markers
    return marker_array, stats


def log_stats(stats):
    rospy.loginfo(
        "MGeo marker map ready: global_keys=%d hash_entries=%d nodes=%d links=%d "
        "link_points=%d link_segments=%d markers=%d",
        stats["global_keys"],
        stats["hash_entries"],
        stats["nodes"],
        stats["links"],
        stats["link_points"],
        stats["link_segments"],
        stats["markers"],
    )
    rospy.loginfo(
        "MGeo semantic stats: stop_nodes=%d traffic_light_nodes=%d lane_change_left=%d "
        "lane_change_right=%d signal_links=%d width_edge_segments=%d",
        stats["stop_nodes"],
        stats["traffic_light_nodes"],
        stats["lane_change_left"],
        stats["lane_change_right"],
        stats["signal_links"],
        stats["width_edge_segments"],
    )
    rospy.loginfo("MGeo node_types: %s", counter_summary(stats["node_types"]))
    rospy.loginfo("MGeo link_types: %s", counter_summary(stats["link_types"]))
    rospy.loginfo("MGeo road_types: %s", counter_summary(stats["road_types"]))
    rospy.loginfo("MGeo ego_lanes: %s", counter_summary(stats["ego_lanes"]))
    rospy.loginfo("MGeo related_signals: %s", counter_summary(stats["related_signals"]))
    rospy.loginfo("MGeo geometry_methods: %s", counter_summary(stats["geometry_methods"]))
    rospy.loginfo("MGeo node_fields: %s", ", ".join(stats["node_fields"]))
    rospy.loginfo("MGeo link_fields: %s", ", ".join(stats["link_fields"]))
    if stats["dangling_from_nodes"] or stats["dangling_to_nodes"] or stats["dangling_lane_change_targets"]:
        rospy.logwarn(
            "MGeo dangling references: from_nodes=%d to_nodes=%d lane_change_targets=%d",
            stats["dangling_from_nodes"],
            stats["dangling_to_nodes"],
            stats["dangling_lane_change_targets"],
        )
    for mismatch in stats["hash_mismatches"]:
        rospy.logwarn("MGeo hash mismatch: %s", mismatch)


def main():
    rospy.init_node("mgeo_json_marker_publisher")
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo_root = os.path.abspath(os.path.join(package_dir, "..", "..", ".."))

    paths = {
        "global_file": rospy.get_param(
            "~global_file", os.path.join(repo_root, "kcity_map", "global_info.json")
        ),
        "node_file": rospy.get_param("~node_file", os.path.join(repo_root, "kcity_map", "node_set.json")),
        "link_file": rospy.get_param("~link_file", os.path.join(repo_root, "kcity_map", "link_set.json")),
    }
    params = {
        "frame_id": rospy.get_param("~frame_id", "morai_map"),
        "output_topic": rospy.get_param("~output_topic", "/morai_mgeo/markers"),
        "publish_rate": float(rospy.get_param("~publish_rate", 0.0)),
        "alpha": float(rospy.get_param("~alpha", 0.85)),
        "z_offset": float(rospy.get_param("~z_offset", 0.08)),
        "line_width": float(rospy.get_param("~line_width", 0.16)),
        "width_line_width": float(rospy.get_param("~width_line_width", 0.06)),
        "node_scale": float(rospy.get_param("~node_scale", 0.45)),
        "stop_node_scale": float(rospy.get_param("~stop_node_scale", 0.95)),
        "signal_node_scale": float(rospy.get_param("~signal_node_scale", 0.85)),
        "signal_arrow_length": float(rospy.get_param("~signal_arrow_length", 4.0)),
        "origin_scale": float(rospy.get_param("~origin_scale", 1.2)),
        "origin_axis_length": float(rospy.get_param("~origin_axis_length", 8.0)),
        "metadata_text_size": float(rospy.get_param("~metadata_text_size", 1.8)),
        "metadata_text_z": float(rospy.get_param("~metadata_text_z", 5.0)),
        "show_width_edges": as_bool(rospy.get_param("~show_width_edges", True)),
        "show_nodes": as_bool(rospy.get_param("~show_nodes", True)),
        "show_stop_nodes": as_bool(rospy.get_param("~show_stop_nodes", True)),
        "show_signal_nodes": as_bool(rospy.get_param("~show_signal_nodes", True)),
        "show_lane_changes": as_bool(rospy.get_param("~show_lane_changes", True)),
        "show_signal_dirs": as_bool(rospy.get_param("~show_signal_dirs", True)),
        "show_origin": as_bool(rospy.get_param("~show_origin", True)),
        "show_metadata_text": as_bool(rospy.get_param("~show_metadata_text", True)),
    }

    rospy.loginfo("Loading MGeo global JSON: %s", paths["global_file"])
    rospy.loginfo("Loading MGeo node JSON: %s", paths["node_file"])
    rospy.loginfo("Loading MGeo link JSON: %s", paths["link_file"])
    global_info = load_json(paths["global_file"])
    nodes = load_json(paths["node_file"])
    links = load_json(paths["link_file"])
    marker_array, stats = make_mgeo_markers(global_info, nodes, links, params, paths)
    log_stats(stats)

    publisher = rospy.Publisher(params["output_topic"], MarkerArray, queue_size=1, latch=True)

    def publish_once():
        publisher.publish(marker_array)

    rospy.sleep(0.5)
    publish_once()
    rospy.loginfo("Published MGeo markers to %s in frame %s", params["output_topic"], params["frame_id"])

    if params["publish_rate"] > 0.0:
        rate = rospy.Rate(params["publish_rate"])
        while not rospy.is_shutdown():
            publish_once()
            rate.sleep()
    else:
        rospy.spin()


if __name__ == "__main__":
    main()
