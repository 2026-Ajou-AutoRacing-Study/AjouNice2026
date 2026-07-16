#!/usr/bin/env python3
#
# Copyright 2019 Autoware Foundation. All rights reserved.
# Copyright 2026 AjouNice2026 contributors.
#
# Licensed under the Apache License, Version 2.0.
#
# This ROS1 node follows the Autoware.AI lanelet2_map_visualization output
# contract: publish a visualization_msgs/MarkerArray that RViz can render.
# It reads OSM directly so the K-City LocalCartesianUTM projector can be
# verified without requiring the full Autoware.AI lanelet2 binary-msg stack.

import math
import os
import xml.etree.ElementTree as ET

import rospy
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996
MGRS_GRID_SIZE = 100000.0


def read_projector(path):
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if cfg.get("projector_type") != "LocalCartesianUTM":
        raise ValueError("Only LocalCartesianUTM is supported, got %r" % cfg.get("projector_type"))
    origin = cfg["map_origin"]
    lat = float(origin["latitude"])
    lon = float(origin["longitude"])
    alt = float(origin.get("altitude", 0.0))
    zone = int(math.floor((lon + 180.0) / 6.0) + 1)
    origin_e, origin_n = wgs84_to_utm(lat, lon, zone)
    return {
        "origin_lat": lat,
        "origin_lon": lon,
        "origin_alt": alt,
        "zone": zone,
        "origin_easting": origin_e,
        "origin_northing": origin_n,
    }


def read_mgrs_projector(map_file):
    root = ET.parse(map_file).getroot()
    first_node = next((element for element in root if element.tag == "node"), None)
    if first_node is None:
        raise ValueError("Lanelet2 map contains no nodes")
    lat = float(first_node.attrib["lat"])
    lon = float(first_node.attrib["lon"])
    zone = int(math.floor((lon + 180.0) / 6.0) + 1)
    easting, northing = wgs84_to_utm(lat, lon, zone)
    return {
        "origin_lat": 0.0,
        "origin_lon": 0.0,
        "origin_alt": 0.0,
        "zone": zone,
        "origin_easting": math.floor(easting / MGRS_GRID_SIZE) * MGRS_GRID_SIZE,
        "origin_northing": math.floor(northing / MGRS_GRID_SIZE) * MGRS_GRID_SIZE,
    }


def projector_configuration(map_file, projector_file, projector_type):
    normalized = str(projector_type or "LocalCartesianUTM").lower().replace("_", "")
    if normalized == "mgrs":
        return read_mgrs_projector(map_file)
    if normalized in ("localcartesianutm", "localutm"):
        return read_projector(projector_file)
    raise ValueError("Unsupported projector type: %s" % projector_type)


def wgs84_to_utm(lat_deg, lon_deg, zone):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)

    e2 = WGS84_F * (2.0 - WGS84_F)
    ep2 = e2 / (1.0 - e2)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    tan_lat = math.tan(lat)

    n = WGS84_A / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    t = tan_lat * tan_lat
    c = ep2 * cos_lat * cos_lat
    a = cos_lat * (lon - lon0)

    m = WGS84_A * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat)
        - (35 * e2**3 / 3072) * math.sin(6 * lat)
    )

    easting = UTM_K0 * n * (
        a
        + (1 - t + c) * a**3 / 6
        + (5 - 18 * t + t**2 + 72 * c - 58 * ep2) * a**5 / 120
    ) + 500000.0
    northing = UTM_K0 * (
        m
        + n * tan_lat * (
            a**2 / 2
            + (5 - t + 9 * c + 4 * c**2) * a**4 / 24
            + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * a**6 / 720
        )
    )
    if lat_deg < 0.0:
        northing += 10000000.0
    return easting, northing


def color(r, g, b, a):
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


def point_from_xyz(x, y, z):
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def parse_osm(path, projector, flatten_z=False):
    tree = ET.parse(path)
    root = tree.getroot()

    nodes = {}
    ways = {}
    relations = []

    for elem in root:
        if elem.tag == "node":
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            node_id = elem.attrib["id"]
            lat = float(elem.attrib["lat"])
            lon = float(elem.attrib["lon"])
            ele = float(tags.get("ele", projector["origin_alt"]))
            easting, northing = wgs84_to_utm(lat, lon, projector["zone"])
            z = 0.0 if flatten_z else ele - projector["origin_alt"]
            nodes[node_id] = point_from_xyz(
                easting - projector["origin_easting"],
                northing - projector["origin_northing"],
                z,
            )
        elif elem.tag == "way":
            way_id = elem.attrib["id"]
            refs = [nd.attrib["ref"] for nd in elem if nd.tag == "nd"]
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            ways[way_id] = {"refs": refs, "tags": tags}
        elif elem.tag == "relation":
            members = [
                {
                    "type": m.attrib.get("type"),
                    "ref": m.attrib.get("ref"),
                    "role": m.attrib.get("role", ""),
                }
                for m in elem
                if m.tag == "member"
            ]
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            relations.append({"id": elem.attrib["id"], "members": members, "tags": tags})

    return nodes, ways, relations


def way_points(way, nodes):
    return [nodes[ref] for ref in way["refs"] if ref in nodes]


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


def append_line_segments(marker, points):
    if len(points) < 2:
        return
    for i in range(len(points) - 1):
        marker.points.append(points[i])
        marker.points.append(points[i + 1])


def make_way_line_marker(frame_id, namespace, marker_id, ways, nodes, predicate, marker_color, width):
    marker = marker_base(frame_id, namespace, marker_id, Marker.LINE_LIST, width, marker_color)
    count = 0
    for way in ways.values():
        if predicate(way["tags"]):
            pts = way_points(way, nodes)
            append_line_segments(marker, pts)
            count += 1
    return marker, count


def relation_way_by_role(relation, role):
    for member in relation["members"]:
        if member["type"] == "way" and member["role"] == role:
            return member["ref"]
    return None


def append_lanelet_triangles(marker, left_pts, right_pts):
    polygon = list(left_pts) + list(reversed(right_pts))
    if len(polygon) < 3:
        return
    anchor = polygon[0]
    for i in range(1, len(polygon) - 1):
        marker.points.extend([anchor, polygon[i], polygon[i + 1]])


def make_lanelet_markers(frame_id, nodes, ways, relations, viz_centerline=True, publish_fill=False):
    road_fill = marker_base(
        frame_id, "road_lanelets", 0, Marker.TRIANGLE_LIST, 1.0, color(0.2, 0.7, 0.7, 0.30)
    )
    crosswalk_fill = marker_base(
        frame_id, "crosswalk_lanelets", 1, Marker.TRIANGLE_LIST, 1.0, color(0.2, 0.7, 0.2, 0.30)
    )
    boundaries = marker_base(
        frame_id, "lanelet_boundaries", 2, Marker.LINE_LIST, 0.20, color(1.0, 1.0, 1.0, 0.95)
    )
    centerlines = marker_base(
        frame_id, "lanelet_centerlines", 3, Marker.LINE_LIST, 0.10, color(1.0, 0.9, 0.2, 0.75)
    )

    road_count = 0
    crosswalk_count = 0
    for relation in relations:
        if relation["tags"].get("type") != "lanelet":
            continue
        left_id = relation_way_by_role(relation, "left")
        right_id = relation_way_by_role(relation, "right")
        center_id = relation_way_by_role(relation, "centerline")
        if left_id not in ways or right_id not in ways:
            continue

        left_pts = way_points(ways[left_id], nodes)
        right_pts = way_points(ways[right_id], nodes)
        append_line_segments(boundaries, left_pts)
        append_line_segments(boundaries, right_pts)
        if viz_centerline and center_id in ways:
            append_line_segments(centerlines, way_points(ways[center_id], nodes))

        subtype = relation["tags"].get("subtype")
        if subtype == "crosswalk":
            append_lanelet_triangles(crosswalk_fill, left_pts, right_pts)
            crosswalk_count += 1
        else:
            append_lanelet_triangles(road_fill, left_pts, right_pts)
            road_count += 1

    markers = [boundaries, centerlines]
    if publish_fill:
        markers = [road_fill, crosswalk_fill] + markers
    return markers, road_count, crosswalk_count


def make_traffic_light_marker(frame_id, ways, nodes):
    marker = marker_base(
        frame_id, "traffic_lights", 20, Marker.SPHERE_LIST, 0.7, color(0.7, 0.7, 0.7, 0.85)
    )
    count = 0
    for way in ways.values():
        if way["tags"].get("type") != "traffic_light":
            continue
        pts = way_points(way, nodes)
        if not pts:
            continue
        x = sum(p.x for p in pts) / len(pts)
        y = sum(p.y for p in pts) / len(pts)
        z = sum(p.z for p in pts) / len(pts)
        marker.points.append(point_from_xyz(x, y, z))
        count += 1
    return marker, count


def build_marker_array(
    map_file,
    projector_file,
    frame_id="map",
    flatten_z=False,
    publish_lanelet_fill=False,
    projector_type="LocalCartesianUTM",
):
    projector = projector_configuration(map_file, projector_file, projector_type)
    nodes, ways, relations = parse_osm(map_file, projector, flatten_z=flatten_z)
    markers = []

    lanelet_markers, road_lanelets, crosswalk_lanelets = make_lanelet_markers(
        frame_id, nodes, ways, relations, publish_fill=publish_lanelet_fill
    )
    markers.extend(lanelet_markers)

    road_borders, road_border_count = make_way_line_marker(
        frame_id,
        "road_borders",
        10,
        ways,
        nodes,
        lambda tags: tags.get("type") == "road_border",
        color(0.65, 0.55, 0.75, 0.95),
        0.18,
    )
    lane_markings, lane_marking_count = make_way_line_marker(
        frame_id,
        "lane_markings",
        11,
        ways,
        nodes,
        lambda tags: tags.get("type") == "line_thin",
        color(0.88, 0.88, 0.88, 0.90),
        0.10,
    )
    stop_lines, stop_line_count = make_way_line_marker(
        frame_id,
        "stop_lines",
        12,
        ways,
        nodes,
        lambda tags: tags.get("subtype") == "stop_line",
        color(1.0, 0.45, 0.0, 0.95),
        0.35,
    )
    traffic_lights, traffic_light_count = make_traffic_light_marker(frame_id, ways, nodes)

    markers.extend([road_borders, lane_markings, stop_lines, traffic_lights])

    marker_array = MarkerArray()
    marker_array.markers = [m for m in markers if m.points]
    stats = {
        "nodes": len(nodes),
        "ways": len(ways),
        "relations": len(relations),
        "road_lanelets": road_lanelets,
        "crosswalk_lanelets": crosswalk_lanelets,
        "road_borders": road_border_count,
        "lane_markings": lane_marking_count,
        "stop_lines": stop_line_count,
        "traffic_lights": traffic_light_count,
        "markers": len(marker_array.markers),
        "projector_zone": projector["zone"],
        "origin_easting": projector["origin_easting"],
        "origin_northing": projector["origin_northing"],
    }
    return marker_array, stats


def main():
    rospy.init_node("lanelet2_osm_marker_publisher")
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo_root = os.path.abspath(os.path.join(package_dir, "..", "..", ".."))
    default_map = os.path.join(repo_root, "kcity_map", "lanelet_map", "lanelet2_map_v23.osm")
    default_projector = os.path.join(repo_root, "kcity_map", "lanelet_map", "map_projector_info_v1.yaml")

    map_file = rospy.get_param("~map_file", default_map)
    projector_file = rospy.get_param("~projector_file", default_projector)
    projector_type = rospy.get_param("~projector_type", "LocalCartesianUTM")
    frame_id = rospy.get_param("~frame_id", "map")
    output_topic = rospy.get_param("~output_topic", "/map/lanelet_map")
    legacy_output_topic = rospy.get_param("~legacy_output_topic", "")
    publish_rate = float(rospy.get_param("~publish_rate", 0.0))
    flatten_z = as_bool(rospy.get_param("~flatten_z", False))
    publish_lanelet_fill = as_bool(rospy.get_param("~publish_lanelet_fill", False))

    rospy.loginfo("Loading lanelet OSM: %s", map_file)
    rospy.loginfo("Loading projector: type=%s file=%s", projector_type, projector_file)
    marker_array, stats = build_marker_array(
        map_file,
        projector_file,
        frame_id,
        flatten_z,
        publish_lanelet_fill,
        projector_type,
    )
    rospy.loginfo(
        "Lanelet marker map ready: nodes=%d ways=%d relations=%d road_lanelets=%d "
        "crosswalk_lanelets=%d markers=%d UTM zone=%d origin=(%.6f, %.6f)",
        stats["nodes"],
        stats["ways"],
        stats["relations"],
        stats["road_lanelets"],
        stats["crosswalk_lanelets"],
        stats["markers"],
        stats["projector_zone"],
        stats["origin_easting"],
        stats["origin_northing"],
    )

    publishers = [rospy.Publisher(output_topic, MarkerArray, queue_size=1, latch=True)]
    if legacy_output_topic and legacy_output_topic != output_topic:
        publishers.append(rospy.Publisher(legacy_output_topic, MarkerArray, queue_size=1, latch=True))

    def publish_once():
        for pub in publishers:
            pub.publish(marker_array)

    rospy.sleep(0.5)
    publish_once()
    rospy.loginfo(
        "Published lanelet markers to %s%s",
        output_topic,
        "" if not legacy_output_topic or legacy_output_topic == output_topic else " and " + legacy_output_topic,
    )

    if publish_rate > 0.0:
        rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():
            publish_once()
            rate.sleep()
    else:
        rospy.spin()


if __name__ == "__main__":
    main()
