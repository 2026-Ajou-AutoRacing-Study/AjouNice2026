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
from autoware_map_msgs.msg import LaneletMapBin
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


def clamp(value, lower=0.0, upper=1.0):
    return max(lower, min(upper, value))


def color(r, g, b, a):
    c = ColorRGBA()
    c.r = clamp(r)
    c.g = clamp(g)
    c.b = clamp(b)
    c.a = clamp(a)
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


def visualization_defaults():
    """Fallback values for a missing key in the RViz style YAML."""
    return {
        "alpha": 1.0,
        "road_fill_alpha": 0.16,
        "crosswalk_fill_alpha": 0.30,
        "left_boundary_alpha": 0.95,
        "right_boundary_alpha": 0.95,
        "centerline_alpha": 0.85,
        "road_border_alpha": 0.85,
        "lane_marking_alpha": 0.90,
        "stop_line_alpha": 1.0,
        "traffic_light_alpha": 0.95,
        "left_boundary_width": 0.18,
        "right_boundary_width": 0.18,
        "centerline_width": 0.12,
        "road_border_width": 0.18,
        "lane_marking_width": 0.10,
        "stop_line_width": 0.32,
        "traffic_light_scale": 0.70,
        # Small offsets keep overlapping map primitives legible in RViz.
        "fill_z_offset": -0.02,
        "boundary_z_offset": 0.03,
        "centerline_z_offset": 0.06,
        "feature_z_offset": 0.09,
        "dashed_line_length": 3.0,
        "dashed_gap_length": 2.0,
        "show_left_boundaries": True,
        "show_right_boundaries": True,
        "show_centerlines": True,
        "show_road_borders": True,
        "show_lane_markings": True,
        "show_stop_lines": True,
        "show_traffic_lights": True,
        "show_road_fill": False,
        "show_crosswalk_fill": False,
        "colors": {
            "road_fill": (0.20, 0.70, 0.70),
            "crosswalk_fill": (0.20, 0.70, 0.20),
            "left_boundary": (0.30, 0.80, 1.00),
            "right_boundary": (1.00, 0.55, 0.25),
            "centerline": (1.00, 0.90, 0.20),
            "road_border": (0.65, 0.55, 0.75),
            "lane_marking": (0.88, 0.88, 0.88),
            "stop_line": (1.00, 0.25, 0.05),
            "traffic_light": (0.85, 0.25, 0.95),
        },
    }


def load_visualization_params(style_file):
    """Read the user-editable RViz style YAML and validate its small schema."""
    params = visualization_defaults()
    with open(style_file, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        raise ValueError("Visualization style YAML must contain a mapping: %s" % style_file)

    sections = {
        "alpha": {
            "global": "alpha",
            "road_fill": "road_fill_alpha",
            "crosswalk_fill": "crosswalk_fill_alpha",
            "left_boundary": "left_boundary_alpha",
            "right_boundary": "right_boundary_alpha",
            "centerline": "centerline_alpha",
            "road_border": "road_border_alpha",
            "lane_marking": "lane_marking_alpha",
            "stop_line": "stop_line_alpha",
            "traffic_light": "traffic_light_alpha",
        },
        "width": {
            "left_boundary": "left_boundary_width",
            "right_boundary": "right_boundary_width",
            "centerline": "centerline_width",
            "road_border": "road_border_width",
            "lane_marking": "lane_marking_width",
            "stop_line": "stop_line_width",
            "traffic_light": "traffic_light_scale",
        },
        "z_offset": {
            "fill": "fill_z_offset",
            "boundary": "boundary_z_offset",
            "centerline": "centerline_z_offset",
            "feature": "feature_z_offset",
        },
        "dash": {
            "line_length": "dashed_line_length",
            "gap_length": "dashed_gap_length",
        },
        "show": {
            "left_boundaries": "show_left_boundaries",
            "right_boundaries": "show_right_boundaries",
            "centerlines": "show_centerlines",
            "road_borders": "show_road_borders",
            "lane_markings": "show_lane_markings",
            "stop_lines": "show_stop_lines",
            "traffic_lights": "show_traffic_lights",
            "road_fill": "show_road_fill",
            "crosswalk_fill": "show_crosswalk_fill",
        },
    }

    unknown_sections = set(config) - set(sections) - {"color"}
    if unknown_sections:
        raise ValueError("Unknown visualization style section(s): %s" % ", ".join(sorted(unknown_sections)))

    for section, keys in sections.items():
        values = config.get(section, {})
        if not isinstance(values, dict):
            raise ValueError("Visualization style section '%s' must be a mapping" % section)
        unknown_keys = set(values) - set(keys)
        if unknown_keys:
            raise ValueError(
                "Unknown visualization style key(s) in %s: %s" % (section, ", ".join(sorted(unknown_keys)))
            )
        for yaml_key, param_key in keys.items():
            if yaml_key in values:
                default = params[param_key]
                params[param_key] = as_bool(values[yaml_key]) if isinstance(default, bool) else float(values[yaml_key])

    colors = config.get("color", {})
    if not isinstance(colors, dict):
        raise ValueError("Visualization style section 'color' must be a mapping")
    unknown_colors = set(colors) - set(params["colors"])
    if unknown_colors:
        raise ValueError("Unknown visualization color(s): %s" % ", ".join(sorted(unknown_colors)))
    for name, rgb in colors.items():
        if not isinstance(rgb, (list, tuple)) or len(rgb) != 3:
            raise ValueError("Visualization color '%s' must be an RGB list with three values" % name)
        params["colors"][name] = tuple(clamp(float(channel)) for channel in rgb)

    if params["dashed_line_length"] <= 0.0 or params["dashed_gap_length"] <= 0.0:
        raise ValueError("Visualization dash line_length and gap_length must both be greater than zero")

    return params


def styled_color(color_key, alpha_key, params):
    return color(*params["colors"][color_key], params[alpha_key] * params["alpha"])


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


def offset_point(point, z_offset):
    if z_offset == 0.0:
        return point
    return point_from_xyz(point.x, point.y, point.z + z_offset)


def append_line_segments(marker, points, z_offset=0.0):
    if len(points) < 2:
        return
    for i in range(len(points) - 1):
        marker.points.append(offset_point(points[i], z_offset))
        marker.points.append(offset_point(points[i + 1], z_offset))


def append_dashed_line_segments(marker, points, line_length, gap_length, z_offset=0.0):
    """Append alternating visible and hidden pieces while preserving polyline distance."""
    if len(points) < 2:
        return

    cycle_length = line_length + gap_length
    traversed = 0.0
    for start, end in zip(points, points[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z
        segment_length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if segment_length <= 1e-9:
            continue

        segment_position = 0.0
        while segment_position < segment_length - 1e-9:
            cycle_position = traversed % cycle_length
            is_visible = cycle_position < line_length
            until_transition = (line_length - cycle_position) if is_visible else (cycle_length - cycle_position)
            step = min(segment_length - segment_position, until_transition)

            if is_visible and step > 1e-9:
                start_ratio = segment_position / segment_length
                end_ratio = (segment_position + step) / segment_length
                marker.points.append(
                    offset_point(
                        point_from_xyz(
                            start.x + dx * start_ratio,
                            start.y + dy * start_ratio,
                            start.z + dz * start_ratio,
                        ),
                        z_offset,
                    )
                )
                marker.points.append(
                    offset_point(
                        point_from_xyz(
                            start.x + dx * end_ratio,
                            start.y + dy * end_ratio,
                            start.z + dz * end_ratio,
                        ),
                        z_offset,
                    )
                )

            segment_position += step
            traversed += step


def append_line_by_style(marker, points, tags, params, z_offset=0.0):
    if tags.get("subtype") == "dashed":
        append_dashed_line_segments(
            marker,
            points,
            params["dashed_line_length"],
            params["dashed_gap_length"],
            z_offset,
        )
    else:
        append_line_segments(marker, points, z_offset)


def make_way_line_marker(
    frame_id, namespace, marker_id, ways, nodes, predicate, marker_color, width, params, z_offset=0.0
):
    marker = marker_base(frame_id, namespace, marker_id, Marker.LINE_LIST, width, marker_color)
    count = 0
    for way in ways.values():
        if predicate(way["tags"]):
            pts = way_points(way, nodes)
            append_line_by_style(marker, pts, way["tags"], params, z_offset)
            count += 1
    return marker, count


def relation_way_by_role(relation, role):
    for member in relation["members"]:
        if member["type"] == "way" and member["role"] == role:
            return member["ref"]
    return None


def append_lanelet_triangles(marker, left_pts, right_pts, z_offset=0.0):
    polygon = list(left_pts) + list(reversed(right_pts))
    if len(polygon) < 3:
        return
    anchor = polygon[0]
    for i in range(1, len(polygon) - 1):
        marker.points.extend(
            [
                offset_point(anchor, z_offset),
                offset_point(polygon[i], z_offset),
                offset_point(polygon[i + 1], z_offset),
            ]
        )


def make_lanelet_markers(frame_id, nodes, ways, relations, params, publish_fill=False):
    road_fill = marker_base(
        frame_id,
        "lanelet/road_fill",
        0,
        Marker.TRIANGLE_LIST,
        1.0,
        styled_color("road_fill", "road_fill_alpha", params),
    )
    crosswalk_fill = marker_base(
        frame_id,
        "lanelet/crosswalk_fill",
        1,
        Marker.TRIANGLE_LIST,
        1.0,
        styled_color("crosswalk_fill", "crosswalk_fill_alpha", params),
    )
    left_boundaries = marker_base(
        frame_id,
        "lanelet/left_boundaries",
        2,
        Marker.LINE_LIST,
        params["left_boundary_width"],
        styled_color("left_boundary", "left_boundary_alpha", params),
    )
    right_boundaries = marker_base(
        frame_id,
        "lanelet/right_boundaries",
        3,
        Marker.LINE_LIST,
        params["right_boundary_width"],
        styled_color("right_boundary", "right_boundary_alpha", params),
    )
    centerlines = marker_base(
        frame_id,
        "lanelet/centerlines",
        4,
        Marker.LINE_LIST,
        params["centerline_width"],
        styled_color("centerline", "centerline_alpha", params),
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
        append_line_by_style(
            left_boundaries, left_pts, ways[left_id]["tags"], params, params["boundary_z_offset"]
        )
        append_line_by_style(
            right_boundaries, right_pts, ways[right_id]["tags"], params, params["boundary_z_offset"]
        )
        if params["show_centerlines"] and center_id in ways:
            append_line_by_style(
                centerlines,
                way_points(ways[center_id], nodes),
                ways[center_id]["tags"],
                params,
                params["centerline_z_offset"],
            )

        subtype = relation["tags"].get("subtype")
        if subtype == "crosswalk":
            append_lanelet_triangles(crosswalk_fill, left_pts, right_pts, params["fill_z_offset"])
            crosswalk_count += 1
        else:
            append_lanelet_triangles(road_fill, left_pts, right_pts, params["fill_z_offset"])
            road_count += 1

    markers = []
    if params["show_left_boundaries"]:
        markers.append(left_boundaries)
    if params["show_right_boundaries"]:
        markers.append(right_boundaries)
    if params["show_centerlines"]:
        markers.append(centerlines)
    if publish_fill:
        if params["show_road_fill"]:
            markers.insert(0, road_fill)
        if params["show_crosswalk_fill"]:
            markers.insert(1 if params["show_road_fill"] else 0, crosswalk_fill)
    return markers, road_count, crosswalk_count


def make_traffic_light_marker(frame_id, ways, nodes, params):
    marker = marker_base(
        frame_id,
        "features/traffic_lights",
        20,
        Marker.SPHERE_LIST,
        params["traffic_light_scale"],
        styled_color("traffic_light", "traffic_light_alpha", params),
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
        marker.points.append(point_from_xyz(x, y, z + params["feature_z_offset"]))
        count += 1
    return marker, count


def build_marker_array(
    map_file,
    projector_file,
    frame_id="map",
    flatten_z=False,
    publish_lanelet_fill=False,
    projector_type="LocalCartesianUTM",
    visual_params=None,
):
    params = visualization_defaults()
    if visual_params:
        params.update(visual_params)
    projector = projector_configuration(map_file, projector_file, projector_type)
    nodes, ways, relations = parse_osm(map_file, projector, flatten_z=flatten_z)
    markers = []

    lanelet_markers, road_lanelets, crosswalk_lanelets = make_lanelet_markers(
        frame_id,
        nodes,
        ways,
        relations,
        params,
        publish_fill=(publish_lanelet_fill or params["show_road_fill"] or params["show_crosswalk_fill"]),
    )
    markers.extend(lanelet_markers)

    road_borders, road_border_count = make_way_line_marker(
        frame_id,
        "features/road_borders",
        10,
        ways,
        nodes,
        lambda tags: tags.get("type") == "road_border",
        styled_color("road_border", "road_border_alpha", params),
        params["road_border_width"],
        params,
        params["feature_z_offset"],
    )
    lane_markings, lane_marking_count = make_way_line_marker(
        frame_id,
        "features/lane_markings",
        11,
        ways,
        nodes,
        lambda tags: tags.get("type") == "line_thin" and tags.get("subtype") != "stop_line",
        styled_color("lane_marking", "lane_marking_alpha", params),
        params["lane_marking_width"],
        params,
        params["feature_z_offset"],
    )
    stop_lines, stop_line_count = make_way_line_marker(
        frame_id,
        "features/stop_lines",
        12,
        ways,
        nodes,
        lambda tags: tags.get("subtype") == "stop_line",
        styled_color("stop_line", "stop_line_alpha", params),
        params["stop_line_width"],
        params,
        params["feature_z_offset"],
    )
    traffic_lights, traffic_light_count = make_traffic_light_marker(frame_id, ways, nodes, params)

    if params["show_road_borders"]:
        markers.append(road_borders)
    if params["show_lane_markings"]:
        markers.append(lane_markings)
    if params["show_stop_lines"]:
        markers.append(stop_lines)
    if params["show_traffic_lights"]:
        markers.append(traffic_lights)

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
    default_map = os.path.join(repo_root, "kcity_map", "lanelet_map", "lanelet2_map_v72.osm")
    default_projector = os.path.join(repo_root, "kcity_map", "lanelet_map", "map_projector_info_v1.yaml")
    default_style = os.path.join(package_dir, "config", "lanelet_marker_style.yaml")

    map_file = rospy.get_param("~map_file", default_map)
    projector_file = rospy.get_param("~projector_file", default_projector)
    projector_type = rospy.get_param("~projector_type", "LocalCartesianUTM")
    frame_id = rospy.get_param("~frame_id", "map")
    output_topic = rospy.get_param("~output_topic", "/map/lanelet_map")
    legacy_output_topic = rospy.get_param("~legacy_output_topic", "")
    vector_map_topic = rospy.get_param("~vector_map_topic", "/map/vector_map")
    publish_vector_map = as_bool(rospy.get_param("~publish_vector_map", True))
    vector_map_name = rospy.get_param("~vector_map_name", os.path.basename(map_file))
    vector_map_version = rospy.get_param(
        "~vector_map_version", os.path.splitext(os.path.basename(map_file))[0]
    )
    publish_rate = float(rospy.get_param("~publish_rate", 0.0))
    flatten_z = as_bool(rospy.get_param("~flatten_z", False))
    publish_lanelet_fill = as_bool(rospy.get_param("~publish_lanelet_fill", False))
    style_file = rospy.get_param("~style_file", default_style)
    visual_params = load_visualization_params(style_file)

    rospy.loginfo("Loading lanelet OSM: %s", map_file)
    rospy.loginfo("Loading projector: type=%s file=%s", projector_type, projector_file)
    rospy.loginfo("Loading visualization style: %s", style_file)
    marker_array, stats = build_marker_array(
        map_file,
        projector_file,
        frame_id=frame_id,
        flatten_z=flatten_z,
        publish_lanelet_fill=publish_lanelet_fill,
        projector_type=projector_type,
        visual_params=visual_params,
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

    marker_publishers = [rospy.Publisher(output_topic, MarkerArray, queue_size=1, latch=True)]
    if legacy_output_topic and legacy_output_topic != output_topic:
        marker_publishers.append(rospy.Publisher(legacy_output_topic, MarkerArray, queue_size=1, latch=True))

    vector_map = None
    vector_map_publisher = None
    if publish_vector_map:
        with open(map_file, "rb") as stream:
            osm_bytes = stream.read()
        vector_map = LaneletMapBin()
        vector_map.header.frame_id = frame_id
        vector_map.version_map_format = "lanelet2_osm"
        vector_map.version_map = str(vector_map_version)
        vector_map.name_map = str(vector_map_name)
        vector_map.data = osm_bytes
        vector_map_publisher = rospy.Publisher(
            vector_map_topic, LaneletMapBin, queue_size=1, latch=True
        )
        rospy.set_param("/map/projector_file", projector_file)
        rospy.set_param("/map/projector_type", projector_type)
        rospy.set_param("/map/map_file", map_file)

    def publish_once():
        for pub in marker_publishers:
            pub.publish(marker_array)
        if vector_map_publisher is not None and vector_map is not None:
            vector_map.header.stamp = rospy.Time.now()
            vector_map_publisher.publish(vector_map)

    rospy.sleep(0.5)
    publish_once()
    rospy.loginfo(
        "Published lanelet markers to %s%s",
        output_topic,
        "" if not legacy_output_topic or legacy_output_topic == output_topic else " and " + legacy_output_topic,
    )
    if vector_map_publisher is not None:
        rospy.loginfo(
            "Published planning LaneletMapBin to %s: name=%s version=%s bytes=%d",
            vector_map_topic,
            vector_map.name_map,
            vector_map.version_map,
            len(vector_map.data),
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
