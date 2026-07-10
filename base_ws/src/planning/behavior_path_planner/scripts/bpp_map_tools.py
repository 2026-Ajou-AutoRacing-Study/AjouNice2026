#!/usr/bin/env python3

import json
import math
import os
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:  # pragma: no cover - yaml exists in the target ROS image.
    yaml = None


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996


def distance_2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


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


def read_projector(path):
    if not path:
        return None
    if yaml is None:
        raise RuntimeError("PyYAML is required to read projector files")

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if cfg.get("projector_type") != "LocalCartesianUTM":
        raise ValueError("Only LocalCartesianUTM projector is supported")

    origin = cfg["map_origin"]
    lat = float(origin["latitude"])
    lon = float(origin["longitude"])
    alt = float(origin.get("altitude", 0.0))
    zone = int(math.floor((lon + 180.0) / 6.0) + 1)
    origin_e, origin_n = wgs84_to_utm(lat, lon, zone)
    return {
        "origin_alt": alt,
        "zone": zone,
        "origin_easting": origin_e,
        "origin_northing": origin_n,
    }


def project_node(lat, lon, ele, projector):
    if projector is None:
        # Useful for small synthetic tests. Real maps should pass a projector.
        return (float(lon), float(lat), float(ele))
    easting, northing = wgs84_to_utm(lat, lon, projector["zone"])
    return (
        easting - projector["origin_easting"],
        northing - projector["origin_northing"],
        ele - projector["origin_alt"],
    )


def relation_way_by_role(relation, role):
    for member in relation["members"]:
        if member["type"] == "way" and member["role"] == role:
            return member["ref"]
    return None


def way_points(way_id, ways, nodes):
    if way_id not in ways:
        return []
    return [nodes[ref] for ref in ways[way_id]["refs"] if ref in nodes]


def cumulative_lengths(points):
    if not points:
        return []
    distances = [0.0]
    for prev, curr in zip(points[:-1], points[1:]):
        distances.append(distances[-1] + distance_2d(prev, curr))
    return distances


def interpolate_polyline(points, target_s):
    distances = cumulative_lengths(points)
    if not distances:
        return None
    if target_s <= 0.0:
        return points[0]
    if target_s >= distances[-1]:
        return points[-1]

    for idx in range(len(points) - 1):
        s0 = distances[idx]
        s1 = distances[idx + 1]
        if target_s > s1:
            continue
        length = max(s1 - s0, 1.0e-6)
        ratio = (target_s - s0) / length
        a = points[idx]
        b = points[idx + 1]
        return (
            a[0] + (b[0] - a[0]) * ratio,
            a[1] + (b[1] - a[1]) * ratio,
            a[2] + (b[2] - a[2]) * ratio,
        )
    return points[-1]


def resample_polyline(points, interval):
    if len(points) < 2:
        return list(points)
    distances = cumulative_lengths(points)
    total = distances[-1]
    if total <= 1.0e-6:
        return [points[0]]

    interval = max(interval, 0.1)
    samples = []
    sample_count = int(math.floor(total / interval))
    for idx in range(sample_count + 1):
        samples.append(interpolate_polyline(points, idx * interval))
    if distance_2d(samples[-1], points[-1]) > 1.0e-3:
        samples.append(points[-1])
    return samples


def project_point_to_polyline(points, target):
    if not points:
        return None
    if len(points) == 1:
        return 0.0, distance_2d(points[0], target)

    cumulative = cumulative_lengths(points)
    best_s = 0.0
    best_distance = None
    for idx in range(len(points) - 1):
        a = points[idx]
        b = points[idx + 1]
        vx = b[0] - a[0]
        vy = b[1] - a[1]
        length_sq = vx * vx + vy * vy
        if length_sq < 1.0e-9:
            continue
        wx = target[0] - a[0]
        wy = target[1] - a[1]
        ratio = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
        px = a[0] + vx * ratio
        py = a[1] + vy * ratio
        distance = math.hypot(target[0] - px, target[1] - py)
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_s = cumulative[idx] + math.sqrt(length_sq) * ratio
    return best_s, best_distance if best_distance is not None else 0.0


def midpoint_polyline(left, right):
    count = min(len(left), len(right))
    if count < 2:
        return []
    return [
        (
            (left[idx][0] + right[idx][0]) * 0.5,
            (left[idx][1] + right[idx][1]) * 0.5,
            (left[idx][2] + right[idx][2]) * 0.5,
        )
        for idx in range(count)
    ]


class CenterlineMap:
    def __init__(self, source, centerlines):
        self.source = source
        self.centerlines = centerlines

    @classmethod
    def load(cls, map_file, projector_file="", flatten_z=False):
        if not map_file:
            return None
        ext = os.path.splitext(map_file)[1].lower()
        if ext in (".osm", ".xml"):
            return cls.from_osm_file(map_file, projector_file, flatten_z)
        if ext == ".json":
            return cls.from_mgeo_link_file(map_file, flatten_z)
        raise ValueError("Unsupported centerline map file extension: %s" % ext)

    @classmethod
    def from_osm_file(cls, map_file, projector_file="", flatten_z=False):
        projector = read_projector(projector_file) if projector_file else None
        tree = ET.parse(map_file)
        return cls.from_osm_root(tree.getroot(), projector, map_file, flatten_z)

    @classmethod
    def from_osm_bytes(cls, data, source="LaneletMapBin", flatten_z=False):
        root = ET.fromstring(bytes(data).decode("utf-8"))
        return cls.from_osm_root(root, None, source, flatten_z)

    @classmethod
    def from_osm_root(cls, root, projector, source, flatten_z=False):
        nodes = {}
        ways = {}
        relations = []

        for elem in root:
            if elem.tag == "node":
                tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
                node_id = elem.attrib["id"]
                lat = float(elem.attrib["lat"])
                lon = float(elem.attrib["lon"])
                ele = float(tags.get("ele", 0.0))
                point = project_node(lat, lon, ele, projector)
                if flatten_z:
                    point = (point[0], point[1], 0.0)
                nodes[node_id] = point
            elif elem.tag == "way":
                ways[elem.attrib["id"]] = {
                    "refs": [nd.attrib["ref"] for nd in elem if nd.tag == "nd"],
                    "tags": {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"},
                }
            elif elem.tag == "relation":
                relations.append(
                    {
                        "id": elem.attrib["id"],
                        "members": [
                            {
                                "type": m.attrib.get("type"),
                                "ref": m.attrib.get("ref"),
                                "role": m.attrib.get("role", ""),
                            }
                            for m in elem
                            if m.tag == "member"
                        ],
                        "tags": {
                            t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"
                        },
                    }
                )

        centerlines = {}
        for relation in relations:
            if relation["tags"].get("type") != "lanelet":
                continue

            center_id = relation_way_by_role(relation, "centerline")
            points = way_points(center_id, ways, nodes) if center_id else []
            if len(points) < 2:
                left = way_points(relation_way_by_role(relation, "left"), ways, nodes)
                right = way_points(relation_way_by_role(relation, "right"), ways, nodes)
                points = midpoint_polyline(left, right)
            if len(points) >= 2:
                centerlines[str(relation["id"])] = points

        return cls(source, centerlines)

    @classmethod
    def from_mgeo_link_file(cls, link_file, flatten_z=False):
        with open(link_file, "r", encoding="utf-8") as f:
            links = json.load(f)

        centerlines = {}
        for link in links:
            points = []
            for point in link.get("points", []):
                if not isinstance(point, list) or len(point) < 3:
                    continue
                z = 0.0 if flatten_z else float(point[2])
                points.append((float(point[0]), float(point[1]), z))
            if len(points) >= 2:
                centerlines[str(link["idx"])] = points
        return cls(link_file, centerlines)

    def get_centerline(self, lane_id):
        return self.centerlines.get(str(lane_id))

    def has_lane(self, lane_id):
        return str(lane_id) in self.centerlines

    def lane_count(self):
        return len(self.centerlines)


def stitch_route_centerlines(centerline_map, lane_ids, start_point=None):
    stitched = []
    used_lane_ids = []
    for lane_id in lane_ids:
        points = centerline_map.get_centerline(lane_id)
        if not points:
            continue
        points = list(points)

        if not stitched:
            if start_point is not None and distance_2d(start_point, points[-1]) < distance_2d(
                start_point, points[0]
            ):
                points.reverse()
        elif distance_2d(stitched[-1], points[-1]) < distance_2d(stitched[-1], points[0]):
            points.reverse()

        if stitched and distance_2d(stitched[-1], points[0]) < 1.0e-3:
            stitched.extend(points[1:])
        else:
            stitched.extend(points)
        used_lane_ids.append(lane_id)

    return stitched, used_lane_ids


def trim_polyline(points, start_point=None, goal_point=None):
    if len(points) < 2:
        return points

    distances = cumulative_lengths(points)
    start_s = 0.0
    end_s = distances[-1]

    if start_point is not None:
        projected = project_point_to_polyline(points, start_point)
        if projected is not None:
            start_s = max(0.0, min(projected[0], end_s))
    if goal_point is not None:
        projected = project_point_to_polyline(points, goal_point)
        if projected is not None and projected[0] > start_s + 1.0:
            end_s = max(start_s, min(projected[0], end_s))

    trimmed = [interpolate_polyline(points, start_s)]
    for point, distance in zip(points, distances):
        if start_s < distance < end_s:
            trimmed.append(point)
    trimmed.append(interpolate_polyline(points, end_s))

    cleaned = []
    for point in trimmed:
        if point is None:
            continue
        if cleaned and distance_2d(cleaned[-1], point) < 1.0e-3:
            continue
        cleaned.append(point)
    return cleaned
