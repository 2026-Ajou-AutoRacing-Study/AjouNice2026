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
MGRS_GRID_SIZE = 100000.0


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


def read_projector(projector_type, path, first_node):
    configured_type = str(projector_type or "none")
    origin = None
    if path:
        if yaml is None:
            raise RuntimeError("PyYAML is required to read projector files")
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        configured_type = str(cfg.get("projector_type", configured_type))
        map_origin = cfg.get("map_origin")
        if map_origin:
            origin = (
                float(map_origin["latitude"]),
                float(map_origin["longitude"]),
                float(map_origin.get("altitude", 0.0)),
            )

    normalized = configured_type.lower().replace("_", "")
    if normalized in ("none", "raw", "geographic"):
        return None

    reference = origin or first_node
    zone = int(math.floor((reference[1] + 180.0) / 6.0) + 1)
    if normalized == "mgrs":
        first_e, first_n = wgs84_to_utm(first_node[0], first_node[1], zone)
        return {
            "type": "MGRS",
            "zone": zone,
            "origin_alt": 0.0,
            "origin_easting": math.floor(first_e / MGRS_GRID_SIZE) * MGRS_GRID_SIZE,
            "origin_northing": math.floor(first_n / MGRS_GRID_SIZE) * MGRS_GRID_SIZE,
        }
    if normalized == "utm":
        return {
            "type": "UTM",
            "zone": zone,
            "origin_alt": 0.0,
            "origin_easting": 0.0,
            "origin_northing": 0.0,
        }
    if normalized in ("localcartesianutm", "localutm"):
        if origin is None:
            raise ValueError("LocalCartesianUTM requires map_origin in projector_file")
        origin_e, origin_n = wgs84_to_utm(origin[0], origin[1], zone)
        return {
            "type": "LocalCartesianUTM",
            "zone": zone,
            "origin_alt": origin[2],
            "origin_easting": origin_e,
            "origin_northing": origin_n,
        }
    raise ValueError(
        "unsupported projector_type '{}': use MGRS, UTM, or LocalCartesianUTM".format(
            configured_type
        )
    )


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


def project_point_to_polyline(points, target, min_s=None, max_s=None):
    if not points:
        return None
    if len(points) == 1:
        if (min_s is not None and min_s > 0.0) or (
            max_s is not None and max_s < 0.0
        ):
            return None
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
        segment_length = math.sqrt(length_sq)
        segment_start = cumulative[idx]
        segment_end = cumulative[idx + 1]
        allowed_start = segment_start if min_s is None else max(segment_start, min_s)
        allowed_end = segment_end if max_s is None else min(segment_end, max_s)
        if allowed_start > allowed_end:
            continue
        minimum_ratio = (allowed_start - segment_start) / segment_length
        maximum_ratio = (allowed_end - segment_start) / segment_length
        wx = target[0] - a[0]
        wy = target[1] - a[1]
        ratio = max(
            minimum_ratio,
            min(maximum_ratio, (wx * vx + wy * vy) / length_sq),
        )
        px = a[0] + vx * ratio
        py = a[1] + vy * ratio
        distance = math.hypot(target[0] - px, target[1] - py)
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_s = segment_start + segment_length * ratio
    if best_distance is None:
        return None
    return best_s, best_distance


def closest_lane_id(centerline_map, lane_ids, point):
    scored = []
    for lane_id in lane_ids:
        centerline = centerline_map.get_centerline(lane_id)
        if not centerline:
            continue
        projection = project_point_to_polyline(centerline, point)
        if projection is not None:
            scored.append((float(projection[1]), int(lane_id)))
    if not scored:
        return None
    return min(scored)[1]


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
    def __init__(
        self,
        source,
        centerlines,
        boundaries=None,
        boundary_ids=None,
        lane_attributes=None,
        boundary_node_ids=None,
    ):
        self.source = source
        self.centerlines = centerlines
        self.boundaries = boundaries or {}
        self.boundary_ids = boundary_ids or {}
        self.lane_attributes = lane_attributes or {}
        self.boundary_node_ids = boundary_node_ids or {}

    @classmethod
    def load(cls, map_file, projector_file="", flatten_z=False, projector_type="none"):
        if not map_file:
            return None
        ext = os.path.splitext(map_file)[1].lower()
        if ext in (".osm", ".xml"):
            return cls.from_osm_file(
                map_file, projector_file, flatten_z, projector_type
            )
        if ext == ".json":
            return cls.from_mgeo_link_file(map_file, flatten_z)
        raise ValueError("Unsupported centerline map file extension: %s" % ext)

    @classmethod
    def from_osm_file(
        cls, map_file, projector_file="", flatten_z=False, projector_type="none"
    ):
        tree = ET.parse(map_file)
        root = tree.getroot()
        first_node = None
        for elem in root:
            if elem.tag != "node":
                continue
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            first_node = (
                float(elem.attrib["lat"]),
                float(elem.attrib["lon"]),
                float(tags.get("ele", 0.0)),
            )
            break
        if first_node is None:
            raise ValueError("Lanelet2 OSM does not contain a node")
        projector = read_projector(projector_type, projector_file, first_node)
        return cls.from_osm_root(root, projector, map_file, flatten_z)

    @classmethod
    def from_osm_bytes(
        cls, data, source="LaneletMapBin", flatten_z=False, projector_type="none"
    ):
        root = ET.fromstring(bytes(data).decode("utf-8"))
        first_node = None
        for elem in root:
            if elem.tag != "node":
                continue
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            first_node = (
                float(elem.attrib["lat"]),
                float(elem.attrib["lon"]),
                float(tags.get("ele", 0.0)),
            )
            break
        if first_node is None:
            raise ValueError("Lanelet2 OSM does not contain a node")
        projector = read_projector(projector_type, "", first_node)
        return cls.from_osm_root(root, projector, source, flatten_z)

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
        boundaries = {}
        boundary_ids = {}
        boundary_node_ids = {}
        lane_attributes = {}
        for relation in relations:
            if relation["tags"].get("type") != "lanelet":
                continue

            center_id = relation_way_by_role(relation, "centerline")
            left_id = relation_way_by_role(relation, "left")
            right_id = relation_way_by_role(relation, "right")
            points = way_points(center_id, ways, nodes) if center_id else []
            left = way_points(left_id, ways, nodes)
            right = way_points(right_id, ways, nodes)
            if len(points) < 2:
                points = midpoint_polyline(left, right)
            if len(points) >= 2:
                centerlines[str(relation["id"])] = points
                lane_attributes[str(relation["id"])] = dict(relation["tags"])
            if len(left) >= 2 and len(right) >= 2:
                boundaries[str(relation["id"])] = (left, right)
                boundary_ids[str(relation["id"])] = (left_id, right_id)
                boundary_node_ids[str(relation["id"])] = (
                    (ways[left_id]["refs"][0], ways[left_id]["refs"][-1]),
                    (ways[right_id]["refs"][0], ways[right_id]["refs"][-1]),
                )

        return cls(
            source,
            centerlines,
            boundaries,
            boundary_ids,
            lane_attributes,
            boundary_node_ids,
        )

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

    def get_boundaries(self, lane_id):
        return self.boundaries.get(str(lane_id))

    def get_boundary_ids(self, lane_id):
        return self.boundary_ids.get(str(lane_id))

    def get_boundary_node_ids(self, lane_id):
        return self.boundary_node_ids.get(str(lane_id))

    def get_lane_attributes(self, lane_id):
        return self.lane_attributes.get(str(lane_id), {})

    def has_lane(self, lane_id):
        return str(lane_id) in self.centerlines

    def lane_count(self):
        return len(self.centerlines)

    def same_direction(self, first_lane_id, second_lane_id):
        return self._same_direction(first_lane_id, second_lane_id)

    def is_following(self, first_lane_id, second_lane_id, geometric_tolerance=1.0):
        """Return whether second_lane_id is a longitudinal successor."""
        first_nodes = self.get_boundary_node_ids(first_lane_id)
        second_nodes = self.get_boundary_node_ids(second_lane_id)
        if first_nodes is not None and second_nodes is not None:
            return (
                first_nodes[0][1] == second_nodes[0][0]
                and first_nodes[1][1] == second_nodes[1][0]
            )

        first = self.get_centerline(first_lane_id)
        second = self.get_centerline(second_lane_id)
        if not first or not second or not self._same_direction(first_lane_id, second_lane_id):
            return False
        return distance_2d(first[-1], second[0]) <= max(float(geometric_tolerance), 0.0)

    def is_lateral_neighbor(self, first_lane_id, second_lane_id):
        first_ids = self.get_boundary_ids(first_lane_id)
        second_ids = self.get_boundary_ids(second_lane_id)
        if first_ids is None or second_ids is None:
            return False
        return bool(set(first_ids) & set(second_ids)) and self._same_direction(
            first_lane_id, second_lane_id
        )

    def _same_direction(self, first_lane_id, second_lane_id):
        first = self.get_centerline(first_lane_id)
        second = self.get_centerline(second_lane_id)
        if not first or not second:
            return True
        first_vector = (first[-1][0] - first[0][0], first[-1][1] - first[0][1])
        second_vector = (second[-1][0] - second[0][0], second[-1][1] - second[0][1])
        return first_vector[0] * second_vector[0] + first_vector[1] * second_vector[1] >= 0.0

    def adjacent_lane(self, lane_id, side, use_lane_type="opposite_direction_lane"):
        """Return the lane and its outer side across a shared Lanelet boundary."""
        if use_lane_type == "current_lane":
            return None

        ids = self.get_boundary_ids(lane_id)
        if ids is None:
            return None
        side_index = 0 if side == "left" else 1
        shared_id = ids[side_index]
        if shared_id is None:
            return None

        for candidate_id, candidate_ids in self.boundary_ids.items():
            if str(candidate_id) == str(lane_id) or shared_id not in candidate_ids:
                continue
            if (
                use_lane_type == "same_direction_lane"
                and not self._same_direction(lane_id, candidate_id)
            ):
                continue
            shared_candidate_side = "left" if candidate_ids[0] == shared_id else "right"
            outer_side = "right" if shared_candidate_side == "left" else "left"
            return int(candidate_id), outer_side
        return None

    def expanded_outer_boundary(
        self, lane_id, side, use_lane_type="opposite_direction_lane", max_depth=8
    ):
        """Get the outermost boundary reachable through shared lane boundaries."""
        current_lane_id = int(lane_id)
        current_side = side
        visited = {current_lane_id}
        boundary = None

        for _ in range(max(int(max_depth), 1)):
            boundaries = self.get_boundaries(current_lane_id)
            if boundaries is None:
                break
            boundary = boundaries[0 if current_side == "left" else 1]
            adjacent = self.adjacent_lane(
                current_lane_id, current_side, use_lane_type
            )
            if adjacent is None or adjacent[0] in visited:
                break
            current_lane_id, current_side = adjacent
            visited.add(current_lane_id)
        return boundary

    def expanded_bound_distances(
        self, lane_id, point, use_lane_type="opposite_direction_lane"
    ):
        """Return path-center distances to expanded left and right drivable bounds."""
        distances = []
        for side in ("left", "right"):
            boundary = self.expanded_outer_boundary(lane_id, side, use_lane_type)
            projection = project_point_to_polyline(boundary, point) if boundary else None
            distances.append(float(projection[1]) if projection is not None else None)
        return tuple(distances)

    def lane_ids_near_point(self, route_lane_ids, point, tolerance=0.3):
        scored = []
        for lane_id in route_lane_ids:
            centerline = self.get_centerline(lane_id)
            if not centerline:
                continue
            projection = project_point_to_polyline(centerline, point)
            if projection is not None:
                scored.append((float(projection[1]), int(lane_id)))
        if not scored:
            return list(route_lane_ids)

        scored.sort()
        best_distance = scored[0][0]
        cutoff = (
            float(tolerance)
            if best_distance <= float(tolerance)
            else best_distance + 0.05
        )
        selected = {
            lane_id
            for distance, lane_id in scored
            if distance <= cutoff
        }
        return [int(lane_id) for lane_id in route_lane_ids if int(lane_id) in selected]


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


def stitch_route_bounds(centerline_map, lane_ids, start_point=None):
    stitched_left = []
    stitched_right = []
    previous_center = None
    for lane_id in lane_ids:
        center = centerline_map.get_centerline(lane_id)
        boundaries = centerline_map.get_boundaries(lane_id)
        if not center or not boundaries:
            continue

        center = list(center)
        left = list(boundaries[0])
        right = list(boundaries[1])
        reverse = False
        if previous_center is None:
            reverse = start_point is not None and distance_2d(
                start_point, center[-1]
            ) < distance_2d(start_point, center[0])
        else:
            reverse = distance_2d(previous_center, center[-1]) < distance_2d(
                previous_center, center[0]
            )

        if reverse:
            center.reverse()
            left, right = list(reversed(right)), list(reversed(left))

        for output, points in ((stitched_left, left), (stitched_right, right)):
            if output and distance_2d(output[-1], points[-1]) < distance_2d(
                output[-1], points[0]
            ):
                points.reverse()
            if output and distance_2d(output[-1], points[0]) < 1.0e-3:
                output.extend(points[1:])
            else:
                output.extend(points)
        previous_center = center[-1]

    return stitched_left, stitched_right


def trim_polyline(
    points,
    start_point=None,
    goal_point=None,
    start_offset=0.0,
    end_offset=0.0,
    include_previous_start=False,
    max_forward_length=None,
):
    if len(points) < 2:
        return points

    distances = cumulative_lengths(points)
    start_s = 0.0
    end_s = distances[-1]
    reference_start_s = None

    if start_point is not None:
        projected = project_point_to_polyline(points, start_point)
        if projected is not None:
            reference_start_s = projected[0]
            start_s = max(0.0, min(reference_start_s + float(start_offset), end_s))
            if include_previous_start:
                previous = [distance for distance in distances if distance <= start_s]
                if previous:
                    start_s = previous[-1]
    if goal_point is not None:
        projected = project_point_to_polyline(
            points, goal_point, min_s=start_s + 1.0
        )
        if projected is not None and projected[0] > start_s + 1.0:
            end_s = max(start_s, min(projected[0] + float(end_offset), end_s))
    if max_forward_length is not None and reference_start_s is not None:
        end_s = min(
            end_s,
            reference_start_s + max(float(max_forward_length), 0.0),
        )

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
