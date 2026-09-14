import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996
MGRS_GRID_SIZE = 100000.0


@dataclass(frozen=True)
class MapLineString:
    line_id: int
    points: Tuple[Tuple[float, float, float], ...]
    tags: Dict[str, str]
    regulatory_subtypes: Tuple[str, ...] = ()
    lane_ids: Tuple[int, ...] = ()

    @property
    def is_traffic_light_ref_line(self):
        return "traffic_light" in self.regulatory_subtypes


@dataclass(frozen=True)
class MapSpeedBump:
    regulatory_id: int
    polygon_id: int
    points: Tuple[Tuple[float, float, float], ...]
    tags: Dict[str, str]
    lane_ids: Tuple[int, ...] = ()


@dataclass(frozen=True)
class MapNoStoppingArea:
    regulatory_id: int
    polygon_id: int
    points: Tuple[Tuple[float, float, float], ...]
    tags: Dict[str, str]
    lane_ids: Tuple[int, ...] = ()
    stop_line: Optional[MapLineString] = None


@dataclass(frozen=True)
class MapDetectionArea:
    regulatory_id: int
    polygons: Tuple[Tuple[Tuple[float, float, float], ...], ...]
    polygon_ids: Tuple[int, ...]
    lane_ids: Tuple[int, ...] = ()
    stop_line: Optional[MapLineString] = None


@dataclass(frozen=True)
class MapVirtualTrafficLight:
    regulatory_id: int
    instrument: MapLineString
    start_line: MapLineString
    stop_line: Optional[MapLineString]
    end_lines: Tuple[MapLineString, ...]
    lane_ids: Tuple[int, ...] = ()


@dataclass(frozen=True)
class MapTrafficLight:
    regulatory_id: int
    stop_line_id: int
    stop_line_points: Tuple[Tuple[float, float, float], ...]
    traffic_light_ids: Tuple[int, ...]
    lane_ids: Tuple[int, ...]
    lane_turn_directions: Tuple[Tuple[int, str], ...]
    has_static_arrow: bool = False

    def turn_direction(self, lane_id):
        return dict(self.lane_turn_directions).get(int(lane_id), "else")


@dataclass(frozen=True)
class MapPedestrianCrossing:
    lanelet_id: int
    subtype: str
    points: Tuple[Tuple[float, float, float], ...]
    tags: Dict[str, str]
    stop_lines: Tuple[MapLineString, ...] = ()
    road_lane_ids: Tuple[int, ...] = ()


@dataclass(frozen=True)
class MapRoadLanelet:
    lanelet_id: int
    left_bound: Tuple[Tuple[float, float, float], ...]
    right_bound: Tuple[Tuple[float, float, float], ...]
    centerline: Tuple[Tuple[float, float, float], ...]
    polygon: Tuple[Tuple[float, float, float], ...]
    tags: Dict[str, str]
    regulatory_ids: Tuple[int, ...] = ()
    previous_ids: Tuple[int, ...] = ()
    following_ids: Tuple[int, ...] = ()

    @property
    def turn_direction(self):
        return self.tags.get("turn_direction", "else")

    @property
    def intersection_area_id(self):
        value = self.tags.get("intersection_area")
        try:
            return int(value) if value is not None else None
        except (TypeError, ValueError):
            return None


@dataclass(frozen=True)
class MapIntersectionArea:
    area_id: int
    points: Tuple[Tuple[float, float, float], ...]
    tags: Dict[str, str]


@dataclass(frozen=True)
class MapRightOfWay:
    regulatory_id: int
    right_of_way_lane_ids: Tuple[int, ...]
    yield_lane_ids: Tuple[int, ...]


def _resample_polyline(points, count):
    if not points:
        return []
    if len(points) == 1 or count <= 1:
        return [points[0]] * max(int(count), 1)
    lengths = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        lengths.append(
            lengths[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    total = lengths[-1]
    if total <= 1.0e-9:
        return [points[0]] * count
    output = []
    for index in range(count):
        station = total * index / float(count - 1)
        segment = 0
        while segment + 1 < len(lengths) and lengths[segment + 1] < station:
            segment += 1
        following = min(segment + 1, len(points) - 1)
        segment_length = max(lengths[following] - lengths[segment], 1.0e-9)
        ratio = (station - lengths[segment]) / segment_length
        output.append(
            tuple(
                float(points[segment][axis])
                + (float(points[following][axis]) - float(points[segment][axis]))
                * ratio
                for axis in range(3)
            )
        )
    return output


def _centerline_from_bounds(left_bound, right_bound):
    count = max(len(left_bound), len(right_bound), 2)
    left = _resample_polyline(left_bound, count)
    right = _resample_polyline(right_bound, count)
    return tuple(
        (
            0.5 * (left_point[0] + right_point[0]),
            0.5 * (left_point[1] + right_point[1]),
            0.5 * (left_point[2] + right_point[2]),
        )
        for left_point, right_point in zip(left, right)
    )


def _bbox(points):
    return (
        min(point[0] for point in points),
        min(point[1] for point in points),
        max(point[0] for point in points),
        max(point[1] for point in points),
    )


def _bbox_intersects(first, second, margin=0.0):
    return not (
        first[2] + margin < second[0]
        or second[2] + margin < first[0]
        or first[3] + margin < second[1]
        or second[3] + margin < first[1]
    )


def _orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (
        c[0] - a[0]
    )


def _segments_intersect(a, b, c, d, tolerance=1.0e-8):
    first = _orientation(a, b, c)
    second = _orientation(a, b, d)
    third = _orientation(c, d, a)
    fourth = _orientation(c, d, b)
    if first * second < -tolerance and third * fourth < -tolerance:
        return True

    def on_segment(start, end, point):
        return (
            abs(_orientation(start, end, point)) <= tolerance
            and min(start[0], end[0]) - tolerance
            <= point[0]
            <= max(start[0], end[0]) + tolerance
            and min(start[1], end[1]) - tolerance
            <= point[1]
            <= max(start[1], end[1]) + tolerance
        )

    return any(
        (
            on_segment(a, b, c),
            on_segment(a, b, d),
            on_segment(c, d, a),
            on_segment(c, d, b),
        )
    )


def _point_in_polygon(point, polygon):
    if len(polygon) < 3:
        return False
    x, y = float(point[0]), float(point[1])
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous[0], previous[1]
        x2, y2 = current[0], current[1]
        if (y1 > y) != (y2 > y):
            boundary_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < boundary_x:
                inside = not inside
        previous = current
    return inside


def _polygons_intersect(first, second):
    if not _bbox_intersects(_bbox(first), _bbox(second)):
        return False
    first_edges = list(zip(first, first[1:] + first[:1]))
    second_edges = list(zip(second, second[1:] + second[:1]))
    if any(
        _segments_intersect(first_a, first_b, second_a, second_b)
        for first_a, first_b in first_edges
        for second_a, second_b in second_edges
    ):
        return True
    return _point_in_polygon(first[0], second) or _point_in_polygon(second[0], first)


def _wgs84_to_utm(lat_deg, lon_deg, zone):
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
        + n
        * tan_lat
        * (
            a**2 / 2
            + (5 - t + 9 * c + 4 * c**2) * a**4 / 24
            + (61 - 58 * t + t**2 + 600 * c - 330 * ep2) * a**6 / 720
        )
    )
    if lat_deg < 0.0:
        northing += 10000000.0
    return easting, northing


def _read_projector(projector_type, projector_file, first_node):
    configured_type = str(projector_type or "none")
    origin = None
    if projector_file:
        if yaml is None:
            raise RuntimeError("PyYAML is required to read projector files")
        with open(projector_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}
        configured_type = str(config.get("projector_type", configured_type))
        map_origin = config.get("map_origin")
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
        first_easting, first_northing = _wgs84_to_utm(first_node[0], first_node[1], zone)
        return {
            "type": "MGRS",
            "zone": zone,
            "origin_alt": 0.0,
            "origin_easting": math.floor(first_easting / MGRS_GRID_SIZE) * MGRS_GRID_SIZE,
            "origin_northing": math.floor(first_northing / MGRS_GRID_SIZE) * MGRS_GRID_SIZE,
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
        origin_easting, origin_northing = _wgs84_to_utm(origin[0], origin[1], zone)
        return {
            "type": "LocalCartesianUTM",
            "zone": zone,
            "origin_alt": origin[2],
            "origin_easting": origin_easting,
            "origin_northing": origin_northing,
        }
    raise ValueError("unsupported projector_type '{}'".format(configured_type))


def _project_node(lat, lon, elevation, projector):
    if projector is None:
        return float(lon), float(lat), float(elevation)
    easting, northing = _wgs84_to_utm(lat, lon, projector["zone"])
    return (
        easting - projector["origin_easting"],
        northing - projector["origin_northing"],
        float(elevation) - projector["origin_alt"],
    )


class BehaviorVelocityMap:
    def __init__(
        self,
        source,
        stop_lines=None,
        speed_bumps=None,
        no_stopping_areas=None,
        traffic_lights=None,
        pedestrian_crossings=None,
        road_lanelets=None,
        intersection_areas=None,
        right_of_ways=None,
        detection_areas=None,
        virtual_traffic_lights=None,
    ):
        self.source = source
        self.stop_lines = stop_lines or {}
        self.speed_bumps = speed_bumps or {}
        self.no_stopping_areas = no_stopping_areas or {}
        self.traffic_lights = traffic_lights or {}
        self.pedestrian_crossings = pedestrian_crossings or {}
        self.road_lanelets = road_lanelets or {}
        self.intersection_areas = intersection_areas or {}
        self.right_of_ways = right_of_ways or {}
        self.detection_areas = detection_areas or {}
        self.virtual_traffic_lights = virtual_traffic_lights or {}
        self._conflicting_lanelets = {}

    @classmethod
    def load(cls, map_file, projector_file="", projector_type="MGRS", flatten_z=False):
        if not map_file:
            return None
        extension = os.path.splitext(map_file)[1].lower()
        if extension not in (".osm", ".xml"):
            raise ValueError("BehaviorVelocityMap requires a Lanelet2 OSM/XML file")
        root = ET.parse(map_file).getroot()
        return cls._from_root(root, map_file, projector_file, projector_type, flatten_z)

    @classmethod
    def from_osm_bytes(
        cls,
        data,
        source="LaneletMapBin",
        projector_file="",
        projector_type="MGRS",
        flatten_z=False,
    ):
        root = ET.fromstring(bytes(data).decode("utf-8"))
        return cls._from_root(root, source, projector_file, projector_type, flatten_z)

    @classmethod
    def _from_root(cls, root, source, projector_file, projector_type, flatten_z):
        first_node = None
        for element in root:
            if element.tag != "node":
                continue
            tags = {
                tag.attrib.get("k"): tag.attrib.get("v")
                for tag in element
                if tag.tag == "tag"
            }
            first_node = (
                float(element.attrib["lat"]),
                float(element.attrib["lon"]),
                float(tags.get("ele", 0.0)),
            )
            break
        if first_node is None:
            raise ValueError("Lanelet2 OSM does not contain a node")
        projector = _read_projector(projector_type, projector_file, first_node)

        nodes = {}
        ways = {}
        relations = {}
        for element in root:
            if element.tag == "node":
                tags = {
                    tag.attrib.get("k"): tag.attrib.get("v")
                    for tag in element
                    if tag.tag == "tag"
                }
                point = _project_node(
                    float(element.attrib["lat"]),
                    float(element.attrib["lon"]),
                    float(tags.get("ele", 0.0)),
                    projector,
                )
                if flatten_z:
                    point = (point[0], point[1], 0.0)
                nodes[element.attrib["id"]] = point
            elif element.tag == "way":
                ways[element.attrib["id"]] = {
                    "refs": [item.attrib["ref"] for item in element if item.tag == "nd"],
                    "tags": {
                        tag.attrib.get("k"): tag.attrib.get("v")
                        for tag in element
                        if tag.tag == "tag"
                    },
                }
            elif element.tag == "relation":
                relations[element.attrib["id"]] = {
                    "members": [
                        {
                            "type": member.attrib.get("type"),
                            "ref": member.attrib.get("ref"),
                            "role": member.attrib.get("role", ""),
                        }
                        for member in element
                        if member.tag == "member"
                    ],
                    "tags": {
                        tag.attrib.get("k"): tag.attrib.get("v")
                        for tag in element
                        if tag.tag == "tag"
                    },
                }

        regulatory_lane_ids = {}
        lanelet_tags = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "lanelet":
                continue
            lane_id = int(relation_id)
            lanelet_tags[lane_id] = dict(relation["tags"])
            for member in relation["members"]:
                if member["type"] == "relation" and member["role"] == "regulatory_element":
                    regulatory_lane_ids.setdefault(member["ref"], set()).add(lane_id)

        raw_road_lanelets = {}
        for relation_id, relation in relations.items():
            tags = relation["tags"]
            if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
                continue
            members_by_role = {
                member["role"]: member["ref"]
                for member in relation["members"]
                if member["type"] == "way"
                and member["role"] in ("left", "right", "centerline")
            }
            left_way = ways.get(members_by_role.get("left"))
            right_way = ways.get(members_by_role.get("right"))
            if left_way is None or right_way is None:
                continue
            left_bound = tuple(
                nodes[node_id] for node_id in left_way["refs"] if node_id in nodes
            )
            right_bound = tuple(
                nodes[node_id] for node_id in right_way["refs"] if node_id in nodes
            )
            if len(left_bound) < 2 or len(right_bound) < 2:
                continue
            centerline_way = ways.get(members_by_role.get("centerline"))
            centerline = ()
            if centerline_way is not None:
                centerline = tuple(
                    nodes[node_id]
                    for node_id in centerline_way["refs"]
                    if node_id in nodes
                )
            if len(centerline) < 2:
                centerline = _centerline_from_bounds(left_bound, right_bound)
            polygon = tuple(left_bound + tuple(reversed(right_bound)))
            left_refs = left_way["refs"]
            right_refs = right_way["refs"]
            start_key = tuple(sorted((left_refs[0], right_refs[0])))
            end_key = tuple(sorted((left_refs[-1], right_refs[-1])))
            regulatory_ids = tuple(
                sorted(
                    int(member["ref"])
                    for member in relation["members"]
                    if member["type"] == "relation"
                    and member["role"] == "regulatory_element"
                )
            )
            raw_road_lanelets[int(relation_id)] = {
                "left_bound": left_bound,
                "right_bound": right_bound,
                "centerline": centerline,
                "polygon": polygon,
                "tags": dict(tags),
                "regulatory_ids": regulatory_ids,
                "start_key": start_key,
                "end_key": end_key,
            }

        starts = {}
        ends = {}
        for lanelet_id, lanelet in raw_road_lanelets.items():
            starts.setdefault(lanelet["start_key"], set()).add(lanelet_id)
            ends.setdefault(lanelet["end_key"], set()).add(lanelet_id)
        road_lanelets = {}
        for lanelet_id, lanelet in raw_road_lanelets.items():
            road_lanelets[lanelet_id] = MapRoadLanelet(
                lanelet_id=lanelet_id,
                left_bound=lanelet["left_bound"],
                right_bound=lanelet["right_bound"],
                centerline=lanelet["centerline"],
                polygon=lanelet["polygon"],
                tags=lanelet["tags"],
                regulatory_ids=lanelet["regulatory_ids"],
                previous_ids=tuple(
                    sorted(ends.get(lanelet["start_key"], set()) - {lanelet_id})
                ),
                following_ids=tuple(
                    sorted(starts.get(lanelet["end_key"], set()) - {lanelet_id})
                ),
            )

        intersection_areas = {}
        for way_id, way in ways.items():
            if way["tags"].get("type") != "intersection_area":
                continue
            points = tuple(nodes[node_id] for node_id in way["refs"] if node_id in nodes)
            if len(points) >= 3:
                intersection_areas[int(way_id)] = MapIntersectionArea(
                    area_id=int(way_id), points=points, tags=dict(way["tags"])
                )

        right_of_ways = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "regulatory_element":
                continue
            if relation["tags"].get("subtype") != "right_of_way":
                continue
            right_of_way_lane_ids = []
            yield_lane_ids = []
            for member in relation["members"]:
                if member["type"] != "relation":
                    continue
                if member["role"] == "right_of_way":
                    right_of_way_lane_ids.append(int(member["ref"]))
                elif member["role"] == "yield":
                    yield_lane_ids.append(int(member["ref"]))
            right_of_ways[int(relation_id)] = MapRightOfWay(
                regulatory_id=int(relation_id),
                right_of_way_lane_ids=tuple(sorted(right_of_way_lane_ids)),
                yield_lane_ids=tuple(sorted(yield_lane_ids)),
            )

        line_regulatory_subtypes = {}
        line_lane_ids = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "regulatory_element":
                continue
            subtype = relation["tags"].get("subtype", "")
            for member in relation["members"]:
                if member["type"] != "way" or member["role"] != "ref_line":
                    continue
                line_regulatory_subtypes.setdefault(member["ref"], set()).add(subtype)
                line_lane_ids.setdefault(member["ref"], set()).update(
                    regulatory_lane_ids.get(relation_id, set())
                )

        stop_lines = {}
        for way_id, way in ways.items():
            if way["tags"].get("subtype") != "stop_line":
                continue
            points = tuple(nodes[node_id] for node_id in way["refs"] if node_id in nodes)
            if len(points) < 2:
                continue
            stop_lines[int(way_id)] = MapLineString(
                line_id=int(way_id),
                points=points,
                tags=dict(way["tags"]),
                regulatory_subtypes=tuple(sorted(line_regulatory_subtypes.get(way_id, set()))),
                lane_ids=tuple(sorted(line_lane_ids.get(way_id, set()))),
            )
        speed_bumps = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "regulatory_element":
                continue
            if relation["tags"].get("subtype") != "speed_bump":
                continue
            referred_ways = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "refers"
            ]
            for way_id in referred_ways:
                way = ways.get(way_id)
                if way is None or way["tags"].get("type") != "speed_bump":
                    continue
                points = tuple(
                    nodes[node_id] for node_id in way["refs"] if node_id in nodes
                )
                if len(points) < 3:
                    continue
                speed_bumps[int(relation_id)] = MapSpeedBump(
                    regulatory_id=int(relation_id),
                    polygon_id=int(way_id),
                    points=points,
                    tags=dict(way["tags"]),
                    lane_ids=tuple(
                        sorted(regulatory_lane_ids.get(relation_id, set()))
                    ),
                )
        no_stopping_areas = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "regulatory_element":
                continue
            if relation["tags"].get("subtype") != "no_stopping_area":
                continue
            referred_ways = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "refers"
            ]
            ref_line_ids = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "ref_line"
            ]
            stop_line = None
            if ref_line_ids:
                stop_line_way = ways.get(ref_line_ids[0])
                if stop_line_way is not None:
                    stop_line_points = tuple(
                        nodes[node_id]
                        for node_id in stop_line_way["refs"]
                        if node_id in nodes
                    )
                    if len(stop_line_points) >= 2:
                        stop_line = MapLineString(
                            line_id=int(ref_line_ids[0]),
                            points=stop_line_points,
                            tags=dict(stop_line_way["tags"]),
                            regulatory_subtypes=("no_stopping_area",),
                            lane_ids=tuple(
                                sorted(regulatory_lane_ids.get(relation_id, set()))
                            ),
                        )
            for way_id in referred_ways:
                way = ways.get(way_id)
                if way is None or way["tags"].get("type") != "no_stopping_area":
                    continue
                points = tuple(
                    nodes[node_id] for node_id in way["refs"] if node_id in nodes
                )
                if len(points) < 3:
                    continue
                no_stopping_areas[int(relation_id)] = MapNoStoppingArea(
                    regulatory_id=int(relation_id),
                    polygon_id=int(way_id),
                    points=points,
                    tags=dict(way["tags"]),
                    lane_ids=tuple(
                        sorted(regulatory_lane_ids.get(relation_id, set()))
                    ),
                    stop_line=stop_line,
                )
                break

        def map_line(way_id, regulatory_subtype, lane_ids):
            way = ways.get(str(way_id))
            if way is None:
                return None
            points = tuple(nodes[node_id] for node_id in way["refs"] if node_id in nodes)
            if len(points) < 2:
                return None
            return MapLineString(
                line_id=int(way_id),
                points=points,
                tags=dict(way["tags"]),
                regulatory_subtypes=(regulatory_subtype,),
                lane_ids=tuple(sorted(lane_ids)),
            )

        detection_areas = {}
        for relation_id, relation in relations.items():
            tags = relation["tags"]
            if tags.get("type") != "regulatory_element" or tags.get("subtype") != "detection_area":
                continue
            lane_ids = tuple(sorted(regulatory_lane_ids.get(relation_id, set())))
            polygon_ids = [
                int(member["ref"])
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "refers"
            ]
            polygons = []
            valid_polygon_ids = []
            for polygon_id in polygon_ids:
                way = ways.get(str(polygon_id))
                if way is None:
                    continue
                points = tuple(nodes[node_id] for node_id in way["refs"] if node_id in nodes)
                if len(points) >= 3:
                    polygons.append(points)
                    valid_polygon_ids.append(polygon_id)
            stop_line_ids = [
                int(member["ref"])
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "ref_line"
            ]
            stop_line = (
                map_line(stop_line_ids[0], "detection_area", lane_ids)
                if stop_line_ids
                else None
            )
            if polygons:
                detection_areas[int(relation_id)] = MapDetectionArea(
                    regulatory_id=int(relation_id),
                    polygons=tuple(polygons),
                    polygon_ids=tuple(valid_polygon_ids),
                    lane_ids=lane_ids,
                    stop_line=stop_line,
                )

        virtual_traffic_lights = {}
        for relation_id, relation in relations.items():
            tags = relation["tags"]
            if tags.get("type") != "regulatory_element" or tags.get("subtype") != "virtual_traffic_light":
                continue
            lane_ids = tuple(sorted(regulatory_lane_ids.get(relation_id, set())))
            role_ids = {}
            for member in relation["members"]:
                if member["type"] == "way":
                    role_ids.setdefault(member["role"], []).append(int(member["ref"]))
            instrument = (
                map_line(role_ids["refers"][0], "virtual_traffic_light", lane_ids)
                if role_ids.get("refers")
                else None
            )
            start_line = (
                map_line(role_ids["start_line"][0], "virtual_traffic_light", lane_ids)
                if role_ids.get("start_line")
                else None
            )
            stop_line = (
                map_line(role_ids["ref_line"][0], "virtual_traffic_light", lane_ids)
                if role_ids.get("ref_line")
                else None
            )
            end_lines = tuple(
                line
                for line in (
                    map_line(line_id, "virtual_traffic_light", lane_ids)
                    for line_id in role_ids.get("end_line", [])
                )
                if line is not None
            )
            if instrument is not None and start_line is not None and end_lines:
                virtual_traffic_lights[int(relation_id)] = MapVirtualTrafficLight(
                    regulatory_id=int(relation_id),
                    instrument=instrument,
                    start_line=start_line,
                    stop_line=stop_line,
                    end_lines=end_lines,
                    lane_ids=lane_ids,
                )
        traffic_lights = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "regulatory_element":
                continue
            if relation["tags"].get("subtype") != "traffic_light":
                continue
            stop_line_ids = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "ref_line"
            ]
            traffic_light_ids = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "refers"
            ]
            if not stop_line_ids:
                continue
            stop_line_id = stop_line_ids[0]
            stop_line_way = ways.get(stop_line_id)
            if stop_line_way is None:
                continue
            stop_line_points = tuple(
                nodes[node_id]
                for node_id in stop_line_way["refs"]
                if node_id in nodes
            )
            if len(stop_line_points) < 2:
                continue
            lane_ids = tuple(sorted(regulatory_lane_ids.get(relation_id, set())))
            lane_turn_directions = tuple(
                (lane_id, lanelet_tags.get(lane_id, {}).get("turn_direction", "else"))
                for lane_id in lane_ids
            )
            has_static_arrow = any(
                "arrow" in ways.get(light_id, {}).get("tags", {}).get("subtype", "")
                for light_id in traffic_light_ids
            )
            traffic_lights[int(relation_id)] = MapTrafficLight(
                regulatory_id=int(relation_id),
                stop_line_id=int(stop_line_id),
                stop_line_points=stop_line_points,
                traffic_light_ids=tuple(int(item) for item in traffic_light_ids),
                lane_ids=lane_ids,
                lane_turn_directions=lane_turn_directions,
                has_static_arrow=has_static_arrow,
            )

        crossing_regulatory_elements = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "regulatory_element":
                continue
            if relation["tags"].get("subtype") != "crosswalk":
                continue
            referred_lanelets = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "relation" and member["role"] == "refers"
            ]
            ref_line_ids = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "ref_line"
            ]
            for lanelet_id in referred_lanelets:
                crossing_regulatory_elements.setdefault(lanelet_id, []).append(
                    (relation_id, ref_line_ids)
                )

        pedestrian_crossings = {}
        for relation_id, relation in relations.items():
            if relation["tags"].get("type") != "lanelet":
                continue
            subtype = relation["tags"].get("subtype", "")
            if subtype not in ("crosswalk", "walkway"):
                continue
            left_ids = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "left"
            ]
            right_ids = [
                member["ref"]
                for member in relation["members"]
                if member["type"] == "way" and member["role"] == "right"
            ]
            if not left_ids or not right_ids:
                continue
            left_way = ways.get(left_ids[0])
            right_way = ways.get(right_ids[0])
            if left_way is None or right_way is None:
                continue
            left_points = [
                nodes[node_id] for node_id in left_way["refs"] if node_id in nodes
            ]
            right_points = [
                nodes[node_id] for node_id in right_way["refs"] if node_id in nodes
            ]
            polygon = tuple(left_points + list(reversed(right_points)))
            if len(polygon) < 3:
                continue

            crossing_stop_lines = []
            road_lane_ids = set()
            for regulatory_id, ref_line_ids in crossing_regulatory_elements.get(
                relation_id, []
            ):
                road_lane_ids.update(regulatory_lane_ids.get(regulatory_id, set()))
                for ref_line_id in ref_line_ids:
                    line = stop_lines.get(int(ref_line_id))
                    if line is not None:
                        crossing_stop_lines.append(line)
            pedestrian_crossings[int(relation_id)] = MapPedestrianCrossing(
                lanelet_id=int(relation_id),
                subtype=subtype,
                points=polygon,
                tags=dict(relation["tags"]),
                stop_lines=tuple(crossing_stop_lines),
                road_lane_ids=tuple(sorted(road_lane_ids)),
            )
        return cls(
            source=source,
            stop_lines=stop_lines,
            speed_bumps=speed_bumps,
            no_stopping_areas=no_stopping_areas,
            traffic_lights=traffic_lights,
            pedestrian_crossings=pedestrian_crossings,
            road_lanelets=road_lanelets,
            intersection_areas=intersection_areas,
            right_of_ways=right_of_ways,
            detection_areas=detection_areas,
            virtual_traffic_lights=virtual_traffic_lights,
        )

    def stop_line_candidates(self, include_traffic_light_ref_lines=False):
        candidates = self.stop_lines.values()
        if not include_traffic_light_ref_lines:
            candidates = [line for line in candidates if not line.is_traffic_light_ref_line]
        return sorted(candidates, key=lambda line: line.line_id)

    def stop_line_count(self, include_traffic_light_ref_lines=True):
        return len(self.stop_line_candidates(include_traffic_light_ref_lines))

    def speed_bump_candidates(self):
        return sorted(
            self.speed_bumps.values(), key=lambda speed_bump: speed_bump.regulatory_id
        )

    def speed_bump_count(self):
        return len(self.speed_bumps)

    def no_stopping_area_candidates(self):
        return sorted(
            self.no_stopping_areas.values(),
            key=lambda area: area.regulatory_id,
        )

    def no_stopping_area_count(self):
        return len(self.no_stopping_areas)

    def detection_area_candidates(self):
        return sorted(
            self.detection_areas.values(), key=lambda area: area.regulatory_id
        )

    def detection_area_count(self):
        return len(self.detection_areas)

    def virtual_traffic_light_candidates(self):
        return sorted(
            self.virtual_traffic_lights.values(),
            key=lambda element: element.regulatory_id,
        )

    def virtual_traffic_light_count(self):
        return len(self.virtual_traffic_lights)

    def traffic_light_candidates(self):
        return sorted(
            self.traffic_lights.values(),
            key=lambda traffic_light: traffic_light.regulatory_id,
        )

    def traffic_light_count(self):
        return len(self.traffic_lights)

    def pedestrian_crossing_candidates(self, subtype=None):
        candidates = self.pedestrian_crossings.values()
        if subtype is not None:
            candidates = [item for item in candidates if item.subtype == subtype]
        return sorted(candidates, key=lambda item: item.lanelet_id)

    def pedestrian_crossing_count(self, subtype=None):
        return len(self.pedestrian_crossing_candidates(subtype))

    def road_lanelet_candidates(self, turn_directions=None):
        candidates = self.road_lanelets.values()
        if turn_directions is not None:
            allowed = set(turn_directions)
            candidates = [
                lanelet
                for lanelet in candidates
                if lanelet.turn_direction in allowed
            ]
        return sorted(candidates, key=lambda lanelet: lanelet.lanelet_id)

    def road_lanelet_count(self):
        return len(self.road_lanelets)

    def road_lanelet_by_id(self, lanelet_id):
        return self.road_lanelets.get(int(lanelet_id))

    def intersection_lanelet_candidates(self, route_lane_ids=None):
        candidates = self.road_lanelet_candidates(("straight", "left", "right"))
        if route_lane_ids is not None:
            route_lane_ids = {int(lane_id) for lane_id in route_lane_ids}
            candidates = [
                lanelet
                for lanelet in candidates
                if lanelet.lanelet_id in route_lane_ids
            ]
        return candidates

    def right_of_way_for_lanelet(self, lanelet_id):
        lanelet_id = int(lanelet_id)
        return [
            element
            for element in self.right_of_ways.values()
            if lanelet_id in element.right_of_way_lane_ids
            or lanelet_id in element.yield_lane_ids
        ]

    def conflicting_road_lanelets(self, lanelet_id):
        lanelet_id = int(lanelet_id)
        if lanelet_id in self._conflicting_lanelets:
            return [
                self.road_lanelets[item]
                for item in self._conflicting_lanelets[lanelet_id]
            ]
        target = self.road_lanelets.get(lanelet_id)
        if target is None:
            return []
        topologically_related = set(target.previous_ids + target.following_ids)
        candidates = []
        target_bbox = _bbox(target.polygon)
        for other in self.road_lanelets.values():
            if other.lanelet_id == lanelet_id:
                continue
            if other.lanelet_id in topologically_related:
                continue
            if not _bbox_intersects(target_bbox, _bbox(other.polygon), margin=0.05):
                continue
            if _polygons_intersect(target.polygon, other.polygon):
                candidates.append(other.lanelet_id)
        self._conflicting_lanelets[lanelet_id] = tuple(sorted(candidates))
        return [self.road_lanelets[item] for item in sorted(candidates)]
