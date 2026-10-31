#!/usr/bin/env python3
"""Build K-City Lanelet2 v44 from v43 with MGeo-authoritative lane changes.

The map historically represented the two sides of a changeable boundary with
different OSM ways.  Lanelet2 then cannot infer a left/right relation even
when both geometries coincide.  This builder makes one canonical dashed way
for every directly mappable MGeo lane-change pair, while preserving the
explicit tollgate separation around lanelets 201399, 201400, and 200147.

GIS B2_SURFACELINEMARK geometry is used when it covers an entire source link;
otherwise the canonical geometry is the midpoint of the two existing
boundaries.  The latter is deliberately reported so it can be reviewed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import shapefile
from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString
from shapely.ops import linemerge, unary_union
from shapely.strtree import STRtree


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "EPSG:32652"
WGS84 = "EPSG:4326"

Point3 = Tuple[float, float, float]

# These pairs contain source links that v39 split around the tollgate.  Two
# already have correct shared boundaries.  The remaining two must not be
# globally joined because that would bridge the tollgate openings.
TOLLGATE_COMPLEX_PAIRS = {
    tuple(sorted(("A2256W000424", "A2256W000432"))),
    tuple(sorted(("A2256W000153", "A2256W000432"))),
}

TOLLGATE_ALREADY_SHARED_PAIRS = {
    tuple(sorted(("A2256W000426", "A2256W000427"))),
    tuple(sorted(("A2256W000427", "A2256W000451"))),
}

TOLLGATE_SPLIT_LINKS = {
    "A2256W000432": ["200185", "201400"],
    "A2256W000153": ["200360", "201399"],
    "A2256W000451": ["200088", "201401"],
    "A2256W000427": ["200150", "201402"],
    "A2256W000426": ["200149", "201403"],
}

# 201178 is an overlapping custom intersection lanelet whose centreline was
# copied from road lanelet 200266.  MGeo A2256W000291 is the ordinary road
# link between 200263/200265, so its two MGeo change permissions must modify
# 200266 rather than that auxiliary intersection representation.
AUTHORITATIVE_GIS_LANELET = {
    "A2256W000291": "200266",
}

# These dense non-course junction groups have incompatible duplicated source
# bounds.  Reusing either whole source way folds a neighboring polygon; their
# midpoint representation is the only one that preserves valid polygons for
# all involved lanelets.  They are individually validated after generation.
MIDPOINT_FALLBACK_PAIRS = {
    tuple(sorted(("A2256W000029", "A2256W000010"))),
    tuple(sorted(("A2256W000175", "A2256W000029"))),
    tuple(sorted(("A2256W000172", "A2256W000176"))),
    tuple(sorted(("A2256W000406", "A2256W000175"))),
    tuple(sorted(("A2256W000346", "A2256W000176"))),
    tuple(sorted(("A2256W001096", "A2256W000869"))),
}

# The user-designated competition change 200134 <-> 200130 has two candidate
# corridors meeting 200130.  Retaining 200130's original left bound prevents
# those independently shared bounds from crossing inside 200130.
PREFER_DESTINATION_BOUNDARY_PAIRS = {
    tuple(sorted(("A2256W000423", "A2256W000422"))),
}


@dataclass(frozen=True)
class GisFeature:
    feature_id: str
    type_code: str
    kind_code: str
    line: LineString


@dataclass(frozen=True)
class LaneletInfo:
    lanelet_id: str
    members: Mapping[str, str]


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def set_tag(element: etree._Element, key: str, value: str) -> None:
    for tag in element.findall("tag"):
        if tag.get("k") == key:
            tag.set("v", value)
            return
    element.append(etree.Element("tag", k=key, v=value))


def way_refs(way: etree._Element) -> List[str]:
    return [nd.get("ref") for nd in way.findall("nd")]


def replace_member_ref(relation: etree._Element, role: str, way_id: str) -> None:
    members = [
        member
        for member in relation.findall("member")
        if member.get("type") == "way" and member.get("role") == role
    ]
    if len(members) != 1:
        raise RuntimeError(f"relation {relation.get('id')} role={role}: expected one way member, got {len(members)}")
    members[0].set("ref", way_id)


def next_numeric_id(root: etree._Element, element_name: str) -> int:
    values = [int(element.get("id")) for element in root.findall(element_name) if element.get("id", "").lstrip("-").isdigit()]
    return max(values) + 1


def euclidean(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def line_length(points: Sequence[Point3]) -> float:
    return sum(euclidean(a, b) for a, b in zip(points, points[1:]))


def orient_points(points: List[Point3], reference: Sequence[Point3]) -> List[Point3]:
    forward = euclidean(points[0], reference[0]) + euclidean(points[-1], reference[-1])
    reverse = euclidean(points[-1], reference[0]) + euclidean(points[0], reference[-1])
    return points if forward <= reverse else list(reversed(points))


def resample(points: Sequence[Point3], count: int) -> List[Point3]:
    if len(points) < 2:
        raise RuntimeError("cannot resample a one-point linestring")
    lengths = [0.0]
    for first, second in zip(points, points[1:]):
        lengths.append(lengths[-1] + euclidean(first, second))
    total = lengths[-1]
    if total <= 1e-9:
        return [tuple(points[0]) for _ in range(count)]
    result: List[Point3] = []
    segment = 0
    for index in range(count):
        target = total * index / max(1, count - 1)
        while segment + 1 < len(lengths) - 1 and lengths[segment + 1] < target:
            segment += 1
        start = points[segment]
        end = points[segment + 1]
        span = lengths[segment + 1] - lengths[segment]
        ratio = 0.0 if span <= 1e-9 else (target - lengths[segment]) / span
        result.append(
            (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
                start[2] + (end[2] - start[2]) * ratio,
            )
        )
    return result


def midpoint_geometry(first: Sequence[Point3], second: Sequence[Point3]) -> List[Point3]:
    second_oriented = orient_points(list(second), first)
    count = max(len(first), len(second_oriented), 2)
    first_resampled = resample(first, count)
    second_resampled = resample(second_oriented, count)
    return [
        (
            (a[0] + b[0]) * 0.5,
            (a[1] + b[1]) * 0.5,
            (a[2] + b[2]) * 0.5,
        )
        for a, b in zip(first_resampled, second_resampled)
    ]


def make_line(points: Sequence[Point3]) -> LineString:
    return LineString([(point[0], point[1]) for point in points])


def read_gis_a2(path: Path) -> Dict[str, LineString]:
    reader = shapefile.Reader(str(path), encoding="utf-8", encodingErrors="replace")
    field_names = [field[0] for field in reader.fields[1:]]
    result = {}
    for shape_record in reader.iterShapeRecords():
        record = dict(zip(field_names, shape_record.record))
        result[str(record["ID"])] = LineString(shape_record.shape.points)
    return result


def read_gis_b2(path: Path) -> Dict[Tuple[str, str], List[GisFeature]]:
    reader = shapefile.Reader(str(path), encoding="utf-8", encodingErrors="replace")
    field_names = [field[0] for field in reader.fields[1:]]
    result: Dict[Tuple[str, str], List[GisFeature]] = defaultdict(list)
    for shape_record in reader.iterShapeRecords():
        record = dict(zip(field_names, shape_record.record))
        right = str(record.get("R_LinkID") or "").strip()
        left = str(record.get("L_LinkID") or "").strip()
        if not right or not left:
            continue
        z_values = list(getattr(shape_record.shape, "z", []) or [])
        points = [
            (float(x), float(y), float(z_values[index]) if index < len(z_values) else 0.0)
            for index, (x, y) in enumerate(shape_record.shape.points)
        ]
        if len(points) < 2:
            continue
        key = tuple(sorted((right, left)))
        result[key].append(
            GisFeature(
                feature_id=str(record["ID"]),
                type_code=str(record.get("Type") or ""),
                kind_code=str(record.get("Kind") or ""),
                line=LineString(points),
            )
        )
    return result


def merge_gis_features(features: Sequence[GisFeature]) -> Optional[LineString]:
    if not features:
        return None
    unioned = unary_union([feature.line for feature in features])
    # Shapely's linemerge accepts multi-line geometries, but raises for a
    # single LineString.  B2 has both representations.
    merged = unioned if isinstance(unioned, LineString) else linemerge(unioned)
    if isinstance(merged, LineString):
        return merged
    return None


def line_points_from_way(way: etree._Element, nodes: Mapping[str, etree._Element], to_utm: Transformer) -> List[Point3]:
    points = []
    for node_id in way_refs(way):
        node = nodes[node_id]
        x, y = to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        point_tags = tags(node)
        points.append((x, y, float(point_tags.get("ele", "0"))))
    if len(points) < 2:
        raise RuntimeError(f"way {way.get('id')} has fewer than two points")
    return points


def line_match_error(first: LineString, second: LineString, samples: int = 41) -> float:
    distances = [
        first.interpolate(index / (samples - 1), normalized=True).distance(second.interpolate(index / (samples - 1), normalized=True))
        for index in range(samples)
    ]
    return max(distances)


def map_lanelets_to_gis(
    lanelets: Mapping[str, LaneletInfo],
    ways: Mapping[str, etree._Element],
    nodes: Mapping[str, etree._Element],
    gis_a2: Mapping[str, LineString],
    to_utm: Transformer,
) -> Dict[str, List[str]]:
    gis_ids = list(gis_a2)
    gis_lines = [gis_a2[gis_id] for gis_id in gis_ids]
    tree = STRtree(gis_lines)
    mapped: Dict[str, List[str]] = {}
    for lanelet_id, lanelet in lanelets.items():
        centerline_id = lanelet.members.get("centerline")
        if not centerline_id:
            continue
        centerline = make_line(line_points_from_way(ways[centerline_id], nodes, to_utm))
        choices = []
        for index in tree.query(centerline.buffer(2.0)):
            error = line_match_error(centerline, gis_lines[index])
            choices.append((error, gis_ids[index]))
        if not choices:
            continue
        error, gis_id = min(choices)
        if error <= 0.10:
            mapped.setdefault(gis_id, []).append(lanelet_id)

    # A source link should map to one lanelet.  The known tollgate links are
    # intentionally represented by two longitudinal lanelets in v43.
    for gis_id, lanelet_ids in TOLLGATE_SPLIT_LINKS.items():
        mapped[gis_id] = list(lanelet_ids)
    for gis_id, lanelet_id in AUTHORITATIVE_GIS_LANELET.items():
        mapped[gis_id] = [lanelet_id]
    return mapped


def mgeo_change_pairs(path: Path) -> Dict[Tuple[str, str], Tuple[str, str, str]]:
    with path.open(encoding="utf-8") as handle:
        links = {link["idx"]: link for link in json.load(handle)}
    pairs: Dict[Tuple[str, str], Tuple[str, str, str]] = {}
    for source_id, link in links.items():
        for role, field in (("left", "left_lane_change_dst_link_idx"), ("right", "right_lane_change_dst_link_idx")):
            destination_id = link.get(field)
            if not destination_id:
                continue
            key = tuple(sorted((source_id, destination_id)))
            # Reciprocal rows express the same physical boundary.  Either
            # direction yields the same source role/opposite role pair.
            pairs.setdefault(key, (source_id, role, destination_id))
    return pairs


def create_nodes_and_way(
    root: etree._Element,
    points: Sequence[Point3],
    next_node_id: int,
    next_way_id: int,
    to_wgs84: Transformer,
) -> Tuple[str, int, int]:
    references = []
    for x, y, z in points:
        lon, lat = to_wgs84.transform(x, y)
        node = etree.Element("node", id=str(next_node_id), lat=f"{lat:.11f}", lon=f"{lon:.11f}")
        node.append(etree.Element("tag", k="ele", v=f"{z:.6f}"))
        root.append(node)
        references.append(str(next_node_id))
        next_node_id += 1
    way = etree.Element("way", id=str(next_way_id))
    for reference in references:
        way.append(etree.Element("nd", ref=reference))
    way.append(etree.Element("tag", k="type", v="line_thin"))
    way.append(etree.Element("tag", k="subtype", v="dashed"))
    root.append(way)
    return str(next_way_id), next_node_id, next_way_id + 1


def all_referenced_way_ids(root: etree._Element) -> set[str]:
    return {
        member.get("ref")
        for relation in root.findall("relation")
        for member in relation.findall("member")
        if member.get("type") == "way" and member.get("ref")
    }


def remove_unreferenced_replaced_ways(root: etree._Element, replaced_way_ids: Iterable[str]) -> int:
    references = all_referenced_way_ids(root)
    removed = 0
    for way in list(root.findall("way")):
        if way.get("id") in replaced_way_ids and way.get("id") not in references:
            root.remove(way)
            removed += 1
    return removed


def reference_errors(root: etree._Element) -> List[str]:
    node_ids = {node.get("id") for node in root.findall("node")}
    way_ids = {way.get("id") for way in root.findall("way")}
    relation_ids = {relation.get("id") for relation in root.findall("relation")}
    errors = []
    for way in root.findall("way"):
        for reference in way.findall("nd"):
            if reference.get("ref") not in node_ids:
                errors.append(f"way:{way.get('id')}:node:{reference.get('ref')}")
    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            layer = {"node": node_ids, "way": way_ids, "relation": relation_ids}.get(member.get("type"))
            if layer is None or member.get("ref") not in layer:
                errors.append(f"relation:{relation.get('id')}:{member.get('type')}:{member.get('ref')}")
    return errors


def duplicate_ids(root: etree._Element) -> Dict[str, List[str]]:
    result = {}
    for element_name in ("node", "way", "relation"):
        counts: Dict[str, int] = defaultdict(int)
        for element in root.findall(element_name):
            counts[element.get("id")] += 1
        result[element_name] = [identifier for identifier, count in counts.items() if count > 1]
    return result


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_csv(path: Path, rows: Sequence[Mapping[str, object]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def course_lanelet_priority(path: Path) -> Dict[str, int]:
    with path.open(newline="", encoding="utf-8") as handle:
        result: Dict[str, int] = {}
        for row in csv.DictReader(handle):
            lanelet_id = row.get("lanelet_id")
            if not lanelet_id:
                continue
            role = row.get("role")
            if role == "preferred_course":
                result[lanelet_id] = 2
            elif role == "candidate_corridor":
                result.setdefault(lanelet_id, 1)
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v43.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v44.osm")
    parser.add_argument("--source-package", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v43")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v44")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports")
    parser.add_argument(
        "--course-csv",
        type=Path,
        default=ROOT / "competition_route/v43/drive_course_v43_lanelets.csv",
        help="Preferred course whose established longitudinal topology takes geometry priority.",
    )
    args = parser.parse_args()

    parser_xml = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(args.input), parser_xml)
    root = tree.getroot()
    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", "44")

    to_utm = Transformer.from_crs(WGS84, UTM52, always_xy=True)
    to_wgs84 = Transformer.from_crs(UTM52, WGS84, always_xy=True)
    nodes = {node.get("id"): node for node in root.findall("node")}
    ways = {way.get("id"): way for way in root.findall("way")}
    relations = {relation.get("id"): relation for relation in root.findall("relation")}
    lanelets: Dict[str, LaneletInfo] = {}
    for relation_id, relation in relations.items():
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
            continue
        members = {
            member.get("role"): member.get("ref")
            for member in relation.findall("member")
            if member.get("type") == "way" and member.get("role") in {"left", "right", "centerline"}
        }
        if {"left", "right", "centerline"} <= set(members):
            lanelets[relation_id] = LaneletInfo(relation_id, members)

    gis_a2 = read_gis_a2(ROOT / "gis_map/A2_LINK.shp")
    gis_b2 = read_gis_b2(ROOT / "gis_map/B2_SURFACELINEMARK.shp")
    gis_to_lanelets = map_lanelets_to_gis(lanelets, ways, nodes, gis_a2, to_utm)
    changes = mgeo_change_pairs(ROOT / "mgeo_map/link_set.json")
    course_priorities = course_lanelet_priority(args.course_csv)

    next_node_id = next_numeric_id(root, "node")
    next_way_id = next_numeric_id(root, "way")
    rows: List[Dict[str, object]] = []
    replaced_way_ids: set[str] = set()

    for pair_key, (source_link, source_role, destination_link) in sorted(changes.items()):
        source_lanelets = gis_to_lanelets.get(source_link, [])
        destination_lanelets = gis_to_lanelets.get(destination_link, [])
        row: Dict[str, object] = {
            "mgeo_link_a": source_link,
            "mgeo_link_b": destination_link,
            "lanelet_a": "|".join(source_lanelets),
            "lanelet_b": "|".join(destination_lanelets),
            "source_side": source_role,
            "status": "",
            "old_way_a": "",
            "old_way_b": "",
            "new_way": "",
            "geometry_source": "",
            "gis_b2_ids": "|".join(feature.feature_id for feature in gis_b2.get(pair_key, [])),
            "gis_b2_type_kind": "|".join(f"{feature.type_code}/{feature.kind_code}" for feature in gis_b2.get(pair_key, [])),
            "b2_coverage_ratio": "",
            "detail": "",
        }
        if pair_key in TOLLGATE_COMPLEX_PAIRS:
            row.update(status="preserved_tollgate_exception", detail="Do not bridge 201399/201400/200147; partial upstream handling requires lanelet segmentation.")
            rows.append(row)
            continue
        if pair_key in TOLLGATE_ALREADY_SHARED_PAIRS:
            retagged = []
            for lanelet_a_id, lanelet_b_id in zip(source_lanelets, destination_lanelets):
                lanelet_a = lanelets[lanelet_a_id]
                lanelet_b = lanelets[lanelet_b_id]
                role_b = "right" if source_role == "left" else "left"
                way_a = lanelet_a.members[source_role]
                way_b = lanelet_b.members[role_b]
                if way_a != way_b:
                    raise RuntimeError(
                        f"Expected tollgate pair {lanelet_a_id}/{lanelet_b_id} to share one boundary, got {way_a}/{way_b}"
                    )
                shared_tags = tags(ways[way_a])
                if shared_tags.get("type") != "line_thin" or shared_tags.get("subtype") != "dashed":
                    set_tag(ways[way_a], "type", "line_thin")
                    set_tag(ways[way_a], "subtype", "dashed")
                    retagged.append(way_a)
            row.update(
                status="retagged_existing_shared_tollgate_dashed" if retagged else "already_shared_tollgate",
                detail=(
                    f"MGeo permits the change; retagged shared tollgate boundary way(s): {'|'.join(retagged)}."
                    if retagged
                    else "Both pre/post split sections already share dashed boundary ways."
                ),
            )
            rows.append(row)
            continue
        if len(source_lanelets) != 1 or len(destination_lanelets) != 1:
            row.update(status="unmapped_or_split", detail="Expected exactly one v43 lanelet for each MGeo source link.")
            rows.append(row)
            continue

        lanelet_a = lanelets[source_lanelets[0]]
        lanelet_b = lanelets[destination_lanelets[0]]
        enabled_vehicle_access = []
        for lanelet in (lanelet_a, lanelet_b):
            relation = relations[lanelet.lanelet_id]
            if tags(relation).get("participant:vehicle") != "yes":
                # MGeo's link-level lane-change permission is authoritative
                # only for vehicle links.  A stale "no" removes the lanelet
                # from Lanelet2's vehicle routing graph entirely, so correct
                # it before establishing the shared change boundary.
                set_tag(relation, "participant:vehicle", "yes")
                enabled_vehicle_access.append(lanelet.lanelet_id)
        role_a = source_role
        role_b = "right" if source_role == "left" else "left"
        old_way_a = lanelet_a.members[role_a]
        old_way_b = lanelet_b.members[role_b]
        row.update(old_way_a=old_way_a, old_way_b=old_way_b)
        if old_way_a == old_way_b:
            shared_tags = tags(ways[old_way_a])
            if shared_tags.get("type") != "line_thin" or shared_tags.get("subtype") != "dashed":
                set_tag(ways[old_way_a], "type", "line_thin")
                set_tag(ways[old_way_a], "subtype", "dashed")
                row.update(
                    status="retagged_existing_shared_dashed",
                    new_way=old_way_a,
                    detail=(
                        "MGeo permits the change; the existing shared boundary was retagged dashed."
                        + (f" vehicle_access_enabled={'|'.join(enabled_vehicle_access)}." if enabled_vehicle_access else "")
                    ),
                )
            else:
                row.update(
                    status="already_shared",
                    new_way=old_way_a,
                    detail=(
                        "Existing shared dashed boundary retained."
                        + (f" vehicle_access_enabled={'|'.join(enabled_vehicle_access)}." if enabled_vehicle_access else "")
                    ),
                )
            rows.append(row)
            continue

        points_a = line_points_from_way(ways[old_way_a], nodes, to_utm)
        points_b = line_points_from_way(ways[old_way_b], nodes, to_utm)
        line_a = make_line(points_a)
        line_b = make_line(points_b)
        direction_forward = euclidean(points_a[0], points_b[0]) + euclidean(points_a[-1], points_b[-1])
        direction_reverse = euclidean(points_a[0], points_b[-1]) + euclidean(points_a[-1], points_b[0])
        if direction_reverse + 0.01 < direction_forward:
            row.update(status="manual_orientation_review", detail="Boundary directions oppose; not rewritten automatically.")
            rows.append(row)
            continue

        features = gis_b2.get(pair_key, [])
        merged_b2 = merge_gis_features(features)
        coverage_ratio = 0.0
        if features:
            coverage_ratio = sum(feature.line.length for feature in features) / max(1e-9, min(line_a.length, line_b.length))
        row["b2_coverage_ratio"] = f"{coverage_ratio:.6f}" if features else ""

        # Reuse one of the existing ways instead of inventing midpoint nodes.
        # Lanelet2 builds longitudinal following relations from exact boundary
        # endpoints.  A midpoint-only way made lateral changes work but moved
        # those endpoints and broke existing course transitions.  Prefer the
        # established competition-course side if exactly one side is in it;
        # otherwise retain the MGeo source side.  The selected way's original
        # node references preserve its already validated topology exactly.
        priority_a = course_priorities.get(lanelet_a.lanelet_id, 0)
        priority_b = course_priorities.get(lanelet_b.lanelet_id, 0)
        if pair_key in MIDPOINT_FALLBACK_PAIRS:
            canonical_way, next_node_id, next_way_id = create_nodes_and_way(
                root,
                midpoint_geometry(points_a, points_b),
                next_node_id,
                next_way_id,
                to_wgs84,
            )
            replace_member_ref(relations[lanelet_a.lanelet_id], role_a, canonical_way)
            replace_member_ref(relations[lanelet_b.lanelet_id], role_b, canonical_way)
            replaced_way_ids.update({old_way_a, old_way_b})
            geometry_source = "midpoint_junction_polygon_fallback"
        elif priority_b > priority_a or pair_key in PREFER_DESTINATION_BOUNDARY_PAIRS:
            canonical_way = old_way_b
            replace_member_ref(relations[lanelet_a.lanelet_id], role_a, canonical_way)
            replaced_way_ids.add(old_way_a)
            geometry_source = "existing_higher_priority_course_boundary"
        else:
            canonical_way = old_way_a
            replace_member_ref(relations[lanelet_b.lanelet_id], role_b, canonical_way)
            replaced_way_ids.add(old_way_b)
            geometry_source = (
                "existing_higher_priority_course_boundary"
                if priority_a > priority_b
                else "existing_mgeo_source_boundary"
            )
        # Newly created midpoint ways are already tagged at construction;
        # reused source ways need their line-mark tags normalized here.
        if canonical_way in ways:
            set_tag(ways[canonical_way], "type", "line_thin")
            set_tag(ways[canonical_way], "subtype", "dashed")
        row.update(
            status="changed_shared_dashed",
            new_way=canonical_way,
            geometry_source=geometry_source,
            detail=(
                f"old_p95_m={line_match_error(line_a, line_b):.6f}"
                + (f"; vehicle_access_enabled={'|'.join(enabled_vehicle_access)}" if enabled_vehicle_access else "")
            ),
        )
        rows.append(row)

    removed_ways = remove_unreferenced_replaced_ways(root, replaced_way_ids)
    errors = reference_errors(root)
    duplicates = duplicate_ids(root)
    if errors or any(duplicates.values()):
        raise RuntimeError(f"ref_errors={errors[:5]} duplicate_ids={duplicates}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    args.package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        source = args.source_package / name
        if source.exists():
            shutil.copy2(source, args.package_dir / name)

    fields = [
        "mgeo_link_a", "mgeo_link_b", "lanelet_a", "lanelet_b", "source_side", "status",
        "old_way_a", "old_way_b", "new_way", "geometry_source", "gis_b2_ids",
        "gis_b2_type_kind", "b2_coverage_ratio", "detail",
    ]
    write_csv(args.report_dir / "v44_mgeo_lane_change_boundary_audit.csv", rows, fields)
    status_counts: Dict[str, int] = defaultdict(int)
    geometry_counts: Dict[str, int] = defaultdict(int)
    for row in rows:
        status_counts[str(row["status"])] += 1
        if row["geometry_source"]:
            geometry_counts[str(row["geometry_source"])] += 1
    summary = {
        "input": str(args.input.relative_to(ROOT)),
        "output": str(args.output.relative_to(ROOT)),
        "sha256": sha256(args.output),
        "road_lanelets": len(lanelets),
        "mgeo_lane_change_pairs": len(changes),
        "status_counts": dict(sorted(status_counts.items())),
        "geometry_counts": dict(sorted(geometry_counts.items())),
        "removed_unreferenced_replaced_ways": removed_ways,
        "reference_errors": errors,
        "duplicate_ids": duplicates,
    }
    summary_path = args.report_dir / "v44_mgeo_lane_change_boundary_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
