#!/usr/bin/env python3
"""Compare and version-align K-City Lanelet OSM against MORAI MGeo links.

The MGeo directory currently contains link/node/global JSON only.  This tool
therefore treats MGeo link centerlines plus their width-derived edges as the
quantitative reference and leaves the configured map->morai_map static TF
untouched.
"""

import argparse
import json
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

import yaml


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
UTM_K0 = 0.9996


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


def utm_to_wgs84(easting, northing, zone):
    e2 = WGS84_F * (2.0 - WGS84_F)
    ep2 = e2 / (1.0 - e2)
    e1 = (1.0 - math.sqrt(1.0 - e2)) / (1.0 + math.sqrt(1.0 - e2))
    x = easting - 500000.0
    y = northing
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)

    m = y / UTM_K0
    mu = m / (WGS84_A * (1.0 - e2 / 4.0 - 3.0 * e2**2 / 64.0 - 5.0 * e2**3 / 256.0))
    j1 = 3.0 * e1 / 2.0 - 27.0 * e1**3 / 32.0
    j2 = 21.0 * e1**2 / 16.0 - 55.0 * e1**4 / 32.0
    j3 = 151.0 * e1**3 / 96.0
    j4 = 1097.0 * e1**4 / 512.0
    fp = mu + j1 * math.sin(2 * mu) + j2 * math.sin(4 * mu) + j3 * math.sin(6 * mu) + j4 * math.sin(8 * mu)

    sin_fp = math.sin(fp)
    cos_fp = math.cos(fp)
    tan_fp = math.tan(fp)
    c1 = ep2 * cos_fp * cos_fp
    t1 = tan_fp * tan_fp
    n1 = WGS84_A / math.sqrt(1.0 - e2 * sin_fp * sin_fp)
    r1 = WGS84_A * (1.0 - e2) / ((1.0 - e2 * sin_fp * sin_fp) ** 1.5)
    d = x / (n1 * UTM_K0)

    lat = fp - (n1 * tan_fp / r1) * (
        d**2 / 2.0
        - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1**2 - 9.0 * ep2) * d**4 / 24.0
        + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1**2 - 252.0 * ep2 - 3.0 * c1**2) * d**6 / 720.0
    )
    lon = lon0 + (
        d
        - (1.0 + 2.0 * t1 + c1) * d**3 / 6.0
        + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1**2 + 8.0 * ep2 + 24.0 * t1**2) * d**5 / 120.0
    ) / cos_fp
    return math.degrees(lat), math.degrees(lon)


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_projector(path):
    cfg = load_yaml(path)
    if cfg.get("projector_type") != "LocalCartesianUTM":
        raise ValueError("Only LocalCartesianUTM is supported")
    origin = cfg["map_origin"]
    lat = float(origin["latitude"])
    lon = float(origin["longitude"])
    alt = float(origin.get("altitude", 0.0))
    zone = int(math.floor((lon + 180.0) / 6.0) + 1)
    origin_e, origin_n = wgs84_to_utm(lat, lon, zone)
    return {
        "zone": zone,
        "origin_easting": origin_e,
        "origin_northing": origin_n,
        "origin_alt": alt,
    }


def map_xy_to_lat_lon(x, y, projector):
    return utm_to_wgs84(
        projector["origin_easting"] + x,
        projector["origin_northing"] + y,
        projector["zone"],
    )


def read_static_tf(path):
    cfg = load_yaml(path)["map_to_morai_map"]
    tx, ty, tz = [float(v) for v in cfg["xyz"]]
    yaw = float(cfg.get("rpy", [0.0, 0.0, 0.0])[2])
    return tx, ty, tz, yaw


def transform_morai_to_map(point, tf_cfg):
    tx, ty, tz, yaw = tf_cfg
    c = math.cos(yaw)
    s = math.sin(yaw)
    x, y, z = point
    return (c * x - s * y + tx, s * x + c * y + ty, z + tz)


def parse_lanelet(path, projector):
    tree = ET.parse(path)
    root = tree.getroot()
    nodes = {}
    ways = {}
    relations = []

    for elem in root:
        if elem.tag == "node":
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            lat = float(elem.attrib["lat"])
            lon = float(elem.attrib["lon"])
            ele = float(tags.get("ele", projector["origin_alt"]))
            easting, northing = wgs84_to_utm(lat, lon, projector["zone"])
            nodes[elem.attrib["id"]] = {
                "elem": elem,
                "xy": (easting - projector["origin_easting"], northing - projector["origin_northing"]),
                "z": ele - projector["origin_alt"],
                "ele": ele,
            }
        elif elem.tag == "way":
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            refs = [nd.attrib["ref"] for nd in elem if nd.tag == "nd"]
            ways[elem.attrib["id"]] = {"elem": elem, "tags": tags, "refs": refs}
        elif elem.tag == "relation":
            tags = {t.attrib.get("k"): t.attrib.get("v") for t in elem if t.tag == "tag"}
            members = [
                {"type": m.attrib.get("type"), "ref": m.attrib.get("ref"), "role": m.attrib.get("role", "")}
                for m in elem
                if m.tag == "member"
            ]
            relations.append({"elem": elem, "id": elem.attrib["id"], "tags": tags, "members": members})
    return tree, root, nodes, ways, relations


def relation_way_id(relation, role):
    for member in relation["members"]:
        if member["type"] == "way" and member["role"] == role:
            return member["ref"]
    return None


def way_points(way_id, ways, nodes, include_z=False):
    pts = []
    if way_id not in ways:
        return pts
    for ref in ways[way_id]["refs"]:
        if ref not in nodes:
            continue
        x, y = nodes[ref]["xy"]
        if include_z:
            pts.append((x, y, nodes[ref]["z"]))
        else:
            pts.append((x, y))
    return pts


def load_mgeo_links(path, tf_cfg):
    with open(path, "r", encoding="utf-8") as f:
        links = json.load(f)
    result = []
    for link in links:
        pts = [transform_morai_to_map(p, tf_cfg) for p in link["points"]]
        result.append(
            {
                "idx": link["idx"],
                "points": pts,
                "width_start": float(link.get("width_start", 3.5)),
                "width_end": float(link.get("width_end", 3.5)),
                "link_length": float(link.get("link_length", polyline_length(pts))),
                "raw": link,
            }
        )
    return result


def dist_point_segment_sq(p, a, b):
    px, py = p[:2]
    ax, ay = a[:2]
    bx, by = b[:2]
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return (px - ax) ** 2 + (py - ay) ** 2, 0.0
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = max(0.0, min(1.0, t))
    qx = ax + t * dx
    qy = ay + t * dy
    return (px - qx) ** 2 + (py - qy) ** 2, t


class SegmentIndex:
    def __init__(self, segments, cell_size=5.0):
        self.segments = segments
        self.cell_size = float(cell_size)
        self.grid = defaultdict(list)
        for i, seg in enumerate(segments):
            a, b = seg["a"], seg["b"]
            min_x = min(a[0], b[0])
            max_x = max(a[0], b[0])
            min_y = min(a[1], b[1])
            max_y = max(a[1], b[1])
            cx0 = math.floor(min_x / self.cell_size)
            cx1 = math.floor(max_x / self.cell_size)
            cy0 = math.floor(min_y / self.cell_size)
            cy1 = math.floor(max_y / self.cell_size)
            for cx in range(cx0, cx1 + 1):
                for cy in range(cy0, cy1 + 1):
                    self.grid[(cx, cy)].append(i)

    def nearest(self, point, max_ring=256):
        if not self.segments:
            return None
        cx0 = math.floor(point[0] / self.cell_size)
        cy0 = math.floor(point[1] / self.cell_size)
        checked = set()
        best = None
        best_d2 = float("inf")
        ring = 0
        while ring <= max_ring:
            for cx in range(cx0 - ring, cx0 + ring + 1):
                for cy in range(cy0 - ring, cy0 + ring + 1):
                    if ring and abs(cx - cx0) < ring and abs(cy - cy0) < ring:
                        continue
                    for idx in self.grid.get((cx, cy), []):
                        if idx in checked:
                            continue
                        checked.add(idx)
                        seg = self.segments[idx]
                        d2, t = dist_point_segment_sq(point, seg["a"], seg["b"])
                        if d2 < best_d2:
                            best_d2 = d2
                            best = (idx, t, seg)
            if best is not None and math.sqrt(best_d2) < max(0.0, (ring - 1) * self.cell_size):
                break
            ring = ring + 1 if ring < 8 else ring * 2
        if best is None:
            return None
        return {"distance": math.sqrt(best_d2), "segment_index": best[0], "t": best[1], "segment": best[2]}


def polyline_length(points):
    total = 0.0
    for a, b in zip(points, points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def cumulative_lengths(points):
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return lengths


def interpolate_polyline(points, lengths, s):
    if not points:
        raise ValueError("empty polyline")
    if s <= 0.0:
        return points[0]
    if s >= lengths[-1]:
        return points[-1]
    lo, hi = 0, len(lengths) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if lengths[mid] <= s:
            lo = mid
        else:
            hi = mid
    span = lengths[lo + 1] - lengths[lo]
    t = 0.0 if span <= 1e-12 else (s - lengths[lo]) / span
    a = points[lo]
    b = points[lo + 1]
    if len(a) >= 3:
        return (
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        )
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def subline(points, s0, s1):
    if not points:
        return []
    lengths = cumulative_lengths(points)
    total = lengths[-1]
    s0 = max(0.0, min(total, s0))
    s1 = max(0.0, min(total, s1))
    reverse = s1 < s0
    lo, hi = (s1, s0) if reverse else (s0, s1)
    out = [interpolate_polyline(points, lengths, lo)]
    for p, s in zip(points[1:-1], lengths[1:-1]):
        if lo < s < hi:
            out.append(p)
    out.append(interpolate_polyline(points, lengths, hi))
    if reverse:
        out.reverse()
    return dedupe_points(out)


def dedupe_points(points, eps=1e-4):
    out = []
    for p in points:
        if out and math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) < eps:
            continue
        out.append(p)
    return out


def make_segments(polyline, feature_id, source=None):
    segments = []
    for i, (a, b) in enumerate(zip(polyline, polyline[1:])):
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6:
            continue
        segments.append({"a": a, "b": b, "feature_id": feature_id, "part_index": i, "source": source})
    return segments


def mgeo_edge_points(points, width_start, width_end, side):
    if len(points) < 2:
        return []
    lengths = cumulative_lengths(points)
    total = max(lengths[-1], 1e-9)
    out = []
    sign = 1.0 if side == "left" else -1.0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            continue
        nx = -dy / norm
        ny = dx / norm
        w0 = width_start + (width_end - width_start) * (lengths[i] / total)
        w1 = width_start + (width_end - width_start) * (lengths[i + 1] / total)
        out.append((a[0] + sign * nx * w0 * 0.5, a[1] + sign * ny * w0 * 0.5, a[2] if len(a) > 2 else 0.0))
        out.append((b[0] + sign * nx * w1 * 0.5, b[1] + sign * ny * w1 * 0.5, b[2] if len(b) > 2 else 0.0))
    return dedupe_points(out, eps=1e-5)


def mgeo_edge_segments(link, side):
    points = link["points"]
    if len(points) < 2:
        return []
    lengths = cumulative_lengths(points)
    total = max(lengths[-1], 1e-9)
    sign = 1.0 if side == "left" else -1.0
    segments = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            continue
        nx = -dy / norm
        ny = dx / norm
        w0 = link["width_start"] + (link["width_end"] - link["width_start"]) * (lengths[i] / total)
        w1 = link["width_start"] + (link["width_end"] - link["width_start"]) * (lengths[i + 1] / total)
        edge_a = (a[0] + sign * nx * w0 * 0.5, a[1] + sign * ny * w0 * 0.5, a[2] if len(a) > 2 else 0.0)
        edge_b = (b[0] + sign * nx * w1 * 0.5, b[1] + sign * ny * w1 * 0.5, b[2] if len(b) > 2 else 0.0)
        segments.append({"a": edge_a, "b": edge_b, "feature_id": link["idx"] + ":" + side, "part_index": i, "source": link})
    return segments


def projected_point(nearest):
    seg = nearest["segment"]
    t = nearest["t"]
    a = seg["a"]
    b = seg["b"]
    z_a = a[2] if len(a) > 2 else 0.0
    z_b = b[2] if len(b) > 2 else z_a
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        z_a + (z_b - z_a) * t,
    )


def metric_points_to_index(name, feature_points, target_index):
    count = 0
    sum_sq = 0.0
    max_d = 0.0
    misses = 0
    per_feature = defaultdict(lambda: [0, 0.0, 0.0])
    nearest_features = Counter()
    for feature_id, points in feature_points:
        for p in points:
            nearest = target_index.nearest(p)
            if nearest is None:
                misses += 1
                continue
            d = nearest["distance"]
            count += 1
            sum_sq += d * d
            max_d = max(max_d, d)
            rec = per_feature[feature_id]
            rec[0] += 1
            rec[1] += d * d
            rec[2] = max(rec[2], d)
            nearest_features[nearest["segment"]["feature_id"]] += 1
    rmse = math.sqrt(sum_sq / count) if count else float("nan")
    mean_feature_rmse = (
        sum(math.sqrt(v[1] / v[0]) for v in per_feature.values() if v[0]) / len(per_feature)
        if per_feature
        else float("nan")
    )
    max_feature_rmse = (
        max(math.sqrt(v[1] / v[0]) for v in per_feature.values() if v[0])
        if per_feature
        else float("nan")
    )
    return {
        "name": name,
        "points": count,
        "misses": misses,
        "rmse": rmse,
        "max_distance": max_d,
        "mean_feature_rmse": mean_feature_rmse,
        "max_feature_rmse": max_feature_rmse,
        "features": len(per_feature),
    }


def print_metric(metric):
    print(
        "{name}: points={points} features={features} rmse={rmse:.6f} "
        "mean_feature_rmse={mean_feature_rmse:.6f} max_feature_rmse={max_feature_rmse:.6f} "
        "max_distance={max_distance:.6f} misses={misses}".format(**metric)
    )


def road_relations(relations):
    return [r for r in relations if r["tags"].get("type") == "lanelet" and r["tags"].get("subtype") == "road"]


def build_lanelet_segment_sets(nodes, ways, relations):
    centers = []
    lefts = []
    rights = []
    boundaries = []
    road_borders = []
    line_thins = []
    virtuals = []
    rel_center_points = {}

    for relation in road_relations(relations):
        rel_id = relation["id"]
        center_id = relation_way_id(relation, "centerline")
        left_id = relation_way_id(relation, "left")
        right_id = relation_way_id(relation, "right")
        center = way_points(center_id, ways, nodes)
        left = way_points(left_id, ways, nodes)
        right = way_points(right_id, ways, nodes)
        rel_center_points[rel_id] = center
        centers.extend(make_segments(center, rel_id, center_id))
        lefts.extend(make_segments(left, rel_id, left_id))
        rights.extend(make_segments(right, rel_id, right_id))
        boundaries.extend(make_segments(left, rel_id, left_id))
        boundaries.extend(make_segments(right, rel_id, right_id))

    for way_id, way in ways.items():
        pts = way_points(way_id, ways, nodes)
        way_type = way["tags"].get("type")
        if way_type == "road_border":
            road_borders.extend(make_segments(pts, way_id, way_id))
        elif way_type == "line_thin":
            line_thins.extend(make_segments(pts, way_id, way_id))
        elif way_type == "virtual":
            virtuals.extend(make_segments(pts, way_id, way_id))
    return {
        "centers": centers,
        "lefts": lefts,
        "rights": rights,
        "boundaries": boundaries,
        "road_borders": road_borders,
        "line_thins": line_thins,
        "virtuals": virtuals,
        "rel_center_points": rel_center_points,
    }


def build_mgeo_feature_points(links):
    center = []
    left = []
    right = []
    width_edges = []
    for link in links:
        pts = link["points"]
        lpts = mgeo_edge_points(pts, link["width_start"], link["width_end"], "left")
        rpts = mgeo_edge_points(pts, link["width_start"], link["width_end"], "right")
        center.append((link["idx"], pts))
        left.append((link["idx"], lpts))
        right.append((link["idx"], rpts))
        width_edges.append((link["idx"] + ":left", lpts))
        width_edges.append((link["idx"] + ":right", rpts))
    return center, left, right, width_edges


def build_mgeo_outer_edge_feature_points(links):
    outer = []
    for link in links:
        raw = link["raw"]
        if not raw.get("left_lane_change_dst_link_idx"):
            outer.append(
                (
                    link["idx"] + ":left_outer",
                    mgeo_edge_points(link["points"], link["width_start"], link["width_end"], "left"),
                )
            )
        if not raw.get("right_lane_change_dst_link_idx"):
            outer.append(
                (
                    link["idx"] + ":right_outer",
                    mgeo_edge_points(link["points"], link["width_start"], link["width_end"], "right"),
                )
            )
    return outer


def compare(args):
    projector = load_projector(args.projector)
    tf_cfg = read_static_tf(args.static_tf)
    _, _, nodes, ways, relations = parse_lanelet(args.lanelet, projector)
    links = load_mgeo_links(args.mgeo_link, tf_cfg)
    segs = build_lanelet_segment_sets(nodes, ways, relations)
    mgeo_center, mgeo_left, mgeo_right, mgeo_edges = build_mgeo_feature_points(links)
    mgeo_outer_edges = build_mgeo_outer_edge_feature_points(links)

    indexes = {
        "lanelet_centerlines": SegmentIndex(segs["centers"], args.cell_size),
        "lanelet_left_boundaries": SegmentIndex(segs["lefts"], args.cell_size),
        "lanelet_right_boundaries": SegmentIndex(segs["rights"], args.cell_size),
        "lanelet_relation_boundaries": SegmentIndex(segs["boundaries"], args.cell_size),
        "lanelet_road_borders": SegmentIndex(segs["road_borders"], args.cell_size),
        "lanelet_line_thin": SegmentIndex(segs["line_thins"], args.cell_size),
    }

    print("lanelet=%s" % args.lanelet)
    print("mgeo_link=%s" % args.mgeo_link)
    print("static_tf=%s" % args.static_tf)
    print(
        "counts: lanelet_nodes=%d lanelet_ways=%d road_lanelets=%d mgeo_links=%d"
        % (len(nodes), len(ways), len(road_relations(relations)), len(links))
    )
    metrics = [
        metric_points_to_index("mgeo_center_to_lanelet_centerline", mgeo_center, indexes["lanelet_centerlines"]),
        metric_points_to_index("mgeo_left_edge_to_lanelet_left", mgeo_left, indexes["lanelet_left_boundaries"]),
        metric_points_to_index("mgeo_right_edge_to_lanelet_right", mgeo_right, indexes["lanelet_right_boundaries"]),
        metric_points_to_index("mgeo_width_edges_to_lanelet_boundaries", mgeo_edges, indexes["lanelet_relation_boundaries"]),
        metric_points_to_index("mgeo_outer_edges_to_lanelet_road_borders", mgeo_outer_edges, indexes["lanelet_road_borders"]),
        metric_points_to_index("mgeo_width_edges_to_lanelet_road_borders", mgeo_edges, indexes["lanelet_road_borders"]),
        metric_points_to_index("mgeo_width_edges_to_lanelet_line_thin", mgeo_edges, indexes["lanelet_line_thin"]),
    ]
    for metric in metrics:
        print_metric(metric)
    return metrics


def project_point_to_polyline(point, points, lengths=None):
    if len(points) < 2:
        return None
    if lengths is None:
        lengths = cumulative_lengths(points)
    best_d2 = float("inf")
    best_s = 0.0
    best_i = 0
    best_t = 0.0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        d2, t = dist_point_segment_sq(point, a, b)
        if d2 < best_d2:
            best_d2 = d2
            best_s = lengths[i] + math.hypot(b[0] - a[0], b[1] - a[1]) * t
            best_i = i
            best_t = t
    return {"distance": math.sqrt(best_d2), "s": best_s, "segment_index": best_i, "t": best_t}


def feature_index_for_links(links):
    segments = []
    for link in links:
        segments.extend(make_segments(link["points"], link["idx"], link))
    return SegmentIndex(segments, 8.0)


def match_lanelets_to_mgeo(nodes, ways, relations, links, threshold):
    link_by_id = {link["idx"]: link for link in links}
    index = feature_index_for_links(links)
    max_ring = max(2, int(math.ceil(max(threshold * 2.0, index.cell_size) / index.cell_size)) + 2)
    matches = {}
    unmatched = []
    counts = Counter()
    for relation in road_relations(relations):
        center_id = relation_way_id(relation, "centerline")
        center = way_points(center_id, ways, nodes)
        if len(center) < 2:
            unmatched.append((relation, "empty_centerline", float("inf")))
            continue
        per_link = defaultdict(lambda: [0, 0.0])
        for p in center:
            nearest = index.nearest(p, max_ring=max_ring)
            if nearest is None:
                continue
            link_id = nearest["segment"]["feature_id"]
            per_link[link_id][0] += 1
            per_link[link_id][1] += nearest["distance"] ** 2
        if not per_link:
            unmatched.append((relation, "no_candidate", float("inf")))
            continue
        best_link_id, best_vals = min(per_link.items(), key=lambda kv: math.sqrt(kv[1][1] / kv[1][0]))
        rmse = math.sqrt(best_vals[1] / best_vals[0])
        if rmse > threshold:
            unmatched.append((relation, best_link_id, rmse))
            continue
        link = link_by_id[best_link_id]
        link_lengths = cumulative_lengths(link["points"])
        projections = [project_point_to_polyline(p, link["points"], link_lengths) for p in center]
        projections = [p for p in projections if p is not None]
        if not projections:
            unmatched.append((relation, best_link_id, rmse))
            continue
        s_values = [p["s"] for p in projections if p["distance"] <= max(threshold * 2.0, 1.0)]
        if len(s_values) < 2:
            s_values = [p["s"] for p in projections]
        start_s = projections[0]["s"]
        end_s = projections[-1]["s"]
        s_min = max(0.0, min(s_values))
        s_max = min(link_lengths[-1], max(s_values))
        if s_max - s_min < 0.2:
            unmatched.append((relation, best_link_id, rmse))
            continue
        matches[relation["id"]] = {
            "relation": relation,
            "link": link,
            "rmse": rmse,
            "s_min": s_min,
            "s_max": s_max,
            "same_direction": end_s >= start_s,
        }
        counts[best_link_id] += 1
    return matches, unmatched, counts


def next_id(existing):
    return max(int(v) for v in existing) + 1


def remove_children_by_tag(elem, tag):
    for child in list(elem):
        if child.tag == tag:
            elem.remove(child)


def replace_way_refs(way, refs):
    elem = way["elem"]
    first_tag_index = None
    for i, child in enumerate(list(elem)):
        if child.tag == "tag":
            first_tag_index = i
            break
    remove_children_by_tag(elem, "nd")
    insert_at = 0 if first_tag_index is None else 0
    for offset, ref in enumerate(refs):
        nd = ET.Element("nd", {"ref": str(ref)})
        elem.insert(insert_at + offset, nd)
    way["refs"] = [str(r) for r in refs]


def set_node_position(node, point, projector):
    lat, lon = map_xy_to_lat_lon(point[0], point[1], projector)
    node["elem"].attrib["lat"] = "%.11f" % lat
    node["elem"].attrib["lon"] = "%.11f" % lon
    node["xy"] = (point[0], point[1])
    if len(point) >= 3:
        ele = point[2] + projector["origin_alt"]
        node["z"] = point[2]
        node["ele"] = ele
        ele_tag = None
        for child in node["elem"]:
            if child.tag == "tag" and child.attrib.get("k") == "ele":
                ele_tag = child
                break
        if ele_tag is None:
            ele_tag = ET.SubElement(node["elem"], "tag", {"k": "ele", "v": "%.3f" % ele})
        else:
            ele_tag.attrib["v"] = "%.3f" % ele


def append_node(root, nodes, node_id, point, projector):
    lat, lon = map_xy_to_lat_lon(point[0], point[1], projector)
    elem = ET.Element(
        "node",
        {
            "id": str(node_id),
            "visible": "true",
            "version": "1",
            "lat": "%.11f" % lat,
            "lon": "%.11f" % lon,
        },
    )
    ele = point[2] + projector["origin_alt"] if len(point) >= 3 else projector["origin_alt"]
    ET.SubElement(elem, "tag", {"k": "ele", "v": "%.3f" % ele})
    root.append(elem)
    nodes[str(node_id)] = {"elem": elem, "xy": (point[0], point[1]), "z": ele - projector["origin_alt"], "ele": ele}
    return str(node_id)


def append_way(root, ways, way_id, refs, tags):
    elem = ET.Element("way", {"id": str(way_id), "visible": "true", "version": "1"})
    for ref in refs:
        ET.SubElement(elem, "nd", {"ref": str(ref)})
    for k, v in tags.items():
        ET.SubElement(elem, "tag", {"k": str(k), "v": str(v)})
    root.append(elem)
    ways[str(way_id)] = {"elem": elem, "tags": dict(tags), "refs": [str(r) for r in refs]}
    return str(way_id)


def append_relation(root, relations, relation_id, left_way, right_way, center_way):
    elem = ET.Element("relation", {"id": str(relation_id), "visible": "true", "version": "1"})
    ET.SubElement(elem, "member", {"type": "way", "ref": str(left_way), "role": "left"})
    ET.SubElement(elem, "member", {"type": "way", "ref": str(right_way), "role": "right"})
    ET.SubElement(elem, "member", {"type": "way", "ref": str(center_way), "role": "centerline"})
    ET.SubElement(elem, "tag", {"k": "type", "v": "lanelet"})
    ET.SubElement(elem, "tag", {"k": "subtype", "v": "road"})
    root.append(elem)
    relation = {
        "elem": elem,
        "id": str(relation_id),
        "tags": {"type": "lanelet", "subtype": "road"},
        "members": [
            {"type": "way", "ref": str(left_way), "role": "left"},
            {"type": "way", "ref": str(right_way), "role": "right"},
            {"type": "way", "ref": str(center_way), "role": "centerline"},
        ],
    }
    relations.append(relation)
    return str(relation_id)


def replace_way_geometry(root, nodes, ways, way_id, points, projector, id_state):
    if way_id not in ways or len(points) < 2:
        return 0
    refs = []
    for point in points:
        ref = str(id_state["next_node"])
        id_state["next_node"] += 1
        refs.append(append_node(root, nodes, ref, point, projector))
    replace_way_refs(ways[way_id], refs)
    return len(refs)


def update_meta_version(root, version):
    for elem in root:
        if elem.tag == "MetaInfo":
            elem.attrib["map_version"] = str(version)
            return


def indent_xml(elem, level=0):
    indent = "\n" + level * "  "
    child_indent = "\n" + (level + 1) * "  "
    children = list(elem)
    if children:
        if not elem.text or not elem.text.strip():
            elem.text = child_indent
        for child in children:
            indent_xml(child, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent
    else:
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent


def detect_version(path):
    m = re.search(r"_v(\d+)\.osm$", os.path.basename(path))
    return int(m.group(1)) if m else None


def default_next_output(input_path):
    directory = os.path.dirname(input_path) or "."
    base = os.path.basename(input_path)
    m = re.match(r"(.+)_v(\d+)\.osm$", base)
    if not m:
        return os.path.join(directory, base + ".aligned.osm"), None
    prefix = m.group(1)
    versions = []
    for name in os.listdir(directory):
        vm = re.match(re.escape(prefix) + r"_v(\d+)\.osm$", name)
        if vm:
            versions.append(int(vm.group(1)))
    version = max(versions) + 1 if versions else int(m.group(2)) + 1
    return os.path.join(directory, "%s_v%d.osm" % (prefix, version)), version


def merge_intervals(intervals, tolerance):
    if not intervals:
        return []
    intervals = sorted((max(0.0, a), max(0.0, b)) for a, b in intervals if b > a)
    merged = [list(intervals[0])]
    for a, b in intervals[1:]:
        if a <= merged[-1][1] + tolerance:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    return [(a, b) for a, b in merged]


def uncovered_intervals(total, covered, min_length):
    out = []
    cursor = 0.0
    for a, b in covered:
        if a - cursor >= min_length:
            out.append((cursor, a))
        cursor = max(cursor, b)
    if total - cursor >= min_length:
        out.append((cursor, total))
    return out


def create_lanelet_from_link_interval(root, nodes, ways, relations, link, s0, s1, projector, id_state):
    center = subline(link["points"], s0, s1)
    if len(center) < 2:
        return None
    left = mgeo_edge_points(center, link["width_start"], link["width_end"], "left")
    right = mgeo_edge_points(center, link["width_start"], link["width_end"], "right")
    if len(left) < 2 or len(right) < 2:
        return None
    left_refs = []
    right_refs = []
    center_refs = []
    for pts, refs in ((left, left_refs), (right, right_refs), (center, center_refs)):
        for point in pts:
            ref = str(id_state["next_node"])
            id_state["next_node"] += 1
            refs.append(append_node(root, nodes, ref, point, projector))
    left_way = append_way(root, ways, id_state["next_way"], left_refs, {"type": "road_border"})
    id_state["next_way"] += 1
    right_way = append_way(root, ways, id_state["next_way"], right_refs, {"type": "road_border"})
    id_state["next_way"] += 1
    center_way = append_way(root, ways, id_state["next_way"], center_refs, {"type": "virtual"})
    id_state["next_way"] += 1
    relation_id = append_relation(root, relations, id_state["next_relation"], left_way, right_way, center_way)
    id_state["next_relation"] += 1
    return relation_id


def create_road_border_way_from_link_interval(root, nodes, ways, link, side, s0, s1, projector, id_state):
    center = subline(link["points"], s0, s1)
    if len(center) < 2:
        return None
    edge = mgeo_edge_points(center, link["width_start"], link["width_end"], side)
    if len(edge) < 2:
        return None
    refs = []
    for point in edge:
        ref = str(id_state["next_node"])
        id_state["next_node"] += 1
        refs.append(append_node(root, nodes, ref, point, projector))
    way_id = append_way(root, ways, id_state["next_way"], refs, {"type": "road_border"})
    id_state["next_way"] += 1
    return way_id


def mgeo_edge_points_with_s(link, side):
    points = link["points"]
    if len(points) < 2:
        return []
    lengths = cumulative_lengths(points)
    total = max(lengths[-1], 1e-9)
    sign = 1.0 if side == "left" else -1.0
    out = []
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            continue
        nx = -dy / norm
        ny = dx / norm
        w0 = link["width_start"] + (link["width_end"] - link["width_start"]) * (lengths[i] / total)
        w1 = link["width_start"] + (link["width_end"] - link["width_start"]) * (lengths[i + 1] / total)
        out.append(
            (
                (a[0] + sign * nx * w0 * 0.5, a[1] + sign * ny * w0 * 0.5, a[2] if len(a) > 2 else 0.0),
                lengths[i],
            )
        )
        out.append(
            (
                (b[0] + sign * nx * w1 * 0.5, b[1] + sign * ny * w1 * 0.5, b[2] if len(b) > 2 else 0.0),
                lengths[i + 1],
            )
        )
    return out


def intervals_from_bad_s(values, total, threshold_gap, margin, min_length):
    if not values:
        return []
    values = sorted(max(0.0, min(total, v)) for v in values)
    intervals = []
    start = values[0]
    prev = values[0]
    for value in values[1:]:
        if value - prev <= threshold_gap:
            prev = value
            continue
        intervals.append((max(0.0, start - margin), min(total, prev + margin)))
        start = value
        prev = value
    intervals.append((max(0.0, start - margin), min(total, prev + margin)))
    return [(a, b) for a, b in merge_intervals(intervals, margin) if b - a >= min_length]


def generate_augment(args):
    output, version = (args.output, detect_version(args.output)) if args.output else default_next_output(args.lanelet)
    if os.path.exists(output) and not args.force:
        raise RuntimeError("%s already exists; pass --force to overwrite" % output)

    projector = load_projector(args.projector)
    tf_cfg = read_static_tf(args.static_tf)
    tree, root, nodes, ways, relations = parse_lanelet(args.lanelet, projector)
    links = load_mgeo_links(args.mgeo_link, tf_cfg)
    segs = build_lanelet_segment_sets(nodes, ways, relations)
    center_index = SegmentIndex(segs["centers"], args.cell_size)
    boundary_index = SegmentIndex(segs["boundaries"], args.cell_size)
    id_state = {
        "next_node": next_id(nodes.keys()),
        "next_way": next_id(ways.keys()),
        "next_relation": next_id([r["id"] for r in relations]),
    }
    stats = Counter()

    for link in links:
        lengths = cumulative_lengths(link["points"])
        total = lengths[-1]
        bad_s = []
        for point, s in zip(link["points"], lengths):
            nearest = center_index.nearest(point)
            stats["center_checked"] += 1
            if nearest is None or nearest["distance"] > args.target_distance:
                bad_s.append(s)
                stats["center_bad"] += 1
        for side in ("left", "right"):
            for point, s in mgeo_edge_points_with_s(link, side):
                nearest = boundary_index.nearest(point)
                stats["edge_checked"] += 1
                if nearest is None or nearest["distance"] > args.target_distance:
                    bad_s.append(s)
                    stats["edge_bad"] += 1
        for s0, s1 in intervals_from_bad_s(
            bad_s, total, args.augment_gap, args.augment_margin, args.augment_min_length
        ):
            relation_id = create_lanelet_from_link_interval(root, nodes, ways, relations, link, s0, s1, projector, id_state)
            if relation_id:
                stats["created_relations"] += 1

    if version is not None:
        update_meta_version(root, version)
    indent_xml(root)
    tree.write(output, encoding="UTF-8", xml_declaration=True)

    if args.copy_projector:
        projector_output = re.sub(r"_v\d+\.yaml$", "_v%d.yaml" % version, args.projector) if version else args.projector
        if projector_output != args.projector and (args.force or not os.path.exists(projector_output)):
            shutil.copy2(args.projector, projector_output)

    print("wrote=%s" % output)
    print(
        "target_distance=%.3f created_relations=%d center_bad=%d/%d edge_bad=%d/%d"
        % (
            args.target_distance,
            stats["created_relations"],
            stats["center_bad"],
            stats["center_checked"],
            stats["edge_bad"],
            stats["edge_checked"],
        )
    )
    print(
        "next_ids node=%d way=%d relation=%d"
        % (id_state["next_node"], id_state["next_way"], id_state["next_relation"])
    )
    return output


def generate_semantic_augment(args):
    output, version = (args.output, detect_version(args.output)) if args.output else default_next_output(args.lanelet)
    if os.path.exists(output) and not args.force:
        raise RuntimeError("%s already exists; pass --force to overwrite" % output)

    projector = load_projector(args.projector)
    tf_cfg = read_static_tf(args.static_tf)
    tree, root, nodes, ways, relations = parse_lanelet(args.lanelet, projector)
    links = load_mgeo_links(args.mgeo_link, tf_cfg)
    segs = build_lanelet_segment_sets(nodes, ways, relations)
    left_index = SegmentIndex(segs["lefts"], args.cell_size)
    right_index = SegmentIndex(segs["rights"], args.cell_size)
    road_border_index = SegmentIndex(segs["road_borders"], args.cell_size)
    id_state = {
        "next_node": next_id(nodes.keys()),
        "next_way": next_id(ways.keys()),
        "next_relation": next_id([r["id"] for r in relations]),
    }
    stats = Counter()

    for link in links:
        total = polyline_length(link["points"])
        bad_role_s = []
        for side, index in (("left", left_index), ("right", right_index)):
            for point, s in mgeo_edge_points_with_s(link, side):
                nearest = index.nearest(point)
                stats[side + "_checked"] += 1
                if nearest is None or nearest["distance"] > args.target_distance:
                    bad_role_s.append(s)
                    stats[side + "_bad"] += 1
        for s0, s1 in intervals_from_bad_s(
            bad_role_s, total, args.augment_gap, args.augment_margin, args.augment_min_length
        ):
            relation_id = create_lanelet_from_link_interval(root, nodes, ways, relations, link, s0, s1, projector, id_state)
            if relation_id:
                stats["created_role_relations"] += 1

        raw = link["raw"]
        for side, is_outer in (
            ("left", not raw.get("left_lane_change_dst_link_idx")),
            ("right", not raw.get("right_lane_change_dst_link_idx")),
        ):
            if not is_outer:
                continue
            bad_border_s = []
            for point, s in mgeo_edge_points_with_s(link, side):
                nearest = road_border_index.nearest(point)
                stats["outer_border_checked"] += 1
                if nearest is None or nearest["distance"] > args.target_distance:
                    bad_border_s.append(s)
                    stats["outer_border_bad"] += 1
            for s0, s1 in intervals_from_bad_s(
                bad_border_s, total, args.augment_gap, args.augment_margin, args.augment_min_length
            ):
                way_id = create_road_border_way_from_link_interval(
                    root, nodes, ways, link, side, s0, s1, projector, id_state
                )
                if way_id:
                    stats["created_road_border_ways"] += 1

    if version is not None:
        update_meta_version(root, version)
    indent_xml(root)
    tree.write(output, encoding="UTF-8", xml_declaration=True)

    if args.copy_projector:
        projector_output = re.sub(r"_v\d+\.yaml$", "_v%d.yaml" % version, args.projector) if version else args.projector
        if projector_output != args.projector and (args.force or not os.path.exists(projector_output)):
            shutil.copy2(args.projector, projector_output)

    print("wrote=%s" % output)
    print(
        "target_distance=%.3f created_role_relations=%d created_road_border_ways=%d"
        % (args.target_distance, stats["created_role_relations"], stats["created_road_border_ways"])
    )
    print(
        "left_bad=%d/%d right_bad=%d/%d outer_border_bad=%d/%d"
        % (
            stats["left_bad"],
            stats["left_checked"],
            stats["right_bad"],
            stats["right_checked"],
            stats["outer_border_bad"],
            stats["outer_border_checked"],
        )
    )
    return output


def generate(args):
    output, version = (args.output, detect_version(args.output)) if args.output else default_next_output(args.lanelet)
    if os.path.exists(output) and not args.force:
        raise RuntimeError("%s already exists; pass --force to overwrite" % output)

    projector = load_projector(args.projector)
    tf_cfg = read_static_tf(args.static_tf)
    tree, root, nodes, ways, relations = parse_lanelet(args.lanelet, projector)
    links = load_mgeo_links(args.mgeo_link, tf_cfg)
    matches, unmatched, counts = match_lanelets_to_mgeo(nodes, ways, relations, links, args.match_threshold)

    id_state = {
        "next_node": next_id(nodes.keys()),
        "next_way": next_id(ways.keys()),
        "next_relation": next_id([r["id"] for r in relations]),
    }

    updated_relations = 0
    updated_ways = 0
    skipped_shared = 0
    touched_ways = set()
    intervals_by_link = defaultdict(list)

    for rel_id, match in sorted(matches.items(), key=lambda kv: int(kv[0])):
        relation = match["relation"]
        link = match["link"]
        s_min = match["s_min"]
        s_max = match["s_max"]
        same_direction = match["same_direction"]
        center = subline(link["points"], s_min, s_max)
        if len(center) < 2:
            continue
        if not same_direction:
            center = list(reversed(center))
        left = mgeo_edge_points(center, link["width_start"], link["width_end"], "left")
        right = mgeo_edge_points(center, link["width_start"], link["width_end"], "right")
        center_id = relation_way_id(relation, "centerline")
        left_id = relation_way_id(relation, "left")
        right_id = relation_way_id(relation, "right")
        if not same_direction:
            left, right = right, left

        for way_id, points in ((center_id, center), (left_id, left), (right_id, right)):
            if way_id in touched_ways:
                skipped_shared += 1
                continue
            changed = replace_way_geometry(root, nodes, ways, way_id, points, projector, id_state)
            if changed:
                touched_ways.add(way_id)
                updated_ways += 1
        intervals_by_link[link["idx"]].append((s_min, s_max))
        updated_relations += 1

    created_relations = 0
    for link in links:
        total = polyline_length(link["points"])
        covered = merge_intervals(intervals_by_link[link["idx"]], args.coverage_gap)
        for s0, s1 in uncovered_intervals(total, covered, args.create_missing_min_length):
            relation_id = create_lanelet_from_link_interval(root, nodes, ways, relations, link, s0, s1, projector, id_state)
            if relation_id:
                created_relations += 1
                intervals_by_link[link["idx"]].append((s0, s1))

    if version is not None:
        update_meta_version(root, version)
    indent_xml(root)
    tree.write(output, encoding="UTF-8", xml_declaration=True)
    if args.copy_projector:
        projector_output = re.sub(r"_v\d+\.yaml$", "_v%d.yaml" % version, args.projector) if version else args.projector
        if projector_output != args.projector and (args.force or not os.path.exists(projector_output)):
            shutil.copy2(args.projector, projector_output)

    print("wrote=%s" % output)
    print(
        "matched_lanelet_relations=%d unmatched_lanelet_relations=%d updated_relations=%d "
        "updated_ways=%d shared_way_update_skips=%d created_missing_relations=%d"
        % (len(matches), len(unmatched), updated_relations, updated_ways, skipped_shared, created_relations)
    )
    if unmatched:
        worst = sorted(unmatched, key=lambda item: item[2], reverse=True)[:10]
        print("top_unmatched_lanelets:")
        for relation, candidate, rmse in worst:
            print("  relation=%s candidate=%s rmse=%.3f" % (relation["id"], candidate, rmse))
    print("mgeo_links_with_matched_lanelets=%d/%d" % (len(intervals_by_link), len(links)))
    uncovered_count = 0
    for link in links:
        total = polyline_length(link["points"])
        covered = merge_intervals(intervals_by_link[link["idx"]], args.coverage_gap)
        uncovered_count += len(uncovered_intervals(total, covered, args.create_missing_min_length))
    print("remaining_uncovered_mgeo_intervals_longer_than_%.2fm=%d" % (args.create_missing_min_length, uncovered_count))
    return output


def generate_snap(args):
    output, version = (args.output, detect_version(args.output)) if args.output else default_next_output(args.lanelet)
    if os.path.exists(output) and not args.force:
        raise RuntimeError("%s already exists; pass --force to overwrite" % output)

    projector = load_projector(args.projector)
    tf_cfg = read_static_tf(args.static_tf)
    tree, root, nodes, ways, relations = parse_lanelet(args.lanelet, projector)
    links = load_mgeo_links(args.mgeo_link, tf_cfg)

    center_way_ids = set()
    relation_boundary_way_ids = set()
    for relation in road_relations(relations):
        center_id = relation_way_id(relation, "centerline")
        left_id = relation_way_id(relation, "left")
        right_id = relation_way_id(relation, "right")
        if center_id:
            center_way_ids.add(center_id)
        if left_id:
            relation_boundary_way_ids.add(left_id)
        if right_id:
            relation_boundary_way_ids.add(right_id)

    road_border_way_ids = {way_id for way_id, way in ways.items() if way["tags"].get("type") == "road_border"}
    boundary_way_ids = relation_boundary_way_ids | road_border_way_ids

    center_segments = []
    edge_segments = []
    for link in links:
        center_segments.extend(make_segments(link["points"], link["idx"], link))
        edge_segments.extend(mgeo_edge_segments(link, "left"))
        edge_segments.extend(mgeo_edge_segments(link, "right"))
    center_index = SegmentIndex(center_segments, args.cell_size)
    edge_index = SegmentIndex(edge_segments, args.cell_size)
    max_ring = max(2, int(math.ceil(max(args.snap_threshold, args.cell_size) / args.cell_size)) + 2)

    targets = defaultdict(list)
    stats = Counter()

    def collect_targets(way_ids, index, kind):
        seen_refs = set()
        for way_id in way_ids:
            way = ways.get(way_id)
            if not way:
                continue
            for ref in way["refs"]:
                if ref in seen_refs or ref not in nodes:
                    continue
                seen_refs.add(ref)
                node = nodes[ref]
                p = (node["xy"][0], node["xy"][1], node["z"])
                stats[kind + "_candidate_nodes"] += 1
                nearest = index.nearest(p, max_ring=max_ring)
                if nearest is None or nearest["distance"] > args.snap_threshold:
                    stats[kind + "_protected_nodes"] += 1
                    continue
                targets[ref].append((projected_point(nearest), nearest["distance"], kind))
                stats[kind + "_target_nodes"] += 1

    collect_targets(center_way_ids, center_index, "center")
    collect_targets(boundary_way_ids, edge_index, "boundary")

    updated_nodes = 0
    conflict_nodes = 0
    total_before_sq = 0.0
    max_before = 0.0
    for ref, values in targets.items():
        if not values:
            continue
        if len(values) > 1:
            max_spread = 0.0
            for i, (a, _, _) in enumerate(values):
                for b, _, _ in values[i + 1 :]:
                    max_spread = max(max_spread, math.hypot(a[0] - b[0], a[1] - b[1]))
            if max_spread > args.conflict_tolerance:
                conflict_nodes += 1
                values = [min(values, key=lambda item: item[1])]
        x = sum(v[0][0] for v in values) / len(values)
        y = sum(v[0][1] for v in values) / len(values)
        z = sum(v[0][2] for v in values) / len(values)
        before = sum(v[1] for v in values) / len(values)
        total_before_sq += before * before
        max_before = max(max_before, before)
        set_node_position(nodes[ref], (x, y, z), projector)
        updated_nodes += 1

    if version is not None:
        update_meta_version(root, version)
    indent_xml(root)
    tree.write(output, encoding="UTF-8", xml_declaration=True)

    if args.copy_projector:
        projector_output = re.sub(r"_v\d+\.yaml$", "_v%d.yaml" % version, args.projector) if version else args.projector
        if projector_output != args.projector and (args.force or not os.path.exists(projector_output)):
            shutil.copy2(args.projector, projector_output)

    rms_before = math.sqrt(total_before_sq / updated_nodes) if updated_nodes else float("nan")
    print("wrote=%s" % output)
    print(
        "snap_threshold=%.3f updated_nodes=%d conflict_nodes=%d mean_snap_distance=%.6f max_snap_distance=%.6f"
        % (args.snap_threshold, updated_nodes, conflict_nodes, rms_before, max_before)
    )
    print(
        "center_candidate_nodes=%d center_target_nodes=%d center_protected_nodes=%d"
        % (stats["center_candidate_nodes"], stats["center_target_nodes"], stats["center_protected_nodes"])
    )
    print(
        "boundary_candidate_nodes=%d boundary_target_nodes=%d boundary_protected_nodes=%d"
        % (stats["boundary_candidate_nodes"], stats["boundary_target_nodes"], stats["boundary_protected_nodes"])
    )
    print(
        "mgeo_reference: links=%d center_segments=%d width_edge_segments=%d"
        % (len(links), len(center_segments), len(edge_segments))
    )
    return output


def build_feature_polylines(links):
    center_features = {}
    edge_features = {}
    for link in links:
        center_features[link["idx"]] = link["points"]
        edge_features[link["idx"] + ":left"] = mgeo_edge_points(
            link["points"], link["width_start"], link["width_end"], "left"
        )
        edge_features[link["idx"] + ":right"] = mgeo_edge_points(
            link["points"], link["width_start"], link["width_end"], "right"
        )
    return center_features, edge_features


def build_index_from_features(features, cell_size):
    segments = []
    for feature_id, points in features.items():
        segments.extend(make_segments(points, feature_id, points))
    return SegmentIndex(segments, cell_size), {feature_id: cumulative_lengths(points) for feature_id, points in features.items()}


def nearest_feature_projection(point, index, features, feature_lengths, threshold, max_ring):
    nearest = index.nearest(point, max_ring=max_ring)
    if nearest is None or nearest["distance"] > threshold:
        return None
    seg = nearest["segment"]
    feature_id = seg["feature_id"]
    points = features[feature_id]
    lengths = feature_lengths[feature_id]
    i = seg["part_index"]
    a = points[i]
    b = points[i + 1]
    seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
    s = lengths[i] + seg_len * nearest["t"]
    return {
        "feature_id": feature_id,
        "s": s,
        "point": projected_point(nearest),
        "distance": nearest["distance"],
    }


def rebuild_points_from_projections(original_points, projections, features, feature_lengths):
    if len(original_points) < 2:
        return original_points
    first = projections[0]["point"] if projections[0] else original_points[0]
    out = [first]
    for i in range(len(original_points) - 1):
        current = projections[i]
        nxt = projections[i + 1]
        if current and nxt and current["feature_id"] == nxt["feature_id"]:
            feature_id = current["feature_id"]
            section = subline(features[feature_id], current["s"], nxt["s"])
            for point in section[1:]:
                if not out or math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) > 1e-5:
                    out.append(point)
        else:
            point = nxt["point"] if nxt else original_points[i + 1]
            if not out or math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) > 1e-5:
                out.append(point)
    return out


def generate_densify(args):
    output, version = (args.output, detect_version(args.output)) if args.output else default_next_output(args.lanelet)
    if os.path.exists(output) and not args.force:
        raise RuntimeError("%s already exists; pass --force to overwrite" % output)

    projector = load_projector(args.projector)
    tf_cfg = read_static_tf(args.static_tf)
    tree, root, nodes, ways, relations = parse_lanelet(args.lanelet, projector)
    links = load_mgeo_links(args.mgeo_link, tf_cfg)

    center_way_ids = set()
    relation_boundary_way_ids = set()
    for relation in road_relations(relations):
        center_id = relation_way_id(relation, "centerline")
        left_id = relation_way_id(relation, "left")
        right_id = relation_way_id(relation, "right")
        if center_id:
            center_way_ids.add(center_id)
        if left_id:
            relation_boundary_way_ids.add(left_id)
        if right_id:
            relation_boundary_way_ids.add(right_id)
    road_border_way_ids = {way_id for way_id, way in ways.items() if way["tags"].get("type") == "road_border"}
    boundary_way_ids = relation_boundary_way_ids | road_border_way_ids

    center_features, edge_features = build_feature_polylines(links)
    center_index, center_lengths = build_index_from_features(center_features, args.cell_size)
    edge_index, edge_lengths = build_index_from_features(edge_features, args.cell_size)
    max_ring = max(2, int(math.ceil(max(args.densify_threshold, args.cell_size) / args.cell_size)) + 2)
    id_state = {"next_node": next_id(nodes.keys())}
    stats = Counter()

    def rebuild_way_set(way_ids, index, features, lengths, kind):
        rebuilt = 0
        old_node_total = 0
        new_node_total = 0
        partial = 0
        for way_id in sorted(way_ids, key=lambda v: int(v)):
            if way_id not in ways:
                continue
            refs = ways[way_id]["refs"]
            original = []
            for ref in refs:
                if ref not in nodes:
                    continue
                node = nodes[ref]
                original.append((node["xy"][0], node["xy"][1], node["z"]))
            if len(original) < 2:
                continue
            projections = [
                nearest_feature_projection(point, index, features, lengths, args.densify_threshold, max_ring)
                for point in original
            ]
            projected_count = sum(1 for p in projections if p)
            stats[kind + "_projected_nodes"] += projected_count
            stats[kind + "_protected_nodes"] += len(projections) - projected_count
            if projected_count < 2:
                continue
            if projected_count < len(projections):
                partial += 1
            rebuilt_points = rebuild_points_from_projections(original, projections, features, lengths)
            if len(rebuilt_points) < 2:
                continue
            replace_way_geometry(root, nodes, ways, way_id, rebuilt_points, projector, id_state)
            rebuilt += 1
            old_node_total += len(original)
            new_node_total += len(rebuilt_points)
        stats[kind + "_rebuilt_ways"] += rebuilt
        stats[kind + "_partial_ways"] += partial
        stats[kind + "_old_nodes"] += old_node_total
        stats[kind + "_new_nodes"] += new_node_total

    rebuild_way_set(center_way_ids, center_index, center_features, center_lengths, "center")
    rebuild_way_set(boundary_way_ids, edge_index, edge_features, edge_lengths, "boundary")

    if version is not None:
        update_meta_version(root, version)
    indent_xml(root)
    tree.write(output, encoding="UTF-8", xml_declaration=True)

    if args.copy_projector:
        projector_output = re.sub(r"_v\d+\.yaml$", "_v%d.yaml" % version, args.projector) if version else args.projector
        if projector_output != args.projector and (args.force or not os.path.exists(projector_output)):
            shutil.copy2(args.projector, projector_output)

    print("wrote=%s" % output)
    print("densify_threshold=%.3f" % args.densify_threshold)
    print(
        "center_rebuilt_ways=%d center_partial_ways=%d center_nodes=%d->%d center_projected_nodes=%d center_protected_nodes=%d"
        % (
            stats["center_rebuilt_ways"],
            stats["center_partial_ways"],
            stats["center_old_nodes"],
            stats["center_new_nodes"],
            stats["center_projected_nodes"],
            stats["center_protected_nodes"],
        )
    )
    print(
        "boundary_rebuilt_ways=%d boundary_partial_ways=%d boundary_nodes=%d->%d boundary_projected_nodes=%d boundary_protected_nodes=%d"
        % (
            stats["boundary_rebuilt_ways"],
            stats["boundary_partial_ways"],
            stats["boundary_old_nodes"],
            stats["boundary_new_nodes"],
            stats["boundary_projected_nodes"],
            stats["boundary_protected_nodes"],
        )
    )
    return output


def main():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lanelet", default=os.path.join(repo_root, "kcity_map", "lanelet_map", "lanelet2_map_v1.osm"))
    parser.add_argument("--projector", default=os.path.join(repo_root, "kcity_map", "lanelet_map", "map_projector_info_v1.yaml"))
    parser.add_argument("--mgeo-link", default=os.path.join(repo_root, "kcity_map", "mgeo_map", "link_set.json"))
    parser.add_argument("--static-tf", default=os.path.join(repo_root, "configs", "tf", "morai_lanelet_static_tf.yaml"))
    parser.add_argument("--cell-size", type=float, default=5.0)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("compare")

    generate_parser = subparsers.add_parser("generate")
    generate_parser.add_argument("--output")
    generate_parser.add_argument("--force", action="store_true")
    generate_parser.add_argument("--match-threshold", type=float, default=4.0)
    generate_parser.add_argument("--coverage-gap", type=float, default=1.0)
    generate_parser.add_argument("--create-missing-min-length", type=float, default=2.0)
    generate_parser.add_argument("--copy-projector", action="store_true")

    snap_parser = subparsers.add_parser("snap-generate")
    snap_parser.add_argument("--output")
    snap_parser.add_argument("--force", action="store_true")
    snap_parser.add_argument("--snap-threshold", type=float, default=2.0)
    snap_parser.add_argument("--conflict-tolerance", type=float, default=0.2)
    snap_parser.add_argument("--copy-projector", action="store_true")

    densify_parser = subparsers.add_parser("densify-generate")
    densify_parser.add_argument("--output")
    densify_parser.add_argument("--force", action="store_true")
    densify_parser.add_argument("--densify-threshold", type=float, default=0.5)
    densify_parser.add_argument("--copy-projector", action="store_true")

    augment_parser = subparsers.add_parser("augment-generate")
    augment_parser.add_argument("--output")
    augment_parser.add_argument("--force", action="store_true")
    augment_parser.add_argument("--target-distance", type=float, default=0.02)
    augment_parser.add_argument("--augment-gap", type=float, default=2.0)
    augment_parser.add_argument("--augment-margin", type=float, default=1.0)
    augment_parser.add_argument("--augment-min-length", type=float, default=0.2)
    augment_parser.add_argument("--copy-projector", action="store_true")

    semantic_parser = subparsers.add_parser("semantic-augment-generate")
    semantic_parser.add_argument("--output")
    semantic_parser.add_argument("--force", action="store_true")
    semantic_parser.add_argument("--target-distance", type=float, default=0.02)
    semantic_parser.add_argument("--augment-gap", type=float, default=2.0)
    semantic_parser.add_argument("--augment-margin", type=float, default=1.0)
    semantic_parser.add_argument("--augment-min-length", type=float, default=0.2)
    semantic_parser.add_argument("--copy-projector", action="store_true")

    args = parser.parse_args()
    if args.command == "compare":
        compare(args)
    elif args.command == "generate":
        generate(args)
    elif args.command == "snap-generate":
        generate_snap(args)
    elif args.command == "densify-generate":
        generate_densify(args)
    elif args.command == "augment-generate":
        generate_augment(args)
    elif args.command == "semantic-augment-generate":
        generate_semantic_augment(args)


if __name__ == "__main__":
    main()
