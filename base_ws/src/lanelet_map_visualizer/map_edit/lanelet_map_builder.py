#!/usr/bin/env python3
"""Build and validate a VMB-friendly K-City Lanelet map from v1.

The repository MGeo export only has link/node/global data, so the builder uses
MGeo link centerlines and width-derived edges as a geometric reference.  It
keeps the v1 Lanelet regulatory/facility structure, snaps only existing v1
nodes to high-confidence MGeo geometry, then fixes duplicated adjacent lane
boundaries by making neighboring lanelets share the same boundary way.
"""

import argparse
import math
import os
import re
import shutil
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "scripts", "map", "kcity"))

import kcity_map_align as align  # noqa: E402


DEFAULT_PROJECTOR = os.path.join(REPO_ROOT, "kcity_map", "lanelet_map", "map_projector_info_v1.yaml")
DEFAULT_SOURCE = os.path.join(REPO_ROOT, "kcity_map", "lanelet_map", "lanelet2_map_v1.osm")
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "kcity_map", "lanelet_map", "lanelet2_map_v23.osm")
DEFAULT_MGEO_LINK = os.path.join(REPO_ROOT, "kcity_map", "mgeo_map", "link_set.json")
DEFAULT_STATIC_TF = os.path.join(REPO_ROOT, "configs", "tf", "morai_lanelet_static_tf.yaml")


def relation_way_id(relation, role):
    return align.relation_way_id(relation, role)


def direction(points):
    if len(points) < 2:
        return (0.0, 0.0)
    dx = points[-1][0] - points[0][0]
    dy = points[-1][1] - points[0][1]
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        return (0.0, 0.0)
    return (dx / norm, dy / norm)


def point_polyline_distance(point, points):
    best = float("inf")
    for a, b in zip(points, points[1:]):
        distance_sq, _ = align.dist_point_segment_sq(point, a, b)
        best = min(best, math.sqrt(distance_sq))
    return best


def sample_points(points, max_samples):
    if len(points) <= max_samples:
        return list(points)
    return [points[round(i * (len(points) - 1) / float(max_samples - 1))] for i in range(max_samples)]


def distance_profile(source_points, target_points, max_samples=25):
    distances = [point_polyline_distance(point, target_points) for point in sample_points(source_points, max_samples)]
    if not distances:
        return float("inf"), float("inf"), 0.0
    return (
        sum(distances) / len(distances),
        max(distances),
        sum(1 for distance in distances if distance <= 0.08) / float(len(distances)),
    )


def centerline_separation(source_center, target_center):
    distances = [point_polyline_distance(point, target_center) for point in sample_points(source_center, 15)]
    return sum(distances) / len(distances) if distances else float("inf")


def target_is_right_of_source(source_record, target_record):
    source_mid = source_record["center"][len(source_record["center"]) // 2]
    target_mid = target_record["center"][len(target_record["center"]) // 2]
    dx = target_mid[0] - source_mid[0]
    dy = target_mid[1] - source_mid[1]
    direction_x, direction_y = source_record["direction"]
    return direction_x * dy - direction_y * dx < -0.5


def way_points(way_id, ways, nodes):
    return align.way_points(way_id, ways, nodes) if way_id in ways else []


def build_road_records(nodes, ways, relations):
    records = []
    for relation in align.road_relations(relations):
        center_id = relation_way_id(relation, "centerline")
        left_id = relation_way_id(relation, "left")
        right_id = relation_way_id(relation, "right")
        center = way_points(center_id, ways, nodes)
        left = way_points(left_id, ways, nodes)
        right = way_points(right_id, ways, nodes)
        if len(center) < 2 and len(left) >= 2 and len(right) >= 2:
            reference = left if align.polyline_length(left) >= align.polyline_length(right) else right
            center = midpoint_polyline_from_boundaries(left, right, reference, 0.0)
        if len(center) < 2 or len(left) < 2 or len(right) < 2:
            continue
        records.append(
            {
                "id": relation["id"],
                "relation": relation,
                "center": center,
                "left": left,
                "right": right,
                "left_id": left_id,
                "right_id": right_id,
                "direction": direction(center),
            }
        )
    return records


def add_to_grid(grid, record_index, points, cell_size):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    for cx in range(math.floor(min(xs) / cell_size) - 1, math.floor(max(xs) / cell_size) + 2):
        for cy in range(math.floor(min(ys) / cell_size) - 1, math.floor(max(ys) / cell_size) + 2):
            grid[(cx, cy)].append(record_index)


def nearby_record_indexes(grid, points, cell_size):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    indexes = set()
    for cx in range(math.floor(min(xs) / cell_size) - 1, math.floor(max(xs) / cell_size) + 2):
        for cy in range(math.floor(min(ys) / cell_size) - 1, math.floor(max(ys) / cell_size) + 2):
            indexes.update(grid.get((cx, cy), ()))
    return indexes


def detect_shared_lane_candidates(nodes, ways, relations, args):
    records = build_road_records(nodes, ways, relations)
    left_grid = defaultdict(list)
    for index, record in enumerate(records):
        add_to_grid(left_grid, index, record["left"], args.grid_cell_size)

    candidates = []
    for source_index, source in enumerate(records):
        for target_index in nearby_record_indexes(left_grid, source["right"], args.grid_cell_size):
            if source_index == target_index:
                continue
            target = records[target_index]
            direction_dot = (
                source["direction"][0] * target["direction"][0] + source["direction"][1] * target["direction"][1]
            )
            if direction_dot < args.min_direction_dot:
                continue
            if not target_is_right_of_source(source, target):
                continue

            separation = centerline_separation(source["center"], target["center"])
            if separation < args.min_center_separation or separation > args.max_center_separation:
                continue

            source_to_target = distance_profile(source["right"], target["left"])
            target_to_source = distance_profile(target["left"], source["right"])
            mean_distance = max(source_to_target[0], target_to_source[0])
            max_distance = max(source_to_target[1], target_to_source[1])
            coverage = min(source_to_target[2], target_to_source[2])

            if mean_distance > args.max_mean_distance:
                continue
            if max_distance > args.max_max_distance:
                continue
            if coverage < args.min_coverage:
                continue

            candidates.append(
                {
                    "source_relation_id": source["id"],
                    "target_relation_id": target["id"],
                    "source_right_way": source["right_id"],
                    "target_left_way": target["left_id"],
                    "target_relation": target["relation"],
                    "target_role": "left",
                    "mean_distance": mean_distance,
                    "max_distance": max_distance,
                    "coverage": coverage,
                    "center_separation": separation,
                    "direction_dot": direction_dot,
                }
            )
    candidates.sort(key=lambda item: (item["target_relation_id"], item["target_role"], item["mean_distance"]))
    return candidates


def set_relation_member_ref(relation, role, old_ref, new_ref):
    changed = False
    for member in relation["members"]:
        if member["type"] == "way" and member["role"] == role and member["ref"] == old_ref:
            member["ref"] = new_ref
            changed = True
    for child in relation["elem"]:
        if (
            child.tag == "member"
            and child.attrib.get("type") == "way"
            and child.attrib.get("role") == role
            and child.attrib.get("ref") == old_ref
        ):
            child.attrib["ref"] = new_ref
    return changed


def snap_existing_nodes_to_mgeo(nodes, ways, relations, links, args):
    center_way_ids = set()
    relation_boundary_way_ids = set()
    for relation in align.road_relations(relations):
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
        center_segments.extend(align.make_segments(link["points"], link["idx"], link))
        edge_segments.extend(align.mgeo_edge_segments(link, "left"))
        edge_segments.extend(align.mgeo_edge_segments(link, "right"))
    center_index = align.SegmentIndex(center_segments, args.snap_cell_size)
    edge_index = align.SegmentIndex(edge_segments, args.snap_cell_size)
    max_ring = max(2, int(math.ceil(max(args.snap_threshold, args.snap_cell_size) / args.snap_cell_size)) + 2)

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
                point = (node["xy"][0], node["xy"][1], node["z"])
                stats[kind + "_candidate_nodes"] += 1
                nearest = index.nearest(point, max_ring=max_ring)
                if nearest is None or nearest["distance"] > args.snap_threshold:
                    stats[kind + "_protected_nodes"] += 1
                    continue
                targets[ref].append((align.projected_point(nearest), nearest["distance"], kind))
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
            for index, (a, _, _) in enumerate(values):
                for b, _, _ in values[index + 1 :]:
                    max_spread = max(max_spread, math.hypot(a[0] - b[0], a[1] - b[1]))
            if max_spread > args.snap_conflict_tolerance:
                conflict_nodes += 1
                values = [min(values, key=lambda item: item[1])]
        x = sum(value[0][0] for value in values) / len(values)
        y = sum(value[0][1] for value in values) / len(values)
        z = sum(value[0][2] for value in values) / len(values)
        before = sum(value[1] for value in values) / len(values)
        total_before_sq += before * before
        max_before = max(max_before, before)
        align.set_node_position(nodes[ref], (x, y, z), args.projector_data)
        updated_nodes += 1

    stats["updated_nodes"] = updated_nodes
    stats["conflict_nodes"] = conflict_nodes
    stats["mean_snap_distance"] = math.sqrt(total_before_sq / updated_nodes) if updated_nodes else float("nan")
    stats["max_snap_distance"] = max_before
    stats["mgeo_links"] = len(links)
    stats["center_segments"] = len(center_segments)
    stats["edge_segments"] = len(edge_segments)
    return stats


def relation_way_reference_counts(relations):
    counts = Counter()
    for relation in relations:
        for member in relation["members"]:
            if member["type"] == "way":
                counts[member["ref"]] += 1
    return counts


class DisjointSet:
    def __init__(self):
        self.parent = {}

    def find(self, item):
        self.parent.setdefault(item, item)
        if self.parent[item] != item:
            self.parent[item] = self.find(self.parent[item])
        return self.parent[item]

    def union(self, a, b):
        root_a = self.find(a)
        root_b = self.find(b)
        if root_a != root_b:
            self.parent[root_b] = root_a

    def groups(self):
        result = defaultdict(set)
        for item in self.parent:
            result[self.find(item)].add(item)
        return list(result.values())


def remove_way(root, ways, way_id):
    way = ways.pop(way_id, None)
    if way is None:
        return False
    root.remove(way["elem"])
    return True


def prune_unused_nodes(root, nodes, ways):
    used_nodes = {ref for way in ways.values() for ref in way["refs"]}
    unused_nodes = set(nodes) - used_nodes
    if not unused_nodes:
        return 0
    root[:] = [
        elem
        for elem in root
        if not (elem.tag == "node" and elem.attrib.get("id") in unused_nodes)
    ]
    for node_id in unused_nodes:
        del nodes[node_id]
    return len(unused_nodes)


def set_node_elevation(node, z, projector):
    ele = z + projector["origin_alt"]
    node["z"] = z
    node["ele"] = ele
    ele_tag = None
    for child in node["elem"]:
        if child.tag == "tag" and child.attrib.get("k") == "ele":
            ele_tag = child
            break
    if ele_tag is None:
        ET.SubElement(node["elem"], "tag", {"k": "ele", "v": "%.3f" % ele})
    else:
        ele_tag.attrib["v"] = "%.3f" % ele


def flatten_node_elevations(nodes, projector, value=None):
    zs = [node.get("z", 0.0) for node in nodes.values()]
    if not zs:
        return {
            "nodes": 0,
            "target_z": 0.0,
            "min_before": 0.0,
            "max_before": 0.0,
            "mean_before": 0.0,
        }
    target_z = float(value) if value is not None else float(statistics.median(zs))
    for node in nodes.values():
        set_node_elevation(node, target_z, projector)
    return {
        "nodes": len(nodes),
        "target_z": target_z,
        "min_before": min(zs),
        "max_before": max(zs),
        "mean_before": sum(zs) / len(zs),
    }


def adjacent_distance(nodes, first_ref, second_ref):
    if first_ref not in nodes or second_ref not in nodes:
        return None
    first = nodes[first_ref]["xy"]
    second = nodes[second_ref]["xy"]
    return math.hypot(first[0] - second[0], first[1] - second[1])


def replace_way_refs(way, refs):
    way["refs"] = refs
    elem = way["elem"]
    for child in list(elem):
        if child.tag == "nd":
            elem.remove(child)
    for index, ref in enumerate(refs):
        elem.insert(index, ET.Element("nd", {"ref": ref}))


def lanelet_boundary_way_ids(relations):
    way_ids = set()
    for relation in relations:
        if relation["tags"].get("type") != "lanelet":
            continue
        for member in relation["members"]:
            if member["type"] == "way" and member["role"] in ("left", "right"):
                way_ids.add(member["ref"])
    return way_ids


def way_point_angle_degrees(nodes, refs, index):
    if index <= 0 or index >= len(refs) - 1:
        return None
    prev_ref = refs[index - 1]
    ref = refs[index]
    next_ref = refs[index + 1]
    if prev_ref not in nodes or ref not in nodes or next_ref not in nodes:
        return None
    prev_xy = nodes[prev_ref]["xy"]
    xy = nodes[ref]["xy"]
    next_xy = nodes[next_ref]["xy"]
    first = (prev_xy[0] - xy[0], prev_xy[1] - xy[1])
    second = (next_xy[0] - xy[0], next_xy[1] - xy[1])
    first_norm = math.hypot(first[0], first[1])
    second_norm = math.hypot(second[0], second[1])
    if first_norm < 1e-9 or second_norm < 1e-9:
        return None
    dot = max(-1.0, min(1.0, (first[0] * second[0] + first[1] * second[1]) / (first_norm * second_norm)))
    return math.degrees(math.acos(dot))


def steep_lanelet_angle_stats(nodes, ways, relations, min_angle):
    stats = Counter()
    examples = []
    for way_id in lanelet_boundary_way_ids(relations):
        way = ways.get(way_id)
        if not way:
            continue
        for index in range(1, len(way["refs"]) - 1):
            angle = way_point_angle_degrees(nodes, way["refs"], index)
            if angle is None or angle >= min_angle:
                continue
            stats["steep_angle_points"] += 1
            if len(examples) < 8:
                examples.append((angle, way_id, way["refs"][index - 1], way["refs"][index], way["refs"][index + 1]))
    stats["examples"] = examples
    return stats


def remove_steep_lanelet_angles(root, nodes, ways, relations, min_angle):
    stats = Counter()
    if min_angle <= 0.0:
        return stats
    for way_id in sorted(lanelet_boundary_way_ids(relations), key=lambda value: int(value)):
        way = ways.get(way_id)
        if not way:
            continue
        refs = list(way["refs"])
        original_len = len(refs)
        index = 1
        while index < len(refs) - 1:
            angle = way_point_angle_degrees(nodes, refs, index)
            if angle is None or angle >= min_angle:
                index += 1
                continue
            stats["removed_steep_angle_points"] += 1
            if len(stats.get("examples", [])) < 8:
                stats.setdefault("examples", []).append((angle, way_id, refs[index - 1], refs[index], refs[index + 1]))
            refs.pop(index)
            if index > 1:
                index -= 1
        if len(refs) == original_len:
            continue
        replace_way_refs(way, refs)
        stats["changed_ways"] += 1

    stats["removed_unused_nodes"] = prune_unused_nodes(root, nodes, ways)
    return stats


def adjacent_segment_stats(nodes, ways, min_distance):
    stats = Counter()
    stats["min_distance_mm"] = 0
    min_distance_seen = float("inf")
    examples = []
    for way_id, way in ways.items():
        refs = way["refs"]
        for first_ref, second_ref in zip(refs, refs[1:]):
            distance = adjacent_distance(nodes, first_ref, second_ref)
            if distance is None:
                continue
            min_distance_seen = min(min_distance_seen, distance)
            if distance < min_distance:
                stats["short_adjacent_pairs"] += 1
                if len(refs) <= 2:
                    stats["short_unremovable_pairs"] += 1
                if len(examples) < 8:
                    examples.append((distance, way_id, first_ref, second_ref, way["tags"].get("type", "")))
    if min_distance_seen < float("inf"):
        stats["min_distance_mm"] = int(round(min_distance_seen * 1000.0))
    stats["examples"] = examples
    return stats


def linestring_z_step_stats(nodes, ways, max_step):
    stats = Counter()
    stats["max_step_mm"] = 0
    max_step_seen = 0.0
    examples = []
    for way_id, way in ways.items():
        refs = way["refs"]
        for first_ref, second_ref in zip(refs, refs[1:]):
            if first_ref not in nodes or second_ref not in nodes:
                continue
            dz = abs(nodes[first_ref].get("z", 0.0) - nodes[second_ref].get("z", 0.0))
            max_step_seen = max(max_step_seen, dz)
            if dz > max_step:
                stats["over_threshold"] += 1
                if len(examples) < 8:
                    stats.setdefault("examples", []).append(
                        (
                            dz,
                            way_id,
                            first_ref,
                            second_ref,
                            way["tags"].get("type", ""),
                            way["tags"].get("subtype", ""),
                        )
                    )
    stats["max_step_mm"] = int(round(max_step_seen * 1000.0))
    stats["examples"] = examples
    return stats


def remove_short_adjacent_way_points(root, nodes, ways, min_distance):
    stats = Counter()
    if min_distance <= 0.0:
        return stats

    for way_id, way in list(ways.items()):
        refs = list(way["refs"])
        if len(refs) < 2:
            continue
        original_len = len(refs)
        removed_refs = []
        index = 0
        while index < len(refs) - 1:
            distance = adjacent_distance(nodes, refs[index], refs[index + 1])
            if distance is None or distance >= min_distance:
                index += 1
                continue
            if len(refs) <= 2:
                stats["unremovable_short_pairs"] += 1
                index += 1
                continue

            if index == 0:
                remove_index = 1
            elif index == len(refs) - 2:
                remove_index = index
            else:
                remove_index = index + 1
            removed_refs.append(refs.pop(remove_index))
            if remove_index <= index and index > 0:
                index -= 1

        if len(refs) == original_len:
            continue
        replace_way_refs(way, refs)
        stats["changed_ways"] += 1
        stats["removed_way_points"] += original_len - len(refs)
        if len(removed_refs) and len(stats.get("examples", [])) < 8:
            stats.setdefault("examples", []).append((way_id, removed_refs[:4]))

    stats["removed_unused_nodes"] = prune_unused_nodes(root, nodes, ways)
    return stats


def remove_tag(elem, tags, key):
    tags.pop(key, None)
    for child in list(elem):
        if child.tag == "tag" and child.attrib.get("k") == key:
            elem.remove(child)


def set_tag(elem, tags, key, value):
    value = str(value)
    tags[key] = value
    for child in elem:
        if child.tag == "tag" and child.attrib.get("k") == key:
            child.attrib["v"] = value
            return
    ET.SubElement(elem, "tag", {"k": key, "v": value})


def format_speed(value):
    try:
        speed = float(value)
    except (TypeError, ValueError):
        return None
    if abs(speed - round(speed)) < 1e-6:
        return "%d km/h" % int(round(speed))
    return "%.3f km/h" % speed


def way_mgeo_edge_rmse(way_id, nodes, ways, edge_index, max_ring):
    if edge_index is None or way_id not in ways:
        return float("inf")
    distances = []
    for point in sample_points(way_points(way_id, ways, nodes), 25):
        nearest = edge_index.nearest(point, max_ring=max_ring)
        if nearest is None:
            continue
        distances.append(nearest["distance"])
    if not distances:
        return float("inf")
    return math.sqrt(sum(distance * distance for distance in distances) / len(distances))


def is_lane_marking_way(way):
    tags = way.get("tags", {})
    return tags.get("type") == "line_thin" and tags.get("subtype") != "stop_line"


def dominant_mgeo_link_for_center(center_points, link_index, max_ring):
    votes = defaultdict(lambda: [0, 0.0])
    for point in sample_points(center_points, 25):
        nearest = link_index.nearest(point, max_ring=max_ring)
        if nearest is None:
            continue
        link = nearest["segment"]["source"]
        votes[link["idx"]][0] += 1
        votes[link["idx"]][1] += nearest["distance"] ** 2
    if not votes:
        return None, float("inf")
    link_id, values = min(votes.items(), key=lambda item: math.sqrt(item[1][1] / item[1][0]))
    return link_id, math.sqrt(values[1] / values[0])


def normalize_road_lanelet_tags(nodes, ways, relations, links, args):
    link_by_id = {link["idx"]: link for link in links}
    center_segments = []
    for link in links:
        center_segments.extend(align.make_segments(link["points"], link["idx"], link))
    link_index = align.SegmentIndex(center_segments, args.snap_cell_size)
    max_ring = max(2, int(math.ceil(max(args.speed_match_threshold, args.snap_cell_size) / args.snap_cell_size)) + 2)

    stats = Counter()
    for relation in align.road_relations(relations):
        center_id = relation_way_id(relation, "centerline")
        center = way_points(center_id, ways, nodes)
        link_id, rmse = dominant_mgeo_link_for_center(center, link_index, max_ring) if center else (None, float("inf"))
        link = link_by_id.get(link_id)

        set_tag(relation["elem"], relation["tags"], "location", relation["tags"].get("location") or "urban")
        set_tag(relation["elem"], relation["tags"], "one_way", relation["tags"].get("one_way") or "yes")
        set_tag(
            relation["elem"],
            relation["tags"],
            "participant:vehicle",
            relation["tags"].get("participant:vehicle") or "yes",
        )

        speed = None
        if link is not None and rmse <= args.speed_match_threshold:
            speed = format_speed(link["raw"].get("max_speed"))
        if speed is None:
            speed = relation["tags"].get("speed_limit") or "15 km/h"
        previous = relation["tags"].get("speed_limit")
        set_tag(relation["elem"], relation["tags"], "speed_limit", speed)

        stats["road_lanelets"] += 1
        if link is not None and rmse <= args.speed_match_threshold:
            stats["speed_from_mgeo"] += 1
        if previous != speed:
            stats["speed_changed"] += 1
    return stats


def relation_by_id(relations):
    return {relation["id"]: relation for relation in relations}


def midpoint(points):
    if not points:
        return None
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
        sum((point[2] if len(point) > 2 else 0.0) for point in points) / len(points),
    )


def interpolate_ratio(points, ratio):
    if not points:
        return None
    lengths = align.cumulative_lengths(points)
    if lengths[-1] <= 1e-9:
        return points[0]
    return align.interpolate_polyline(points, lengths, max(0.0, min(1.0, ratio)) * lengths[-1])


def line_direction(points, fallback_points):
    if len(points) >= 2:
        dx = points[-1][0] - points[0][0]
        dy = points[-1][1] - points[0][1]
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            return (dx / norm, dy / norm)
    best = None
    best_dist = 0.0
    for index, a_point in enumerate(fallback_points):
        for b_point in fallback_points[index + 1 :]:
            dist = math.hypot(b_point[0] - a_point[0], b_point[1] - a_point[1])
            if dist > best_dist:
                best = (a_point, b_point)
                best_dist = dist
    if best is None or best_dist <= 1e-6:
        return (1.0, 0.0)
    return ((best[1][0] - best[0][0]) / best_dist, (best[1][1] - best[0][1]) / best_dist)


def orient_points_to_reference(points, reference):
    if len(points) < 2 or len(reference) < 2:
        return list(points)
    forward_cost = math.hypot(points[0][0] - reference[0][0], points[0][1] - reference[0][1])
    forward_cost += math.hypot(points[-1][0] - reference[-1][0], points[-1][1] - reference[-1][1])
    reverse_cost = math.hypot(points[-1][0] - reference[0][0], points[-1][1] - reference[0][1])
    reverse_cost += math.hypot(points[0][0] - reference[-1][0], points[0][1] - reference[-1][1])
    return list(points) if forward_cost <= reverse_cost else list(reversed(points))


def midpoint_polyline_from_boundaries(left_points, right_points, reference_points, max_spacing):
    if len(left_points) < 2 or len(right_points) < 2 or len(reference_points) < 2:
        return []
    left = orient_points_to_reference(left_points, reference_points)
    right = orient_points_to_reference(right_points, reference_points)
    left_lengths = align.cumulative_lengths(left)
    right_lengths = align.cumulative_lengths(right)
    reference_lengths = align.cumulative_lengths(reference_points)
    total = reference_lengths[-1]
    if total <= 1e-6 or left_lengths[-1] <= 1e-6 or right_lengths[-1] <= 1e-6:
        return []

    if max_spacing > 0.0:
        sample_count = max(2, int(math.ceil(total / max_spacing)) + 1)
        sample_distances = [i * total / float(sample_count - 1) for i in range(sample_count)]
    else:
        sample_distances = list(reference_lengths)
    sample_distances = sorted(set(round(value, 6) for value in sample_distances))

    points = []
    for distance in sample_distances:
        ratio = max(0.0, min(1.0, distance / total))
        left_point = align.interpolate_polyline(left, left_lengths, ratio * left_lengths[-1])
        right_point = align.interpolate_polyline(right, right_lengths, ratio * right_lengths[-1])
        z = ((left_point[2] if len(left_point) > 2 else 0.0) + (right_point[2] if len(right_point) > 2 else 0.0)) * 0.5
        point = ((left_point[0] + right_point[0]) * 0.5, (left_point[1] + right_point[1]) * 0.5, z)
        if points and math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) < 1e-4:
            continue
        points.append(point)
    return points


def rebuild_lanelet_centerlines_from_boundaries(root, nodes, ways, relations, projector, args):
    stats = Counter()
    id_state = {
        "next_node": max(int(node_id) for node_id in nodes) + 1,
        "next_way": max(int(way_id) for way_id in ways) + 1,
    }
    way_ref_counts = relation_way_reference_counts(relations)

    for relation in align.road_relations(relations):
        center_id = relation_way_id(relation, "centerline")
        left_id = relation_way_id(relation, "left")
        right_id = relation_way_id(relation, "right")
        center = way_points(center_id, ways, nodes)
        left = way_points(left_id, ways, nodes)
        right = way_points(right_id, ways, nodes)
        if len(center) < 2 or len(left) < 2 or len(right) < 2:
            stats["skipped"] += 1
            continue
        rebuilt = midpoint_polyline_from_boundaries(left, right, center, args.centerline_max_spacing)
        if len(rebuilt) < 2:
            stats["skipped"] += 1
            continue

        refs = []
        for point in rebuilt:
            ref = str(id_state["next_node"])
            id_state["next_node"] += 1
            refs.append(align.append_node(root, nodes, ref, point, projector))

        if way_ref_counts[center_id] > 1:
            old_tags = ways.get(center_id, {}).get("tags", {})
            new_way_id = str(id_state["next_way"])
            id_state["next_way"] += 1
            align.append_way(root, ways, new_way_id, refs, dict(old_tags) if old_tags else {"type": "virtual"})
            set_relation_member_ref(relation, "centerline", center_id, new_way_id)
            stats["created_centerline_ways"] += 1
        else:
            replace_way_refs(ways[center_id], refs)
            stats["replaced_centerline_ways"] += 1
        stats["created_centerline_nodes"] += len(refs)

    sync_relation_members_from_xml(relations)
    stats["removed_unused_nodes"] = prune_unused_nodes(root, nodes, ways)
    return stats


def align_stop_lines_to_referrer_lanelets(root, nodes, ways, relations, projector, args):
    relations_by_id = relation_by_id(relations)
    stats = Counter()
    id_state = {
        "next_node": max(int(node_id) for node_id in nodes) + 1,
        "next_way": max(int(way_id) for way_id in ways) + 1,
    }
    old_stopline_ids = set()

    for lanelet in align.road_relations(relations):
        regulatory_ids = [
            member["ref"] for member in lanelet["members"] if member["type"] == "relation" and member["role"] == "regulatory_element"
        ]
        if not regulatory_ids:
            continue
        center = way_points(relation_way_id(lanelet, "centerline"), ways, nodes)
        left = way_points(relation_way_id(lanelet, "left"), ways, nodes)
        right = way_points(relation_way_id(lanelet, "right"), ways, nodes)
        if len(center) < 2 or len(left) < 2 or len(right) < 2:
            continue

        for regulatory_id in regulatory_ids:
            regulatory = relations_by_id.get(regulatory_id)
            if not regulatory or regulatory["tags"].get("subtype") != "traffic_light":
                continue
            ref_line_ids = [
                member["ref"] for member in regulatory["members"] if member["type"] == "way" and member["role"] == "ref_line"
            ]
            for stopline_id in ref_line_ids:
                original = way_points(stopline_id, ways, nodes)
                original_mid = midpoint(original)
                if original_mid is None:
                    continue
                projection = align.project_point_to_polyline(original_mid, center)
                if projection is None:
                    continue
                center_lengths = align.cumulative_lengths(center)
                if center_lengths[-1] > args.stopline_lane_margin * 2.0:
                    projected_s = max(
                        args.stopline_lane_margin,
                        min(center_lengths[-1] - args.stopline_lane_margin, projection["s"]),
                    )
                    ratio = projected_s / center_lengths[-1]
                else:
                    ratio = 0.5
                left_point = interpolate_ratio(left, ratio)
                right_point = interpolate_ratio(right, ratio)
                if left_point is None or right_point is None:
                    continue

                direction_x, direction_y = line_direction(original, [left_point, right_point])
                s_left = left_point[0] * direction_x + left_point[1] * direction_y
                s_right = right_point[0] * direction_x + right_point[1] * direction_y
                low_point, high_point = (left_point, right_point) if s_left <= s_right else (right_point, left_point)
                old_stopline_ids.add(stopline_id)

                refs = []
                for point in (low_point, high_point):
                    ref = str(id_state["next_node"])
                    id_state["next_node"] += 1
                    refs.append(align.append_node(root, nodes, ref, point, projector))
                old_tags = ways.get(stopline_id, {}).get("tags", {})
                tags = dict(old_tags) if old_tags else {"type": "line_thin", "subtype": "stop_line"}
                tags.setdefault("type", "line_thin")
                tags.setdefault("subtype", "stop_line")
                new_way_id = str(id_state["next_way"])
                id_state["next_way"] += 1
                align.append_way(root, ways, new_way_id, refs, tags)

                for member in regulatory["members"]:
                    if member["type"] == "way" and member["role"] == "ref_line" and member["ref"] == stopline_id:
                        member["ref"] = new_way_id
                for child in regulatory["elem"]:
                    if (
                        child.tag == "member"
                        and child.attrib.get("type") == "way"
                        and child.attrib.get("role") == "ref_line"
                        and child.attrib.get("ref") == stopline_id
                    ):
                        child.attrib["ref"] = new_way_id
                stats["lanelet_stopline_links"] += 1
                stats["created_stoplines"] += 1

    ref_counts = relation_way_reference_counts(relations)
    for stopline_id in sorted(old_stopline_ids, key=lambda value: int(value)):
        way = ways.get(stopline_id)
        if not way or ref_counts[stopline_id] > 0:
            continue
        if way["tags"].get("type") == "line_thin" and way["tags"].get("subtype") == "stop_line":
            if remove_way(root, ways, stopline_id):
                stats["removed_original_stoplines"] += 1

    # Extend after choosing the per-lane direction so exact boundary contact is
    # robust against strict browser-side segment intersection checks.
    for way_id, way in list(ways.items()):
        if way["tags"].get("type") != "line_thin" or way["tags"].get("subtype") != "stop_line":
            continue
        if int(way_id) < id_state["next_way"] - stats["created_stoplines"]:
            continue
        points = way_points(way_id, ways, nodes)
        if len(points) != 2:
            continue
        direction_x, direction_y = line_direction(points, points)
        extension = args.stopline_extension
        low_z = points[0][2] if len(points[0]) > 2 else 0.0
        high_z = points[1][2] if len(points[1]) > 2 else low_z
        low = (points[0][0] - direction_x * extension, points[0][1] - direction_y * extension, low_z)
        high = (points[1][0] + direction_x * extension, points[1][1] + direction_y * extension, high_z)
        align.set_node_position(nodes[way["refs"][0]], low, projector)
        align.set_node_position(nodes[way["refs"][1]], high, projector)

    stats["removed_unused_nodes"] = prune_unused_nodes(root, nodes, ways)
    return stats


def apply_shared_boundary_fixes(root, nodes, ways, relations, args):
    candidates = detect_shared_lane_candidates(nodes, ways, relations, args)
    dsu = DisjointSet()
    for item in candidates:
        if item["source_right_way"] == item["target_left_way"]:
            continue
        dsu.union(item["source_right_way"], item["target_left_way"])

    ref_counts_before = relation_way_reference_counts(relations)
    canonical_by_way = {}
    canonical_style_by_way = {}
    edge_index = getattr(args, "canonical_edge_index", None)
    edge_max_ring = getattr(args, "canonical_edge_max_ring", 8)
    for group in dsu.groups():
        if len(group) < 2:
            continue

        def canonical_key(way_id):
            way = ways.get(way_id, {})
            way_type = way.get("tags", {}).get("type", "")
            type_rank = 0 if is_lane_marking_way(way) else 1 if way_type == "road_border" else 2
            geo_rmse = way_mgeo_edge_rmse(way_id, nodes, ways, edge_index, edge_max_ring)
            return (geo_rmse, type_rank, -ref_counts_before[way_id], int(way_id))

        canonical = min(group, key=canonical_key)
        style_candidates = [way_id for way_id in group if is_lane_marking_way(ways.get(way_id, {}))]
        if style_candidates:
            style_way = min(style_candidates, key=canonical_key)
            canonical_style_by_way[canonical] = ways[style_way]["tags"]
        for way_id in group:
            canonical_by_way[way_id] = canonical

    for way_id, style_tags in canonical_style_by_way.items():
        way = ways.get(way_id)
        if not way:
            continue
        set_tag(way["elem"], way["tags"], "type", "line_thin")
        if style_tags.get("subtype"):
            set_tag(way["elem"], way["tags"], "subtype", style_tags["subtype"])
        else:
            remove_tag(way["elem"], way["tags"], "subtype")

    applied = []
    superseded_way_ids = set()
    for relation in relations:
        if relation["tags"].get("type") != "lanelet":
            continue
        for member in relation["members"]:
            if member["type"] != "way" or member["role"] not in ("left", "right"):
                continue
            canonical = canonical_by_way.get(member["ref"])
            if canonical is None or canonical == member["ref"]:
                continue
            old_ref = member["ref"]
            member["ref"] = canonical
            superseded_way_ids.add(old_ref)
            applied.append(
                {
                    "target_relation_id": relation["id"],
                    "target_left_way": old_ref,
                    "source_right_way": canonical,
                    "source_relation_id": "canonical",
                    "mean_distance": 0.0,
                    "max_distance": 0.0,
                    "center_separation": 0.0,
                }
            )
        for child in relation["elem"]:
            if child.tag != "member" or child.attrib.get("type") != "way" or child.attrib.get("role") not in (
                "left",
                "right",
            ):
                continue
            canonical = canonical_by_way.get(child.attrib.get("ref"))
            if canonical is not None:
                child.attrib["ref"] = canonical

    ref_counts = relation_way_reference_counts(relations)
    removed_ways = 0
    kept_referenced = 0
    for way_id in sorted(superseded_way_ids, key=lambda value: int(value)):
        if ref_counts[way_id] > 0:
            kept_referenced += 1
            continue
        if remove_way(root, ways, way_id):
            removed_ways += 1
    removed_nodes = prune_unused_nodes(root, nodes, ways)
    return {
        "candidate_count": len(candidates),
        "applied_count": len(applied),
        "removed_superseded_ways": removed_ways,
        "kept_still_referenced_ways": kept_referenced,
        "removed_unused_nodes": removed_nodes,
        "applied": applied,
    }


def generate_full_from_v1(args):
    temp_output = re.sub(r"\.osm$", ".full_tmp.osm", args.output)
    if temp_output == args.output:
        temp_output = args.output + ".full_tmp.osm"
    if os.path.exists(temp_output):
        os.remove(temp_output)
    generate_args = argparse.Namespace(
        lanelet=args.source,
        projector=args.projector,
        mgeo_link=args.mgeo_link,
        static_tf=args.static_tf,
        cell_size=args.snap_cell_size,
        output=temp_output,
        force=True,
        match_threshold=args.match_threshold,
        coverage_gap=args.coverage_gap,
        create_missing_min_length=args.create_missing_min_length,
        copy_projector=False,
    )
    align.generate(generate_args)
    return temp_output


def update_meta_version(root, version):
    for elem in root:
        if elem.tag == "MetaInfo":
            elem.attrib["map_version"] = str(version)
            return


def sync_relation_members_from_xml(relations):
    for relation in relations:
        relation["members"] = [
            {"type": child.attrib.get("type"), "ref": child.attrib.get("ref"), "role": child.attrib.get("role", "")}
            for child in relation["elem"]
            if child.tag == "member"
        ]


def relation_member_key(member):
    return (member.get("type"), member.get("ref"), member.get("role", ""))


def relation_ref_counts(relations):
    counts = Counter()
    for relation in relations:
        for member in relation["members"]:
            if member["type"] == "relation":
                counts[member["ref"]] += 1
    return counts


def append_relation_member(relation, member):
    key = relation_member_key(member)
    if any(relation_member_key(existing) == key for existing in relation["members"]):
        return False
    new_member = {"type": member["type"], "ref": member["ref"], "role": member.get("role", "")}
    relation["members"].append(new_member)
    elem = ET.Element("member", {"type": new_member["type"], "ref": new_member["ref"], "role": new_member["role"]})
    insert_index = 0
    for index, child in enumerate(relation["elem"]):
        if child.tag == "tag":
            break
        insert_index = index + 1
    relation["elem"].insert(insert_index, elem)
    return True


def remove_relation_member(relation, member_type, member_ref, member_role):
    before = len(relation["members"])
    relation["members"] = [
        member
        for member in relation["members"]
        if not (
            member["type"] == member_type
            and member["ref"] == member_ref
            and member.get("role", "") == member_role
        )
    ]
    removed = before - len(relation["members"])
    for child in list(relation["elem"]):
        if (
            child.tag == "member"
            and child.attrib.get("type") == member_type
            and child.attrib.get("ref") == member_ref
            and child.attrib.get("role", "") == member_role
        ):
            relation["elem"].remove(child)
    return removed


def remove_relation(root, relations, relation_id):
    for index, relation in enumerate(list(relations)):
        if relation["id"] != relation_id:
            continue
        root.remove(relation["elem"])
        del relations[index]
        return True
    return False


def drop_explicit_lanelet_centerlines(root, nodes, ways, relations):
    stats = Counter()
    removed_way_candidates = set()
    for relation in relations:
        if relation["tags"].get("type") != "lanelet":
            continue
        for member in list(relation["members"]):
            if member["type"] == "way" and member["role"] == "centerline":
                removed_way_candidates.add(member["ref"])
                stats["removed_centerline_members"] += remove_relation_member(
                    relation, "way", member["ref"], "centerline"
                )

    ref_counts = relation_way_reference_counts(relations)
    for way_id in sorted(removed_way_candidates, key=lambda value: int(value)):
        if ref_counts[way_id] > 0:
            stats["kept_referenced_centerline_ways"] += 1
            continue
        if remove_way(root, ways, way_id):
            stats["removed_centerline_ways"] += 1

    stats["removed_unused_nodes"] = prune_unused_nodes(root, nodes, ways)
    sync_relation_members_from_xml(relations)
    return stats


def inventory(nodes, ways, relations):
    counts = Counter()
    for way in ways.values():
        counts[("way_type", way["tags"].get("type", ""), way["tags"].get("subtype", ""))] += 1

    road_lanelets = 0
    lanelets = 0
    road_missing = Counter()
    for relation in relations:
        if relation["tags"].get("type") == "lanelet":
            lanelets += 1
        if relation["tags"].get("type") == "lanelet" and relation["tags"].get("subtype") == "road":
            road_lanelets += 1
            for key in ("speed_limit", "location", "one_way", "participant:vehicle"):
                if not relation["tags"].get(key):
                    road_missing[key] += 1
        if relation["tags"].get("type") == "lanelet" and relation["tags"].get("subtype") == "crosswalk":
            counts[("relation", "crosswalk")] += 1
        if relation["tags"].get("type") == "regulatory_element":
            counts[("relation", "regulatory_element", relation["tags"].get("subtype", ""))] += 1

    used_nodes = {ref for way in ways.values() for ref in way["refs"]}
    used_ways = {member["ref"] for relation in relations for member in relation["members"] if member["type"] == "way"}
    node_without_ele = sum(1 for node in nodes.values() if node["elem"].find("./tag[@k='ele']") is None)
    return {
        "nodes": len(nodes),
        "ways": len(ways),
        "relations": len(relations),
        "road_lanelets": road_lanelets,
        "lanelets": lanelets,
        "crosswalk_lanelets": counts[("relation", "crosswalk")],
        "road_borders": counts[("way_type", "road_border", "")],
        "line_thin": sum(value for key, value in counts.items() if key[:2] == ("way_type", "line_thin")),
        "stop_lines": counts[("way_type", "line_thin", "stop_line")],
        "traffic_lights": counts[("way_type", "traffic_light", "")],
        "traffic_light_regulatory_elements": counts[("relation", "regulatory_element", "traffic_light")],
        "unused_nodes": len(set(nodes) - used_nodes),
        "unused_ways": len(set(ways) - used_ways),
        "node_without_ele": node_without_ele,
        "road_missing": road_missing,
    }


def lanelet_multi_traffic_light_regulatory_stats(relations):
    relation_by_id_map = relation_by_id(relations)
    stats = Counter()
    examples = []
    for relation in relations:
        if relation["tags"].get("type") != "lanelet":
            continue
        traffic_light_regs = []
        for member in relation["members"]:
            if member["type"] != "relation" or member["role"] != "regulatory_element":
                continue
            regulatory = relation_by_id_map.get(member["ref"])
            if regulatory and regulatory["tags"].get("subtype") == "traffic_light":
                traffic_light_regs.append(member["ref"])
        if len(traffic_light_regs) <= 1:
            continue
        stats["lanelets_with_multiple_traffic_light_regs"] += 1
        stats["extra_traffic_light_reg_members"] += len(traffic_light_regs) - 1
        if len(examples) < 8:
            examples.append((relation["id"], len(traffic_light_regs), traffic_light_regs[:]))
    stats["examples"] = examples
    return stats


def traffic_light_multi_stopline_stats(relations):
    stats = Counter()
    examples = []
    for relation in relations:
        if relation["tags"].get("type") != "regulatory_element":
            continue
        if relation["tags"].get("subtype") != "traffic_light":
            continue
        ref_lines = [
            member["ref"]
            for member in relation["members"]
            if member["type"] == "way" and member["role"] == "ref_line"
        ]
        if len(ref_lines) <= 1:
            continue
        stats["traffic_light_regs_with_multiple_stoplines"] += 1
        stats["extra_stopline_members"] += len(ref_lines) - 1
        if len(examples) < 8:
            examples.append((relation["id"], len(ref_lines), ref_lines[:]))
    stats["examples"] = examples
    return stats


def integrity(nodes, ways, relations):
    stats = Counter()
    for way in ways.values():
        if not way["refs"]:
            stats["empty_ways"] += 1
        for ref in way["refs"]:
            if ref not in nodes:
                stats["missing_way_node_refs"] += 1
    relation_ids = {relation["id"] for relation in relations}
    for relation in relations:
        role_counts = Counter()
        for member in relation["members"]:
            if member["type"] == "way":
                if member["ref"] not in ways:
                    stats["missing_relation_way_refs"] += 1
                role_counts[member["role"]] += 1
            elif member["type"] == "relation" and member["ref"] not in relation_ids:
                stats["missing_relation_relation_refs"] += 1
        if relation["tags"].get("type") == "lanelet":
            for role in ("left", "right"):
                if role_counts[role] == 0:
                    stats["lanelets_missing_required_boundary_roles"] += 1
                elif role_counts[role] > 1:
                    stats["lanelets_duplicate_required_boundary_roles"] += role_counts[role] - 1
            if role_counts["centerline"] > 1:
                stats["lanelets_duplicate_centerline_roles"] += role_counts["centerline"] - 1
    return stats


def merge_lanelet_traffic_light_regulatory_elements(root, nodes, ways, relations):
    stats = Counter()
    relations_by_id = relation_by_id(relations)
    extra_stopline_way_ids = set()

    def stopline_progress(regulatory, lanelet_center):
        if len(lanelet_center) < 2:
            return float("-inf")
        best = float("-inf")
        for member in regulatory["members"]:
            if member["type"] != "way" or member["role"] != "ref_line":
                continue
            stopline_mid = midpoint(way_points(member["ref"], ways, nodes))
            if stopline_mid is None:
                continue
            projection = align.project_point_to_polyline(stopline_mid, lanelet_center)
            if projection is None:
                continue
            best = max(best, projection["s"])
        return best

    def choose_keep_ref(lanelet, traffic_light_refs):
        center = way_points(relation_way_id(lanelet, "centerline"), ways, nodes)
        scores = []
        for regulatory_ref in traffic_light_refs:
            regulatory = relations_by_id.get(regulatory_ref)
            progress = stopline_progress(regulatory, center) if regulatory else float("-inf")
            scores.append((-progress, int(regulatory_ref), regulatory_ref))
        scores.sort()
        return scores[0][2]

    for lanelet in relations:
        if lanelet["tags"].get("type") != "lanelet":
            continue
        traffic_light_refs = []
        for member in lanelet["members"]:
            if member["type"] != "relation" or member["role"] != "regulatory_element":
                continue
            regulatory = relations_by_id.get(member["ref"])
            if regulatory and regulatory["tags"].get("subtype") == "traffic_light":
                traffic_light_refs.append(member["ref"])
        if len(traffic_light_refs) <= 1:
            continue

        keep_ref = choose_keep_ref(lanelet, traffic_light_refs)
        keep_reg = relations_by_id[keep_ref]
        keep_has_ref_line = any(
            member["type"] == "way" and member["role"] == "ref_line" for member in keep_reg["members"]
        )
        for remove_ref in traffic_light_refs:
            if remove_ref == keep_ref:
                continue
            remove_reg = relations_by_id.get(remove_ref)
            if remove_reg is None:
                continue
            for member in remove_reg["members"]:
                if member["type"] == "way" and member["role"] == "ref_line":
                    if keep_has_ref_line:
                        extra_stopline_way_ids.add(member["ref"])
                        stats["skipped_extra_stopline_members"] += 1
                        continue
                    if append_relation_member(keep_reg, member):
                        keep_has_ref_line = True
                        stats["merged_members"] += 1
                    continue
                if append_relation_member(keep_reg, member):
                    stats["merged_members"] += 1
            stats["removed_lanelet_reg_members"] += remove_relation_member(
                lanelet, "relation", remove_ref, "regulatory_element"
            )
        stats["lanelets_merged"] += 1

    for relation in relations:
        if relation["tags"].get("type") != "regulatory_element":
            continue
        if relation["tags"].get("subtype") != "traffic_light":
            continue
        ref_lines = [
            member["ref"]
            for member in relation["members"]
            if member["type"] == "way" and member["role"] == "ref_line"
        ]
        for stopline_id in ref_lines[1:]:
            extra_stopline_way_ids.add(stopline_id)
            stats["trimmed_extra_stopline_members"] += remove_relation_member(
                relation, "way", stopline_id, "ref_line"
            )

    counts = relation_ref_counts(relations)
    for relation in list(relations):
        if relation["tags"].get("type") != "regulatory_element":
            continue
        if relation["tags"].get("subtype") != "traffic_light":
            continue
        if counts[relation["id"]] > 0:
            continue
        if remove_relation(root, relations, relation["id"]):
            stats["removed_unreferenced_regs"] += 1

    sync_relation_members_from_xml(relations)
    way_ref_counts = relation_way_reference_counts(relations)
    for way_id in sorted(extra_stopline_way_ids, key=lambda value: int(value)):
        way = ways.get(way_id)
        if not way or way_ref_counts[way_id] > 0:
            continue
        if way["tags"].get("type") == "line_thin" and way["tags"].get("subtype") == "stop_line":
            if remove_way(root, ways, way_id):
                stats["removed_extra_stopline_ways"] += 1
    stats["removed_extra_stopline_nodes"] = prune_unused_nodes(root, nodes, ways)

    return stats


def build(args):
    if os.path.exists(args.output) and not args.force:
        raise RuntimeError("%s already exists; pass --force to overwrite" % args.output)

    projector = align.load_projector(args.projector)
    args.projector_data = projector
    tf_cfg = align.read_static_tf(args.static_tf)
    links = align.load_mgeo_links(args.mgeo_link, tf_cfg)
    edge_segments = []
    for link in links:
        edge_segments.extend(align.mgeo_edge_segments(link, "left"))
        edge_segments.extend(align.mgeo_edge_segments(link, "right"))
    args.canonical_edge_index = align.SegmentIndex(edge_segments, args.snap_cell_size)
    args.canonical_edge_max_ring = max(
        2, int(math.ceil(max(args.canonical_match_threshold, args.snap_cell_size) / args.snap_cell_size)) + 2
    )

    temp_output = None
    parse_source = args.source
    if args.mode == "full":
        temp_output = generate_full_from_v1(args)
        parse_source = temp_output

    tree, root, nodes, ways, relations = align.parse_lanelet(parse_source, projector)

    snap_stats = Counter()
    if args.mode == "snap" and not args.no_snap:
        snap_stats = snap_existing_nodes_to_mgeo(nodes, ways, relations, links, args)

    tag_stats = normalize_road_lanelet_tags(nodes, ways, relations, links, args)
    shared_passes = []
    for _ in range(args.max_shared_iterations):
        shared_stats = apply_shared_boundary_fixes(root, nodes, ways, relations, args)
        sync_relation_members_from_xml(relations)
        shared_passes.append(shared_stats)
        if shared_stats["applied_count"] == 0:
            break
    sync_relation_members_from_xml(relations)
    stopline_stats = align_stop_lines_to_referrer_lanelets(root, nodes, ways, relations, projector, args)
    traffic_light_merge_stats = merge_lanelet_traffic_light_regulatory_elements(root, nodes, ways, relations)
    sync_relation_members_from_xml(relations)
    steep_angle_stats = remove_steep_lanelet_angles(root, nodes, ways, relations, args.min_lanelet_angle)
    centerline_stats = Counter()
    centerline_drop_stats = Counter()
    if args.keep_explicit_centerlines:
        centerline_stats = rebuild_lanelet_centerlines_from_boundaries(root, nodes, ways, relations, projector, args)
    else:
        centerline_drop_stats = drop_explicit_lanelet_centerlines(root, nodes, ways, relations)
    short_segment_stats = remove_short_adjacent_way_points(root, nodes, ways, args.min_adjacent_point_distance)
    final_shared_passes = []
    for _ in range(args.max_shared_iterations):
        final_shared_stats = apply_shared_boundary_fixes(root, nodes, ways, relations, args)
        sync_relation_members_from_xml(relations)
        final_shared_passes.append(final_shared_stats)
        if final_shared_stats["applied_count"] == 0:
            break
    final_centerline_stats = Counter()
    final_short_segment_stats = Counter()
    if args.keep_explicit_centerlines and sum(item["applied_count"] for item in final_shared_passes) > 0:
        final_centerline_stats = rebuild_lanelet_centerlines_from_boundaries(root, nodes, ways, relations, projector, args)
        final_short_segment_stats = remove_short_adjacent_way_points(root, nodes, ways, args.min_adjacent_point_distance)
    elevation_stats = {}
    if not args.preserve_ele:
        elevation_stats = flatten_node_elevations(nodes, projector, args.flat_ele_value)
    if args.version is not None:
        update_meta_version(root, args.version)
    align.indent_xml(root)
    tree.write(args.output, encoding="UTF-8", xml_declaration=True)
    if temp_output and os.path.exists(temp_output):
        os.remove(temp_output)

    if args.copy_projector:
        projector_out = re.sub(r"_v\d+\.yaml$", "_v%d.yaml" % args.version, args.projector)
        if projector_out != args.projector:
            shutil.copy2(args.projector, projector_out)

    print("source=%s" % args.source)
    print("mode=%s" % args.mode)
    print("wrote=%s" % args.output)
    if snap_stats:
        print(
            "snap updated_nodes=%d conflict_nodes=%d mean_distance=%.6f max_distance=%.6f "
            "center_target=%d/%d boundary_target=%d/%d"
            % (
                snap_stats["updated_nodes"],
                snap_stats["conflict_nodes"],
                snap_stats["mean_snap_distance"],
                snap_stats["max_snap_distance"],
                snap_stats["center_target_nodes"],
                snap_stats["center_candidate_nodes"],
                snap_stats["boundary_target_nodes"],
                snap_stats["boundary_candidate_nodes"],
            )
        )
    print(
        "tags road_lanelets=%d speed_from_mgeo=%d speed_changed=%d"
        % (tag_stats["road_lanelets"], tag_stats["speed_from_mgeo"], tag_stats["speed_changed"])
    )
    total_applied = sum(item["applied_count"] for item in shared_passes)
    total_removed_ways = sum(item["removed_superseded_ways"] for item in shared_passes)
    total_removed_nodes = sum(item["removed_unused_nodes"] for item in shared_passes)
    total_kept = sum(item["kept_still_referenced_ways"] for item in shared_passes)
    print(
        "shared passes=%d first_candidates=%d total_applied=%d removed_superseded_ways=%d "
        "removed_unused_nodes=%d kept_still_referenced=%d"
        % (
            len(shared_passes),
            shared_passes[0]["candidate_count"] if shared_passes else 0,
            total_applied,
            total_removed_ways,
            total_removed_nodes,
            total_kept,
        )
    )
    print(
        "stoplines created=%d removed_original=%d lanelet_links=%d removed_unused_nodes=%d"
        % (
            stopline_stats["created_stoplines"],
            stopline_stats["removed_original_stoplines"],
            stopline_stats["lanelet_stopline_links"],
            stopline_stats["removed_unused_nodes"],
        )
    )
    print(
        "short_segments threshold=%.3fm changed_ways=%d removed_way_points=%d removed_unused_nodes=%d "
        "unremovable_short_pairs=%d"
        % (
            args.min_adjacent_point_distance,
            short_segment_stats["changed_ways"],
            short_segment_stats["removed_way_points"],
            short_segment_stats["removed_unused_nodes"],
            short_segment_stats["unremovable_short_pairs"],
        )
    )
    print(
        "traffic_light_reg_merge lanelets=%d removed_lanelet_members=%d merged_members=%d "
        "skipped_extra_stoplines=%d trimmed_extra_stoplines=%d removed_extra_stopline_ways=%d "
        "removed_extra_stopline_nodes=%d removed_regs=%d"
        % (
            traffic_light_merge_stats["lanelets_merged"],
            traffic_light_merge_stats["removed_lanelet_reg_members"],
            traffic_light_merge_stats["merged_members"],
            traffic_light_merge_stats["skipped_extra_stopline_members"],
            traffic_light_merge_stats["trimmed_extra_stopline_members"],
            traffic_light_merge_stats["removed_extra_stopline_ways"],
            traffic_light_merge_stats["removed_extra_stopline_nodes"],
            traffic_light_merge_stats["removed_unreferenced_regs"],
        )
    )
    print(
        "steep_angles threshold=%.1fdeg changed_ways=%d removed_points=%d removed_unused_nodes=%d"
        % (
            args.min_lanelet_angle,
            steep_angle_stats["changed_ways"],
            steep_angle_stats["removed_steep_angle_points"],
            steep_angle_stats["removed_unused_nodes"],
        )
    )
    print(
        "centerline_rebuild replaced_ways=%d created_ways=%d created_nodes=%d removed_unused_nodes=%d skipped=%d"
        % (
            centerline_stats["replaced_centerline_ways"],
            centerline_stats["created_centerline_ways"],
            centerline_stats["created_centerline_nodes"],
            centerline_stats["removed_unused_nodes"],
            centerline_stats["skipped"],
        )
    )
    print(
        "centerline_drop removed_members=%d removed_ways=%d removed_unused_nodes=%d kept_referenced_ways=%d"
        % (
            centerline_drop_stats["removed_centerline_members"],
            centerline_drop_stats["removed_centerline_ways"],
            centerline_drop_stats["removed_unused_nodes"],
            centerline_drop_stats["kept_referenced_centerline_ways"],
        )
    )
    print(
        "final_shared passes=%d total_applied=%d removed_superseded_ways=%d removed_unused_nodes=%d"
        % (
            len(final_shared_passes),
            sum(item["applied_count"] for item in final_shared_passes),
            sum(item["removed_superseded_ways"] for item in final_shared_passes),
            sum(item["removed_unused_nodes"] for item in final_shared_passes),
        )
    )
    if final_centerline_stats:
        print(
            "final_centerline_rebuild replaced_ways=%d created_ways=%d created_nodes=%d removed_unused_nodes=%d skipped=%d"
            % (
                final_centerline_stats["replaced_centerline_ways"],
                final_centerline_stats["created_centerline_ways"],
                final_centerline_stats["created_centerline_nodes"],
                final_centerline_stats["removed_unused_nodes"],
                final_centerline_stats["skipped"],
            )
        )
        print(
            "final_short_segments threshold=%.3fm changed_ways=%d removed_way_points=%d removed_unused_nodes=%d "
            "unremovable_short_pairs=%d"
            % (
                args.min_adjacent_point_distance,
                final_short_segment_stats["changed_ways"],
                final_short_segment_stats["removed_way_points"],
                final_short_segment_stats["removed_unused_nodes"],
                final_short_segment_stats["unremovable_short_pairs"],
            )
        )
    if elevation_stats:
        print(
            "elevation_flatten nodes=%d target_z=%.3f before_min=%.3f before_max=%.3f before_mean=%.3f"
            % (
                elevation_stats["nodes"],
                elevation_stats["target_z"],
                elevation_stats["min_before"],
                elevation_stats["max_before"],
                elevation_stats["mean_before"],
            )
        )
    applied_examples = []
    for shared_stats in shared_passes:
        applied_examples.extend(shared_stats["applied"])
    for item in applied_examples[: args.print_limit]:
        print(
            "  shared rel=%s left:%s->%s neighbor_rel=%s mean=%.3f max=%.3f sep=%.2f"
            % (
                item["target_relation_id"],
                item["target_left_way"],
                item["source_right_way"],
                item["source_relation_id"],
                item["mean_distance"],
                item["max_distance"],
                item["center_separation"],
            )
        )


def validate(args):
    projector = align.load_projector(args.projector)
    _, _, nodes, ways, relations = align.parse_lanelet(args.map_file, projector)
    inv = inventory(nodes, ways, relations)
    stats = integrity(nodes, ways, relations)
    candidates = detect_shared_lane_candidates(nodes, ways, relations, args)
    adjacent_stats = adjacent_segment_stats(nodes, ways, args.min_adjacent_point_distance)
    z_step_stats = linestring_z_step_stats(nodes, ways, args.max_linestring_z_step)
    steep_angle_stats = steep_lanelet_angle_stats(nodes, ways, relations, args.min_lanelet_angle)
    traffic_light_multi_stats = lanelet_multi_traffic_light_regulatory_stats(relations)
    traffic_light_stopline_stats = traffic_light_multi_stopline_stats(relations)
    remaining = [item for item in candidates if item["source_right_way"] != item["target_left_way"]]
    role_counts = Counter((item["target_relation_id"], item["target_role"]) for item in remaining)
    actionable = [item for item in remaining if role_counts[(item["target_relation_id"], item["target_role"])] == 1]

    print("map=%s" % args.map_file)
    print(
        "inventory nodes={nodes} ways={ways} relations={relations} road_lanelets={road_lanelets} "
        "crosswalk_lanelets={crosswalk_lanelets} road_borders={road_borders} line_thin={line_thin} "
        "stop_lines={stop_lines} traffic_lights={traffic_lights} traffic_light_regulatory_elements={traffic_light_regulatory_elements} "
        "unused_nodes={unused_nodes} unused_ways={unused_ways} node_without_ele={node_without_ele}".format(**inv)
    )
    print(
        "vmb_required_missing speed_limit=%d location=%d one_way=%d participant_vehicle=%d"
        % (
            inv["road_missing"]["speed_limit"],
            inv["road_missing"]["location"],
            inv["road_missing"]["one_way"],
            inv["road_missing"]["participant:vehicle"],
        )
    )
    print(
        "integrity missing_way_node_refs=%d missing_relation_way_refs=%d missing_relation_relation_refs=%d "
        "empty_ways=%d lanelets_missing_required_boundary_roles=%d duplicate_boundary_roles=%d duplicate_centerline_roles=%d"
        % (
            stats["missing_way_node_refs"],
            stats["missing_relation_way_refs"],
            stats["missing_relation_relation_refs"],
            stats["empty_ways"],
            stats["lanelets_missing_required_boundary_roles"],
            stats["lanelets_duplicate_required_boundary_roles"],
            stats["lanelets_duplicate_centerline_roles"],
        )
    )
    print(
        "shared_lane_candidates=%d remaining_unshared=%d actionable_remaining=%d conflict_targets=%d"
        % (len(candidates), len(remaining), len(actionable), sum(1 for count in role_counts.values() if count > 1))
    )
    print(
        "adjacent_segments min_distance=%dmm short_pairs_lt_%.3fm=%d unremovable=%d"
        % (
            adjacent_stats["min_distance_mm"],
            args.min_adjacent_point_distance,
            adjacent_stats["short_adjacent_pairs"],
            adjacent_stats["short_unremovable_pairs"],
        )
    )
    for distance, way_id, first_ref, second_ref, way_type in adjacent_stats["examples"][: args.print_limit]:
        print(
            "  short_adjacent way=%s %s-%s type=%s distance=%.6fm"
            % (way_id, first_ref, second_ref, way_type, distance)
        )
    print(
        "linestring_z_steps threshold=%.3fm max_step=%dmm over_threshold=%d"
        % (args.max_linestring_z_step, z_step_stats["max_step_mm"], z_step_stats["over_threshold"])
    )
    for dz, way_id, first_ref, second_ref, way_type, way_subtype in z_step_stats["examples"][: args.print_limit]:
        print(
            "  z_step way=%s %s-%s type=%s/%s dz=%.6fm"
            % (way_id, first_ref, second_ref, way_type, way_subtype, dz)
        )
    print(
        "traffic_light_regulatory lanelets_with_multiple=%d extra_members=%d"
        % (
            traffic_light_multi_stats["lanelets_with_multiple_traffic_light_regs"],
            traffic_light_multi_stats["extra_traffic_light_reg_members"],
        )
    )
    for lanelet_id, count, refs in traffic_light_multi_stats["examples"][: args.print_limit]:
        print("  multi_traffic_light_reg lanelet=%s count=%d refs=%s" % (lanelet_id, count, ",".join(refs)))
    print(
        "traffic_light_stoplines regs_with_multiple=%d extra_members=%d"
        % (
            traffic_light_stopline_stats["traffic_light_regs_with_multiple_stoplines"],
            traffic_light_stopline_stats["extra_stopline_members"],
        )
    )
    for relation_id, count, refs in traffic_light_stopline_stats["examples"][: args.print_limit]:
        print("  multi_stopline_reg relation=%s count=%d refs=%s" % (relation_id, count, ",".join(refs)))
    print(
        "steep_lanelet_angles threshold=%.1fdeg points=%d"
        % (args.min_lanelet_angle, steep_angle_stats["steep_angle_points"])
    )
    for angle, way_id, prev_ref, ref, next_ref in steep_angle_stats["examples"][: args.print_limit]:
        print(
            "  steep_angle way=%s %s-%s-%s angle=%.3fdeg"
            % (way_id, prev_ref, ref, next_ref, angle)
        )
    for item in remaining[: args.print_limit]:
        print(
            "  remaining rel=%s left=%s neighbor_rel=%s right=%s mean=%.3f max=%.3f"
            % (
                item["target_relation_id"],
                item["target_left_way"],
                item["source_relation_id"],
                item["source_right_way"],
                item["mean_distance"],
                item["max_distance"],
            )
        )

    failures = []
    for key in (
        "missing_way_node_refs",
        "missing_relation_way_refs",
        "missing_relation_relation_refs",
        "empty_ways",
        "lanelets_missing_required_boundary_roles",
        "lanelets_duplicate_required_boundary_roles",
        "lanelets_duplicate_centerline_roles",
    ):
        if stats[key]:
            failures.append("%s=%d" % (key, stats[key]))
    if inv["unused_nodes"]:
        failures.append("unused_nodes=%d" % inv["unused_nodes"])
    if inv["node_without_ele"]:
        failures.append("node_without_ele=%d" % inv["node_without_ele"])
    for key, value in inv["road_missing"].items():
        if value:
            failures.append("road_missing_%s=%d" % (key.replace(":", "_"), value))
    if remaining:
        failures.append("remaining_unshared=%d" % len(remaining))
    if adjacent_stats["short_adjacent_pairs"]:
        failures.append(
            "short_adjacent_pairs_lt_%.3fm=%d"
            % (args.min_adjacent_point_distance, adjacent_stats["short_adjacent_pairs"])
        )
    if z_step_stats["over_threshold"]:
        failures.append(
            "linestring_z_steps_gt_%.3fm=%d" % (args.max_linestring_z_step, z_step_stats["over_threshold"])
        )
    if traffic_light_multi_stats["lanelets_with_multiple_traffic_light_regs"]:
        failures.append(
            "lanelets_with_multiple_traffic_light_regs=%d"
            % traffic_light_multi_stats["lanelets_with_multiple_traffic_light_regs"]
        )
    if traffic_light_stopline_stats["traffic_light_regs_with_multiple_stoplines"]:
        failures.append(
            "traffic_light_regs_with_multiple_stoplines=%d"
            % traffic_light_stopline_stats["traffic_light_regs_with_multiple_stoplines"]
        )
    if steep_angle_stats["steep_angle_points"]:
        failures.append("steep_lanelet_angle_points=%d" % steep_angle_stats["steep_angle_points"])
    if failures:
        raise RuntimeError("validation failed: " + ", ".join(failures))
    print("validation: PASS")


def add_candidate_args(parser):
    parser.add_argument("--grid-cell-size", type=float, default=8.0)
    parser.add_argument("--min-direction-dot", type=float, default=0.95)
    parser.add_argument("--min-center-separation", type=float, default=2.0)
    parser.add_argument("--max-center-separation", type=float, default=5.5)
    parser.add_argument("--max-mean-distance", type=float, default=0.08)
    parser.add_argument("--max-max-distance", type=float, default=0.20)
    parser.add_argument("--min-coverage", type=float, default=0.80)
    parser.add_argument("--print-limit", type=int, default=20)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--projector", default=DEFAULT_PROJECTOR)
    subparsers = parser.add_subparsers(dest="command", required=True)

    build_parser = subparsers.add_parser("build")
    build_parser.add_argument("--source", default=DEFAULT_SOURCE)
    build_parser.add_argument("--output", default=DEFAULT_OUTPUT)
    build_parser.add_argument("--mgeo-link", default=DEFAULT_MGEO_LINK)
    build_parser.add_argument("--static-tf", default=DEFAULT_STATIC_TF)
    build_parser.add_argument("--version", type=int, default=23)
    build_parser.add_argument("--force", action="store_true")
    build_parser.add_argument("--copy-projector", action="store_true")
    build_parser.add_argument("--mode", choices=("full", "snap"), default="full")
    build_parser.add_argument("--no-snap", action="store_true")
    build_parser.add_argument("--snap-threshold", type=float, default=2.0)
    build_parser.add_argument("--snap-conflict-tolerance", type=float, default=0.2)
    build_parser.add_argument("--snap-cell-size", type=float, default=5.0)
    build_parser.add_argument("--match-threshold", type=float, default=4.0)
    build_parser.add_argument("--coverage-gap", type=float, default=1.0)
    build_parser.add_argument("--create-missing-min-length", type=float, default=2.0)
    build_parser.add_argument("--speed-match-threshold", type=float, default=4.0)
    build_parser.add_argument("--canonical-match-threshold", type=float, default=4.0)
    build_parser.add_argument("--stopline-extension", type=float, default=0.25)
    build_parser.add_argument("--stopline-lane-margin", type=float, default=0.5)
    build_parser.add_argument("--centerline-max-spacing", type=float, default=2.0)
    build_parser.add_argument("--keep-explicit-centerlines", action="store_true")
    build_parser.add_argument("--min-lanelet-angle", type=float, default=45.0)
    build_parser.add_argument("--min-adjacent-point-distance", type=float, default=0.2)
    build_parser.add_argument("--max-linestring-z-step", type=float, default=0.2)
    build_parser.add_argument("--preserve-ele", action="store_true")
    build_parser.add_argument("--flat-ele-value", type=float)
    build_parser.add_argument("--max-shared-iterations", type=int, default=8)
    add_candidate_args(build_parser)

    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("--map-file", default=DEFAULT_OUTPUT)
    validate_parser.add_argument("--min-adjacent-point-distance", type=float, default=0.2)
    validate_parser.add_argument("--min-lanelet-angle", type=float, default=45.0)
    validate_parser.add_argument("--max-linestring-z-step", type=float, default=0.2)
    add_candidate_args(validate_parser)

    args = parser.parse_args()
    if args.command == "build":
        build(args)
    elif args.command == "validate":
        validate(args)


if __name__ == "__main__":
    main()
