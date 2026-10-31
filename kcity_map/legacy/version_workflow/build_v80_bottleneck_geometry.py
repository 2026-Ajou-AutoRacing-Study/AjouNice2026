#!/usr/bin/env python3
"""Build v80 by straightening the final highway bottleneck transition."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "lanelet_map/lanelet2_map_v79.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v80.osm"
REPORT = ROOT / "docs/kcity_v80_bottleneck_geometry_build.json"

TARGET_LANELETS = (200360, 201453, 200185, 201476, 201477)
STRAIGHT_WAYS = (
    105603, 105605, 105607,  # 200360
    105597, 105599, 105601,  # 201453
    105456, 105449, 105489,  # 200185
    105598, 105618, 105602,  # 201476 and shared separator
    105606, 105608,          # 201477
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    if OUTPUT.exists():
        raise RuntimeError(f"released output already exists: {OUTPUT}")

    tree = etree.parse(str(SOURCE), etree.XMLParser(huge_tree=True))
    root = tree.getroot()
    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}

    forward = Transformer.from_crs(4326, 32652, always_xy=True)
    inverse = Transformer.from_crs(32652, 4326, always_xy=True)
    origin_x, origin_y = forward.transform(126.7730819386, 37.2380038082)

    def tags(element):
        return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}

    def point(node_id: int) -> np.ndarray:
        element = nodes[node_id]
        x, y = forward.transform(float(element.get("lon")), float(element.get("lat")))
        return np.array([
            x - origin_x,
            y - origin_y,
            float(tags(element).get("ele", 0.0)),
        ])

    changed_nodes = {}

    def put(node_id: int, value: np.ndarray) -> None:
        before = point(node_id)
        longitude, latitude = inverse.transform(value[0] + origin_x, value[1] + origin_y)
        element = nodes[node_id]
        element.set("lon", f"{longitude:.14f}")
        element.set("lat", f"{latitude:.14f}")
        elevation = next(
            (tag for tag in element.findall("tag") if tag.get("k") == "ele"), None
        )
        if elevation is None:
            elevation = etree.SubElement(element, "tag", k="ele")
        elevation.set("v", f"{value[2]:.9f}")
        changed_nodes[str(node_id)] = {
            "before": before.tolist(),
            "after": value.tolist(),
            "shift_m": float(np.linalg.norm(value[:2] - before[:2])),
        }

    def refs(way_id: int) -> list[int]:
        return [int(item.get("ref")) for item in ways[way_id].findall("nd")]

    changed_ways = {}
    removed_point_candidates = set()

    def set_refs(way_id: int, node_ids: list[int]) -> None:
        before = refs(way_id)
        removed_point_candidates.update(before[1:-1])
        for item in list(ways[way_id].findall("nd")):
            ways[way_id].remove(item)
        for node_id in reversed(node_ids):
            ways[way_id].insert(0, etree.Element("nd", ref=str(node_id)))
        changed_ways[str(way_id)] = {"before": before, "after": node_ids}

    # 105449 -> 105599 used to bend west and then return east to the 105603
    # terminal point. Put their joint on the straight chord instead.
    start = point(228079)
    finish = point(232984)
    old_joint = point(228463)
    ratio = (old_joint[1] - start[1]) / (finish[1] - start[1])
    put(228463, start + ratio * (finish - start))

    # The 200360 -> 201477 entrance was only 2.87 m wide. Preserve its
    # direction and make the cross-section a regular 3.20 m vehicle lane.
    middle = point(232984)
    outer = point(232987)
    direction = outer - middle
    direction[2] = 0.0
    direction /= np.linalg.norm(direction)
    widened = middle + 3.20 * direction
    widened[2] = outer[2]
    put(232987, widened)

    # Put each target centerline exactly halfway between its boundary joint.
    center_joints = (
        (229565, 228096, 228079),
        (229618, 228523, 228463),
        (232985, 232983, 232984),
        (232988, 232984, 232987),
        (229636, 228526, 228462),
        (229334, 228462, 228477),
    )
    for center, left, right in center_joints:
        put(center, (point(left) + point(right)) / 2.0)

    # These lanelets are straight transition pieces. Retain their existing
    # endpoint IDs and way IDs, but remove the unnecessary intermediate shape
    # points that produced wavy and multi-corner polygons in PlanViz/VMB.
    for way_id in STRAIGHT_WAYS:
        node_ids = refs(way_id)
        set_refs(way_id, [node_ids[0], node_ids[-1]])

    used_nodes = {
        int(item.get("ref"))
        for way in root.findall("way")
        for item in way.findall("nd")
    }
    used_nodes.update(
        int(member.get("ref"))
        for relation in root.findall("relation")
        for member in relation.findall("member")
        if member.get("type") == "node"
    )
    removed_orphan_nodes = sorted(removed_point_candidates - used_nodes)
    for node_id in removed_orphan_nodes:
        root.remove(nodes[node_id])

    root.find("MetaInfo").set("map_version", "80")
    root[:] = sorted(
        root,
        key=lambda item: {"MetaInfo": 0, "node": 1, "way": 2, "relation": 3}.get(
            item.tag, 0
        ),
    )
    etree.indent(tree, space="  ")
    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True)

    report = {
        "input": str(SOURCE.relative_to(ROOT)),
        "input_sha256": sha256(SOURCE),
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": sha256(OUTPUT),
        "target_lanelets": list(TARGET_LANELETS),
        "straightened_ways": list(STRAIGHT_WAYS),
        "changed_nodes": changed_nodes,
        "changed_ways": changed_ways,
        "removed_orphan_nodes": removed_orphan_nodes,
        "lanelets_deleted": [],
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
