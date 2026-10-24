#!/usr/bin/env python3
"""Render v78/v79 bottleneck geometry and emitted route-lane comparison."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch
from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/kcity_v79_bottleneck_comparison.png"
REMOVED = {100690, 101993, 104167, 104201, 105436, 105476, 105478, 105480, 105482}
V78_ROUTE = {201473, 201474, 201475, 201439, 201441, 201446, 200360, 201476, 201477}
V79_ROUTE = {201473, 201474, 201475, 201439, 200360, 201477}


def load(version):
    root = etree.parse(
        str(ROOT / f"lanelet_map/lanelet2_map_v{version}.osm"),
        etree.XMLParser(huge_tree=True),
    ).getroot()
    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    forward = Transformer.from_crs(4326, 32652, always_xy=True)
    origin_x, origin_y = forward.transform(126.7730819386, 37.2380038082)

    def point(node_id):
        node = nodes[node_id]
        x, y = forward.transform(float(node.get("lon")), float(node.get("lat")))
        return x - origin_x, y - origin_y

    def way_points(way_id):
        return [point(int(item.get("ref"))) for item in ways[way_id].findall("nd")]

    def polygon(lanelet_id):
        members = {
            item.get("role"): int(item.get("ref"))
            for item in relations[lanelet_id].findall("member")
            if item.get("type") == "way"
        }
        return way_points(members["left"]) + list(reversed(way_points(members["right"])))

    return ways, way_points, polygon


def setup(axis, title):
    axis.set_title(title)
    axis.set_xlim(196, 214)
    axis.set_ylim(285, 405)
    axis.set_aspect("equal")
    axis.grid(alpha=0.15)


def main():
    figure, axes = plt.subplots(1, 4, figsize=(14, 9), constrained_layout=True)
    for column, version in enumerate((78, 79)):
        ways, way_points, _ = load(version)
        setup(axes[column], f"v{version} raw geometry")
        for way_id in sorted(ways):
            points = way_points(way_id)
            if not any(196 <= x <= 214 and 285 <= y <= 405 for x, y in points):
                continue
            color = "#d62728" if version == 78 and way_id in REMOVED else "#707070"
            width = 2.2 if color == "#d62728" else 0.7
            axes[column].plot(*zip(*points), color=color, linewidth=width)

    for column, (version, route_ids) in enumerate(((78, V78_ROUTE), (79, V79_ROUTE)), start=2):
        _, _, polygon = load(version)
        setup(axes[column], f"v{version} emitted route lanes")
        for lanelet_id in sorted(route_ids):
            patch = Patch(polygon(lanelet_id), closed=True, facecolor="#4daf4a", alpha=0.32,
                          edgecolor="white", linewidth=1.1)
            axes[column].add_patch(patch)
            points = polygon(lanelet_id)
            x = sum(item[0] for item in points) / len(points)
            y = sum(item[1] for item in points) / len(points)
            axes[column].text(x, y, str(lanelet_id), fontsize=7, ha="center")

    figure.suptitle("K-City highway bottleneck cleanup: v78 to v79")
    figure.savefig(OUTPUT, dpi=180)
    print(OUTPUT)


if __name__ == "__main__":
    main()
