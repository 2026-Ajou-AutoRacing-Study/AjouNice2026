#!/usr/bin/env python3
"""Render the focused v79/v80 bottleneck geometry comparison."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/kcity_v80_bottleneck_geometry_comparison.png"
LANES = (
    (200185, "#2ca9df"),
    (200360, "#f2c40e"),
    (201453, "#75b843"),
    (201476, "#d94d2b"),
    (201477, "#ef7d22"),
)


def draw(axis, version):
    path = ROOT / f"lanelet_map/lanelet2_map_v{version}.osm"
    root = etree.parse(str(path), etree.XMLParser(huge_tree=True)).getroot()
    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    forward = Transformer.from_crs(4326, 32652, always_xy=True)
    origin_x, origin_y = forward.transform(126.7730819386, 37.2380038082)

    def xy(node_id):
        node = nodes[node_id]
        x, y = forward.transform(float(node.get("lon")), float(node.get("lat")))
        return x - origin_x, y - origin_y

    def boundary(lanelet_id, role):
        relation = relations[lanelet_id]
        way_id = int(next(
            member.get("ref") for member in relation.findall("member")
            if member.get("type") == "way" and member.get("role") == role
        ))
        return [xy(int(item.get("ref"))) for item in ways[way_id].findall("nd")]

    for lanelet_id, color in LANES:
        left = boundary(lanelet_id, "left")
        right = boundary(lanelet_id, "right")
        polygon = left + list(reversed(right))
        axis.add_patch(Polygon(
            polygon, closed=True, facecolor=color, edgecolor=color, alpha=0.30, linewidth=2
        ))
        axis.plot(*zip(*left), color=color, linewidth=1.3)
        axis.plot(*zip(*right), color=color, linewidth=1.3)
        center_x = sum(point[0] for point in polygon) / len(polygon)
        center_y = sum(point[1] for point in polygon) / len(polygon)
        axis.text(center_x, center_y, str(lanelet_id), fontsize=8, ha="center")

    axis.set_xlim(198, 214)
    axis.set_ylim(288, 332)
    axis.set_aspect("equal")
    axis.grid(alpha=0.25)
    axis.set_title(f"v{version}")


def main():
    figure, axes = plt.subplots(1, 2, figsize=(9, 10), dpi=180)
    draw(axes[0], 79)
    draw(axes[1], 80)
    figure.suptitle("K-City final highway bottleneck geometry: v79 to v80")
    figure.tight_layout()
    figure.savefig(OUTPUT, bbox_inches="tight")
    print(OUTPUT)


if __name__ == "__main__":
    main()
