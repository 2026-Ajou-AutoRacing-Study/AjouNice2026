#!/usr/bin/env python3
"""Render v76/v77 highway geometry comparison for visual review."""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
MAPS = {
    "v76": ROOT / "lanelet_map/lanelet2_map_v76.osm",
    "v77": ROOT / "lanelet_map/lanelet2_map_v77.osm",
}
OUTPUT = ROOT / "docs/kcity_v77_highway_geometry_comparison.png"
TARGETS = {
    200099, 200102, 200127, 200128, 200130, 200134, 200137, 200138,
    200141, 200148, 200186, 200187, 201435, 201436, 201437, 201438,
    201439, 201440, 201441, 201444, 201445, 201446, 201456, 201457,
    201458, 201459, 201463, 201464, 201465, 201466, 201467, 201468,
    201469, 201470, 201471, 201472, 201473, 201474, 201475,
}
WINDOWS = (
    ("north merge", (178.0, 204.0), (620.0, 706.0)),
    ("middle LC", (178.0, 204.0), (378.0, 459.0)),
    ("bottleneck", (178.0, 204.0), (300.0, 397.0)),
)


def load(path):
    root = etree.parse(str(path), etree.XMLParser(huge_tree=True)).getroot()
    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    transform = Transformer.from_crs(4326, 32652, always_xy=True)
    origin = transform.transform(126.7730819386, 37.2380038082)

    def xy(way_id):
        result = []
        for item in ways[way_id].findall("nd"):
            node = nodes[int(item.get("ref"))]
            x, y = transform.transform(float(node.get("lon")), float(node.get("lat")))
            result.append((x - origin[0], y - origin[1]))
        return result

    result = {}
    for lanelet_id in TARGETS:
        relation = relations[lanelet_id]
        roles = {
            item.get("role"): int(item.get("ref"))
            for item in relation.findall("member") if item.get("type") == "way"
        }
        result[lanelet_id] = (xy(roles["left"]), xy(roles["right"]), xy(roles["centerline"]))
    return result


def main():
    data = {name: load(path) for name, path in MAPS.items()}
    figure, axes = plt.subplots(3, 2, figsize=(10, 18), constrained_layout=True)
    for row, (window_name, xlim, ylim) in enumerate(WINDOWS):
        for column, version in enumerate(("v76", "v77")):
            axis = axes[row, column]
            for lanelet_id, (left, right, center) in data[version].items():
                polygon = left + list(reversed(right))
                if max(point[1] for point in polygon) < ylim[0] or min(point[1] for point in polygon) > ylim[1]:
                    continue
                px = [point[0] for point in polygon] + [polygon[0][0]]
                py = [point[1] for point in polygon] + [polygon[0][1]]
                axis.fill(px, py, alpha=0.16, color="#3b82f6")
                axis.plot(px, py, linewidth=0.8, color="#dbeafe")
                axis.plot([point[0] for point in center], [point[1] for point in center],
                          linewidth=0.8, color="#facc15")
                midpoint = center[len(center) // 2]
                axis.text(midpoint[0], midpoint[1], str(lanelet_id), fontsize=6,
                          color="#f8fafc", rotation=90, ha="center", va="center")
            axis.set_xlim(*xlim)
            axis.set_ylim(*ylim)
            axis.set_aspect("equal")
            axis.set_facecolor("#111827")
            axis.grid(color="#374151", linewidth=0.3)
            axis.set_title(f"{version} - {window_name}")
    figure.suptitle("K-City highway geometry: v76 vs v77", fontsize=14)
    figure.savefig(OUTPUT, dpi=190, facecolor="white")
    print(OUTPUT)


if __name__ == "__main__":
    main()
