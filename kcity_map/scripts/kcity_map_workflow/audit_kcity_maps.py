#!/usr/bin/env python3
"""Generate K-City map inventory and coarse alignment reports."""

from __future__ import annotations

import csv
import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import shapefile
from lxml import etree
from pyproj import Transformer
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "reports"
UTM52 = "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def write_csv(path: Path, fieldnames: List[str], rows: Iterable[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def osm_summary(path: Path) -> Dict[str, object]:
    root = etree.parse(str(path)).getroot()
    meta = root.find("MetaInfo")
    rel_counts = {}
    way_counts = {}
    speed = {}
    ele = []
    road_missing_speed = 0
    road_vehicle_yes = 0
    road_vehicle_no = 0
    for node in root.findall("node"):
        tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
        if "ele" in tags:
            ele.append(float(tags["ele"]))
    for way in root.findall("way"):
        tags = {tag.get("k"): tag.get("v") for tag in way.findall("tag")}
        key = f"{tags.get('type', '')}/{tags.get('subtype', '')}"
        way_counts[key] = way_counts.get(key, 0) + 1
    for relation in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
        key = f"{tags.get('type', '')}/{tags.get('subtype', '')}"
        rel_counts[key] = rel_counts.get(key, 0) + 1
        if tags.get("type") == "lanelet" and tags.get("subtype") == "road":
            if tags.get("participant:vehicle", "yes") == "yes":
                road_vehicle_yes += 1
            elif tags.get("participant:vehicle") == "no":
                road_vehicle_no += 1
            value = tags.get("speed_limit")
            if value is None:
                road_missing_speed += 1
            else:
                speed[value] = speed.get(value, 0) + 1
    return {
        "file": str(path.relative_to(ROOT)),
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
        "format_version": meta.get("format_version") if meta is not None else "",
        "map_version": meta.get("map_version") if meta is not None else "",
        "nodes": len(root.findall("node")),
        "ways": len(root.findall("way")),
        "relations": len(root.findall("relation")),
        "road_lanelets": rel_counts.get("lanelet/road", 0),
        "road_vehicle_yes": road_vehicle_yes,
        "road_vehicle_no": road_vehicle_no,
        "crosswalk_lanelets": rel_counts.get("lanelet/crosswalk", 0),
        "traffic_light_regs": rel_counts.get("regulatory_element/traffic_light", 0),
        "stop_line_ways": way_counts.get("line_thin/stop_line", 0),
        "road_missing_speed": road_missing_speed,
        "ele_count": len(ele),
        "ele_min": min(ele) if ele else "",
        "ele_max": max(ele) if ele else "",
        "ele_unique_rounded_mm": len({round(value, 3) for value in ele}),
        "speed_distribution": json.dumps(speed, ensure_ascii=False, sort_keys=True),
    }


def shapefile_inventory() -> List[Dict[str, object]]:
    rows = []
    for shp in sorted((ROOT / "gis_map").glob("*.shp")):
        reader = shapefile.Reader(str(shp))
        rows.append(
            {
                "source": "GIS",
                "file": str(shp.relative_to(ROOT)),
                "sha256": sha256(shp),
                "records": len(reader),
                "shape_type": reader.shapeTypeName,
                "bbox": json.dumps(list(getattr(reader, "bbox", []))),
                "zbox": json.dumps(list(getattr(reader, "zbox", []))),
                "fields": json.dumps([field[0] for field in reader.fields[1:]], ensure_ascii=False),
            }
        )
    return rows


def json_inventory() -> List[Dict[str, object]]:
    rows = []
    for base in ["mgeo_map", "example_map"]:
        for path in sorted((ROOT / base).glob("*.json")):
            data = json.load(path.open())
            count = len(data) if hasattr(data, "__len__") else ""
            if isinstance(data, list):
                fields = list(data[0].keys()) if data else []
            elif isinstance(data, dict):
                fields = list(data.keys())
            else:
                fields = []
            rows.append(
                {
                    "source": "MGeo" if "mgeo_map" in base else "Example",
                    "file": str(path.relative_to(ROOT)),
                    "sha256": sha256(path),
                    "records": count,
                    "shape_type": "json",
                    "bbox": "",
                    "zbox": "",
                    "fields": json.dumps(fields, ensure_ascii=False),
                }
            )
    return rows


def lanelet_centerline_points(path: Path) -> List[Tuple[float, float]]:
    root = etree.parse(str(path)).getroot()
    transformer = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    nodes = {}
    for node in root.findall("node"):
        nodes[node.get("id")] = transformer.transform(float(node.get("lon")), float(node.get("lat")))
    ways = {way.get("id"): [nd.get("ref") for nd in way.findall("nd")] for way in root.findall("way")}
    points = []
    for rel in root.findall("relation"):
        tags = {tag.get("k"): tag.get("v") for tag in rel.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        for member in rel.findall("member"):
            if member.get("role") == "centerline":
                points.extend(nodes[ref] for ref in ways.get(member.get("ref"), []) if ref in nodes)
    return points


def gis_points(layer: str) -> List[Tuple[float, float]]:
    reader = shapefile.Reader(str(ROOT / f"gis_map/{layer}.shp"))
    points = []
    for shape in reader.shapes():
        points.extend((float(x), float(y)) for x, y in shape.points)
    return points


def mgeo_link_points(base: str) -> List[Tuple[float, float]]:
    global_info = json.load((ROOT / base / "global_info.json").open())
    origin = global_info.get("local_origin_in_global", [0.0, 0.0, 0.0])
    points = []
    for link in json.load((ROOT / base / "link_set.json").open()):
        for point in link.get("points", []):
            points.append((point[0] + origin[0], point[1] + origin[1]))
    return points


def nn_stats(a: List[Tuple[float, float]], b: List[Tuple[float, float]]) -> Dict[str, float]:
    tree = cKDTree(b)
    distances, _ = tree.query(a, k=1)
    values = sorted(float(d) for d in distances)
    if not values:
        return {"count": 0, "median_m": math.nan, "p95_m": math.nan, "max_m": math.nan}
    return {
        "count": len(values),
        "median_m": values[len(values) // 2],
        "p95_m": values[min(len(values) - 1, int(len(values) * 0.95))],
        "max_m": values[-1],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--alignment-lanelet-map",
        default=str(ROOT / "lanelet_map/lanelet2_map_v31.osm"),
        help="Lanelet2 map used for nearest-neighbor alignment summary.",
    )
    args = parser.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    inventory_rows = shapefile_inventory() + json_inventory()
    write_csv(
        OUT / "source_inventory.csv",
        ["source", "file", "sha256", "records", "shape_type", "bbox", "zbox", "fields"],
        inventory_rows,
    )

    osm_rows = [osm_summary(path) for path in sorted((ROOT / "lanelet_map").glob("lanelet2_map_v*.osm"))]
    write_csv(
        OUT / "lanelet_candidate_summary.csv",
        [
            "file",
            "sha256",
            "bytes",
            "format_version",
            "map_version",
            "nodes",
            "ways",
            "relations",
            "road_lanelets",
            "road_vehicle_yes",
            "road_vehicle_no",
            "crosswalk_lanelets",
            "traffic_light_regs",
            "stop_line_ways",
            "road_missing_speed",
            "ele_count",
            "ele_min",
            "ele_max",
            "ele_unique_rounded_mm",
            "speed_distribution",
        ],
        osm_rows,
    )

    lanelet_points = lanelet_centerline_points(Path(args.alignment_lanelet_map))
    datasets = {
        "GIS_A2_LINK": gis_points("A2_LINK"),
        "GIS_B2_SURFACELINEMARK": gis_points("B2_SURFACELINEMARK"),
        "MGeo_current_link": mgeo_link_points("mgeo_map"),
        "Example_link": mgeo_link_points("example_map"),
    }
    alignment_rows = []
    for name, points in datasets.items():
        for direction, a, b in [
            (f"lanelet_centerline_to_{name}", lanelet_points, points),
            (f"{name}_to_lanelet_centerline", points, lanelet_points),
        ]:
            stats = nn_stats(a, b)
            stats["comparison"] = direction
            alignment_rows.append(stats)
    write_csv(OUT / "alignment_summary.csv", ["comparison", "count", "median_m", "p95_m", "max_m"], alignment_rows)
    print(f"wrote {OUT / 'source_inventory.csv'}")
    print(f"wrote {OUT / 'lanelet_candidate_summary.csv'}")
    print(f"wrote {OUT / 'alignment_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
