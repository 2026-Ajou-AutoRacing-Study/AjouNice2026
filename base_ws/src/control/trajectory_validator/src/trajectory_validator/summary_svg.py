"""Thin-line SVG overview matching the competition-route map style."""

import csv
import hashlib
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml
from pyproj import Transformer


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path, required_fields):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(required_fields).difference(reader.fieldnames or [])
        if missing:
            raise ValueError("{} is missing fields {}".format(path, sorted(missing)))
        return list(reader)


def read_source_trajectory_xy(path):
    rows = read_rows(path, ("x", "y"))
    return _xy_points(rows, "x", "y")


def _xy_points(rows, x_field, y_field):
    output = []
    for row in rows:
        try:
            point = (float(row.get(x_field, "")), float(row.get(y_field, "")))
        except (TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in point):
            output.append(point)
    return output


def _projector(path):
    with Path(path).open() as stream:
        document = yaml.safe_load(stream)
    if document.get("projector_type") != "LocalCartesianUTM":
        raise ValueError("only LocalCartesianUTM projectors are supported")
    origin = document["map_origin"]
    latitude = float(origin["latitude"])
    longitude = float(origin["longitude"])
    zone = int((longitude + 180.0) // 6.0) + 1
    epsg = 32600 + zone if latitude >= 0.0 else 32700 + zone
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:{}".format(epsg), always_xy=True)
    origin_x, origin_y = transformer.transform(longitude, latitude)
    return transformer, origin_x, origin_y


def _inside_bounds(points, bounds):
    min_x, max_x, min_y, max_y = bounds
    return not (
        max(x for x, _ in points) < min_x
        or min(x for x, _ in points) > max_x
        or max(y for _, y in points) < min_y
        or min(y for _, y in points) > max_y
    )


def way_linestyle(tags):
    if tags.get("type") == "virtual":
        return (0, (0.4, 1.2))
    if tags.get("type") == "line_thin" and tags.get("subtype") == "dashed":
        return (0, (1.2, 1.0))
    if tags.get("type") == "line_thin" and tags.get("subtype") == "solid_dashed":
        return (0, (2.0, 0.8, 0.4, 0.8))
    return "-"


def lanelet_segments(map_path, projector_path, bounds):
    """Return cropped road boundaries and markings in competition-route style."""
    root = ET.parse(str(map_path)).getroot()
    transformer, origin_x, origin_y = _projector(projector_path)
    nodes = {}
    for node in root.findall("node"):
        x, y = transformer.transform(float(node.attrib["lon"]), float(node.attrib["lat"]))
        nodes[int(node.attrib["id"])] = (x - origin_x, y - origin_y)
    ways = {}
    for way in root.findall("way"):
        way_id = int(way.attrib["id"])
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in way.findall("tag")}
        points = [
            nodes[int(node.attrib["ref"])]
            for node in way.findall("nd")
            if int(node.attrib["ref"]) in nodes
        ]
        ways[way_id] = (points, tags)
    boundary_ids = set()
    for relation in root.findall("relation"):
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        for member in relation.findall("member"):
            if member.attrib.get("type") == "way" and member.attrib.get("role") in ("left", "right"):
                boundary_ids.add(int(member.attrib["ref"]))
    output = []
    for way_id in sorted(boundary_ids):
        points, tags = ways.get(way_id, ([], {}))
        if len(points) >= 2 and _inside_bounds(points, bounds):
            output.append((points, "#b8b8b8", 0.075, 0.55, way_linestyle(tags)))
    for way_id in sorted(ways):
        points, tags = ways[way_id]
        if tags.get("type") != "line_thin" or len(points) < 2 or not _inside_bounds(points, bounds):
            continue
        color = "#737373" if tags.get("subtype") == "dashed" else "#606060"
        output.append((points, color, 0.085, 0.45, way_linestyle(tags)))
    return output


def _draw_line(axis, points, color, linewidth, alpha=1.0, linestyle="-", zorder=1):
    if len(points) < 2:
        return
    axis.plot(
        [point[0] for point in points],
        [point[1] for point in points],
        color=color,
        linewidth=linewidth,
        alpha=alpha,
        linestyle=linestyle,
        solid_capstyle="butt",
        dash_capstyle="butt",
        zorder=zorder,
    )


def _atomic_save_svg(figure, output_path):
    output_path = Path(output_path)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=output_path.stem + ".", suffix=".svg.tmp", dir=str(output_path.parent)
    )
    os.close(descriptor)
    os.chmod(temporary_name, 0o644)
    try:
        figure.savefig(
            temporary_name,
            format="svg",
            bbox_inches="tight",
            pad_inches=0.02,
            facecolor="white",
        )
        os.replace(temporary_name, output_path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def render_run_summary_svg(
    run_directory,
    map_path,
    projector_path,
    filename="summary.svg",
    margin_m=20.0,
    gt_points=None,
):
    run_directory = Path(run_directory)
    if Path(filename).name != filename or not filename.lower().endswith(".svg"):
        raise ValueError("summary SVG filename must be an SVG basename")
    if gt_points is None:
        gt_points = _xy_points(
            read_rows(run_directory / "gt.csv", ("gt_x_m", "gt_y_m")),
            "gt_x_m",
            "gt_y_m",
        )
    else:
        gt_points = [(float(x), float(y)) for x, y in gt_points]
    sample_path = run_directory / "tracking_samples.csv"
    if not sample_path.is_file():
        sample_path = run_directory / "control_cycles.csv"
    ego_points = _xy_points(
        read_rows(sample_path, ("ego_x_m", "ego_y_m")), "ego_x_m", "ego_y_m"
    )
    if len(gt_points) < 2:
        raise ValueError("summary SVG requires at least two GT points")
    if not math.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("summary SVG margin_m must be finite and non-negative")
    points = gt_points + ego_points
    min_x = min(x for x, _ in points) - margin_m
    max_x = max(x for x, _ in points) + margin_m
    min_y = min(y for _, y in points) - margin_m
    max_y = max(y for _, y in points) + margin_m
    if max_x - min_x < 20.0:
        center = (min_x + max_x) / 2.0
        min_x, max_x = center - 10.0, center + 10.0
    if max_y - min_y < 20.0:
        center = (min_y + max_y) / 2.0
        min_y, max_y = center - 10.0, center + 10.0
    bounds = (min_x, max_x, min_y, max_y)
    segments = lanelet_segments(map_path, projector_path, bounds)
    width_in = 8.0
    height_in = width_in * (max_y - min_y) / (max_x - min_x)
    figure, axis = plt.subplots(figsize=(width_in, height_in), dpi=620)
    axis.set_facecolor("white")
    for segment in segments:
        _draw_line(axis, *segment, zorder=1)
    _draw_line(axis, gt_points, "#00a651", 0.13, 0.95, zorder=5)
    if ego_points:
        _draw_line(axis, ego_points, "#0066cc", 0.13, 0.95, zorder=6)
    axis.scatter(*gt_points[0], s=4.0, color="#2e7d32", zorder=7)
    axis.scatter(*gt_points[-1], s=4.0, color="#00a651", zorder=7)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(min_x, max_x)
    axis.set_ylim(min_y, max_y)
    axis.axis("off")
    output_path = run_directory / filename
    _atomic_save_svg(figure, output_path)
    plt.close(figure)
    return {
        "status": "complete",
        "file": filename,
        "sha256": sha256_file(output_path),
        "gt_point_count": len(gt_points),
        "ego_point_count": len(ego_points),
    }
