#!/usr/bin/env python3

import argparse
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

import lanelet2_osm_marker_publisher as viz  # noqa: E402


def cs2cs_utm(lat, lon, zone):
    if not shutil.which("cs2cs"):
        return None
    cmd = [
        "cs2cs",
        "-f",
        "%.9f",
        "+proj=longlat",
        "+datum=WGS84",
        "+to",
        "+proj=utm",
        "+zone=%d" % zone,
        "+datum=WGS84",
        "+units=m",
        "+no_defs",
    ]
    result = subprocess.run(
        cmd,
        input="%.12f %.12f\n" % (lon, lat),
        text=True,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    parts = result.stdout.strip().split()
    return float(parts[0]), float(parts[1])


def iter_sample_nodes(map_file, max_samples):
    count = 0
    for _, elem in ET.iterparse(map_file, events=("end",)):
        if elem.tag == "node":
            yield float(elem.attrib["lat"]), float(elem.attrib["lon"])
            count += 1
            if count >= max_samples:
                return
        elem.clear()


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def get_expected_static_tf(static_cfg, projector):
    source = static_cfg["map_to_morai_map"]["source"]
    lanelet_origin = [
        projector["origin_easting"],
        projector["origin_northing"],
        projector["origin_alt"],
    ]

    if "morai_runtime_origin_delta_xyz" in source:
        expected_xyz = list(source["morai_runtime_origin_delta_xyz"])
    elif "morai_runtime_gps_east_north_offset" in source:
        morai_origin = source["morai_runtime_gps_east_north_offset"]
        expected_xyz = [
            morai_origin[0] - lanelet_origin[0],
            morai_origin[1] - lanelet_origin[1],
            0.0,
        ]
    else:
        morai_origin = source["morai_local_origin_in_global"]
        expected_xyz = [
            morai_origin[0] - lanelet_origin[0],
            morai_origin[1] - lanelet_origin[1],
            morai_origin[2] - lanelet_origin[2],
        ]

    refinement = source.get("alignment_refinement", {})
    delta = refinement.get("translation_delta_from_runtime_origin_m")
    if delta is not None:
        expected_xyz = [expected_xyz[i] + delta[i] for i in range(3)]
    return expected_xyz, refinement


def validate_projector(map_file, projector_file, static_tf_file, max_samples):
    projector = viz.read_projector(projector_file)
    print("projector_type: LocalCartesianUTM")
    print("utm_zone: %d" % projector["zone"])
    print(
        "origin_utm: easting=%.9f northing=%.9f altitude=%.3f"
        % (projector["origin_easting"], projector["origin_northing"], projector["origin_alt"])
    )

    cs_origin = cs2cs_utm(projector["origin_lat"], projector["origin_lon"], projector["zone"])
    if cs_origin is not None:
        de = projector["origin_easting"] - cs_origin[0]
        dn = projector["origin_northing"] - cs_origin[1]
        print("origin_vs_cs2cs_error_m: e=%.9g n=%.9g" % (de, dn))
        if abs(de) > 0.01 or abs(dn) > 0.01:
            raise RuntimeError("origin UTM conversion differs from cs2cs by more than 1 cm")

    max_error = 0.0
    samples = 0
    for lat, lon in iter_sample_nodes(map_file, max_samples):
        ours = viz.wgs84_to_utm(lat, lon, projector["zone"])
        ref = cs2cs_utm(lat, lon, projector["zone"])
        if ref is None:
            break
        error = math.hypot(ours[0] - ref[0], ours[1] - ref[1])
        max_error = max(max_error, error)
        samples += 1
    if samples:
        print("sample_vs_cs2cs: samples=%d max_error_m=%.9g" % (samples, max_error))
        if max_error > 0.01:
            raise RuntimeError("sample UTM conversion differs from cs2cs by more than 1 cm")
    else:
        print("sample_vs_cs2cs: skipped (cs2cs not available)")

    static_cfg = load_yaml(static_tf_file)
    expected_xyz, refinement = get_expected_static_tf(static_cfg, projector)
    configured_xyz = static_cfg["map_to_morai_map"]["xyz"]
    diff = [configured_xyz[i] - expected_xyz[i] for i in range(3)]
    print("expected_map_to_morai_xyz: [%.9f, %.9f, %.9f]" % tuple(expected_xyz))
    print("configured_map_to_morai_xyz: [%.9f, %.9f, %.9f]" % tuple(configured_xyz))
    print("static_tf_error_m: [%.9g, %.9g, %.9g]" % tuple(diff))
    if max(abs(v) for v in diff) > 0.01:
        raise RuntimeError("static TF differs from projector-derived UTM delta by more than 1 cm")

    configured_yaw = static_cfg["map_to_morai_map"].get("rpy", [0.0, 0.0, 0.0])[2]
    expected_yaw = refinement.get("yaw_rad", 0.0)
    yaw_diff = configured_yaw - expected_yaw
    print("expected_map_to_morai_yaw_rad: %.12f" % expected_yaw)
    print("configured_map_to_morai_yaw_rad: %.12f" % configured_yaw)
    print("static_tf_yaw_error_rad: %.9g" % yaw_diff)
    if abs(yaw_diff) > 1e-9:
        raise RuntimeError("static TF yaw differs from recorded refinement")

    _, stats = viz.build_marker_array(map_file, projector_file, frame_id="map", flatten_z=False)
    print(
        "marker_stats: nodes={nodes} ways={ways} relations={relations} "
        "road_lanelets={road_lanelets} crosswalk_lanelets={crosswalk_lanelets} "
        "road_borders={road_borders} stop_lines={stop_lines} traffic_lights={traffic_lights} "
        "markers={markers}".format(**stats)
    )
    if stats["road_lanelets"] == 0 or stats["markers"] == 0:
        raise RuntimeError("marker generation produced no lanelet visualization")


def main():
    repo_root = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-file", default=os.path.join(repo_root, "kcity_map", "lanelet_map", "lanelet2_map_v23.osm"))
    parser.add_argument(
        "--projector-file",
        default=os.path.join(repo_root, "kcity_map", "lanelet_map", "map_projector_info_v1.yaml"),
    )
    parser.add_argument(
        "--static-tf-file", default=os.path.join(repo_root, "configs", "tf", "morai_lanelet_static_tf.yaml")
    )
    parser.add_argument("--samples", type=int, default=200)
    args = parser.parse_args()
    validate_projector(args.map_file, args.projector_file, args.static_tf_file, args.samples)


if __name__ == "__main__":
    main()
