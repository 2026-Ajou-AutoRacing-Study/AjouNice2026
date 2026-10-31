#!/usr/bin/env python3
"""Offline association tolerance sweep using saved detections and reviewed boxes.

The covariance/yaw offsets below are declared stress inputs, not measured pose
errors. Named housing annotations are incomplete and not independent human GT.
This tool never runs ROS, a simulator, GPU inference, or changes runtime config.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "base_ws/src/perception/traffic_light_detection"
sys.path[:0] = [str(PACKAGE / "scripts"), str(Path(__file__).parent)]
from traffic_light_map_selector import (associate_boxes, covariance_sigmas,
    load_camera, load_geometry, project_heads, selector_config)
from evaluate_traffic_light_pipeline import development_camera_transform, manual_identity

PRESETS = {
    "baseline_yaw3_x90_y200": {"max_yaw_sigma_degrees": 3., "max_padding_px": 90.},
    "yaw4_x120_y200": {"max_yaw_sigma_degrees": 4., "max_padding_px": 120.},
    "yaw5_x160_y240": {"max_yaw_sigma_degrees": 5., "max_padding_px": 160.,
                        "max_vertical_padding_px": 240.},
    "broad_yaw10_x320_y360": {"max_yaw_sigma_degrees": 10., "max_padding_px": 320.,
                             "max_vertical_padding_px": 360.},
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def box(values):
    a, b, c, d, confidence = values
    return ((a+c)/2560, (b+d)/1440, (c-a)/1280, (d-b)/720,
            (a, b, c, d, confidence))


def projected(geometry, camera, record, group, config, yaw_sigma, yaw_offset):
    covariance = [0.] * 36
    covariance[0] = covariance[7] = .027136410963869192 ** 2
    covariance[14] = 1.
    covariance[35] = math.radians(yaw_sigma) ** 2
    xy, z, yaw = covariance_sigmas(covariance, config.max_position_sigma_m,
                                  config.max_yaw_sigma_degrees, config.max_vertical_sigma_m)
    ego = copy.deepcopy(record["ego"])
    key = "heading_degrees" if "heading_degrees" in ego else "heading"
    ego[key] += yaw_offset
    transform = development_camera_transform(ego, camera)
    return project_heads(geometry, camera, transform, group, xy, yaw,
                         vertical_sigma_m=z, **config.projection_kwargs())[0]


def evaluate(predictions, annotations, poses, geometry, camera, config, yaw_sigma, yaw_offset):
    counts = Counter()
    changes = []
    head_groups = {h["id"]: h.get("group_id") for h in geometry["heads"]}
    for row in predictions:
        sha = row["image_sha256"]
        key = (sha, row['image_path'])
        if key not in annotations or key not in poses:
            continue
        annotation, group = annotations[key], row["group_regulatory_id"]
        counts["frames"] += 1
        try:
            rois = projected(geometry, camera, poses[key], group, config, yaw_sigma, yaw_offset)
        except ValueError as error:
            counts["rejected:" + str(error)] += 1
            continue
        selected, decision = associate_boxes([box(v) for v in row["detections"]], rois, group, config=config)
        if selected is not None:
            counts["selected"] += 1
            identity = manual_identity(list(selected[4][:4]), annotation["boxes"])
            if identity["known"]:
                same = head_groups.get(identity["native_id"]) == group
                counts["correct_named_mission" if same else "wrong_named_mission"] += 1
                if not same:
                    changes.append(dict(image_sha256=sha, image_path=row["image_path"],
                        group_id=group, selected_native_id=identity["native_id"], selection=decision))
            else:
                counts["selected_identity_unverified"] += 1
        for other in annotation["boxes"]:
            if head_groups.get(other["native_id"]) != group:
                counts["actual_other_head_challenges"] += 1
                other_selected, _ = associate_boxes([box(other["xyxy"] + [.9])], rois, group, config=config)
                counts["actual_other_head_false_matches"] += other_selected is not None
    return dict(counts=dict(counts), wrong_named_examples=changes[:10])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.relative_to(ROOT / "artifacts/evaluation")
    if output.exists():
        raise FileExistsError("Fresh audit output required")
    directory = ROOT / "artifacts/evaluation/traffic_light_robustness_261025"
    paths = [directory / "pipeline_baseline640_development348_v1.json",
             directory / "housing_detector_development_curated_v1.json",
             directory / "pipeline_final960_continuous969_v1.json",
             directory / "continuous_natural_campaign_v1/identity_qc_v2.json",
             directory / "continuous_natural_campaign_v1/manifest.json",
             PACKAGE / "config/traffic_light_map_geometry.json",
             ROOT / "configs/sensors/Sensor_competition_local.json",
             PACKAGE / "scripts/traffic_light_map_selector.py", Path(__file__).resolve()]
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in paths}
    documents = [json.loads(p.read_text()) for p in paths[:5]]
    geometry, camera = load_geometry(paths[5]), load_camera(paths[6])
    datasets = {
        "development348_saved640_detector": (documents[0]["records"], documents[1]["records"], documents[1]["records"]),
        "natural969_saved_final960_detector": (documents[2]["records"], documents[3]["records"], documents[4]["records"]),
    }
    results = {}
    for name, (predictions, annotations, poses) in datasets.items():
        annotated = {(r["image_sha256"], r['image_path']): r for r in annotations}
        positioned = {(r["image_sha256"], r['image_path']): r for r in poses}
        results[name] = {}
        for yaw_sigma, yaw_offset in ((1.6644, 0.), (3.5, 0.), (3.5, -3.), (3.5, 3.)):
            scenario = "sigma%g_mean_yaw_offset%g" % (yaw_sigma, yaw_offset)
            results[name][scenario] = {preset: evaluate(predictions, annotated, positioned,
                geometry, camera, selector_config(values), yaw_sigma, yaw_offset)
                for preset, values in PRESETS.items()}
    if any(digest(ROOT / name) != value for name, value in hashes.items()):
        raise RuntimeError("Audit source changed")
    report = dict(scope="saved_detection_association_only_no_new_detector_or_lamp_accuracy",
        sources=hashes, presets={k: selector_config(v).to_dict() for k, v in PRESETS.items()},
        stress_inputs=dict(xy_sigma_m=.027136410963869192, z_sigma_m=1.,
                           yaw_sigma_degrees=[1.6644, 3.5], mean_yaw_offset_degrees=[0., -3., 3.]),
        results=results, limitations=["Declared covariance and yaw offsets are stress inputs, not field frequencies.",
            "Named boxes are incomplete reviewed development annotations, not independent human GT.",
            "Wrong named mission and selected identity unverified are different outcomes.",
            "This audit measures selection; it does not classify lamp state or evaluate driving."])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(dict(output=str(output), results={name: {s: {k: v["counts"] for k,v in values.items()}
        for s,values in scenarios.items()} for name,scenarios in results.items()}), ensure_ascii=False))


if __name__ == "__main__":
    main()
