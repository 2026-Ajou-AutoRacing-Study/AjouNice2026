#!/usr/bin/env python3
"""GT-only source-footer clock sensitivity for single-batch 50-packet inputs.

Sensor bytes, model timestamps, camera selection and predictions are unchanged.
Only the diagnostic current-object GT uses an experimental closest-UTC-hour
source batch anchor. This is not a runtime clock correction, per-ray timing,
tracker velocity/FDE evaluation or synchronized-LAN timestamp certification.
"""

import argparse
import bisect
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np

from prepare_independent_vehicle_inputs import (
    ROOT, CLASSES, bracket_pose, digest, local_bottom_box, normalize, pose_matrix,
    save, transform_pose, unresolved_nearby, world_gravity_box, count_support,
    read_case_events,
)
from lidar_packet_reassembly import footer_nearest_hour, rotations, quantiles


def scan_anchor(scan):
    """Accept only one clear native source batch in exactly 50 packets."""
    packets = scan.packets
    if len(packets) != 50:
        return dict(eligible=False, reason="not_exactly_50_packets", packet_count=len(packets))
    footers, anchors = [], []
    for packet in packets:
        rotations(packet)  # Standard block/azimuth contract, no decoder launch.
        footer = int.from_bytes(bytes(packet.data)[1200:1204], "little")
        footers.append(footer)
        anchors.append(footer_nearest_hour(packet.stamp.to_sec(), footer))
    if len(set(footers)) != 1 or max(anchors)-min(anchors) > 1.e-6:
        return dict(eligible=False, reason="mixed_or_hour_ambiguous_source_batches", packet_count=50,
                    footer_distinct=len(set(footers)))
    if abs(scan.header.stamp.to_sec()-packets[0].stamp.to_sec()) > 1.e-6:
        raise ValueError("Original first-packet header contract changed")
    return dict(eligible=True, reason=None, packet_count=50, footer_distinct=1,
                source_footer_usec=footers[0], evaluation_reference_stamp=max(anchors),
                source_footer_minus_header_sec=max(anchors)-scan.header.stamp.to_sec())


def unchanged_sensor_contract(before, after):
    keys = ("token", "case", "scene", "policy", "stamp", "receipt", "raw_lidar_index",
            "point_path", "point_sha256", "images", "image_sha256", "camera_offsets_sec",
            "camera_stamps", "camera_receipts", "ego", "raw_ego", "lidar_translation", "world_from_lidar")
    for key in keys:
        if key in before and after.get(key) != before[key]:
            raise ValueError("GT-only sensitivity changed frozen input/header pose: " + key)


def read_anchor_and_ego(bag, heading):
    import rosbag
    anchors, ego = {}, {}
    with rosbag.Bag(str(bag)) as stream:
        for topic, message, receipt in stream.read_messages(topics=["/sensor/lidar1/packets", "/Ego_topic"]):
            stamp = message.header.stamp.to_sec() or receipt.to_sec()
            if topic == "/sensor/lidar1/packets":
                if stamp in anchors:
                    raise ValueError("Duplicate original scan stamp")
                anchors[stamp] = scan_anchor(message)
            else:
                ego[stamp] = dict(x=float(message.position.x), y=float(message.position.y), z=float(message.position.z),
                                 yaw=normalize(math.radians(float(message.heading)+heading)))
    return anchors, sorted(ego.items())


def derive(args):
    if args.output.exists():
        raise FileExistsError("Fresh derived GT output required")
    inputs = json.loads(args.inputs.read_text())
    progress = json.loads(args.progress.read_text())
    case_paths = {f"{row['scene']}_{row['policy']}": row for row in progress["completed"]}
    if len(case_paths) != len(progress["completed"]):
        raise ValueError("Unique scene/policy sources required")
    frozen = {str(path.resolve()): digest(path) for path in (args.inputs, args.progress, args.sensor, args.catalog)}
    if inputs["sensor_sha256"] != digest(args.sensor) or inputs["catalog_sha256"] != digest(args.catalog):
        raise ValueError("Sensitivity calibration/catalog differs from frozen native preparation")
    sys.path[:0] = [str(ROOT / "docker/bevfusion"), str(ROOT / "base_ws/src/tracking_prediction_evaluation/src")]
    from app.sensor4_calibration import load_sensor4_calibration
    from tracking_prediction_evaluation.trajectory import read_morai_trajectories, interpolate_point
    from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
    from types import SimpleNamespace

    calibration = load_sensor4_calibration(args.sensor)
    if not np.allclose(calibration["base_from_lidar"][:3, :3], np.eye(3)):
        raise ValueError("Planar GT sensitivity requires unrotated LiDAR mounting")
    translation = calibration["base_from_lidar"][:3, 3]
    cache, rows, counts, deltas = {}, [], Counter(), []
    for original in inputs["rows"]:
        row = dict(original, source_footer_gt_eligible=False, evaluation_reference_stamp=None,
                   evaluation_purpose="Current detector-center clock sensitivity only; never feed this reference into tracker velocity or FDE.")
        for key in ("gt_boxes", "gt_labels", "gt_points", "gt_assets", "parent_uids", "gt_world_boxes", "gt_actor_metadata"):
            row["header_"+key] = original.get(key)
        counts["original_rows"] += 1
        if not original["scene_epoch_valid"] or original["policy"] != "fixed50":
            row.update(scoring_eligible=False, source_footer_exclusion_reason="preload_or_not_fixed50")
            rows.append(row)
            continue
        name = row["case"]
        if name not in cache:
            case = case_paths[name]
            bag = Path(case["bag"])
            if digest(bag) != case["bag_sha256"]:
                raise ValueError("Raw source changed")
            anchors, ego = read_anchor_and_ego(bag, inputs["heading_correction_deg"])
            raw = read_morai_trajectories(bag, include_classes=CLASSES, minimum_points=1,
                                         heading_correction_deg=inputs["heading_correction_deg"])
            corrected, resolution = transform_segments_to_asset_catalog_bbox_center(raw, args.catalog)
            cache[name] = dict(bag=str(bag), bag_sha256=case["bag_sha256"], anchors=anchors, ego=ego,
                               events=read_case_events(case["folder"]),
                               raw=raw, corrected=corrected, resolved_keys={segment.key for segment in corrected},
                               catalog_resolution=resolution)
        source = cache[name]
        anchor = source["anchors"].get(row["stamp"])
        if anchor is None:
            raise ValueError("Original cloud lacks matching recorded scan header")
        if not anchor["eligible"]:
            row.update(scoring_eligible=False, source_footer_exclusion_reason=anchor["reason"], source_footer_anchor=anchor)
            counts[anchor["reason"]] += 1
            rows.append(row)
            continue
        reference = anchor["evaluation_reference_stamp"]
        if reference < source["events"]["scene_ready_stamp"]:
            row.update(scoring_eligible=False, source_footer_exclusion_reason="footer_outside_loaded_epoch", source_footer_anchor=anchor)
            counts["footer_outside_loaded_epoch"] += 1
            rows.append(row)
            continue
        raw_ego = bracket_pose(source["ego"], reference, inputs["pose_bracket_sec"])
        if raw_ego is None:
            row.update(scoring_eligible=False, source_footer_exclusion_reason="unbracketed_footer_ego", source_footer_anchor=anchor)
            counts["unbracketed_footer_ego"] += 1
            rows.append(row)
            continue
        world_ego = transform_pose(raw_ego, row["map_from_morai"])
        world_from_lidar = pose_matrix(world_ego) @ calibration["base_from_lidar"]
        nearby_unknown = unresolved_nearby(source["raw"], source["resolved_keys"], raw_ego, reference,
                                           interpolate_point, inputs["pose_bracket_sec"])
        boxes, labels, assets, keys, support, world_boxes, metadata, unsupported = [], [], [], [], [], [], [], []
        points = np.fromfile(row["point_path"], "<f4").reshape(-1, 4)
        finite = points[np.isfinite(points).all(axis=1) & (points[:, :3] != 0).any(axis=1)]
        if digest(row["point_path"]) != row["point_sha256"]:
            raise ValueError("Frozen model input changed")
        for segment in source["corrected"]:
            if not segment.points[0].timestamp <= reference <= segment.points[-1].timestamp:
                continue
            point = interpolate_point(segment.points, reference, inputs["pose_bracket_sec"])
            if point is None:
                unsupported.append(segment.key)
                continue
            box = local_bottom_box(point, raw_ego, translation)
            if abs(box[0]) > 100 or abs(box[1]) > 30:
                continue
            boxes.append(box)
            labels.append(CLASSES.index(segment.class_name))
            assets.append(segment.asset_name)
            keys.append(name+"|"+segment.key)
            support.append(count_support(finite, box))
            world_boxes.append(world_gravity_box(box, world_from_lidar))
            metadata.append(dict(reconstructed_key=segment.key, asset=segment.asset_name,
                                 reference_stamp=reference, purpose="Catalog current-center diagnostic"))
        row.update(
            gt_boxes=boxes, gt_labels=labels, gt_points=support, gt_assets=assets, parent_uids=keys,
            gt_world_boxes=world_boxes, gt_actor_metadata=metadata, gt_unknown_near_roi=nearby_unknown,
            gt_unbracketed_actor_keys=unsupported, evaluation_reference_stamp=reference,
            evaluation_ego=world_ego, evaluation_raw_ego=raw_ego,
            evaluation_world_from_lidar=world_from_lidar.tolist(), source_footer_anchor=anchor,
            source_footer_gt_eligible=True, source_footer_exclusion_reason=None,
            scoring_eligible=bool(original["scoring_eligible"] and not nearby_unknown and not unsupported),
            gt_world_transform_warning="gt_world_boxes use evaluation_world_from_lidar; ego/world_from_lidar/stamp remain original model-header contract.",
        )
        unchanged_sensor_contract(original, row)
        rows.append(row)
        counts["eligible_single_source_fixed50_rows"] += 1
        counts["eligible_scoring_rows"] += row["scoring_eligible"]
        deltas.append(anchor["source_footer_minus_header_sec"]*1000)
    for original, derived in zip(inputs["rows"], rows):
        unchanged_sensor_contract(original, derived)
    if frozen != {path: digest(path) for path in frozen}:
        raise ValueError("Frozen GT/input contracts changed during sensitivity extraction")
    report = dict(
        schema_version=1, rows=rows, input_source_sha256=frozen,
        source_inputs_sha256=digest(args.inputs), original_inputs_unchanged=True,
        header_vs_latest_footer_ms=quantiles(deltas), counts=dict(counts),
        per_case={key: dict(bag=value["bag"], bag_sha256=value["bag_sha256"],
                            catalog_resolution=value["catalog_resolution"]) for key, value in cache.items()},
        purpose="GT-only current detector center-error sensitivity on exact single-source-batch fixed50 frames.",
        limitations=["Nearest UTC-hour footer is an experimental source-batch anchor, not calibrated wall time or exact ray acquisition.",
                     "Do not use derived reference as tracker velocity/FDE clock, model timestamp, online queue age or runtime correction.",
                     "Header pose/points/JPEG are frozen; alternate GT pose is explicitly separated in evaluation_* metadata.",
                     "Mixed-source 51 frames are excluded from this sensitivity, not claimed worse solely because they lack one anchor."],
    )
    save(args.output, report)
    print(json.dumps(dict(output=str(args.output.resolve()), counts=dict(counts),
                          header_vs_latest_footer_ms=report["header_vs_latest_footer_ms"])), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--progress", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sensor", type=Path, default=ROOT / "configs/sensors/Sensor_competition_local.json")
    parser.add_argument("--catalog", type=Path,
                        default=ROOT / "dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json")
    derive(parser.parse_args())


if __name__ == "__main__":
    main()
