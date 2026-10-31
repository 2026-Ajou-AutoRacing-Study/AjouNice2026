#!/usr/bin/env python3
"""Build read-only references to native-state/map-oracle crop RESEARCH data.

Repeated --group arguments are SPLIT:POSE_GROUP:NATIVE_ID:MATRIX_DIRECTORY.
The output JSON's parent must be a common existing artifact root containing ALL
matrix directories and referenced original JPEGs. No images are copied/edited.
Only complete cells with matching pre/post environment API evidence qualify.
Partial/failed cells and rejected raw retry frames are reported, not relabeled.

Native ID/state verification is NOT visible-lamp or crop ground truth. Housing
ROIs are approximate static-map projections, not YOLO detections. Manual image,
visibility, housing identity, crop and phase-label QC remain mandatory. This
builder performs no training, model inference, ROS, network or simulator calls.
"""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

from project_traffic_light_housing import (
    digest, load_fixed_camera, load_signal, parse_ego_document, project_housing,
)
from sample_traffic_light_bag_frames import jpeg_dimensions
from train_traffic_light_crop_probe import validate_crop, validate_labels


ENVIRONMENTS = [(weather, hour) for weather in ("Sunny", "Foggy") for hour in (13, 11, 15)]
STATES = {1, 4, 16, 32, 33, 48, 5}
CLASSES = ("red", "yellow", "green", "left_arrow")


class SkipFrame(ValueError):
    def __init__(self, reason, detail):
        super().__init__(detail)
        self.reason = reason


def parse_group(value):
    parts = value.split(":", 3)
    if (len(parts) != 4 or parts[0] not in ("train", "val", "test")
            or not re.fullmatch(r"[A-Za-z0-9_.-]+", parts[1])
            or not re.fullmatch(r"[A-Za-z0-9_-]{1,12}", parts[2]) or not parts[3]):
        raise ValueError("Group must be train|val|test:pose_group:native_id:matrix_dir")
    return {"split": parts[0], "group_id": parts[1], "native_id": parts[2], "matrix_dir": parts[3]}


def labels_from_observed_state(state):
    if type(state) is not int or state not in STATES:
        raise ValueError("Native observed_state must be one of {}".format(sorted(STATES)))
    return dict(zip(CLASSES, [int(bool(state & bit)) for bit in (1, 4, 16, 32)]))


def check_environment(before, after, weather, hour):
    if before.get("status") != "READBACK_MATCH" or before.get("readback_match") is not True:
        raise ValueError("before_api_not_readback_match")
    if after.get("status") not in ("READ_ONLY_GET", "READBACK_MATCH"):
        raise ValueError("after_api_not_successful_get")
    if after.get("status") == "READBACK_MATCH" and after.get("readback_match") is not True:
        raise ValueError("after_api_not_readback_match")
    for name, report in (("before", before), ("after", after)):
        actual = report.get("actual")
        if (not isinstance(actual, dict) or actual.get("weather") != weather
                or type(actual.get("hour")) is not int or actual["hour"] != hour):
            raise ValueError(name + "_environment_weather_hour_mismatch")


def _relative(path, root, label):
    try:
        return path.relative_to(root).as_posix()
    except ValueError as error:
        raise ValueError("{} must remain under output.parent artifact root".format(label)) from error


def qualify_frame(frame, *, group, cell, artifact_root, signal, camera, margin_px=4):
    """Return one exact crop-probe record or a typed, auditable rejection."""
    if frame.get("accepted_for_sample_quota") is False:
        raise SkipFrame("retry_not_accepted_for_quota", "Raw retry frame not accepted for sample quota")
    if frame.get("native_GT_verified") is not True:
        raise SkipFrame("native_gt_not_verified", "Per-frame native_GT_verified must be true")
    if frame.get("requested_native_id") != group["native_id"]:
        raise SkipFrame("requested_native_id_mismatch", "Frame requested another native ID")
    native = frame.get("native_gt")
    if not isinstance(native, dict) or native.get("observed_native_id") != group["native_id"]:
        raise SkipFrame("observed_native_id_mismatch", "Native readback observed another/missing ID")
    try:
        labels = labels_from_observed_state(native.get("observed_state"))
    except ValueError as error:
        raise SkipFrame("unsupported_native_observed_state", str(error)) from error
    relative = frame.get("image_path")
    if (not isinstance(relative, str) or not relative or Path(relative).is_absolute()
            or ".." in Path(relative).parts or Path(relative).suffix.lower() not in (".jpg", ".jpeg")):
        raise SkipFrame("unsafe_or_non_jpeg_image_path", "Expected a safe relative original JPEG path")
    try:
        image = (cell / relative).resolve(strict=True)
        image.relative_to(cell)
        image_relative = _relative(image, artifact_root, "Image")
    except (ValueError, OSError) as error:
        raise SkipFrame("image_missing_or_path_escape", str(error)) from error
    if not image.is_file():
        raise SkipFrame("image_not_regular_file", "Image is not a regular file")
    try:
        payload = image.read_bytes()
    except OSError as error:
        raise SkipFrame("image_read_error", str(error)) from error
    actual_sha = hashlib.sha256(payload).hexdigest()
    expected_sha = frame.get("sha256")
    if not isinstance(expected_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_sha) or actual_sha != expected_sha:
        raise SkipFrame("image_sha256_mismatch", "Original JPEG SHA256 does not match source manifest")
    try:
        width, height = jpeg_dimensions(payload)
    except ValueError as error:
        raise SkipFrame("invalid_jpeg_structure", str(error)) from error
    if (width, height) != (camera["width"], camera["height"]):
        raise SkipFrame("image_camera_resolution_mismatch", "JPEG dimensions differ from fixed camera1")
    if (frame.get("width", width), frame.get("height", height)) != (width, height):
        raise SkipFrame("image_manifest_resolution_mismatch", "JPEG dimensions differ from source frame")
    try:
        ego_xyz, heading, _ = parse_ego_document(frame)
        projection = project_housing(signal, ego_xyz, heading, camera, margin_px=margin_px)
    except (KeyError, TypeError, ValueError) as error:
        raise SkipFrame("invalid_ego_or_projection", str(error)) from error
    if projection.get("valid") is not True:
        raise SkipFrame("invalid_projected_roi", ",".join(projection.get("invalid_reasons", [])))
    try:
        crop = list(validate_crop(projection["bbox_xyxy_integer_clipped"], width, height))
        validate_labels(labels)
    except (KeyError, TypeError, ValueError) as error:
        raise SkipFrame("invalid_crop_or_label_contract", str(error)) from error
    return {"image_path": image_relative, "image_sha256": actual_sha, "crop_xyxy": crop,
            "labels": labels, "split": group["split"], "group_id": group["group_id"],
            "environment": "", "native_gt_verified": True,
            "provenance": {"scope": "research_native_state_and_static_map_oracle_crop_not_visibility_GT",
                "source_image_path_in_cell": relative, "requested_native_id": frame["requested_native_id"],
                "source_compressed_format": frame.get("message_format"),
                "requested_state_not_used_as_label": frame.get("requested_state"),
                "native_observed_id": native["observed_native_id"], "native_observed_state": native["observed_state"],
                "requested_matches_observed_state": frame.get("requested_state") == native["observed_state"],
                "native_gt_evidence": native, "source_header_stamp": frame.get("header_stamp"),
                "source_receipt_ros_stamp": frame.get("receipt_ros_stamp"), "ego": frame.get("ego", frame.get("status")),
                "oracle_projection": projection, "manual_visibility_and_crop_qc_required": True,
                "manual_visibility_and_crop_qc_performed": False}}


def build_documents(group_specs, mgeo, artifact_root, *, margin_px=4):
    if isinstance(margin_px, bool) or not math.isfinite(margin_px) or not 0 <= margin_px <= 32:
        raise ValueError('margin_px must be finite and in 0..32')
    """Read originals and return dataset/skips documents without writing files."""
    artifact_root = Path(artifact_root).resolve(strict=True)
    if not artifact_root.is_dir() or not group_specs:
        raise ValueError("An existing common artifact root and explicit groups are required")
    camera, camera_source = load_fixed_camera()
    mgeo = Path(mgeo).resolve(strict=True)
    mgeo_sha = digest(mgeo)
    groups, group_splits, matrix_paths = [], {}, set()
    for spec in group_specs:
        group = parse_group(spec) if isinstance(spec, str) else dict(spec)
        matrix = Path(group["matrix_dir"]).resolve(strict=True)
        _relative(matrix, artifact_root, "Matrix directory")
        if not matrix.is_dir() or matrix in matrix_paths:
            raise ValueError("Each matrix directory must be unique and exist")
        if group["group_id"] in group_splits and group_splits[group["group_id"]] != group["split"]:
            raise ValueError("One physical pose group cannot cross train/val/test splits")
        group_splits[group["group_id"]] = group["split"]
        matrix_paths.add(matrix)
        group["matrix_dir"] = matrix
        groups.append(group)
    records, group_reports, skipped_cells, skipped_frames = [], [], [], []
    image_splits, path_splits = {}, {}
    for group in groups:
        signal, signal_source = load_signal(mgeo, group["native_id"])
        cell_reports = []
        group_start = len(records)
        group_skip_start = len(skipped_frames)
        for weather, hour in ENVIRONMENTS:
            name = "{}{}".format(weather.lower(), hour)
            cell = group["matrix_dir"] / name
            cell_info = {"group_id": group["group_id"], "split": group["split"], "cell": name,
                         "matrix_dir": _relative(group["matrix_dir"], artifact_root, "Matrix"),
                         "frames_examined": 0, "frames_accepted": 0, "frames_skipped": 0, "status": "skipped"}
            cell_reports.append(cell_info)
            try:
                before_path = group["matrix_dir"] / (name + "_environment_before.json")
                after_path = group["matrix_dir"] / (name + "_environment_after.json")
                before_bytes, after_bytes = before_path.read_bytes(), after_path.read_bytes()
                before, after = json.loads(before_bytes), json.loads(after_bytes)
                check_environment(before, after, weather, hour)
                manifest_path = cell / "manifest.json"
                payload = manifest_path.read_bytes()
                manifest = json.loads(payload)
                frames = manifest.get("frames")
                if not isinstance(frames, list) or manifest.get("frame_count", len(frames)) != len(frames):
                    raise ValueError("cell_manifest_frames_or_count_invalid")
                cell_info["frames_declared"] = len(frames)
                if manifest.get("status") != "complete":
                    cell_info["frames_skipped"] = len(frames)
                    raise ValueError("cell_manifest_not_complete")
                if manifest.get("requested_native_id") != group["native_id"]:
                    cell_info["frames_skipped"] = len(frames)
                    raise ValueError("cell_manifest_native_id_mismatch")
            except (OSError, KeyError, TypeError, ValueError, AttributeError) as error:
                cell_info["reason"] = str(error)
                detail = str(error)
                cell_info["reason_code"] = ("missing_cell_or_environment_evidence" if isinstance(error, FileNotFoundError)
                                             else detail if re.fullmatch(r"[a-z_]+", detail)
                                             else "invalid_cell_or_environment_evidence")
                skipped_cells.append(dict(cell_info))
                continue
            cell_info["status"] = "environment_and_complete_manifest_verified"
            evidence = {"manifest_path": _relative(manifest_path.resolve(), artifact_root, "Manifest"),
                        "manifest_sha256": hashlib.sha256(payload).hexdigest(),
                        "environment_before_sha256": hashlib.sha256(before_bytes).hexdigest(),
                        "environment_after_sha256": hashlib.sha256(after_bytes).hexdigest(),
                        "environment_api_before_after_match": True, "environment_renderer_verified": False,
                        "signal_source": signal_source}
            for frame_index, frame in enumerate(frames):
                cell_info["frames_examined"] += 1
                try:
                    if not isinstance(frame, dict):
                        raise SkipFrame("frame_not_object", "Source frame must be a JSON object")
                    record = qualify_frame(frame, group=group, cell=cell.resolve(), artifact_root=artifact_root,
                                           signal=signal, camera=camera, margin_px=margin_px)
                except SkipFrame as error:
                    cell_info["frames_skipped"] += 1
                    skipped_frames.append({"group_id": group["group_id"], "cell": name, "frame_index": frame_index,
                                           "image_path": frame.get("image_path") if isinstance(frame, dict) else None,
                                           "reason": error.reason, "detail": str(error)})
                    continue
                record["environment"] = "{}{}".format(weather, hour)
                record["provenance"].update(evidence, frame_index=frame_index)
                for mapping, key, label in ((image_splits, record["image_sha256"], "Identical JPEG bytes"),
                                            (path_splits, record["image_path"], "Image path")):
                    if key in mapping and mapping[key] != group["split"]:
                        raise ValueError(label + " cross train/val/test splits; no dataset written")
                    mapping[key] = group["split"]
                records.append(record)
                cell_info["frames_accepted"] += 1
            if (digest(manifest_path) != evidence["manifest_sha256"]
                    or digest(before_path) != evidence["environment_before_sha256"]
                    or digest(after_path) != evidence["environment_after_sha256"]):
                raise ValueError("Capture/environment evidence changed during build; no dataset written")
        subset = records[group_start:]
        group_reports.append({"group_id": group["group_id"], "split": group["split"],
                              "native_id": group["native_id"], "matrix_dir": _relative(group["matrix_dir"], artifact_root, "Matrix"),
                              "accepted_record_count": len(subset), "cells": cell_reports,
                              "frames_examined": sum(cell["frames_examined"] for cell in cell_reports),
                              "known_frames_skipped": sum(cell["frames_skipped"] for cell in cell_reports),
                              "skipped_cell_count": sum(cell["status"] == "skipped" for cell in cell_reports),
                              "skipped_frame_reason_counts": dict(Counter(row["reason"] for row in skipped_frames[group_skip_start:])),
                              "accepted_environment_counts": dict(Counter(row["environment"] for row in subset)),
                              "accepted_observed_state_counts": dict(Counter(str(row["provenance"]["native_observed_state"]) for row in subset))})
    if digest(mgeo) != mgeo_sha:
        raise ValueError("MGeo changed during build; no dataset written")
    train_rows = [row for row in records if row["split"] == "train"]
    train_positive = {key: sum(row["labels"][key] for row in train_rows) for key in CLASSES}
    summary = {"record_count": len(records), "unique_image_sha256_count": len(image_splits),
               "group_count": len(group_splits), "matrix_count": len(groups),
               "expected_cell_count": len(groups) * len(ENVIRONMENTS),
               "qualified_cell_count": sum(cell["status"] != "skipped" for group in group_reports for cell in group["cells"]),
               "split_counts": dict(Counter(row["split"] for row in records)),
               "environment_counts": dict(Counter(row["environment"] for row in records)),
               "skipped_cell_count": len(skipped_cells), "skipped_frame_count": len(skipped_frames),
               "skipped_cell_reason_counts": dict(Counter(row["reason_code"] for row in skipped_cells)),
               "skipped_frame_reason_counts": dict(Counter(row["reason"] for row in skipped_frames)),
               "whole_cell_frames_skipped": sum(row["frames_skipped"] for row in skipped_cells),
               "total_known_skipped_frames": len(skipped_frames) + sum(row["frames_skipped"] for row in skipped_cells),
               "skipped_cells_with_unknown_frame_count": sum("frames_declared" not in row for row in skipped_cells),
               "structural_trainer_requirements_before_manual_QC": {
                   "all_splits_nonempty": all(any(row["split"] == split for row in records) for split in ("train", "val", "test")),
                   "at_least_two_train_pose_groups": len({row["group_id"] for row in train_rows}) >= 2,
                   "train_positive_counts": train_positive,
                   "each_train_bit_has_positive_and_negative": bool(train_rows) and all(0 < value < len(train_rows)
                                                                                            for value in train_positive.values())}}
    dataset = {"schema_version": 1,
               "scope": "research_native_GT_state_plus_static_map_oracle_crop_not_production_accuracy",
               "manual_visibility_and_crop_qc_required": True, "manual_visibility_and_crop_qc_performed": False,
               "label_source": "native_gt.observed_state_never_requested_state",
               "limitations": ["Native state does not prove lamp visibility or visible-head identity",
                               "Approximate projected crop requires manual image/QC review",
                               "Environment API readback does not independently verify renderer appearance"],
               "provenance": {"builder_source_sha256": digest(__file__), "margin_px": margin_px, "mgeo_path": str(mgeo),
                              "mgeo_sha256": mgeo_sha, "camera": camera_source,
                              "projection_source_sha256": digest(Path(__file__).with_name("project_traffic_light_housing.py"))},
               "summary": summary, "groups": group_reports, "records": records}
    skipped = {"schema_version": 1, "scope": dataset["scope"], "summary": summary,
               "groups": group_reports, "skipped_cells": skipped_cells, "skipped_frames": skipped_frames}
    return dataset, skipped


def build_dataset(group_specs, mgeo, output, *, margin_px=4):
    output = Path(output).absolute()
    if output.suffix != ".json":
        raise ValueError("Output must be a new .json file under the common artifact root")
    sidecar = output.with_suffix(".skipped.json")
    if any(path.exists() or path.is_symlink() for path in (output, sidecar)):
        raise FileExistsError("Dataset and skipped sidecar must both be fresh files")
    dataset, skipped = build_documents(group_specs, mgeo, output.parent, margin_px=margin_px)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(dataset, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    with sidecar.open("x", encoding="utf-8") as stream:
        json.dump(skipped, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return dataset, sidecar


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--group", action="append", required=True, help="split:pose_group:native_id:matrix_dir")
    parser.add_argument("--mgeo", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New JSON at common artifact root containing all images")
    parser.add_argument("--margin-px", type=float, default=4, help="Explicit oracle-ROI margin; always requires new visual QC")
    args = parser.parse_args(argv)
    dataset, sidecar = build_dataset(args.group, args.mgeo, args.output, margin_px=args.margin_px)
    print(json.dumps({"dataset": str(args.output.absolute()), "skipped": str(sidecar), "summary": dataset["summary"]}))


if __name__ == "__main__":
    main()
