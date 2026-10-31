#!/usr/bin/env python3
"""Offline YOLO -> target identity -> HSV/CNN -> current policy diagnostics.

Never contacts ROS/MORAI or trains/changes a model. Archived raw Ego is a
development oracle converted once, NOT an operating dependency. Map projection
is not its own identity GT: approved image-bound housing boxes are required.
Native-state agreement is not pixel-visibility accuracy or traffic-rule safety.
Sparse stationary photos do not support smoother/transition-delay evaluation.
Default evaluation retains historical JPEG-SHA deduplication. The explicit
sequence option keeps timestamped events and isolates their own native brackets.
Reviewed named-head identity annotations are not full-frame detector-training
labels or manual lamp-state ground truth.
"""
import argparse
from collections import defaultdict
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "base_ws/src/perception/traffic_light_detection"
sys.path.insert(0, str(PACKAGE / "scripts"))

from light_color_utils import classify_crop, pick_ego_lane_box
from signal_phase_policy import SignalPhasePolicy
from traffic_light_map_selector import (MISSION_GROUP_IDS, RigidTransform, associate_boxes,
    load_camera, load_geometry, project_heads, quaternion_matrix, rotate, DEFAULT_SELECTOR_CONFIG,
    SELECTOR_CONTRACT_VERSION)

CLASSES = ("red", "yellow", "green", "left_arrow")
CLASS_TUPLES = (("red", "circle", None), ("yellow", "circle", None),
                ("green", "circle", None), ("green", "arrow", True))
NATIVE_BITS = (1, 4, 16, 32)
KNOWN_NATIVE_CODES = frozenset((1, 4, 5, 16, 20, 32, 33, 36, 48))
IDENTITY_ONLY_SCOPE = "named_housing_identity_evaluation_only_not_detector_training_GT"
BASELINE_SHA256 = "2bb73720568a975c94d79a54cc097a0a4b70ec7c8de23552b4d0e71112160818"
DEFAULT_WEIGHTS = PACKAGE / "weights/best.pt"
DEFAULT_GEOMETRY = PACKAGE / "config/traffic_light_map_geometry.json"
DEFAULT_SENSOR = ROOT / "configs/sensors/Sensor_competition_local.json"
DEFAULT_POLICY = PACKAGE / "config/traffic_signal_profiles.yaml"
DEFAULT_BVP_CONFIG = ROOT / "planning_ws/src/autoware_launch/config/ros1/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light.param.yaml"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def bits_from_tuples(values):
    values = {tuple(value) for value in values}
    return tuple(int(value in values) for value in CLASS_TUPLES)


def tuples_from_bits(bits):
    return [value for value, present in zip(CLASS_TUPLES, bits) if present]


def native_bits(code):
    if isinstance(code, bool) or not isinstance(code, int) or code not in KNOWN_NATIVE_CODES:
        return None
    return tuple(int(bool(code & flag)) for flag in NATIVE_BITS)


def stamp_seconds(value):
    if not isinstance(value, dict):
        return None
    seconds = value.get("secs", value.get("sec"))
    nanoseconds = value.get("nsecs", value.get("nanosec"))
    if isinstance(seconds, bool) or isinstance(nanoseconds, bool):
        return None
    if not isinstance(seconds, int) or not isinstance(nanoseconds, int) or not 0 <= nanoseconds < 10**9:
        return None
    return seconds+nanoseconds/1e9 if seconds > 0 else None


def utc_seconds(value):
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        return parsed.timestamp() if parsed.tzinfo is not None else None
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _actual_native(readback, native_id):
    """Only actual exact-ID modern responses qualify, never request/label fields."""
    if not isinstance(readback, dict) or readback.get("readback_success") is not True:
        return None
    if readback.get("protocol") != "modern" or readback.get("requested_id") != native_id:
        return None
    observations = readback.get("observations", [])
    if len(observations) != 1:
        return None
    actual = observations[0]
    if (actual.get("tl_id") != native_id or actual.get("identity_matches_request") is not True
            or actual.get("color_field_present") is not True
            or actual.get("color_usable_for_gt") is not True
            or native_bits(actual.get("tl_color")) is None):
        return None
    timestamp = utc_seconds(readback.get("received_at_utc"))
    return (actual["tl_color"], timestamp) if timestamp is not None else None


def native_head_reference(row, native_id, max_bracket_sec=5.):
    """Read selected head only. Synchronized neighbors never inherit its GT."""
    provenance = row.get("provenance", {})
    before, after = row.get("native_before"), row.get("native_after")
    evidence = row.get("native_gt") or provenance.get("native_gt_evidence")
    if before is None and isinstance(evidence, dict):
        if evidence.get("jpeg_receipt_bracketed") is not True:
            return {"known": False, "reason": "JPEG_not_explicitly_bracketed"}
        before = [evidence.get("before", {}).get("readback", {})]
        after = [evidence.get("after", {}).get("readback", {})]
    if not isinstance(before, list) or not isinstance(after, list):
        return {"known": False, "reason": "selected_head_has_no_native_brackets"}
    pre = [_actual_native(readback, native_id) for readback in before
           if isinstance(readback, dict) and readback.get("requested_id") == native_id]
    post = [_actual_native(readback, native_id) for readback in after
            if isinstance(readback, dict) and readback.get("requested_id") == native_id]
    if len(pre) != 1 or len(post) != 1 or pre[0] is None or post[0] is None:
        return {"known": False, "reason": "missing_invalid_or_duplicate_exact_head_readback"}
    if pre[0][0] != post[0][0]:
        return {"known": False, "reason": "native_state_changed_across_JPEG"}
    image_time = stamp_seconds(row.get("header_stamp") or provenance.get("source_header_stamp"))
    if image_time is None or not pre[0][1] <= image_time <= post[0][1]:
        return {"known": False, "reason": "native_times_do_not_bracket_JPEG_stamp"}
    if not 0 <= post[0][1]-pre[0][1] <= max_bracket_sec:
        return {"known": False, "reason": "native_bracket_exceeds_limit"}
    return {"known": True, "native_id": native_id, "state": pre[0][0],
            "bits": list(native_bits(pre[0][0])), "reason": "exact_head_actual_before_after_agree",
            "bracket_sec": post[0][1]-pre[0][1], "pixel_visibility_verified": False}


def intersection_over_union(first, second):
    overlap = max(0., min(first[2], second[2])-max(first[0], second[0])) * max(0., min(first[3], second[3])-max(first[1], second[1]))
    area = max(0., first[2]-first[0])*max(0., first[3]-first[1])
    other = max(0., second[2]-second[0])*max(0., second[3]-second[1])
    return overlap/(area+other-overlap) if area+other-overlap > 0 else 0.


def manual_identity(selected_xyxy, annotations, min_iou=.25):
    """Conservative approved-box identity; map estimates are not a fallback GT."""
    if selected_xyxy is None:
        return {"known": False, "reason": "no_selected_detection"}
    matches = []
    cx, cy = (selected_xyxy[0]+selected_xyxy[2])/2, (selected_xyxy[1]+selected_xyxy[3])/2
    for annotation in annotations:
        if (annotation.get("box_qc_accepted") is not True
                or annotation.get("manual_direct_visual_qc_accepted") is not True
                or annotation.get("proposal_only") is not False or not annotation.get("native_id")):
            continue
        bbox = annotation.get("xyxy", [])
        if len(bbox) != 4 or not all(math.isfinite(float(v)) for v in bbox):
            continue
        iou = intersection_over_union(selected_xyxy, bbox)
        if iou >= min_iou and bbox[0]-2 <= cx <= bbox[2]+2 and bbox[1]-2 <= cy <= bbox[3]+2:
            matches.append((iou, annotation["native_id"]))
    ids = {identifier for _, identifier in matches}
    if len(ids) != 1:
        return {"known": False, "reason": "manual_identity_ambiguous" if ids else "no_approved_identity_match"}
    return {"known": True, "native_id": next(iter(ids)), "iou": max(iou for iou, _ in matches),
            "reason": "approved_2D_box_match_not_independent_human_GT"}


def development_camera_transform(ego, camera):
    """Archived RAW Ego only. Runtime TF/localization does NOT use this offset."""
    ego = ego.get("ego", ego)
    position = ego.get("position_xyz_m", ego.get("position"))
    if isinstance(position, dict):
        position = [position[axis] for axis in "xyz"]
    if not isinstance(position, (tuple, list)) or len(position) != 3:
        raise ValueError("measured development Ego xyz is required")
    heading = ego.get("heading_degrees", ego.get("heading"))
    values = [float(value) for value in list(position)+[heading]]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("invalid development Ego pose")
    yaw = math.radians(values[3]-1.347)
    world_to_body = quaternion_matrix((0, 0, -math.sin(yaw/2), math.cos(yaw/2)))
    body_to_camera = quaternion_matrix(tuple(-v for v in camera.quaternion[:3])+(camera.quaternion[3],))
    rotation = tuple(tuple(sum(body_to_camera[i][k]*world_to_body[k][j] for k in range(3))
                           for j in range(3)) for i in range(3))
    translated = rotate(world_to_body, tuple(-value for value in values[:3]))
    translation = rotate(body_to_camera, tuple(translated[i]-camera.position[i] for i in range(3)))
    return RigidTransform(rotation, translation)


def permission_proxy(tuples, group_id, planner_flags=None):
    """Current route-program proxy, not legal entry permission/planner emulation."""
    bits = bits_from_tuples(tuples)
    _red, yellow, green, left = bits
    flags = planner_flags or {"yellow_light_is_go": True, "competition_fail_open": True}
    if group_id not in MISSION_GROUP_IDS:
        return "non_mission"
    if not any(bits):
        return "go_phase_proxy" if flags["competition_fail_open"] else "stop_phase_proxy"
    if yellow and flags["yellow_light_is_go"]:
        return "go_phase_proxy"
    if group_id == 201326:
        return "go_phase_proxy" if left else "stop_phase_proxy"
    if green:
        return "go_phase_proxy"
    if yellow:
        return "yellow_requires_motion_context"
    return "stop_phase_proxy"


def policy_diagnostic(policy, raw_tuples, raw_known, reference, group_id, planner_flags=None):
    corrected, reason = policy.apply([tuple(value) for value in raw_tuples] if raw_known else [], group_id)
    output = {"corrected_tuples": corrected, "reason": reason,
              "permission_proxy": permission_proxy(corrected, group_id, planner_flags),
              "temporal_smoother_evaluated": False, "formal_traffic_rule_evaluation": False}
    if reference.get("known"):
        expected, expected_reason = policy.apply(tuples_from_bits(reference["bits"]), group_id)
        expected_permission = permission_proxy(expected, group_id, planner_flags)
        output.update(reference_corrected_tuples=expected, reference_policy_reason=expected_reason,
            corrected_exact=bits_from_tuples(corrected) == bits_from_tuples(expected),
            reference_permission_proxy=expected_permission,
            permission_proxy_match=output["permission_proxy"] == expected_permission,
            false_go_proxy=(output["permission_proxy"] == "go_phase_proxy" and expected_permission == "stop_phase_proxy"))
    return output


def _indexed_rows(document):
    if "records" in document:
        if document.get("status") not in (None, "complete"):
            raise ValueError("incomplete/failed campaign is not an evaluation dataset")
        yield from enumerate(document["records"])
    elif "frames" in document:
        for index, row in enumerate(document["frames"]):
            if row.get("accepted_for_sample_quota") is True:
                yield index, dict(row, requested_native_id=row.get("requested_native_id", document.get("requested_native_id")))
    else:
        raise ValueError("dataset must contain records or accepted frames")


def _rows(document):
    return [row for _, row in _indexed_rows(document)]


def load_rows(path):
    path = Path(path).expanduser().resolve(strict=True)
    document = json.loads(path.read_text())
    rows = []
    for row_index, source in _indexed_rows(document):
        if source.get("annotation_kind") == "manual_background_no_lamps":
            continue
        row = dict(source)
        # Bulk-reviewed documents may provide defaults, but a per-frame refusal wins.
        for key in ("annotation_complete", "identity_annotations_reviewed", "annotation_scope"):
            if key not in row and key in document:
                row[key] = document[key]
        row["image_sha256"] = row.get("image_sha256", row.get("sha256"))
        if not isinstance(row["image_sha256"], str) or len(row["image_sha256"]) != 64:
            raise ValueError("every input record needs its original JPEG SHA256")
        row["_image_absolute"] = str((path.parent/row["image_path"]).resolve(strict=True))
        row["_input_document"] = str(path)
        row["_input_row_index"] = row_index
        rows.append(row)
    return rows


def index_rows(rows):
    index = defaultdict(list)
    for row in rows:
        index[row["image_sha256"]].append(row)
    return index


def image_stamp_ns(row):
    stamp = row.get("header_stamp") or row.get("provenance", {}).get("source_header_stamp")
    if not isinstance(stamp, dict):
        return None
    sec, nsec = stamp.get("secs", stamp.get("sec")), stamp.get("nsecs", stamp.get("nanosec"))
    if type(sec) is not int or type(nsec) is not int or sec <= 0 or not 0 <= nsec < 10**9:
        return None
    return sec*10**9+nsec


def sequence_frame_key(row):
    stamp = image_stamp_ns(row)
    document, index = row.get("_input_document"), row.get("_input_row_index")
    pose, environment = row.get("group_id"), row.get("environment")
    if (stamp is None or not isinstance(document, str) or not document
            or type(index) is not int or index < 0
            or not isinstance(pose, str) or not pose or not isinstance(environment, str) or not environment):
        raise ValueError("sequence frames require input document/original row index/pose/environment/exact positive image stamp")
    return {"input_document": document, "input_row_index": index, "source_stamp_ns": stamp,
            "pose_group": pose, "environment": environment}


def evaluation_rows(source_rows, preserve_sequence_frames=False, split=None):
    """Default keeps the historical SHA dedup; sequence mode preserves every event."""
    selected, seen_hashes, seen_stamps, last_by_sequence = [], set(), set(), {}
    for row in source_rows:
        if split and row.get("split") != split:
            continue
        if preserve_sequence_frames:
            key = sequence_frame_key(row)
            # A row index must not disguise duplicate camera timestamps as new evidence.
            stamp_key = (key["input_document"], key["source_stamp_ns"])
            if stamp_key in seen_stamps:
                raise ValueError("ambiguous repeated image stamp in sequence input")
            seen_stamps.add(stamp_key)
            sequence = (key["input_document"], key["pose_group"], key["environment"])
            if key["source_stamp_ns"] <= last_by_sequence.get(sequence, -1):
                raise ValueError("sequence image stamps regress in input record order")
            last_by_sequence[sequence] = key["source_stamp_ns"]
        else:
            if row["image_sha256"] in seen_hashes:
                continue
            seen_hashes.add(row["image_sha256"])
        selected.append(row)
    return selected


def frame_native_evidence(row, evidence_index, preserve_sequence_frames=False):
    if not preserve_sequence_frames:
        return evidence_index.get(row["image_sha256"], [])
    stamp = image_stamp_ns(row)
    if stamp is None:
        raise ValueError("sequence native evidence requires an exact image stamp")
    # The caller supplies external-only rows in sequence mode. Never borrow a
    # neighbor's native state merely because the static JPEG bytes are identical.
    return [row]+[candidate for candidate in evidence_index.get(row["image_sha256"], [])
                  if image_stamp_ns(candidate) == stamp and candidate is not row]


def identity_annotation_contract(annotation_rows):
    if not annotation_rows:
        return {"reviewed_for_identity": False, "full_frame_detector_training_GT": False, "scope": "none"}
    partial = False
    for row in annotation_rows:
        if row.get("annotation_complete") is True:
            continue
        if (row.get("annotation_complete") is False and row.get("identity_annotations_reviewed") is True
                and row.get("annotation_scope") == IDENTITY_ONLY_SCOPE):
            partial = True
            continue
        return {"reviewed_for_identity": False, "full_frame_detector_training_GT": False,
                "scope": "unapproved_or_partial_proposals_rejected"}
    return {"reviewed_for_identity": True, "full_frame_detector_training_GT": not partial,
            "scope": IDENTITY_ONLY_SCOPE if partial else "complete_housing_annotations"}


def frame_group(row, geometry, evidence_rows=()):
    group = row.get("group_regulatory_id")
    if group is not None:
        group = int(group)
    else:
        identifier = row.get("requested_native_id") or row.get("provenance", {}).get("requested_native_id")
        matches = [head.get("group_id") for head in geometry["heads"] if head["id"] == identifier]
        group = matches[0] if len(matches) == 1 else None
    if group not in MISSION_GROUP_IDS:
        matches = set()
        for source in evidence_rows:
            try:
                matches.add(frame_group(source, geometry))
            except ValueError:
                pass
        if len(matches) == 1:
            return next(iter(matches))
        raise ValueError("frame has no unambiguous explicit four-mission route identity")
    return group


def head_reference(evidence_rows, native_id):
    references = [native_head_reference(row, native_id) for row in evidence_rows]
    known = [reference for reference in references if reference.get("known")]
    if any(reference.get("reason") == "native_state_changed_across_JPEG" for reference in references):
        return {"known": False, "reason": "native_state_changed_in_available_evidence"}
    if not known:
        return {"known": False, "reason": "no_valid_exact_head_native_evidence",
                "evidence_reasons": sorted({r["reason"] for r in references})}
    if len({tuple(reference["bits"]) for reference in known}) != 1:
        return {"known": False, "reason": "conflicting_native_evidence"}
    return known[0]


def make_box(xyxy, confidence, width, height):
    x1, y1, x2, y2 = [float(value) for value in xyxy]
    return ((x1+x2)/(2*width), (y1+y2)/(2*height), (x2-x1)/width,
            (y2-y1)/height, (x1, y1, x2, y2, float(confidence)))


def summarize(results):
    """Every accuracy denominator is explicit; unknown/wrong-target never succeeds."""
    summary = {}
    for mode in ("legacy_center", "map_association"):
        selected = [row[mode] for row in results]
        identity_known = [row for row in selected if row["identity"].get("known")]
        wrong = [row for row in identity_known if not row["target_identity_correct"]]
        stats = {"frames": len(results), "selected": sum(row["selected_bbox_xyxy"] is not None for row in selected),
                 "identity_known": len(identity_known), "target_identity_correct": sum(row["target_identity_correct"] is True for row in selected),
                 "identity_unknown": sum(row["selected_bbox_xyxy"] is not None and not row["identity"].get("known") for row in selected),
                 "wrong_mission": len(wrong), "wrong_mission_denominator_identity_known": len(identity_known),
                 "wrong_mission_rate_among_identity_known": len(wrong)/len(identity_known) if identity_known else None,
                 "claimed_map_head_disagrees_with_manual_identity": sum(
                     row.get("map_head_identity_agrees_manual_box") is False for row in selected),
                 "selection_missing": sum(row["selected_bbox_xyxy"] is None for row in selected),
                 "selection_failed_despite_target_housing_detection": sum(
                     original.get("target_housing_detected", False) and row["target_identity_correct"] is not True
                     for original, row in zip(results, selected)),
                 "classifiers": {}}
        for classifier in ("hsv", "cnn"):
            available = [row for row in selected if classifier in row["classifiers"]]
            eligible = [row for row in available if row["target_identity_correct"] is True and row["reference"].get("known")]
            known = [row for row in eligible if row["classifiers"][classifier]["known"]]
            exact = sum(row["classifiers"][classifier]["bits"] == row["reference"]["bits"] for row in known)
            positive = [row for row in eligible if row["reference"]["bits"][3]]
            negative = [row for row in eligible if not row["reference"]["bits"][3]]
            arrow_tp = sum(row["classifiers"][classifier]["known"] and bool(row["classifiers"][classifier]["bits"][3]) for row in positive)
            arrow_fp = sum(row["classifiers"][classifier]["known"] and bool(row["classifiers"][classifier]["bits"][3]) for row in negative)
            stats["classifiers"][classifier] = {
                "evaluated": bool(available), "frames_with_classifier": len(available),
                "all_frame_raw_known": sum(row["classifiers"][classifier]["known"] for row in available),
                "all_frame_raw_unknown_or_no_selection": sum(not row["classifiers"][classifier]["known"] for row in available),
                "native_reference_eligible_correct_identity": len(eligible), "raw_known": len(known),
                "unknown": len(eligible)-len(known), "known_coverage": len(known)/len(eligible) if eligible else None,
                "raw_4bit_exact_correct": exact, "raw_4bit_exact_among_known": exact/len(known) if known else None,
                "raw_4bit_exact_including_unknown_as_unsuccessful": exact/len(eligible) if eligible else None,
                "arrow_true_positive": arrow_tp, "arrow_positive_native_denominator": len(positive),
                "arrow_recall_including_unknown_as_miss": arrow_tp/len(positive) if positive else None,
                "arrow_false_positive": arrow_fp, "arrow_negative_native_denominator": len(negative),
                "phase_corrected_exact": sum(row["classifiers"][classifier]["policy"].get("corrected_exact", False) for row in eligible),
                "permission_proxy_match": sum(row["classifiers"][classifier]["policy"].get("permission_proxy_match", False) for row in eligible),
                "false_go_proxy": sum(row["classifiers"][classifier]["policy"].get("false_go_proxy", False) for row in eligible),
                "end_to_end_raw_exact_successes": exact, "end_to_end_frame_denominator": len(available),
                "native_agreement_is_not_pixel_visibility_accuracy": True}
        summary[mode] = stats
    housing = [head for row in results for head in row.get("mission_head_detection", [])]
    summary["shared_detector"] = {"manually_approved_mission_head_count": len(housing),
        "detected_head_count": sum(head["detected"] for head in housing),
        "head_recall_at_manual_identity_gate": sum(head["detected"] for head in housing)/len(housing) if housing else None,
        "frames_with_manually_approved_mission_head": sum(bool(row.get("mission_head_detection")) for row in results),
        "frames_with_target_housing_detection": sum(row.get("target_housing_detected", False) for row in results),
        "named_identity_only_annotation_frames": sum(row.get("identity_annotation_contract", {}).get("scope") == IDENTITY_ONLY_SCOPE for row in results),
        "full_frame_annotation_frames": sum(row.get("identity_annotation_contract", {}).get("full_frame_detector_training_GT", False) for row in results),
        "not_a_full_frame_detector_precision_or_training_label_evaluation": True}
    return summary


def evaluate_frame(row, image, boxes, geometry, camera, policy, cnn, evidence_rows, annotation_rows,
                   cnn_margin_px=12, planner_flags=None):
    height, width = image.shape[:2]
    group = frame_group(row, geometry, evidence_rows)
    ego = row.get("ego") or row.get("provenance", {}).get("ego")
    rois, rejected = project_heads(geometry, camera, development_camera_transform(ego, camera), group)
    annotation_contract = identity_annotation_contract(annotation_rows)
    annotations = ([annotation for source in annotation_rows for annotation in source.get("boxes", [])]
                   if annotation_contract["reviewed_for_identity"] else [])
    id_groups = {head["id"]: head.get("group_id") for head in geometry["heads"]}
    detected_ids = {manual_identity(box[4][:4], annotations).get("native_id") for box in boxes}
    target_annotations = {annotation["native_id"]: annotation for annotation in annotations
        if annotation.get("box_qc_accepted") is True and annotation.get("manual_direct_visual_qc_accepted") is True
        and annotation.get("proposal_only") is False and id_groups.get(annotation.get("native_id")) == group}
    mission_detection = [{"native_id": identifier, "manual_bbox_xyxy": annotation["xyxy"],
                          "detected": identifier in detected_ids}
                         for identifier, annotation in target_annotations.items()]
    result = {"image_sha256": row["image_sha256"], "image_path": row["image_path"],
              "environment": row.get("environment"), "split": row.get("split"), "pose_group": row.get("group_id"),
              "group_regulatory_id": group, "detections": [list(box[4]) for box in boxes],
              "source_stamp_ns": image_stamp_ns(row), "identity_annotation_contract": annotation_contract,
              "natural_program_capture": row.get("natural_program_capture"),
              "requested_signal_override": row.get("requested_signal_override"),
              "capture_native_bracket_stable": row.get("native_bracket_stable"),
              "development_pose_oracle_not_operational_localization": True,
              "additional_raw_Ego_heading_correction_applied_once_degrees": -1.347,
              "mission_head_detection": mission_detection,
              "target_housing_detected": any(head["detected"] for head in mission_detection),
              "projected_heads": rois, "rejected_projected_heads": rejected}
    map_best, map_trace = associate_boxes(boxes, rois, group)
    selections = {"legacy_center": (pick_ego_lane_box(boxes), {"reason": "actual_legacy_center_picker"}),
                  "map_association": (map_best, map_trace)}
    cache = {}
    for mode, (best, trace) in selections.items():
        bbox = list(best[4][:4]) if best is not None else None
        identity = manual_identity(bbox, annotations)
        identifier = identity.get("native_id")
        target_correct = id_groups.get(identifier) == group if identity.get("known") else None
        reference = head_reference(evidence_rows, identifier) if identifier else {"known": False, "reason": "selected_identity_unknown"}
        # Wrong-target native bits are retained diagnostically, but never scored as success.
        output = {"selected_bbox_xyxy": bbox, "selection": trace, "identity": identity,
                  "target_identity_correct": target_correct, "reference": reference, "classifiers": {}}
        if mode == "map_association" and identity.get("known"):
            output["map_head_identity_agrees_manual_box"] = trace.get("head_id") == identifier
        for name in ("hsv", "cnn"):
            if name == "cnn" and cnn is None:
                continue
            margin = cnn_margin_px if name == "cnn" else 0
            coordinates = None if bbox is None else [max(0, int(bbox[0])-margin), max(0, int(bbox[1])-margin),
                min(width, int(bbox[2])+margin), min(height, int(bbox[3])+margin)]
            key = (name, tuple(coordinates) if coordinates else None)
            if key not in cache:
                started = time.perf_counter()
                if coordinates is None or coordinates[2] <= coordinates[0] or coordinates[3] <= coordinates[1]:
                    observation = {"known": False, "tuples": [], "bits": None, "reason": "no_selected_crop"}
                else:
                    crop = image[coordinates[1]:coordinates[3], coordinates[0]:coordinates[2]]
                    if name == "hsv":
                        tuples, counts = classify_crop(crop)
                        observation = {"known": True, "tuples": tuples, "bits": list(bits_from_tuples(tuples)),
                                       "counts": counts, "reason": "actual_HSV_classifier"}
                    else:
                        actual = cnn.classify_bgr(crop)
                        observation = actual.to_dict()
                        observation["partial_bits"] = observation.get("bits")
                        observation["bits"] = list(bits_from_tuples(actual.tuples)) if actual.known else None
                observation.update(crop_xyxy=coordinates, crop_margin_px=margin,
                                   classify_wall_ms=(time.perf_counter()-started)*1000)
                cache[key] = observation
            observation = dict(cache[key])
            observation["policy"] = policy_diagnostic(policy, observation["tuples"], observation["known"], reference, group, planner_flags)
            output["classifiers"][name] = observation
        result[mode] = output
    return result


def validate_output(path):
    path = Path(path).expanduser().resolve()
    allowed = (ROOT / "artifacts/evaluation").resolve()
    if allowed not in path.parents or path.suffix != ".json" or path.exists():
        raise ValueError("output must be a NEW JSON under repository artifacts/evaluation")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--native-evidence", action="append", default=[], type=Path)
    parser.add_argument("--housing-qc", action="append", default=[], type=Path)
    parser.add_argument("--detector-weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--allow-candidate-detector", action="store_true")
    parser.add_argument("--cnn-checkpoint", type=Path)
    parser.add_argument("--device", default="cpu", help="YOLO device: cpu or explicit GPU index")
    parser.add_argument("--cnn-device", default="cpu")
    parser.add_argument("--cnn-min-bit-confidence", type=float, default=.6)
    parser.add_argument("--cnn-margin-px", type=int, default=12)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--confidence", type=float, default=.25)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--split")
    parser.add_argument("--preserve-sequence-frames", action="store_true",
                        help="Keep exact timestamped input events; do not SHA-deduplicate or borrow another frame's native evidence")
    parser.add_argument("--geometry", type=Path, default=DEFAULT_GEOMETRY)
    parser.add_argument("--sensor", type=Path, default=DEFAULT_SENSOR)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--planner-signal-config", type=Path, default=DEFAULT_BVP_CONFIG)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        output = validate_output(args.output)
        source_paths = [args.dataset, args.geometry, args.sensor, args.policy, args.planner_signal_config,
                        args.detector_weights, Path(__file__), PACKAGE/"scripts/traffic_light_map_selector.py",
                        PACKAGE/"scripts/traffic_light_cnn_classifier.py", PACKAGE/"scripts/light_color_utils.py",
                        PACKAGE/"scripts/signal_phase_policy.py"] + args.native_evidence + args.housing_qc
        if args.cnn_checkpoint:
            source_paths.append(args.cnn_checkpoint)
        initial_source_hashes = {str(path.resolve()): sha256(path) for path in source_paths}
        if (not 0 < args.confidence <= 1 or not 0 <= args.cnn_margin_px <= 64
                or not 320 <= args.imgsz <= 1920 or args.imgsz % 32):
            raise ValueError("invalid confidence/crop margin/inference resolution")
        if args.limit is not None and args.limit < 1:
            raise ValueError("limit must be positive")
        weights_digest = sha256(args.detector_weights)
        if weights_digest != BASELINE_SHA256 and not args.allow_candidate_detector:
            raise ValueError("non-baseline detector requires explicit --allow-candidate-detector")
        geometry, camera = load_geometry(args.geometry), load_camera(args.sensor)
        policy = SignalPhasePolicy.from_yaml(args.policy)
        import yaml
        planner_config = yaml.safe_load(args.planner_signal_config.read_text())
        planner_flags = {key: planner_config.get("traffic_light/"+key)
                         for key in ("yellow_light_is_go", "competition_fail_open")}
        if any(type(value) is not bool for value in planner_flags.values()):
            raise ValueError("planner proxy requires explicit boolean yellow/fail-open flags")
        source_rows = load_rows(args.dataset)
        external_evidence = [row for path in args.native_evidence for row in load_rows(path)]
        evidence = index_rows(external_evidence if args.preserve_sequence_frames else source_rows+external_evidence)
        annotations = index_rows([row for path in args.housing_qc for row in load_rows(path)])
        rows = evaluation_rows(source_rows, args.preserve_sequence_frames, args.split)
        for row in rows:
            frame_group(row, geometry, frame_native_evidence(row, evidence, args.preserve_sequence_frames))
        if args.limit:
            rows = rows[:args.limit]
        if not rows:
            raise ValueError("no evaluable frames selected")
        cnn = None
        cnn_metadata = None
        if args.cnn_checkpoint:
            from traffic_light_cnn_classifier import initialize_classifier
            startup = initialize_classifier(args.cnn_checkpoint, device=args.cnn_device,
                min_bit_confidence=args.cnn_min_bit_confidence, warmup=True)
            if not startup.ready:
                raise ValueError("explicit CNN unavailable: "+startup.reason)
            cnn, cnn_metadata = startup.classifier, startup.metadata
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    os.environ["YOLO_OFFLINE"] = "true"
    import cv2
    from ultralytics import YOLO
    model = YOLO(str(args.detector_weights))
    if len(model.names) != 1 or (isinstance(model.names, dict) and 0 not in model.names):
        raise ValueError("pipeline comparison requires the explicit one-class housing detector contract")
    records, timings = [], []
    for index, row in enumerate(rows):
        image_path = Path(row["_image_absolute"])
        if sha256(image_path) != row["image_sha256"]:
            raise ValueError("original JPEG SHA256 changed: "+str(image_path))
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (camera.height, camera.width):
            raise ValueError("JPEG dimensions differ from explicit camera contract")
        started = time.perf_counter()
        prediction = model.predict(image, conf=args.confidence, device=args.device,
                                   imgsz=args.imgsz, verbose=False, save=False)[0]
        boxes = [make_box(box.xyxy[0].tolist(), float(box.conf[0]), camera.width, camera.height)
                 for box in prediction.boxes if int(box.cls[0]) == 0]
        if "cuda" in str(model.device):
            import torch
            torch.cuda.synchronize(model.device)
        timings.append((time.perf_counter()-started)*1000)
        record = evaluate_frame(row, image, boxes, geometry, camera, policy, cnn,
            frame_native_evidence(row, evidence, args.preserve_sequence_frames),
            annotations[row["image_sha256"]], args.cnn_margin_px, planner_flags)
        if args.preserve_sequence_frames:
            record["sequence_frame_key"] = sequence_frame_key(row)
        record["detector_wall_ms"] = timings[-1]
        records.append(record)
        if sha256(image_path) != row["image_sha256"]:
            raise ValueError("original JPEG changed during frame evaluation")
        print(json.dumps({"evaluated": index+1, "total": len(rows), "image_sha256": row["image_sha256"]}), flush=True)
    if sha256(args.detector_weights) != weights_digest:
        raise ValueError("detector source changed during evaluation")
    if any(sha256(path) != digest for path, digest in initial_source_hashes.items()):
        raise ValueError("source document/config/model changed during evaluation")
    report = {"schema_version": 1, "scope": "offline_stage_separated_research_not_deployment_accuracy",
        "classes_in_bit_order": list(CLASSES), "native_state_bit_masks": dict(zip(CLASSES, NATIVE_BITS)),
        "no_ROS_simulator_or_training_calls": True, "smoother_or_temporal_delay_evaluated": False,
        "preserve_sequence_frames": args.preserve_sequence_frames,
        "native_evidence_join_rule": "own_frame_plus_same_SHA_and_exact_image_stamp_only" if args.preserve_sequence_frames else "historical_SHA_join",
        "sequence_ordering": "original_input_row_order_not_resampled" if args.preserve_sequence_frames else "first_record_per_unique_JPEG_SHA",
        "all_evaluated_original_JPEG_SHA256_before_after_equal": True,
        "detector_actual_device": str(model.device),
        "selector_contract_version": SELECTOR_CONTRACT_VERSION,
        "selector_config": DEFAULT_SELECTOR_CONFIG.to_dict(),
        "reference_limitations": ["assistant visual housing QC is not independent human identity GT",
            "named-head identity-only annotations are not complete detector labels or manual lamp-state GT",
            "native states do not certify render visibility or readable arrow pixels",
            "development raw Ego oracle is not operating localization accuracy",
            "permission proxy is current profile behavior, not traffic-rule/planner safety"],
        "parameters": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()
                       if key not in ("native_evidence", "housing_qc")},
        "sources": {"tool_sha256": sha256(__file__), "dataset_sha256": sha256(args.dataset),
            "all_input_and_source_SHA256_before_after_equal": True,
            "initial_source_hashes": initial_source_hashes,
            "detector_sha256": weights_digest, "is_fixed_original_detector_baseline": weights_digest == BASELINE_SHA256,
            "geometry_sha256": sha256(args.geometry), "sensor_sha256": sha256(args.sensor),
            "policy_sha256": sha256(args.policy), "selector_sha256": sha256(PACKAGE/"scripts/traffic_light_map_selector.py"),
            "planner_signal_config_sha256": sha256(args.planner_signal_config), "planner_proxy_flags": planner_flags,
            "classifier_sha256": sha256(PACKAGE/"scripts/traffic_light_cnn_classifier.py"),
            "native_evidence": [{"path": str(path), "sha256": sha256(path)} for path in args.native_evidence],
            "housing_qc": [{"path": str(path), "sha256": sha256(path)} for path in args.housing_qc],
            "cnn_metadata": cnn_metadata},
        "detector_wall_timing_ms": {"count": len(timings), "first": timings[0],
            "mean_including_first": sum(timings)/len(timings),
            "mean_after_first": sum(timings[1:])/len(timings[1:]) if len(timings)>1 else None},
        "summary": summarize(records),
        "by_environment": {value: summarize([row for row in records if row.get("environment") == value])
                           for value in sorted({row.get("environment") for row in records}, key=str)},
        "by_pose_group": {value: summarize([row for row in records if row.get("pose_group") == value])
                          for value in sorted({row.get("pose_group") for row in records}, key=str)},
        "records": records}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({"output": str(output), "frames": len(records), "summary": report["summary"]}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
