#!/usr/bin/env python3
"""명시적 시각 QC와 원본 SHA/ROI를 결합하는 연구용 데이터 큐레이션.

원본·QC·JPEG는 수정하지 않는다. 출력 JSON은 원본 JSON과 같은 부모의 새
경로여야 한다. 배경 QC는 records 배열에서 SHA+ROI, visual_labels_known,
manual_qc_accepted, manual_no_lamp_qc_accepted가 모두 명시되어야 한다.
읽기 어려운 실제 신호등을 무램프 배경으로 바꾸지 않는다.
"""

import argparse
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import re

from train_traffic_light_crop_probe import (
    CLASSES, SPLITS, annotation_source, image_size, sha256, train_pos_weight,
    validate_crop, validate_labels,
)


BACKGROUND = "manual_background_no_lamps"


def _read(path):
    path = Path(path).resolve(strict=True)
    payload = path.read_bytes()
    return json.loads(payload), {"path": str(path), "sha256": hashlib.sha256(payload).hexdigest()}


def _roi(value):
    box = validate_crop(value, 2**31, 2**31)
    if any(float(a) != b for a, b in zip(value, box)):
        raise ValueError("QC ROI는 명시적 정수 픽셀 좌표여야 합니다")
    return box


def _qc_rows(document):
    # records에는 evidence/reviewer도 보존된다. 같은 파일의 by-SHA는 보조 색인이다.
    if isinstance(document.get("records"), list):
        return document["records"]
    indexed = document.get("by_image_sha256")
    if not isinstance(indexed, dict):
        raise ValueError("QC에는 records 또는 by_image_sha256이 필요합니다")
    result = []
    for digest, value in indexed.items():
        for row in value if isinstance(value, list) else [value]:
            if not isinstance(row, dict) or row.get("image_sha256", digest) != digest:
                raise ValueError("QC SHA 색인이 일치하지 않습니다")
            result.append(dict(row, image_sha256=digest))
    return result


def load_qc(paths):
    index, sources = {}, []
    for path in paths:
        document, source = _read(path)
        if not isinstance(document, dict):
            raise ValueError("QC는 JSON 객체여야 합니다")
        sources.append(source)
        for position, row in enumerate(_qc_rows(document)):
            digest = row.get("image_sha256")
            if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
                raise ValueError("QC image_sha256이 올바르지 않습니다")
            corrected = "corrected_crop_xyxy" in row
            box = _roi(row.get("corrected_crop_xyxy") if corrected else row.get("crop_xyxy"))
            decision = {key: row.get(key) for key in (
                "manual_qc_accepted", "visual_labels_known", "visual_labels",
                "has_unknown_visual_labels", "manual_no_lamp_qc_accepted")}
            decision["has_unknown_visual_labels"] = row.get("has_unknown_visual_labels") is True
            decision["annotation_kind"] = BACKGROUND if row.get("annotation_kind") == BACKGROUND else None
            signature = json.dumps(decision, sort_keys=True, ensure_ascii=False)
            key = digest, box
            if key in index and index[key]["signature"] != signature:
                raise ValueError("명시적 QC 파일 사이 SHA/ROI 판정 충돌: {} {}".format(digest, box))
            reference = {**source, "qc_record_index": position, "corrected_crop_xyxy": list(box),
                         "reviewer_kind": row.get("manual_qc_reviewer_kind", document.get("reviewer_kind")),
                         "evidence": row.get("evidence")}
            if key not in index:
                index[key] = {"row": row, "roi": box, "signature": signature,
                              "explicit_corrected_roi": corrected, "references": []}
            index[key]["references"].append(reference)
    return index, sources


def _counts(rows):
    result = {}
    for kind in ("native_verified", BACKGROUND):
        selected = [row for row in rows if (BACKGROUND if row.get("annotation_kind") == BACKGROUND else "native_verified") == kind]
        def count(subset):
            return {"records": len(subset), "class_positive_counts": {
                key: sum(int(row["labels"][key]) for row in subset) for key in CLASSES}}
        result[kind] = {"overall": count(selected)}
        for field in ("split", "environment"):
            result[kind]["by_" + field] = {value: count([row for row in selected if row[field] == value])
                                           for value in sorted({row[field] for row in selected})}
    return result


def curate(dataset, qc_results, background_proposals=None, *, size_reader=image_size):
    raw, source = _read(dataset)
    root = Path(source["path"]).parent
    if not isinstance(raw, dict) or not isinstance(raw.get("records"), list):
        raise ValueError("원본 데이터에 records 배열이 필요합니다")
    qc_index, qc_sources = load_qc(qc_results)
    proposals, background_source = [], None
    if background_proposals is not None:
        background, background_source = _read(background_proposals)
        proposals = background.get("records") if isinstance(background, dict) else None
        if not isinstance(proposals, list):
            raise ValueError("배경 proposal에는 records 배열이 필요합니다")
    accepted, rejected, verified_images = [], [], {}
    anchors, split_groups = {}, {}
    # 원본의 split/group은 QC로 바꾸지 않으며 배경의 불변 기준이 된다.
    for row in raw["records"]:
        key = row.get("image_sha256")
        identity = (row.get("image_path"), row.get("split"), row.get("group_id"), row.get("environment"))
        if key in anchors and anchors[key] != identity:
            raise ValueError("같은 JPEG 원본의 경로/split/group/environment가 충돌합니다")
        anchors[key] = identity
        group, split = row.get("group_id"), row.get("split")
        if group in split_groups and split_groups[group] != split:
            raise ValueError("원본 pose group이 split을 넘습니다")
        split_groups[group] = split
    for origin, rows in (("positive", raw["records"]), ("background", proposals)):
        for position, row in enumerate(rows):
            reason = None
            try:
                if not isinstance(row, dict):
                    raise ValueError("record_not_object")
                digest, relative = row.get("image_sha256"), row.get("image_path")
                if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
                    raise ValueError("invalid_image_sha256")
                if not isinstance(relative, str) or not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
                    raise ValueError("unsafe_relative_image_path")
                path = (root / relative).resolve(strict=True)
                path.relative_to(root)
                if path not in verified_images:
                    dimensions = size_reader(path)
                    if (not isinstance(dimensions, (tuple, list)) or len(dimensions) != 2
                            or any(type(value) is not int or value <= 0 for value in dimensions)):
                        raise ValueError("invalid_image_dimensions")
                    verified_images[path] = (sha256(path), dimensions)
                actual, (width, height) = verified_images[path]
                if actual != digest:
                    raise ValueError("image_sha256_mismatch")
                raw_roi = _roi(row.get("crop_xyxy"))
                validate_crop(raw_roi, width, height)
                bits = validate_labels(row.get("labels"))
                if row.get("split") not in SPLITS or not isinstance(row.get("group_id"), str) or not row["group_id"].strip() or not isinstance(row.get("environment"), str) or not row["environment"].strip():
                    raise ValueError("invalid_split_group_environment")
                if origin == "positive":
                    if row.get("native_gt_verified") is not True or row.get("annotation_kind") == BACKGROUND:
                        raise ValueError("positive_native_gt_not_verified")
                    if not any(bits):
                        raise ValueError("positive_all_zero_not_background")
                else:
                    if row.get("annotation_kind") != BACKGROUND or row.get("native_gt_verified") is not False or any(bits):
                        raise ValueError("invalid_manual_background_contract")
                    if anchors.get(digest) != (relative, row.get("split"), row.get("group_id"), row.get("environment")):
                        raise ValueError("background_source_frame_split_group_mismatch")
                match = qc_index.get((digest, raw_roi))
                corrected_from_raw = False
                if match is None and origin == "positive":
                    candidates = [entry for (sha, _), entry in qc_index.items()
                                  if sha == digest and entry["explicit_corrected_roi"]
                                  and entry["row"].get("annotation_kind") != BACKGROUND
                                  and entry["row"].get("manual_no_lamp_qc_accepted") is not True]
                    if len(candidates) > 1:
                        raise ValueError("ambiguous_corrected_qc_roi")
                    if candidates:
                        match, corrected_from_raw = candidates[0], True
                if match is None:
                    raise ValueError("missing_exact_sha_roi_qc")
                qc = match["row"]
                if qc.get("manual_qc_accepted") is not True:
                    raise ValueError("manual_qc_not_accepted")
                if qc.get("visual_labels_known") is not True or qc.get("has_unknown_visual_labels") is True:
                    raise ValueError("visual_labels_unknown")
                try:
                    visual_bits = validate_labels(qc.get("visual_labels"))
                except ValueError as error:
                    raise ValueError("visual_bits_not_all_known") from error
                if visual_bits != bits:
                    raise ValueError("native_or_background_visual_label_mismatch")
                if origin == "background":
                    if qc.get("manual_no_lamp_qc_accepted") is not True or qc.get("annotation_kind") != BACKGROUND:
                        raise ValueError("manual_no_lamp_qc_not_accepted")
                elif qc.get("manual_no_lamp_qc_accepted") is True or qc.get("annotation_kind") == BACKGROUND:
                    raise ValueError("positive_qc_is_background")
                final_roi = list(validate_crop(match["roi"], width, height))
                item = copy.deepcopy(row)
                item.update(crop_xyxy=final_roi, crop_integer_xyxy=final_roi,
                            manual_qc_accepted=True, visual_labels_known=True,
                            visual_labels=copy.deepcopy(qc["visual_labels"]))
                if origin == "background":
                    item["manual_no_lamp_qc_accepted"] = True
                item["curation_provenance"] = {
                    "source_kind": origin, "source_record_index": position,
                    "original_crop_xyxy": list(raw_roi), "explicit_corrected_roi_applied": corrected_from_raw,
                    "approved_sha256_and_roi": {"image_sha256": digest, "crop_xyxy": final_roi},
                    "qc_references": match["references"],
                    "image_sha256_reverified": True, "decoded_image_dimensions": [width, height],
                    "native_gt_preserved_without_relabeling": origin == "positive",
                    "independent_human_or_deployment_gt_certified": False}
                annotation_source(item, allow_manual_background=True)
                accepted.append(item)
            except (ValueError, OSError, TypeError, KeyError) as error:
                reason = str(error)
            if reason is not None:
                rejected.append({"source_kind": origin, "source_record_index": position,
                                 "image_path": row.get("image_path") if isinstance(row, dict) else None,
                                 "image_sha256": row.get("image_sha256") if isinstance(row, dict) else None,
                                 "split": row.get("split") if isinstance(row, dict) else None,
                                 "environment": row.get("environment") if isinstance(row, dict) else None,
                                 "reason": reason})
    # 승인되지 않은 원본 프레임도 위 anchors의 split/group 기준은 유지한다.
    doc = copy.deepcopy(raw)
    doc["records"] = accepted
    if "summary" in doc:
        doc["source_raw_summary"] = doc.pop("summary")
    if "groups" in doc:
        doc["source_raw_groups"] = doc.pop("groups")
    doc["summary"] = {"record_count": len(accepted),
                      "split_counts": dict(Counter(row["split"] for row in accepted)),
                      "environment_counts": dict(Counter(row["environment"] for row in accepted)),
                      "accepted_counts": _counts(accepted)}
    provenance = {"raw_dataset": source, "qc_files": qc_sources, "background_proposals": background_source,
                  "no_mtime_or_implicit_qc_selection": True, "script_sha256": sha256(__file__),
                  "scope": "research_only_explicit_visual_QC_not_independent_human_or_production_GT"}
    doc.update(research_only=True, curation=provenance, record_count=len(accepted))
    summary = {"research_only": True, "provenance": provenance,
               "accepted_records": len(accepted), "rejected_records": len(rejected),
               "rejected_reason_counts": dict(Counter(row["reason"] for row in rejected)),
               "accepted_counts": _counts(accepted), "rejected": rejected,
               "unique_source_images_rechecked": len(verified_images)}
    summary["rejected_counts_by_source_kind"] = {}
    for origin in ("positive", "background"):
        subset = [row for row in rejected if row["source_kind"] == origin]
        summary["rejected_counts_by_source_kind"][origin] = {
            "records": len(subset), "reason_counts": dict(Counter(row["reason"] for row in subset)),
            "by_split": dict(Counter(str(row["split"]) for row in subset)),
            "by_environment": dict(Counter(str(row["environment"]) for row in subset))}
    try:
        if any(not any(row["split"] == split for row in accepted) for split in SPLITS):
            raise ValueError("train/val/test가 모두 필요합니다")
        if len({row["group_id"] for row in accepted if row["split"] == "train"}) < 2:
            raise ValueError("train pose group 두 개 이상이 필요합니다")
        summary["train_pos_weight"] = dict(zip(CLASSES, train_pos_weight([
            dict(row, bits=validate_labels(row["labels"])) for row in accepted])))
        summary["training_population_gate_passed"] = True
    except ValueError as error:
        summary.update(training_population_gate_passed=False, training_population_gate_error=str(error))
    for artifact in [source] + qc_sources + ([background_source] if background_source else []):
        if sha256(artifact["path"]) != artifact["sha256"]:
            raise ValueError("큐레이션 중 원본/QC/proposal 파일이 변경되었습니다")
    return doc, summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--qc-results", type=Path, action="append", required=True)
    parser.add_argument("--background-proposals", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    raw = args.dataset.resolve(strict=True)
    output = args.output.absolute()
    sidecar = output.with_name(output.stem + ".curation.json")
    if output.parent.resolve(strict=True) != raw.parent or output.suffix.lower() != ".json":
        parser.error("출력 JSON은 원본 JSON과 같은 부모 폴더에 있어야 합니다")
    if any(path.exists() or path.is_symlink() for path in (output, sidecar)):
        parser.error("출력 및 sidecar는 새 경로여야 합니다")
    doc, summary = curate(raw, args.qc_results, args.background_proposals)
    for path, data in ((output, doc), (sidecar, summary)):
        with path.open("x", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    print(json.dumps({"출력": str(output), "요약": str(sidecar), "승인": len(doc["records"]),
                      "제외": summary["rejected_records"], "학습표본검사": summary["training_population_gate_passed"]}, ensure_ascii=False))
    return 0 if summary["training_population_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
