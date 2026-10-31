#!/usr/bin/env python3
"""Render existing pipeline results and original crops; no inference/ROS/X11.

Examples are deterministic illustrations, not aggregate accuracy or manual lamp
GT. Identity errors never earn color-agreement credit. Recorded classifier crop
coordinates are used exactly, including the recorded CNN margin, without edits
to original JPEG/PNG files. No policy is re-run or label inferred from pixels.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CLASSES = ["red", "yellow", "green", "left_arrow"]
FAILURES = {"wrong_identity", "raw_mismatch", "no_selection", "reference_identity_mismatch"}
UNSUCCESSFUL = FAILURES | {"classifier_unknown", "classifier_not_run", "identity_unknown"}
STATUS_KO = {"native_agreement": "정확한 대상 · native 상태 일치",
             "raw_mismatch": "정확한 대상 · 원시 4bit 불일치",
             "wrong_identity": "대상 오류 — 색 일치여도 성공 아님",
             "no_selection": "선택 crop 없음",
             "identity_unknown": "대상 identity 미확인",
             "classifier_unknown": "분류 UNKNOWN — 소등과 구분",
             "classifier_not_run": "선택한 분류기 결과 없음",
             "native_unknown": "선택 head의 native 정답 미확인",
             "reference_identity_mismatch": "native와 검토 head ID 불일치"}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def four_bits(value):
    return tuple(value) if (isinstance(value, (list, tuple)) and len(value) == 4
                           and all(type(bit) is int and bit in (0, 1) for bit in value)) else None


def side_status(side, classifier):
    if side.get("selected_bbox_xyxy") is None:
        return "no_selection"
    if side.get("target_identity_correct") is False:
        return "wrong_identity"
    identity = side.get("identity", {})
    if (not identity.get("known") or side.get("target_identity_correct") is not True
            or not isinstance(identity.get("native_id"), str) or not identity["native_id"]):
        return "identity_unknown"
    observation = side.get("classifiers", {}).get(classifier)
    if observation is None:
        return "classifier_not_run"
    if observation.get("known") is not True or four_bits(observation.get("bits")) is None:
        return "classifier_unknown"
    reference = side.get("reference", {})
    if reference.get("known") is not True or four_bits(reference.get("bits")) is None:
        return "native_unknown"
    if reference.get("native_id") != identity.get("native_id"):
        return "reference_identity_mismatch"
    return "native_agreement" if four_bits(observation["bits"]) == four_bits(reference["bits"]) else "raw_mismatch"


def stamp_ns(row):
    key = row.get("sequence_frame_key")
    value = key.get("source_stamp_ns") if isinstance(key, dict) else row.get("source_stamp_ns")
    if type(value) is int and value > 0:
        return value
    stamp = row.get("header_stamp") or row.get("provenance", {}).get("source_header_stamp", {})
    sec, nsec = stamp.get("secs", stamp.get("sec")), stamp.get("nsecs", stamp.get("nanosec"))
    if type(sec) is int and type(nsec) is int and sec > 0 and 0 <= nsec < 10**9:
        return sec*10**9+nsec
    return None


def align_pipeline(document, manifest_rows, manifest_path, mode, classifier):
    """Exact event association; SHA-only fallback is allowed only when unique."""
    if document.get("schema_version") != 1 or document.get("status") not in (None, "complete"):
        raise ValueError("완성된 pipeline schema_version=1 JSON이 필요합니다")
    if document.get("classes_in_bit_order") != CLASSES:
        raise ValueError("pipeline의 4bit 순서가 R/Y/G/좌화살표와 다릅니다")
    expected = document.get("sources", {}).get("dataset_sha256")
    if expected != digest(manifest_path):
        raise ValueError("pipeline 원본 dataset SHA와 manifest가 다릅니다")
    by_sha, aligned = {}, {}
    for index, row in enumerate(manifest_rows):
        by_sha.setdefault(row["image_sha256"], []).append(index)
    for row in document.get("records", []):
        matches = by_sha.get(row.get("image_sha256"), [])
        key = row.get("sequence_frame_key")
        if key is not None:
            if not isinstance(key, dict) or stamp_ns(row) is None:
                raise ValueError("잘못된 exact sequence frame key")
            index = key.get("input_row_index")
            # Manifest SHA + original row/stamp are portable across repository paths.
            # An absolute input_document path in the report is provenance, not a
            # requirement to keep another teammate's /home directory unchanged.
            if type(index) is not int or index not in matches:
                raise ValueError("sequence frame key가 원본 manifest와 일치하지 않습니다")
            matches = [index]
        timestamp = stamp_ns(row)
        if timestamp is not None:
            matches = [index for index in matches if stamp_ns(manifest_rows[index]) == timestamp]
        if len(matches) != 1:
            raise ValueError("원본 frame 연결이 없거나 모호합니다; 반복 SHA에는 exact stamp가 필요합니다")
        index = matches[0]
        if index in aligned:
            raise ValueError("pipeline에 동일 원본 frame 결과가 중복됐습니다")
        source = manifest_rows[index]
        if row.get("pose_group") != source.get("group_id") or row.get("environment") != source.get("environment"):
            raise ValueError("pipeline pose/environment와 원본 frame이 다릅니다")
        if mode not in row or classifier not in row[mode].get("classifiers", {}):
            raise ValueError("요청한 선택 모드/분류기가 pipeline에 없습니다")
        aligned[index] = row
    if not aligned:
        raise ValueError("pipeline에 비교 가능한 frame이 없습니다")
    return aligned


def pair_results(baseline, candidate, manifest, manifest_path, baseline_mode="legacy_center",
                 baseline_classifier="hsv", candidate_mode="map_association", candidate_classifier="cnn"):
    if manifest.get("status") not in (None, "complete") or manifest.get("errors"):
        raise ValueError("완료된 원본 manifest만 허용합니다")
    rows = manifest["records"]
    old = align_pipeline(baseline, rows, manifest_path, baseline_mode, baseline_classifier)
    new = align_pipeline(candidate, rows, manifest_path, candidate_mode, candidate_classifier)
    if set(old) != set(new):
        raise ValueError("두 pipeline의 평가 frame 집합이 다릅니다")
    result = []
    for index in sorted(old):
        a, b = old[index][baseline_mode], new[index][candidate_mode]
        result.append({"manifest_index": index, "source": rows[index], "source_stamp_ns": stamp_ns(rows[index]),
            "baseline": a, "candidate": b, "baseline_status": side_status(a, baseline_classifier),
            "candidate_status": side_status(b, candidate_classifier),
            "stratum": (str(rows[index].get("group_id")), str(rows[index].get("environment")))})
    return result


def select_examples(pairs, maximum=12):
    if type(maximum) is not int or not 1 <= maximum <= 12:
        raise ValueError("예시는 최대 1~12개입니다")
    def stable(row):
        return row["stratum"]+(row.get("source_stamp_ns") or -1, row["manifest_index"], row["source"]["image_sha256"])
    remaining = sorted(pairs, key=stable)
    selected, counts = [], Counter()
    def take(row, reason):
        selected.append((row, reason)); counts[row["stratum"]] += 1
        remaining.remove(row)
    failures = [row for row in remaining if row["candidate_status"] in FAILURES]
    if failures:
        take(failures[0], "후보 실패 사례 보존")
    else:
        unknown = [row for row in remaining if row["candidate_status"] in UNSUCCESSFUL]
        if unknown:
            take(unknown[0], "후보 UNKNOWN 사례 보존 (확정 오답과 구분)")
    improved = [row for row in remaining if row["baseline_status"] == "raw_mismatch"
                and row["candidate_status"] == "native_agreement"]
    if improved and len(selected) < maximum:
        take(improved[0], "baseline 원시 오류 → 후보 native 일치")
    while remaining and len(selected) < maximum:
        minimum = min(counts[row["stratum"]] for row in remaining)
        balanced = [row for row in remaining if counts[row["stratum"]] == minimum]
        def priority(row):
            improved = row["baseline_status"] == "raw_mismatch" and row["candidate_status"] == "native_agreement"
            rank = 0 if improved else 1 if row["candidate_status"] in UNSUCCESSFUL else 2 if row["candidate_status"] != "native_agreement" else 3
            return (rank, stable(row))
        row = min(balanced, key=priority)
        take(row, "group/environment 균형 표본")
    return selected


def phase_text(tuples):
    names = {("red", "circle", None): "R", ("yellow", "circle", None): "Y",
             ("green", "circle", None): "G", ("green", "arrow", True): "L",
             ("green", "arrow", False): "우화살표"}
    return "+".join(names.get(tuple(value), str(value)) for value in tuples) if tuples else "∅"


def crop_box(side, classifier, image_size):
    if side.get("selected_bbox_xyxy") is None:
        return None
    box = side["classifiers"][classifier].get("crop_xyxy")
    width, height = image_size
    if (not isinstance(box, list) or len(box) != 4 or any(type(value) is not int for value in box)
            or not 0 <= box[0] < box[2] <= width or not 0 <= box[1] < box[3] <= height):
        raise ValueError("기록된 classifier crop 좌표가 유효하지 않습니다")
    return tuple(box)


def load_font(path=None, size=18):
    from PIL import ImageFont
    candidates = [Path(path)] if path else [Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
                                            Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf")]
    for candidate in candidates:
        if candidate.is_file():
            return ImageFont.truetype(str(candidate), size), candidate.resolve()
    raise ValueError("한글 font가 필요합니다; --font로 명시하세요")


def render_panel(canvas, position, original, side, classifier, title, status, font):
    from PIL import Image, ImageDraw
    x, y = position; draw = ImageDraw.Draw(canvas)
    draw.rectangle((x, y, x+760, y+340), fill="#202a36", outline="#536171")
    draw.text((x+12, y+7), title, font=font, fill="white")
    box = crop_box(side, classifier, original.size)
    display = None
    if box is not None:
        crop = original.crop(box)
        ratio = min(730/crop.width, 130/crop.height)
        display = (max(1, round(crop.width*ratio)), max(1, round(crop.height*ratio)))
        nearest = getattr(Image, "Resampling", Image).NEAREST
        canvas.paste(crop.resize(display, nearest), (x+(760-display[0])//2, y+38+(135-display[1])//2))
        geometry = f"원본 crop {crop.width}×{crop.height} → 표시 {display[0]}×{display[1]} (NEAREST)"
    else:
        draw.text((x+225, y+85), "선택 crop 없음", font=font, fill="#b7c3d0")
        geometry = "원본 crop 없음 — GT crop으로 대체하지 않음"
    observation = side["classifiers"][classifier]
    raw = four_bits(observation.get("bits")) if observation.get("known") is True else None
    raw_text = "".join(map(str, raw)) if raw is not None else "UNKNOWN"
    if raw == (0, 0, 0, 0): raw_text += " (확신 있는 소등)"
    reference = side.get("reference", {})
    native = four_bits(reference.get("bits")) if reference.get("known") is True else None
    native_text = "".join(map(str, native)) if native is not None else "미확인"
    identity = side.get("identity", {})
    head = identity.get("native_id") if identity.get("known") else "미확인"
    claim = side.get("selection", {}).get("head_id", "없음")
    color = "#8de4a7" if status == "native_agreement" else "#ff9b9b" if status in FAILURES else "#ffd28b"
    lines = [(STATUS_KO[status], color),
             (f"선택 주장: {claim} / QC 연결 head: {head}", "white"),
             (f"원시 RYGL: {raw_text} / native: {native_text} (수동 램프 GT 아님)", "white"),
             (f"정책 보정: {phase_text(observation.get('policy', {}).get('corrected_tuples', []))}", "white"),
             (geometry, "#b7c3d0"),
             (f"crop={list(box) if box else None}, 기록된 margin={observation.get('crop_margin_px', '미기록')}px", "#b7c3d0")]
    for offset, (line, fill) in enumerate(lines):
        draw.text((x+12, y+180+offset*25), line, font=font, fill=fill)
    return {"crop_xyxy": list(box) if box else None, "display_size_wh": list(display) if display else None,
            "original_crop_pixel_SHA256_RGB": digest_crop(original, box) if box else None,
            "display_interpolation": "NEAREST",
            "status": status, "raw_bits": list(raw) if raw is not None else None,
            "native_reference": reference, "identity": identity,
            "normalized_policy": observation.get("policy", {}).get("corrected_tuples", []),
            "recorded_margin_px": observation.get("crop_margin_px")}


def digest_crop(original, box):
    return hashlib.sha256(original.crop(box).tobytes()).hexdigest()


def validate_output(path):
    original = Path(path).expanduser()
    resolved = original.resolve()
    if original.exists() or original.is_symlink() or (ROOT/"artifacts/evaluation").resolve() not in resolved.parents:
        raise ValueError("output은 저장소 artifacts/evaluation 아래의 새 폴더여야 합니다")
    return resolved


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("baseline", "candidate", "manifest", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--max-examples", type=int, default=12)
    parser.add_argument("--baseline-mode", default="legacy_center")
    parser.add_argument("--baseline-classifier", default="hsv")
    parser.add_argument("--candidate-mode", default="map_association")
    parser.add_argument("--candidate-classifier", default="cnn")
    parser.add_argument("--font", type=Path)
    args = parser.parse_args(argv)
    try:
        output = validate_output(args.output)
        paths = [args.baseline.resolve(strict=True), args.candidate.resolve(strict=True), args.manifest.resolve(strict=True)]
        inputs = {str(path): digest(path) for path in paths}
        baseline, candidate, manifest = [json.loads(path.read_text()) for path in paths]
        pairs = pair_results(baseline, candidate, manifest, paths[2], args.baseline_mode,
                             args.baseline_classifier, args.candidate_mode, args.candidate_classifier)
        examples = select_examples(pairs, args.max_examples)
        font, font_path = load_font(args.font)
        from PIL import Image, ImageDraw
        images, protected = {}, dict(inputs)
        protected[str(font_path)] = digest(font_path)
        protected[str(Path(__file__).resolve())] = digest(__file__)
        for pair, _ in examples:
            row = pair["source"]; relative = Path(row["image_path"])
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError("안전하지 않은 원본 이미지 경로")
            path = (paths[2].parent/relative).resolve(strict=True)
            if paths[2].parent not in path.parents or digest(path) != row["image_sha256"]:
                raise ValueError("원본 이미지 경로/SHA가 바뀌었습니다")
            protected[str(path)] = row["image_sha256"]
            with Image.open(path) as image: images[pair["manifest_index"]] = image.convert("RGB")
            crop_box(pair["baseline"], args.baseline_classifier, images[pair["manifest_index"]].size)
            crop_box(pair["candidate"], args.candidate_classifier, images[pair["manifest_index"]].size)
        output.mkdir(parents=True, exist_ok=False)
        entries = []
        for batch in range(0, len(examples), 4):
            group = examples[batch:batch+4]
            page = Image.new("RGB", (1580, 150+len(group)*390), "#111923")
            draw = ImageDraw.Draw(page)
            draw.text((20, 12), "신호등 파이프라인 — 원본 crop 비교", font=font, fill="white")
            draw.text((20, 40), "native API 상태 일치 ≠ 수동 램프 GT. 대상 오류·UNKNOWN은 색 정답으로 보상하지 않음.", font=font, fill="#ffdc91")
            draw.text((20, 68), "선정 예시는 전체 성능이 아님. sparse 정지 사진으로 전환 지연·공식 통행 규칙 준수를 평가하지 않음.", font=font, fill="#b7c3d0")
            draw.text((20, 96), f"YOLO 입력 imgsz: 기존 {baseline.get('parameters', {}).get('imgsz', '미기록')} / 후보 {candidate.get('parameters', {}).get('imgsz', '미기록')}. 표시 crop 확대는 모델 입력 전처리와 다름.", font=font, fill="#b7c3d0")
            for offset, (pair, reason) in enumerate(group):
                number = batch+offset+1; top = 142+offset*390
                draw.text((20, top), f"예시 {number:02d} | {pair['stratum'][0]} / {pair['stratum'][1]} | {reason}", font=font, fill="white")
                image = images[pair["manifest_index"]]
                a = render_panel(page, (20, top+30), image, pair["baseline"], args.baseline_classifier,
                                 f"기존: {args.baseline_mode}/{args.baseline_classifier} (원본 crop)", pair["baseline_status"], font)
                b = render_panel(page, (800, top+30), image, pair["candidate"], args.candidate_classifier,
                                 f"후보: {args.candidate_mode}/{args.candidate_classifier} (기록된 margin)", pair["candidate_status"], font)
                entries.append({"example_index": number, "manifest_index": pair["manifest_index"],
                    "source_stamp_ns": pair["source_stamp_ns"], "image_sha256": pair["source"]["image_sha256"],
                    "pose_group": pair["stratum"][0], "environment": pair["stratum"][1], "reason": reason,
                    "sheet": f"comparison_{batch//4:02d}.png", "baseline": a, "candidate": b})
            page.save(output/f"comparison_{batch//4:02d}.png")
        if any(digest(path) != sha for path, sha in protected.items()):
            raise ValueError("렌더링 중 원본 입력이 변경됐습니다")
        report = {"schema_version": 1, "purpose": "deterministic_illustrations_not_aggregate_accuracy",
            "no_model_inference_ROS_X11_or_image_generation": True, "originals_unchanged": True,
            "not_manual_lamp_GT_or_temporal_latency_certification": True,
            "no_credit_for_wrong_identity": True, "paired_frame_count": len(pairs),
            "selected_example_count": len(entries), "maximum_examples": args.max_examples,
            "selection_rule": "reserve_candidate_failure_or_prediction_UNKNOWN_then_raw_improvement_then_least_represented_group_environment; missing_native_reference_is_not_candidate_failure",
            "source_SHA256": protected, "renderer_sha256": digest(__file__),
            "derived_sheet_SHA256": {path.name: digest(path) for path in sorted(output.glob("comparison_*.png"))},
            "font_path": str(font_path), "font_sha256": digest(font_path),
            "display_is_not_model_preprocessing": True,
            "detector_imgsz_baseline_candidate": [baseline.get("parameters", {}).get("imgsz"), candidate.get("parameters", {}).get("imgsz")],
            "detector_SHA256_baseline_candidate": [baseline.get("sources", {}).get("detector_sha256"), candidate.get("sources", {}).get("detector_sha256")],
            "baseline_mode_classifier": [args.baseline_mode, args.baseline_classifier],
            "candidate_mode_classifier": [args.candidate_mode, args.candidate_classifier], "examples": entries}
        with (output/"index.json").open("x") as stream: json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        print(json.dumps({"output": str(output), "examples": len(entries), "pages": (len(entries)+3)//4}, ensure_ascii=False))
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
