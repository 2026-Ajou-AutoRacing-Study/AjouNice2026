#!/usr/bin/env python3
"""Current detector-center correspondence under frozen header/footer GT.

All prediction bytes and operating thresholds are identical. This is not AP,
proven false-positive adjudication, tracker velocity/FDE, or a runtime clock
correction. Footer GT is an experimental source-batch reference on one PC.
"""

import argparse
from collections import Counter, defaultdict
import gzip
import json
from pathlib import Path

import numpy as np

from analyze_live_perception_candidates import final, matches
from analyze_base5_proposal_peaks import CLASSES, THRESHOLDS, distribution
from prepare_independent_vehicle_inputs import digest, save
from prepare_source_footer_gt_sensitivity import unchanged_sensor_contract


def evaluated_frame(prediction, truth, gate):
    labels = truth["gt_labels"]
    pairs = matches(truth["gt_boxes"], prediction["pred_boxes"], labels, prediction["pred_labels"], gate)
    return {truth["parent_uids"][g]: dict(gt_index=g, prediction_index=p, xy_error_m=error,
                                         label=int(labels[g])) for g, p, error in pairs}


def summarize(predictions, truths, gate):
    counts, errors = defaultdict(Counter), defaultdict(list)
    for prediction, truth in zip(predictions, truths):
        if prediction["token"] != truth["token"]:
            raise ValueError("Correspondence must use exact frozen frame tokens")
        paired = evaluated_frame(prediction, truth, gate)
        for label, name in enumerate(CLASSES):
            counts[name]["frames"] += 1
            counts[name]["gt_observations"] += sum(value == label for value in truth["gt_labels"])
            counts[name]["operating_predictions"] += sum(value == label for value in prediction["pred_labels"])
            values = [value for value in paired.values() if value["label"] == label]
            counts[name]["matched"] += len(values)
            errors[name].extend(value["xy_error_m"] for value in values)
    result = {}
    for name in CLASSES:
        count = counts[name]
        result[name] = dict(count, gt_match_fraction=count["matched"]/count["gt_observations"] if count["gt_observations"] else None,
                            unmatched_operating=count["operating_predictions"]-count["matched"],
                            xy_error_m=distribution(errors[name]))
    return dict(frames=len(predictions), gate_m=gate, classes=result,
                gate_comparison="Strict distance < gate, matching existing same-class maximum-cardinality diagnostic.")


def paired_errors(predictions, headers, footers, gate):
    counts, values, same_prediction_values, rows = Counter(), defaultdict(list), defaultdict(list), []
    for prediction, header, footer in zip(predictions, headers, footers):
        if not prediction["token"] == header["token"] == footer["token"]:
            raise ValueError("Clock comparison frames differ")
        if len(set(header["parent_uids"])) != len(header["parent_uids"]) or len(set(footer["parent_uids"])) != len(footer["parent_uids"]):
            raise ValueError("GT actor identity duplicated in a frame")
        before = evaluated_frame(prediction, header, gate)
        after = evaluated_frame(prediction, footer, gate)
        hg = {key: label for key, label in zip(header["parent_uids"], header["gt_labels"])}
        fg = {key: label for key, label in zip(footer["parent_uids"], footer["gt_labels"])}
        counts["frames"] += 1
        counts["header_gt_observations"] += len(hg)
        counts["footer_gt_observations"] += len(fg)
        counts["header_only_gt_observations"] += len(set(hg)-set(fg))
        counts["footer_only_gt_observations"] += len(set(fg)-set(hg))
        for key in sorted(set(hg) & set(fg)):
            if hg[key] != fg[key]:
                counts["same_actor_class_changed"] += 1
                continue
            counts["same_actor_frame_observations"] += 1
            a, b = before.get(key), after.get(key)
            if a is None and b is None:
                counts["unmatched_both"] += 1
            elif b is None:
                counts["matched_header_only"] += 1
            elif a is None:
                counts["matched_footer_only"] += 1
            else:
                counts["matched_both"] += 1
                delta = b["xy_error_m"]-a["xy_error_m"]
                same = a["prediction_index"] == b["prediction_index"]
                counts["matched_both_same_prediction" if same else "matched_both_changed_prediction"] += 1
                for name, value in (("header_xy_m", a["xy_error_m"]), ("footer_xy_m", b["xy_error_m"]),
                                    ("footer_minus_header_xy_m", delta)):
                    values[name].append(value)
                    if same:
                        same_prediction_values[name].append(value)
                rows.append(dict(token=header["token"], stamp=header["stamp"], actor_key=key,
                                 class_name=CLASSES[hg[key]], header_error_m=a["xy_error_m"],
                                 footer_error_m=b["xy_error_m"], footer_minus_header_m=delta,
                                 same_prediction=same, gate_m=gate))
    return dict(counts=dict(counts), matched_both_distribution={key: distribution(value) for key, value in values.items()},
                same_prediction_distribution={key: distribution(value) for key, value in same_prediction_values.items()},
                caveat="Paired only when the same actor exists and matches in both references. Header-only/footer-only GT and matches are separately reported."), rows


def analyze(args):
    if args.output.exists():
        raise FileExistsError("Fresh analysis output required")
    source = json.loads(args.inputs.read_text())
    footer = json.loads(args.footer_gt.read_text())
    with gzip.open(args.predictions, "rt") as stream:
        predictions = json.load(stream)
    if predictions["input_sha256"] != digest(args.inputs) or footer["source_inputs_sha256"] != digest(args.inputs):
        raise ValueError("Inference/derived GT do not share the same frozen input manifest")
    source_by_token = {row["token"]: row for row in source["rows"]}
    footer_by_token = {row["token"]: row for row in footer["rows"]}
    prediction_by_token = {row["token"]: row for row in predictions["rows"]}
    if (len(source_by_token) != len(source["rows"]) or len(footer_by_token) != len(footer["rows"])
            or len(prediction_by_token) != len(predictions["rows"])
            or not set(source_by_token) == set(footer_by_token) == set(prediction_by_token)):
        raise ValueError("Missing/duplicate frozen frame tokens")
    operated, headers, footers = [], [], []
    per_case = defaultdict(lambda: dict(predictions=[], headers=[], footers=[]))
    for token, raw in source_by_token.items():
        prediction, alternative = prediction_by_token[token], footer_by_token[token]
        unchanged_sensor_contract(raw, prediction)
        unchanged_sensor_contract(raw, alternative)
        if not raw["scene_epoch_valid"] or not raw["scoring_eligible"]:
            continue
        value = final(prediction)
        operated.append(value)
        headers.append(raw)
        footers.append(alternative)
        case = per_case[raw["case"]]
        case["predictions"].append(value)
        case["headers"].append(raw)
        case["footers"].append(alternative)
    cases, paired_rows = {}, []
    for name, cohort in sorted(per_case.items()):
        report = dict(header={}, footer={}, paired={})
        for gate in (1., 2.):
            slot = str(int(gate))+"m"
            report["header"][slot] = summarize(cohort["predictions"], cohort["headers"], gate)
            eligible = [index for index, truth in enumerate(cohort["footers"])
                        if truth["source_footer_gt_eligible"] and truth["scoring_eligible"]]
            if eligible:
                cp = [cohort["predictions"][i] for i in eligible]
                ch = [cohort["headers"][i] for i in eligible]
                cf = [cohort["footers"][i] for i in eligible]
                report["footer"][slot] = dict(header_same_frames=summarize(cp, ch, gate),
                                             footer_same_frames=summarize(cp, cf, gate))
                paired, data = paired_errors(cp, ch, cf, gate)
                report["paired"][slot] = paired
                paired_rows.extend(data)
        report["source_footer_frames"] = len(eligible)
        cases[name] = report
    eligible = [index for index, truth in enumerate(footers) if truth["source_footer_gt_eligible"] and truth["scoring_eligible"]]
    cp = [operated[i] for i in eligible]
    ch = [headers[i] for i in eligible]
    cf = [footers[i] for i in eligible]
    pooled = {}
    for gate in (1., 2.):
        slot = str(int(gate))+"m"
        paired, _ = paired_errors(cp, ch, cf, gate)
        pooled[slot] = dict(header_same_frames=summarize(cp, ch, gate),
                           footer_same_frames=summarize(cp, cf, gate), paired=paired)
    report = dict(
        schema_version=1, purpose=__doc__, cases=cases, fixed50_clock_pooled=pooled,
        original_prediction_frames=len(predictions["rows"]), scored_loaded_header_frames=len(headers),
        fixed50_footer_same_frame_cohort=len(cp),
        model_input_contract=dict(unchanged=True, scope="Same forward outputs and sensor/header bytes, only diagnostic GT reference changes."),
        operating_point=dict(classes=CLASSES, thresholds=THRESHOLDS, server_floor=.1, server_top100=True,
                             class_aware_rotated_nms_iou=.5, postprocessor="analyze_live_perception_candidates.final"),
        gt_contract=dict(scope="Catalog current-center XY correspondence, no visibility filtering or AP.",
                         header_gt="Explicit physical MORAI yaw correction and original LiDAR header pose.",
                         footer_gt="Local boxes computed at experimental source-footer pose; never use original header ego to world-transform derived GT.",
                         unmatched="Unmatched prediction != proven false positive; no sensor annotation oracle here."),
        source_sha256={str(path.resolve()): digest(path) for path in (args.inputs, args.footer_gt, args.predictions)},
        excluded_prediction_frames=len(predictions["rows"])-len(headers),
        no_tracker_velocity_or_fde=True,
        limitations=["Fixed51/fixed50 are separately recorded live runs with different NPC phases and frame counts; the case table is not a causal same-input policy A/B.",
                     "Header/footer clock comparison alone is frozen to exactly the same fixed50 forwards and frames.",
                     "Footer GT changing ROI entry/exit changes its total GT denominator; paired actor/frame counts and both unmatched cohorts are explicit.",
                     "A lower pooled average need not improve every actor; signed paired tail errors remain in paired_rows.json.gz."],
    )
    args.output.mkdir(parents=True)
    save(args.output / "report.json", report)
    with gzip.open(args.output / "paired_rows.json.gz", "wt") as stream:
        json.dump(paired_rows, stream, allow_nan=False)
    print(json.dumps(dict(output=str(args.output.resolve()), loaded_header_frames=len(headers),
                          fixed50_clock_frames=len(cp),
                          pooled_vehicle={slot: dict(header=value["header_same_frames"]["classes"]["vehicle"],
                                                       footer=value["footer_same_frames"]["classes"]["vehicle"])
                                          for slot, value in pooled.items()})), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--footer-gt", type=Path, required=True)
    parser.add_argument("--predictions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    analyze(parser.parse_args())


if __name__ == "__main__":
    main()
