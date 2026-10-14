#!/usr/bin/env python3
"""Audit qualified private-ROS tracker variants through current Map-CV.

The native 30Hz outputs preserve the recorded detector header/arrival ages.
Instrumented common-clock barriers qualify logic, not async latency, GPU,
TF delivery, MORAI, planner consumption, or closed-loop safety.
"""
import argparse
import gzip
import importlib.util
import json
from pathlib import Path
import sys

import yaml

from audit_vehicle_map_prediction import (ROOT, compare_paired, digest,
    distribution, evaluate, key_tuple, load_json, normalize_frames)


def qualify_replay(outputs, receipt):
    """Refuse effect claims when the guard's stamp/center/ID golden fails."""
    required = {"baseline", "guard", "innovation", "combined"}
    if set(outputs) != required:
        raise ValueError("Require all four frozen ROS variants")
    for key in ("input_age_preserved", "original_header_preserved",
                "original_receipt_schedule_preserved", "private_boundary_barriers"):
        if receipt.get(key) is not True:
            raise ValueError("Missing qualification proof: "+key)
    if receipt.get("golden_pass") is False or receipt.get("qualification_pass") is False:
        raise ValueError("Upstream golden qualification explicitly failed")
    if not receipt["sent_frames"] or receipt["sent_frames"] != receipt["valid_source_frames"]:
        raise ValueError("Incomplete input sequence")
    counts = {mode: len(frames) for mode, frames in outputs.items()}
    if counts != receipt["outputs"] or len(set(counts.values())) != 1:
        raise ValueError("Output cohort counts differ")
    baseline_stamps = [frame["stamp"] for frame in outputs["baseline"]]
    if len(baseline_stamps) < receipt["sent_frames"]:
        raise ValueError("Insufficient outputs")
    if any(b <= a for a, b in zip(baseline_stamps, baseline_stamps[1:])):
        raise ValueError("Non-increasing output stamps")
    for mode, frames in outputs.items():
        if [frame["stamp"] for frame in frames] != baseline_stamps:
            raise ValueError("Output clock phases differ: "+mode)
    for name, first, second in (("baseline_guard", "baseline", "guard"),
                                ("innovation_combined", "innovation", "combined")):
        golden = receipt["paired"][name]
        if (golden["reference_frames"] != counts[first] or golden["unpaired_frames"]
                or golden["identity_set_changed_frames"]
                or golden["stats"]["stamp_delta"]["maximum"] != 0.
                or golden["stats"]["center_delta"]["maximum"] != 0.):
            raise ValueError("Guard-only golden failed: "+name)
        # Independently inspect the payload instead of trusting summary alone.
        for left, right in zip(outputs[first], outputs[second]):
            a = {obj["id"]: obj for obj in left["objects"]}
            b = {obj["id"]: obj for obj in right["objects"]}
            if len(a) != len(left["objects"]) or len(b) != len(right["objects"]) or set(a) != set(b):
                raise ValueError("Duplicate or changed UUID in guard-only comparison")
            for identity in a:
                if a[identity]["center"] != b[identity]["center"] or a[identity]["label"] != b[identity]["label"]:
                    raise ValueError("Guard altered native center/class")
    if "native_arrival_counts" in receipt and any(
            value != receipt["sent_frames"] for value in receipt["native_arrival_counts"].values()):
        raise ValueError("Not every native node acknowledged all inputs")
    return dict(pass_golden=True, matched_clock_and_uuid_sets=True, native_center_exact=True,
                source_input_count=receipt["sent_frames"], output_frames=counts,
                note="Instrumented clock/callback barriers qualify deterministic logic only")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, required=True, help="Qualified outputs.json.gz")
    parser.add_argument("--gtpack", type=Path, required=True)
    parser.add_argument("--gt-case", default="team_261014")
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--window", nargs=2, type=float)
    parser.add_argument("--actor-pair", action="append", default=[],
                        help="Preregistered UUID=GT_trajectory_key diagnostic without current-position gate")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Fresh output directory required")
    outputs = load_json(args.replay)
    upstream_path = args.replay.with_name("report.json")
    receipt = load_json(upstream_path)
    qualification = qualify_replay(outputs, receipt)
    source = Path(receipt["source_frames"])
    if digest(source) != receipt["source_sha256"]:
        raise ValueError("Original source frames changed")
    inputs = [frame for frame in load_json(source) if frame["world_objects"] is not None]
    if len(inputs) != receipt["valid_source_frames"]:
        raise ValueError("Original input eligibility cohort changed")
    input_age = distribution([frame["receipt"]-frame["stamp"] for frame in inputs])
    sys.path[:0] = [str(ROOT/"base_ws/src/perception_object_pipeline/src")]
    import perception_object_pipeline.map_based_prediction as core
    parameters = yaml.safe_load(args.params.read_text())["map_based_prediction"]
    if parameters["prediction_time_step"] != .1 or parameters["prediction_horizon"] != 3.:
        raise ValueError("Current 0.1s/3s prediction contract required")
    config = core.MapPredictionConfig(**{k: v for k, v in parameters.items()
                                        if k in core.MapPredictionConfig.__dataclass_fields__})
    network = core.LaneletNetwork.load(ROOT/"kcity_map/lanelet_map"/Path(parameters["map_file"]).name,
        ROOT/"kcity_map/lanelet_map"/Path(parameters["projector_file"]).name, config)
    node_path = ROOT/"base_ws/src/perception_object_pipeline/scripts/map_based_prediction_node.py"
    spec = importlib.util.spec_from_file_location("audited_ros_vehicle_map_node", node_path)
    node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    case = load_json(args.gtpack)["cases"][args.gt_case]
    if int(case["provenance"].get("excluded_scoring_frames", 0)):
        raise ValueError("Preserve the GT pack's excluded scoring cohort before using this audit")
    trajectories = {key_tuple(row["key"]): ([p[0] for p in row["series"]], [p[1] for p in row["series"]])
                    for row in case["gt_trajectories"]}
    report = dict(qualification=qualification, upstream_scope=receipt["scope"],
        direction_guard_clock=receipt.get("direction_guard_clock", "publication-time 0.4s hold (V2b)"),
        original_input_header_to_receipt_age_sec=input_age,
        age_caveat="Output stamp is native scheduler time, not the last detection association time. Do not infer fresh observations from 30Hz headers.",
        scope="Actual isolated ROS native outputs followed by CPU-only current Map-CV and audited GT. Common-clock barriers, not async runtime latency or closed-loop certification.",
        window=args.window, gt_provenance=case["provenance"], variants={}, paired={}, fixed_actor_diagnostics={},
        provenance=dict(replay_sha256=digest(args.replay), upstream_report_sha256=digest(upstream_path),
            source_input_sha256=receipt["source_sha256"], script_sha256=digest(__file__),
            helper_sha256=digest(Path(__file__).with_name("audit_vehicle_map_prediction.py")),
            gtpack_sha256=digest(args.gtpack), parameters_sha256=digest(args.params),
            map_sha256=network.map_sha256, node_sha256=digest(node_path),
            predictor_sha256=digest(ROOT/"base_ws/src/perception_object_pipeline/src/perception_object_pipeline/map_based_prediction.py")))
    args.output.mkdir(parents=True)
    all_rows = {}
    for mode in ("baseline", "guard", "innovation", "combined"):
        frames = normalize_frames(outputs[mode], "ros", {})
        summary, rows = evaluate(frames, trajectories, network, node, args.window)
        report["variants"][mode] = summary
        all_rows[mode] = rows
        with gzip.open(args.output/(mode+"_rows.json.gz"), "wt") as stream:
            json.dump(rows, stream, allow_nan=False)
        print(json.dumps(dict(mode=mode, **summary["counts"])), flush=True)
    for name, first, second in (("baseline_guard", "baseline", "guard"),
                                ("baseline_innovation", "baseline", "innovation"),
                                ("innovation_combined", "innovation", "combined"),
                                ("baseline_combined", "baseline", "combined")):
        report["paired"][name] = compare_paired(all_rows[first], all_rows[second])
    for argument in args.actor_pair:
        identity, gt_identity = argument.split("=", 1)
        key = (args.gt_case, gt_identity)
        if key not in trajectories:
            raise ValueError("Explicit fixed actor has no GT trajectory")
        selected_rows = {}
        selected_frames = {}
        diagnostic = dict(uuid=identity, gt_key=list(key),
            scope="Explicit event identity diagnostic; no current-position gate. Not general association, AP, or visibility recall.",
            variants={}, protected={})
        for mode in outputs:
            frames = normalize_frames(outputs[mode], "ros", {})
            selected = [dict(stamp=f["stamp"], objects=[o for o in f["objects"] if o["id"] == identity])
                        for f in frames]
            summary, rows = evaluate(selected, {key: trajectories[key]}, network, node, args.window, float("inf"))
            diagnostic["variants"][mode] = summary
            selected_rows[mode] = rows
            selected_frames[mode] = selected
            with gzip.open(args.output/(mode+"_fixed_"+identity+"_rows.json.gz"), "wt") as stream:
                json.dump(rows, stream, allow_nan=False)
        for name, first, second in (("baseline_guard", "baseline", "guard"),
                                    ("innovation_combined", "innovation", "combined")):
            changed = set()
            for a, b in zip(selected_frames[first], selected_frames[second]):
                if a["objects"] and b["objects"]:
                    before, after = a["objects"][0], b["objects"][0]
                    if sum((x-y)**2 for x, y in zip(before["world_velocity"], after["world_velocity"])) > 1.e-18:
                        changed.add(round(a["stamp"], 6))
            left = [row for row in selected_rows[first] if round(row["stamp"], 6) in changed]
            right = [row for row in selected_rows[second] if round(row["stamp"], 6) in changed]
            diagnostic["protected"][name] = dict(changed_output_frames=len(changed),
                paired_fde=compare_paired(left, right))
        report["fixed_actor_diagnostics"][identity] = diagnostic
    (args.output/"report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
