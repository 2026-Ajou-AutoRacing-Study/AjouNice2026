#!/usr/bin/env python3
"""Prepare common source-snapshot pairs and run learned E24 offline inference.

Geometry grouping, host-first reference, and experimental footer reference are
separate variants. No training, live driver or production policy is modified.
"""
import argparse
import bisect
import datetime
import hashlib
import json
import pathlib
import time

import numpy as np

from lidar_packet_reassembly import footer_nearest_hour, quantiles, read_source

CAMERAS = ("front", "front_left", "front_right", "back")


def nearest_received(rows, stamp, available, tolerance=.05):
    candidates = [(abs(t - stamp), t, received, message) for t, received, message in rows
                  if abs(t - stamp) <= tolerance and received <= available]
    return min(candidates, key=lambda row: row[0]) if candidates else None


def drive_window(path, messages):
    # Bounded raw QA uses its own controller, not validator mission events.
    # Keep the real moving interval; do not substitute a failed full mission.
    motion_path = path.parent / 'motion_trace.json'
    if motion_path.exists():
        moving = [r['time'] for r in json.loads(motion_path.read_text()) if r['speed'] > .3]
        if len(moving) < 10:
            raise ValueError('Insufficient moving interval')
        return min(moving)+.2, max(moving)-.2
    events_path = path.parent / "events.jsonl"
    if events_path.exists():
        events = [json.loads(line) for line in events_path.read_text().splitlines() if line]
        start = next(datetime.datetime.fromisoformat(row["time"]).timestamp()
                     for row in events if row["event"] == "CONTROLLER_START") + 1
        end = next(datetime.datetime.fromisoformat(row["time"]).timestamp()
                   for row in events if row["event"] == "RUN_FINISHED") - .5
        return start, end
    times = [stamp for stamp, _, _ in messages["/sensor/lidar1"]]
    return times[0] + 1, times[-1] - 1


def frozen_camera_controls(args):
    """Reuse exact inputs; separate geometry from camera/reference changes.

    The cut0_footer slot becomes a same-cut0-input A/A control, explicitly
    tagged. It is NOT a footer-reference experiment in this manifest.
    """
    if args.output.exists():
        raise ValueError('Fresh controls output required')
    original = json.loads(args.inputs.read_text())
    groups = {}
    for row in original['rows']:
        groups.setdefault(row['pair'], {})[row['policy']] = row
    rows = []
    for group in groups.values():
        baseline = group['fixed51']
        for policy in ('fixed51', 'fixed50', 'cut0', 'cut180', 'cut0_footer'):
            frame = group['cut0'] if policy == 'cut0_footer' else group[policy]
            rows.append({**frame, 'token': frame['pair']+'_'+policy+'_frozen_camera',
                         'policy': policy, 'geometry_original_stamp': frame['stamp'],
                         'stamp': baseline['stamp'], 'images': baseline['images'],
                         'camera_offsets_sec': baseline['camera_offsets_sec'],
                         'control_role': 'cut0_AA_repeat' if policy == 'cut0_footer' else 'geometry_only'})
    args.output.mkdir(parents=True)
    manifest = {**original, 'rows': rows, 'frozen_camera_and_reference': True,
                'control_note': 'All policies use fixed51 original JPEGs and declared host-first reference. cut0_footer is a repeat of cut0, NOT a footer-reference change.',
                'original_input_sha256': hashlib.sha256(args.inputs.read_bytes()).hexdigest()}
    (args.output/'inputs.json').write_text(json.dumps(manifest, indent=2)+'\n')


def prepare(args):
    if args.samples_per_case < 1:
        raise ValueError('Positive per-case sample count required')
    args.output.mkdir(parents=True, exist_ok=False)
    decoded = json.loads((args.decoded / "manifest.json").read_text())
    rows, case_info = [], []
    for case in decoded["cases"]:
        path = pathlib.Path(case["source_bag"])
        source = read_source(path)
        start_time, end_time = drive_window(path, source["messages"])
        cloud_receipts = {stamp: received for stamp, received, _ in source["messages"]["/sensor/lidar1"]}
        original_delays = [cloud_receipts[row["stamp"]] - source["packets"][row["end"] - 1].stamp.to_sec()
                           for row in source["scans"]]
        candidate_delay = float(np.median(original_delays))
        by_policy = {}
        for frame in case["frames"]:
            if frame["startup_partial"] or frame["packet_span_sec"] > .15:
                continue
            packet = source["packets"][frame["packet_end"] - 1]
            footer = int.from_bytes(bytes(packet.data)[1200:1204], "little")
            anchor = footer_nearest_hour(packet.stamp.to_sec(), footer)
            if not start_time <= anchor <= end_time:
                continue
            by_policy.setdefault(frame["policy"], {})[footer] = {**frame, "anchor": anchor,
                "available": cloud_receipts[frame["stamp"]] if frame["policy"] == "fixed51" else
                packet.stamp.to_sec() + candidate_delay}
        common = sorted(set.intersection(*(set(by_policy[policy]) for policy in ("fixed51", "fixed50", "cut0", "cut180"))),
                        key=lambda footer: by_policy["fixed51"][footer]["anchor"])
        eligible, exclusions = [], {"camera_sync_or_availability": 0}
        for footer in common:
            candidates = {policy: frame[footer] for policy, frame in by_policy.items()}
            candidates["cut0_footer"] = {**candidates["cut0"], "stamp": candidates["cut0"]["anchor"]}
            images = {}
            for policy, frame in candidates.items():
                chosen = [nearest_received(source["messages"][f"/sensor/camera/{camera}/compressed"],
                                           frame["stamp"], frame["available"]) for camera in CAMERAS]
                if any(item is None for item in chosen):
                    exclusions["camera_sync_or_availability"] += 1
                    break
                images[policy] = chosen
            else:
                eligible.append((footer, candidates, images))
        selected = np.unique(np.rint(np.linspace(0, len(eligible) - 1,
                             min(args.samples_per_case, len(eligible)))).astype(int)) if eligible else []
        if not len(selected):
            raise ValueError(f"No common synchronized pairs in case {case['case']}")
        case_folder = args.output / f"case{case['case']}"
        case_folder.mkdir()
        image_files = {}
        for selection in selected:
            footer, candidates, images = eligible[selection]
            for policy, frame in candidates.items():
                image_paths, offsets = {}, {}
                for camera, item in zip(CAMERAS, images[policy]):
                    _, image_stamp, received, message = item
                    key = (camera, image_stamp)
                    if key not in image_files:
                        dest = case_folder / f"camera_{camera}_{int(round(image_stamp * 1e6))}.jpg"
                        data = bytes(message.data)
                        if not data.startswith(b"\xff\xd8"):
                            raise ValueError("Native JPEG contract mismatch")
                        dest.write_bytes(data)
                        image_files[key] = str(dest)
                    image_paths["image/camera_" + camera] = image_files[key]
                    offsets[camera] = image_stamp - frame["stamp"]
                rows.append({"token": f"case{case['case']}_{footer}_{policy}", "case": case["case"],
                             "pair": f"case{case['case']}_{footer}", "policy": policy,
                             "stamp": frame["stamp"], "native_latest_stamp": frame["anchor"],
                             "npz": str(args.decoded / frame["path"]), "images": image_paths,
                             "camera_offsets_sec": offsets, "point_count": frame["points"],
                             "footer_distinct": frame["footer_distinct"], "candidate_available": frame["available"]})
        case_info.append({"case": case["case"], "source": str(path), "common": len(common),
                          "eligible": len(eligible), "selected_pairs": len(selected), "exclusions": exclusions,
                          "drive_window": [start_time, end_time], "candidate_decode_receipt_delay_estimate_ms": candidate_delay * 1000})
        print(case_info[-1], flush=True)
    manifest = {"rows": rows, "cases": case_info,
                "scope": "Common latest source-footer pairs, not exact live candidate publication/ROS queue replay.",
                "footer_reference": "Experimental closest UTC hour on this single PC; not validated across unsynchronized LAN clocks.",
                "availability": "Original cloud receipt for51; candidate last packet host stamp +median original decoder/receipt lag."}
    (args.output / "inputs.json").write_text(json.dumps(manifest, indent=2) + "\n")


def infer(args):
    import cv2
    import torch
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration

    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    metadata = json.loads(args.inputs.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    calibration = load_sensor4_calibration(args.sensor, ["camera_" + camera for camera in CAMERAS])
    model = BevFusionModel(str(args.config), str(args.checkpoint), calibration,
                           use_amp=True, preprocess_workers=4, warmup_iterations=2,
                           base5_runtime=True, parallel_camera=True, point_intensity_source="ros255")
    outputs = []
    try:
        for index, row in enumerate(metadata["rows"]):
            with np.load(row["npz"]) as data:
                points = np.column_stack([data[name] for name in ("x", "y", "z", "intensity")]).astype(np.float32)
            images = {name: pathlib.Path(path).read_bytes() for name, path in row["images"].items()}
            predictions, prep_ms, infer_ms = model.infer(points, images, index, .0, 200,
                                                        timestamp_sec=row["stamp"])
            outputs.append({**row, "predictions": predictions, "preprocess_ms": prep_ms,
                            "inference_ms": infer_ms, "postprocess_ms": model.last_runtime_timings["postprocess_ms"]})
            if index % 25 == 0:
                print(json.dumps({"frames": index + 1, "total": len(metadata["rows"]),
                                  "policy": row["policy"], "predictions": len(predictions), "ms": infer_ms}), flush=True)
                (args.output / "checkpoint.json").write_text(json.dumps(outputs) + "\n")
        digest = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
        result = {"rows": outputs, "checkpoint_sha256": digest, "config": str(args.config),
                  "input_sha256": hashlib.sha256(args.inputs.read_bytes()).hexdigest(),
                  "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  "sensor_sha256": hashlib.sha256(args.sensor.read_bytes()).hexdigest(),
                  "device": torch.cuda.get_device_name(), "amp": True, "parallel_camera": True,
                  "compile": False, "score_floor": 0, "max_detections": 200,
                  "reference_note": "Raw decoder study. Operating bridge floor/.4/.1/.1 +NMS must be applied separately.",
                  "inference_ms": quantiles([row["inference_ms"] for row in outputs]),
                  "max_memory_mib": torch.cuda.max_memory_allocated() / 1024 ** 2}
        (args.output / "predictions.json").write_text(json.dumps(result) + "\n")
    finally:
        model.preprocessor.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    make = commands.add_parser("prepare")
    make.add_argument("--decoded", type=pathlib.Path, required=True)
    make.add_argument("--output", type=pathlib.Path, required=True)
    make.add_argument("--samples-per-case", type=int, default=100)
    controls = commands.add_parser('controls')
    controls.add_argument('--inputs', type=pathlib.Path, required=True)
    controls.add_argument('--output', type=pathlib.Path, required=True)
    evaluate = commands.add_parser("infer")
    evaluate.add_argument("--inputs", type=pathlib.Path, required=True)
    evaluate.add_argument("--output", type=pathlib.Path, required=True)
    evaluate.add_argument("--config", type=pathlib.Path, required=True)
    evaluate.add_argument("--checkpoint", type=pathlib.Path, required=True)
    evaluate.add_argument("--sensor", type=pathlib.Path, required=True)
    args = parser.parse_args()
    {'prepare': prepare, 'controls': frozen_camera_controls, 'infer': infer}[args.command](args)


if __name__ == "__main__":
    main()
