#!/usr/bin/env python3
"""CPU-only Map-CV audit of frozen vehicle tracker variants.

This is a saved-input diagnostic, not a live scheduler or planner safety test.
Core-state inputs require world velocity and yaw rate; ROS-like inputs require
body velocity and yaw rate. Evaluation GT is never supplied to the predictor.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment
import yaml

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_json(path):
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as stream:
        return json.load(stream)


def load_recorded_tracks(path):
    """Keep the recorded publication schedule; no detector or EKF rerun."""
    import rosbag
    frames = []
    with rosbag.Bag(str(path)) as bag:
        for _, message, _ in bag.read_messages(topics=["/perception/object_recognition/tracking/objects"]):
            if message.header.frame_id.lstrip("/") != "map":
                raise ValueError("Recorded tracker must be in map frame")
            objects = []
            for obj in message.objects:
                label = max(obj.classification, key=lambda c: c.probability).label if obj.classification else 0
                if label not in (1, 2, 3, 4):
                    continue
                pose = obj.kinematics.pose_with_covariance.pose
                q = pose.orientation
                twist = obj.kinematics.twist_with_covariance.twist
                objects.append(dict(id=bytes(obj.object_id.uuid).hex(), label=label,
                    center=[pose.position.x, pose.position.y, pose.position.z],
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                    body_velocity=[twist.linear.x, twist.linear.y], yaw_rate=twist.angular.z))
            frames.append(dict(stamp=message.header.stamp.to_sec(), objects=objects))
    return frames


def key_tuple(value):
    return tuple(key_tuple(v) if isinstance(v, list) else v for v in value)


def distribution(values):
    if not values:
        return {"n": 0}
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError("Nonfinite statistic")
    return dict(n=len(a), mean=float(a.mean()), p50=float(np.median(a)),
                p90=float(np.percentile(a, 90)), p95=float(np.percentile(a, 95)),
                p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def interpolate(series, stamp, max_gap=.15):
    times, points = series
    index = bisect_left(times, stamp)
    if index < len(times) and abs(times[index] - stamp) < 1e-6:
        return np.asarray(points[index], dtype=float)
    if index == 0 or index == len(times) or times[index] - times[index-1] > max_gap:
        return None
    alpha = (stamp - times[index-1]) / (times[index] - times[index-1])
    return (1-alpha)*np.asarray(points[index-1]) + alpha*np.asarray(points[index])


def normalize_frames(payload, input_mode, canonicalization):
    """Match the wrapper's yaw hysteresis and world-to-body velocity rotation.

    No direction correction is performed here. A pi-axis flip cannot change
    the world velocity. Core replay omits the live 30Hz publication scheduler.
    """
    frames = payload.get("frames", payload.get("rows")) if isinstance(payload, dict) else payload
    if not isinstance(frames, list):
        raise ValueError("Expected list of frames")
    flipped = {}
    normalized = []
    previous_stamp = -math.inf
    for frame in frames:
        stamp = float(frame["stamp"])
        if not math.isfinite(stamp) or stamp <= previous_stamp:
            raise ValueError("Strictly increasing finite stamps required")
        previous_stamp = stamp
        if input_mode == "core" and "tracks" not in frame:
            raise ValueError("Core input must have tracks")
        source = frame["tracks"] if input_mode == "core" else frame["objects"]
        vehicles = []
        active_ids = set()
        for obj in source:
            identity = str(obj["id"])
            active_ids.add(identity)
            if input_mode == "core":
                if not obj["confirmed"] or obj["core_class"] not in (1, 2):
                    continue
                label = int(obj["core_class"])
            else:
                label = int(obj.get("label", 1 if obj.get("class_name") == "vehicle" else 0))
                if label not in (1, 2, 3, 4):
                    continue
            yaw = float(obj["yaw"])
            yaw_rate = float(obj["yaw_rate"])
            center = [float(v) for v in obj["center"]]
            if len(center) == 2:
                center.append(0.)
            if len(center) != 3:
                raise ValueError("XYZ center required")
            if input_mode == "core":
                vx, vy = (float(v) for v in obj["velocity"])
                is_flipped = flipped.get(identity, False)
                if not canonicalization["enabled"]:
                    is_flipped = False
                elif math.hypot(vx, vy) >= canonicalization["minimum_speed"]:
                    error = abs(math.atan2(math.sin(yaw-math.atan2(vy, vx)),
                                           math.cos(yaw-math.atan2(vy, vx))))
                    if not is_flipped and error >= canonicalization["enter"]:
                        is_flipped = True
                    elif is_flipped and error <= canonicalization["exit"]:
                        is_flipped = False
                flipped[identity] = is_flipped
                yaw = math.atan2(math.sin(yaw + (math.pi if is_flipped else 0.)),
                                 math.cos(yaw + (math.pi if is_flipped else 0.)))
                body_velocity = [math.cos(yaw)*vx + math.sin(yaw)*vy,
                                 -math.sin(yaw)*vx + math.cos(yaw)*vy]
                world_velocity = [vx, vy]
            else:
                body_velocity = [float(v) for v in obj["body_velocity"]]
                vx, vy = body_velocity
                world_velocity = [math.cos(yaw)*vx - math.sin(yaw)*vy,
                                  math.sin(yaw)*vx + math.cos(yaw)*vy]
            if not np.isfinite(center + body_velocity + world_velocity + [yaw, yaw_rate]).all():
                raise ValueError("Nonfinite state")
            vehicles.append(dict(id=identity, label=label, center=center, yaw=yaw,
                yaw_rate=yaw_rate, body_velocity=body_velocity, world_velocity=world_velocity))
        flipped = {k: v for k, v in flipped.items() if k in active_ids}
        normalized.append(dict(stamp=stamp, objects=vehicles))
    return normalized


def evaluate(frames, trajectories, network, node, window=None, current_gate_m=2.):
    from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects
    from perception_object_pipeline.kinematic_prediction import yaw_to_quaternion
    horizons = [index/10. for index in range(1, 31)]
    counts = Counter()
    values = defaultdict(list)
    rows = []
    for frame in frames:
        stamp = frame["stamp"]
        if window and not window[0] <= stamp <= window[1]:
            continue
        counts["frames"] += 1
        objects = frame["objects"]
        msg = TrackedObjects()
        msg.header.frame_id = "map"
        for obj in objects:
            tracked = TrackedObject()
            tracked.classification = [ObjectClassification(label=obj["label"], probability=1.)]
            p = tracked.kinematics.pose_with_covariance.pose
            p.position.x, p.position.y, p.position.z = obj["center"]
            p.orientation = yaw_to_quaternion(obj["yaw"])
            twist = tracked.kinematics.twist_with_covariance.twist
            twist.linear.x, twist.linear.y = obj["body_velocity"]
            twist.angular.z = obj["yaw_rate"]
            msg.objects.append(tracked)
        output, mapped, fallback, paths = node.make_map_based_predicted_objects(msg, horizons, network)
        counts.update(dict(vehicle_observations=len(objects), mapped=mapped, fallback=fallback, paths=paths))
        observations = [(key, point) for key, series in trajectories.items()
                        if (point := interpolate(series, stamp)) is not None]
        if not objects or not observations:
            counts["unmatched_outputs"] += len(objects)
            continue
        costs = np.linalg.norm(np.asarray([o["center"][:2] for o in objects])[:, None]
                               - np.asarray([p[:2] for _, p in observations])[None], axis=2)
        ii, jj = linear_sum_assignment(np.where(costs <= current_gate_m, costs, 1e6))
        pairs = [(int(i), int(j)) for i, j in zip(ii, jj) if costs[i, j] <= current_gate_m]
        counts["matched_current_2m" if current_gate_m == 2. else "fixed_actor_identity_paired"] += len(pairs)
        counts["unmatched_outputs"] += len(objects) - len(pairs)
        for i, j in pairs:
            obj, predicted = objects[i], output.objects[i]
            key, current_gt = observations[j]
            series = trajectories[key]
            minus, plus = interpolate(series, stamp-.1), interpolate(series, stamp+.1)
            gt_velocity = (plus-minus)[:2]/.2 if minus is not None and plus is not None else None
            speed = float(np.linalg.norm(gt_velocity)) if gt_velocity is not None else None
            opposite = (bool(speed >= 1. and np.linalg.norm(obj["world_velocity"]) >= .2 and
                        np.dot(gt_velocity, obj["world_velocity"]) < 0.) if speed is not None else None)
            counts["opposite_world_motion"] += opposite is True
            for horizon in (1, 2, 3):
                future = interpolate(series, stamp+horizon)
                if future is None:
                    continue
                endpoints = [np.asarray([path.path[horizon*10].position.x,
                             path.path[horizon*10].position.y])
                             for path in predicted.kinematics.predicted_paths]
                errors = [float(np.linalg.norm(point-future[:2])) for point in endpoints]
                values[f"top1_{horizon}s_m"].append(errors[0])
                values[f"minK_{horizon}s_m"].append(min(errors))
                oracle = (float(np.linalg.norm(current_gt[:2]+gt_velocity*horizon-future[:2]))
                          if gt_velocity is not None else None)
                if oracle is not None:
                    values[f"perfect_current_gt_cv_{horizon}s_m"].append(oracle)
                rows.append(dict(stamp=stamp, id=obj["id"], gt_key=list(key), horizon=horizon,
                    current_error_m=float(costs[i, j]), top1_m=errors[0], minK_m=min(errors),
                    oracle_current_cv_m=oracle, opposite_world_motion=opposite,
                    gt_speed_mps=speed, world_velocity=obj["world_velocity"],
                    endpoint=endpoints[0].tolist(), path_count=len(endpoints)))
    return dict(counts=dict(counts), metrics={k: distribution(v) for k, v in values.items()}), rows


def compare_paired(baseline, candidate):
    # Pair on physical GT and stamp, not UUID: ID lifetimes can differ.
    def index(rows):
        return {(round(r["stamp"], 6), key_tuple(r["gt_key"]), r["horizon"]): r for r in rows}
    first, second = index(baseline), index(candidate)
    overlap = set(first) & set(second)
    result = dict(baseline_only=len(set(first)-set(second)), candidate_only=len(set(second)-set(first)))
    for horizon in (1, 2, 3):
        keys = sorted((k for k in overlap if k[-1] == horizon), key=repr)
        delta = [second[k]["top1_m"]-first[k]["top1_m"] for k in keys]
        result[str(horizon)] = dict(paired=len(keys), delta_candidate_minus_baseline_m=distribution(delta),
            baseline_m=distribution([first[k]["top1_m"] for k in keys]),
            candidate_m=distribution([second[k]["top1_m"] for k in keys]),
            improved_gt1m=sum(v < -1. for v in delta), worsened_gt1m=sum(v > 1. for v in delta))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", action="append", required=True, help="NAME=JSON[.gz] or NAME=BAG (ros mode); first is baseline")
    parser.add_argument("--input-mode", choices=("core", "ros"), required=True)
    parser.add_argument("--gt-pack", type=Path, required=True)
    parser.add_argument("--gt-case", required=True)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--window", nargs=2, type=float)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Fresh output required")
    sys.path[:0] = [str(ROOT/"base_ws/src/perception_object_pipeline/src")]
    import perception_object_pipeline.map_based_prediction as core
    parameters = yaml.safe_load(args.parameters.read_text())
    params = parameters["map_based_prediction"]
    configure = parameters.get("configure", parameters.get("ekf_multi_object_tracking", {}).get("configure", {}))
    if args.input_mode == "core" and (configure.get("center_motion_override_output_velocity", False)
            or int(configure.get("detection_velocity_fusion_mode", 0)) in (4, 5)):
        raise ValueError("Core conversion cannot silently omit active wrapper velocity overrides")
    if abs(float(params["prediction_time_step"])-.1) > 1.e-9 or abs(float(params["prediction_horizon"])-3.) > 1.e-9:
        raise ValueError("This audit requires the current 0.1s/3s prediction contract")
    canonical = dict(enabled=bool(configure.get("canonicalize_vehicle_orientation_to_motion", True)),
        minimum_speed=float(configure.get("orientation_canonicalization_min_speed_mps", 1.)),
        enter=math.radians(float(configure.get("orientation_flip_enter_error_deg", 100.))),
        exit=math.radians(float(configure.get("orientation_flip_exit_error_deg", 80.))))
    config = core.MapPredictionConfig(**{k: v for k, v in params.items()
                                        if k in core.MapPredictionConfig.__dataclass_fields__})
    network = core.LaneletNetwork.load(ROOT/"kcity_map/lanelet_map"/Path(params["map_file"]).name,
        ROOT/"kcity_map/lanelet_map"/Path(params["projector_file"]).name, config)
    spec = importlib.util.spec_from_file_location("audited_vehicle_map_node",
        ROOT/"base_ws/src/perception_object_pipeline/scripts/map_based_prediction_node.py")
    node = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node)
    case = load_json(args.gt_pack)["cases"][args.gt_case]
    if int(case["provenance"].get("excluded_scoring_frames", 0)):
        raise ValueError("Pack has excluded scoring frames: preserve its exclusion cohort before reusing this audit")
    trajectories = {key_tuple(row["key"]): ([p[0] for p in row["series"]], [p[1] for p in row["series"]])
                    for row in case["gt_trajectories"]}
    args.output.mkdir(parents=True)
    report = dict(scope="Frozen tracker core/input replay, current predictor. No live 30Hz scheduler, AP, or planner safety claim.",
        gt_provenance=case["provenance"], parameters_sha256=digest(args.parameters),
        gt_pack_sha256=digest(args.gt_pack), script_sha256=digest(__file__), map_sha256=network.map_sha256,
        canonicalization=canonical, window=args.window, variants={}, paired={},
        predictor_source_sha256=digest(ROOT/"base_ws/src/perception_object_pipeline/src/perception_object_pipeline/map_based_prediction.py"),
        kinematic_source_sha256=digest(ROOT/"base_ws/src/perception_object_pipeline/src/perception_object_pipeline/kinematic_prediction.py"),
        node_source_sha256=digest(ROOT/"base_ws/src/perception_object_pipeline/scripts/map_based_prediction_node.py"))
    all_rows = {}
    for argument in args.variant:
        name, filename = argument.split("=", 1)
        if name in all_rows or not name.replace("_", "").replace("-", "").isalnum():
            raise ValueError("Unique simple variant name required")
        path = Path(filename)
        if path.suffix == ".bag" and args.input_mode != "ros":
            raise ValueError("Recorded bag requires ROS input mode")
        payload = load_recorded_tracks(path) if path.suffix == ".bag" else load_json(path)
        frames = normalize_frames(payload, args.input_mode, canonical)
        summary, rows = evaluate(frames, trajectories, network, node, args.window)
        summary["source_sha256"] = digest(filename)
        report["variants"][name] = summary
        all_rows[name] = rows
        with gzip.open(args.output/(name+"_rows.json.gz"), "wt") as stream:
            json.dump(rows, stream, allow_nan=False)
        print(json.dumps(dict(variant=name, **summary["counts"])), flush=True)
    baseline = next(iter(all_rows))
    for name, rows in all_rows.items():
        if name != baseline:
            report["paired"][name] = compare_paired(all_rows[baseline], rows)
    (args.output/"report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
