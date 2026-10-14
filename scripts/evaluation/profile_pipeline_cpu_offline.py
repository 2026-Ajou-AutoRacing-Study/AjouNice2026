#!/usr/bin/env python3
"""Paired, exact-output CPU regression on saved production tracking messages.

No ROS master, node, simulator, GPU or deployed install space is modified.
Reference source comes from an explicit Git commit, not the current worktree.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager
import cProfile
from datetime import datetime, timezone
import gc
import hashlib
import io
import json
from pathlib import Path
import pstats
import platform
import subprocess
import sys
import time
import types

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "base_ws/src/perception_object_pipeline"
sys.path[:0] = [str(PACKAGE / "src"), str(PACKAGE / "scripts")]


def stats(values):
    if not values:
        return {"count": 0}
    a = np.asarray(values, dtype=float)
    return {"count": len(values), "mean": float(a.mean()),
            "median": float(np.median(a)), "p90": float(np.percentile(a, 90)),
            "p95": float(np.percentile(a, 95)), "p99": float(np.percentile(a, 99)),
            "max": float(a.max())}


def serialize(message):
    output = io.BytesIO()
    message.serialize(output)
    return output.getvalue()


def file_digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024*1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@contextmanager
def observe_gc():
    starts = {}
    pauses = []
    def callback(phase, info):
        generation = info["generation"]
        if phase == "start":
            starts[generation] = time.perf_counter()
        elif generation in starts:
            pauses.append({"generation": generation,
                           "ms": (time.perf_counter()-starts.pop(generation))*1000})
    gc.callbacks.append(callback)
    try:
        yield pauses
    finally:
        gc.callbacks.remove(callback)


def reference_module(ref, relative, name):
    source = subprocess.check_output(
        ["git", "show", ref+":"+relative], cwd=ROOT, text=True)
    module = types.ModuleType(name)
    module.__file__ = ref+":"+relative
    sys.modules[name] = module
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    return module, hashlib.sha256(source.encode()).hexdigest()


def main():
    import rosbag
    import map_based_prediction_node as candidate_node
    from perception_object_pipeline import map_based_prediction as candidate_core
    from perception_object_pipeline import object_motion_policy as candidate_policy
    from perception_object_pipeline import message_copy, node_processing_timing

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline-ref", required=True)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--profile-frames", type=int, default=200)
    parser.add_argument("--timing-mode", choices=("paired", "reference", "candidate"),
                        default="paired", help="Separate processes attribute GC tails without cross-mode heap contamination; pair first for wire equality")
    args = parser.parse_args()
    if args.repeat < 1 or args.stride < 1:
        parser.error("repeat and stride must be positive")
    if args.output.exists():
        raise FileExistsError("Fresh output directory required")
    args.output.mkdir(parents=True)
    ref = subprocess.check_output(["git", "rev-parse", args.baseline_ref], cwd=ROOT,
                                  text=True).strip()
    prefix = "base_ws/src/perception_object_pipeline/"
    reference_core, core_digest = reference_module(ref, prefix+"src/perception_object_pipeline/map_based_prediction.py", "cpu_reference_core")
    reference_node, node_digest = reference_module(ref, prefix+"scripts/map_based_prediction_node.py", "cpu_reference_node")
    reference_policy, policy_digest = reference_module(ref, prefix+"src/perception_object_pipeline/object_motion_policy.py", "perception_object_pipeline.cpu_reference_policy")
    config = yaml.safe_load((PACKAGE/"config/map_based_prediction.yaml").read_text())
    map_file = ROOT/"kcity_map/lanelet_map/lanelet2_map_v85.osm"
    projector = ROOT/"kcity_map/lanelet_map/map_projector_info_v1.yaml"
    modes = ("reference", "candidate") if args.timing_mode == "paired" else (args.timing_mode,)
    cores = {"reference": reference_core, "candidate": candidate_core}
    networks = {mode: cores[mode].LaneletNetwork.load(map_file, projector, cores[mode].MapPredictionConfig(**config))
                for mode in modes}
    makers = {"reference": reference_node.make_map_based_predicted_objects,
              "candidate": candidate_node.make_map_based_predicted_objects}
    policies = {"reference": reference_policy.apply_object_motion_policy,
                "candidate": candidate_policy.apply_object_motion_policy}
    horizons = candidate_node.build_prediction_horizons(0.1, 3.0)
    report = {"created_at_utc": datetime.now(timezone.utc).isoformat(),
              "python_version": platform.python_version(), "platform": platform.platform(),
              "reference_commit": ref,
              "reference_source_sha256": {"core": core_digest, "node": node_digest, "policy": policy_digest},
              "candidate_source_sha256": {
                  "core": hashlib.sha256(Path(candidate_core.__file__).read_bytes()).hexdigest(),
                  "node": hashlib.sha256(Path(candidate_node.__file__).read_bytes()).hexdigest(),
                  "policy": hashlib.sha256(Path(candidate_policy.__file__).read_bytes()).hexdigest(),
                  "message_copy": file_digest(Path(message_copy.__file__)),
                  "node_processing_timing": file_digest(Path(node_processing_timing.__file__))},
              "map_sha256": next(iter(networks.values())).map_sha256, "config": config,
              "timing_mode": args.timing_mode,
              "cases": [], "limits": [
                  "Local CPU-only, no live ROS scheduling/publish/network latency or planner safety claim.",
                  "Deserialization is outside callback timing; message generation and explicit wire serialization are timed separately.",
                  "Normal GC remains enabled; all tails included; cProfile only in a separate pass.",
                  "First 30 input frames excluded from timing, never from full wire equality.",
                  "Map uses current v85 and production config for every case, not a claim to reproduce historic map output."]}
    profile_samples = []
    for bag_path in args.bag:
        frames = []
        topic = "/perception/object_recognition/tracking/objects"
        with rosbag.Bag(str(bag_path)) as bag:
            topics = bag.get_type_and_topic_info().topics
            if topic not in topics:
                raise ValueError("Tracking topic missing in "+str(bag_path))
            for index, (_, message, _) in enumerate(bag.read_messages(topics=[topic])):
                if index % args.stride == 0:
                    frames.append((type(message), serialize(message)))
        timings = defaultdict(list)
        counts = Counter()
        tails = []
        for repeat in range(args.repeat):
            for index, (message_type, wire) in enumerate(frames):
                message = message_type().deserialize(wire)
                results = {}
                policy_results = {}
                order = ("candidate", "reference") if (index+repeat)%2 else ("reference", "candidate")
                if len(modes) == 1:
                    order = modes
                for mode in order:
                    start = time.perf_counter()
                    policy_output, constrained = policies[mode](message)
                    policy_ms = (time.perf_counter()-start)*1000
                    policy_results[mode] = (serialize(policy_output), constrained)
                    cpu_start = time.process_time()
                    with observe_gc() as pauses:
                        start = time.perf_counter()
                        output, mapped, fallback, paths = makers[mode](message, horizons, networks[mode])
                        make_ms = (time.perf_counter()-start)*1000
                    cpu_ms = (time.process_time()-cpu_start)*1000
                    start = time.perf_counter()
                    result_wire = serialize(output)
                    ser_ms = (time.perf_counter()-start)*1000
                    results[mode] = (result_wire, mapped, fallback, paths)
                    if index >= 30 and message.objects:
                        timings[mode+"/motion_policy_ms"].append(policy_ms)
                        timings[mode+"/make_ms"].append(make_ms)
                        timings[mode+"/serialize_ms"].append(ser_ms)
                        timings[mode+"/make_and_serialize_ms"].append(make_ms+ser_ms)
                        timings[mode+"/process_cpu_make_ms"].append(cpu_ms)
                        if make_ms > 20:
                            tails.append({"frame": index, "repeat": repeat, "mode": mode,
                                          "objects": len(message.objects), "make_ms": make_ms,
                                          "cpu_ms": cpu_ms, "gc_pauses": pauses})
                if len(modes) == 2 and results["reference"] != results["candidate"]:
                    raise AssertionError("PredictedObjects wire/count differs: "+str((bag_path,index,repeat)))
                if len(modes) == 2 and policy_results["reference"] != policy_results["candidate"]:
                    raise AssertionError("Motion policy differs: "+str((bag_path,index,repeat)))
                if repeat == 0:
                    counts["frames"] += 1
                    counts["empty_frames"] += not bool(message.objects)
                    counts["objects"] += len(message.objects)
                    counts["mapped"] += mapped
                    counts["fallback"] += fallback
                    counts["paths"] += paths
                    counts["unknown_constrained"] += constrained
                    if message.objects:
                        profile_samples.append((message_type, wire))
        cache = getattr(networks.get("candidate"), "_route_geometry_cache", ())
        case = {"bag": str(bag_path), "bag_sha256": file_digest(bag_path),
                "counts": dict(counts), "repeat": args.repeat, "stride": args.stride,
                "exact_predicted_wire_and_counts": True if len(modes) == 2 else None,
                "exact_motion_policy_wire_and_counts": True if len(modes) == 2 else None,
                "timing_ms": {key: stats(value) for key,value in timings.items()},
                "tails_gt20ms": tails, "route_cache_entries": len(cache)}
        report["cases"].append(case)
        print(json.dumps(case), flush=True)
        (args.output/"report.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    profile_samples = profile_samples[::max(1,len(profile_samples)//max(1,args.profile_frames))][:args.profile_frames]
    for mode in modes:
        profiler = cProfile.Profile()
        profiler.enable()
        for message_type, wire in profile_samples:
            message = message_type().deserialize(wire)
            serialize(makers[mode](message, horizons, networks[mode])[0])
        profiler.disable()
        stream = io.StringIO()
        pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats("cumulative").print_stats(60)
        (args.output/(mode+"_cprofile.txt")).write_text(stream.getvalue())


if __name__ == "__main__":
    main()
