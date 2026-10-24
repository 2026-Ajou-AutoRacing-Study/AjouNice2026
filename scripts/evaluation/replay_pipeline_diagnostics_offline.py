#!/usr/bin/env python3
"""Private-master installed UNKNOWN -> Map-CV transport qualification.

No detector, GPU, UDP driver, localization, planner, control or simulator runs.
Actual ROS transport and scheduling are exercised, not closed-loop safety.
"""
from __future__ import annotations

import argparse
from collections import Counter
import gc
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "base_ws/src/perception_object_pipeline"
INSTALL = ROOT / "base_ws/install_aw149"
INPUT_TOPIC = "/perception/object_recognition/tracking/objects"


def stats(values):
    if not values:
        return {"count": 0}
    values = np.asarray(values, dtype=float)
    return dict(count=len(values), mean=float(values.mean()), median=float(np.median(values)),
                p90=float(np.percentile(values, 90)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), max=float(values.max()))


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def wire(message, normalize_seq=False):
    original = message.header.seq if normalize_seq else None
    if normalize_seq:
        message.header.seq = 0
    try:
        stream = io.BytesIO()
        message.serialize(stream)
        return stream.getvalue()
    finally:
        if normalize_seq:
            message.header.seq = original


def select_window(rows, count):
    """Keep original adjacent stamps, favor a bounded mixed/UNKNOWN cohort."""
    if count > len(rows) or count < 2:
        raise ValueError("Requested bounded cohort does not fit bag")
    prefix_nonempty = [0]
    prefix_unknown = [0]
    for row in rows:
        prefix_nonempty.append(prefix_nonempty[-1] + int(row["objects"] > 0))
        prefix_unknown.append(prefix_unknown[-1] + row["unknown"])
    choices = []
    for start in range(len(rows) - count + 1):
        nonempty = prefix_nonempty[start + count] - prefix_nonempty[start]
        unknown = prefix_unknown[start + count] - prefix_unknown[start]
        # Mixing is primary, then exercise the actual UNKNOWN path. No tuning
        # of prediction/track policy is performed from the evaluation outputs.
        score = (min(nonempty, count - nonempty), int(unknown > 0), unknown)
        choices.append((score, -start, start))
    start = max(choices)[2]
    selected = rows[start:start + count]
    if not any(row["objects"] for row in selected) or not any(not row["objects"] for row in selected):
        raise ValueError("Bag has no bounded mixed nonempty/empty cohort")
    stamps = [row["stamp_ns"] for row in selected]
    if len(set(stamps)) != len(stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Cohort requires unique strictly increasing stamps for transport tokens")
    return start, selected


def validate_received(expected, received):
    unexpected = sorted(set(received) - set(expected))
    mismatch = sorted(token for token in set(expected) & set(received)
                      if expected[token] != received[token])
    return dict(expected=len(expected), received=len(received), missing=sorted(set(expected) - set(received)),
                unexpected=unexpected, wire_mismatch=mismatch, exact_subset=not unexpected and not mismatch)


def select_balanced_cohort(rows, count):
    """Bounded original frames spanning empty, UNKNOWN, pedestrian and vehicle."""
    if not 2 <= count <= len(rows):
        raise ValueError("Requested bounded cohort does not fit bag")
    buckets = [("empty", 2), ("unknown", 1), ("pedestrian", 1), ("vehicle", 1)]
    available = {name: [index for index, row in enumerate(rows)
                       if (not row["objects"] if name == "empty" else row.get(name, 0) > 0)]
                 for name, _ in buckets}
    if not available["empty"] or not any(row["objects"] for row in rows):
        raise ValueError("Bag must contain nonempty and empty frames")
    total_weight = sum(weight for name, weight in buckets if available[name])
    selected = set()
    for name, weight in buckets:
        candidates = available[name]
        quota = min(len(candidates), count * weight // total_weight)
        if quota:
            selected.update(candidates[int(offset)] for offset in np.linspace(0, len(candidates) - 1, quota))
    remaining = [index for index in range(len(rows)) if index not in selected]
    missing = count - len(selected)
    if missing:
        selected.update(remaining[int(offset)] for offset in np.linspace(0, len(remaining) - 1, missing))
    indices = sorted(selected)
    cohort = [rows[index] for index in indices]
    stamps = [row["stamp_ns"] for row in cohort]
    if len(set(stamps)) != len(stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Cohort requires unique strictly increasing original stamp tokens")
    return indices, cohort


def free_port(preferred):
    with socket.socket() as connection:
        try:
            connection.bind(("127.0.0.1", preferred))
        except OSError:
            connection.bind(("127.0.0.1", 0))
        return connection.getsockname()[1]


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--rates", type=float, nargs="+", choices=(1.0, 3.0), default=[1.0, 3.0])
    parser.add_argument("--master-port", type=int, default=11327)
    args = parser.parse_args()
    if not 100 <= args.frames <= 1000:
        parser.error("frames must be 100..1000")
    if not 1 <= args.master_port <= 65535:
        parser.error("master-port must be 1..65535")
    if args.output.exists():
        raise FileExistsError("Fresh output directory required")
    args.output.mkdir(parents=True)
    # rospy caches caller namespace at import, including through the installed
    # oracle node module. Set this before any ROS-related import occurs.
    os.environ["ROS_NAMESPACE"] = "/offline_probe"
    installed_dist = INSTALL / "lib/python3/dist-packages"
    sys.path.insert(0, str(installed_dist))
    import rosbag
    from autoware_perception_msgs.msg import TrackedObjects, PredictedObjects, ObjectClassification
    from perception_object_pipeline.map_based_prediction import LaneletNetwork, MapPredictionConfig
    from perception_object_pipeline.object_motion_policy import apply_object_motion_policy
    from perception_object_pipeline.kinematic_prediction import dominant_label, VEHICLE_LABELS
    import perception_object_pipeline.map_based_prediction as map_core
    node_path = INSTALL / "lib/perception_object_pipeline/map_based_prediction_node.py"
    policy_path = INSTALL / "lib/perception_object_pipeline/tracked_object_motion_policy_node.py"
    for path in (node_path, policy_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    if installed_dist not in Path(map_core.__file__).parents:
        raise RuntimeError("Pure oracle must also load installed package, not worktree module")
    map_node = load_module(node_path, "offline_installed_map_node")
    config = yaml.safe_load((PACKAGE / "config/map_based_prediction.yaml").read_text())
    map_file = ROOT / "kcity_map/lanelet_map/lanelet2_map_v87.osm"
    projector = ROOT / "kcity_map/lanelet_map/map_projector_info_v1.yaml"
    network = LaneletNetwork.load(map_file, projector, MapPredictionConfig(**config))
    horizons = map_node.build_prediction_horizons(0.1, 3.0)
    rows = []
    with rosbag.Bag(str(args.bag)) as bag:
        for _, message, receipt in bag.read_messages(topics=[INPUT_TOPIC]):
            if message.header.frame_id != "map":
                raise ValueError("Input must be the original map-frame tracking message")
            labels = [dominant_label(obj) for obj in message.objects]
            rows.append(dict(wire=wire(message), stamp_ns=message.header.stamp.to_nsec(),
                             receipt_ns=receipt.to_nsec(), objects=len(message.objects),
                             unknown=sum(label == ObjectClassification.UNKNOWN for label in labels),
                             pedestrian=sum(label == ObjectClassification.PEDESTRIAN for label in labels),
                             vehicle=sum(label in VEHICLE_LABELS for label in labels)))
    cadence_sec = float(np.median(np.diff([row["stamp_ns"] for row in rows]))) / 1.e9
    indices, rows = select_balanced_cohort(rows, args.frames)
    expected_policy, expected_map = {}, {}
    oracle_counts = Counter()
    pure_make_ms, gc_events, gc_starts = [], [], {}
    def gc_callback(phase, info):
        generation = info["generation"]
        if phase == "start":
            gc_starts[generation] = time.perf_counter()
        elif generation in gc_starts:
            gc_events.append(dict(generation=generation,
                                  ms=(time.perf_counter() - gc_starts.pop(generation)) * 1000.))
    gc.callbacks.append(gc_callback)
    try:
        for row in rows:
            message = TrackedObjects().deserialize(row["wire"])
            started = time.perf_counter()
            policy, constrained = apply_object_motion_policy(message)
            predicted, mapped, fallback, paths = map_node.make_map_based_predicted_objects(policy, horizons, network)
            oracle_counts.update(unknown_constrained=constrained, mapped=mapped, fallback=fallback, paths=paths)
            pure_make_ms.append((time.perf_counter() - started) * 1000.)
            expected_policy[row["stamp_ns"]] = wire(policy, True)
            expected_map[row["stamp_ns"]] = wire(predicted, True)
    finally:
        gc.callbacks.remove(gc_callback)
    port = free_port(args.master_port)
    environment = dict(os.environ, ROS_MASTER_URI=f"http://127.0.0.1:{port}",
                       ROS_IP="127.0.0.1", ROS_NAMESPACE="/offline_probe",
                       ROS_LOG_DIR=str(args.output / "roslogs"), AJOUNICE_ROOT=str(ROOT))
    environment.pop("ROS_HOSTNAME", None)
    environment["PYTHONPATH"] = str(installed_dist) + ":" + environment.get("PYTHONPATH", "")
    os.environ.update({key: value for key, value in environment.items() if key.startswith("ROS_")})
    os.environ.pop("ROS_HOSTNAME", None)
    import rospy
    import rosgraph
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    master_process = None
    node_processes = []
    streams = []
    owned = []
    report = dict(bag=str(args.bag), bag_sha256=digest(args.bag), cohort_original_frame_indices=indices,
        cohort_frames=len(rows), cohort_empty=sum(not row["objects"] for row in rows),
        cohort_objects=sum(row["objects"] for row in rows), cohort_unknown=sum(row["unknown"] for row in rows),
        cohort_vehicle=sum(row["vehicle"] for row in rows), cohort_pedestrian=sum(row["pedestrian"] for row in rows),
        wall_replay_cadence_sec=cadence_sec,
        original_duration_sec=(rows[-1]["stamp_ns"] - rows[0]["stamp_ns"]) / 1.e9,
        source_sha256={}, installed_sha256={}, master_port=port, cases=[],
        oracle_object_and_path_counts=dict(oracle_counts),
        pure_oracle_make_ms=stats(pure_make_ms), pure_oracle_gc_events=gc_events,
        child_node_gc_pause_not_instrumented=True,
        limits=["Private localhost ROS master and two installed nodes; no sensor/driver/localization/planner/control/GPU/MORAI.",
                "Recorded tracking/objects may already include UNKNOWN policy; not hidden raw EKF state replay.",
                "Test /clock is generated from selected object.header.stamp, NOT replayed from a bag /clock topic; independent TCP delivery is not callback-clock synchronization.",
                "Original selected header stamps stay unchanged; skipped gaps compress to the bag's median header cadence in wall replay.",
                "Stratified transport cohort, not original full-course timing/dynamics or a model-quality sample; wall scheduling is nondeterministic.",
                "Output sequence numbers normalize publisher-generated seq only; all other wire bytes are exact.",
                "Concurrent host GPU/CPU experiments mean this is not a performance certification.",
                "Node diagnostic callback CPU/wall excludes diagnostic JSON/publish; driver delivery wall includes ROS transport and scheduling.",
                "Only pure-oracle GC pauses observed; actual node GC is not directly attributed from tails."])
    source_to_install = {
        PACKAGE / "scripts/map_based_prediction_node.py": node_path,
        PACKAGE / "scripts/tracked_object_motion_policy_node.py": policy_path,
        PACKAGE / "src/perception_object_pipeline/map_based_prediction.py": Path(map_core.__file__),
        PACKAGE / "src/perception_object_pipeline/message_copy.py": installed_dist / "perception_object_pipeline/message_copy.py",
        PACKAGE / "src/perception_object_pipeline/object_motion_policy.py": installed_dist / "perception_object_pipeline/object_motion_policy.py",
        PACKAGE / "src/perception_object_pipeline/kinematic_prediction.py": installed_dist / "perception_object_pipeline/kinematic_prediction.py",
        PACKAGE / "src/perception_object_pipeline/node_processing_timing.py": installed_dist / "perception_object_pipeline/node_processing_timing.py",
        PACKAGE / "launch/object_pipeline.launch": INSTALL / "share/perception_object_pipeline/launch/object_pipeline.launch",
        PACKAGE / "launch/object_prediction.launch": INSTALL / "share/perception_object_pipeline/launch/object_prediction.launch",
    }
    for source, installed in source_to_install.items():
        report["source_sha256"][str(source.relative_to(ROOT))] = digest(source)
        report["installed_sha256"][str(installed.relative_to(ROOT))] = digest(installed)
    report["source_install_hash_equal"] = all(digest(source) == digest(installed)
                                                for source, installed in source_to_install.items())
    report["runner_sha256"] = digest(Path(__file__))
    report["map_sha256"] = digest(map_file)
    report["projector_sha256"] = digest(projector)
    report["config_sha256"] = digest(PACKAGE / "config/map_based_prediction.yaml")
    try:
        stream = (args.output / "master.log").open("w"); streams.append(stream)
        master_process = subprocess.Popen(["roscore", "-p", str(port)], env=environment,
                                          stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        owned.append(master_process)
        deadline = time.monotonic() + 15
        while True:
            if master_process.poll() is not None:
                raise RuntimeError("Private master failed to start")
            try:
                if xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"]).getPid("/offline_probe/qualification")[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise TimeoutError("Private master startup timeout")
            time.sleep(.05)
        xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"]).setParam("/offline_probe/qualification", "/use_sim_time", True)
        rospy.init_node("replay_pipeline_diagnostics", argv=[sys.argv[0], "__ns:=/offline_probe"], disable_signals=True)
        clock_pub = rospy.Publisher("/clock", Clock, queue_size=10, latch=True, tcp_nodelay=True)
        first_clock = rospy.Time(rows[0]["stamp_ns"] // 1_000_000_000, rows[0]["stamp_ns"] % 1_000_000_000)
        clock_pub.publish(Clock(clock=first_clock))
        for rate in args.rates:
            for enabled in (False, True):
                namespace = f"/offline_probe/rate{int(rate)}_{'on' if enabled else 'off'}"
                input_topic, policy_topic, output_topic = (namespace + part for part in ("/input", "/policy", "/predicted"))
                diag_topics = {"policy": namespace + "/policy_timing", "map": namespace + "/map_timing"}
                state = {"policy": {}, "map": {}, "diagnostics": {"policy": [], "map": []},
                         "duplicates": {"policy": [], "map": []}, "delivery_ms": {"policy": [], "map": []}}
                sent_at, lock = {}, threading.Lock()
                # Reset while prior nodes are stopped; new nodes receive a
                # correct latched initial clock instead of prior run's end.
                clock_pub.publish(Clock(clock=first_clock))
                deadline = time.monotonic() + 5
                while rospy.Time.now() != first_clock:
                    if time.monotonic() > deadline:
                        raise TimeoutError("Private simulated clock did not reset")
                    time.sleep(.005)
                def receive(message, stage):
                    token = message.header.stamp.to_nsec()
                    received_at = time.perf_counter()
                    payload = wire(message, True)
                    with lock:
                        if token in state[stage]:
                            state["duplicates"][stage].append(token)
                        state[stage][token] = payload
                        if token in sent_at:
                            state["delivery_ms"][stage].append((received_at - sent_at[token]) * 1000.)
                def diagnostic(message, stage):
                    with lock:
                        state["diagnostics"][stage].append(json.loads(message.data))
                subscribers = [rospy.Subscriber(policy_topic, TrackedObjects, lambda msg: receive(msg, "policy"), queue_size=1000),
                               rospy.Subscriber(output_topic, PredictedObjects, lambda msg: receive(msg, "map"), queue_size=1000)]
                for stage, topic in diag_topics.items():
                    subscribers.append(rospy.Subscriber(topic, String, lambda msg, s=stage: diagnostic(msg, s), queue_size=1000))
                input_pub = rospy.Publisher(input_topic, TrackedObjects, queue_size=1000)
                for stage, path, in_topic, out_topic in (("policy", policy_path, input_topic, policy_topic),
                                                       ("map", node_path, policy_topic, output_topic)):
                    name = namespace + "/" + stage
                    rospy.set_param(name + "/publish_processing_diagnostics", enabled)
                    if stage == "map":
                        for key, value in config.items():
                            rospy.set_param(name + "/" + key, value)
                        rospy.set_param(name + "/map_file", str(map_file))
                        rospy.set_param(name + "/projector_file", str(projector))
                        rospy.set_param(name + "/prediction_time_step", .1)
                        rospy.set_param(name + "/prediction_horizon", 3.)
                    stream = (args.output / f"rate{int(rate)}_{enabled}_{stage}.log").open("w"); streams.append(stream)
                    process = subprocess.Popen([sys.executable, str(path), f"__name:={stage}", f"__ns:={namespace}",
                        f"~input/objects:={in_topic}", f"~output/objects:={out_topic}",
                        f"~diagnostics/processing_time_ms:={diag_topics[stage]}"], env=environment,
                        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                    node_processes.append(process); owned.append(process)
                deadline = time.monotonic() + 30
                while not (input_pub.get_num_connections() and all(sub.get_num_connections() for sub in subscribers[:2])):
                    if any(process.poll() is not None for process in node_processes):
                        raise RuntimeError("Private installed node exited")
                    if time.monotonic() > deadline:
                        raise TimeoutError("Private pipeline connections timed out")
                    time.sleep(.05)
                master = rosgraph.Master("/offline_probe/replay_driver")
                graph = master.getSystemState()
                node_names = {node for groups in graph for _, nodes in groups for node in nodes}
                if any(not (name.startswith("/offline_probe/") or name == "/rosout") for name in node_names):
                    raise RuntimeError("Unexpected node in private master: " + repr(node_names))
                wall_start = time.perf_counter()
                for frame_index, row in enumerate(rows):
                    target = wall_start + frame_index * cadence_sec / rate
                    wait = target - time.perf_counter()
                    if wait > 0:
                        time.sleep(wait)
                    stamp = rospy.Time(row["stamp_ns"] // 1_000_000_000, row["stamp_ns"] % 1_000_000_000)
                    clock_pub.publish(Clock(clock=stamp))
                    message = TrackedObjects().deserialize(row["wire"])
                    with lock:
                        sent_at[row["stamp_ns"]] = time.perf_counter()
                    input_pub.publish(message)
                # Wall-clock drain works even though deterministic /clock is frozen.
                previous_count, stable_since = -1, time.monotonic()
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    with lock:
                        count = len(state["map"])
                    if count != previous_count:
                        previous_count, stable_since = count, time.monotonic()
                    if count == len(rows) or time.monotonic() - stable_since > 1:
                        break
                    time.sleep(.02)
                time.sleep(.2)
                for process in reversed(node_processes):
                    stop_process(process)
                node_processes.clear()
                for subscriber in subscribers:
                    subscriber.unregister()
                input_pub.unregister()
                policy_validation = validate_received(expected_policy, state["policy"])
                map_validation = validate_received(expected_map, state["map"])
                metrics = {}
                diagnostic_tokens = {}
                for stage in ("policy", "map"):
                    samples = state["diagnostics"][stage]
                    tokens = [sample["input_stamp_ns"] for sample in samples]
                    diagnostic_tokens[stage] = dict(unique=len(set(tokens)),
                        duplicates=len(tokens) - len(set(tokens)),
                        unexpected=sorted(set(tokens) - set(expected_policy)))
                    metrics[stage] = {key: stats([sample[key] for sample in samples if sample.get(key) is not None])
                        for key in ("message_make_ms", "synchronous_publish_ms", "callback_wall_ms_excluding_diagnostics",
                                    "callback_thread_cpu_ms_excluding_diagnostics", "callback_start_gap_ms", "input_header_age_ms")}
                case = dict(rate=rate, diagnostics=enabled, sent=len(rows), policy=policy_validation, map=map_validation,
                            duplicates=state["duplicates"], diagnostics_counts={key: len(value) for key, value in state["diagnostics"].items()},
                            diagnostic_tokens=diagnostic_tokens,
                            callback_diagnostics_ms=metrics, driver_delivery_wall_ms={key: stats(value) for key, value in state["delivery_ms"].items()},
                            graph_nodes=sorted(node_names))
                report["cases"].append(case)
                (args.output / f"rate{int(rate)}_{enabled}_diagnostics.json").write_text(json.dumps(state["diagnostics"], indent=2) + "\n")
                if not policy_validation["exact_subset"] or not map_validation["exact_subset"] or any(state["duplicates"].values()):
                    raise AssertionError("Private ROS wire/token regression")
                if any(value["duplicates"] or value["unexpected"] for value in diagnostic_tokens.values()):
                    raise AssertionError("Diagnostic token outside unique original input subset")
                if enabled and any(len(state["diagnostics"][key]) == 0 for key in ("policy", "map")):
                    raise AssertionError("Enabled diagnostics missing")
                if not enabled and any(state["diagnostics"].values()):
                    raise AssertionError("Default OFF unexpectedly published diagnostics")
                print(json.dumps(case), flush=True)
    finally:
        for process in reversed(node_processes):
            stop_process(process)
        if master_process is not None:
            stop_process(master_process)
        if rospy.core.is_initialized() and not rospy.is_shutdown():
            rospy.signal_shutdown("Private offline qualification complete")
        for stream in streams:
            stream.close()
        report["owned_processes"] = [process.pid for process in owned]
        report["alive_owned_processes_after_cleanup"] = [process.pid for process in owned if process.poll() is None]
        report["private_ros_process_cleanup_exact"] = not report["alive_owned_processes_after_cleanup"]
        (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"report": str(args.output / "report.json"),
                      "cleanup": report["private_ros_process_cleanup_exact"]}), flush=True)


if __name__ == "__main__":
    main()
