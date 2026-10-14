#!/usr/bin/env python3
"""Stream full saved tracks through an installed, bounded-heap private Map-CV.

The ignored child shim observes natural GC only and flushes on shutdown.
No production files, GC thresholds, collection policy, or output fields change.
"""
from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client

import yaml

from replay_pipeline_diagnostics_offline import ROOT, PACKAGE, INSTALL, INPUT_TOPIC, digest, wire, stats, free_port, load_module, stop_process


GC_SHIM = r'''#!/usr/bin/env python3
import gc, json, os, runpy, sys, time
from collections import deque

def rss():
    result = {}
    with open('/proc/self/status') as stream:
        for line in stream:
            key = line.split(':', 1)[0]
            if key in ('VmRSS', 'VmHWM'):
                result[key + '_kib'] = int(line.split()[1])
    return result

installed_node, trace_path = sys.argv[1:3]
sys.argv = [installed_node] + sys.argv[3:]
trace = deque(maxlen=10000)
starts = {}
total_events = 0
startup_rss = rss()
callback_first_rss = None
callback_count = 0
first_callback_start = None
last_callback_end = None

def gc_callback(phase, info):
    global total_events
    generation = info['generation']
    if phase == 'start':
        starts[generation] = time.perf_counter()
    elif generation in starts:
        ended = time.perf_counter()
        trace.append((generation, starts.pop(generation), ended))
        total_events += 1

gc.callbacks.append(gc_callback)
import rospy
trial_tcp_nodelay = os.environ.get('AJOUNICE_OFFLINE_TCP_NODELAY', 'false') == 'true'
trial_subscriber_requests = []

def make_trial_subscriber(original, enabled, requests):
    def subscribe(name, *args, **kwargs):
        if name == '~input/objects':
            kwargs['tcp_nodelay'] = enabled
            requests.append((name, enabled))
        return original(name, *args, **kwargs)
    return subscribe

# Only the installed Map-CV input subscription, never /clock or other topics.
rospy.Subscriber = make_trial_subscriber(rospy.Subscriber, trial_tcp_nodelay, trial_subscriber_requests)
from perception_object_pipeline.node_processing_timing import NodeProcessingTiming
original_finish = NodeProcessingTiming.finish

def timed_finish(token, *args, **kwargs):
    global callback_first_rss, callback_count, first_callback_start, last_callback_end
    result = original_finish(token, *args, **kwargs)
    # Diagnostic-only monotonic boundaries permit exact GC-overlap analysis.
    # No callback input/output is retained, and canonical objects are untouched.
    started = token[1]
    ended = started + result['callback_wall_ms_excluding_diagnostics'] / 1000.
    result['offline_callback_start_monotonic'] = started
    result['offline_callback_end_monotonic'] = ended
    result['offline_objects_publish_return_monotonic'] = args[2]
    if callback_count == 0:
        first_callback_start = started
        callback_first_rss = rss()
    last_callback_end = ended
    callback_count += 1
    return result

NodeProcessingTiming.finish = staticmethod(timed_finish)
try:
    runpy.run_path(installed_node, run_name='__main__')
finally:
    gc.callbacks.remove(gc_callback)
    ending_rss = rss()  # excludes JSON flush allocation; VmHWM is process peak.
    result = dict(pid=os.getpid(), gc_enabled=gc.isenabled(), gc_thresholds=gc.get_threshold(),
        trace_capacity=10000, total_gc_events=total_events,
        overwritten_gc_events=max(0, total_events-len(trace)), gc_events=list(trace),
        rss_startup=startup_rss, rss_first_callback=callback_first_rss, rss_end=ending_rss,
        callback_count=callback_count, first_callback_start=first_callback_start,
        last_callback_end=last_callback_end,
        trial_input_subscriber_requests=trial_subscriber_requests,
        trace_scope='Only immutable generation/time tuples; no input or output message retention. Natural GC, no threshold changes/forced collection. Trace and one RSS observation add observer work.')
    with open(trace_path, 'w') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
'''


class PendingExpectations:
    """Bounded parent wire oracle, never accumulates full input messages."""
    def __init__(self, capacity=512):
        self.capacity = capacity
        self.pending = OrderedDict()
        self.bytes = self.peak_bytes = self.peak_count = 0
        self.evicted = []
        self.evicted_count = 0

    def put(self, token, payload, sent_at):
        if token in self.pending:
            raise ValueError("Input stamp token repeated")
        while len(self.pending) >= self.capacity:
            old_token, (old_payload, _) = self.pending.popitem(last=False)
            self.bytes -= len(old_payload)
            self.evicted_count += 1
            if len(self.evicted) < 64:
                self.evicted.append(old_token)
        self.pending[token] = payload, sent_at
        self.bytes += len(payload)
        self.peak_bytes = max(self.bytes, self.peak_bytes)
        self.peak_count = max(len(self.pending), self.peak_count)

    def pop(self, token):
        result = self.pending.pop(token, None)
        if result is not None:
            self.bytes -= len(result[0])
        return result


def overlap(start, end, events):
    return [event for event in events if event[1] < end and event[2] > start]


class HeaderPacer:
    """Start wall replay only after the first bag frame/oracle is ready."""
    def __init__(self, rate):
        self.rate = rate
        self.first_stamp = self.started = None

    def target(self, stamp_ns, ready_wall):
        if self.first_stamp is None:
            self.first_stamp, self.started = stamp_ns, ready_wall
        return self.started + (stamp_ns - self.first_stamp) / 1.e9 / self.rate


class PairedTimingLedger:
    """Bounded scalar joins; retain only aggregate scalars and top four tails."""
    def __init__(self, capacity=512, tail_capacity=4):
        self.capacity, self.tail_capacity = capacity, tail_capacity
        self.pending = OrderedDict()
        self.metrics = {}
        self.tails = []
        self.completed = self.evicted = self.peak_count = 0

    def put(self, token, **fields):
        if token not in self.pending:
            while len(self.pending) >= self.capacity:
                self.pending.popitem(last=False)
                self.evicted += 1
            self.pending[token] = {}
        self.pending[token].update(fields)
        self.peak_count = max(self.peak_count, len(self.pending))
        required = ("send_start", "publish_return", "observer_entry", "wire_verify_end",
                    "callback_start", "callback_end", "objects_publish_return")
        record = self.pending[token]
        if not all(name in record for name in required):
            return
        self.pending.pop(token)
        self.completed += 1
        deltas = {
            "sender_publish_call": record["publish_return"] - record["send_start"],
            "send_to_callback_start": record["callback_start"] - record["send_start"],
            "callback_start_to_objects_publish_return": record["objects_publish_return"] - record["callback_start"],
            "objects_publish_return_to_observer_entry": record["observer_entry"] - record["objects_publish_return"],
            "observer_entry_to_wire_verify": record["wire_verify_end"] - record["observer_entry"],
            "callback_body": record["callback_end"] - record["callback_start"],
            "callback_end_to_observer_entry": record["observer_entry"] - record["callback_end"],
            "send_to_wire_verify": record["wire_verify_end"] - record["send_start"],
        }
        metrics = {name: value * 1000. for name, value in deltas.items()}
        for name, value in metrics.items():
            self.metrics.setdefault(name, []).append(value)
        sample = dict(token_ns=token, monotonic_seconds=dict(record), paired_ms=metrics)
        if len(self.tails) < self.tail_capacity:
            self.tails.append(sample)
        elif metrics["send_to_wire_verify"] > min(row["paired_ms"]["send_to_wire_verify"] for row in self.tails):
            smallest = min(range(len(self.tails)), key=lambda index: self.tails[index]["paired_ms"]["send_to_wire_verify"])
            self.tails[smallest] = sample


def worker(args):
    output = args.output
    output.mkdir(parents=True)
    port = free_port(args.master_port)
    environment = dict(os.environ, ROS_MASTER_URI=f"http://127.0.0.1:{port}", ROS_IP="127.0.0.1",
        ROS_NAMESPACE="/offline_probe", ROS_LOG_DIR=str(output / "roslogs"), AJOUNICE_ROOT=str(ROOT))
    environment.pop("ROS_HOSTNAME", None)
    environment["AJOUNICE_OFFLINE_TCP_NODELAY"] = args.trial_tcp_nodelay
    dist = INSTALL / "lib/python3/dist-packages"
    environment["PYTHONPATH"] = str(dist) + ":" + environment.get("PYTHONPATH", "")
    os.environ.update({key: value for key, value in environment.items() if key.startswith("ROS_")})
    os.environ.pop("ROS_HOSTNAME", None)
    sys.path.insert(0, str(dist))
    import rosbag, rosgraph, rospy
    from rosgraph_msgs.msg import Clock
    from autoware_perception_msgs.msg import TrackedObjects, PredictedObjects
    from std_msgs.msg import String
    from perception_object_pipeline.map_based_prediction import LaneletNetwork, MapPredictionConfig
    import perception_object_pipeline.map_based_prediction as core
    node = INSTALL / "lib/perception_object_pipeline/map_based_prediction_node.py"
    module = load_module(node, "installed_stream_oracle")
    source_config = PACKAGE / "config/map_based_prediction.yaml"
    installed_config = INSTALL / "share/perception_object_pipeline/config/map_based_prediction.yaml"
    if digest(source_config) != digest(installed_config):
        raise RuntimeError("Installed/source map config mismatch")
    config = yaml.safe_load(installed_config.read_text())
    map_file = ROOT / "kcity_map/lanelet_map/lanelet2_map_v85.osm"
    projector = ROOT / "kcity_map/lanelet_map/map_projector_info_v1.yaml"
    network = LaneletNetwork.load(map_file, projector, MapPredictionConfig(**config))
    horizons = module.build_prediction_horizons(.1, 3.)
    shim = output / "gc_observer_shim.py"
    shim.write_text(GC_SHIM)
    report = dict(rate=args.worker_rate, bag=str(args.bag), bag_sha256=digest(args.bag),
        trial_tcp_nodelay_input=args.trial_tcp_nodelay == "true", trial_tcp_nodelay_output=args.trial_tcp_nodelay == "true",
        master_uri=environment["ROS_MASTER_URI"], gc_modified=False, production_modified=False,
        source_node_sha256=digest(PACKAGE / "scripts/map_based_prediction_node.py"),
        installed_node_sha256=digest(node), installed_core_sha256=digest(Path(core.__file__)),
        source_config_sha256=digest(source_config), installed_config_sha256=digest(installed_config),
        installed_module_path=core.__file__, map_sha256=digest(map_file),
        shim_sha256=digest(shim), helper_sha256=digest(Path(__file__)),
        limits=["Private localhost master + one actual installed Map-CV node; no production nodes, driver, localization, planner, control, GPU or MORAI.",
                "Original consecutive tracked bag headers generate test /clock; independent TCP clock receipt is not sensor-age synchronization.",
                "Original header timing paced by rate; child retains only real callback/cache plus bounded immutable GC tuples, not messages.",
                "Parent streams bag and retains at most512 wire expectations; token/count metrics retain primitive values only.",
                "Paired timing joins retain at most512 tokens plus top4 completed tails; aggregate metrics retain scalar durations, never full-token/message history.",
                "parent_send_to_observer_wire_verify includes parent observer scheduling/serialization/oracle comparison, not actual downstream node delivery completion.",
                "Callback end is after objects publication/logging, so callback-end -> observer-entry can be negative; objects-publish-return is only a local publish-call boundary, not network completion.",
                "GC/shim and diagnostic observers add small work; normal GC stays enabled with unchanged thresholds and no forced collection.",
                "Host load is uncontrolled; wall/CPU distributions are not performance certification."])
    master = child = None
    streams, owned = [], []
    pending = PendingExpectations()
    lock = threading.Lock()
    timing = PairedTimingLedger()
    errors, delivery = [], []
    diagnostic_metrics = {name: [] for name in ("wall_excluding_diagnostics", "thread_cpu_excluding_diagnostics", "message_make", "synchronous_publish")}
    diagnostic_spikes = []
    last_received = last_diagnostic = None
    counts = Counter()
    def record_error(kind, token):
        counts["errors"] += 1
        if len(errors) < 64:
            errors.append((kind, token))
    def receive(message):
        nonlocal last_received
        observer_entry = time.perf_counter()
        token = message.header.stamp.to_nsec()
        payload = wire(message, True)
        with lock:
            if last_received is not None and token <= last_received:
                record_error("duplicate_or_out_of_order", token); return
            expected = pending.pop(token)
            if expected is None:
                record_error("unexpected_or_unverified_late_token", token); return
            if payload != expected[0]:
                record_error("wire_mismatch", token)
            verified_end = time.perf_counter()
            last_received = token
            counts["received"] += 1
            delivery.append((verified_end - expected[1]) * 1000.)
            timing.put(token, observer_entry=observer_entry, wire_verify_end=verified_end)
            counts["received_nonempty"] += bool(message.objects)
    def diagnostic(message):
        nonlocal last_diagnostic
        value = json.loads(message.data)
        # Scalar samples only; no nested object or path messages are retained.
        with lock:
            token = value["input_stamp_ns"]
            if last_diagnostic is not None and token <= last_diagnostic:
                record_error("duplicate_or_out_of_order_diagnostic", token); return
            last_diagnostic = token
            counts["diagnostics"] += 1
            for target, source in (("wall_excluding_diagnostics", "callback_wall_ms_excluding_diagnostics"),
                ("thread_cpu_excluding_diagnostics", "callback_thread_cpu_ms_excluding_diagnostics"),
                ("message_make", "message_make_ms"), ("synchronous_publish", "synchronous_publish_ms")):
                diagnostic_metrics[target].append(value[source])
            timing.put(token, callback_start=value["offline_callback_start_monotonic"],
                callback_end=value["offline_callback_end_monotonic"],
                objects_publish_return=value["offline_objects_publish_return_monotonic"])
            if value["callback_wall_ms_excluding_diagnostics"] >= 20:
                counts["spikes_gt20ms"] += 1
                diagnostic_spikes.append(tuple(value.get(key) for key in (
                    "input_stamp_ns", "offline_callback_start_monotonic", "offline_callback_end_monotonic",
                    "callback_wall_ms_excluding_diagnostics", "callback_thread_cpu_ms_excluding_diagnostics", "input_objects")))
                diagnostic_spikes.sort(key=lambda row: -row[3])
                del diagnostic_spikes[4:]
    try:
        stream = (output / "master.log").open("w"); streams.append(stream)
        master = subprocess.Popen(["roscore", "-p", str(port)], env=environment,
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True); owned.append(master)
        deadline = time.monotonic() + 15
        while True:
            if master.poll() is not None:
                raise RuntimeError("Private master exited")
            try:
                if xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"]).getPid("/offline_probe/stream")[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise TimeoutError("Private master startup timeout")
            time.sleep(.05)
        xmlrpc.client.ServerProxy(environment["ROS_MASTER_URI"]).setParam("/offline_probe/stream", "/use_sim_time", True)
        rospy.init_node("map_gc_stream", argv=[sys.argv[0], "__ns:=/offline_probe"], disable_signals=True)
        namespace = "/offline_probe/map_gc"
        for key, value in config.items():
            rospy.set_param(namespace + "/" + key, value)
        for key, value in dict(map_file=str(map_file), projector_file=str(projector),
            prediction_time_step=.1, prediction_horizon=3., publish_processing_diagnostics=True).items():
            rospy.set_param(namespace + "/" + key, value)
        input_topic, output_topic, timing_topic = "/offline_probe/input", "/offline_probe/predicted", "/offline_probe/timing"
        input_pub = rospy.Publisher(input_topic, TrackedObjects, queue_size=10)
        clock_pub = rospy.Publisher("/clock", Clock, queue_size=10, latch=True, tcp_nodelay=True)
        subscribers = [rospy.Subscriber(output_topic, PredictedObjects, receive, queue_size=1000,
                                       tcp_nodelay=args.trial_tcp_nodelay == "true"),
                       rospy.Subscriber(timing_topic, String, diagnostic, queue_size=1000)]
        stream = (output / "child.log").open("w"); streams.append(stream)
        trace_file = output / "child_gc_trace.json"
        child = subprocess.Popen([sys.executable, str(shim), str(node), str(trace_file),
            "__name:=map_gc", "__ns:=/offline_probe", f"~input/objects:={input_topic}",
            f"~output/objects:={output_topic}", f"~diagnostics/processing_time_ms:={timing_topic}"],
            env=environment, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True); owned.append(child)
        deadline = time.monotonic() + 30
        while not (input_pub.get_num_connections() and all(sub.get_num_connections() for sub in subscribers)):
            if child.poll() is not None:
                raise RuntimeError("Observed installed child exited")
            if time.monotonic() > deadline:
                raise TimeoutError("Private child connections timed out")
            time.sleep(.05)
        graph = rosgraph.Master("/offline_probe/map_gc_stream").getSystemState()
        graph_nodes = {name for groups in graph for _, names in groups for name in names}
        if any(not (name.startswith("/offline_probe/") or name == "/rosout") for name in graph_nodes):
            raise RuntimeError("Unexpected private graph node")
        report["graph_nodes"] = sorted(graph_nodes)
        previous_stamp = None
        pacer = HeaderPacer(args.worker_rate)
        with rosbag.Bag(str(args.bag)) as bag:
            report["bag_tracking_messages"] = bag.get_type_and_topic_info().topics[INPUT_TOPIC].message_count
            for frame_index, (_, message, _) in enumerate(bag.read_messages(topics=[INPUT_TOPIC])):
                if frame_index < args.skip_frames:
                    counts["skipped_original_frames"] += 1; continue
                if args.limit and counts["sent"] >= args.limit:
                    break
                if message.header.frame_id != "map":
                    counts["rejected_frame_id"] += 1; continue
                token = message.header.stamp.to_nsec()
                if previous_stamp is not None and token <= previous_stamp:
                    raise ValueError("Non-increasing original header tokens")
                previous_stamp = token
                expected, mapped, fallback, paths = module.make_map_based_predicted_objects(message, horizons, network)
                payload = wire(expected, True)
                target = pacer.target(token, time.perf_counter())
                delay = target - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                clock_pub.publish(Clock(clock=message.header.stamp))
                with lock:
                    send_start = time.perf_counter()
                    pending.put(token, payload, send_start)
                    timing.put(token, send_start=send_start)
                input_pub.publish(message)
                publish_return = time.perf_counter()
                with lock:
                    timing.put(token, publish_return=publish_return)
                counts.update(sent=1, sent_nonempty=bool(message.objects), objects=len(message.objects),
                              mapped=mapped, fallback=fallback, paths=paths)
                if counts["sent"] % 1000 == 0:
                    print(json.dumps(dict(rate=args.worker_rate, sent=counts["sent"], received=counts["received"])), flush=True)
        report["replay_wall_sec"] = time.perf_counter() - pacer.started
        deadline, previous_count, stable = time.monotonic() + 10, -1, time.monotonic()
        while time.monotonic() < deadline:
            with lock:
                count = counts["received"]
            if count != previous_count:
                previous_count, stable = count, time.monotonic()
            if count == counts["sent"] or time.monotonic() - stable > 1:
                break
            time.sleep(.02)
        time.sleep(.2)
        stop_process(child)
        if not trace_file.is_file():
            raise RuntimeError("Child shutdown trace did not flush")
        trace = json.loads(trace_file.read_text())
        runtime_gc = [event for event in trace["gc_events"] if trace["first_callback_start"] is not None
            and trace["first_callback_start"] <= event[1] <= trace["last_callback_end"]]
        spikes = []
        for row in diagnostic_spikes:
            associated = overlap(row[1], row[2], runtime_gc)
            spikes.append(dict(stamp_ns=row[0], callback_wall_ms=row[3], callback_thread_cpu_ms=row[4],
                input_objects=row[5], overlapping_gc=[dict(generation=event[0], ms=(event[2]-event[1])*1000.) for event in associated]))
        tails = sorted(timing.tails, key=lambda row: -row["paired_ms"]["send_to_wire_verify"])
        for row in tails:
            bounds = row["monotonic_seconds"]
            row["overlapping_child_gc"] = [dict(generation=event[0], ms=(event[2]-event[1])*1000.)
                for event in overlap(bounds["send_start"], bounds["wire_verify_end"], runtime_gc)]
        report.update(counts=dict(counts), received=counts["received"], missing_count=counts["sent"] - counts["received"],
            missing_tokens_sample=list(pending.pending)[:64] + pending.evicted[:64],
            exact_received_token_wire_subset=not errors, errors=errors,
            diagnostic_count=counts["diagnostics"], callback_count=trace["callback_count"],
            callback_ms={key: stats(values) for key, values in diagnostic_metrics.items()},
            parent_send_to_observer_wire_verify_ms=stats(delivery),
            paired_timing_ms={key: stats(values) for key, values in timing.metrics.items()},
            paired_completed=timing.completed, paired_incomplete=len(timing.pending),
            paired_evicted=timing.evicted, paired_pending_peak=timing.peak_count,
            paired_tail_examples=tails,
            child_rss=dict(startup=trace["rss_startup"], first_callback=trace["rss_first_callback"], end=trace["rss_end"]),
            natural_gc_runtime_ms={str(generation): stats([(event[2]-event[1])*1000. for event in runtime_gc if event[0] == generation])
                                   for generation in range(3)},
            natural_gc_startup_event_count=len(trace["gc_events"]) - len(runtime_gc),
            gc_trace_capacity=trace["trace_capacity"], gc_trace_overwritten=trace["overwritten_gc_events"],
            callback_spikes_gt20ms=spikes,
            parent_pending_peak_count=pending.peak_count, parent_pending_peak_bytes=pending.peak_bytes,
            pending_evicted=pending.evicted_count, child_gc_thresholds=trace["gc_thresholds"], child_gc_enabled=trace["gc_enabled"])
        report["trial_input_subscriber_requests"] = trace["trial_input_subscriber_requests"]
        if errors:
            raise AssertionError("Private streaming output regression")
    finally:
        if child is not None:
            stop_process(child)
        if master is not None:
            stop_process(master)
        if rospy.core.is_initialized() and not rospy.is_shutdown():
            rospy.signal_shutdown("Private streaming GC qualification finished")
        for stream in streams:
            stream.close()
        groups = {process.pid for process in owned}
        members = []
        for row in subprocess.check_output(["ps", "-eo", "pid=,pgid=,args="], text=True).splitlines():
            fields = row.strip().split(None, 2)
            if len(fields) >= 2 and int(fields[1]) in groups:
                members.append(row.strip())
        report["owned_pids"] = sorted(groups)
        report["alive_owned_group_members_after_cleanup"] = members
        report["cleanup_exact"] = not members and all(process.poll() is not None for process in owned)
        (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(dict(rate=args.worker_rate, report=str(output / "report.json"), cleanup=report["cleanup_exact"])), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--master-port", type=int, default=11327)
    parser.add_argument("--limit", type=int, default=0, help="0 streams all; optional bounded debug scope")
    parser.add_argument("--skip-frames", type=int, default=0, help="Original leading frames skipped before paced admission")
    parser.add_argument("--trial-tcp-nodelay", choices=("false", "true"), default="false",
                        help="Private input/output subscriber request only; default false, never modifies installed source.")
    parser.add_argument("--worker-rate", type=float, choices=(1., 3.), help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 1 <= args.master_port <= 65535 or args.limit < 0 or args.skip_frames < 0:
        parser.error("Invalid private port/limit")
    if args.output.exists():
        raise FileExistsError("Fresh output directory required")
    if args.worker_rate is not None:
        worker(args)
        return
    args.output.mkdir(parents=True)
    cases = []
    for rate in (1, 3):
        output = args.output / f"rate{rate}"
        subprocess.run([sys.executable, str(Path(__file__)), "--bag", str(args.bag), "--output", str(output),
            "--master-port", str(args.master_port), "--limit", str(args.limit), "--skip-frames", str(args.skip_frames),
            "--trial-tcp-nodelay", args.trial_tcp_nodelay, "--worker-rate", str(rate)], check=True)
        cases.append(json.loads((output / "report.json").read_text()))
    report = dict(cases=cases, live_like_child_heap=True, performance_certification=False,
        source_changed=False, gc_changed=False, helper_sha256=digest(Path(__file__)))
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
