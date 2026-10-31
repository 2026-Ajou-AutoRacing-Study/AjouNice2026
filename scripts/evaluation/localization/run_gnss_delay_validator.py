#!/usr/bin/env python3
"""One-shot real-validator drive, small-topic rosbag and GNSS delay estimate.

This is a development calibration, not a production GNSS source or controller.
MORAI and roscore must already be connected. Existing manual stacks are never
stopped. The unchanged validator owns driving, recording and stop/P verification.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
from datetime import datetime, timezone
from urllib.parse import urlparse
import uuid
import xmlrpc.client

import yaml


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator"
SCENE = PACKAGE / "config/missions/gps_shadow/base/scenarios/hid_soa_baseline_00.json"
TOPIC_PROFILE = PACKAGE / "config/internal/topic_profiles/localization_analysis.json"
ANALYZER = ROOT / "scripts/evaluation/localization/analyze_gnss_delay.py"
CONFLICT_TOPICS = ("/Ego_topic", "/Object_topic", "/sensor/gps", "/ctrl_cmd",
                   "/planning/trajectory", "/localization/kinematic_state",
                   "/perception/object_recognition/detection/objects")


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def resolve(value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else ROOT / path).resolve()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def default_document():
    return {
        "schema_version": 2, "campaign_name": "gnss_delay_calibration",
        "scenarios": {"directory": str(SCENE.parent), "glob": SCENE.name, "repeat": 2},
        "stack": {"pose_source": "localization", "detector": "od_gt",
                  "traffic_light": False, "planviz": False},
        "recording": {"topic_profile": str(TOPIC_PROFILE), "bag_enabled": True,
                      "include_sensors": False, "minimum_free_disk_gib": 3.0,
                      "video": {"enabled": False}},
        "upload": {"enabled": False, "profile": "bag"},
        "default_case": {"goal": {"x": -13.382585757, "y": 93.782754520,
                                  "z": 28.489, "yaw_deg": 60.815},
                         "waypoints": [], "timeout_sec": 90, "arrival_mode": "stop"},
        "network": {"sensor_json": "configs/sensors/Sensor_competition_local.json",
                    "ego_network_json": str(PACKAGE / "config/internal/generated/ego_network_validator_local.json")},
        "morai": {"map_name": "R_KR_PR_K-city_2025", "scenario_install_dir": "auto",
                  "scenario_load_host": "127.0.0.1", "scenario_load_port": 9095},
        "validation": {"goal_tolerance_m": 2.0, "goal_hold_sec": 1.0,
                       "goal_speed_max_mps": .5, "topic_loss_timeout_sec": 4.0,
                       "stuck_timeout_sec": 0.0, "localization_safety": {"enabled": True}},
        "resume": False, "continue_on_system_error": False,
        "readiness_timeout_sec": 60, "scenario_ready_timeout_sec": 35,
        "route_ready_timeout_sec": 75,
    }


def campaign_document(args):
    document = (yaml.safe_load(resolve(args.config).read_text(encoding="utf-8"))
                if args.config else default_document())
    if not isinstance(document, dict):
        raise ValueError("--config must contain a schema_version: 2 campaign mapping")
    document = copy.deepcopy(document)
    stack = document.setdefault("stack", {})
    stack.update(pose_source="localization", detector="od_gt", traffic_light=False,
                 planviz=args.planviz)
    # Explicitly zero the old seconds argument to prevent double compensation.
    loc = stack.setdefault("localization_args", {})
    loc.update(initial_position_source="ego", gnss_motion_compensation_sec=0.0,
               gnss_delay_ms=args.gnss_delay_ms)
    recording = document.setdefault("recording", {})
    recording.update(bag_enabled=True, include_sensors=False, analysis_mode=False,
                     topic_profile=str(TOPIC_PROFILE), profile_preset="localization_analysis")
    recording.pop("selected_topics", None)
    recording.setdefault("video", {})["enabled"] = False
    document.setdefault("upload", {})["enabled"] = False
    document.setdefault("validation", {}).setdefault("localization_safety", {})["enabled"] = True
    document["validation"]["localization_safety"]["park_after_run"] = True
    document.setdefault("scenarios", {})["repeat"] = args.repeats
    document["scenarios"].pop("randomization", None)
    document.update(resume=False, continue_on_system_error=False)
    return document


def check_document(document):
    """Offline only: no ROS requests, directory creation or scenario installation."""
    sys.path.insert(0, str(PACKAGE / "src"))
    from planning_scenario_validator.config import validate_campaign_document, scenario_sources
    from planning_scenario_validator.network_contract import inspect_network_contract
    validate_campaign_document(document)
    sources = scenario_sources(document["scenarios"])
    for source in sources:
        scene = json.loads(source.read_text(encoding="utf-8"))
        if scene.get("mapInfo", {}).get("mapName") != document.get("morai", {}).get("map_name", "R_KR_PR_K-city_2025"):
            raise ValueError("Scenario map mismatch: {}".format(source))
        if any(scene.get(key) for key in ("vehicleList", "pedestrianList", "objectList", "spawnPointList", "pedspawnPointList", "aiPedstrianList")):
            raise ValueError("Calibration requires empty scenes, not live mission actors: {}".format(source))
    network = document["network"]
    ego = json.loads(resolve(network["ego_network_json"]).read_text(encoding="utf-8"))
    sensor = json.loads(resolve(network["sensor_json"]).read_text(encoding="utf-8"))
    report = inspect_network_contract(document, ego, sensor)
    if report["errors"]:
        raise ValueError("\n".join(report["errors"]))
    topics = json.loads(TOPIC_PROFILE.read_text(encoding="utf-8"))
    missing = {"/Ego_topic", "/Competition_topic", "/sensor/gps", "/sensor/imu",
               "/localization/gnss/odom", "/localization/gt/twist", "/tf", "/tf_static"} - set(topics)
    if missing:
        raise ValueError("Calibration profile misses required topics: {}".format(sorted(missing)))
    return sources, report


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3
        return connection


def master_preflight(uri=None):
    """Only inspect the already-running master; never create or terminate one."""
    uri = uri or os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    if urlparse(uri).scheme != "http":
        raise ValueError("ROS_MASTER_URI must be an explicit ROS1 http master URI")
    try:
        with xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport()) as master:
            pid = master.getPid("/gnss_delay_calibration")
            response = master.getSystemState("/gnss_delay_calibration")
    except (OSError, xmlrpc.client.Error) as error:
        raise ValueError("Existing roscore unavailable; start roscore and connect MORAI first: {}".format(error)) from error
    if pid[0] != 1 or response[0] != 1:
        raise ValueError("ROS master refused status request")
    owners = dict(response[2][0])
    conflicts = {topic: owners[topic] for topic in CONFLICT_TOPICS if owners.get(topic)}
    if conflicts:
        raise ValueError("Manual stack is active: {}. Stop its UDP/localization/OD/planning/control terminals normally; keep only roscore and MORAI. Nothing was killed.".format(conflicts))
    return {"uri": uri, "pid": pid[2], "checked_at": utc_now(), "conflicts": conflicts}


def make_inputs(document, output, sources, *, keep_gps_shadow=False):
    """Write fresh private inputs only; shared and frozen scenarios stay intact."""
    inputs = output / "inputs"
    scenes = inputs / "scenarios"
    scenes.mkdir(parents=True)
    hashes = {}
    for index, source in enumerate(sources):
        original = source.read_bytes()
        scene = json.loads(original)
        if not keep_gps_shadow:
            # Calibration requires visible GNSS for the complete moving window.
            # This is a private derived scene, never a mutation of shared scenes.
            scene["shadedAreaList"] = []
        payload = (json.dumps(scene, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
        destination = scenes / ("{:03d}_".format(index) + source.name)
        destination.write_bytes(payload)
        hashes[str(source)] = {"original_sha256": hashlib.sha256(original).hexdigest(),
                               "derived_sha256": hashlib.sha256(payload).hexdigest(),
                               "gps_shadow_removed": not keep_gps_shadow}
    document = copy.deepcopy(document)
    document["scenarios"].update(directory=str(scenes), glob="*.json")
    document["scenarios"].pop("include", None)
    document["campaign_name"] = "gnss_delay_calibration"
    document["output_root"] = str(output / "executions")
    path = inputs / "campaign.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return path, hashes


def worker(campaign):
    """Isolated ROS worker; use real validator lifecycle, but no master startup."""
    sys.path.insert(0, str(PACKAGE / "src"))
    from planning_scenario_validator import runner
    from planning_scenario_validator.executions import prepare_execution
    from planning_scenario_validator.runtime import RuntimeFailure

    def existing_master_only(_processes, _timeout):
        try:
            master_preflight()
        except ValueError as error:
            raise RuntimeFailure(str(error)) from error

    runner._start_master = existing_master_only
    existing_master_only(None, 0)
    config = prepare_execution(campaign)
    save_json(Path(campaign).parents[1] / "execution_pointer.json", {"execution": str(config["output"])})

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, interrupted)
    try:
        return runner.run(config, resume=False, rerun_failed=False)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print("GNSS validator worker failed: {}".format(error), file=sys.stderr, flush=True)
        return 1


def worker_command(campaign):
    # Immutable shell text; user paths are positional arguments, never shell code.
    return ["bash", "-c", 'set -e; master_uri="${ROS_MASTER_URI:-http://127.0.0.1:11311}"; source "$1"; export ROS_MASTER_URI="$master_uri"; exec /usr/bin/python3 "$2" --internal-worker "$3"',
            "gnss-calibration", str(ROOT / "scripts/workspace/source_aw149.sh"),
            str(Path(__file__).resolve()), str(campaign)]


def bounded_run(command, log, timeout, shutdown_timeout=120):
    """Interrupt only our worker; its validator performs braking/P/owned cleanup."""
    def stop_requested(_signum, _frame):
        raise KeyboardInterrupt()

    old_term = signal.getsignal(signal.SIGTERM)
    old_int = signal.getsignal(signal.SIGINT)
    old_hup = signal.getsignal(signal.SIGHUP)
    with Path(log).open("x", encoding="utf-8") as handle:
        process = subprocess.Popen(command, cwd=str(ROOT), stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        reason = None
        try:
            signal.signal(signal.SIGTERM, stop_requested)
            signal.signal(signal.SIGHUP, stop_requested)
            try:
                code = process.wait(timeout=timeout)
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
                reason = "TIMEOUT" if isinstance(error, subprocess.TimeoutExpired) else "USER_INTERRUPT"
                # A second stop request must not orphan the safety owner halfway
                # through brake/P/owned-process cleanup.
                signal.signal(signal.SIGINT, signal.SIG_IGN)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                signal.signal(signal.SIGHUP, signal.SIG_IGN)
                process.send_signal(signal.SIGINT)
                try:
                    code = process.wait(timeout=shutdown_timeout)
                except subprocess.TimeoutExpired:
                    # Do not SIGKILL a safety owner midway through braking or leave a
                    # false Park claim. A bounded caller returns with an explicit alarm.
                    return {"returncode": None, "reason": reason,
                            "shutdown": "UNCONFIRMED_WORKER_STILL_RUNNING", "pid": process.pid}
        finally:
            signal.signal(signal.SIGTERM, old_term)
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGHUP, old_hup)
        return {"returncode": code, "reason": reason, "shutdown": "WORKER_EXITED", "pid": process.pid}


def canonical_cases(execution):
    """Validator moves completed cases here; share_payload/backups are copies."""
    execution = resolve(execution)
    return sorted(path for category in ("success", "failure", "error")
                  for path in (execution / category).glob("*")
                  if path.is_dir() and not path.is_symlink())


def execution_bags(execution):
    execution = resolve(execution)
    bags = sorted(path for case in canonical_cases(execution)
                  for path in case.glob("*.bag") if path.is_file() and not path.is_symlink())
    if not bags:
        raise ValueError("No finalized .bag in execution (active or missing recording): {}".format(execution))
    return bags


def park_audit(execution, expected):
    receipts = [case / "safety_receipt.json" for case in canonical_cases(execution)
                if (case / "safety_receipt.json").is_file()]
    rows = []
    for path in receipts:
        value = json.loads(path.read_text(encoding="utf-8"))
        rows.append({"path": str(path), "enabled": value.get("enabled"),
                     "shutdown_status": value.get("shutdown", {}).get("status"),
                     "park_verified": value.get("shutdown", {}).get("stop", {}).get("park_verified")})
    passed = len(rows) == expected and all(row["enabled"] and row["shutdown_status"] == "VERIFIED"
                                         and row["park_verified"] is True for row in rows)
    return {"verified": passed, "expected_cases": expected, "receipts": rows}


def analyze(bags, output, args):
    analyzer_command = ["/usr/bin/python3", str(ANALYZER)] + [str(path) for path in bags] + [
        "--output-dir", str(output / "analysis"), "--heading-offset-deg", str(args.heading_offset_deg),
        "--acquisition", args.acquisition]
    # A subprocess's sourced environment does not return to its parent. Source
    # the ROS/message overlay again for bag decoding when Codex did not run ajou.
    command = ["bash", "-c", 'set -e; source "$1"; shift; exec "$@"', "gnss-delay-analysis",
               str(ROOT / "scripts/workspace/source_aw149.sh")] + analyzer_command
    completed = subprocess.run(command, cwd=str(ROOT), timeout=args.analysis_timeout_sec)
    return completed.returncode


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="Optional empty-scene campaign with this PC's network/goal; copied, never edited")
    parser.add_argument("--output-dir", help="New directory only; default unique planner_logs/analysis/gnss_delay_calibration")
    parser.add_argument("--check", action="store_true", help="Offline, read-only config/file checks; no drive or output creation")
    parser.add_argument("--dry-run", action="store_true", help="Read-only plan; no ROS connection or files created")
    parser.add_argument("--analyze-execution", help="Analyze finalized bags in a prior execution without starting ROS")
    parser.add_argument("--repeats", type=int, default=2, help="Independent same-route runs, 1..3 (default 2)")
    parser.add_argument("--gnss-delay-ms", type=float, default=0., help="Applied localization advance for OFF/ON validation; never automatically set from recommendation")
    parser.add_argument("--heading-offset-deg", type=float, default=-1.347, help="Offline MR Ego oracle correction, once only")
    parser.add_argument("--acquisition", default="MR", choices=("MR", "MRP", "LAN_FIELD", "OFFLINE_REPLAY"))
    parser.add_argument("--planviz", action="store_true", help="Match a loaded RViz environment; default OFF to reduce load")
    parser.add_argument("--keep-gps-shadow", action="store_true", help="Retain scene GPS shadows for explicit OFF/ON recovery trials; may leave insufficient calibration samples")
    parser.add_argument("--timeout-sec", type=float, default=600., help="Whole live-worker limit; then allow bounded safety cleanup")
    parser.add_argument("--analysis-timeout-sec", type=float, default=180.)
    parser.add_argument("--internal-worker", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not 1 <= args.repeats <= 3:
        parser.error("--repeats must be 1..3")
    if not math.isfinite(args.gnss_delay_ms) or not 0 <= args.gnss_delay_ms <= 100:
        parser.error("--gnss-delay-ms must be finite 0..100")
    if not math.isfinite(args.heading_offset_deg):
        parser.error("--heading-offset-deg must be finite")
    if any(not math.isfinite(value) or value <= 0 for value in (args.timeout_sec, args.analysis_timeout_sec)):
        parser.error("timeouts must be finite and positive")
    if not args.analyze_execution and args.acquisition != "MR":
        parser.error("Live validator uses MR/Competition; other acquisition labels are analyze-only")
    if not args.analyze_execution and abs(args.heading_offset_deg + 1.347) > 1e-9:
        parser.error("Live MR/Competition oracle requires heading offset -1.347, not a second runtime rotation")
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.internal_worker:
        return worker(args.internal_worker)
    manifest = {"schema_version": 1, "workflow": "gnss-delay-real-validator-v1",
                "started_at": utc_now(), "recommendation_applied_automatically": False,
                "applied_gnss_delay_ms": None if args.analyze_execution else args.gnss_delay_ms,
                "acquisition": args.acquisition,
                "gt_source": None if args.analyze_execution else "competition",
                "estimator_heading_offset_deg": None if args.analyze_execution else -1.347,
                "oracle_heading_offset_deg": args.heading_offset_deg,
                "time_policy": "ROS receipt stamps unchanged; use_sim_time=false",
                "startup_position_source": "ego development scene, not official START",
                "load_condition": {"detector": "od_gt", "rviz": args.planviz,
                                   "camera_lidar_bag": False, "sensor_generation_changed": False},
                "gps_shadow_removed_in_private_scene": not args.keep_gps_shadow,
                "scope": "Relative GNSS/Ego observation epoch; not physical network latency"}
    output = None
    try:
        if args.analyze_execution:
            bags = execution_bags(args.analyze_execution)
            document, sources = None, []
        else:
            document = campaign_document(args)
            sources, _report = check_document(document)
            bags = []
        if args.check or args.dry_run:
            print(json.dumps({"mode": "READ_ONLY", "runs": len(sources) * args.repeats,
                "scenario_sources": [str(path) for path in sources], "bags": [str(path) for path in bags],
                "applied_gnss_delay_ms": args.gnss_delay_ms,
                "note": "No ROS, file writes, driving or safety certification. Normal mode requires manual stacks OFF; roscore + MORAI only."}, ensure_ascii=False, indent=2))
            return 0
        if document is not None:
            manifest["master_preflight"] = master_preflight()
        destination = (resolve(args.output_dir) if args.output_dir else ROOT / "planner_logs/analysis/gnss_delay_calibration" /
                       (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8]))
        destination.mkdir(parents=True, exist_ok=False)
        output = destination  # Own only successfully created directories, never existing results.
        manifest["output_dir"] = str(output)
        save_json(output / "workflow.json", manifest)
        print("GNSS calibration output: {}".format(output), flush=True)
        if document is not None:
            campaign, manifest["source_scene_sha256"] = make_inputs(
                document, output, sources, keep_gps_shadow=args.keep_gps_shadow)
            save_json(output / "workflow.json", manifest)
            print("Real validator: localization {:.1f}ms, od_gt, {} run(s). Log: {}".format(
                args.gnss_delay_ms, len(sources) * args.repeats, output / "validator.log"), flush=True)
            manifest["worker"] = bounded_run(worker_command(campaign), output / "validator.log", args.timeout_sec)
            pointer = output / "execution_pointer.json"
            if pointer.is_file():
                execution = Path(json.loads(pointer.read_text())["execution"])
                manifest["execution"] = str(execution)
                manifest["park_audit"] = park_audit(execution, len(sources) * args.repeats)
            if (manifest["worker"]["returncode"] != 0 or
                    not manifest.get("park_audit", {}).get("verified")):
                raise ValueError("Drive/stop-P verification did not complete. No recommendation emitted; inspect validator.log and workflow.json. Do not assume vehicle is parked.")
            bags = execution_bags(execution)
        else:
            manifest["execution"] = str(resolve(args.analyze_execution))
            manifest["note"] = "Offline-only reanalysis. No new driving, Park claim or runtime parameter application."
        manifest["bags"] = [str(path) for path in bags]
        code = analyze(bags, output, args)
        manifest.update(status="ANALYZED" if code == 0 else "INSUFFICIENT" if code == 2 else "ANALYSIS_ERROR",
                        analyzer_returncode=code, completed_at=utc_now())
        save_json(output / "workflow.json", manifest)
        print("GNSS calibration output: {}".format(output), flush=True)
        return code
    except (OSError, ValueError, subprocess.TimeoutExpired, yaml.YAMLError) as error:
        if output is not None:
            manifest.update(status="ERROR", error=str(error), completed_at=utc_now())
            save_json(output / "workflow.json", manifest)
        print("GNSS calibration failed: {}".format(error), file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
