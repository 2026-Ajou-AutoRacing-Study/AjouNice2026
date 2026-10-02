"""시나리오 폴더를 순회하고 ROS 주행·녹화·판정을 재개 가능하게 실행한다."""
import argparse
import csv
import fcntl
import hashlib
import json
import os
import shutil
import shlex
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from .config import ConfigError, ROOT, load_campaign
from .traffic_light import runtime_mode, runtime_settings
from .failure_analysis import analyze_failure, compact_summary, log_offsets, scan_log_signals
from .runtime import (BrakeHold, LiveMonitor, Processes, Recording, RuntimeFailure,
                      ScenarioFailure,
                      atomic_json, check_no_conflicting_publishers,
                      arrange_split_windows, focus_planviz, master_alive, now, prepare_scenario, publish_brake,
                      request_route)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="MORAI planning 시나리오 자동 검증")
    parser.add_argument("config", help="캠페인 YAML 한 개")
    parser.add_argument("--check", action="store_true", help="ROS 실행 없이 설정과 파일만 검사")
    parser.add_argument("--dry-run", action="store_true", help="실행할 순서와 설정을 출력")
    parser.add_argument("--resume", choices=["true", "false"], help="중단된 캠페인 재개 여부")
    parser.add_argument("--rerun-failed", action="store_true", help="이전 실패 시나리오도 다시 실행")
    return parser.parse_args(argv)


def _stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _bool(value):
    return "true" if value else "false"


def _launch(package, filename, **arguments):
    return ["roslaunch", package, filename] + ["{}:={}".format(key, _bool(value) if isinstance(value, bool) else value)
                                                  for key, value in arguments.items()]


def _env():
    env = os.environ.copy()
    env["AJOUNICE_ROOT"] = str(ROOT)
    env.setdefault("PYTHONNOUSERSITE", "1")
    env["DISABLE_ROS1_EOL_WARNINGS"] = "1"
    return env


def _check_traffic_light(cfg):
    if not cfg["stack"].get("traffic_light", False):
        return
    python, device = runtime_settings(cfg["stack"], ROOT, check_exists=True)
    command = [python] + (["-s"] if runtime_mode(cfg["stack"]) == "virtualenv" else [])
    command += [str(ROOT / "scripts/runtime/check_traffic_light_env.py"),
                "--device", device, "--check-ros"]
    if runtime_mode(cfg["stack"]) == "virtualenv":
        command.append("--isolate-python")
    print("신호등 실행 환경 검사: {} / device={}".format(python, device), flush=True)
    try:
        checked = subprocess.run(command, env=_env(), stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RuntimeFailure("신호등 환경 검사 실행 실패: {}".format(error)) from error
    (cfg["output"] / "stack_logs").mkdir(parents=True, exist_ok=True)
    (cfg["output"] / "stack_logs/traffic_light_preflight.log").write_text(checked.stdout, encoding="utf-8")
    print(checked.stdout, flush=True)
    if checked.returncode:
        raise RuntimeFailure("신호등 환경 검사가 실패했습니다. 차량 주행은 시작하지 않았습니다. 환경 설치 가이드를 확인하세요")


def _start_master(processes, monitor_timeout):
    if master_alive():
        check_no_conflicting_publishers()
        return
    processes.start("roscore", ["roscore"])
    deadline = time.monotonic() + monitor_timeout
    while time.monotonic() < deadline:
        if master_alive():
            return
        time.sleep(0.2)
    raise RuntimeFailure("roscore가 준비되지 않았습니다")


def _start_persistent(cfg, processes, monitor):
    network, stack = cfg["network"], cfg["stack"]
    sensor = network["sensor_json"]
    processes.start("udp", _launch("sensor_interface", "morai_udp_interface.launch",
        sensor_config=sensor, network_config=network["ego_network_json"],
        enable_lidar=network["has_lidar"]))
    processes.start("sensing", _launch("bringup", "sensing.launch", sensor_config=sensor,
        publish_lidar_tf=network["has_lidar"], publish_camera_tf=network["camera_count"] > 0))
    timeout = float(cfg["document"].get("readiness_timeout_sec", 45))
    monitor.wait(lambda: monitor.stamps.get("ego", 0) > 0, timeout, "/Ego_topic")
    scenario_host = cfg["document"].get("morai", {}).get("scenario_load_host", "127.0.0.1")
    scenario_port = int(cfg["document"].get("morai", {}).get("scenario_load_port", 9095))
    if scenario_host in ("127.0.0.1", "localhost") and shutil.which("ss"):
        bound = subprocess.run(["ss", "-H", "-lun", "sport", "=", ":{}".format(scenario_port)],
                               capture_output=True, text=True, timeout=5)
        if bound.returncode != 0 or not bound.stdout.strip():
            raise RuntimeFailure("MORAI가 ScenarioLoad UDP {}을 열지 않았습니다. MR GUI Ego Network에 ego_network_validator_local.json을 다시 Load/Connect하세요".format(scenario_port))
    if stack["detector"] == "bevfusion":
        processes.start("bevfusion", ["bash", str(ROOT / "scripts/runtime/run_bevfusion.sh"), str(sensor)])
    if stack.get("planviz", True):
        processes.start("planviz", _launch("bringup", "aw149_planning_rviz.launch"))


def _start_case_stack(cfg, processes, monitor):
    stack, network = cfg["stack"], cfg["network"]
    timeout = float(cfg["document"].get("readiness_timeout_sec", 45))
    after = time.monotonic()
    if stack["pose_source"] == "localization":
        processes.start("pose", _launch("bringup", "localization_gnss_imu_gt_ctra.launch",
            gt_source="competition", use_heading_correction=True, heading_yaw_offset_deg=-1.347,
            publish_sensor_tf=False, publish_control_state=True,
            use_quantized_gnss_control_state=False, control_translation_tau_sec=0.5,
            control_yaw_tau_sec=0.5, sensor_config=network["sensor_json"]))
    else:
        processes.start("pose", _launch("bringup", "maptf_morai.launch",
            ego_topic="/Ego_topic", start_lanelet_map=False))
    monitor.wait_odom(after=after, timeout=timeout)
    if stack["detector"] == "od_gt":
        processes.start("detector", _launch("perception_object_pipeline", "morai_gt_detection.launch"))
    processes.start("tracking", _launch("perception_object_pipeline", "object_pipeline.launch",
        start_detector=False, prediction_model="map_based",
        maximum_tracking_internal_prediction_age_sec=0.5))
    if stack.get("traffic_light", False):
        python, device = runtime_settings(stack, ROOT, check_exists=True)
        processes.start("traffic_light", _launch("traffic_light_detection", "traffic_light_node.launch",
            inference_python=shlex.quote(python), device=device,
            isolate_python=runtime_mode(stack) == "virtualenv"))
    processes.start("planning", _launch("ajounice_planning_integration", "planning.launch",
        repo_root=ROOT, start_requester=False, is_simulation=False,
        behavior_path_forward_length=400.0, intersection_attention_area_length=100.0,
        enable_traffic_light_planner=stack.get("traffic_light", False),
        publish_fixed_autonomous_mode=True, use_ground_pointcloud=False,
        use_occupancy_grid=False))


CASE_STACK = ["controller", "planning", "traffic_light", "tracking", "detector", "pose"]


def _load_scene(cfg, installed, monitor, ego_xy):
    from trajectory_batch_runner.morai_udp_client import build_scenario_load_payload
    request = {"delete_all": False, "load_network_connection_data": True,
               "load_ego_vehicle_data": True, "load_surrounding_vehicle_data": True,
               "load_pedestrian_data": True, "load_obstacle_data": True, "set_pause": False}
    payload = build_scenario_load_payload(installed.stem, request)
    udp = cfg["document"].get("morai", {})
    host, port = udp.get("scenario_load_host", "127.0.0.1"), int(udp.get("scenario_load_port", 9095))
    timeout = float(cfg["document"].get("scenario_ready_timeout_sec", 35))
    for attempt in range(1, 4):
        baseline_xy = None if monitor.ego is None else (
            float(monitor.ego.position.x), float(monitor.ego.position.y))
        after = time.monotonic()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            sent = connection.sendto(payload, (host, port))
        if sent != len(payload):
            raise RuntimeFailure("ScenarioLoad UDP packet이 일부만 송신됐습니다")
        try:
            monitor.wait_scenario(ego_xy, after=after, timeout=timeout,
                                  tolerance=2.0, baseline_xy=baseline_xy)
            return
        except RuntimeFailure:
            if attempt == 3:
                raise RuntimeFailure("ScenarioLoad 3회 실패. MORAI GUI Ego Network에 validator 전용 JSON을 Load/Connect했는지 확인하세요")
            print("ScenarioLoad Ego 위치 미확인: {}/3회, 재시도".format(attempt), flush=True)


def _record_result(cfg, state, case, result, folder):
    state["results"][case.name] = {"source_sha256": case.source_sha256,
        "status": result["status"], "reason": result["reason"],
        "failure_category": result["failure_analysis"]["category"],
        "suspected_stage": result["failure_analysis"]["suspected_stage"],
        "result_path": str(folder / (result["basename"] + ".json")),
        "attempts": int(state["results"].get(case.name, {}).get("attempts", 0)) + 1}
    state["updated_at"] = now()
    _write_state(cfg, state)


def _write_state(cfg, state):
    atomic_json(cfg["output"] / "campaign_state.json", state)
    path = cfg["output"] / "summary.csv"
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scenario", "status", "reason", "failure_category", "suspected_stage",
                         "evidence_summary", "elapsed_sec", "collision_count",
                         "minimum_goal_distance_m", "attempts", "result_path"])
        for name, row in sorted(state["results"].items()):
            result_path = Path(row["result_path"])
            try:
                details = json.loads(result_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                details = {}
            metric = details.get("metrics", {})
            analysis = details.get("failure_analysis", {})
            try:
                shared_result_path = str(result_path.relative_to(cfg["output"]))
            except ValueError:
                shared_result_path = str(result_path)
            writer.writerow([name, row["status"], row["reason"], analysis.get("category", ""),
                analysis.get("suspected_stage", ""), compact_summary(analysis),
                metric.get("elapsed_sec", ""), metric.get("collision_count", ""),
                metric.get("minimum_goal_distance_m", ""), row["attempts"], shared_result_path])
    os.replace(temporary, path)


def _upload_commands(cfg, folder):
    """Build one campaign-profile upload without changing the local evidence."""
    remote = str(cfg["upload"]["rclone_remote"]).rstrip("/")
    base = "{}/{}".format(remote, cfg["name"])
    profile = cfg["upload"].get("profile", "basic")
    excludes = []
    if profile not in ("bag", "full"):
        excludes.extend(["--exclude", "*.bag", "--exclude", "*.bag.active"])
    if profile not in ("video", "full"):
        excludes.extend(["--exclude", "*.mp4"])
    case_target = "{}/{}/{}".format(base, folder.parent.name, folder.name)
    drive_flags = _upload_drive_flags(cfg)
    case_command = ["rclone", "copy", str(folder), case_target] + excludes + drive_flags
    metadata = [(cfg["output"] / name, "{}/{}".format(base, name))
                for name in ("summary.csv", "campaign_config.yaml")]
    return case_command, [["rclone", "copyto", str(source), target]
                          + drive_flags
                          for source, target in metadata]


def _upload_drive_flags(cfg):
    root_id = cfg["upload"].get("drive_root_folder_id")
    return ["--drive-root-folder-id", str(root_id)] if root_id else []


def _upload(cfg, folder, state, case):
    upload = cfg["upload"]
    if not upload.get("enabled", False):
        return
    state.setdefault("upload_errors", [])
    if not shutil.which("rclone"):
        state["upload_errors"].append({"scenario": case.name, "message": "rclone 미설치"})
        _write_state(cfg, state)
        return
    case_command, metadata_commands = _upload_commands(cfg, folder)
    state_command = ["rclone", "copyto", str(cfg["output"] / "campaign_state.json"),
                     "{}/{}/campaign_state.json".format(str(upload["rclone_remote"]).rstrip("/"),
                                                          cfg["name"])] + _upload_drive_flags(cfg)
    for command in [case_command] + metadata_commands:
        try:
            completed = subprocess.run(command, capture_output=True, text=True, timeout=3600)
            if completed.returncode:
                raise RuntimeError(completed.stderr[-500:] or "rclone 종료 코드 {}".format(completed.returncode))
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            state["upload_errors"].append({"scenario": case.name, "target": command[3],
                                           "message": str(error)[-500:]})
    _write_state(cfg, state)
    try:
        completed = subprocess.run(state_command, capture_output=True, text=True, timeout=3600)
        if completed.returncode:
            raise RuntimeError(completed.stderr[-500:] or "rclone 종료 코드 {}".format(completed.returncode))
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        state["upload_errors"].append({"scenario": case.name, "target": state_command[3],
                                       "message": str(error)[-500:]})
        _write_state(cfg, state)


def _run_case(cfg, processes, monitor, case, state):
    case_log_offsets = log_offsets(processes.log_dir)
    attempt = int(state["results"].get(case.name, {}).get("attempts", 0)) + 1
    timestamp = _stamp()
    safe_name = "".join(c if c.isascii() and (c.isalnum() or c in "_-") else "_" for c in case.name)[:50]
    basename = "planbag_{}_{}_{}".format(safe_name, "R", timestamp)
    folder = cfg["output"] / "running" / ("{}_{}_{:02d}".format(safe_name, timestamp, attempt))
    folder.mkdir(parents=True, exist_ok=False)
    cfg_snapshot = {"campaign": cfg["name"], "scenario": str(case.source),
                    "scenario_sha256": case.source_sha256, "goal": case.goal,
                    "waypoints": case.waypoints, "timeout_sec": case.timeout_sec,
                    "pose_source": cfg["stack"]["pose_source"],
                    "detector": cfg["stack"]["detector"],
                    "sensor_json": str(cfg["network"]["sensor_json"]),
                    "ego_network_json": str(cfg["network"]["ego_network_json"]),
                    "network_sha256": hashlib.sha256(cfg["network"]["ego_network_json"].read_bytes()).hexdigest(),
                    "topic_profile": str(cfg["recording"]["topic_profile"]),
                    "recorded_topics": cfg["recording"]["topics"]}
    atomic_json(folder / "input.json", cfg_snapshot)
    state["current"] = {"scenario": case.name, "phase": "SCENARIO_LOAD", "attempt": attempt,
                        "started_at": now(), "metrics": {}}
    _write_state(cfg, state)
    recorder = Recording(cfg["recording"], folder, processes.env)
    brake_hold = None
    reason, elapsed, route_id, error = "SYSTEM_ERROR", 0.0, None, None
    started_at = now()
    wall_started = time.monotonic()
    try:
        monitor.begin_case(folder / "events.jsonl")
        installed, ego_xy = prepare_scenario(case, cfg["install"], cfg["network"]["ego_network_json"])
        shutil.copy2(installed, folder / "scenario.json")
        brake_hold = BrakeHold()
        brake_hold.start()
        # Route rejection and stack startup failures are useful campaign evidence too.
        # Start both recorders before ScenarioLoad instead of only after route readiness.
        video_cfg = cfg["recording"].get("video", {})
        if (video_cfg.get("enabled", False) and cfg["stack"].get("planviz", True)
                and video_cfg.get("window_layout") == "split"):
            try:
                detail = arrange_split_windows(
                    bottom_bar_px=video_cfg["bottom_bar_px"],
                    display=video_cfg["display"],
                    startup_grace_sec=(6.0 if not processes.split_layout_initialized else 0.0))
                processes.split_layout_initialized = True
                monitor.event("VIDEO_LAYOUT_APPLIED", detail=detail)
            except RuntimeFailure as exc:
                monitor.event("VIDEO_LAYOUT_WARNING", message=str(exc))
                print("[WARN] {}: 수동 반반 배치로 영상은 계속 녹화합니다".format(exc), file=sys.stderr)
        elif (video_cfg.get("enabled", False) and cfg["stack"].get("planviz", True)
              and video_cfg.get("focus_planviz", True)):
            try:
                focus_planviz(timeout=float(cfg["document"].get("readiness_timeout_sec", 45)),
                              display=video_cfg["display"])
            except RuntimeFailure as exc:
                monitor.event("VIDEO_FOCUS_WARNING", message=str(exc))
                print("[WARN] {}: 영상은 계속 녹화합니다".format(exc), file=sys.stderr)
        recorder.start(basename)
        monitor.event("SCENARIO_LOAD_REQUEST", installed=installed.name)
        _load_scene(cfg, installed, monitor, ego_xy)
        monitor.event("SCENARIO_READY")
        if cfg["stack"]["detector"] == "bevfusion":
            detector_after = time.monotonic()
            monitor.wait_detection(after=detector_after, timeout=max(
                60.0, float(cfg["document"].get("readiness_timeout_sec", 45))),
                process=processes.running["bevfusion"],
                log_path=processes.log_dir / "bevfusion.log")
            monitor.event("BEVFUSION_READY")
        publish_brake()
        state["current"]["phase"] = "STACK_READY"
        _write_state(cfg, state)
        _start_case_stack(cfg, processes, monitor)
        state["current"]["phase"] = "ROUTE_REQUEST"
        _write_state(cfg, state)
        monitor.event("ROUTE_REQUEST")
        route_id = request_route(case, monitor,
            timeout=float(cfg["document"].get("route_ready_timeout_sec", 75)))
        monitor.arm(folder / "events.jsonl")
        monitor.event("CONTROLLER_START")
        brake_hold.stop()
        brake_hold = None
        controller_state = "/localization/control_kinematic_state" if cfg["stack"]["pose_source"] == "localization" else "/localization/kinematic_state"
        processes.start("controller", _launch("trajectory_follower", "pp_pi_autoware.launch",
            kinematic_state_topic=controller_state))
        import rospy
        from morai_msgs.msg import CtrlCmd
        rospy.wait_for_message("/ctrl_cmd", CtrlCmd, timeout=15)
        state["current"]["phase"] = "DRIVING"
        _write_state(cfg, state)
        drive_start = time.monotonic()
        monitor.start_driving(drive_start)
        last_state = [0.0]

        def progress(metrics):
            if processes.running["controller"].poll() is not None:
                raise RuntimeFailure("controller가 주행 중 종료됐습니다")
            if time.monotonic() - last_state[0] > 1.0:
                state["current"]["metrics"] = metrics
                _write_state(cfg, state)
                last_state[0] = time.monotonic()

        reason, elapsed = monitor.run_until_done(case, cfg["document"].get("validation", {}),
                                                  drive_start, progress)
        monitor.event("RUN_FINISHED", reason=reason)
    except ScenarioFailure as exc:
        reason, error = exc.reason, str(exc)
    except KeyboardInterrupt:
        reason, error = "MANUAL_ABORT", "사용자 중단"
    except Exception as exc:
        error = str(exc)
        reason = "SYSTEM_ERROR"
        print("[{}] {}: {}".format(case.name, reason, error), file=sys.stderr)
    finally:
        monitor.event("CASE_RESULT", reason=reason, error=error)
        drive_evidence = monitor.drive_evidence()
        processes.stop("controller")
        if brake_hold is not None:
            brake_hold.stop()
        try:
            publish_brake()
        except Exception:
            pass
        monitor.disarm()
        code = "P" if reason == "SUCCESS" else "E" if reason in ("SYSTEM_ERROR", "MANUAL_ABORT") else "F"
        final_base = basename.replace("_R_", "_{}_".format(code))
        try:
            bag, video = recorder.stop(final_base)
        except Exception as exc:
            bag = video = None
            reason, code = "RECORDING_FAILURE", "E"
            error = str(exc)
            final_base = basename.replace("_R_", "_E_")
        processes.stop_many(CASE_STACK)
        metrics = {"elapsed_sec": round(elapsed, 2),
            "wall_elapsed_sec": round(time.monotonic() - wall_started, 2),
            "collision_count": monitor.collision_count,
            "minimum_goal_distance_m": None if monitor.goal_min == float("inf") else round(monitor.goal_min, 3),
            "maximum_speed_mps": round(monitor.max_speed, 3), **drive_evidence}
        try:
            failure_analysis = analyze_failure(reason, metrics, error=error,
                goal_tolerance_m=float(cfg["document"].get("validation", {}).get("goal_tolerance_m", 2.0)),
                log_signals=scan_log_signals(processes.log_dir, case_log_offsets))
        except Exception as exc:
            # A report-generation error must not erase the bag and drive result.
            failure_analysis = {"schema_version": 1, "category": "ANALYSIS_ERROR",
                "summary": "실패 분석기 오류; 원본 기록을 확인하세요", "suspected_stage": "undetermined",
                "stage_confidence": "none", "evidence": [str(exc)[:240]],
                "log_signals": [], "root_cause_confirmed": False}
        with (folder / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"time": now(), "event": "FAILURE_ANALYSIS",
                "category": failure_analysis["category"],
                "suspected_stage": failure_analysis["suspected_stage"]},
                ensure_ascii=False) + "\n")
        result = {"schema_version": 1, "basename": final_base,
            "scenario": str(case.source), "scenario_sha256": case.source_sha256,
            "attempt": attempt, "started_at": started_at, "finished_at": now(),
            "status": "PASS" if code == "P" else "ERROR" if code == "E" else "FAIL",
            "reason": reason, "error": error, "route_uuid": route_id,
            "metrics": metrics, "failure_analysis": failure_analysis,
            "files": {"bag": bag.name if bag else None, "video": video.name if video else None,
                      "events": "events.jsonl"}}
        atomic_json(folder / (final_base + ".json"), result)
        classification = "success" if code == "P" else "failure" if code == "F" else "error"
        destination = cfg["output"] / classification / folder.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        folder.rename(destination)
        _record_result(cfg, state, case, result, destination)
        state["current"] = None
        _write_state(cfg, state)
        _upload(cfg, destination, state, case)
    return result


def _load_state(cfg, resume):
    path = cfg["output"] / "campaign_state.json"
    if path.is_file() and not resume:
        raise RuntimeFailure("이미 존재하는 캠페인입니다. --resume=true 또는 campaign_name 변경: {}".format(path))
    if path.is_file():
        state = json.loads(path.read_text(encoding="utf-8"))
        if state.get("config_path") != str(cfg["path"]):
            raise RuntimeFailure("resume YAML 경로가 이전과 다릅니다")
        if state.get("config_sha256") != _config_sha(cfg):
            raise RuntimeFailure("resume 도중 YAML 내용이 바뀌었습니다. 이전 설정을 복원하거나 새 campaign_name을 쓰세요")
        return state
    return {"schema_version": 1, "campaign": cfg["name"], "config_path": str(cfg["path"]),
            "config_sha256": _config_sha(cfg),
            "created_at": now(), "updated_at": now(), "total": len(cfg["cases"]),
            "current": None, "results": {}, "upload_errors": []}


def _config_sha(cfg):
    return hashlib.sha256(cfg["path"].read_bytes()).hexdigest()


def _recover_interrupted(cfg):
    """Keep evidence from a SIGKILL/power loss instead of mixing it with a rerun."""
    running = cfg["output"] / "running"
    if not running.is_dir():
        return
    for folder in sorted(running.iterdir()):
        if not folder.is_dir():
            continue
        destination = cfg["output"] / "error" / folder.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        suffix = 1
        while destination.exists():
            destination = cfg["output"] / "error" / "{}_recovered{:02d}".format(folder.name, suffix)
            suffix += 1
        atomic_json(folder / "interrupted.json", {"status": "ERROR", "reason": "PROCESS_INTERRUPTED",
                                               "recovered_at": now(), "note": "bag.active may be incomplete"})
        folder.rename(destination)


def check_free_disk(cfg):
    minimum = cfg["recording"].get("minimum_free_disk_gib")
    if minimum is not None:
        free = shutil.disk_usage(cfg["output"]).free / (1024 ** 3)
        if free < float(minimum):
            raise RuntimeFailure("저장 공간 부족: {:.1f} GiB 남음 / 최소 {} GiB. 공간 확보 후 재개하세요".format(free, minimum))


def run(cfg, resume=True, rerun_failed=False):
    cfg["output"].mkdir(parents=True, exist_ok=True)
    lock = (cfg["output"] / ".lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeFailure("같은 캠페인이 이미 실행 중입니다") from error
    state = _load_state(cfg, resume)
    snapshot = cfg["output"] / "campaign_config.yaml"
    if not snapshot.exists():
        shutil.copy2(cfg["path"], snapshot)
    _recover_interrupted(cfg)
    state["total"] = len(cfg["cases"])
    state["current"] = None
    _write_state(cfg, state)
    todo = []
    for case in cfg["cases"]:
        previous = state["results"].get(case.name)
        if previous and previous["source_sha256"] != case.source_sha256:
            raise RuntimeFailure("시나리오 내용이 이전 실행 후 바뀌었습니다: {}. 새 campaign_name을 쓰세요".format(case.source))
        if previous and previous["status"] in ("PASS", "FAIL") and not (rerun_failed and previous["status"] == "FAIL"):
            continue
        todo.append(case)
    if not todo:
        print("완료된 캠페인입니다: {}".format(cfg["output"]))
        return 0
    processes = Processes(cfg["output"] / "stack_logs", _env())
    try:
        check_free_disk(cfg)
        _check_traffic_light(cfg)
        _start_master(processes, float(cfg["document"].get("readiness_timeout_sec", 45)))
        import rospy
        rospy.init_node("planning_scenario_validator", anonymous=False, disable_signals=True)
        monitor = LiveMonitor()
        _start_persistent(cfg, processes, monitor)
        for index, case in enumerate(todo, 1):
            # Check between cases without corrupting an in-progress recorder.
            check_free_disk(cfg)
            print("[{}/{}] {}".format(index, len(todo), case.name), flush=True)
            result = _run_case(cfg, processes, monitor, case, state)
            print("  -> {} ({})".format(result["status"], result["reason"]), flush=True)
            if result["reason"] == "MANUAL_ABORT":
                break
            if result["status"] == "ERROR" and not cfg["document"].get("continue_on_system_error", False):
                break
    finally:
        # A second Ctrl+C during ROS shutdown must not orphan roscore/UDP/Planviz.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        processes.close()
        state["current"] = None
        _write_state(cfg, state)
        lock.close()
    counts = {label: sum(row["status"] == label for row in state["results"].values())
              for label in ("PASS", "FAIL", "ERROR")}
    print("캠페인: {} | 성공 {} / 실패 {} / 시스템 오류 {} / 전체 {}".format(
        cfg["output"], counts["PASS"], counts["FAIL"], counts["ERROR"], len(cfg["cases"])))
    return 0 if len(state["results"]) == len(cfg["cases"]) and not counts["ERROR"] else 1


def main(argv=None):
    args = parse_args(argv)
    try:
        cfg = load_campaign(args.config)
        if args.check or args.dry_run:
            print("설정 확인: {} | 시나리오 {}개 | 설치 {}".format(cfg["name"], len(cfg["cases"]), cfg["install"]))
            if args.dry_run:
                for index, case in enumerate(cfg["cases"], 1):
                    print("{:02d}. {} -> goal ({:.1f}, {:.1f}) / {}초".format(index, case.source.name,
                        case.goal["x"], case.goal["y"], case.timeout_sec))
            return 0
        resume = args.resume == "true" if args.resume is not None else bool(cfg["document"].get("resume", True))
        def on_terminate(_signal, _frame):
            raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM, on_terminate)
        return run(cfg, resume, args.rerun_failed)
    except (ConfigError, RuntimeFailure) as error:
        print("planning_validator 오류: {}".format(error), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("중단했습니다. 같은 YAML로 재실행하면 resume합니다.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
