"""시나리오 폴더를 순회하고 ROS 주행·녹화·판정을 재개 가능하게 실행한다."""
import argparse
import csv
import fcntl
import hashlib
import json
import math
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

from .config import (ConfigError, EXECUTION_CONTRACT, ROOT, load_campaign, resolve_path,
                     prediction_horizon, prediction_mixed_horizon_enabled,
                     prediction_highway_horizon, prediction_highway_mode,
                     traffic_temporal_options)
from .route_profile import ProfileGeometry
from .executions import is_snapshot, load_execution, prepare_execution
from .traffic_light import (runtime_mode, runtime_settings, profile_name, candidate_assets,
                           profile_check_arguments, launch_options as traffic_launch_options)
from .models import resolve_contract, verify_contract, selected_model, runtime_image_selection, CANONICAL
from .recording_audit import audit_bag
from .recording_options import diagnostics_enabled, sharing_warnings, sharing_enabled, localization_analysis_enabled
from .localization_contract import effective_localization_args, effective_gt_pose_args, resolve_localization_contract
from .failure_analysis import analyze_failure, compact_summary, log_offsets, scan_log_signals
from .repetitions import sampled_scene
from .startup_audit import mark_startup_phase, record_startup_failure, complete_startup
from .runtime import (BrakeHold, LiveMonitor, Processes, Recording, RuntimeFailure, SafetyFailure,
                      ScenarioFailure,
                      atomic_json, check_no_conflicting_publishers,
                      arrange_split_windows, focus_planviz, master_alive, now, prepare_scenario, publish_brake,
                      request_route)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="MORAI planning 시나리오 자동 검증")
    parser.add_argument("config", nargs="?", help="편집용 실험 YAML 한 개 (새 실행)")
    parser.add_argument("--execution", help="재개할 기존 실험 결과 폴더")
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
    command += profile_check_arguments(cfg['stack'], ROOT)
    print("신호등 실행 환경 검사: {} / device={}".format(python, device), flush=True)
    logs = cfg['output'] / 'stack_logs'
    logs.mkdir(parents=True, exist_ok=True)
    receipt = {'schema_version': 1, 'command': command, 'started_at': now(),
               'python': python, 'device': device, 'profile': profile_name(cfg['stack']), 'status': 'RUNNING'}
    atomic_json(logs / 'traffic_light_preflight.json', receipt)
    try:
        checked = subprocess.run(command, env=_env(), stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired) as error:
        output = getattr(error, 'stdout', '') or ''
        if isinstance(output, bytes):
            output = output.decode('utf-8', errors='replace')
        (logs / 'traffic_light_preflight.log').write_text(output, encoding='utf-8')
        receipt.update(status='ERROR', error=str(error), finished_at=now())
        atomic_json(logs / 'traffic_light_preflight.json', receipt)
        raise RuntimeFailure("신호등 환경 검사 실행 실패: {}".format(error)) from error
    (logs / 'traffic_light_preflight.log').write_text(checked.stdout, encoding='utf-8')
    probe_status = None
    decoder = json.JSONDecoder()
    for index, char in enumerate(checked.stdout):
        if char != '{':
            continue
        try:
            value, _ = decoder.raw_decode(checked.stdout[index:])
        except ValueError:
            continue
        if isinstance(value, dict) and value.get('status') in ('PASS', 'FAIL'):
            probe_status = value['status']
            break
    receipt.update(status='PASS' if checked.returncode == 0 else 'FAIL',
        returncode=checked.returncode, probe_report_status=probe_status,
        reported_pass_but_process_failed=probe_status == 'PASS' and checked.returncode != 0,
        finished_at=now())
    atomic_json(logs / 'traffic_light_preflight.json', receipt)
    print(checked.stdout, flush=True)
    if checked.returncode:
        raise RuntimeFailure('신호등 환경 검사 프로세스 종료코드 {} (보고서 status={}). '
            '차량 주행은 시작하지 않았습니다. 상세: {}'.format(
                checked.returncode, probe_status, logs / 'traffic_light_preflight.json'))


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
        enable_lidar=network["has_lidar"], enable_control=False))
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
        if selected_model(stack) == 'e25':
            command = ["bash", str(ROOT / "scripts/runtime/run_bevfusion.sh"), "--model", "e25", str(sensor)]
            if diagnostics_enabled(cfg.get('recording', {})):
                command.insert(-1, '--publish-frame-timing')
        else:
            receipt = cfg['output'] / 'model_process_receipt.json'
            if receipt.exists():
                receipt.rename(processes.log_dir / ('model_process_receipt_previous_' + str(time.time_ns()) + '.json'))
            command = ['/usr/bin/python3', str(ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/scripts/run_bevfusion_trial.py'),
                       '--contract', str(cfg['output'] / 'model_runtime.json')]
            if diagnostics_enabled(cfg.get('recording', {})):
                command += ['--publish-frame-timing']
        atomic_json(cfg['output'] / 'model_launch.json', {'command': command, 'model': cfg.get('model_contract')})
        processes.start("bevfusion", command)
        if selected_model(stack) != 'e25':
            def server_ready():
                if processes.running['bevfusion'].poll() is not None:
                    raise RuntimeFailure('Base5 서버 준비 중 종료. stack_logs/bevfusion.log를 확인하세요')
                path = cfg['output'] / 'model_process_receipt.json'
                return path.is_file() and bool(json.loads(path.read_text()).get('server_ready'))
            monitor.wait(server_ready, 180, 'Base5 checkpoint·센서 정합·cold start')
    if stack.get("planviz", True):
        processes.start("planviz", _launch("bringup", "aw149_planning_rviz.launch"))


def _start_case_stack(cfg, processes, monitor):
    stack, network = cfg["stack"], cfg["network"]
    traffic_options = cfg.get('traffic_light_options', traffic_launch_options(stack))
    temporal = cfg.get('traffic_light_temporal_options')
    if temporal is None:
        temporal = traffic_temporal_options(stack,
            legacy_snapshot=Path(cfg.get('path', '')).name == 'config_snapshot.yaml')
    timeout = float(cfg["document"].get("readiness_timeout_sec", 45))
    after = time.monotonic()
    if stack["pose_source"] == "localization":
        processes.start("pose", _launch("bringup", "localization_gnss_imu_gt_ctra.launch",
            **effective_localization_args(cfg, ROOT)))
    else:
        if cfg.get('gt_pose_launch_args'):
            processes.start("pose", _launch("bringup", "localization_morai_gt.launch",
                **effective_gt_pose_args(cfg, ROOT)))
        else:
            processes.start("pose", _launch("bringup", "maptf_morai.launch",
                ego_topic="/Ego_topic", start_lanelet_map=False))
    monitor.wait_odom(after=after, timeout=timeout)
    if stack["detector"] == "od_gt":
        processes.start("detector", _launch("perception_object_pipeline", "morai_gt_detection.launch"))
    processes.start("tracking", _launch("perception_object_pipeline", "object_pipeline.launch",
        start_detector=False, prediction_model="map_based",
        prediction_horizon=prediction_horizon(stack),
        prediction_mixed_horizon_enabled=prediction_mixed_horizon_enabled(stack),
        prediction_highway_horizon=prediction_highway_horizon(stack),
        prediction_highway_mode=prediction_highway_mode(stack),
        publish_processing_diagnostics=diagnostics_enabled(cfg.get('recording', {})),
        maximum_tracking_internal_prediction_age_sec=0.5))
    if stack.get("traffic_light", False):
        python, device = runtime_settings(stack, ROOT, check_exists=True)
        selected = candidate_assets(stack, ROOT, check_exists=True)
        extra = {} if selected is None else dict(models_root=selected['root'], repo_root=ROOT,
                                                sensor_config=cfg['network']['sensor_json'])
        filename = 'traffic_light_legacy.launch' if selected is None else 'traffic_light_v5_candidate.launch'
        processes.start("traffic_light", _launch("traffic_light_detection", filename,
            inference_python=shlex.quote(python), device=device,
            isolate_python=runtime_mode(stack) == "virtualenv",
            publish_debug_image=traffic_options['publish_debug_image'],
            preserve_unreadable_color_evidence=temporal['preserve_unreadable_color_evidence'],
            planner_compliance_observations=temporal['planner_compliance_observations'],
            stamped_vote_window_size=temporal['stamped_vote_window_size'], **extra))
        if traffic_options['debug_view']:
            processes.start('traffic_light_debug_view', _launch('traffic_light_detection',
                'traffic_light_debug_view.launch',
                image_topic='/traffic_light_node/debug_image/compressed',
                display=cfg['recording']['video']['display']))
    processes.start("planning", _launch("ajounice_planning_integration", "planning.launch",
        repo_root=ROOT, start_requester=False, is_simulation=False,
        behavior_path_forward_length=400.0, intersection_attention_area_length=100.0,
        enable_traffic_light_planner=stack.get("traffic_light", False),
        enable_signal_compliance_guard=temporal['enable_signal_compliance_guard'],
        signal_compliance_unknown_grace_sec=temporal['signal_compliance_unknown_grace_sec'],
        enable_yellow_passthrough=traffic_options['yellow_passthrough'],
        signal_compliance_yellow_deceleration_mps2=traffic_options.get('signal_compliance_yellow_deceleration_mps2', 8.0),
        signal_compliance_yellow_jerk_mps3=traffic_options.get('signal_compliance_yellow_jerk_mps3', 16.0),
        signal_compliance_required_green_observations=temporal['signal_compliance_required_green_observations'],
        signal_compliance_min_yellow_sec=traffic_options.get('signal_compliance_min_yellow_sec', 2.0),
        signal_compliance_yellow_time_margin_sec=traffic_options.get('signal_compliance_yellow_time_margin_sec', 0.2),
        # Old timed fields remain in frozen inputs, but cannot reactivate the replaced policy.
        signal_compliance_enable_timed_yellow=False,
        publish_fixed_autonomous_mode=True, use_ground_pointcloud=False,
        use_occupancy_grid=False))


CASE_STACK = ["controller", "planning", "traffic_light_debug_view", "traffic_light", "tracking", "detector", "pose"]


def _load_scene(cfg, installed, monitor, ego_xy):
    from trajectory_batch_runner.morai_udp_client import build_scenario_load_payload
    request = {"delete_all": False, "load_network_connection_data": True,
               "load_ego_vehicle_data": True, "load_surrounding_vehicle_data": True,
               "load_pedestrian_data": True, "load_obstacle_data": True, "set_pause": False}
    payload = build_scenario_load_payload(installed.stem, request)
    udp = cfg["document"].get("morai", {})
    host, port = udp.get("scenario_load_host", "127.0.0.1"), int(udp.get("scenario_load_port", 9095))
    timeout = float(cfg["document"].get("scenario_ready_timeout_sec", 35))
    if udp.get("clear_before_load", False):
        # MORAI delete_all clears selected actor classes instead of loading
        # their new contents. Wait for that distinct scene state before reload;
        # sending both packets in one physics step cannot establish a clean scene.
        clear_request = dict(request, delete_all=True, load_ego_vehicle_data=False)
        clear_payload = build_scenario_load_payload(installed.stem, clear_request)
        clear_after = time.monotonic()
        monitor.event("SCENE_CLEAR_REQUESTED", installed=installed.name)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            if connection.sendto(clear_payload, (host, port)) != len(clear_payload):
                raise RuntimeFailure("ScenarioLoad 객체 제거 패킷 일부만 송신됨")
        monitor.wait_empty_scene(after=clear_after, timeout=timeout)
        monitor.event("SCENE_CLEAR_CONFIRMED")
        time.sleep(1.0)  # Let the unpaused physics scene process actor removal.
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
        "attempts": int(state["results"].get(case.name, {}).get("attempts", 0)) + 1,
        "analysis_complete": result.get('analysis_available', False),
        "recording_warning": (result.get('recording_audit') or {}).get('status') == 'WARN'}
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
                         "minimum_goal_distance_m", "attempts", "result_path",
                         "source_scenario", "repeat_index", "random_seed", "model_preset",
                         "recording_complete", "sensor_replayable", "missing_recorded_topics",
                         "bag_enabled", "video_enabled", "recording_warning", "latency_complete",
                         "analysis_complete", "share_status"])
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
                metric.get("minimum_goal_distance_m", ""), row["attempts"], shared_result_path,
                Path(details.get("scenario", name)).name, details.get("repeat_index", 1),
                details.get("randomization", {}).get("seed", ""), (details.get('model') or {}).get('preset', ''),
                (details.get('recording_audit') or {}).get('complete_for_drive', ''),
                (details.get('recording_audit') or {}).get('re_inference_possible', ''),
                ';'.join((details.get('recording_audit') or {}).get('missing_required', [])),
                details.get('recording', {}).get('bag_enabled', True),
                details.get('recording', {}).get('video_enabled', bool(details.get('files', {}).get('video'))),
                (details.get('recording_audit') or {}).get('status') == 'WARN',
                (details.get('recording_audit') or {}).get('complete_for_latency', ''),
                row.get('analysis_complete', False), row.get('share_status', 'NOT_UPLOADED')])
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
        report = {'status': 'DISABLED', 'verified': False,
                  'warnings': sharing_warnings(cfg.get('recording', {}), upload)}
        if sharing_enabled(cfg.get('recording', {})):
            # Manual sharing must receive the same logs/receipts/sanitization.
            from .sharing import build_bundle
            try:
                _, payload, manifest = build_bundle(cfg, folder)
                report.update(status='READY_FOR_MANUAL_SHARE', payload=str(payload),
                              warnings=manifest['warnings'],
                              analysis_available_in_share=manifest['analysis_available_in_share'])
            except Exception as error:
                report.update(status='ERROR', error=str(error))
        atomic_json(folder / 'sharing_audit.json', report)
        return
    state.setdefault("upload_errors", [])
    if not shutil.which("rclone"):
        state["upload_errors"].append({"scenario": case.name, "message": "rclone 미설치"})
        atomic_json(folder / 'sharing_audit.json', {'status': 'ERROR', 'verified': False, 'error': 'rclone 미설치'})
        _write_state(cfg, state)
        return
    from .sharing import build_bundle, verify_listing, safe_copy
    try:
        payload_root, payload, manifest = build_bundle(cfg, folder)
    except Exception as error:
        state['upload_errors'].append({'scenario': case.name, 'message': '공유 자료 준비 실패: ' + str(error)})
        atomic_json(folder / 'sharing_audit.json', {'status': 'ERROR', 'verified': False, 'error': str(error)})
        _write_state(cfg, state)
        return
    share_cfg = dict(cfg, output=payload_root)
    case_command, metadata_commands = _upload_commands(share_cfg, payload)
    errors_before = len(state['upload_errors'])
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
    safe_copy(cfg['output'] / 'campaign_state.json', payload_root / 'campaign_state.json')
    state_command[2] = str(payload_root / 'campaign_state.json')
    try:
        completed = subprocess.run(state_command, capture_output=True, text=True, timeout=3600)
        if completed.returncode:
            raise RuntimeError(completed.stderr[-500:] or "rclone 종료 코드 {}".format(completed.returncode))
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        state["upload_errors"].append({"scenario": case.name, "target": state_command[3],
                                       "message": str(error)[-500:]})
        _write_state(cfg, state)
    report = {'status': 'ERROR', 'verified': False, 'profile': upload.get('profile', 'basic'),
              'target': case_command[3], 'warnings': manifest['warnings'],
              'analysis_available_in_share': manifest['analysis_available_in_share'],
              'manifest': 'share_manifest.json'}
    if len(state['upload_errors']) == errors_before:
        try:
            listing_command = ['rclone', 'lsjson', case_command[3], '--recursive', '--hash'] + _upload_drive_flags(cfg)
            listing = subprocess.run(listing_command, capture_output=True, text=True, timeout=3600, check=True)
            missing = verify_listing(payload, json.loads(listing.stdout))
            if missing:
                raise RuntimeError('공유 파일 크기/hash 불일치 또는 hash 미제공: ' + ', '.join(missing))
            report.update(status='VERIFIED', verified=True, verified_files=len(manifest['files']) + 1)
        except Exception as error:
            report['error'] = str(error)[-500:]
            state['upload_errors'].append({'scenario': case.name, 'message': report['error']})
    atomic_json(folder / 'sharing_audit.json', report)
    if case.name in state.get('results', {}):
        state['results'][case.name]['share_status'] = report['status']
    _write_state(cfg, state)
    # Publish the verification receipt only after inspecting actual remote files.
    if report['verified']:
        safe_copy(folder / 'sharing_audit.json', payload / 'sharing_audit.json')
        try:
            subprocess.run(['rclone', 'copyto', str(payload / 'sharing_audit.json'),
                            case_command[3] + '/sharing_audit.json'] + _upload_drive_flags(cfg),
                           capture_output=True, text=True, timeout=3600, check=True)
            listing = subprocess.run(['rclone', 'lsjson', case_command[3], '--recursive', '--hash']
                                     + _upload_drive_flags(cfg), capture_output=True, text=True,
                                     timeout=3600, check=True)
            if verify_listing(payload, json.loads(listing.stdout)):
                raise RuntimeError('공유 검증 receipt 업로드 확인 실패')
            base_target = '{}/{}'.format(str(upload['rclone_remote']).rstrip('/'), cfg['name'])
            for name in ('summary.csv', 'campaign_state.json'):
                safe_copy(cfg['output'] / name, payload_root / name)
                subprocess.run(['rclone', 'copyto', str(payload_root / name), base_target + '/' + name]
                               + _upload_drive_flags(cfg), capture_output=True, text=True,
                               timeout=3600, check=True)
            metadata_listing = subprocess.run(['rclone', 'lsjson', base_target, '--hash'] + _upload_drive_flags(cfg),
                                              capture_output=True, text=True, timeout=3600, check=True)
            if verify_listing(payload_root, json.loads(metadata_listing.stdout),
                              files=[payload_root / name for name in ('summary.csv', 'campaign_config.yaml', 'campaign_state.json')]):
                raise RuntimeError('공유 CSV/YAML/state 최종 내용 검증 실패')
        except Exception as error:
            report.update(status='ERROR', verified=False, error=str(error)[-500:])
            atomic_json(folder / 'sharing_audit.json', report)
            state['upload_errors'].append({'scenario': case.name, 'message': report['error']})
            if case.name in state.get('results', {}):
                state['results'][case.name]['share_status'] = 'ERROR'
            _write_state(cfg, state)


def _capture_model_receipt(cfg, folder):
    """Capture the live server identity, not just the requested launch config."""
    receipt = {'model': cfg.get('model_contract'), 'status': 'UNKNOWN'}
    try:
        if str(ROOT / 'docker/bevfusion') not in sys.path:
            sys.path.insert(0, str(ROOT / 'docker/bevfusion'))
        from app.protocol import TYPE_PING, TYPE_PONG, send_message, recv_message
        with socket.create_connection(('127.0.0.1', cfg['model_contract']['server_port']), timeout=2) as connection:
            connection.settimeout(2)
            send_message(connection, TYPE_PING)
            kind, metadata, _ = recv_message(connection)
        receipt['server_ping'] = metadata
        identity = metadata.get('runtime_identity', {})
        receipt['status'] = 'VERIFIED' if (kind == TYPE_PONG and metadata.get('model_loaded')
            and identity.get('checkpoint_sha256') == cfg['model_contract']['checkpoint_sha256']
            and identity.get('config_sha256') == cfg['model_contract']['model_config_sha256']
            and metadata.get('calibration', {}).get('config_sha256') == cfg['model_contract']['sensor_sha256']) else 'MISMATCH'
    except Exception as error:
        receipt['error'] = str(error)
    atomic_json(folder / 'actual_model_receipt.json', receipt)
    return receipt


def _control_sender_ready(processes):
    if processes.running['udp_control'].poll() is not None:
        raise SafetyFailure('UDP 제어 송신기가 준비 중 종료됐습니다')
    import rosgraph
    subscriptions = dict(rosgraph.Master('/planning_validator_control_guard').getSystemState()[1])
    return '/morai_ctrl_cmd_udp_sender' in subscriptions.get('/ctrl_cmd', [])


def _verify_control_owner(monitor, after=0):
    import rosgraph
    owners = dict(rosgraph.Master('/planning_validator_control_guard').getSystemState()[0]).get('/ctrl_cmd', [])
    if owners != ['/longi_pi_autoware']:
        raise SafetyFailure('/ctrl_cmd 소유권 불일치: {}'.format(owners))
    if monitor.control_source != '/longi_pi_autoware' or monitor.stamps.get('control', 0) <= after:
        raise SafetyFailure('현재 제어기의 새로운 명령이 확인되지 않았습니다')


def _capture_runtime_receipt(cfg, processes, folder, stage='control_ready'):
    import rosgraph
    subprocess.run(['rosparam', 'dump', str(folder / 'ros_parameters_after_control.yaml')],
                   check=True, timeout=10, env=processes.env)
    master = rosgraph.Master('/planning_validator_runtime_receipt')
    atomic_json(folder / 'runtime_receipt.json', {
        'recorded_at': now(), 'stage': stage, 'localization_contract': cfg.get('localization_contract'),
        'python_executable': sys.executable, 'runner_module': str(Path(__file__).resolve()),
        'git_sha': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=str(ROOT), text=True).strip(),
        'tracked_modified_files': subprocess.check_output(['git', 'diff', '--name-only', 'HEAD'],
            cwd=str(ROOT), text=True).splitlines(),
        'commands': processes.commands, 'ros_system_state': master.getSystemState(),
        'topic_types': master.getTopicTypes(), 'controller_state_topic':
            '/localization/control_kinematic_state' if cfg['stack']['pose_source'] == 'localization'
            else '/localization/kinematic_state',
        'resolved_parameters': 'ros_parameters_after_control.yaml'})


def _stop_and_verify(cfg, processes, monitor):
    """Opt-in trial shutdown: brake, observed stop, owned P sender, observed P."""
    safety = cfg['localization_safety']
    hold = None
    stopped_after = time.monotonic()
    try:
        processes.stop('parking_prepare')
        publish_brake()
        if 'udp_control' in processes.running:
            monitor.verify_stopped(after=stopped_after, timeout=safety['stop_timeout_sec'],
                hold_sec=safety['stationary_hold_sec'], speed_max_mps=safety['speed_max_mps'],
                freshness_sec=safety['freshness_sec'])
        processes.stop('udp_control')
        if safety['park_after_run']:
            hold = BrakeHold()
            hold.start()
            parked_after = time.monotonic()
            processes.start('udp_control', ['roslaunch', str(ROOT /
                'base_ws/src/planning_addons/planning_scenario_validator/launch/parking_hold.launch'),
                'network_config:=' + str(cfg['network']['ego_network_json'])])
            monitor.wait(lambda: _control_sender_ready(processes), safety['stop_timeout_sec'], 'Park 송신기')
            receipt = monitor.verify_stopped(after=parked_after, timeout=safety['stop_timeout_sec'],
                hold_sec=safety['stationary_hold_sec'], speed_max_mps=safety['speed_max_mps'],
                freshness_sec=safety['freshness_sec'], require_park=True)
        else:
            receipt = monitor.verify_stopped(after=stopped_after, timeout=safety['stop_timeout_sec'],
                hold_sec=safety['stationary_hold_sec'], speed_max_mps=safety['speed_max_mps'],
                freshness_sec=safety['freshness_sec'])
        return {'status': 'VERIFIED', 'stop': receipt}
    finally:
        if hold is not None:
            hold.stop()
        processes.stop('udp_control')


def _run_case(cfg, processes, monitor, case, state):
    case_log_offsets = log_offsets(processes.log_dir)
    attempt = int(state["results"].get(case.name, {}).get("attempts", 0)) + 1
    timestamp = _stamp()
    safe_name = "".join(c if c.isascii() and (c.isalnum() or c in "_-") else "_" for c in case.name)
    suffix = '_r{:03d}'.format(case.repeat_index) if case.repeat_total > 1 else ''
    safe_name = safe_name[:-len(suffix)][:50 - len(suffix)] + suffix if suffix else safe_name[:50]
    basename = "planbag_{}_{}_{}".format(safe_name, "R", timestamp)
    folder = cfg["output"] / "running" / ("{}_{}_{:02d}".format(safe_name, timestamp, attempt))
    folder.mkdir(parents=True, exist_ok=False)
    _, randomization = sampled_scene(json.loads(case.source.read_text(encoding="utf-8")),
        source_sha256=case.source_sha256, repeat_index=case.repeat_index, settings=case.randomization)
    cfg_snapshot = {"campaign": cfg["name"], "scenario": str(case.source),
                    "repeat_index": case.repeat_index, "repeat_total": case.repeat_total,
                    "randomization": randomization,
                    "scenario_sha256": case.source_sha256, "goal": case.goal,
                    "waypoints": case.waypoints, "timeout_sec": case.timeout_sec,
                    "pose_source": cfg["stack"]["pose_source"],
                    "detector": cfg["stack"]["detector"],
                    "model": cfg.get('model_contract'),
                    "traffic_light_options": cfg.get('traffic_light_options', traffic_launch_options(cfg['stack'])),
                    "git_sha": subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=str(ROOT), text=True).strip(),
                    "sensor_json": str(cfg["network"]["sensor_json"]),
                    "ego_network_json": str(cfg["network"]["ego_network_json"]),
                    "network_sha256": hashlib.sha256(cfg["network"]["ego_network_json"].read_bytes()).hexdigest(),
                    "topic_profile": str(cfg["recording"].get("topic_profile", "")),
                    "bag_enabled": cfg['recording'].get('bag_enabled', True),
                    "recorded_topics": cfg["recording"]["topics"],
                    "recorded_topic_regex": cfg['recording'].get('topic_regex', [])}
    if cfg.get('route'):
        cfg_snapshot['route'] = {'name':cfg['route']['name'],
            'definition_sha256':cfg['route']['definition_sha256'],'goal_preserved':True}
    atomic_json(folder / "input.json", cfg_snapshot)
    state["current"] = {"scenario": case.name, "phase": "SCENARIO_LOAD", "attempt": attempt,
                        "started_at": now(), "metrics": {}}
    _write_state(cfg, state)
    recorder = Recording(cfg["recording"], folder, processes.env)
    brake_hold = None
    safety = cfg.get('localization_safety', {'enabled': False})
    safety_receipt, halt_campaign = {'enabled': safety['enabled']}, False
    reason, elapsed, route_id, error = "SYSTEM_ERROR", 0.0, None, None
    started_at = now()
    wall_started = time.monotonic()
    try:
        monitor.begin_case(folder / "events.jsonl")
        installed, ego_xy = prepare_scenario(case, cfg["install"], cfg["network"]["ego_network_json"])
        shutil.copy2(installed, folder / "scenario.json")
        brake_hold = BrakeHold()
        brake_hold.start()
        if safety['enabled']:
            # The prepare sender sends P + brake, never D. Keep it separate
            # from driving control until route and stationary gates pass.
            processes.start('parking_prepare', ['roslaunch', str(ROOT /
                'base_ws/src/planning_addons/planning_scenario_validator/launch/parking_hold.launch'),
                'network_config:=' + str(cfg['network']['ego_network_json'])])
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
        loaded_after = time.monotonic()
        _load_scene(cfg, installed, monitor, ego_xy)
        monitor.event("SCENARIO_READY")
        if safety['enabled']:
            safety_receipt['startup'] = monitor.wait_stationary(after=loaded_after,
                timeout=safety['startup_timeout_sec'], hold_sec=safety['stationary_hold_sec'],
                speed_max_mps=safety['speed_max_mps'], freshness_sec=safety['freshness_sec'],
                required=('ego', 'gear', 'control_mode', 'collision', 'competition', 'gps', 'imu'),
                require_valid_gps=safety['require_valid_gps'])
        if cfg["stack"]["detector"] == "bevfusion":
            detector_after = time.monotonic()
            monitor.wait_detection(after=detector_after, timeout=max(
                60.0, float(cfg["document"].get("readiness_timeout_sec", 45))),
                process=processes.running["bevfusion"],
                log_path=processes.log_dir / "bevfusion.log")
            monitor.event("BEVFUSION_READY")
            _capture_model_receipt(cfg, folder)
        publish_brake()
        state["current"]["phase"] = "STACK_READY"
        _write_state(cfg, state)
        _start_case_stack(cfg, processes, monitor)
        subprocess.run(['rosparam', 'dump', str(folder / 'ros_parameters.yaml')], check=True, timeout=10, env=processes.env)
        state["current"]["phase"] = "ROUTE_REQUEST"
        _write_state(cfg, state)
        monitor.event("ROUTE_REQUEST")
        def _build_route_args():
            args = {}
            if cfg.get('route'):
                geometry = cfg.get('_route_geometry')
                if geometry is None:
                    geometry = cfg['_route_geometry'] = ProfileGeometry(cfg['route'], resolve_path)
                odom = monitor.odom
                if odom is None or odom.header.frame_id != 'map':
                    raise RuntimeFailure('Route B 요청에 map 좌표의 현재 위치가 필요합니다.')
                p, q = odom.pose.pose.position, odom.pose.pose.orientation
                yaw = math.degrees(math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
                args['route_plan'] = geometry.plan(
                    dict(x=p.x, y=p.y, z=p.z, yaw_deg=yaw), case.goal)
            return args

        # route 요청이 간헐적으로 거부된다("DefaultPlanner returned an empty route").
        # 같은 시나리오가 통과/실패를 반복하므로 내용 문제가 아니라, localization
        # pose가 lanelet에 올라오기 전에 요청이 나가는 레이스로 본다. 재요청으로 흡수.
        route_id = None
        route_tries = int(cfg["document"].get("route_request_attempts", 3))
        for route_attempt in range(1, route_tries + 1):
            route_args = _build_route_args()
            if 'route_plan' in route_args:
                atomic_json(folder / 'route_profile.json', route_args['route_plan'])
            try:
                route_id = request_route(case, monitor,
                    timeout=float(cfg["document"].get("route_ready_timeout_sec", 75)),
                    process=processes.running['planning'], **route_args)
                break
            except RuntimeFailure as exc:
                if route_attempt >= route_tries:
                    raise
                monitor.event("ROUTE_RETRY", attempt=route_attempt, error=str(exc)[:200])
                print("route 요청 거부: {}/{}회, 4초 후 재시도".format(
                    route_attempt, route_tries), flush=True)
                time.sleep(4.0)
        if safety['enabled']:
            monitor.wait_stationary(after=loaded_after, timeout=safety['startup_timeout_sec'],
                hold_sec=safety['stationary_hold_sec'], speed_max_mps=safety['speed_max_mps'],
                freshness_sec=safety['freshness_sec'], required=('ego', 'gear', 'control_mode', 'collision'))
        monitor.arm(folder / "events.jsonl")
        # The UDP sender puts the vehicle in D on every command, including brake.
        # Never start it with the persistent sensor receivers. Route/trajectory
        # readiness must precede any control packet reaching MORAI.
        monitor.event("CONTROL_TRANSPORT_START")
        processes.stop('parking_prepare')
        processes.start("udp_control", _launch("sensor_interface", "morai_udp_interface.launch",
            network_config=cfg["network"]["ego_network_json"],
            enable_sensors=False, enable_lidar=False, enable_ego_network=False,
            enable_control=True))
        def control_connected():
            if processes.running["udp_control"].poll() is not None:
                raise RuntimeFailure("UDP 제어 송신기가 준비 중 종료됐습니다")
            import rosgraph
            subscribers = dict(rosgraph.Master('/planning_validator_control_guard').getSystemState()[1])
            # The live monitor also subscribes to /ctrl_cmd: connection count
            # alone would incorrectly consider the sender ready.
            return '/morai_ctrl_cmd_udp_sender' in subscribers.get('/ctrl_cmd', [])

        monitor.wait(control_connected, 15, "주행 직전 UDP 제어 송신기 연결")
        # Brake remains applied during the controller launch; it is not released
        # until the controller sends its own first command.
        publish_brake(duration=0.2)
        monitor.event("CONTROLLER_START")
        brake_hold.stop()
        brake_hold = None
        controller_state = "/localization/control_kinematic_state" if cfg["stack"]["pose_source"] == "localization" else "/localization/kinematic_state"
        processes.start("controller", _launch("trajectory_follower", "pp_pi_autoware.launch",
            kinematic_state_topic=controller_state))
        import rospy
        from morai_msgs.msg import CtrlCmd
        rospy.wait_for_message("/ctrl_cmd", CtrlCmd, timeout=15)
        if safety['enabled']:
            monitor.wait(lambda: monitor.control_source == '/longi_pi_autoware', 15, '제어기 명령 발행자')
            _verify_control_owner(monitor)
            _capture_runtime_receipt(cfg, processes, folder)
        state["current"]["phase"] = "DRIVING"
        _write_state(cfg, state)
        drive_start = time.monotonic()
        monitor.start_driving(drive_start)
        last_state = [0.0]

        def progress(metrics):
            if safety['enabled']:
                monitor.localization_watchdog(pose_source=cfg['stack']['pose_source'],
                    freshness_sec=safety['freshness_sec'])
                _verify_control_owner(monitor)
                if processes.running['udp_control'].poll() is not None:
                    raise SafetyFailure('주행 중 UDP 제어 송신기 종료')
            if processes.running["controller"].poll() is not None:
                raise RuntimeFailure("controller가 주행 중 종료됐습니다")
            if cfg['stack']['detector'] == 'bevfusion':
                if processes.running['bevfusion'].poll() is not None:
                    raise RuntimeFailure('선택 모델 서버/bridge가 주행 중 종료됐습니다')
                import rosgraph
                owners = dict(rosgraph.Master('/planning_validator_guard').getSystemState()[0]).get(CANONICAL, [])
                if len(owners) != 1:
                    raise RuntimeFailure('canonical detection publisher는 정확히 하나여야 합니다: {}'.format(owners))
            if time.monotonic() - last_state[0] > 1.0:
                state["current"]["metrics"] = metrics
                _write_state(cfg, state)
                last_state[0] = time.monotonic()

        reason, elapsed = monitor.run_until_done(case, cfg["document"].get("validation", {}),
                                                  drive_start, progress)
        if safety['enabled'] and reason == 'SUCCESS':
            # Arrival can return before the next progress callback. Do not
            # certify SUCCESS using a stale final observation/control stream.
            monitor.localization_watchdog(pose_source=cfg['stack']['pose_source'],
                freshness_sec=safety['freshness_sec'])
            _verify_control_owner(monitor)
            if processes.running['udp_control'].poll() is not None:
                raise SafetyFailure('도착 확인 직전 UDP 제어 송신기 종료')
        monitor.event("RUN_FINISHED", reason=reason)
    except ScenarioFailure as exc:
        reason, error = exc.reason, str(exc)
    except SafetyFailure as exc:
        reason, error, halt_campaign = 'SAFETY_FAILURE', str(exc), True
    except KeyboardInterrupt:
        reason, error = "MANUAL_ABORT", "사용자 중단"
    except Exception as exc:
        error = str(exc)
        reason = "SYSTEM_ERROR"
        print("[{}] {}: {}".format(case.name, reason, error), file=sys.stderr)
    finally:
        monitor.event("CASE_RESULT", reason=reason, error=error)
        drive_evidence = monitor.drive_evidence()
        driving_started = monitor.drive_started_at is not None
        processes.stop("controller")
        if brake_hold is not None:
            brake_hold.stop()
        drive_reason = reason
        if safety['enabled']:
            if drive_reason == 'SYSTEM_ERROR':
                halt_campaign = True
            try:
                safety_receipt['shutdown'] = _stop_and_verify(cfg, processes, monitor)
            except Exception as exc:
                reason, error, halt_campaign = 'SAFETY_STOP_UNCONFIRMED', str(exc), True
                safety_receipt['shutdown'] = {'status': 'UNCONFIRMED', 'error': str(exc)}
            if drive_reason == 'COLLISION' and safety['stop_on_collision']:
                halt_campaign = True
            atomic_json(folder / 'safety_receipt.json', safety_receipt)
            if not (folder / 'runtime_receipt.json').is_file():
                try:
                    _capture_runtime_receipt(cfg, processes, folder, stage='failed_before_control_ready')
                except Exception as exc:
                    atomic_json(folder / 'runtime_receipt.json', {'stage': 'failed_before_control_ready',
                        'status': 'UNCONFIRMED', 'error': str(exc),
                        'localization_contract': cfg.get('localization_contract')})
        else:
            try:
                publish_brake()
            except Exception:
                pass
            processes.stop("udp_control")
        monitor.disarm()
        code = "P" if reason == "SUCCESS" else "E" if reason in ("SYSTEM_ERROR", "MANUAL_ABORT", 'SAFETY_FAILURE', 'SAFETY_STOP_UNCONFIRMED') else "F"
        final_base = basename.replace("_R_", "_{}_".format(code))
        recording_audit = None
        if not cfg['recording'].get('bag_enabled', True):
            recording_audit = {'status': 'DISABLED', 'bag_enabled': False,
                               're_inference_possible': False, 'complete_for_drive': False,
                               'note': '사용자가 bag OFF 선택. 실시간 분석·JSON·CSV는 유지됩니다.'}
            atomic_json(folder / 'recording_audit.json', recording_audit)
        try:
            bag, video = recorder.stop(final_base)
        except Exception as exc:
            bag = video = None
            reason, code = "RECORDING_FAILURE", "E"
            error = str(exc)
            final_base = basename.replace("_R_", "_E_")
            recording_audit = {'error': str(exc)}
        if bag is not None:
            try:
                recording_audit = audit_bag(bag, cfg, driving_started=driving_started)
            except Exception as exc:
                recording_audit = {'error': str(exc), 'status': 'WARN', 'warnings': [str(exc)],
                                   'complete_for_drive': False, 'complete_for_latency': False}
            atomic_json(folder / 'recording_audit.json', recording_audit)
            if recording_audit.get('status') == 'WARN':
                print('[WARN] 주행 판정과 별개: 녹화 경고. recording_audit.json 확인: {}'.format(
                    recording_audit.get('warnings', recording_audit.get('error'))), flush=True)
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
            "repeat_index": case.repeat_index, "repeat_total": case.repeat_total,
            "randomization": randomization,
            "realized_scenario_sha256": (hashlib.sha256((folder / "scenario.json").read_bytes()).hexdigest()
                                          if (folder / "scenario.json").is_file() else None),
            "attempt": attempt, "started_at": started_at, "finished_at": now(),
            "status": "PASS" if code == "P" else "ERROR" if code == "E" else "FAIL",
            "reason": reason, "error": error, "route_uuid": route_id,
            "route_profile": (json.loads((folder / 'route_profile.json').read_text())
                              if (folder / 'route_profile.json').is_file() else None),
            "drive_reason": drive_reason, "safety": safety_receipt, "halt_campaign": halt_campaign,
            "metrics": metrics, "failure_analysis": failure_analysis,
            "model": cfg.get('model_contract'), "recording_audit": recording_audit,
            "analysis_available": bool((recording_audit or {}).get('complete_for_analysis'))
                and (processes.log_dir / 'commands.json').is_file()
                and (not localization_analysis_enabled(cfg['recording']) or
                     (safety_receipt.get('shutdown', {}).get('status') == 'VERIFIED'
                      and (folder / 'runtime_receipt.json').is_file()))
                and (cfg['stack']['detector'] != 'bevfusion' or
                     ((folder / 'actual_model_receipt.json').is_file() and
                      json.loads((folder / 'actual_model_receipt.json').read_text()).get('status') == 'VERIFIED')),
            "recording": {'bag_enabled': cfg['recording'].get('bag_enabled', True),
                          'video_enabled': cfg['recording'].get('video', {}).get('enabled', False),
                          'requested_topics': cfg['recording']['topics'],
                          'requested_regex': cfg['recording'].get('topic_regex', [])},
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
        if state.get("execution_contract") != EXECUTION_CONTRACT:
            raise RuntimeFailure("이전 장면별 설정 방식의 캠페인은 새 공통 설정과 섞어 resume할 수 없습니다. 기존 결과는 보존하고 새 이름의 YAML/campaign_name으로 실행하세요")
        if state.get("config_path") != str(cfg["path"]):
            raise RuntimeFailure("resume YAML 경로가 이전과 다릅니다")
        if state.get("config_sha256") != _config_sha(cfg):
            raise RuntimeFailure("resume 도중 YAML 내용이 바뀌었습니다. 이전 설정을 복원하거나 새 campaign_name을 쓰세요")
        return state
    return {"schema_version": 2, "execution_contract": EXECUTION_CONTRACT,
            "campaign": cfg["name"], "config_path": str(cfg["path"]),
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
    if not cfg['recording'].get('bag_enabled', True) and not cfg['recording'].get('video', {}).get('enabled'):
        minimum = 0.1  # Still reserve room for results and logs, not a multi-GB bag.
    if minimum is not None:
        free = shutil.disk_usage(cfg["output"]).free / (1024 ** 3)
        if free < float(minimum):
            raise RuntimeFailure("저장 공간 부족: {:.1f} GiB 남음 / 최소 {} GiB. 공간 확보 후 재개하세요".format(free, minimum))


def run(cfg, resume=True, rerun_failed=False):
    if cfg.get('path') and is_snapshot(cfg['path']):
        cfg = load_execution(cfg['path'])
    # Campaign-local locks alone allow two different YAMLs to race on one ROS graph.
    directory = ROOT / "planner_logs/validation"
    directory.mkdir(parents=True, exist_ok=True)
    master = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    key = hashlib.sha256(master.encode()).hexdigest()[:16]
    with (directory / (".ros_graph_" + key + ".lock")).open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeFailure("다른 validator가 같은 ROS master를 사용 중입니다. 중단·정리가 끝난 뒤 실행하세요") from error
        return _run_campaign(cfg, resume, rerun_failed)


def _run_campaign(cfg, resume=True, rerun_failed=False):
    cfg["output"].mkdir(parents=True, exist_ok=True)
    # Close even when preflight/contract checks reject a run before Processes exists.
    with (cfg["output"] / ".lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeFailure("같은 캠페인이 이미 실행 중입니다") from error
        return _run_campaign_locked(cfg, resume, rerun_failed)


def _run_campaign_locked(cfg, resume=True, rerun_failed=False):
    mark_startup_phase(cfg, 'CONTRACT_CHECK')
    try:
        code = _run_campaign_impl(cfg, resume, rerun_failed)
        complete_startup(cfg, code)
        return code
    except (Exception, KeyboardInterrupt) as error:
        try:
            record_startup_failure(cfg, error)
            print('실패 단계: {} · 상세 로그: {}'.format(cfg.get('_startup_phase'),
                cfg['output'] / 'stack_logs/startup_error.log'), file=sys.stderr, flush=True)
        except Exception as audit_error:
            print('시작 오류 기록 실패: {}'.format(audit_error), file=sys.stderr, flush=True)
        raise


def _run_campaign_impl(cfg, resume=True, rerun_failed=False):
    mark_startup_phase(cfg, 'NETWORK_CHECK')
    if cfg.get('network_contract', {}).get('errors'):
        raise RuntimeFailure('\n'.join(cfg['network_contract']['errors']))
    mark_startup_phase(cfg, 'CONTRACT_CHECK')
    state = _load_state(cfg, resume)
    model = resolve_contract(cfg['stack'], cfg['network']['sensor_json'], ROOT)
    previous_model = state.get('model_contract') or cfg.get('model_contract')
    if previous_model:
        verify_contract(previous_model, cfg['stack'], cfg['network']['sensor_json'], ROOT)
    elif state.get('results') and model.get('preset') != 'e25' and model.get('preset') != 'od_gt':
        raise RuntimeFailure('모델 계약 없는 이전 결과에 Base5 resume은 금지합니다. 새 실험을 시작하세요')
    cfg['model_contract'] = model
    state['model_contract'] = model
    localization = resolve_localization_contract(cfg, ROOT)
    previous_localization = state.get('localization_contract') or cfg.get('localization_contract')
    if previous_localization and previous_localization.get('fingerprint') != localization['fingerprint']:
        raise RuntimeFailure('측위·제어 소스/설치본/config가 기존 실행과 다릅니다. 새 실험을 시작하세요')
    if not previous_localization and state.get('results') and (
            cfg.get('localization_safety', {}).get('enabled') or localization_analysis_enabled(cfg['recording'])):
        raise RuntimeFailure('측위 계약이 없는 이전 실행은 측위 실험으로 재개할 수 없습니다. 새 실험을 시작하세요')
    cfg['localization_contract'] = localization
    state['localization_contract'] = localization
    atomic_json(cfg['output'] / 'localization_runtime.json', localization)
    if cfg.get('network_contract'):
        atomic_json(cfg['output'] / 'network_contract.json', cfg['network_contract'])
    from .recording_audit import profile_contract_warnings
    state['recording_profile_warnings'] = profile_contract_warnings(cfg)
    state['recording_profile_warnings'] += sharing_warnings(cfg['recording'], cfg['upload'])
    for warning in state['recording_profile_warnings']:
        print('[WARN] ' + warning + ' · 기존 사본은 수정하지 않습니다. 새 실험으로 갱신하세요.', flush=True)
    if model['preset'] != 'od_gt':
        image_selection = runtime_image_selection(model)
        image_id = image_selection['image_id']
        if previous_model and previous_model.get('image_id') not in (None, image_id):
            raise RuntimeFailure('Docker image가 이전 모델 실행과 다릅니다. 같은 결과에 resume할 수 없습니다')
        model['image_id'] = image_id
        atomic_json(cfg['output'] / 'model_image_selection.json', image_selection)
        if image_selection['fallback']:
            print('[WARN] optional TensorRT image 없음: 같은 E12 eager로 시작합니다. ' +
                  image_selection['reason'], flush=True)
    atomic_json(cfg['output'] / 'model_runtime.json', model)
    state.update(run_status='RUNNING', runner_pid=os.getpid())
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
        state['run_status'] = 'DONE'
        _write_state(cfg, state)
        print("완료된 캠페인입니다: {}".format(cfg["output"]))
        return 0
    processes = Processes(cfg["output"] / "stack_logs", _env())
    try:
        mark_startup_phase(cfg, 'DISK_CHECK')
        check_free_disk(cfg)
        mark_startup_phase(cfg, 'TRAFFIC_LIGHT_CHECK')
        _check_traffic_light(cfg)
        mark_startup_phase(cfg, 'ROS_MASTER_CHECK')
        _start_master(processes, float(cfg["document"].get("readiness_timeout_sec", 45)))
        mark_startup_phase(cfg, 'ROS_NODE_INIT')
        import rospy
        rospy.init_node("planning_scenario_validator", anonymous=False, disable_signals=True)
        monitor = LiveMonitor()
        mark_startup_phase(cfg, 'STACK_START')
        _start_persistent(cfg, processes, monitor)
        for index, case in enumerate(todo, 1):
            if is_snapshot(cfg['path']):
                load_execution(cfg['output'])
            # Check between cases without corrupting an in-progress recorder.
            check_free_disk(cfg)
            print("[{}/{}] {}".format(index, len(todo), case.name), flush=True)
            mark_startup_phase(cfg, 'CASE_RUN')
            result = _run_case(cfg, processes, monitor, case, state)
            print("  -> {} ({})".format(result["status"], result["reason"]), flush=True)
            if result.get('halt_campaign'):
                print('안전 gate에 의해 후속 시나리오를 중단합니다.', flush=True)
                break
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
        total_complete = sum(row['status'] in ('PASS', 'FAIL') for row in state['results'].values()) == len(cfg['cases'])
        state['run_status'] = 'DONE' if total_complete else 'INTERRUPTED'
        _write_state(cfg, state)
    counts = {label: sum(row["status"] == label for row in state["results"].values())
              for label in ("PASS", "FAIL", "ERROR")}
    print("캠페인: {} | 성공 {} / 실패 {} / 시스템 오류 {} / 전체 {}".format(
        cfg["output"], counts["PASS"], counts["FAIL"], counts["ERROR"], len(cfg["cases"])))
    return 0 if len(state["results"]) == len(cfg["cases"]) and not counts["ERROR"] else 1


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.execution:
            cfg = load_execution(args.execution)
        elif args.config:
            cfg = load_execution(args.config) if is_snapshot(args.config) else load_campaign(args.config)
        else:
            raise ConfigError('새 실행은 YAML, 재개는 --execution 결과폴더를 지정하세요')
        network_contract = cfg.get('network_contract', {})
        if network_contract.get('errors'):
            raise ConfigError('\n'.join(network_contract['errors']))
        for warning in network_contract.get('warnings', []):
            print('[WARN] ' + warning, flush=True)
        if cfg["ignored_case_yamls"]:
            print("안내: 예전 .case.yaml {}개는 읽지 않습니다. 이 YAML의 공통 목적지·시간을 모든 장면에 적용합니다.".format(
                len(cfg["ignored_case_yamls"])), file=sys.stderr)
        if args.check or args.dry_run:
            if cfg['stack']['detector'] == 'bevfusion' and selected_model(cfg['stack']) != 'e25':
                model = resolve_contract(cfg['stack'], cfg['network']['sensor_json'], ROOT)
                print('모델 계약 확인: {} / SHA256 {} / V/P/O {}'.format(
                    model['label'], model['checkpoint_sha256'], model['thresholds']))
            print("설정 확인: {} | JSON {}개 × 반복 {}회 = 총 {}회 | 설치 {}".format(
                cfg["name"], len({case.source for case in cfg["cases"]}),
                cfg['cases'][0].repeat_total, len(cfg["cases"]), cfg["install"]))
            if cfg.get('route'):
                print('경로 정책: {} / 기존 목적지 유지 / 선호 차선 일치 확인 후 주행'.format(cfg['route']['name']))
            if args.dry_run:
                for index, case in enumerate(cfg["cases"], 1):
                    print("{:02d}. {} -> goal ({:.1f}, {:.1f}) / {}초".format(index, case.name,
                        case.goal["x"], case.goal["y"], case.timeout_sec))
            return 0
        if args.execution or is_snapshot(cfg['path']):
            cfg = load_execution(args.execution or cfg['path'])
        elif args.resume == 'true' and (cfg['output'] / 'campaign_state.json').is_file():
            cfg = load_execution(cfg['output'])  # Existing pre-snapshot records only.
            print('이전 고정 이름 실험을 재개합니다. 새 실험은 --resume 옵션 없이 원본 YAML을 실행하세요.')
        else:
            cfg = prepare_execution(cfg['path'])
            print('새 실험: {}'.format(cfg['output']), flush=True)
        print('실행 기록: {}'.format(cfg['output']), flush=True)
        resume = args.resume != 'false'
        def on_terminate(_signal, _frame):
            raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM, on_terminate)
        return run(cfg, resume, args.rerun_failed)
    except (ConfigError, RuntimeFailure, ValueError, OSError) as error:
        print("planning_validator 오류: {}".format(error), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("중단했습니다. GUI 실험 기록에서 재개하거나 --execution 결과폴더를 지정하세요.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
