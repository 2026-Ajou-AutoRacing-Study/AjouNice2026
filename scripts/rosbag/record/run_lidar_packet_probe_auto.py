#!/usr/bin/env python3
"""Automatic raw-packet probe using existing validator load/control/stop paths.

User connects MORAI once. No edits to the validator or production policies.
"""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import time
import types

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
VALIDATOR = ROOT / "base_ws/src/planning_addons/planning_scenario_validator"
NETWORK = VALIDATOR / "config/internal/generated/ego_network_validator_local.json"
BENCHMARK = ROOT / "configs/scenario/perception_benchmark"
INSTALL = pathlib.Path.home() / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
sys.path.insert(0, str(VALIDATOR / "src"))
sys.path.insert(0, str(ROOT / "base_ws/src/control/trajectory_batch_runner/src"))
import record_lidar_packet_probe as probe


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def prepare(folder):
    """Create independent input snapshots; do not run ROS or overwrite campaigns."""
    folder.mkdir(parents=True, exist_ok=False)
    profile = folder / "topics.json"
    base = json.loads((VALIDATOR / "config/internal/topic_profiles/basic.json").read_text())
    write_json(profile, sorted(set(base) | set(probe.REQUIRED) | {
        "/competition/raw_twist", "/vehicle/status/gear", "/vehicle/status/control_mode",
        "/dataset/scenario_marker"}))
    result = {}
    for mission, scene in (("highway", "bench_highway_merge_raw.json"),
                           ("roundabout", "bench_roundabout_raw.json")):
        cfg = yaml.safe_load((VALIDATOR / f"config/missions/{mission}/base/example.yaml").read_text())
        cfg["campaign_name"] = f"lidar_packet_{mission}"
        cfg["scenarios"] = {"directory": str(BENCHMARK), "glob": "*.json", "include": [scene], "repeat": 1}
        cfg["stack"] = {"pose_source": "maptf_morai", "detector": "od_gt",
                        "traffic_light": False, "planviz": False}
        cfg["recording"] = {"topic_profile": str(profile), "include_sensors": True,
                            "bag_enabled": True, "analysis_mode": False,
                            "topic_regex": probe.OPTIONAL_REGEX,
                            "video": {"enabled": False}, "minimum_free_disk_gib": 10}
        cfg["upload"]["enabled"] = False
        cfg["resume"] = False
        cfg["output_root"] = str(folder / "validator")
        cfg["morai"]["scenario_install_dir"] = str(INSTALL)
        path = folder / (mission + ".yaml")
        path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
        result[mission] = str(path)
    write_json(folder / "setup.json", {
        "campaigns": result, "network": str(NETWORK),
        "scope": "Raw input collection with GT pose/detection; not production full-stack evaluation.",
        "driver_input": "unchanged VLP16 900 RPM, first packet stamp, 51 packets"})
    return result


def static_capture(folder):
    """Reuse validator's collision monitor, brake hold and UDP loader."""
    import rospy
    from planning_scenario_validator.runtime import (
        Processes, LiveMonitor, BrakeHold, check_no_conflicting_publishers,
        prepare_scenario, publish_brake, RuntimeFailure)
    from planning_scenario_validator.runner import _load_scene

    check_no_conflicting_publishers()
    rospy.init_node("lidar_packet_probe_auto", anonymous=True, disable_signals=True)
    processes = Processes(folder / "static_stack_logs", os.environ.copy())
    monitor = LiveMonitor()
    hold = BrakeHold()
    hold.start()
    try:
        processes.start("udp", ["roslaunch", "sensor_interface", "morai_udp_interface.launch",
                                f"sensor_config:={ROOT / 'configs/sensors/Sensor_competition_local.json'}",
                                f"network_config:={NETWORK}"])
        monitor.wait(lambda: monitor.stamps.get("ego", 0) > 0, 15, "/Ego_topic")
        if (monitor.ego.velocity.x ** 2 + monitor.ego.velocity.y ** 2) ** 0.5 > 0.5:
            raise RuntimeFailure("초기 Ego가 아직 움직이고 있어. 정지 확인 후 다시 시작한다.")
        source = BENCHMARK / probe.CASES["static"]
        case = types.SimpleNamespace(source=source, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                                     install_stem="0_lidar_auto_static")
        installed, xy = prepare_scenario(case, INSTALL, NETWORK)
        write_json(folder / "static_loaded_scenario.json", json.loads(installed.read_text()))
        _load_scene({"document": {"scenario_ready_timeout_sec": 20,
                                  "morai": {"scenario_load_host": "127.0.0.1", "scenario_load_port": 9095}}},
                    installed, monitor, xy)
        time.sleep(3)
        before = (monitor.ego.position.x, monitor.ego.position.y)
        monitor.arm(folder / "static_events.jsonl")
        recorder = processes.start("record", [sys.executable, str(pathlib.Path(probe.__file__)),
                                               "--case", "static", "--duration", "30", "--run",
                                               "--network-profile", "validator", "--output-root", str(folder / "static")])
        deadline = time.monotonic() + 70
        while recorder.poll() is None:
            if time.monotonic() > deadline:
                raise RuntimeFailure("정지 녹화 시간 초과")
            if monitor.collision_count or monitor.far_collision:
                raise RuntimeFailure("정지 녹화 중 접촉 발생")
            if time.monotonic() - monitor.stamps.get("ego", 0) > 1.0:
                raise RuntimeFailure("정지 녹화 중 Ego 수신 중단")
            if ((monitor.ego.position.x - before[0]) ** 2 +
                (monitor.ego.position.y - before[1]) ** 2) ** 0.5 > 0.2:
                raise RuntimeFailure("정지 녹화 중 Ego 20cm 초과 이동")
            time.sleep(0.1)
        if recorder.returncode:
            raise RuntimeFailure(f"정지 녹화 검사 실패: {recorder.returncode}")
    finally:
        publish_brake(duration=1)
        hold.stop()
        processes.close()


def run(folder, campaigns):
    from planning_scenario_validator.config import load_campaign

    # Validate both driving cases before any simulator mutation.
    for path in campaigns.values():
        cfg = load_campaign(path)
        if "/sensor/lidar1/packets" not in cfg["recording"]["topics"]:
            raise RuntimeError("원시 패킷 녹화 토픽 누락")
    static_capture(folder)
    for mission, path in campaigns.items():
        command = [sys.executable, str(VALIDATOR / "scripts/planning_validator.py"), path]
        with (folder / (mission + "_runner.log")).open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=420)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                probe.stop_recorder(process)
                raise RuntimeError(f"{mission} 자동 실행 중단; 로그/원본 보존: {folder}")
        if code:
            raise RuntimeError(f"{mission} validator 종료 코드 {code}; 로그: {folder / (mission + '_runner.log')}")
        states = sorted((folder / "validator").glob(f"lidar_packet_{mission}_*/campaign_state.json"))
        if len(states) != 1:
            raise RuntimeError(f"{mission} 결과 폴더가 모호하거나 누락됨")
        state = json.loads(states[0].read_text())
        if not state.get("results") or any(row["status"] != "PASS" for row in state["results"].values()):
            raise RuntimeError(f"{mission} 주행 미완료/실패. 결과 보존: {states[0]}")
        bags = list(states[0].parent.rglob("*.bag"))
        if len(bags) != 1:
            raise RuntimeError(f"{mission} 마감된 bag 개수가 예상과 다름: {len(bags)}")
        audit = probe.audit_bag(bags[0])
        write_json(states[0].parent / "packet_probe_audit.json", audit)
        if not audit["pass"]:
            raise RuntimeError(f"{mission} 원시 입력 검사 실패")
        print(f"{mission} 자동 주행·원시 패킷 검사 완료: {bags[0]}", flush=True)
    verify_final_stop(folder)


def verify_final_stop(folder):
    """Check actual stop after validator braking; never reload merely to park."""
    import rospy
    from planning_scenario_validator.runtime import Processes, check_no_conflicting_publishers
    from morai_msgs.msg import EgoVehicleStatus
    from std_msgs.msg import Int8

    check_no_conflicting_publishers()
    processes = Processes(folder / "final_stop_logs", os.environ.copy())
    try:
        processes.start("udp_readonly", ["roslaunch", "sensor_interface", "morai_udp_interface.launch",
                                         f"sensor_config:={ROOT / 'configs/sensors/Sensor_competition_local.json'}",
                                         f"network_config:={NETWORK}", "enable_control:=false"])
        deadline = time.monotonic() + 15
        stopped_since = None
        while time.monotonic() < deadline:
            try:
                ego = rospy.wait_for_message("/Ego_topic", EgoVehicleStatus, timeout=2)
                gear = rospy.wait_for_message("/vehicle/status/gear", Int8, timeout=2).data
            except rospy.ROSException:
                continue
            speed = (ego.velocity.x ** 2 + ego.velocity.y ** 2 + ego.velocity.z ** 2) ** 0.5
            if gear in (1, 4) and speed < 0.1:
                stopped_since = stopped_since or time.monotonic()
                if time.monotonic() - stopped_since >= 0.5:
                    write_json(folder / "final_stop.json", {"gear": gear, "speed_mps": speed,
                                                          "verified": True, "park_verified": gear == 1})
                    print("제어기 종료 후 실제 정지 확인 완료. 추가 시나리오 로드는 하지 않는다.", flush=True)
                    return
            else:
                stopped_since = None
        raise RuntimeError("제어기 종료 후 실제 정지 수신 확인 실패")
    finally:
        processes.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=pathlib.Path,
                        default=probe.ROOT / "rosbags/eval/perception/lidar_packet_probe" /
                        ("auto_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")))
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()
    campaigns = prepare(args.output)
    print(f"독립 실험 설정 준비: {args.output}", flush=True)
    if args.run:
        run(args.output, campaigns)


if __name__ == "__main__":
    main()
