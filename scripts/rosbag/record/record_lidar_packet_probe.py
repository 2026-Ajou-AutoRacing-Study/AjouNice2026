#!/usr/bin/env python3
"""Record unchanged MORAI input for offline packet-boundary experiments.

Never starts ROS nodes, loads a scenario, drives Ego, or changes parameters.
--list works without ROS; --check receives input without writing a bag.
"""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import signal
import subprocess
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parents[3]
CASES = {
    "static": "bench_static_obstacle_base3_assets_raw.json",
    "highway": "bench_highway_merge_raw.json",
    "curve": "bench_roundabout_raw.json",
}
NETWORKS = {
    "competition": "configs/network/ego_network_competition_local.json",
    "validator": "base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json",
}
REQUIRED = {
    "/sensor/lidar1/packets": "velodyne_msgs/VelodyneScan",
    "/sensor/lidar1": "sensor_msgs/PointCloud2",
    **{f"/sensor/camera/{camera}/compressed": "sensor_msgs/CompressedImage"
       for camera in ("front", "front_left", "front_right", "back")},
    "/sensor/imu": "sensor_msgs/Imu",
    "/Ego_topic": "morai_msgs/EgoVehicleStatus",
    "/Competition_topic": "morai_msgs/EgoVehicleStatus",
    "/Object_topic": "morai_msgs/ObjectStatusList",
}
OPTIONAL_REGEX = [
    "^/(sensor/gps|CollisionData|competition/raw_twist|vehicle/status/(gear|control_mode))$",
    "^/(tf|tf_static|clock|diagnostics|rosout|localization/kinematic_state)$",
    "^/perception/(bevfusion|base5_trial)/(status|latency_ms)$",
    "^/perception/object_recognition/(detection/objects|tracking/measurement_objects|tracking/objects|objects)$",
]


def missing_inputs(counts, minimum=2):
    return [topic for topic in REQUIRED if counts.get(topic, 0) < minimum]


def packet_summary(scans):
    sizes, lengths, first_deltas, spans = set(), set(), [], []
    scans_count = 0
    for scan in scans:
        scans_count += 1
        sizes.add(len(scan.packets))
        lengths.update(len(packet.data) for packet in scan.packets)
        if scan.packets:
            first_deltas.append(scan.header.stamp.to_sec() - scan.packets[0].stamp.to_sec())
            spans.append(scan.packets[-1].stamp.to_sec() - scan.packets[0].stamp.to_sec())
    return {
        "scans": scans_count,
        "packet_counts": sorted(sizes),
        "packet_bytes": sorted(lengths),
        "header_minus_first_packet_sec_max_abs": max(map(abs, first_deltas), default=None),
        "packet_span_sec_min": min(spans, default=None),
        "packet_span_sec_max": max(spans, default=None),
    }


def require_driver_contract(params):
    if params.get("model") != "VLP16" or params.get("rpm") != 900.0:
        raise RuntimeError("기존 VLP16 / 900 RPM 설정이 아니야. 값을 바꾸지 말고 확인해 줘.")
    if params.get("timestamp_first_packet") is not True:
        raise RuntimeError("첫 패킷 timestamp 설정을 확인해 줘.")
    if params.get("cut_angle", 0) >= 0 or params.get("time_offset", 1) != 0:
        raise RuntimeError("현재 cut_angle 비활성 / time_offset=0 계약과 달라.")
    if params.get("gps_time") is not False or params.get("pcap_time") is not False:
        raise RuntimeError("live 호스트 시간 계약과 달라.")
    if "npackets" in params and params["npackets"] != 51:
        raise RuntimeError("npackets override가 51이 아니야.")


def preflight(timeout, network_profile="competition"):
    import rosgraph
    import rospy

    try:
        rosgraph.Master("/lidar_packet_probe_preflight").getPid()
    except Exception as error:
        raise RuntimeError("ROS master에 연결할 수 없어. 먼저 rc와 UDP 수신기를 켜 줘.") from error
    rospy.init_node("lidar_packet_probe_preflight", anonymous=True, disable_signals=True)
    if rospy.get_param("/use_sim_time", False):
        raise RuntimeError("live 녹화는 /use_sim_time=false가 필요해. 실행기가 자동 변경하지는 않아.")
    params = rospy.get_param("/lidar/lidar_driver")
    require_driver_contract(params)
    # Verify that the receiver and the documented profiles describe the same input.
    for node, key, expected in (
        ("morai_sensor_udp_receiver", "sensor_config", "configs/sensors/Sensor_competition_local.json"),
        ("morai_ego_udp_receiver", "network_config", NETWORKS[network_profile]),
    ):
        actual = pathlib.Path(rospy.get_param(f"/{node}/{key}"))
        if actual.read_bytes() != (ROOT / expected).read_bytes():
            raise RuntimeError(f"{node} 설정이 지정한 프로필과 달라: {actual}")
    counts, types, lock = {}, {}, threading.Lock()

    def callback(message, topic):
        with lock:
            counts[topic] = counts.get(topic, 0) + 1
            types[topic] = message._connection_header.get("type")

    subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, callback, callback_args=topic,
                                   queue_size=2) for topic in REQUIRED]
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            with lock:
                complete = not missing_inputs(counts)
            if complete:
                break
            time.sleep(0.05)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
    missing = missing_inputs(counts)
    wrong_types = [topic for topic, actual in types.items() if actual != REQUIRED[topic]]
    if missing or wrong_types:
        raise RuntimeError(f"필수 입력 누락={missing}, 타입 불일치={wrong_types}")
    calibration = pathlib.Path(rospy.get_param("/lidar/lidar_transform/calibration"))
    return {"driver": params, "transform": rospy.get_param("/lidar/lidar_transform"),
            "calibration": {"path": str(calibration), "content": calibration.read_text(),
                            "sha256": hashlib.sha256(calibration.read_bytes()).hexdigest()},
            "fresh_message_counts": counts,
            "types": types, "use_sim_time": False}


def provenance(case, live, network_profile="competition"):
    files = {
        "sensor": ROOT / "configs/sensors/Sensor_competition_local.json",
        "network": ROOT / NETWORKS[network_profile],
        "declared_scenario": ROOT / "configs/scenario/perception_benchmark" / CASES[case],
    }
    snapshots = {key: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                       "content": json.loads(path.read_text())} for key, path in files.items()}
    return {"case": case, "scenario_is_user_declared_not_live_verified": True,
            "environment_is_user_declared": "Sunny13", "profiles": snapshots,
            "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            "git_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
            "live_preflight": live,
            "scope": "local raw input; not team 4090 full-stack/LAN latency certification"}


def audit_bag(path):
    import rosbag

    with rosbag.Bag(str(path)) as bag:
        counts = {topic: info.message_count for topic, info in bag.get_type_and_topic_info().topics.items()}
        packets = packet_summary(message for _, message, _ in bag.read_messages(topics=["/sensor/lidar1/packets"]))
        duration = bag.get_end_time() - bag.get_start_time()
    missing = missing_inputs(counts)
    valid_packets = (packets["packet_counts"] == [51] and packets["packet_bytes"] == [1206]
                     and packets["header_minus_first_packet_sec_max_abs"] is not None
                     and packets["header_minus_first_packet_sec_max_abs"] < 1e-6
                     and packets["packet_span_sec_min"] >= 0)
    return {"counts": counts, "duration_sec": duration, "packets": packets,
            "missing_inputs": missing, "pass": not missing and valid_packets,
            "limitations": "Presence/packet contract only; motion, collisions, frame gaps and GT quality need offline review."}


def stop_recorder(process):
    """Stop only this recorder's process group, including rosbag's CLI child."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, default="static")
    parser.add_argument("--network-profile", choices=NETWORKS, default="competition")
    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--preflight-timeout", type=int, default=10)
    parser.add_argument("--output-root", type=pathlib.Path,
                        default=ROOT / "rosbags/eval/perception/lidar_packet_probe")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--list", action="store_true")
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--run", action="store_true")
    args = parser.parse_args()
    if not 5 <= args.duration <= 120 or not 1 <= args.preflight_timeout <= 30:
        parser.error("duration=5..120, preflight-timeout=1..30")
    if args.list:
        print(json.dumps({"required": REQUIRED, "optional_regex": OPTIONAL_REGEX,
                          "scenarios": CASES}, ensure_ascii=False, indent=2))
        return 0
    live = preflight(args.preflight_timeout, args.network_profile)
    print("필수 입력 수신과 기존 드라이버 설정 확인 완료.", flush=True)
    if args.check:
        print(json.dumps(live, ensure_ascii=False, indent=2))
        return 0
    snapshot = provenance(args.case, live, args.network_profile)
    name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + args.case
    output = args.output_root / name
    output.mkdir(parents=True, exist_ok=False)
    bag_path = output / "raw.bag"
    metadata = output / "capture.json"
    metadata.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n")
    command = ["rosbag", "record", "--lz4", "--buffsize=512", f"--duration={args.duration}",
               "--regex", "-O", str(bag_path), *[f"^{topic}$" for topic in REQUIRED], *OPTIONAL_REGEX]
    print(f"{args.duration}초 녹화: {output}\n시나리오 로드·운전·노드 시작은 자동으로 하지 않아.", flush=True)
    process = subprocess.Popen(command, start_new_session=True)
    try:
        returncode = process.wait(timeout=args.duration + 30)
    except KeyboardInterrupt:
        stop_recorder(process)
        raise RuntimeError(f"사용자가 녹화를 중단했어. 부분 녹화 원본 보존: {output}")
    except subprocess.TimeoutExpired:
        stop_recorder(process)
        raise RuntimeError(f"녹화 종료 시간 초과. {output}의 원본을 보존하고 프로세스 상태를 확인해 줘.")
    if returncode != 0:
        raise RuntimeError(f"rosbag 종료 코드 {returncode}; 원본 보존: {output}")
    audit = audit_bag(bag_path)
    (output / "audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    return 0 if audit["pass"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, KeyError) as error:
        raise SystemExit(f"녹화 준비/검사 실패: {error}")
