"""Conservative, evidence-backed classification of one closed-loop run.

These labels describe *observed symptoms*, not a proven code-level root cause.
Only live monitor values and log lines written during this scenario are used;
the rosbag is retained for a later human investigation but is not replayed.
"""

import re
from pathlib import Path


LOG_PATTERNS = {
    "MPT_NO_ALIGNED_PATH": re.compile(r"no aligned previous path", re.I),
    "MPT_FOOTPRINT_OVERLAP": re.compile(
        r"Measured footprint already overlaps static obstacle clearance; stopping", re.I),
    "EMPTY_ROUTE": re.compile(r"DefaultPlanner returned an empty route", re.I),
}
LOG_FILES = ("planning.log", "controller.log", "pose.log", "bevfusion.log")


def compact_summary(analysis):
    """Keep the CSV scannable while exposing the decisive numerical evidence."""
    important = ("출발점", "궤적 앞", "planner 로그", "제어 명령", "검출 메시지", "충돌 객체", "실행 오류")
    evidence = [item for item in analysis.get("evidence", [])
                if any(token in item for token in important)]
    if not evidence:
        evidence = analysis.get("evidence", [])
    else:
        evidence.sort(key=lambda item: next(index for index, token in enumerate(important)
                                            if token in item))
    return " | ".join([analysis.get("summary", "")] + evidence[:3])[:350]


def log_offsets(log_dir):
    """Bookmark shared campaign logs so a previous case cannot contaminate this one."""
    directory = Path(log_dir)
    return {name: (directory / name).stat().st_size if (directory / name).is_file() else 0
            for name in LOG_FILES}


def scan_log_signals(log_dir, offsets):
    signals = {}
    for name in LOG_FILES:
        path = Path(log_dir) / name
        if not path.is_file():
            continue
        with path.open("rb") as stream:
            stream.seek(min(offsets.get(name, 0), path.stat().st_size))
            for raw_line in stream:
                line = raw_line.decode("utf-8", errors="replace")
                for code, pattern in LOG_PATTERNS.items():
                    if not pattern.search(line):
                        continue
                    key = (name, code)
                    if key not in signals:
                        signals[key] = {"code": code, "log": str(path), "count": 0,
                                        "sample": line.strip()[:240]}
                    signals[key]["count"] += 1
    return sorted(signals.values(), key=lambda item: (item["code"], item["log"]))


def analyze_failure(reason, metrics, *, error=None, goal_tolerance_m=2.0, log_signals=()):
    """Return a concise category and the evidence that supports it."""
    signals = list(log_signals)
    analysis = {"schema_version": 1, "category": "UNDETERMINED",
                "summary": "관측값만으로 실패 양상을 구분할 수 없음",
                "suspected_stage": "undetermined", "stage_confidence": "none",
                "evidence": [], "log_signals": signals,
                "root_cause_confirmed": False}
    evidence = analysis["evidence"]
    if error:
        evidence.append("실행 오류: {}".format(str(error)[:240]))
    if reason == "SUCCESS":
        analysis.update(category="SUCCESS", summary="목적지 도착 및 충돌 없음",
                        suspected_stage="not_applicable")
        return analysis
    if reason == "COLLISION":
        analysis.update(category="COLLISION", summary="/CollisionData에서 충돌 관측",
                        suspected_stage="collision_observation", stage_confidence="high")
        evidence.append("충돌 객체 {}개".format(metrics.get("collision_count", "?")))
        return analysis
    if reason == "ROUTE_FAILURE":
        analysis.update(category="ROUTE_FAILURE", summary="목적지 route 요청 거부 또는 경로 준비 실패",
                        suspected_stage="route_planning", stage_confidence="high")
        return analysis
    if reason == "NO_TRAJECTORY":
        analysis.update(category="NO_TRAJECTORY", summary="route 요청 후 새 planning trajectory가 준비되지 않음",
                        suspected_stage="planning_output", stage_confidence="high")
        return analysis
    if reason in ("SYSTEM_ERROR", "RECORDING_FAILURE"):
        analysis.update(category="SYSTEM_ERROR", summary="실행·연결·녹화 시스템 오류",
                        suspected_stage="runtime", stage_confidence="medium")
        if error and ("localization topic이" in error or
                      "planning trajectory가" in error) and "끊겼습니다" in error:
            analysis.update(category="TOPIC_LOSS", summary="주행 중 필수 위치·궤적 토픽 단절",
                            suspected_stage="localization" if "localization" in error
                            else "planning_output", stage_confidence="high")
        elif error and "BEVFusion detection 준비 제한시간" in error:
            analysis.update(category="DETECTION_UNAVAILABLE",
                            summary="BEVFusion 검출 토픽이 준비되지 않음",
                            suspected_stage="detection_runtime", stage_confidence="high")
        elif error and "controller가 주행 중 종료" in error:
            analysis.update(category="CONTROLLER_EXIT", summary="주행 중 제어 노드 종료",
                            suspected_stage="controller_runtime", stage_confidence="high")
        return analysis
    if reason == "MANUAL_ABORT":
        analysis.update(category="MANUAL_ABORT", summary="사용자가 실행 중단")
        return analysis
    if reason not in ("TIMEOUT", "STUCK"):
        evidence.append("기존 판정: {}".format(reason))
        return analysis

    elapsed = metrics.get("elapsed_sec")
    displacement = metrics.get("max_displacement_m")
    peak_speed = metrics.get("maximum_speed_mps")
    final_speed = metrics.get("final_speed_mps")
    stationary = metrics.get("stationary_duration_sec")
    last_window = metrics.get("last_5s_displacement_m")
    goal_min = metrics.get("minimum_goal_distance_m")
    if elapsed is not None:
        evidence.append("주행 관측 {:.1f}초".format(elapsed))
    if displacement is not None:
        evidence.append("출발점 기준 최대 이동 {:.2f}m, 최대 속도 {:.2f}m/s".format(
            displacement, peak_speed or 0.0))
    if goal_min is not None:
        evidence.append("목적지 최소 거리 {:.2f}m".format(goal_min))
    if stationary is not None and final_speed is not None:
        evidence.append("종료 시 속도 {:.2f}m/s, 마지막 유의미한 속도 이후 {:.1f}초".format(
            final_speed, stationary))
    if displacement is None or peak_speed is None:
        analysis.update(category="INSUFFICIENT_OBSERVATION",
                        summary="주행 중 위치·속도 관측이 부족해 시간초과 유형을 판정할 수 없음")
        return analysis
    if displacement <= 0.5 and peak_speed <= 0.5:
        analysis.update(category="NO_DEPARTURE", summary="제어 시작 후 차량이 출발하지 않음")
    elif goal_min is not None and goal_min <= goal_tolerance_m:
        analysis.update(category="ARRIVAL_CONDITION_UNMET",
                        summary="목적지 반경에 접근했지만 정차·유지 조건 미충족")
    elif (stationary is not None and stationary >= 3.0 and final_speed is not None
          and final_speed <= 0.3 and last_window is not None and last_window <= 0.5):
        analysis.update(category="MID_DRIVE_STOP", summary="주행 후 목적지 이전에 정지")
    else:
        analysis.update(category="GOAL_NOT_REACHED", summary="제한 시간 내 목적지에 도착하지 못함")

    if analysis["category"] in ("NO_DEPARTURE", "MID_DRIVE_STOP"):
        near_speed = (metrics.get("trajectory_near_max_speed_mps")
                      if analysis["category"] == "NO_DEPARTURE" else
                      metrics.get("latest_trajectory_near_speed_mps"))
        command_count = metrics.get("control_command_count", 0)
        max_accel = metrics.get("max_accel_command", 0.0)
        latest_accel = metrics.get("latest_accel_command")
        latest_brake = metrics.get("latest_brake_command")
        if near_speed is not None:
            evidence.append("궤적 앞 5점의 {} 속도 {:.2f}m/s".format(
                "최대" if analysis["category"] == "NO_DEPARTURE" else "종료 시 최대", near_speed))
        evidence.append("제어 명령 {}개, 최대 가속 명령 {:.2f}".format(command_count, max_accel))
        detector_messages = metrics.get("detection_message_count")
        if detector_messages is not None:
            evidence.append("검출 메시지 {}개 중 객체 포함 {}개, 최신 객체 {}개".format(
                detector_messages, metrics.get("detection_nonempty_message_count", "?"),
                metrics.get("latest_detection_objects", "?")))
        if latest_accel is not None and latest_brake is not None:
            evidence.append("마지막 가속/제동 명령 {:.2f}/{:.2f}".format(
                latest_accel, latest_brake))
        mpt_count = sum(item["count"] for item in signals
                        if item["code"] == "MPT_NO_ALIGNED_PATH")
        overlap_count = sum(item["count"] for item in signals
                            if item["code"] == "MPT_FOOTPRINT_OVERLAP")
        if mpt_count:
            evidence.append("해당 장면 planner 로그의 'no aligned previous path' {}회".format(mpt_count))
        if overlap_count:
            evidence.append("해당 장면 planner 로그의 'footprint overlaps static obstacle clearance; stopping' {}회".format(
                overlap_count))
        if near_speed is not None and near_speed <= 0.1:
            if overlap_count >= 3:
                analysis.update(suspected_stage="planning_safety_stop", stage_confidence="high")
            else:
                analysis.update(suspected_stage="planning_output", stage_confidence="medium")
        elif near_speed is not None and near_speed > 0.5 and command_count > 0:
            if analysis["category"] == "NO_DEPARTURE" and max_accel > 0.05:
                analysis.update(suspected_stage="vehicle_or_command_interface",
                                stage_confidence="low")
            elif latest_brake is not None and latest_brake > 0.1 and (
                    latest_accel is None or latest_accel <= 0.05):
                analysis.update(suspected_stage="controller_output", stage_confidence="low")
    return analysis
