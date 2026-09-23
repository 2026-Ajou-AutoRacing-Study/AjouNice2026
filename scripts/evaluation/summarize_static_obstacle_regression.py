"""Summarize a completed replay without treating stops as avoidance successes."""
import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("run", type=Path)
args = parser.parse_args()
rows = json.loads((args.run / "summary.json").read_text())
report = [
    "# 장애물 회피 재생 결과", "",
    "기록된 자차 상태를 입력하는 재생이다. 폐루프 주행 성공률이 아니다.", "",
    "PO 정지 및 최종 정지는 장애물 장면이 활성인 프레임만 센다. "
    "정지 판정은 속도 크기 0.01m/s 미만이다. "
    "괄호 안은 그중 실제 입력 차체가 관측 장애물의 하드 여유와 겹친 프레임이다.", "",
    "도로 검사는 전방 47.5m의 사각 차체 기준이며 수치 허용치는 1mm다. "
    "장애물 검사는 첫 정지점까지의 사각 차체와 관측 envelope + hard margin 기준이다.", "",
    "장면 검증 기록이 있는 실행의 최종 검사는 출력 sequence·CRC32로 연결한 실제 검증 장면과 "
    "자차 상태를 사용한다. 독립 기록기가 먼저 받은 더 최신 장면과의 비교 및 안전 출력 갱신 지연도 "
    "백별로 별도 보고한다. 두 토픽의 기록기 수신 순서는 검증기 수신 순서를 증명하지 않는다.", "",
    "PO 처리시간 통계는 초기화 재생의 측정도 포함한다. 출력 간격·출력 수·형상·정지 검사는 "
    "백 시작 이후를 대상으로 한다.", "",
    "| 백 | PO / 최종 출력 | PO 정지 | 최종 정지 (입력 침범) | 최종 장애물 / 도로 침범 | PO p95 / 최대 ms | 출력 최대 간격 s | 자동 검사 |",
    "|---|---:|---:|---:|---:|---:|---:|---|",
]
details = []
summary = {}
for row in rows:
    name = Path(row["bag"]).stem
    directory = args.run / name
    if "execution_failure" in row:
        report.append(f"| {name[-6:]} | 실행 실패 | | | | | | FAIL |")
        continue
    stage = json.loads((directory / "stage_metrics.json").read_text())
    metrics = json.loads((directory / "metrics.json").read_text())
    replay = json.loads((directory / "result.json").read_text())
    final = stage["/planning/trajectory"]["counts"]
    times = stage["po_processing_ms_p50_p95_max"]
    errors = row.get("safety_coverage_gate_errors", ["not_evaluated"])
    report.append(
        f"| {name[-6:]} | {row['po_frames']} / {final.get('frames', 0)} "
        f"| {row['active_zero_at_ego']} "
        f"| {final.get('zero_at_ego', 0)} ({final.get('zero_with_measured_margin_overlap', 0)}) "
        f"| {final.get('moving_collision_frames', 0)} / {final.get('moving_outside_road_frames', 0)} "
        f"| {times[1]:.1f} / {times[2]:.1f} | {row['max_output_gap']:.3f} "
        f"| {'PASS' if not errors else ', '.join(errors)} |")
    zeros = [f for f in metrics["frames"] if f["objects"] and abs(f["target_v"]) < .01]
    intervals = []
    for frame in zeros:
        if not intervals or frame["t"] - intervals[-1][-1] > .21:
            intervals.append([frame["t"], frame["t"]])
        else:
            intervals[-1][-1] = frame["t"]
    warnings = [{"t": round(t - replay["bag_start"], 3), "message": message}
                for t, level, message in replay["logs"]
                if t >= replay["bag_start"] and level >= 4 and "[MPT]" in message]
    other_warnings = Counter(message for t, level, message in replay["logs"]
                             if t >= replay["bag_start"] and level >= 4 and "[MPT]" not in message)
    invalid = [{"t": round(v["stamp"] - replay["bag_start"], 3),
                "fields": [k for k, value in v.items() if k.startswith("is_valid") and not value],
                "max_lateral_acc": v.get("max_lateral_acc"),
                "min_longitudinal_acc": v.get("min_longitudinal_acc"),
                "max_lateral_jerk": v.get("max_lateral_jerk")}
               for v in replay["validation"] if v["stamp"] >= replay["bag_start"]]
    invalid = [v for v in invalid if v["fields"]]
    latency = [f["latency"] for f in metrics["frames"]]
    snapshots = {int(s["publication_index"]): s for s in replay.get("scene_checks", [])}
    scene_timing = []
    for response in stage.get("new_scene_responses", []):
        snapshot = snapshots.get(response["sequence"])
        if snapshot:
            scene_timing.append({
                **response,
                "checked_scene_elapsed": int(snapshot["scene_stamp_ns"]) / 1e9 - replay["bag_start"],
                # C++ steady_clock and Python monotonic use CLOCK_MONOTONIC on this Linux host.
                "validation_relative_to_observer_scene_ms":
                    (int(snapshot["validated_wall_ns"]) / 1e9 - response["scene_wall"]) * 1000,
            })
    summary[name] = {
        "summary": row, "stages": stage, "po_stop_intervals": intervals,
        "po_stop_frames": zeros, "warnings": warnings, "invalid_frames": invalid,
        "other_warning_counts": dict(other_warnings),
        "latency_seconds_p50_p95_max": np.percentile(latency, [50, 95, 100]).tolist(),
        "max_retained_objects": max(len(s["obstacles"]) for s in replay["scenes"]),
        "new_scene_timing_evidence": scene_timing,
    }
    details.extend(["", f"## {name[-6:]}", "",
                    f"- PO 정지 구간(백 시작 후 초): `{intervals}`",
                    f"- 검증기 실패 필드: `{stage['validator_invalid_fields']}`",
                    f"- 출력 간격 시계: `{row.get('output_gap_clock', 'legacy input header')}`; "
                    f"입력 header 최대 간격: `{row.get('max_input_stamp_gap', row['max_output_gap'])}`초",
                    f"- 기록기 PO 수신 최대 간격: {row.get('max_recorder_output_gap', row['max_output_gap']):.6f}초; "
                    f"PO 발행 기록 연결 누락: {row.get('missing_publication_evidence', 'not recorded')}",
                    f"- 출력/장면 검사: `{stage['coverage']}`", "",
                    f"- 진행 방향과 반대인 방향각: PO "
                    f"{stage['PO']['counts'].get('moving_inconsistent_heading_frames', 'not checked')}, "
                    f"최종 {final.get('moving_inconsistent_heading_frames', 'not checked')}프레임", "",
                    f"- PO 초기 자세·근거리 곡률 실패: {len(stage.get('PO_initial_and_near_curvature_failures', []))}프레임; "
                    f"검사 조건: `{stage.get('PO_near_curvature_check', 'not checked')}`", "",
                    "주요 PO 상태 전이 (로그는 throttle되어 있어 실패 횟수와 같지 않음):", ""])
    if "final_against_latest_observer_scene" in stage:
        observer = stage["final_against_latest_observer_scene"]["counts"]
        responses = stage.get("new_scene_responses", [])
        details[-2:-2] = [
            f"- 기록기 최신 장면 기준 장애물/도로 침범: "
            f"{observer.get('moving_collision_frames', 0)}/{observer.get('moving_outside_road_frames', 0)}프레임",
            f"- 새 장면 수신→안전 출력 수신 지연(ms): "
            f"{[r['safe_output_delay_ms'] for r in responses]}",
            f"- 출력 검증 기록 누락: {final.get('missing_validation_snapshot', 0)}, "
            f"검증 장면 누락: {final.get('missing_snapshot_scene', 0)}",
            f"- 위 차이 프레임의 검증 완료−기록기 새 장면 수신(ms, 음수면 검증이 먼저): "
            f"{[s['validation_relative_to_observer_scene_ms'] for s in scene_timing]}",
            "",
        ]
    for w in warnings:
        details.append(f"- {w['t']:.3f}s: {w['message']}")
    if invalid:
        values = [(v["min_longitudinal_acc"], v["t"]) for v in invalid
                  if v["min_longitudinal_acc"] is not None]
        if values:
            acceleration, elapsed = min(values)
            details.append(f"\n- 검증 경고 프레임의 최소 종가속: {acceleration:.3f} m/s² (+{elapsed:.3f}s).")
    if other_warnings:
        details.append("\n기타 경고 상위 8개 (기록 횟수):\n")
        for message, count in other_warnings.most_common(8):
            details.append(f"- {count}: {message}")
report.extend(["", "자동 검사는 출력 연속성·장면 동기화·노드 종료·형상 안전 검사다. "
               "가속도·횡저크 경고와 정지 여부는 아래에 별도로 남긴다."])
(args.run / "regression_summary.json").write_text(json.dumps(summary, indent=2))
(args.run / "report.md").write_text("\n".join(report + details) + "\n")
print(args.run / "report.md")
