#!/usr/bin/env python3
"""자연 신호 상태 읽기 전용 관측. Set·GUI·ROS·Ego 제어는 하지 않는다.

기존 modern Infrastructure.GetTrafficLightInfo의 정확한 ID/색 검증을 재사용한다.
시작/끝 현시는 잘림으로 표시하고 긴 수신 공백을 연결하지 않는다. 동일 상태
진입 경계 세 개와 동일한 두 전체 상태열이 있어야만 주기 추정치를 제시한다.
130초 관측만으로 120초 주기 두 번을 확인했다고 주장하지 않는다.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import probe_morai_traffic_light_gt as native


DEFAULT_IDS = ("C1256W000077", "C1256W000091", "C1256W000016", "C1256W000052")
READ_PATH = "/" + native.MODERN_SERVICE + "/GetTrafficLightInfo"


def validate_options(seconds, hz, ids, timeout):
    if not math.isfinite(seconds) or not 0 < seconds <= 3600:
        raise ValueError("관측 시간은 0초 초과 3600초 이하여야 합니다")
    if not math.isfinite(hz) or not .1 <= hz <= 20:
        raise ValueError("관측 빈도는 0.1~20 Hz여야 합니다")
    if not ids or len(ids) > 16 or len(set(ids)) != len(ids):
        raise ValueError("서로 다른 native ID를 1~16개 지정해야 합니다")
    for unique_id in ids:
        native.encode_modern_target(unique_id)
    timeout = native.validate_timeout(timeout)
    return timeout


def observe(*, rpc_call, seconds=130.0, hz=5.0, ids=DEFAULT_IDS, timeout=.5,
            monotonic=time.monotonic, sleep=time.sleep, emit=None,
            utc_now=lambda: datetime.now(timezone.utc).isoformat()):
    """주입 가능한 조회 전송/시계. 읽기 실패당 재시도는 최대 한 번이다."""
    timeout = validate_options(seconds, hz, ids, timeout)
    rows = []
    start = monotonic()
    deadline, scan = start + seconds, 0
    while monotonic() < deadline:
        scheduled = start + scan / hz
        if scheduled > monotonic():
            sleep(min(scheduled - monotonic(), max(0, deadline - monotonic())))
        if monotonic() >= deadline:
            break
        for unique_id in ids:
            for attempt in range(2):
                remaining = deadline - monotonic()
                if remaining <= 0:
                    break
                row = {"scan_index": scan, "requested_id": unique_id, "attempt": attempt + 1,
                       "scheduled_monotonic": scheduled, "request_monotonic": monotonic(),
                       "request_utc": utc_now(), "response_hex": None,
                       "readback_success": False, "event": "incomplete"}
                def call(path, request):
                    if path != READ_PATH:
                        raise ValueError("허용되지 않은 RPC 경로입니다")
                    row["rpc_path"], row["request_hex"] = path, request.hex()
                    response = rpc_call(path, request, min(timeout, max(.001, deadline - monotonic())))
                    row["response_hex"] = response.hex()
                    return response
                try:
                    result = native.query_states([("traffic_light", unique_id)], call, protocol="modern")[0]
                    row["readback"] = result
                    row["readback_success"] = result["readback_success"]
                    row["event"] = "observation" if result["readback_success"] else "invalid_readback"
                    if result["readback_success"]:
                        state = result["observations"][0]
                        row.update(native_id=state["tl_id"], color=state["tl_color"],
                                   color_name=state["tl_color_name"])
                except KeyboardInterrupt:
                    row.update(event="interrupted", error="사용자 중단: 미완료 조회는 상태로 사용하지 않음")
                    raise
                except Exception as error:
                    row.update(event="error", error=str(error))
                    if hasattr(error, "code") and callable(error.code):
                        row["grpc_status"] = str(error.code())
                finally:
                    row.update(response_monotonic=monotonic(), response_utc=utc_now())
                    row["read_duration_sec"] = row["response_monotonic"] - row["request_monotonic"]
                    row["schedule_lateness_sec"] = row["request_monotonic"] - scheduled
                    rows.append(row)
                    if emit is not None:
                        emit(row)
                if row["readback_success"]:
                    break
        # 밀린 tick을 몰아서 조회하지 않고 다음 미래 tick으로 진행한다.
        scan = max(scan + 1, math.floor((monotonic() - start) * hz) + 1)
    return rows, {"start_monotonic": start, "end_monotonic": monotonic(),
                  "requested_seconds": seconds, "requested_hz": hz,
                  "max_attempts_per_id_per_scan": 2}


def _statistics(values):
    return {"count": len(values), "min": min(values) if values else None,
            "max": max(values) if values else None,
            "mean": sum(values) / len(values) if values else None}


def _boundary(before, after):
    return {"after_monotonic": before, "by_monotonic": after,
            "midpoint_monotonic": (before + after) / 2}


def timeline(samples, max_gap_sec):
    """이전 요청 시작~다음 응답 수신으로 전이 구간을 보수적으로 넓힌다.

    서버가 요청 처리 중 상태를 읽는다는 가정의 API 상태 추정이며, 렌더 전이
    stamp나 실제 램프 전이의 독립 보증이 아니다.
    """
    phases, gaps, current, chain = [], [], None, 0
    for row in samples:
        stamp = row["response_monotonic"]
        if not math.isfinite(stamp):
            raise ValueError("관측 시각이 유한하지 않습니다")
        if current is not None and stamp <= current["last_seen_monotonic"]:
            raise ValueError("ID별 성공 관측 시각은 엄격하게 증가해야 합니다")
        long_gap = current is not None and stamp - current["last_seen_monotonic"] > max_gap_sec
        if long_gap:
            gaps.append({"after_monotonic": current["last_seen_monotonic"], "by_monotonic": stamp,
                         "elapsed_sec": stamp - current["last_seen_monotonic"],
                         "reason": "수신 공백 임계 초과: 현시 지속/주기를 연결하지 않음"})
            current["right_censored"] = True
            current = None
            chain += 1
        if current is None or row["color"] != current["color"]:
            boundary = None if current is None else _boundary(current["last_request_monotonic"], stamp)
            if current is not None:
                current["end_boundary"] = boundary
                current["right_censored"] = False
            current = {"chain": chain, "color": row["color"], "color_name": row["color_name"],
                       "first_seen_monotonic": stamp, "last_seen_monotonic": stamp,
                       "last_request_monotonic": row.get("request_monotonic", stamp),
                       "left_censored": boundary is None, "right_censored": True,
                       "start_boundary": boundary, "end_boundary": None, "samples": 0}
            phases.append(current)
        current["last_seen_monotonic"] = stamp
        current["last_request_monotonic"] = row.get("request_monotonic", stamp)
        current["samples"] += 1
    for phase in phases:
        phase["duration_basis"] = "이전 요청 시작~다음 응답 수신으로 넓힌 API 전이 추정; 렌더 전이 보증 아님"
        phase["observed_span_sec"] = phase["last_seen_monotonic"] - phase["first_seen_monotonic"]
        phase["duration_estimate_sec"] = None
        phase["complete_duration_bounds_sec"] = None
        if not phase["left_censored"] and not phase["right_censored"]:
            start, end = phase["start_boundary"], phase["end_boundary"]
            phase["duration_estimate_sec"] = end["midpoint_monotonic"] - start["midpoint_monotonic"]
            phase["complete_duration_bounds_sec"] = [
                max(0.0, end["after_monotonic"] - start["by_monotonic"]),
                end["by_monotonic"] - start["after_monotonic"]]
    return phases, gaps


def cycle_estimates(phases):
    """세 동일 상태 진입 경계 사이의 두 완전 반복만 보고한다."""
    estimates = []
    for chain in sorted({phase["chain"] for phase in phases}):
        block = [phase for phase in phases if phase["chain"] == chain]
        for color in sorted({phase["color"] for phase in block}):
            entries = [i for i, phase in enumerate(block)
                       if phase["color"] == color and phase["start_boundary"] is not None]
            for a, b, c in zip(entries, entries[1:], entries[2:]):
                first = [phase["color"] for phase in block[a:b]]
                second = [phase["color"] for phase in block[b:c]]
                if first != second or len(set(first)) < 2:
                    continue
                if any(phase["left_censored"] or phase["right_censored"] for phase in block[a:c]):
                    continue
                boundaries = [block[i]["start_boundary"] for i in (a, b, c)]
                periods, bounds = [], []
                for left, right in zip(boundaries, boundaries[1:]):
                    periods.append(right["midpoint_monotonic"] - left["midpoint_monotonic"])
                    bounds.append([max(0.0, right["after_monotonic"] - left["by_monotonic"]),
                                   right["by_monotonic"] - left["after_monotonic"]])
                estimates.append({"chain": chain, "anchor_color": color,
                                  "ordered_color_sequence": first, "complete_repetitions": 2,
                                  "entry_boundaries": boundaries, "period_estimates_sec": periods,
                                  "period_bounds_sec": bounds, "mean_period_sec": sum(periods) / 2,
                                  "timing_basis": "클라이언트 요청/응답 구간 기반 API 추정; 서버/렌더 timestamp 아님"})
    return estimates


def summarize(rows, ids=DEFAULT_IDS, hz=5.0, max_gap_sec=.6):
    if not math.isfinite(max_gap_sec) or max_gap_sec <= 0:
        raise ValueError("공백 임계는 양수여야 합니다")
    result = {"per_id": {}, "assumed_common_period_sec": None,
              "timestamp_semantics": "클라이언트 UTC/monotonic 수신 시각; 시뮬레이터 렌더/현시 전이 stamp 아님",
              "limitation": "표본 사이의 숨은 전이·렌더 상태는 인증하지 않음. 주기 추정은 관측한 두 반복에 한정함."}
    scans = {}
    for row in rows:
        if row.get("readback_success"):
            scans.setdefault(row["scan_index"], {})[row["requested_id"]] = row["response_monotonic"]
    full_scans = [list(scan.values()) for scan in scans.values() if set(scan) == set(ids)]
    result["inter_id_response_skew_sec"] = _statistics([max(scan) - min(scan) for scan in full_scans])
    for unique_id in ids:
        own = [row for row in rows if row["requested_id"] == unique_id]
        good = [row for row in own if row.get("readback_success") and row.get("event") == "observation"
                and row.get("native_id") == unique_id and row.get("color") in native.COLORS and row["color"] > 0]
        phases, gaps = timeline(good, max_gap_sec)
        intervals = [b["response_monotonic"] - a["response_monotonic"] for a, b in zip(good, good[1:])]
        estimates = cycle_estimates(phases)
        result["per_id"][unique_id] = {
            "successful_samples": len(good), "attempt_event_counts": dict(Counter(row["event"] for row in own)),
            "sample_color_frequency_counts": dict(Counter(str(row["color"]) for row in good)),
            "phase_occurrence_counts": dict(Counter(str(phase["color"]) for phase in phases)),
            "response_interval_sec": _statistics(intervals),
            "interval_jitter_from_requested_period_sec": _statistics([value - 1 / hz for value in intervals]),
            "read_duration_sec": _statistics([row["read_duration_sec"] for row in own]),
            "schedule_lateness_sec": _statistics([row["schedule_lateness_sec"] for row in own]),
            "max_gap_sec": max_gap_sec, "large_gaps": gaps, "phases": phases,
            "observed_order_by_continuity_chain": [
                [phase["color"] for phase in phases if phase["chain"] == chain]
                for chain in sorted({phase["chain"] for phase in phases})],
            "cycle_evidence": "두 완전 반복 확인" if estimates else "두 완전 반복 증거 부족",
            "cycle_estimates": estimates,
        }
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=130, help="관측 시간, 기본 130초")
    parser.add_argument("--hz", type=float, default=5, help="ID별 목표 조회 빈도, 기본 5Hz")
    parser.add_argument("--ids", nargs="+", default=list(DEFAULT_IDS), help="native 신호 ID 목록")
    parser.add_argument("--output", type=Path, required=True, help="존재하지 않는 새 출력 폴더")
    parser.add_argument("--endpoint", default="127.0.0.1:7789", help="숫자 IPv4 loopback만 허용")
    parser.add_argument("--rpc-timeout", type=float, default=.5, help="조회당 제한 시간, 기본 0.5초")
    parser.add_argument("--max-gap-sec", type=float, help="지속/주기를 끊는 공백, 기본 목표 주기의 3배")
    args = parser.parse_args(argv)
    try:
        validate_options(args.seconds, args.hz, args.ids, args.rpc_timeout)
        native.validate_endpoint(args.endpoint)
        gap = args.max_gap_sec if args.max_gap_sec is not None else 3 / args.hz
        if not math.isfinite(gap) or gap <= 0:
            raise ValueError("공백 임계는 양수여야 합니다")
        if args.output.exists() or args.output.is_symlink():
            raise ValueError("출력 폴더는 새 경로여야 합니다")
    except ValueError as error:
        parser.error(str(error))
    import grpc  # 실제 관측에서만 import. Connect/Start/Set은 호출하지 않는다.
    args.output.mkdir(parents=True, exist_ok=False)
    capture_started = time.monotonic()
    rows, run, interrupted = [], {"requested_seconds": args.seconds, "requested_hz": args.hz,
                                 "start_monotonic": capture_started}, False
    with (args.output / "observations.jsonl").open("x") as stream:
        def emit(row):
            rows.append(row)
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
        with grpc.insecure_channel(args.endpoint, options=[("grpc.max_receive_message_length", native.MAX_RESPONSE_BYTES)]) as channel:
            def rpc_call(path, request, timeout):
                if path != READ_PATH:
                    raise ValueError("읽기 전용 조회 외 경로는 허용하지 않습니다")
                return channel.unary_unary(path, request_serializer=lambda value: value,
                                           response_deserializer=lambda value: value)(request, timeout=timeout)
            try:
                _, run = observe(rpc_call=rpc_call, seconds=args.seconds, hz=args.hz,
                                 ids=args.ids, timeout=args.rpc_timeout, emit=emit)
            except KeyboardInterrupt:
                interrupted = True
    run.setdefault("end_monotonic", time.monotonic())
    run["actual_duration_sec"] = run["end_monotonic"] - run["start_monotonic"]
    summary = summarize(rows, args.ids, args.hz, gap)
    all_observed = all(summary["per_id"][unique_id]["successful_samples"] > 0 for unique_id in args.ids)
    summary.update(read_only=True, interrupted=interrupted, run=run, endpoint=args.endpoint,
                   all_requested_ids_observed=all_observed,
                   ids=args.ids, source_provenance=native.source_provenance("modern"),
                   script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   protocol_module_sha256=hashlib.sha256(Path(native.__file__).read_bytes()).hexdigest(),
                   raw_jsonl_sha256=hashlib.sha256((args.output / "observations.jsonl").read_bytes()).hexdigest())
    with (args.output / "summary.json").open("x") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"출력": str(args.output), "읽기전용": True, "관측시도": len(rows),
                      "중단": interrupted, "모든신호120초로가정": False}, ensure_ascii=False))
    return 130 if interrupted else (0 if all_observed else 2)


if __name__ == "__main__":
    sys.exit(main())
