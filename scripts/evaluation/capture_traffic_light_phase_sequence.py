#!/usr/bin/env python3
"""Subscriber-only stationary JPEG capture with explicit signal UDP requests.

No vehicle controls/publishers/services, camera/weather edits, or UI actions.
An existing independent Park/brake harness must remain responsible for Ego.
Signal requests are NOT ground truth: UDP delivery/application is unconfirmed.
Optional modern native API reads can verify the requested ID/color around each
fresh JPEG; native API verification still does not certify visible-head/render
correspondence, which requires visual review.
The requested native ID always receives a final default (-1) reset attempt.
"""

import argparse
import ctypes
import hashlib
import importlib
import json
import math
from pathlib import Path
import re
import signal
import socket
import sys
import threading
import time

from sample_traffic_light_bag_frames import compressed_jpeg, sha256_file, stamp_fields
from probe_morai_traffic_light_gt import (
    COLORS, MAX_RESPONSE_BYTES, MODERN_SERVICE, query_states, source_provenance,
    validate_endpoint, validate_timeout,
)


ROOT = Path(__file__).resolve().parents[2]
OFFICIAL_ROOT = ROOT / "third_party/examples/MORAI-NetworkModule"
OFFICIAL_SOURCE = OFFICIAL_ROOT / "lib/define/SetTrafficLightCtrl.py"
ALLOWED_STATES = {-1, 1, 4, 16, 32, 33, 48, 5}
TOPICS = {"ego": "/Ego_topic", "gear": "/vehicle/status/gear",
          "control_mode": "/vehicle/status/control_mode",
          "camera": "/sensor/camera/front/compressed"}


class NotReady(RuntimeError):
    pass


class UnsafeState(RuntimeError):
    pass


class NativeGTError(RuntimeError):
    pass


def validate_rpc_retries(value):
    if type(value) is not int or not 0 <= value <= 2:
        raise ValueError("native_rpc_retries must be an integer in 0..2")
    return value


def native_rpc_error_status(error, rpc_error_type):
    """Only actual transport errors in the explicitly allowed set may retry."""
    if not isinstance(error, rpc_error_type):
        return "NOT_GRPC_RPC_ERROR", str(error), False
    try:
        status = getattr(error.code(), "name", None)
        details = error.details()
    except Exception:
        return "UNREADABLE_GRPC_STATUS", str(error), False
    allowed = status in ("UNAVAILABLE", "DEADLINE_EXCEEDED") or (
        status == "UNKNOWN" and details == "Exception was thrown by handler.")
    return status or "UNREADABLE_GRPC_STATUS", details, allowed


def read_native_rpc_with_retries(call, *, retries=0, rpc_error_type=(), per_call_timeout=1.,
                                 deadline=None, attempts=None, monotonic=time.monotonic, sleep=time.sleep):
    """Injectable byte transport only; no parsing, native ID approval, or caching.

    Every retry makes a new read. Parsing and strict identity/state validation
    happen outside this helper and cannot trigger transport retries. A provided
    absolute monotonic deadline caps both per-RPC timeout and the 50ms backoff.
    The caller-owned attempts list retains failures even when this raises.
    """
    validate_rpc_retries(retries)
    per_call_timeout = validate_timeout(per_call_timeout)
    if deadline is not None and not math.isfinite(deadline):
        raise ValueError("RPC deadline must be finite")
    attempts = [] if attempts is None else attempts
    for attempt_index in range(retries + 1):
        started_at = monotonic()
        remaining = None if deadline is None else deadline - started_at
        row = {"attempt_index":attempt_index, "started_monotonic":started_at,
               "completed_monotonic":None, "status":"started", "will_retry":False}
        attempts.append(row)
        if remaining is not None and remaining <= 0:
            row.update(status="GLOBAL_DEADLINE_EXHAUSTED", completed_monotonic=monotonic())
            raise TimeoutError("Native RPC capture deadline exhausted")
        timeout = per_call_timeout if remaining is None else min(per_call_timeout, remaining)
        row["per_call_timeout_sec"] = timeout
        try:
            response = call(timeout)
        except Exception as error:
            status, details, retryable = native_rpc_error_status(error, rpc_error_type)
            row.update(status=status, details=details, error="{}: {}".format(type(error).__name__, error),
                       completed_monotonic=monotonic(), retryable_transport_error=retryable)
            if not retryable or attempt_index >= retries:
                raise
            remaining = None if deadline is None else deadline - monotonic()
            if remaining is not None and remaining <= .05:
                row["retry_prevented_by_global_deadline"] = True
                raise
            row.update(will_retry=True, backoff_sec=.05)
            sleep(.05)
        else:
            row.update(status="OK", completed_monotonic=monotonic(),
                       response_bytes=len(response), response_sha256=hashlib.sha256(response).hexdigest())
            if deadline is not None and row["completed_monotonic"] >= deadline:
                row["status"] = "GLOBAL_DEADLINE_EXHAUSTED_AFTER_RESPONSE"
                raise TimeoutError("Native RPC response arrived after capture deadline")
            return response


class ModernNativeReader:
    """One fixed, read-only Infrastructure RPC on numeric loopback only.

    grpc is imported only for explicit --verify-native-gt execution. Signal
    writes remain exclusively the official UDP sender above, never a Set RPC.
    """
    def __init__(self, endpoint="127.0.0.1:7789", timeout_sec=1.0, rpc_retries=0,
                 monotonic=time.monotonic, sleep=time.sleep):
        endpoint, self.timeout_sec = validate_endpoint(endpoint), validate_timeout(timeout_sec)
        self.rpc_retries = validate_rpc_retries(rpc_retries)
        self.monotonic, self.sleep, self.last_rpc_attempts = monotonic, sleep, []
        import grpc

        self.rpc_error_type = grpc.RpcError
        self.path = "/" + MODERN_SERVICE + "/GetTrafficLightInfo"
        self.channel = grpc.insecure_channel(endpoint, options=[
            ("grpc.max_receive_message_length", MAX_RESPONSE_BYTES)])
        self.read_call = self.channel.unary_unary(
            self.path, request_serializer=lambda value: value, response_deserializer=lambda value: value)

    def __call__(self, native_id, *, deadline=None):
        self.last_rpc_attempts = []
        validate_native_id(native_id)
        def read(path, request):
            if path != self.path:
                raise ValueError("Only modern GetTrafficLightInfo is permitted")
            return read_native_rpc_with_retries(
                lambda timeout: self.read_call(request, timeout=timeout), retries=self.rpc_retries,
                rpc_error_type=self.rpc_error_type, per_call_timeout=self.timeout_sec,
                deadline=deadline, attempts=self.last_rpc_attempts,
                monotonic=self.monotonic, sleep=self.sleep)
        result = query_states([("traffic_light", native_id)], read, protocol="modern")[0]
        result["rpc_attempts"] = list(self.last_rpc_attempts)
        return result

    def close(self):
        self.channel.close()


def native_readback_issue(readback, native_id, expected_state=None):
    """Return a strict rejection reason, or None; never trust success alone."""
    if not isinstance(readback, dict) or readback.get("protocol") != "modern":
        return "Modern protocol response required"
    if readback.get("method") != "GetTrafficLightInfo" or readback.get("requested_id") != native_id:
        return "Native read method/request identity mismatch"
    observations = readback.get("observations")
    if not isinstance(observations, list) or len(observations) != 1 or not isinstance(observations[0], dict):
        return "Exactly one native observation required"
    observation = observations[0]
    if observation.get("tl_id") != native_id or observation.get("identity_matches_request") is not True:
        return "Native response identity mismatch"
    color = observation.get("tl_color")
    if (isinstance(color, bool) or not isinstance(color, int) or color <= 0 or color not in COLORS
            or observation.get("color_field_present") is not True
            or observation.get("color_enum_known") is not True
            or observation.get("color_usable_for_gt") is not True
            or readback.get("readback_success") is not True):
        return "Native response lacks an explicit known positive color"
    if expected_state is not None and color != expected_state:
        return "Native color {} does not match requested {}".format(color, expected_state)
    return None


def parse_states(value):
    try:
        states = [int(item.strip()) for item in value.split(",")]
    except (ValueError, AttributeError) as error:
        raise ValueError("states must be comma-separated integer codes") from error
    if not states or len(states) > 8 or len(set(states)) != len(states) or not set(states) <= ALLOWED_STATES:
        raise ValueError("Unique states must belong to {}".format(sorted(ALLOWED_STATES)))
    return states


def validate_native_id(native_id):
    if not isinstance(native_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,12}", native_id):
        raise ValueError("native-id must contain 1..12 ASCII letters/digits/_/-")


def official_packet(native_id, state):
    validate_native_id(native_id)
    if isinstance(state, bool) or state not in ALLOWED_STATES:
        raise ValueError("Unsupported traffic-light state")
    source_root = str(OFFICIAL_ROOT)
    if source_root not in sys.path:
        sys.path.insert(0, source_root)
    previous_bytecode = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        module = importlib.import_module("lib.define.SetTrafficLightCtrl")
    finally:
        sys.dont_write_bytecode = previous_bytecode
    if Path(module.__file__).resolve() != OFFICIAL_SOURCE.resolve():
        raise RuntimeError("Unexpected SetTrafficLightCtrl import origin")
    packet = module.SetTrafficLightCtrl()
    packet.trafficLightIndex = native_id.ljust(12).encode("ascii")
    packet.trafficLightStatus = state
    return ctypes.string_at(ctypes.addressof(packet), ctypes.sizeof(packet))


class OfficialSender:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(.5)

    def __call__(self, native_id, state):
        packet = official_packet(native_id, state)
        count = self.socket.sendto(packet, ("127.0.0.1", 7607))
        if count != len(packet):
            raise RuntimeError("Incomplete UDP send")
        return {"udp_send_success": True, "bytes_sent": count,
                "packet_sha256": hashlib.sha256(packet).hexdigest(),
                "destination": ["127.0.0.1", 7607], "simulator_applied_confirmed": False}

    def close(self):
        self.socket.close()


class LiveObservations:
    def __init__(self, monotonic=time.monotonic, time_ns=time.time_ns):
        self.monotonic, self.time_ns = monotonic, time_ns
        self.lock, self.latest = threading.Lock(), {}

    def update(self, name, message, receipt_ros_stamp=None):
        record = {"message": message, "receipt_monotonic": self.monotonic(),
                  "receipt_unix_ns": self.time_ns(),
                  "receipt_ros_stamp": None if receipt_ros_stamp is None else stamp_fields(receipt_ros_stamp)}
        with self.lock:
            self.latest[name] = record

    def snapshot(self, max_age=.75, max_speed=.05):
        now = self.monotonic()
        with self.lock:
            records = dict(self.latest)
        for name in ("ego", "gear", "control_mode"):
            if name not in records or not 0 <= now - records[name]["receipt_monotonic"] <= max_age:
                raise NotReady("Missing/stale {} observation".format(name))
        gear, mode = records["gear"]["message"].data, records["control_mode"]["message"].data
        if isinstance(gear, bool) or isinstance(mode, bool) or gear != 1 or mode != 2:
            raise UnsafeState("Require actual gear=1 (P) and control_mode=2, got {},{}".format(gear, mode))
        ego = records["ego"]["message"]
        velocity = [float(getattr(ego.velocity, axis)) for axis in "xyz"]
        position = [float(getattr(ego.position, axis)) for axis in "xyz"]
        heading = float(ego.heading)
        if not all(math.isfinite(value) for value in velocity + position + [heading]):
            raise UnsafeState("Non-finite Ego telemetry")
        speed = math.sqrt(sum(value * value for value in velocity))
        if speed > max_speed:
            raise UnsafeState("Actual speed {:.6f} exceeds {:.6f} m/s".format(speed, max_speed))
        telemetry = {"position_xyz_m": position, "heading_degrees": heading,
                     "velocity_xyz_mps": velocity, "speed_mps": speed, "gear": gear, "control_mode": mode,
                     "ego_header_stamp": stamp_fields(ego.header.stamp),
                     "receipt_evidence": {name: {key: value for key, value in records[name].items() if key != "message"}
                                          for name in ("ego", "gear", "control_mode")}}
        return telemetry, records.get("camera")


def capture_sequence(monitor, send_request, output, native_id, states=(1, 4, 16), *,
                     samples_per_state=3, settle_sec=.5, spacing_sec=.5,
                     freshness_sec=.75, timeout_sec=45., weather_annotation="UNKNOWN",
                     time_annotation="UNKNOWN", monotonic=time.monotonic, sleep=time.sleep,
                     is_shutdown=lambda: False, read_native_state=None, native_mismatch_retries=0):
    """Bounded injectable core. Only send_request mutates external signal state.

    read_native_state(native_id) optionally returns one modern query_states row.
    The default None preserves the request-only capture path. Verified phases
    must request positive explicit states, not the dynamic schedule code -1.
    Optional retries preserve rejected raw frames; frame_count always counts all
    saved frames, while accepted_frame_count alone counts completed sample quota.
    """
    validate_native_id(native_id)
    states = parse_states(",".join(str(value) for value in states))
    if read_native_state is not None and (not callable(read_native_state) or -1 in states):
        raise ValueError("Native verification requires a callable and explicit positive phase states")
    if (isinstance(native_mismatch_retries, bool) or not isinstance(native_mismatch_retries, int)
            or not 0 <= native_mismatch_retries <= 2):
        raise ValueError("native_mismatch_retries must be an integer in 0..2")
    if native_mismatch_retries and read_native_state is None:
        raise ValueError("Native mismatch retries require native verification")
    if isinstance(samples_per_state, bool) or not isinstance(samples_per_state, int) or not 1 <= samples_per_state <= 10:
        raise ValueError("samples_per_state must be in 1..10")
    for name, value in (("settle_sec", settle_sec), ("spacing_sec", spacing_sec),
                        ("freshness_sec", freshness_sec), ("timeout_sec", timeout_sec)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("{} must be finite and positive".format(name))
    if not all(isinstance(value, str) and value.strip() for value in (weather_annotation, time_annotation)):
        raise ValueError("Weather/time annotations must be explicit nonempty text or UNKNOWN")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "images").mkdir()
    started, previous_header, previous_receipt = monotonic(), None, None
    deadline = started + timeout_sec
    report = {"schema_version": 1, "status": "running", "frame_count": 0, "frames": [],
              "source": {"kind": "live_ROS_subscriber_only", "topics": TOPICS,
                         "camera_settings_modified": False, "vehicle_controls_sent": False,
                         "official_set_source": str(OFFICIAL_SOURCE), "official_set_source_sha256": sha256_file(OFFICIAL_SOURCE),
                         "official_support_sha256": {name: sha256_file(OFFICIAL_ROOT / "lib/define" / name)
                                                     for name in ("base.py", "type.py")},
                         "capture_source_sha256": sha256_file(__file__)},
              "annotations": {"weather": weather_annotation, "time_of_day": time_annotation,
                              "status": "caller_annotation_not_independently_verified"},
              "requested_native_id": native_id, "requested_states": states,
              "requested_states_are_ground_truth": False, "requests": [], "reset": None,
              "native_GT_verified": False, "accepted_frames_native_GT_verified":False,
              "native_GT_verified_scope":"all_saved_frames_including_rejected_attempts",
              "frame_count_semantics":"len(frames); accepted_frame_count alone satisfies sample quota",
              "native_reads": [], "native_mismatches":[],
              "native_gt_verification": {
                  "enabled": read_native_state is not None,
                  "scope": "native_API_ID_color_brackets_not_render_or_visible_head_identity",
                  "render_visual_review_required": True,
                  "read_source_sha256": sha256_file(Path(__file__).with_name("probe_morai_traffic_light_gt.py")),
                  "source_provenance": source_provenance("modern") if read_native_state is not None else None,
                  "phase_poll_timeout_sec": 3.0,
                  "native_rpc_retries":read_native_state.rpc_retries if isinstance(read_native_state, ModernNativeReader) else 0},
              "parameters": {"samples_per_state": samples_per_state, "settle_sec": settle_sec,
                             "spacing_sec": spacing_sec, "freshness_sec": freshness_sec,
                             "timeout_sec": timeout_sec, "max_speed_mps": .05,
                             "required_gear": 1, "required_control_mode": 2,
                             "native_mismatch_retries":native_mismatch_retries}}
    error = None

    def check_deadline():
        if is_shutdown():
            raise RuntimeError("ROS shutdown during capture")
        if monotonic() >= deadline:
            raise TimeoutError("Traffic-light capture exceeded total timeout")

    def fresh_snapshot():
        while True:
            check_deadline()
            try:
                return monitor.snapshot(freshness_sec, .05)
            except NotReady:
                sleep(min(.02, max(0, deadline - monotonic())))

    def native_read(stage, expected_state=None, phase_index=None, sample_index=None, attempt_index=None):
        row = {"stage":stage, "requested_native_id":native_id, "expected_state":expected_state,
               "phase_index":phase_index, "sample_index":sample_index, "attempt_index":attempt_index,
               "started_monotonic":monotonic(), "readback":None, "verified":False}
        report["native_reads"].append(row)
        try:
            if isinstance(read_native_state, ModernNativeReader):
                # The normal capture budget remains global. Reset gets at most
                # one second of the existing final-reset/logging reserve, even
                # when retries are enabled; no cache is accepted on exhaustion.
                rpc_deadline = monotonic() + 1.0 if stage == "reset_after_udp" else deadline
                row["readback"] = read_native_state(native_id, deadline=rpc_deadline)
            else:
                row["readback"] = read_native_state(native_id)
            issue = native_readback_issue(row["readback"], native_id, expected_state)
            row["verified"] = issue is None
            if issue is not None:
                row["mismatch"] = issue
        except BaseException as caught:
            row["error"] = "{}: {}".format(type(caught).__name__, caught)
            raise
        finally:
            row["completed_monotonic"] = monotonic()
            if isinstance(read_native_state, ModernNativeReader):
                row["rpc_attempts"] = list(read_native_state.last_rpc_attempts)
        return row

    def request_phase(phase_index, state, sample_index, attempt_index):
        nonlocal previous_header
        _, barrier_camera = fresh_snapshot()
        if barrier_camera is not None:
            barrier = stamp_fields(barrier_camera["message"].header.stamp)
            barrier_ns = barrier["secs"] * 1_000_000_000 + barrier["nsecs"]
            previous_header = max(previous_header or 0, barrier_ns)
        check_deadline()
        event = {"phase_index":phase_index, "sample_index":sample_index, "attempt_index":attempt_index,
                 "state":state, "sent_monotonic":monotonic(), "requested_state_confirmed":False}
        report["requests"].append(event)
        event.update(send_request(native_id, state))
        event["send_completed_monotonic"] = monotonic()
        if read_native_state is not None:
            poll_deadline = min(deadline, monotonic() + 3.0)
            while True:
                check_deadline()
                readback = native_read("phase_after_udp", state, phase_index, sample_index, attempt_index)
                if readback["verified"]:
                    event["native_readback"] = readback
                    event["requested_state_confirmed"] = True
                    event["confirmation_scope"] = "native_API_only_render_review_required"
                    break
                if monotonic() >= poll_deadline:
                    raise NativeGTError("Requested phase native readback failed: " + readback["mismatch"])
                sleep(min(.05, max(0, poll_deadline - monotonic())))
        return event["send_completed_monotonic"] + settle_sec

    def capture_frame(phase_index, state, sample_index, attempt_index, not_before):
        nonlocal previous_header, previous_receipt
        before = None
        if read_native_state is not None:
            check_deadline()
            before = native_read("frame_before", state, phase_index, sample_index, attempt_index)
            if not before["verified"]:
                raise NativeGTError("Pre-frame native mismatch: " + before["mismatch"])
            not_before = max(not_before, before["completed_monotonic"])
        while True:
            telemetry, camera = fresh_snapshot()
            now = monotonic()
            if camera is not None and 0 <= now - camera["receipt_monotonic"] <= freshness_sec:
                receipt = camera["receipt_monotonic"]
                header = stamp_fields(camera["message"].header.stamp)
                header_ns = header["secs"] * 1_000_000_000 + header["nsecs"]
                if header_ns == 0:
                    raise ValueError("Camera header stamp is zero; cannot prove a fresh frame")
                if (receipt >= not_before and (before is None or receipt > before["completed_monotonic"])
                        and (previous_receipt is None or receipt > previous_receipt)
                        and (previous_header is None or header_ns > previous_header)):
                    break
            sleep(min(.02, max(0, deadline - monotonic())))
        payload, width, height = compressed_jpeg(camera["message"])
        suffix = "" if attempt_index == 0 else "_attempt_{:02d}".format(attempt_index)
        relative = "images/phase_{:02d}_state_{}_sample_{:02d}{}.jpg".format(
            phase_index, state, sample_index, suffix)
        with (output / relative).open("xb") as stream:
            stream.write(payload)
        frame = {"image_path":relative, "sha256":hashlib.sha256(payload).hexdigest(),
                 "width":width, "height":height, "message_format":camera["message"].format,
                 "header_stamp":header, "receipt_ros_stamp":camera["receipt_ros_stamp"],
                 "receipt_unix_ns":camera["receipt_unix_ns"], "receipt_monotonic":receipt,
                 "phase_index":phase_index, "sample_index":sample_index, "attempt_index":attempt_index,
                 "requested_state":state, "requested_native_id":native_id,
                 "requested_state_confirmed":False, "accepted_for_sample_quota":False,
                 "native_GT_verified":False, "native_gt":{"before":before, "after":None,
                    "jpeg_receipt_bracketed":False, "render_visual_review_required":True},
                 "human_ground_truth":None, "ego":telemetry}
        # Keep every raw attempt; a rejected attempt must not satisfy quota or
        # be selected again as the replacement's supposedly fresh source frame.
        report["frames"].append(frame)
        previous_header, previous_receipt = header_ns, receipt
        if read_native_state is not None:
            after = native_read("frame_after", state, phase_index, sample_index, attempt_index)
            frame["native_gt"]["after"] = after
            bracketed = before["completed_monotonic"] < receipt <= after["started_monotonic"]
            frame["native_gt"]["jpeg_receipt_bracketed"] = bracketed
            if not bracketed:
                raise RuntimeError("JPEG receipt not bracketed; this is not a retryable native-state mismatch")
            if not after["verified"]:
                raise NativeGTError("Post-frame native verification failed: " + after["mismatch"])
            frame["native_GT_verified"] = True
            frame["requested_state_confirmed"] = True
            frame["native_gt"]["observed_native_id"] = native_id
            frame["native_gt"]["observed_state"] = after["readback"]["observations"][0]["tl_color"]
        frame["accepted_for_sample_quota"] = True
        return receipt + spacing_sec

    try:
        for phase_index, state in enumerate(states):
            not_before = None
            for sample_index in range(samples_per_state):
                for attempt_index in range(native_mismatch_retries + 1):
                    check_deadline()
                    first_frame = len(report["frames"])
                    try:
                        if not_before is None or attempt_index > 0:
                            not_before = request_phase(phase_index, state, sample_index, attempt_index)
                        not_before = capture_frame(phase_index, state, sample_index, attempt_index, not_before)
                        break
                    except NativeGTError as mismatch:
                        report["native_mismatches"].append({
                            "phase_index":phase_index, "sample_index":sample_index,
                            "attempt_index":attempt_index, "error":str(mismatch),
                            "retained_frame_indices":list(range(first_frame, len(report["frames"]))),
                            "will_retry":attempt_index < native_mismatch_retries})
                        if attempt_index >= native_mismatch_retries:
                            raise
        report["status"] = "complete"
        report["native_GT_verified"] = (read_native_state is not None and bool(report["frames"])
                                         and all(frame["native_GT_verified"] for frame in report["frames"]))
    except BaseException as caught:
        error = caught
        report["status"] = "failed"
        report["error"] = "{}: {}".format(type(caught).__name__, caught)
    finally:
        reset = {"requested_native_id": native_id, "requested_state": -1,
                 "attempted": True, "udp_send_success": False, "simulator_applied_confirmed": False,
                 "native_identity_and_known_state_verified":False,
                 "schedule_reset_applied_confirmed":False, "previous_phase_restored_confirmed":False}
        try:
            reset.update(send_request(native_id, -1))
            if read_native_state is not None:
                # -1 resumes a schedule; no positive returned color proves that
                # the schedule reset applied or that a previous phase returned.
                reset_read = native_read("reset_after_udp")
                reset["native_readback"] = reset_read
                reset["native_identity_and_known_state_verified"] = reset_read["verified"]
                if not reset_read["verified"]:
                    raise NativeGTError("Reset identity/known-state readback failed: " + reset_read["mismatch"])
        except BaseException as reset_error:
            reset["error"] = "{}: {}".format(type(reset_error).__name__, reset_error)
            if error is None:
                error = RuntimeError("Final default-reset send/readback failed")
                report["status"] = "failed_reset_readback" if reset["udp_send_success"] else "failed_reset_send"
        reset["simulator_applied_confirmed"] = False
        report["reset"] = reset
        if error is not None:
            report["native_GT_verified"] = False
        report["frame_count"] = len(report["frames"])
        report["raw_frame_count"] = len(report["frames"])
        accepted = [frame for frame in report["frames"] if frame["accepted_for_sample_quota"]]
        report["accepted_frame_count"] = len(accepted)
        report["rejected_frame_count"] = len(report["frames"]) - len(accepted)
        report["accepted_frames_native_GT_verified"] = (
            error is None and read_native_state is not None and len(accepted) == len(states) * samples_per_state
            and all(frame["native_GT_verified"] for frame in accepted))
        report["elapsed_sec"] = monotonic() - started
        with (output / "manifest.json").open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    if error is not None:
        raise error
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-id", required=True)
    parser.add_argument("--states", default="1,4,16", type=parse_states)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--weather-annotation", default="UNKNOWN")
    parser.add_argument("--time-annotation", default="UNKNOWN")
    parser.add_argument("--samples-per-state", type=int, default=3)
    parser.add_argument("--settle-sec", type=float, default=.5)
    parser.add_argument("--spacing-sec", type=float, default=.5)
    parser.add_argument("--freshness-sec", type=float, default=.75)
    parser.add_argument("--timeout-sec", type=float, default=45.)
    parser.add_argument("--verify-native-gt", action="store_true",
                        help="Read modern native ID/color before/after fresh JPEGs; renderer still requires review")
    parser.add_argument("--native-mismatch-retries", type=int, default=0, choices=range(3),
                        help="0..2 retries per sample on native-state mismatch only; requires --verify-native-gt")
    parser.add_argument("--native-rpc-retries", type=int, default=0, choices=range(3),
                        help="0..2 retries only for allowed transient Get RPC errors; requires --verify-native-gt")
    args = vars(parser.parse_args(argv))
    verify_native_gt = args.pop("verify_native_gt")
    native_rpc_retries = args.pop("native_rpc_retries")
    if args["native_mismatch_retries"] and not verify_native_gt:
        parser.error("--native-mismatch-retries requires --verify-native-gt")
    if native_rpc_retries and not verify_native_gt:
        parser.error("--native-rpc-retries requires --verify-native-gt")
    validate_native_id(args["native_id"])
    if args["output"].exists() or args["output"].is_symlink():
        raise FileExistsError("Output must be a fresh directory")
    # A wall-clock alarm also bounds ROS initialization, outside the core's
    # monotonic timeout. Three extra seconds are reserved for final reset/logs.
    def wall_timeout(_signum, _frame):
        raise TimeoutError("Whole capture program wall-clock timeout")
    previous_handler = signal.signal(signal.SIGALRM, wall_timeout)
    signal.setitimer(signal.ITIMER_REAL, args["timeout_sec"] + 3.)
    sender, native_reader, subscribers, rospy = None, None, [], None
    try:
        if verify_native_gt:
            native_reader = ModernNativeReader(rpc_retries=native_rpc_retries)
        import rospy
        from morai_msgs.msg import EgoVehicleStatus
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import Int8
        rospy.init_node("traffic_light_phase_capture", anonymous=True)
        monitor, sender = LiveObservations(), OfficialSender()
        for name, msg_type in (("ego", EgoVehicleStatus), ("gear", Int8),
                               ("control_mode", Int8), ("camera", CompressedImage)):
            def callback(message, key=name):
                monitor.update(key, message, rospy.Time.now())
            subscribers.append(rospy.Subscriber(TOPICS[name], msg_type, callback, queue_size=1))
        output = args.pop("output")
        result = capture_sequence(monitor, sender, output, is_shutdown=rospy.is_shutdown,
                                  read_native_state=native_reader, **args)
        print(json.dumps({"manifest": str((output / "manifest.json").resolve()),
                          "frame_count": result["frame_count"], "reset_confirmed": False,
                          "accepted_frame_count":result["accepted_frame_count"],
                          "accepted_frames_native_GT_verified":result["accepted_frames_native_GT_verified"],
                          "native_GT_verified": result["native_GT_verified"], "render_visual_review_required":True}))
    finally:
        if sender is not None:
            sender.close()
        if native_reader is not None:
            native_reader.close()
        for subscriber in subscribers:
            subscriber.unregister()
        if rospy is not None:
            rospy.signal_shutdown("Owned subscriber-only capture finished")
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous_handler)


if __name__ == "__main__":
    main()
