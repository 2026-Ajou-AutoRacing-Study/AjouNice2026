#!/usr/bin/env python3
"""Fake-clock/ROS-message/sender tests; never send network packets."""

import json
from enum import Enum
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from capture_traffic_light_phase_sequence import (
    LiveObservations, ModernNativeReader, NativeGTError, NotReady, UnsafeState,
    capture_sequence, native_readback_issue, official_packet, parse_states,
    native_rpc_error_status, read_native_rpc_with_retries, validate_rpc_retries,
)
from probe_morai_traffic_light_gt import MODERN_SERVICE, ProtocolError, encode_modern_target, encode_varint, query_states
from test_sample_traffic_light_bag_frames import jpeg_fixture


def stamp(number):
    return NS(secs=100 + int(number), nsecs=int((number % 1) * 1e9))


def modern_row(native_id="C119BS010025", color=1, *, omit_color=False):
    encoded_id = native_id.encode()
    nested = b"\x0a" + encode_varint(len(encoded_id)) + encoded_id
    wire = b"\x12" + encode_varint(len(nested)) + nested
    if not omit_color:
        wire += b"\x18" + encode_varint(color if color >= 0 else 2**64 + color)
    return query_states([("traffic_light", "C119BS010025")], lambda path, request: wire)[0]


class PhaseCaptureTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.output = Path(self.temporary.name) / "capture"
        self.now = 0.
        self.monitor = LiveObservations(lambda: self.now, lambda: int((100 + self.now) * 1e9))
        self.sent = []
        self.refresh()

    def refresh(self, gear=1, mode=2, speed=0.):
        ego = NS(position=NS(x=1., y=2., z=3.), velocity=NS(x=speed, y=0., z=0.),
                 heading=123., header=NS(stamp=stamp(self.now)))
        camera = NS(format="jpeg", data=jpeg_fixture(), header=NS(stamp=stamp(self.now)))
        for name, message in (("ego", ego), ("gear", NS(data=gear)),
                               ("control_mode", NS(data=mode)), ("camera", camera)):
            self.monitor.update(name, message, stamp(self.now))

    def sleep(self, seconds):
        self.now += seconds
        self.refresh()

    def send(self, native, state):
        self.sent.append((native, state))
        return {"udp_send_success": True, "simulator_applied_confirmed": False}

    def native_read(self, native):
        state = self.sent[-1][1]
        return modern_row(native, 16 if state == -1 else state)

    def run_capture(self, **kwargs):
        return capture_sequence(self.monitor, self.send, self.output, "C119BS010025",
                                monotonic=lambda: self.now, sleep=self.sleep, **kwargs)

    def test_original_bytes_fresh_frames_and_unconfirmed_requests(self):
        result = self.run_capture(states=[1, 16], samples_per_state=2)
        self.assertEqual(result["frame_count"], 4)
        self.assertEqual([state for _, state in self.sent], [1, 16, -1])
        self.assertFalse(result["requested_states_are_ground_truth"])
        self.assertFalse(result["native_GT_verified"])
        self.assertEqual(result["native_reads"], [])
        self.assertFalse(result["reset"]["simulator_applied_confirmed"])
        previous_receipt, previous_header = -1., -1
        for frame in result["frames"]:
            self.assertEqual((self.output / frame["image_path"]).read_bytes(), jpeg_fixture())
            self.assertGreater(frame["receipt_monotonic"], previous_receipt)
            header = frame["header_stamp"]["secs"] * 10**9 + frame["header_stamp"]["nsecs"]
            self.assertGreater(header, previous_header)
            self.assertFalse(frame["requested_state_confirmed"])
            self.assertIsNone(frame["human_ground_truth"])
            self.assertEqual(frame["ego"]["gear"], 1)
            self.assertEqual(frame["ego"]["control_mode"], 2)
            previous_receipt, previous_header = frame["receipt_monotonic"], header

    def test_unsafe_vehicle_fails_and_always_resets(self):
        self.refresh(gear=4)
        with self.assertRaises(UnsafeState):
            self.run_capture(states=[1], samples_per_state=1)
        self.assertEqual(self.sent, [("C119BS010025", -1)])
        report = json.loads((self.output / "manifest.json").read_text())
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["reset"]["attempted"])

    def test_stale_data_times_out_and_resets(self):
        def no_updates(seconds):
            self.now += seconds
        self.now = 10.
        with self.assertRaises(TimeoutError):
            capture_sequence(self.monitor, self.send, self.output, "C119BS010025", states=[1],
                             timeout_sec=.2, monotonic=lambda: self.now, sleep=no_updates)
        self.assertEqual(self.sent[-1][1], -1)
        self.assertLess(self.now, 10.3)

    def test_network_failure_still_attempts_reset_and_records_failure(self):
        def broken_send(native, state):
            self.sent.append((native, state))
            raise OSError("fake network failure")
        with self.assertRaises(OSError):
            capture_sequence(self.monitor, broken_send, self.output, "C119BS010025", states=[1],
                             monotonic=lambda: self.now, sleep=self.sleep)
        self.assertEqual([state for _, state in self.sent], [1, -1])
        report = json.loads((self.output / "manifest.json").read_text())
        self.assertFalse(report["reset"]["udp_send_success"])
        self.assertFalse(report["reset"]["simulator_applied_confirmed"])

    def test_stationary_gate_rejects_mode_motion_and_nan(self):
        for kwargs in ({"mode": 1}, {"speed": .051}, {"speed": float("nan")}):
            self.refresh(**kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(UnsafeState):
                self.monitor.snapshot()

    def test_output_collision_sends_nothing(self):
        self.output.mkdir()
        with self.assertRaises(FileExistsError):
            self.run_capture()
        self.assertEqual(self.sent, [])

    def test_allowed_states_are_strict(self):
        self.assertEqual(parse_states("-1,1,4,16,32,33,48,5"), [-1, 1, 4, 16, 32, 33, 48, 5])
        for invalid in ("", "nan", "inf", "20", "1,1", "1.0"):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                parse_states(invalid)

    def test_official_ctypes_packet_bytes_without_network(self):
        packet = official_packet("C119BS010025", -1)
        self.assertEqual(len(packet), 46)
        self.assertEqual(packet[:14], b"#TrafficLight$")
        self.assertEqual(packet[30:42], b"C119BS010025")
        self.assertEqual(packet[42:44], b"\xff\xff")
        self.assertEqual(packet[-2:], b"\r\n")

    def test_verified_native_before_after_brackets_each_original_jpeg(self):
        result = self.run_capture(states=[1, 48], samples_per_state=2, read_native_state=self.native_read)
        self.assertTrue(result["native_GT_verified"])
        self.assertFalse(result["requested_states_are_ground_truth"])
        self.assertTrue(result["native_gt_verification"]["render_visual_review_required"])
        self.assertTrue(all(request["requested_state_confirmed"] for request in result["requests"]))
        for frame in result["frames"]:
            self.assertTrue(frame["native_GT_verified"])
            self.assertTrue(frame["requested_state_confirmed"])
            native = frame["native_gt"]
            self.assertTrue(native["jpeg_receipt_bracketed"])
            self.assertLess(native["before"]["completed_monotonic"], frame["receipt_monotonic"])
            self.assertLessEqual(frame["receipt_monotonic"], native["after"]["started_monotonic"])
            self.assertEqual(native["observed_state"], frame["requested_state"])
            self.assertEqual((self.output / frame["image_path"]).read_bytes(), jpeg_fixture())
            self.assertIsNone(frame["human_ground_truth"])
        reset = result["reset"]
        self.assertTrue(reset["native_identity_and_known_state_verified"])
        self.assertEqual(reset["native_readback"]["readback"]["observations"][0]["tl_color"], 16)
        self.assertIsNone(reset["native_readback"]["expected_state"])
        self.assertFalse(reset["simulator_applied_confirmed"])
        self.assertFalse(reset["schedule_reset_applied_confirmed"])
        self.assertFalse(reset["previous_phase_restored_confirmed"])

    def test_phase_polls_native_until_expected_state_not_udp_success_alone(self):
        calls = []
        def delayed(native):
            calls.append(native)
            return modern_row(native, 4 if len(calls) == 1 else 1)
        result = self.run_capture(states=[1], samples_per_state=1, read_native_state=delayed)
        polls = [row for row in result["native_reads"] if row["stage"] == "phase_after_udp"]
        self.assertEqual(len(polls), 2)
        self.assertFalse(polls[0]["verified"])
        self.assertTrue(polls[1]["verified"])
        self.assertTrue(result["native_GT_verified"])

    def test_post_frame_mismatch_keeps_original_raw_and_unverified_record(self):
        calls = []
        def mismatch(native):
            calls.append(native)
            return modern_row(native, 4 if len(calls) == 3 else 1)
        with self.assertRaises(NativeGTError):
            self.run_capture(states=[1], samples_per_state=1, read_native_state=mismatch)
        result = json.loads((self.output / "manifest.json").read_text())
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["frame_count"], 1)
        frame = result["frames"][0]
        self.assertFalse(frame["native_GT_verified"])
        self.assertFalse(frame["requested_state_confirmed"])
        self.assertFalse(result["native_GT_verified"])
        self.assertEqual(frame["native_gt"]["after"]["readback"]["observations"][0]["tl_color"], 4)
        self.assertEqual((self.output / frame["image_path"]).read_bytes(), jpeg_fixture())
        self.assertEqual(self.sent[-1][1], -1)

    def test_post_frame_rpc_error_also_keeps_raw_evidence_and_resets(self):
        calls = []
        def failed(native):
            calls.append(native)
            if len(calls) == 3:
                raise RuntimeError("fake post-RPC error")
            return modern_row(native)
        with self.assertRaisesRegex(RuntimeError, "fake post-RPC"):
            self.run_capture(states=[1], samples_per_state=1, read_native_state=failed)
        result = json.loads((self.output / "manifest.json").read_text())
        self.assertEqual(result["frame_count"], 1)
        self.assertFalse(result["frames"][0]["native_GT_verified"])
        self.assertTrue(any("error" in row for row in result["native_reads"]))
        self.assertTrue((self.output / result["frames"][0]["image_path"]).is_file())
        self.assertEqual(self.sent[-1][1], -1)

    def test_mismatched_identity_cannot_be_native_gt(self):
        with self.assertRaises(NativeGTError):
            self.run_capture(states=[1], samples_per_state=1, read_native_state=lambda native: modern_row("OTHER", 1))
        result = json.loads((self.output / "manifest.json").read_text())
        self.assertFalse(result["native_GT_verified"])
        self.assertFalse(result["requests"][0]["requested_state_confirmed"])
        self.assertEqual(result["frame_count"], 0)
        self.assertEqual(self.sent[-1][1], -1)

    def test_final_reset_bad_identity_marks_failed_readback_without_restoration_claim(self):
        def read(native):
            return modern_row("OTHER" if self.sent[-1][1] == -1 else native)
        with self.assertRaisesRegex(RuntimeError, "reset"):
            self.run_capture(states=[1], samples_per_state=1, read_native_state=read)
        result = json.loads((self.output / "manifest.json").read_text())
        self.assertEqual(result["status"], "failed_reset_readback")
        self.assertFalse(result["native_GT_verified"])
        self.assertTrue(result["frames"][0]["native_GT_verified"])
        self.assertFalse(result["reset"]["previous_phase_restored_confirmed"])

    def test_verification_rejects_schedule_phase_before_any_udp(self):
        with self.assertRaises(ValueError):
            self.run_capture(states=[-1], read_native_state=self.native_read)
        self.assertEqual(self.sent, [])

    def test_one_retry_keeps_rejected_raw_and_completes_accepted_quota(self):
        calls = []
        def once_bad(native):
            calls.append(native)
            if len(calls) == 3:
                return modern_row(native, 4)
            return self.native_read(native)
        result = self.run_capture(states=[1], samples_per_state=2, read_native_state=once_bad,
                                  native_mismatch_retries=2)
        self.assertEqual([state for _, state in self.sent], [1, 1, -1])
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["frame_count"], len(result["frames"]))
        self.assertEqual(result["frame_count"], 3)
        self.assertEqual(result["raw_frame_count"], 3)
        self.assertEqual(result["accepted_frame_count"], 2)
        self.assertEqual(result["rejected_frame_count"], 1)
        self.assertFalse(result["native_GT_verified"])
        self.assertTrue(result["accepted_frames_native_GT_verified"])
        first, replacement, second = result["frames"]
        self.assertFalse(first["native_GT_verified"])
        self.assertFalse(first["accepted_for_sample_quota"])
        self.assertEqual((first["sample_index"], replacement["sample_index"], second["sample_index"]), (0, 0, 1))
        self.assertEqual((first["attempt_index"], replacement["attempt_index"], second["attempt_index"]), (0, 1, 0))
        self.assertTrue(replacement["image_path"].endswith("_attempt_01.jpg"))
        self.assertNotIn("_attempt_", first["image_path"])
        self.assertGreater(replacement["receipt_monotonic"], first["receipt_monotonic"])
        self.assertEqual(len({frame["image_path"] for frame in result["frames"]}), 3)
        for frame in result["frames"]:
            self.assertEqual((self.output / frame["image_path"]).read_bytes(), jpeg_fixture())
        self.assertTrue(all(frame["native_gt"]["jpeg_receipt_bracketed"]
                            for frame in result["frames"] if frame["accepted_for_sample_quota"]))

    def test_two_retries_are_bounded_and_exhausted_attempts_are_preserved(self):
        calls = []
        def always_post_bad(native):
            if self.sent[-1][1] == -1:
                return modern_row(native, 16)
            calls.append(native)
            return modern_row(native, 4 if len(calls) % 3 == 0 else 1)
        with self.assertRaises(NativeGTError):
            self.run_capture(states=[1], samples_per_state=1, read_native_state=always_post_bad,
                             native_mismatch_retries=2)
        result = json.loads((self.output / "manifest.json").read_text())
        self.assertEqual([state for _, state in self.sent], [1, 1, 1, -1])
        self.assertEqual(result["frame_count"], len(result["frames"]))
        self.assertEqual(result["frame_count"], 3)
        self.assertEqual(result["accepted_frame_count"], 0)
        self.assertEqual(result["rejected_frame_count"], 3)
        self.assertEqual([row["will_retry"] for row in result["native_mismatches"]], [True, True, False])
        self.assertEqual(len({frame["image_path"] for frame in result["frames"]}), 3)
        self.assertFalse(result["accepted_frames_native_GT_verified"])

    def test_pre_frame_mismatch_can_retry_without_inventing_a_raw_image(self):
        calls = []
        def before_bad_once(native):
            calls.append(native)
            return modern_row(native, 4) if len(calls) == 2 else self.native_read(native)
        result = self.run_capture(states=[1], samples_per_state=1, read_native_state=before_bad_once,
                                  native_mismatch_retries=1)
        self.assertEqual(result["frame_count"], 1)
        self.assertEqual(result["accepted_frame_count"], 1)
        self.assertEqual(result["native_mismatches"][0]["retained_frame_indices"], [])
        self.assertEqual(result["frames"][0]["attempt_index"], 1)
        self.assertTrue(result["frames"][0]["native_GT_verified"])

    def test_rpc_failure_does_not_retry_even_when_retries_are_enabled(self):
        calls = []
        def failed(native):
            calls.append(native)
            if len(calls) == 3:
                raise RuntimeError("fake transport error")
            return self.native_read(native)
        with self.assertRaisesRegex(RuntimeError, "fake transport"):
            self.run_capture(states=[1], samples_per_state=1, read_native_state=failed,
                             native_mismatch_retries=2)
        self.assertEqual([state for _, state in self.sent], [1, -1])
        result = json.loads((self.output / "manifest.json").read_text())
        self.assertEqual(result["accepted_frame_count"], 0)
        self.assertEqual(result["native_mismatches"], [])

    def test_stationary_failure_after_mismatch_prevents_retry_udp(self):
        calls = []
        def failed(native):
            calls.append(native)
            if len(calls) == 3:
                self.refresh(gear=4)
                return modern_row(native, 4)
            return self.native_read(native)
        with self.assertRaises(UnsafeState):
            self.run_capture(states=[1], samples_per_state=1, read_native_state=failed,
                             native_mismatch_retries=2)
        self.assertEqual([state for _, state in self.sent], [1, -1])

    def test_retry_parameters_require_verification_and_zero_to_two_integer_bound(self):
        for retries in (-1, 3, True, 1.5):
            with self.subTest(retries=retries), self.assertRaises(ValueError):
                self.run_capture(native_mismatch_retries=retries, read_native_state=self.native_read)
        with self.assertRaises(ValueError):
            self.run_capture(native_mismatch_retries=1)
        self.assertEqual(self.sent, [])


class NativeReadContractTest(unittest.TestCase):
    def test_explicit_known_positive_color_and_exact_identity_are_required(self):
        self.assertIsNone(native_readback_issue(modern_row(color=48), "C119BS010025", 48))
        for row in (modern_row("OTHER"), modern_row(color=0), modern_row(color=-1),
                    modern_row(color=9999), modern_row(omit_color=True)):
            self.assertIsNotNone(native_readback_issue(row, "C119BS010025", 1))
        self.assertIsNotNone(native_readback_issue({"success": True}, "C119BS010025", 1))

    def test_modern_reader_calls_only_fixed_read_method_without_real_grpc(self):
        fake_grpc, channel = Mock(), Mock()
        fake_grpc.insecure_channel.return_value = channel
        read_call = channel.unary_unary.return_value
        # Reuse the raw wire response preserved by the actual parser helper.
        read_call.return_value = bytes.fromhex(modern_row(color=48)["response_hex"])
        with patch.dict("sys.modules", {"grpc":fake_grpc}):
            reader = ModernNativeReader()
            result = reader("C119BS010025")
            reader.close()
        self.assertTrue(result["readback_success"])
        self.assertEqual(channel.unary_unary.call_args.args[0],
                         "/" + MODERN_SERVICE + "/GetTrafficLightInfo")
        self.assertEqual(read_call.call_args.args[0], encode_modern_target("C119BS010025"))
        self.assertEqual(read_call.call_args.kwargs["timeout"], 1.0)
        channel.close.assert_called_once()


class FakeStatus(Enum):
    UNKNOWN = 1
    UNAVAILABLE = 2
    DEADLINE_EXCEEDED = 3
    UNIMPLEMENTED = 4


class FakeRpcError(Exception):
    def __init__(self, status, details="fixture"):
        super().__init__(details)
        self.status, self.text = status, details
    def code(self):
        return self.status
    def details(self):
        return self.text


class RpcRetryTests(unittest.TestCase):
    def test_only_named_transient_rpc_errors_and_exact_unknown_message_retry(self):
        for status, details in ((FakeStatus.UNKNOWN, "Exception was thrown by handler."),
                                (FakeStatus.UNAVAILABLE, "temporary"),
                                (FakeStatus.DEADLINE_EXCEEDED, "deadline")):
            with self.subTest(status=status):
                call = Mock(side_effect=[FakeRpcError(status, details), b"actual fresh response"])
                attempts, sleeps = [], []
                response = read_native_rpc_with_retries(call, retries=2, rpc_error_type=FakeRpcError,
                                                       attempts=attempts, sleep=sleeps.append)
                self.assertEqual(response, b"actual fresh response")
                self.assertEqual(call.call_count, 2)
                self.assertEqual(sleeps, [.05])
                self.assertEqual([row["status"] for row in attempts], [status.name, "OK"])
                self.assertTrue(attempts[0]["will_retry"])
                self.assertTrue(all(row["completed_monotonic"] >= row["started_monotonic"] for row in attempts))

    def test_default_zero_and_explicit_two_retry_bound_preserve_errors(self):
        error = FakeRpcError(FakeStatus.UNAVAILABLE)
        for retries, expected in ((0, 1), (2, 3)):
            call, attempts, sleeps = Mock(side_effect=error), [], []
            with self.subTest(retries=retries), self.assertRaises(FakeRpcError):
                read_native_rpc_with_retries(call, retries=retries, rpc_error_type=FakeRpcError,
                                            attempts=attempts, sleep=sleeps.append)
            self.assertEqual(call.call_count, expected)
            self.assertEqual(len(attempts), expected)
            self.assertEqual(len(sleeps), retries)
            self.assertFalse(attempts[-1]["will_retry"])

    def test_unimplemented_other_unknown_protocol_and_non_rpc_errors_never_retry(self):
        errors = [FakeRpcError(FakeStatus.UNIMPLEMENTED), FakeRpcError(FakeStatus.UNKNOWN, "different handler"),
                  ProtocolError("bad wire"), ValueError("identity mismatch"), RuntimeError("other")]
        for error in errors:
            call, attempts = Mock(side_effect=error), []
            with self.subTest(error=error), self.assertRaises(type(error)):
                read_native_rpc_with_retries(call, retries=2, rpc_error_type=FakeRpcError,
                                            attempts=attempts, sleep=lambda _: self.fail("must not sleep"))
            self.assertEqual(call.call_count, 1)
            self.assertFalse(attempts[0]["will_retry"])

    def test_global_deadline_caps_rpc_timeout_backoff_and_late_success(self):
        now = [0.]
        attempts, timeouts = [], []
        def call(timeout):
            timeouts.append(timeout)
            now[0] += .06
            raise FakeRpcError(FakeStatus.UNAVAILABLE)
        with self.assertRaises(FakeRpcError):
            read_native_rpc_with_retries(call, retries=2, rpc_error_type=FakeRpcError, deadline=.1,
                attempts=attempts, monotonic=lambda: now[0], sleep=lambda _: self.fail("budget too short for backoff"))
        self.assertEqual(timeouts, [.1])
        self.assertTrue(attempts[0]["retry_prevented_by_global_deadline"])
        call = Mock(return_value=b"must not be queried")
        with self.assertRaises(TimeoutError):
            read_native_rpc_with_retries(call, deadline=0., monotonic=lambda: 0.)
        call.assert_not_called()
        now[0] = 0.
        def late_success(timeout):
            now[0] = .2
            return b"late actual response"
        with self.assertRaises(TimeoutError):
            read_native_rpc_with_retries(late_success, deadline=.1, monotonic=lambda: now[0])

    def test_parser_failure_and_bad_identity_do_not_trigger_another_rpc(self):
        for wire, exception in ((b"\x80", ProtocolError),
                                (bytes.fromhex(modern_row("OTHER")["response_hex"]), None)):
            fake_grpc, channel = Mock(), Mock()
            fake_grpc.RpcError = FakeRpcError
            fake_grpc.insecure_channel.return_value = channel
            read_call = channel.unary_unary.return_value
            read_call.return_value = wire
            with patch.dict("sys.modules", {"grpc":fake_grpc}):
                reader = ModernNativeReader(rpc_retries=2, sleep=lambda _: self.fail("must not retry"))
                if exception:
                    with self.assertRaises(exception):reader("C119BS010025")
                else:
                    result = reader("C119BS010025")
                    self.assertFalse(result["readback_success"])
                reader.close()
            self.assertEqual(read_call.call_count, 1)

    def test_reader_returns_fresh_response_after_transient_error_with_attempt_log(self):
        fake_grpc, channel = Mock(), Mock()
        fake_grpc.RpcError = FakeRpcError
        fake_grpc.insecure_channel.return_value = channel
        call = channel.unary_unary.return_value
        call.side_effect = [FakeRpcError(FakeStatus.UNKNOWN, "Exception was thrown by handler."),
                            bytes.fromhex(modern_row(color=48)["response_hex"])]
        with patch.dict("sys.modules", {"grpc":fake_grpc}):
            reader = ModernNativeReader(rpc_retries=2, sleep=lambda _: None)
            result = reader("C119BS010025")
            reader.close()
        self.assertTrue(result["readback_success"])
        self.assertEqual(result["observations"][0]["tl_color"], 48)
        self.assertEqual([a["status"] for a in result["rpc_attempts"]], ["UNKNOWN", "OK"])

    def test_retry_parameter_is_strict_zero_to_two_integer(self):
        for invalid in (-1, 3, True, 1.5, "2"):
            with self.assertRaises(ValueError):validate_rpc_retries(invalid)


if __name__ == "__main__":
    unittest.main()
