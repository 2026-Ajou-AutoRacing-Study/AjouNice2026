"""Pure CPU protocol tests: no ROS, simulator, socket, grpc, or subprocess use."""

import importlib.util
from pathlib import Path
import unittest


SPEC = importlib.util.spec_from_file_location(
    "probe_morai_traffic_light_gt", Path(__file__).with_name("probe_morai_traffic_light_gt.py"))
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def string_field(number, value):
    encoded = value.encode("utf-8")
    return probe.encode_varint(number * 8 + 2) + probe.encode_varint(len(encoded)) + encoded


def info(unique_id="C1256W000077", color=1, success=True):
    return (b"\x08" + probe.encode_varint(int(success)) + string_field(2, unique_id)
            + b"\x18" + probe.encode_varint(color if color >= 0 else 2**64 + color))


def list_wire(*messages):
    return b"".join(b"\x0a" + probe.encode_varint(len(message)) + message for message in messages)


class WireTests(unittest.TestCase):
    def test_request_is_utf8_field_one_and_has_no_set_fields(self):
        self.assertEqual(probe.encode_target("IntTL9"), b"\x0a\x06IntTL9")
        self.assertEqual(probe.encode_target("신호"), b"\x0a\x06" + "신호".encode())
        self.assertEqual(probe.encode_target("x" * 130)[:3], b"\x0a\x82\x01")

    def test_invalid_ids(self):
        for value in ("", " x", "x ", "x\n", "x\x00", "x" * 257, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                probe.encode_target(value)

    def test_varint_roundtrip(self):
        for value in (0, 1, 127, 128, 16383, 2**32, 2**64 - 1):
            encoded = probe.encode_varint(value)
            self.assertEqual(probe._varint(encoded, 0), (value, len(encoded)))
        for value in (-1, 2**64, True, 1.5):
            with self.assertRaises(ValueError):
                probe.encode_varint(value)

    def test_native_color_is_not_a_model_label(self):
        parsed = probe.parse_tl_info(info(color=33))
        self.assertEqual(parsed, {"success": True, "tl_id": "C1256W000077",
                                "tl_color": 33, "tl_color_name": "R_WITH_GLEFT",
                                "color_enum_known": True})

    def test_proto3_defaults_and_negative_enum(self):
        self.assertFalse(probe.parse_tl_info(b"")["success"])
        self.assertEqual(probe.parse_tl_info(b"")["tl_color_name"], "NONE")
        for color, name in ((-1, "UNDEFINED"), (-2, "NOT_DETECTED")):
            parsed = probe.parse_tl_info(info(color=color))
            self.assertEqual((parsed["tl_color"], parsed["tl_color_name"]), (color, name))

    def test_future_enum_preserved_without_relabeling(self):
        parsed = probe.parse_tl_info(info(color=9999))
        self.assertEqual(parsed["tl_color"], 9999)
        self.assertFalse(parsed["color_enum_known"])

    def test_unknown_fields_skipped_and_last_singular_value_wins(self):
        unknown = b"\x40\x01\x49" + b"0" * 8 + b"\x52\x02xy\x5d" + b"1" * 4
        parsed = probe.parse_tl_info(info() + unknown + b"\x18\x05")
        self.assertEqual(parsed["tl_color"], 5)

    def test_repeated_tl_info_and_unknown_outer_field(self):
        parsed = probe.parse_tl_info_list(list_wire(info("a", 4), info("b", 48)) + b"\x20\x01")
        self.assertEqual([x["tl_id"] for x in parsed], ["a", "b"])
        self.assertEqual([x["tl_color"] for x in parsed], [4, 48])
        self.assertEqual(probe.parse_tl_info_list(b""), [])

    def test_malformed_wire_rejected(self):
        bad = (b"\x80", b"\x00", b"\x0b", b"\x12\x04x", b"\x18" + b"\xff" * 10,
               b"\x12\x01\xff", b"\x0a\x00", b"\x10\x01", b"\x1a\x00",
               b"\x49" + b"x" * 7, b"\x5d" + b"x" * 3)
        for payload in bad:
            with self.subTest(payload=payload), self.assertRaises(probe.ProtocolError):
                probe.parse_tl_info(payload)
        with self.assertRaises(probe.ProtocolError):
            probe.parse_tl_info_list(b"\x08\x01")
        with self.assertRaises(probe.ProtocolError):
            probe.parse_tl_info(b"x" * (probe.MAX_RESPONSE_BYTES + 1))


class ReadOnlyTests(unittest.TestCase):
    def test_endpoint_is_loopback_only_without_dns(self):
        self.assertEqual(probe.validate_endpoint("127.0.0.1:7789"), "127.0.0.1:7789")
        for endpoint in ("192.168.0.1:7789", "0.0.0.0:7789", "localhost:7789",
                         "example.com:7789", "127.0.0.1:0", "127.0.0.1:65536",
                         "127.0.0.1:-1", "127.0.0.1:1/x", "[::1]:7789"):
            with self.subTest(endpoint=endpoint), self.assertRaises(ValueError):
                probe.validate_endpoint(endpoint)

    def test_timeout_is_bounded(self):
        self.assertEqual(probe.validate_timeout(3), 3.0)
        for value in (0, -1, float("nan"), float("inf"), 31):
            with self.assertRaises(ValueError):
                probe.validate_timeout(value)

    def test_only_two_read_methods_can_be_called(self):
        calls = []
        def fake_call(path, request):
            calls.append((path, request))
            return info() if path.endswith("ByUid") else list_wire(info(color=5))
        results = probe.query_states([("traffic_light", "C1256W000077"),
                                      ("intersection", "IntTL9")], fake_call, "22r3")
        self.assertEqual([path for path, _ in calls], [
            "/" + probe.SERVICE + "/GetTrafficLightInfoByUid",
            "/" + probe.SERVICE + "/GetIntscnTLInfo"])
        self.assertTrue(all(row["readback_success"] for row in results))
        self.assertEqual(results[1]["observations"][0]["tl_color"], 5)
        self.assertEqual(bytes.fromhex(results[0]["response_hex"]), info())
        with self.assertRaises(ValueError):
            probe.query_states([("SetTrafficLightState", "x")], fake_call, "22r3")
        self.assertEqual(len(calls), 2)

    def test_empty_and_rejected_native_readback_not_success(self):
        for kind, payload in (("intersection", b""), ("traffic_light", info(success=False))):
            results = probe.query_states([(kind, "x")], lambda path, request: payload, "22r3")
            self.assertFalse(results[0]["readback_success"])

    def test_provenance_does_not_claim_actual_server_compatibility(self):
        provenance = probe.source_provenance()
        self.assertIn("UNVERIFIED", provenance["actual_server_compatibility"])
        self.assertEqual(len(provenance["sdk_artifacts"]), 2)
        self.assertEqual(set(probe.READ_METHODS.values()),
                         {"GetTrafficLightInfoByUid", "GetIntscnTLInfo"})


def modern_info(unique_id="C1256W000077", color=1, *, omit_color=False):
    nested = string_field(1, unique_id)
    payload = b"\x12" + probe.encode_varint(len(nested)) + nested
    return payload if omit_color else payload + b"\x18" + probe.encode_varint(color if color >= 0 else 2**64 + color)


class ModernProtocolTests(unittest.TestCase):
    def test_request_is_by_native_id_enum_two_string_field_two(self):
        self.assertEqual(probe.encode_modern_target("C1256W000077"),
                         b"\x08\x02\x12\x0cC1256W000077")

    def test_modern_nested_identifier_and_explicit_color(self):
        parsed = probe.parse_modern_tl_info(modern_info(color=48))
        self.assertEqual(parsed["tl_id"], "C1256W000077")
        self.assertEqual(parsed["tl_color"], 48)
        self.assertIsNone(parsed["success"])
        self.assertFalse(parsed["native_success_field_present"])
        self.assertTrue(parsed["color_usable_for_gt"])

    def test_modern_only_exact_requested_identity_and_known_color_are_success(self):
        for payload, expected in ((modern_info(), True), (modern_info("other"), False),
                                  (modern_info(color=0), False), (modern_info(color=-1), False),
                                  (modern_info(color=9999), False), (modern_info(omit_color=True), False),
                                  (b"", False)):
            calls = []
            def call(path, request):
                calls.append(path)
                return payload
            result = probe.query_states([("traffic_light", "C1256W000077")], call)[0]
            self.assertEqual(result["readback_success"], expected)
            self.assertEqual(calls, ["/morai_sim_api.infrastructure.Infrastructure/GetTrafficLightInfo"])

    def test_modern_intersection_is_rejected_before_any_call(self):
        calls = []
        with self.assertRaises(ValueError):
            probe.query_states([("traffic_light", "x"), ("intersection", "IntTL9")],
                               lambda *args: calls.append(args))
        self.assertEqual(calls, [])

    def test_modern_schema_not_confused_with_22r3_and_bad_wire_rejected(self):
        for payload in (b"\x10\x01", b"\x12\x02\x08\x01", b"\x1a\x00", b"\x12\x03\x0a\x01\xff"):
            with self.subTest(payload=payload), self.assertRaises(probe.ProtocolError):
                probe.parse_modern_tl_info(payload)
        with self.assertRaises(probe.ProtocolError):
            probe.parse_modern_tl_info(info())  # Old field2 string is not a modern nested ID.

    def test_default_cli_and_provenance_are_modern(self):
        self.assertEqual(probe._parser().parse_args(["--traffic-light-id", "x"]).protocol, "modern")
        self.assertFalse(probe.source_provenance("modern")["contract"]["native_success_field_present"])


if __name__ == "__main__":
    unittest.main()
