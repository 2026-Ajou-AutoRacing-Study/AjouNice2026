"""Pure tests; all transports and waiting clocks are fake."""

import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).parent))
SPEC = importlib.util.spec_from_file_location("weather_api", Path(__file__).with_name("morai_traffic_light_weather_api.py"))
api = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(api)


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class WeatherApiTests(unittest.TestCase):
    def test_actual_proto_schema_encoding_and_default_sunny(self):
        self.assertEqual(api.encode_request("GetTime"), b"")
        self.assertEqual(api.encode_request("SetWeather", "Sunny"), b"")
        self.assertEqual(api.encode_request("SetWeather", "Foggy"), b"\x08\x02")
        for hour in (11, 13, 15):
            self.assertEqual(api.encode_request("SetTime", hour), bytes((8, hour)))
        self.assertEqual(api.decode_response("GetWeather", b""), {"weather_type": 0, "weather": "Sunny"})
        self.assertEqual(api.decode_response("GetTime", b"\x08\x0f"), {"hour": 15, "data_only": False})
        self.assertEqual(api.decode_response("GetTime", b"\x08\x0b\x10\x01")["data_only"], True)

    def test_no_write_without_explicit_authority_and_prevalidation(self):
        def forbidden(*args):
            self.fail("RPC must not be called")
        for kwargs in ({"weather": "Foggy"}, {"hour": 15},
                       {"weather": "Sunny", "hour": 12, "apply": True}):
            with self.assertRaises(ValueError):
                api.execute(rpc_call=forbidden, **kwargs)

    def test_cli_rejects_set_without_optin_before_transport(self):
        for argv in (["--set-hour", "15"], ["--get", "--set-weather", "Foggy", "--apply-development-weather"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
                api.main(argv)
            self.assertEqual(caught.exception.code, 2)

    def test_get_is_the_default_and_does_not_call_connect_start_or_set(self):
        calls = []
        def rpc(path, request, timeout):
            calls.append((path, request, timeout))
            return b"\x08\x0d" if path.endswith("GetTime") else b"\x08\x02"
        result = api.execute(rpc_call=rpc)
        self.assertEqual([x[0] for x in calls], [api.SERVICE + "GetTime", api.SERVICE + "GetWeather"])
        self.assertTrue(result["read_only"])
        self.assertTrue(result["renderer_not_verified"])
        self.assertEqual(result["actual"]["weather"], "Foggy")
        self.assertEqual(result["actual"]["hour"], 13)
        self.assertEqual(result["status"], "READ_ONLY_GET")

    def test_set_failure_status_stops_without_more_calls(self):
        calls = []
        def rpc(path, request, timeout):
            calls.append(path)
            return b"\x12\x06denied\x1a\x04gate"
        result = api.execute(rpc_call=rpc, weather="Foggy", hour=15, apply=True)
        self.assertEqual(calls, [api.SERVICE + "SetWeather"])
        self.assertEqual(result["status"], "SET_REJECTED")
        self.assertEqual(result["exchanges"][0]["parsed"]["description"], "denied")
        self.assertEqual(result["exchanges"][0]["parsed"]["custom_message"], "gate")
        self.assertEqual(result["exchanges"][0]["response_hex"], "120664656e6965641a0467617465")

    def test_success_then_delayed_get_state_match(self):
        clock = Clock()
        calls = []
        def rpc(path, request, timeout):
            calls.append((path, request, timeout))
            if "/Set" in path:
                return b"\x08\x01"
            if path.endswith("GetTime"):
                return b"\x08" + bytes((15 if clock.now > 0 else 11,))
            return b"\x08\x02" if clock.now > 0 else b""
        result = api.execute(rpc_call=rpc, weather="Foggy", hour=15, apply=True,
                             monotonic=clock.monotonic, sleep=clock.sleep)
        self.assertEqual(result["status"], "READBACK_MATCH")
        self.assertTrue(result["readback_match"])
        self.assertFalse(result["read_only"])
        self.assertTrue(result["renderer_not_verified"])
        self.assertEqual(calls[1][1], b"\x08\x0f")  # data_only remains false/omitted
        self.assertEqual(len(calls), 6)

    def test_get_stuck_old_state_times_out_at_five_seconds(self):
        clock = Clock()
        def rpc(path, request, timeout):
            return b"\x08\x01" if "/Set" in path else (b"\x08\x0b" if path.endswith("GetTime") else b"")
        result = api.execute(rpc_call=rpc, hour=15, apply=True,
                             monotonic=clock.monotonic, sleep=clock.sleep)
        self.assertEqual(result["status"], "READBACK_TIMEOUT")
        self.assertEqual(result["actual"]["hour"], 11)
        self.assertFalse(result["readback_match"])
        self.assertAlmostEqual(clock.now, 5.0)

    def test_rpc_and_protocol_failures_do_not_claim_success(self):
        def broken(path, request, timeout):
            raise RuntimeError("transport unavailable")
        result = api.execute(rpc_call=broken)
        self.assertEqual(result["status"], "RPC_ERROR")
        result = api.execute(rpc_call=lambda *args: b"\x0a\x00")
        self.assertEqual(result["status"], "PROTOCOL_ERROR")
        self.assertEqual(result["exchanges"][0]["response_hex"], "0a00")

    def test_unknown_fields_are_skipped_and_bad_strings_rejected(self):
        self.assertEqual(api.decode_response("SetTime", b"\x08\x01\x20\x7f")["status"], 1)
        with self.assertRaises(api.ProtocolError):
            api.decode_response("SetTime", b"\x12\x01\xff")
        for method in ("Connect", "Start", "SetTrafficLightState"):
            with self.assertRaises(ValueError):
                api.encode_request(method)

    def test_deadline_caps_later_gets_not_only_sleep(self):
        clock = Clock()
        calls = []
        def rpc(path, request, timeout):
            calls.append((path, timeout))
            if path.endswith("SetTime"):
                return b"\x08\x01"
            if path.endswith("GetTime"):
                clock.now += 4.9
                return b"\x08\x0b"
            self.assertLessEqual(timeout, 0.100001)
            clock.now += timeout
            return b""
        result = api.execute(rpc_call=rpc, hour=15, apply=True,
                             monotonic=clock.monotonic, sleep=clock.sleep)
        self.assertEqual(result["status"], "READBACK_TIMEOUT")
        self.assertAlmostEqual(clock.now, 5.0)


if __name__ == "__main__":
    unittest.main()
