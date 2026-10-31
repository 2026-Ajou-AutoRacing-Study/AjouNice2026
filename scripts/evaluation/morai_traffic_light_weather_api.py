#!/usr/bin/env python3
"""MORAI development weather/time API; default is a read-only Get snapshot.

The live simulator owner must run this tool. Set operations require BOTH an
explicit requested value and --apply-development-weather. No Connect, Start,
Stop, actor, traffic-light, vehicle, ROS, or subprocess operations are provided.
API state readback does NOT certify the rendered camera weather or time.
"""

import argparse
from datetime import datetime, timezone
import json
import sys
import time

from probe_morai_traffic_light_gt import (
    ProtocolError, _fields, encode_varint, validate_endpoint, validate_timeout,
)


SERVICE = "/morai_sim_api.environment.Environment/"
METHODS = frozenset(("GetTime", "GetWeather", "SetTime", "SetWeather"))
WEATHER = {"Sunny": 0, "Foggy": 2}
WEATHER_NAMES = {0: "Sunny", 1: "Cloudy", 2: "Foggy", 3: "Storm", 4: "Rainy", 5: "Snowy"}
POLL_SECONDS = 5.0
PROVENANCE = {
    "source": "/home/luv/Applications/WeatherTimeChanger_Linux",
    "source_sha256": "8c47f48a8d54b79b657920a661f582bd587c9f9065b6b55c9e911d42a0369609",
    "inspection": "Read-only PyInstaller CArchive/PYZ Python3.8 bytecode and protobuf descriptors; no entrypoint execution",
    "service": "morai_sim_api.environment.Environment",
    "schema": {
        "GetTime": "Empty -> SimulationTime(hour:int32=1, data_only:bool=2)",
        "SetTime": "SimulationTime -> Result(status:enum=1, description:string=2, custom_message:string=3)",
        "GetWeather": "Empty -> Weather(type:enum=1)",
        "SetWeather": "Weather -> Result",
        "status": {"UNKNOWN_FAILURE": 0, "SUCCESS": 1},
        "weather_type": WEATHER_NAMES,
        "set_time_data_only": False,
    },
    "actual_server_compatibility": "UNVERIFIED_UNTIL_LIVE_OWNER_CHECKS_RESPONSES",
}


class ReadbackDeadline(Exception):
    pass


def _int32(value):
    value &= 0xffffffff
    return value - 2**32 if value >= 2**31 else value


def decode_response(method, payload):
    if method not in METHODS:
        raise ValueError("unsupported RPC method")
    if method.startswith("Set"):
        result = {"status": 0, "description": "", "custom_message": ""}
        for field, wire, value in _fields(payload):
            if field == 1:
                if wire != 0:
                    raise ProtocolError("Result.status must be a varint")
                result["status"] = _int32(value)
            elif field in (2, 3):
                if wire != 2:
                    raise ProtocolError("Result text must be length-delimited")
                try:
                    result["description" if field == 2 else "custom_message"] = value.decode("utf-8")
                except UnicodeDecodeError as error:
                    raise ProtocolError("Result text is not UTF-8") from error
        result["status_name"] = {0: "UNKNOWN_FAILURE", 1: "SUCCESS"}.get(result["status"], "UNKNOWN_ENUM_VALUE")
        return result
    result = {"hour": 0, "data_only": False} if method == "GetTime" else {"weather_type": 0}
    for field, wire, value in _fields(payload):
        if field == 1 or (method == "GetTime" and field == 2):
            if wire != 0:
                raise ProtocolError("time/weather scalar must be a varint")
            if field == 2:
                result["data_only"] = bool(value)
            else:
                result["hour" if method == "GetTime" else "weather_type"] = _int32(value)
    if method == "GetWeather":
        result["weather"] = WEATHER_NAMES.get(result["weather_type"], "UNKNOWN_ENUM_VALUE")
    return result


def encode_request(method, value=None):
    if method not in METHODS:
        raise ValueError("unsupported RPC method")
    if method.startswith("Get"):
        if value is not None:
            raise ValueError("Get requests must be empty")
        return b""
    if method == "SetTime":
        if isinstance(value, bool) or value not in (11, 13, 15):
            raise ValueError("development hour must be 11, 13 or 15")
        # data_only=false is the inspected client's default. Proto3 omits false.
        return b"\x08" + encode_varint(value)
    if value not in WEATHER:
        raise ValueError("development weather must be Sunny or Foggy")
    number = WEATHER[value]
    return b"\x08" + encode_varint(number) if number else b""


def execute(*, rpc_call, weather=None, hour=None, apply=False,
            timeout=3.0, monotonic=time.monotonic, sleep=time.sleep):
    """Injectable transport/clock make all workflow tests network-free."""
    timeout = validate_timeout(timeout)
    mutating = weather is not None or hour is not None
    if mutating and not apply:
        raise ValueError("Set requires --apply-development-weather")
    # Validate every requested value before ANY RPC, including the first Set.
    setters = []
    if weather is not None:
        setters.append(("SetWeather", encode_request("SetWeather", weather)))
    if hour is not None:
        setters.append(("SetTime", encode_request("SetTime", hour)))
    output = {
        "tool": "morai_traffic_light_weather_api",
        "read_only": not mutating,
        "mode": "development_weather_write" if mutating else "read_only_get",
        "requested": {"weather": weather, "hour": hour},
        "actual": None, "renderer_not_verified": True,
        "render_verification_required": "Independently inspect fresh front JPEG/UI rendering; API Get alone is not camera ground truth.",
        "source_provenance": PROVENANCE,
        "readback_match": None, "exchanges": [],
    }

    def exchange(method, request, per_call_timeout):
        if method not in METHODS:
            raise ValueError("unsupported RPC method")
        row = {"method": method, "request_hex": request.hex(),
               "requested_at_utc": datetime.now(timezone.utc).isoformat()}
        output["exchanges"].append(row)
        try:
            response = rpc_call(SERVICE + method, request, per_call_timeout)
            row["response_hex"] = response.hex()
            row["parsed"] = decode_response(method, response)
            return row["parsed"]
        except Exception as error:
            row["error"] = str(error)
            raise

    def snapshot(deadline=None):
        values = {}
        for method in ("GetTime", "GetWeather"):
            remaining = timeout if deadline is None else min(timeout, deadline - monotonic())
            if remaining <= 0:
                raise ReadbackDeadline()
            values.update(exchange(method, b"", remaining))
        values["received_at_utc"] = datetime.now(timezone.utc).isoformat()
        output["actual"] = values
        return values

    try:
        for method, request in setters:
            result = exchange(method, request, timeout)
            if result["status"] != 1:
                output["status"] = "SET_REJECTED"
                return output
        if not mutating:
            snapshot()
            output["status"] = "READ_ONLY_GET"
            return output
        deadline = monotonic() + POLL_SECONDS
        output["poll_deadline_sec"] = POLL_SECONDS
        while True:
            actual = snapshot(deadline)
            matched = ((weather is None or actual["weather_type"] == WEATHER[weather])
                       and (hour is None or actual["hour"] == hour))
            output["readback_match"] = matched
            if matched:
                output["status"] = "READBACK_MATCH"
                return output
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise ReadbackDeadline()
            sleep(min(0.2, remaining))
    except ReadbackDeadline:
        output["status"] = "READBACK_TIMEOUT"
        output["readback_match"] = False
    except Exception as error:
        output["status"] = "PROTOCOL_ERROR" if isinstance(error, ProtocolError) else "RPC_ERROR"
        output["error"] = str(error)
        if hasattr(error, "code") and callable(error.code):
            output["grpc_status"] = str(error.code())
    return output


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--get", action="store_true", help="Read snapshot (the default); cannot accompany Set")
    parser.add_argument("--set-weather", choices=tuple(WEATHER))
    parser.add_argument("--set-hour", type=int, choices=(11, 13, 15))
    parser.add_argument("--apply-development-weather", action="store_true",
                        help="Explicitly authorize the requested local-development Set RPCs")
    parser.add_argument("--endpoint", default="127.0.0.1:7789", help="Numeric IPv4 loopback only")
    parser.add_argument("--timeout", type=float, default=3.0, help="Per-RPC timeout seconds (default 3)")
    return parser


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    mutating = args.set_weather is not None or args.set_hour is not None
    if mutating and (not args.apply_development_weather or args.get):
        parser.error("Set requires --apply-development-weather and cannot accompany --get")
    try:
        endpoint = validate_endpoint(args.endpoint)
        timeout = validate_timeout(args.timeout)
    except ValueError as error:
        parser.error(str(error))
    try:
        import grpc
    except ImportError:
        print(json.dumps({"status": "DEPENDENCY_ERROR", "error": "grpcio is required; nothing was queried",
                          "read_only": not mutating, "renderer_not_verified": True}))
        return 2
    with grpc.insecure_channel(endpoint, options=[("grpc.max_receive_message_length", 1024 * 1024)]) as channel:
        def rpc_call(path, request, per_call_timeout):
            if path not in {SERVICE + method for method in METHODS}:
                raise ValueError("unsupported RPC path")
            return channel.unary_unary(path, request_serializer=lambda value: value,
                                       response_deserializer=lambda value: value)(request, timeout=per_call_timeout)
        result = execute(rpc_call=rpc_call, weather=args.set_weather, hour=args.set_hour,
                         apply=args.apply_development_weather, timeout=timeout)
    result["endpoint"] = endpoint
    result["timeout_sec"] = timeout
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] in ("READ_ONLY_GET", "READBACK_MATCH") else 2


if __name__ == "__main__":
    sys.exit(main())
