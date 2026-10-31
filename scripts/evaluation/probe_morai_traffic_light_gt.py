#!/usr/bin/env python3
"""Read MORAI native traffic-light states, without starting or changing the sim.

The default modern protocol reads Infrastructure.GetTrafficLightInfo only.
--protocol 22r3 retains the old OpenSCENARIO ByUid/Intscn read methods.
Schemas were inspected in official local artifacts, not inferred from images.
Protocol compatibility and correspondence to the visible head need live checks.
Importing this module performs no network calls. grpcio is needed only to query.

Example (the live simulator owner must execute this):
  python3 scripts/evaluation/probe_morai_traffic_light_gt.py --traffic-light-id C1256W000077
"""

import argparse
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import sys


SERVICE = "morai_openscenario_base.MoraiOpenScenarioBaseService"
READ_METHODS = {
    "traffic_light": "GetTrafficLightInfoByUid",
    "intersection": "GetIntscnTLInfo",
}
MODERN_SERVICE = "morai_sim_api.infrastructure.Infrastructure"
MODERN_READ_METHODS = {"traffic_light": "GetTrafficLightInfo"}
MODERN_SOURCE = "/home/luv/Applications/WeatherTimeChanger_Linux"
MODERN_SOURCE_SHA256 = "8c47f48a8d54b79b657920a661f582bd587c9f9065b6b55c9e911d42a0369609"
MAX_RESPONSE_BYTES = 1024 * 1024
SDK_DIRECTORY = (
    "dataset/morai_od/local_reference/morai_openscenario_api_22r3/package/"
    "lib/openscenario/client/__pycache__"
)
SDK_FILES = (
    "morai_openscenario_msgs_pb2.cpython-37.pyc",
    "morai_openscenario_base_pb2.cpython-37.pyc",
)
INSPECTED_SDK_SHA256 = {
    "morai_openscenario_msgs_pb2.cpython-37.pyc":
        "0fab4bd671ecd88b47a94e07431580d63a6c8dc9e2b5d15586a91c253c33ddc1",
    "morai_openscenario_base_pb2.cpython-37.pyc":
        "b9c5a4da8cd0b067d2f05745a7d9553882d06d32268e62b132a8994ad51b56a6",
}
COLORS = {
    0: "NONE", 1: "R", 4: "Y", 16: "SG", 32: "LG", 64: "RG",
    128: "UTG", 256: "ULG", 512: "URG", 1024: "LLG", 2048: "LRG",
    5: "R_WITH_Y", 20: "Y_WITH_G", 36: "Y_WITH_GLEFT",
    48: "G_WITH_GLEFT", 33: "R_WITH_GLEFT", 1040: "LLG_SG",
    1025: "R_LLG", 768: "ULG_URG", -1: "UNDEFINED", -2: "NOT_DETECTED",
}


class ProtocolError(ValueError):
    """A response does not conform to the inspected protobuf wire schema."""


def encode_varint(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**64:
        raise ValueError("varint must be an unsigned 64-bit integer")
    data = bytearray()
    while value > 127:
        data.append((value & 127) | 128)
        value >>= 7
    data.append(value)
    return bytes(data)


def encode_target(unique_id):
    """TargetUniqueId.unique_id is UTF-8, protobuf field 1 (length-delimited)."""
    if not isinstance(unique_id, str) or not unique_id or unique_id != unique_id.strip():
        raise ValueError("ID must be a nonempty string without outer whitespace")
    if any(ord(char) < 32 or ord(char) == 127 for char in unique_id):
        raise ValueError("ID must not contain control characters")
    encoded = unique_id.encode("utf-8")
    if len(encoded) > 256:
        raise ValueError("ID must be at most 256 UTF-8 bytes")
    return b"\x0a" + encode_varint(len(encoded)) + encoded


def _varint(data, offset):
    value = 0
    for index in range(10):
        if offset >= len(data):
            raise ProtocolError("truncated varint")
        byte = data[offset]
        offset += 1
        if index == 9 and byte > 1:
            raise ProtocolError("varint exceeds uint64")
        value |= (byte & 127) << (7 * index)
        if not byte & 128:
            return value, offset
    raise ProtocolError("unterminated varint")


def _fields(data):
    if not isinstance(data, bytes):
        raise ProtocolError("response must be bytes")
    if len(data) > MAX_RESPONSE_BYTES:
        raise ProtocolError("response exceeds size limit")
    offset = 0
    while offset < len(data):
        tag, offset = _varint(data, offset)
        field, wire = tag >> 3, tag & 7
        if not 1 <= field < 2**29:
            raise ProtocolError("invalid protobuf field number")
        if wire == 0:
            value, offset = _varint(data, offset)
        elif wire in (1, 2, 5):
            if wire == 2:
                length, offset = _varint(data, offset)
            else:
                length = 8 if wire == 1 else 4
            end = offset + length
            if end > len(data):
                raise ProtocolError("truncated protobuf field")
            value, offset = data[offset:end], end
        else:
            raise ProtocolError("unsupported protobuf wire type {}".format(wire))
        yield field, wire, value


def parse_tl_info(data):
    """TLInfo: success bool=1, tl_id string=2, tl_color enum(int32)=3."""
    result = {"success": False, "tl_id": "", "tl_color": 0}
    for field, wire, value in _fields(data):
        if field in (1, 3):
            if wire != 0:
                raise ProtocolError("TLInfo field {} requires varint".format(field))
            if field == 1:
                result["success"] = bool(value)
            else:
                signed = value & 0xffffffff
                result["tl_color"] = signed - 2**32 if signed >= 2**31 else signed
        elif field == 2:
            if wire != 2:
                raise ProtocolError("TLInfo.tl_id requires a length-delimited string")
            try:
                result["tl_id"] = value.decode("utf-8")
            except UnicodeDecodeError as error:
                raise ProtocolError("TLInfo.tl_id is not UTF-8") from error
    result["tl_color_name"] = COLORS.get(result["tl_color"], "UNKNOWN_ENUM_VALUE")
    result["color_enum_known"] = result["tl_color"] in COLORS
    return result


def parse_tl_info_list(data):
    """TLInfoList.res_list is repeated length-delimited TLInfo, field 1."""
    results = []
    for field, wire, value in _fields(data):
        if field == 1:
            if wire != 2:
                raise ProtocolError("TLInfoList.res_list requires a nested message")
            results.append(parse_tl_info(value))
    return results


def encode_modern_target(unique_id):
    # GetTrafficLightInfoParam: type=1, enum2=BY_TL_ID; value:string=2.
    old = encode_target(unique_id)
    return b"\x08\x02\x12" + old[1:]


def parse_modern_tl_info(data):
    """Modern TrafficLightInfo has NO success bool; never invent one."""
    result = {"success": None, "native_success_field_present": False,
              "tl_id": "", "tl_color": 0, "color_field_present": False}
    for field, wire, value in _fields(data):
        if field == 2:
            if wire != 2:
                raise ProtocolError("TrafficLightInfo.id requires ObjectIdentifier")
            # A singular nested object uses its last string value; duplicate
            # object fields merge, preserving a prior value when absent.
            for nested_field, nested_wire, nested_value in _fields(value):
                if nested_field == 1:
                    if nested_wire != 2:
                        raise ProtocolError("ObjectIdentifier.value requires string")
                    try:
                        result["tl_id"] = nested_value.decode("utf-8")
                    except UnicodeDecodeError as error:
                        raise ProtocolError("ObjectIdentifier.value is not UTF-8") from error
        elif field == 3:
            if wire != 0:
                raise ProtocolError("TrafficLightInfo.color requires enum varint")
            signed = value & 0xffffffff
            result["tl_color"] = signed - 2**32 if signed >= 2**31 else signed
            result["color_field_present"] = True
    result["tl_color_name"] = ("UNSPECIFIED" if result["tl_color"] == 0
                               else COLORS.get(result["tl_color"], "UNKNOWN_ENUM_VALUE"))
    result["color_enum_known"] = result["tl_color"] in COLORS
    result["color_usable_for_gt"] = (result["color_field_present"]
                                     and result["color_enum_known"]
                                     and result["tl_color"] > 0)
    return result


def validate_endpoint(endpoint):
    """No DNS resolution or remote targets: accept numeric IPv4 loopback only."""
    if not isinstance(endpoint, str) or endpoint.count(":") != 1:
        raise ValueError("endpoint must be numeric IPv4 loopback:port")
    host, port = endpoint.split(":")
    try:
        address = ipaddress.IPv4Address(host)
    except ipaddress.AddressValueError as error:
        raise ValueError("endpoint must use numeric IPv4 loopback") from error
    if not address.is_loopback or not port.isascii() or not port.isdecimal():
        raise ValueError("only numeric IPv4 loopback endpoints are allowed")
    if not 1 <= int(port) <= 65535:
        raise ValueError("endpoint port must be between 1 and 65535")
    return endpoint


def validate_timeout(value):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= 30:
        raise ValueError("timeout must be finite and in (0, 30] seconds")
    return value


def source_provenance(protocol="22r3"):
    if protocol == "modern":
        return {
            "source": MODERN_SOURCE, "inspected_sha256": MODERN_SOURCE_SHA256,
            "inspection": "Read-only CArchive/PYZ Python3.8 code and protobuf descriptors",
            "contract": {
                "request": "GetTrafficLightInfoParam(type:enum=1=2/BY_TL_ID, value:string=2)",
                "response": "TrafficLightInfo(id:ObjectIdentifier=2, color:enum=3)",
                "identifier": "ObjectIdentifier.value:string=1",
                "native_success_field_present": False,
            },
            "actual_server_compatibility": "REQUIRES_LIVE_OWNER_VALIDATION",
            "limitation": "Only an exact native ID and explicit known positive color qualify as readback; visible head identity and render remain separate checks.",
        }
    if protocol != "22r3":
        raise ValueError("unsupported protocol")
    root = Path(__file__).resolve().parents[2]
    artifacts = []
    for filename in SDK_FILES:
        relative = SDK_DIRECTORY + "/" + filename
        path = root / relative
        digest = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        artifacts.append({
            "path": relative,
            "present": path.is_file(),
            "sha256": digest,
            "inspected_sha256": INSPECTED_SDK_SHA256[filename],
            "matches_inspected_source": digest == INSPECTED_SDK_SHA256[filename],
        })
    return {
        "source": "local official MORAI OpenSCENARIO API 22.R3 serialized descriptors",
        "sdk_artifacts": artifacts,
        "contract": {
            "request": "TargetUniqueId.unique_id: string field 1",
            "response": "TLInfo(success:bool=1, tl_id:string=2, tl_color:enum=3)",
            "list_response": "TLInfoList.res_list: repeated TLInfo field 1",
        },
        "actual_server_compatibility": "UNVERIFIED_22R3_SCHEMA_AGAINST_ACTUAL_SERVER",
        "limitation": "A decoded response is not proof of target-head identity or a frozen phase.",
    }


def query_states(queries, rpc_call, protocol="modern"):
    """The injected transport can call only this protocol's fixed read methods."""
    if protocol not in ("modern", "22r3"):
        raise ValueError("unsupported protocol")
    methods = MODERN_READ_METHODS if protocol == "modern" else READ_METHODS
    service = MODERN_SERVICE if protocol == "modern" else SERVICE
    # Validate every query before any read, so a mixed incompatible batch fails
    # without partially querying the simulator.
    for kind, unique_id in queries:
        if kind not in methods:
            raise ValueError("query kind {} unsupported by {}; use --protocol 22r3 for intersections".format(kind, protocol))
        encode_target(unique_id)
    results = []
    for kind, unique_id in queries:
        method = methods[kind]
        request = encode_modern_target(unique_id) if protocol == "modern" else encode_target(unique_id)
        response = rpc_call("/" + service + "/" + method, request)
        if protocol == "modern":
            observations = [parse_modern_tl_info(response)]
            observation = observations[0]
            observation["identity_matches_request"] = observation["tl_id"] == unique_id
            readback_success = (observation["identity_matches_request"]
                                and observation["color_usable_for_gt"])
        else:
            observations = ([parse_tl_info(response)] if kind == "traffic_light"
                            else parse_tl_info_list(response))
            readback_success = bool(observations) and all(item["success"] for item in observations)
        results.append({
            "protocol": protocol, "method": method, "requested_id": unique_id,
            "received_at_utc": datetime.now(timezone.utc).isoformat(),
            "response_bytes": len(response), "response_hex": response.hex(),
            "observations": observations,
            "readback_success": readback_success,
        })
    return results


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", choices=("modern", "22r3"), default="modern",
                        help="Default modern; opt into historical 22r3 explicitly")
    parser.add_argument("--traffic-light-id", action="append", default=[],
                        help="Native MORAI ID, not a Lanelet group ID; repeatable")
    parser.add_argument("--intersection-id", action="append", default=[],
                        help="Native intersection ID such as IntTL9; repeatable")
    parser.add_argument("--endpoint", default="127.0.0.1:7789")
    parser.add_argument("--timeout", type=float, default=3.0, help="Per-RPC timeout, default 3 seconds")
    return parser


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    queries = ([("traffic_light", value) for value in args.traffic_light_id]
               + [("intersection", value) for value in args.intersection_id])
    if not queries:
        parser.error("at least one --traffic-light-id or --intersection-id is required")
    if args.protocol == "modern" and args.intersection_id:
        parser.error("modern supports individual --traffic-light-id only; intersections require --protocol 22r3")
    try:
        endpoint = validate_endpoint(args.endpoint)
        timeout = validate_timeout(args.timeout)
        for _, unique_id in queries:
            encode_target(unique_id)
    except (ValueError, UnicodeError) as error:
        parser.error(str(error))
    output = {
        "tool": "probe_morai_traffic_light_gt", "read_only": True,
        "endpoint": endpoint, "timeout_sec": timeout,
        "protocol": args.protocol, "source_provenance": source_provenance(args.protocol),
    }
    try:
        import grpc
    except ImportError:
        output["error"] = "grpcio is required in the selected interpreter; nothing was queried"
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 2
    try:
        with grpc.insecure_channel(endpoint, options=[
                ("grpc.max_receive_message_length", MAX_RESPONSE_BYTES)]) as channel:
            def call(path, request):
                return channel.unary_unary(
                    path, request_serializer=lambda value: value,
                    response_deserializer=lambda value: value,
                )(request, timeout=timeout)
            output["queries"] = query_states(queries, call, args.protocol)
    except (grpc.RpcError, ProtocolError) as error:
        output["error"] = str(error)
        if isinstance(error, grpc.RpcError):
            output["grpc_status"] = str(error.code())
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 2
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0 if all(item["readback_success"] for item in output["queries"]) else 1


if __name__ == "__main__":
    sys.exit(main())
