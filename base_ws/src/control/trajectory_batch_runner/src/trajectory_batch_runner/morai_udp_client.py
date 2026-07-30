# 목적: MORAI ScenarioLoad UDP 패킷을 규격에 맞게 만들고 송신한다.
# 역할: simulator 통신을 JSON 생성이나 ROS 프로세스 관리와 분리해 재사용 가능하게 한다.
# 주요 흐름: scenario stem 검증 -> 고정 길이 ctypes packet 작성 -> UDP sendto.

"""MORAI ScenarioLoad UDP client."""

import ctypes
import socket
from pathlib import Path
from typing import Any, Dict, Optional

from trajectory_batch_runner.config_loader import BatchConfig


class MoraiUdpError(RuntimeError):
    """ScenarioLoad UDP 요청을 만들거나 보낼 수 없을 때 발생한다."""


class _ScenarioLoadPacket(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("header", ctypes.c_char * 14),
        ("data_length", ctypes.c_int32),
        ("aux_data", ctypes.c_int32 * 3),
        ("filename", ctypes.c_char * 30),
        ("delete_all", ctypes.c_bool),
        ("network", ctypes.c_bool),
        ("ego", ctypes.c_bool),
        ("npc", ctypes.c_bool),
        ("pedestrian", ctypes.c_bool),
        ("object", ctypes.c_bool),
        ("pause", ctypes.c_bool),
        ("tail", ctypes.c_char * 2),
    ]


REQUEST_KEYS = {
    "delete_all": "delete_all",
    "load_network_connection_data": "network",
    "load_ego_vehicle_data": "ego",
    "load_surrounding_vehicle_data": "npc",
    "load_pedestrian_data": "pedestrian",
    "load_obstacle_data": "object",
    "set_pause": "pause",
}


def build_scenario_load_payload(
    scenario_stem: str,
    request: Dict[str, Any],
    *,
    pause_override: Optional[bool] = None,
    max_filename_bytes: int = 30,
) -> bytes:
    """MORAI NetworkModule 예제와 동일한 ScenarioLoad payload를 만든다."""

    if not isinstance(scenario_stem, str) or not scenario_stem:
        raise MoraiUdpError("scenario filename stem이 비어 있습니다")
    try:
        encoded = scenario_stem.encode("ascii")
    except UnicodeEncodeError as error:
        raise MoraiUdpError("MORAI scenario filename은 ASCII여야 합니다") from error
    if len(encoded) > max_filename_bytes or len(encoded) > 30:
        raise MoraiUdpError(
            "scenario filename이 {} byte 제한을 넘습니다: {}".format(
                min(max_filename_bytes, 30), scenario_stem
            )
        )
    if not isinstance(request, dict):
        raise MoraiUdpError("ScenarioLoad request 설정은 mapping이어야 합니다")
    missing = [key for key in REQUEST_KEYS if key not in request]
    if missing:
        raise MoraiUdpError(
            "ScenarioLoad request 설정이 부족합니다: {}".format(", ".join(missing))
        )

    packet = _ScenarioLoadPacket()
    packet.header = b"#ScenarioLoad$"
    packet.data_length = 37
    packet.aux_data = (ctypes.c_int32 * 3)(0, 0, 0)
    packet.filename = encoded
    for config_key, packet_field in REQUEST_KEYS.items():
        value = request[config_key]
        if not isinstance(value, bool):
            raise MoraiUdpError("{} 값은 bool이어야 합니다".format(config_key))
        setattr(packet, packet_field, value)
    if pause_override is not None:
        if not isinstance(pause_override, bool):
            raise MoraiUdpError("pause_override는 bool이어야 합니다")
        packet.pause = pause_override
    packet.tail = b"\r\n"
    return ctypes.string_at(ctypes.addressof(packet), ctypes.sizeof(packet))


class MoraiScenarioLoadClient:
    """config에 지정된 주소로 ScenarioLoad UDP 요청을 보낸다."""

    def __init__(self, config: BatchConfig):
        block = config.get("morai.scenario_load_udp")
        try:
            self.host = str(block["host"])
            self.port = int(block["port"])
            self.max_filename_bytes = int(block["max_filename_bytes"])
            self.request = block["request"]
        except (KeyError, TypeError, ValueError) as error:
            raise MoraiUdpError("scenario_load_udp 설정이 올바르지 않습니다") from error
        if not self.host or not 1 <= self.port <= 65535:
            raise MoraiUdpError("ScenarioLoad UDP host/port가 올바르지 않습니다")
        if not 1 <= self.max_filename_bytes <= 30:
            raise MoraiUdpError("max_filename_bytes는 1~30이어야 합니다")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, scenario_path: Path, *, pause: Optional[bool] = None) -> int:
        """scenario 파일 stem으로 ScenarioLoad 요청을 한 번 송신한다."""

        payload = build_scenario_load_payload(
            scenario_path.stem,
            self.request,
            pause_override=pause,
            max_filename_bytes=self.max_filename_bytes,
        )
        try:
            sent = self.socket.sendto(payload, (self.host, self.port))
        except OSError as error:
            raise MoraiUdpError(
                "ScenarioLoad UDP 송신에 실패했습니다: {}:{}".format(
                    self.host, self.port
                )
            ) from error
        if sent != len(payload):
            raise MoraiUdpError(
                "ScenarioLoad UDP payload 일부만 송신됐습니다: {}/{}".format(
                    sent, len(payload)
                )
            )
        return sent

    def close(self) -> None:
        self.socket.close()

    def __enter__(self) -> "MoraiScenarioLoadClient":
        return self

    def __exit__(self, _type: Any, _value: Any, _traceback: Any) -> None:
        self.close()
