# 목적: manifest가 지정한 MORAI scenario JSON을 trajectory 시작 상태에 맞게 생성한다.
# 역할: template의 상대 차량·장애물 구성은 보존하고 Ego 시작 상태와 F4 네트워크 sidecar를 만든다.
# 주요 흐름: manifest 출력 경로 검증 -> pose 역변환 -> 정지/ExternalCtrl 적용 -> scenario와 _MN.json 저장.

"""manifest 기반 MORAI scenario JSON 생성기."""

import json
import math
import os
import tempfile
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml

from trajectory_batch_runner.config_loader import BatchConfig
from trajectory_batch_runner.manifest_loader import ManifestRow
from trajectory_batch_runner.trajectory_input import TrajectoryStart


class ScenarioBuildError(ValueError):
    """scenario template 또는 생성 결과가 올바르지 않을 때 발생한다."""


@dataclass(frozen=True)
class ScenarioResult:
    """생성하거나 재사용한 scenario와 예상 MORAI 시작 pose다."""

    path: Path
    network_path: Path
    morai_x: float
    morai_y: float
    morai_z: float
    morai_yaw_deg: float
    reused_existing: bool


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise ScenarioBuildError("JSON을 읽을 수 없습니다: {}".format(path)) from error
    if not isinstance(value, dict):
        raise ScenarioBuildError("scenario 최상위 값은 object여야 합니다")
    return value


def _load_planar_transform(config: BatchConfig) -> Tuple[Tuple[float, ...], float]:
    path = config.resolve_config_path(
        "paths.files.morai_static_tf_yaml", expect_directory=False
    )
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        transform = document["map_to_morai_map"]
        xyz = tuple(float(value) for value in transform["xyz"])
        rpy = tuple(float(value) for value in transform["rpy"])
    except (KeyError, TypeError, ValueError) as error:
        raise ScenarioBuildError(
            "morai static TF의 xyz/rpy 형식이 올바르지 않습니다"
        ) from error
    if len(xyz) != 3 or len(rpy) != 3:
        raise ScenarioBuildError("morai static TF의 xyz/rpy는 각각 3개 값이어야 합니다")
    if any(not math.isfinite(value) for value in xyz + rpy):
        raise ScenarioBuildError("morai static TF 값은 유한해야 합니다")
    if abs(rpy[0]) > 1.0e-12 or abs(rpy[1]) > 1.0e-12:
        raise ScenarioBuildError("현재 scenario 생성기는 yaw-only static TF만 지원합니다")
    return xyz, rpy[2]


def _inverse_map_pose(
    start: TrajectoryStart,
    translation: Tuple[float, ...],
    tf_yaw: float,
    vehicle_z_offset_m: float,
) -> Tuple[float, float, float, float]:
    dx = start.x - translation[0]
    dy = start.y - translation[1]
    cosine = math.cos(tf_yaw)
    sine = math.sin(tf_yaw)
    morai_x = cosine * dx + sine * dy
    morai_y = -sine * dx + cosine * dy
    morai_z = start.z - translation[2] + vehicle_z_offset_m
    morai_yaw_deg = math.degrees(start.yaw_rad - tf_yaw) % 360.0
    return morai_x, morai_y, morai_z, morai_yaw_deg


def _apply_initial_state(
    scenario: Dict[str, Any], speed_kph: float, morai_yaw_deg: float, gear: int
) -> None:
    try:
        ego = scenario["egoVehicle"]
        initial_position = ego["initPosition"]
        save_data = initial_position["vehicleSaveData"]
        input_data = save_data["inputData"]
    except (KeyError, TypeError) as error:
        raise ScenarioBuildError(
            "scenario template에 Ego 저장 상태 필드가 부족합니다"
        ) from error
    if not isinstance(input_data, list) or len(input_data) <= 6:
        raise ScenarioBuildError("vehicleSaveData.inputData에 gear 필드가 없습니다")

    speed_mps = speed_kph / 3.6
    yaw_rad = math.radians(morai_yaw_deg)
    velocity = {
        "x": -speed_mps * math.cos(yaw_rad),
        "y": 0.0,
        "z": -speed_mps * math.sin(yaw_rad),
    }
    ego["currentVelocity"] = speed_kph
    ego["customValue"] = speed_kph
    # ScenarioLoad must preserve AV-ExternalCtrl instead of restoring the
    # template's AV-MORAI Built-In control mode.
    ego["beforeControlMode"] = 2
    ego["currentControlMode"] = 2
    initial_position["gear"] = gear
    save_data["velocity"] = speed_kph
    input_data[6] = gear
    try:
        save_data["vehicle"]["lastVelocity"].update(velocity)
        save_data["lastVelocity"].update(velocity)
    except (KeyError, AttributeError, TypeError) as error:
        raise ScenarioBuildError(
            "scenario template의 lastVelocity 형식이 올바르지 않습니다"
        ) from error


def _set_pose(
    scenario: Dict[str, Any],
    morai_x: float,
    morai_y: float,
    morai_z: float,
    morai_yaw_deg: float,
) -> None:
    try:
        position = scenario["egoVehicle"]["initPosition"]["pos"]
        rotation = scenario["egoVehicle"]["initPosition"]["rot"]
    except (KeyError, TypeError) as error:
        raise ScenarioBuildError("scenario template에 Ego pose가 없습니다") from error
    position.update(
        {
            "x": morai_x,
            "y": morai_y,
            "z": morai_z,
            "_x": "{:.3f}".format(morai_x),
            "_y": "{:.3f}".format(morai_y),
            "_z": "{:.3f}".format(morai_z),
        }
    )
    rotation["yaw"] = "{:.6f}".format(morai_yaw_deg)


def _verify_scenario(
    scenario: Dict[str, Any],
    config: BatchConfig,
    row: ManifestRow,
    pose: Tuple[float, float, float, float],
) -> None:
    expected_map = config.get("morai.expected_map_name")
    try:
        actual_map = scenario["mapInfo"]["mapName"]
        ego = scenario["egoVehicle"]
        position = ego["initPosition"]["pos"]
        yaw_deg = float(ego["initPosition"]["rot"]["yaw"])
        speeds = (
            ego["currentVelocity"],
            ego["customValue"],
            ego["initPosition"]["vehicleSaveData"]["velocity"],
        )
        current_control_mode = int(ego["currentControlMode"])
        gear = int(ego["initPosition"]["gear"])
    except (KeyError, TypeError) as error:
        raise ScenarioBuildError("생성 scenario의 필수 필드가 부족합니다") from error
    if actual_map != expected_map:
        raise ScenarioBuildError(
            "scenario mapName={}가 config의 {}와 다릅니다".format(
                actual_map, expected_map
            )
        )
    for key, expected in zip(("x", "y", "z"), pose[:3]):
        if not math.isclose(
            float(position[key]), expected, rel_tol=0.0, abs_tol=1.0e-6
        ):
            raise ScenarioBuildError(
                "기존 scenario의 Ego {}가 trajectory 시작 pose와 다릅니다".format(key)
            )
    yaw_error = abs((yaw_deg - pose[3] + 180.0) % 360.0 - 180.0)
    if yaw_error > 1.0e-6:
        raise ScenarioBuildError(
            "기존 scenario의 Ego yaw가 trajectory 시작 yaw와 다릅니다"
        )
    initial_speed_kph = float(config.get("scenario_generation.initial_speed_kph"))
    for actual in speeds:
        if not math.isclose(
            float(actual), initial_speed_kph, rel_tol=0.0, abs_tol=1.0e-6
        ):
            raise ScenarioBuildError(
                "기존 scenario의 Ego 초기 속도가 scenario_generation.initial_speed_kph와 다릅니다"
            )
    if current_control_mode != 2:
        raise ScenarioBuildError("기존 scenario의 Ego가 AV-ExternalCtrl 모드가 아닙니다")
    expected_gear = int(config.get("scenario_generation.initial_gear"))
    if gear != expected_gear:
        raise ScenarioBuildError("기존 scenario의 Ego 초기 gear가 설정과 다릅니다")


def _atomic_write_json(path: Path, document: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _scenario_output_path(
    config: BatchConfig, row: ManifestRow
) -> Path:
    """manifest의 scenario_file을 생성 및 MORAI 로드 경로로 해석한다."""

    filename = row.scenario_file
    if Path(filename).name != filename or Path(filename).suffix.lower() != ".json":
        raise ScenarioBuildError(
            "manifest scenario_file은 폴더를 포함하지 않는 .json 파일명이어야 합니다: "
            "{}".format(filename)
        )

    max_filename_bytes = int(
        config.get("morai.scenario_load_udp.max_filename_bytes")
    )
    filename_value = str(config.get("morai.scenario_load_udp.filename_value"))
    wire_filename = Path(filename).stem if filename_value == "stem" else filename
    try:
        filename_bytes = wire_filename.encode("ascii")
    except UnicodeEncodeError as error:
        raise ScenarioBuildError(
            "MORAI에 전송할 scenario 이름은 ASCII여야 합니다: {}".format(
                wire_filename
            )
        ) from error
    if len(filename_bytes) > max_filename_bytes:
        raise ScenarioBuildError(
            "manifest scenario_file이 MORAI {} byte 제한을 넘습니다: {}".format(
                max_filename_bytes, wire_filename
            )
        )
    try:
        scenario_path = config.resolve_child(
            "paths.directories.scenario_directory",
            filename,
            "manifest scenario_file",
            must_exist=False,
        )
    except ValueError as error:
        raise ScenarioBuildError(str(error)) from error
    if scenario_path != row.scenario_path:
        raise ScenarioBuildError(
            "manifest scenario_file 해석 결과가 사전 검증 경로와 다릅니다: {}".format(
                filename
            )
        )
    return scenario_path


def _scenario_network_output_path(config: BatchConfig, row: ManifestRow) -> Path:
    """MORAI가 scenario와 함께 읽는 EgoNetwork/<stem>_MN.json 경로다."""

    filename = "{}_MN.json".format(row.scenario_path.stem)
    try:
        return config.resolve_child(
            "paths.directories.scenario_network_directory",
            filename,
            "scenario EgoNetwork sidecar",
            must_exist=False,
        )
    except ValueError as error:
        raise ScenarioBuildError(str(error)) from error


def _network_template(config: BatchConfig) -> Dict[str, Any]:
    path = config.resolve_config_path(
        "paths.files.scenario_network_template_json", expect_directory=False
    )
    return _load_json(path)


def _verify_network_sidecar(actual: Dict[str, Any], expected: Dict[str, Any]) -> None:
    if actual != expected:
        raise ScenarioBuildError(
            "scenario EgoNetwork sidecar가 scenario_network_template_json과 다릅니다"
        )


def _expected_pose(
    config: BatchConfig, start: TrajectoryStart
) -> Tuple[float, float, float, float]:
    """trajectory 시작 상태를 MORAI scenario 좌표로 변환한다."""

    translation, tf_yaw = _load_planar_transform(config)
    z_offset = float(config.get("scenario_generation.vehicle_z_offset_m"))
    if not math.isfinite(z_offset):
        raise ScenarioBuildError("vehicle_z_offset_m은 유한해야 합니다")
    return _inverse_map_pose(start, translation, tf_yaw, z_offset)


def load_scenario(
    config: BatchConfig, row: ManifestRow, start: TrajectoryStart
) -> ScenarioResult:
    """manifest가 지정한 기존 scenario를 검증하고 로드 결과로 반환한다."""

    scenario_path = _scenario_output_path(config, row)
    network_path = _scenario_network_output_path(config, row)
    if not scenario_path.is_file():
        raise ScenarioBuildError(
            "manifest scenario_file이 없습니다: {}. "
            "scenario_json_generator.py로 먼저 생성하거나 파일을 준비하세요".format(
                scenario_path
            )
        )
    pose = _expected_pose(config, start)
    scenario = _load_json(scenario_path)
    _verify_scenario(scenario, config, row, pose)
    if not network_path.is_file():
        raise ScenarioBuildError(
            "scenario EgoNetwork sidecar가 없습니다: {}".format(network_path)
        )
    _verify_network_sidecar(_load_json(network_path), _network_template(config))
    return ScenarioResult(scenario_path, network_path, *pose, reused_existing=True)


def ensure_scenario(
    config: BatchConfig, row: ManifestRow, start: TrajectoryStart
) -> ScenarioResult:
    """manifest 행의 scenario를 생성하거나 기존 파일의 일치 여부를 확인한다."""

    template_path = config.resolve_config_path(
        "paths.files.scenario_template_json", expect_directory=False
    )
    template = _load_json(template_path)
    pose = _expected_pose(config, start)
    scenario_path = _scenario_output_path(config, row)
    network_path = _scenario_network_output_path(config, row)
    network = _network_template(config)

    scenario = deepcopy(template)
    _set_pose(scenario, *pose)
    initial_speed_kph = float(config.get("scenario_generation.initial_speed_kph"))
    initial_gear = int(config.get("scenario_generation.initial_gear"))
    _apply_initial_state(scenario, initial_speed_kph, pose[3], initial_gear)
    _verify_scenario(scenario, config, row, pose)

    reused_scenario = False
    if scenario_path.exists():
        existing = _load_json(scenario_path)
        try:
            _verify_scenario(existing, config, row, pose)
        except ScenarioBuildError:
            if not bool(config.get("scenario_generation.overwrite_existing")):
                raise ScenarioBuildError(
                    "기존 scenario가 manifest와 다르며 overwrite_existing=false입니다: "
                    "{}".format(scenario_path)
                )
        else:
            reused_scenario = True

    reused_network = False
    if network_path.exists():
        try:
            _verify_network_sidecar(_load_json(network_path), network)
        except ScenarioBuildError:
            if not bool(config.get("scenario_generation.overwrite_existing")):
                raise ScenarioBuildError(
                    "기존 EgoNetwork sidecar가 template과 다르며 overwrite_existing=false입니다: "
                    "{}".format(network_path)
                )
        else:
            reused_network = True

    if not reused_scenario:
        _atomic_write_json(scenario_path, scenario)
    if not reused_network:
        _atomic_write_json(network_path, network)
    return ScenarioResult(
        scenario_path,
        network_path,
        *pose,
        reused_existing=reused_scenario and reused_network
    )
