"""캠페인 YAML, 시나리오와 목적지를 실행 전에 검증한다."""
import hashlib
import json
import math
import re
from dataclasses import dataclass, field, replace
from pathlib import Path

import yaml
from .layout import normalize_bar_setting
from .traffic_light import runtime_settings
from .repetitions import repetition_settings
from .models import selected_model
from .route_profile import normalize as normalize_route
from .localization_contract import (normalize_localization_launch_args,
                                    normalize_gt_pose_launch_args,
                                    normalize_localization_safety)


ROOT = Path(__file__).resolve().parents[6]
CONFIG_SCHEMA_VERSION = 2
EXECUTION_CONTRACT = "campaign-common-settings-v2"
UPLOAD_PROFILES = ("basic", "bag", "video", "full")
DEFAULT_UPLOAD_REMOTE = "ajou_drive:planning_validator"
# Same shared AjouNice2026 Drive folder used by scripts/map/*_kcity_map_assets.sh.
AJOU_DRIVE_ROOT_FOLDER_ID = "12T1IuT2ZyIYG5IkMDvsPrN1zzf6O43IP"


class ConfigError(ValueError):
    pass


def resolve_path(value, *, required=True):
    if not value:
        if required:
            raise ConfigError("필수 경로가 비어 있습니다")
        return None
    path = Path(str(value)).expanduser()
    path = (path if path.is_absolute() else ROOT / path).resolve()
    config_root = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config"
    # Keep legacy personal YAML bytes/hashes intact after public assets move.
    if not path.exists():
        for old_dir, new_dir in (("generated", "internal/generated"),
                                 ("topic_profiles", "internal/topic_profiles")):
            try:
                tail = path.relative_to(config_root / old_dir)
            except ValueError:
                continue
            migrated = config_root / new_dir / tail
            if migrated.exists():
                return migrated.resolve()
        if path.parent == config_root / "examples" and path.suffix == ".yaml":
            family = "gps_shadow" if path.name.startswith("static_obstacle_") else "static_obstacle"
            migrated = config_root / "missions" / family / "base" / path.name
            if migrated.exists():
                return migrated.resolve()
    # Read-only compatibility for already saved campaigns; never rewrite their
    # YAML bytes/resume hash just because shared starter files moved.
    old = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/missions"
    try:
        parts = path.relative_to(old).parts
    except ValueError:
        return path
    removed = not path.exists() or (path.is_dir() and not any(path.iterdir()))
    if removed and len(parts) >= 2 and parts[0] in (
            "highway", "roundabout", "static_obstacle", "dynamic_obstacle", "gps_shadow"):
        migrated = old.parents[1] / "missions" / parts[0] / "base" / Path(*parts[1:])
        if migrated.exists():
            return migrated.resolve()
    return path


def positive(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ConfigError("{}은 양수여야 합니다".format(label)) from error
    if not math.isfinite(number) or number <= 0:
        raise ConfigError("{}은 양수여야 합니다".format(label))
    return number


def _pose(value, label):
    if not isinstance(value, dict) or not {"x", "y"} <= set(value):
        raise ConfigError("{}에 x, y가 필요합니다".format(label))
    result = {}
    for key in ("x", "y", "z", "yaw_deg"):
        try:
            result[key] = float(value.get(key, 0.0))
        except (TypeError, ValueError) as error:
            raise ConfigError("{}.{} 값이 숫자가 아닙니다".format(label, key)) from error
        if not math.isfinite(result[key]):
            raise ConfigError("{}.{} 값이 유한하지 않습니다".format(label, key))
    return result


def _yaml(path):
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError("YAML을 읽을 수 없습니다: {}".format(path)) from error
    if not isinstance(value, dict):
        raise ConfigError("YAML 최상위는 mapping이어야 합니다: {}".format(path))
    return value


def prediction_mixed_horizon_enabled(stack):
    """One explicit experimental switch; never enable it through truthiness."""
    value = stack.get('prediction_mixed_horizon_enabled', False)
    if type(value) is not bool:
        raise ConfigError('stack.prediction_mixed_horizon_enabled는 true/false여야 합니다')
    return value


def prediction_horizon(stack):
    """Bound the explicit native prediction horizon on the existing 0.1 s grid."""
    value = stack.get('prediction_horizon', 3.0)
    label = 'stack.prediction_horizon'
    if type(value) not in (int, float):
        raise ConfigError('{}은 0.1~10초의 숫자여야 합니다'.format(label))
    try:
        horizon = float(value)
    except OverflowError as error:
        raise ConfigError('{}은 0.1~10초의 유한한 숫자여야 합니다'.format(label)) from error
    if not math.isfinite(horizon) or not 0.1 <= horizon <= 10.0:
        raise ConfigError('{}은 0.1~10초의 유한한 숫자여야 합니다'.format(label))
    if not math.isclose(horizon * 10.0, round(horizon * 10.0), rel_tol=0.0, abs_tol=1e-8):
        raise ConfigError('{}은 0.1초 단위여야 합니다'.format(label))
    return horizon


def validate_campaign_document(document, *, allow_legacy=False):
    """Structural checks for GUI opening too; no ROS/filesystem prerequisites."""
    if not isinstance(document, dict):
        raise ConfigError("실행용 캠페인 YAML이어야 합니다")
    version = document.get("schema_version")
    if version != CONFIG_SCHEMA_VERSION and not (allow_legacy and version == 1):
        raise ConfigError("schema_version: 2인 실행용 YAML이 필요합니다. 이전 YAML은 GUI에서 열어 새 캠페인으로 저장하세요")
    required = {"campaign_name", "scenarios", "default_case", "stack", "recording", "network"}
    missing = required - set(document)
    if missing:
        raise ConfigError("실행용 YAML의 필수 항목이 없습니다: {}. 장면별 .case.yaml은 실행 파일이 아닙니다".format(
            ", ".join(sorted(missing))))
    if not isinstance(document["campaign_name"], str) or not document["campaign_name"].strip():
        raise ConfigError("campaign_name이 비어 있습니다")
    for key in required - {"campaign_name"}:
        if not isinstance(document[key], dict):
            raise ConfigError("{}은 YAML mapping이어야 합니다".format(key))
    prediction_mixed_horizon_enabled(document['stack'])
    prediction_horizon(document['stack'])
    try:
        selected_model(document['stack'])
        normalize_localization_launch_args(document['stack'], check_paths=False)
        normalize_gt_pose_launch_args(document['stack'], check_paths=False)
        normalize_localization_safety(document)
    except ValueError as error:
        raise ConfigError(str(error)) from error
    for key in ("directory", "glob"):
        if not isinstance(document["scenarios"].get(key), str) or not document["scenarios"][key].strip():
            raise ConfigError("scenarios.{}이 비어 있습니다".format(key))
    validate_scenario_include(document["scenarios"])
    try:
        repetition_settings(document["scenarios"])
    except ValueError as error:
        raise ConfigError(str(error)) from error
    common = document["default_case"]
    if set(common) - {"description", "goal", "waypoints", "timeout_sec", "arrival_mode"}:
        raise ConfigError("default_case에는 공통 목적지·경유점·시간·도착 방식만 설정합니다")
    _pose(common.get("goal"), "default_case.goal")
    positive(common.get("timeout_sec"), "default_case.timeout_sec")
    if not isinstance(common.get("waypoints", []), list):
        raise ConfigError("default_case.waypoints는 공통 경유점 목록이어야 합니다")
    for point in common.get("waypoints", []):
        _pose(point, "default_case.waypoints")
    if common.get("arrival_mode", "stop") not in ("stop", "pass_through"):
        raise ConfigError("default_case.arrival_mode는 stop 또는 pass_through입니다")
    if set(document) & {"cases", "case_overrides", "overrides"}:
        raise ConfigError("장면별 설정 덮어쓰기는 지원하지 않습니다. 목적지나 실행 조건이 다르면 별도 YAML을 만드세요")


def validate_scenario_include(scenarios):
    """Absent include means glob; an empty/broken selection must never run all."""
    if "include" not in scenarios:
        return
    names = scenarios["include"]
    if not isinstance(names, list) or not names:
        raise ConfigError("scenarios.include는 선택한 JSON 파일명 목록이어야 합니다 (최소 1개)")
    for name in names:
        if (not isinstance(name, str) or not name.strip() or "/" in name or "\\" in name
                or Path(name).suffix != ".json" or Path(name).name != name):
            raise ConfigError("scenarios.include에는 현재 폴더의 .json 파일명만 넣으세요")
    if len(set(names)) != len(names):
        raise ConfigError("scenarios.include에 중복 파일명이 있습니다")


def scenario_sources(scenarios):
    """Shared CLI/GUI selection contract; explicit filenames override glob."""
    validate_scenario_include(scenarios)
    directory = resolve_path(scenarios.get("directory"))
    if not directory.is_dir():
        raise ConfigError("시나리오 폴더가 없습니다: {}".format(directory))
    if "include" in scenarios:
        sources = [directory / name for name in sorted(scenarios["include"])]
        for source in sources:
            if not source.is_file() or source.resolve().parent != directory:
                raise ConfigError("선택한 시나리오가 없거나 폴더 밖입니다: {}".format(source.name))
    else:
        pattern = str(scenarios.get("glob", "*.json"))
        if "/" in pattern or "\\" in pattern or ".." in pattern:
            raise ConfigError("scenarios.glob은 현재 폴더의 파일 패턴만 지원합니다")
        sources = sorted(source for source in directory.glob(pattern)
                         if source.is_file() and source.suffix.lower() == ".json")
        if any(source.resolve().parent != directory for source in sources):
            raise ConfigError("시나리오가 선택한 폴더 밖을 가리킵니다")
    if not sources:
        raise ConfigError("시나리오 JSON이 없습니다: {} ({})".format(directory, scenarios.get("glob")))
    return sources


def install_directory(value):
    if value != "auto":
        path = resolve_path(value)
        if not path.is_dir():
            raise ConfigError("MORAI scenario 설치 폴더가 없습니다: {}".format(path))
        return path
    official = Path.home() / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
    if official.is_dir():
        return official.resolve()
    candidates = sorted(Path.home().glob(
        "MoraiLauncher_Lin*/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"))
    if len(candidates) != 1:
        raise ConfigError("MORAI 설치 폴더 자동 탐색 결과 {}개. morai.scenario_install_dir를 지정하세요".format(len(candidates)))
    return candidates[0].resolve()


@dataclass(frozen=True)
class Case:
    source: Path
    name: str
    install_stem: str
    source_sha256: str
    description: str
    goal: dict
    waypoints: list
    timeout_sec: float
    arrival_mode: str
    repeat_index: int = 1
    repeat_total: int = 1
    randomization: dict = field(default_factory=dict)


def _load_case(source, default):
    raw = dict(default)
    sha = hashlib.sha256(source.read_bytes()).hexdigest()
    clean = re.sub(r"[^a-zA-Z0-9_]+", "_", source.stem).strip("_")[:15]
    stem = "pv_{}_{}".format(clean, sha[:8])
    if len(stem.encode("ascii")) > 30:
        raise ConfigError("ScenarioLoad 30-byte 한도를 초과합니다: {}".format(stem))
    goal = _pose(raw.get("goal"), "{}.goal".format(source.name))
    waypoints = [_pose(p, "{}.waypoints".format(source.name)) for p in raw.get("waypoints", [])]
    mode = raw.get("arrival_mode", "stop")
    if mode not in ("stop", "pass_through"):
        raise ConfigError("arrival_mode는 stop 또는 pass_through입니다: {}".format(source))
    return Case(source, source.stem, stem, sha, str(raw.get("description", "")),
                goal, waypoints, positive(raw.get("timeout_sec"), "timeout_sec"), mode)


def load_campaign(path, *, check_install=True):
    path = resolve_path(path)
    if path.name.endswith(".case.yaml"):
        raise ConfigError(".case.yaml은 폐기된 보조 설정입니다. base/example.yaml 또는 본인의 캠페인 YAML을 선택하세요")
    document = _yaml(path)
    validate_campaign_document(document)
    try:
        route = normalize_route(document, resolve_path)
    except (ValueError, KeyError, OSError, TypeError, yaml.YAMLError) as error:
        raise ConfigError('route 설정: {}'.format(error)) from error
    name = str(document.get("campaign_name", ""))
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{1,59}", name):
        raise ConfigError("campaign_name은 영문/숫자로 시작하는 2~60자 ASCII 이름이어야 합니다")
    scenarios = document.get("scenarios", {})
    sources = scenario_sources(scenarios)
    default = document.get("default_case", {})
    originals = [_load_case(source, default) for source in sources]
    if len({case.install_stem for case in originals}) != len(originals):
        raise ConfigError("시나리오 설치 이름이 충돌합니다")
    repeat, randomization = repetition_settings(scenarios)
    if repeat * len(originals) > 10000:
        raise ConfigError("한 YAML의 총 실행은 10000회 이하로 나누세요")
    # Round-robin order: A1/B1/A2/B2, rather than exhausting only A first.
    cases = [replace(case, name=(case.name + "_r{:03d}".format(index) if repeat > 1 else case.name),
                     repeat_index=index, repeat_total=repeat, randomization=randomization)
             for index in range(1, repeat + 1) for case in originals]
    expected_map = document.get("morai", {}).get("map_name", "R_KR_PR_K-city_2025")
    if len({case.name for case in cases}) != len(cases):
        raise ConfigError("반복 시나리오 실행 이름이 충돌합니다")
    for case in originals:
        try:
            scene = json.loads(case.source.read_text(encoding="utf-8"))
            actual_map = scene["mapInfo"]["mapName"]
            scene["egoVehicle"]["initPosition"]["pos"]
        except (OSError, json.JSONDecodeError, KeyError, TypeError) as error:
            raise ConfigError("시나리오 JSON 형식이 잘못됐습니다: {}".format(case.source)) from error
        if actual_map != expected_map:
            raise ConfigError("지도 이름 불일치: {} ({})".format(case.source, actual_map))
    network = document.get("network", {})
    for key in ("sensor_json", "ego_network_json"):
        resolved = resolve_path(network.get(key))
        try:
            json.loads(resolved.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ConfigError("네트워크 JSON을 읽을 수 없습니다: {}".format(resolved)) from error
        network[key] = resolved
    ego_network = json.loads(network["ego_network_json"].read_text(encoding="utf-8"))
    sensor_config = json.loads(network["sensor_json"].read_text(encoding="utf-8"))
    network["has_lidar"] = bool(sensor_config.get("Lidar3DList"))
    network["camera_count"] = len(sensor_config.get("cameraList", []))
    try:
        entries = ego_network["egoNetworkData"][0]["listEgoNetworkInfo"]
    except (KeyError, IndexError, TypeError) as error:
        raise ConfigError("Ego Network JSON 구조가 잘못됐습니다") from error
    scenario_entries = [entry for entry in entries
                        if entry.get("PUBSUB_TYPE") == 783
                        and entry.get("netType") == 1
                        and entry.get("config", {}).get("commType") == 1
                        and entry.get("config", {}).get("udpConfig", {}).get("hostPort") == 9095]
    if len(scenario_entries) != 1:
        raise ConfigError("Ego Network에 UDP ScenarioLoad(9095)가 정확히 하나 필요합니다. 먼저 install.sh를 실행하세요")
    if sum(entry.get("PUBSUB_TYPE") == 783 for entry in entries) != 1:
        raise ConfigError("Ego Network의 PUBSUB_TYPE=783 슬롯이 중복됐습니다. install.sh를 다시 실행하세요")
    stack = document.get("stack", {})
    if stack.get("pose_source") not in ("localization", "maptf_morai"):
        raise ConfigError("stack.pose_source는 localization 또는 maptf_morai입니다")
    if stack.get("detector") not in ("od_gt", "bevfusion"):
        raise ConfigError("stack.detector는 od_gt 또는 bevfusion입니다")
    try:
        localization_launch_args = normalize_localization_launch_args(stack, resolve_path)
        gt_pose_launch_args = normalize_gt_pose_launch_args(stack, resolve_path)
        localization_safety = normalize_localization_safety(document)
    except ValueError as error:
        raise ConfigError(str(error)) from error
    if not isinstance(stack.get("traffic_light", False), bool):
        raise ConfigError("stack.traffic_light는 true/false입니다")
    try:
        runtime_settings(stack, ROOT, check_exists=stack.get("traffic_light", False))
    except ValueError as error:
        raise ConfigError(str(error)) from error
    if stack["detector"] == "bevfusion" and (
            network["camera_count"] < 4 or not network["has_lidar"]):
        raise ConfigError("BEVFusion에는 카메라 4대와 LiDAR가 있는 sensor JSON이 필요합니다")
    if stack.get("traffic_light", False) and network["camera_count"] < 1:
        raise ConfigError("신호등 인지에는 전방 카메라가 있는 sensor JSON이 필요합니다")
    recording = document.get("recording", {})
    if recording.get('analysis_mode', False) and recording.get('bag_enabled', True) and (
            not recording.get('include_sensors', False) or not recording.get('video', {}).get('enabled', False)):
        raise ConfigError('인지 분석 모드는 include_sensors: true + video.enabled: true가 필요합니다')
    if "minimum_free_disk_gib" in recording:
        positive(recording["minimum_free_disk_gib"], "recording.minimum_free_disk_gib")
    from .recording_options import effective_topics, topic_patterns, ensure_video_display
    try:
        ensure_video_display(recording)
        recording['topics'] = effective_topics(recording, resolve_path)
        recording['topic_regex'] = topic_patterns(recording)
    except ValueError as error:
        raise ConfigError(str(error)) from error
    video = recording.get("video", {})
    if video.get("enabled", False) and not video.get("display"):
        raise ConfigError("영상 녹화에는 recording.video.display가 필요합니다")
    if video.get("window_layout", "manual") not in ("manual", "split"):
        raise ConfigError("recording.video.window_layout은 manual 또는 split이어야 합니다")
    try:
        video["bottom_bar_px"] = normalize_bar_setting(video.get("bottom_bar_px", "auto"))
    except ValueError as error:
        raise ConfigError("recording.video.bottom_bar_px: {}".format(error)) from error
    upload = document.get("upload", {})
    if not isinstance(upload, dict):
        raise ConfigError("upload는 YAML mapping이어야 합니다")
    if upload.get("profile", "basic") not in UPLOAD_PROFILES:
        raise ConfigError("upload.profile은 basic, bag, video, full 중 하나여야 합니다")
    if upload.get("enabled", False) and not upload.get("rclone_remote"):
        raise ConfigError("Google Drive 업로드에는 upload.rclone_remote가 필요합니다")
    if "drive_root_folder_id" not in upload and str(upload.get("rclone_remote", "")).startswith("ajou_drive:"):
        upload["drive_root_folder_id"] = AJOU_DRIVE_ROOT_FOLDER_ID
    output = resolve_path(document.get("output_root", "planner_logs/validation")) / name
    install = install_directory(document.get("morai", {}).get("scenario_install_dir", "auto")) if check_install else None
    return {"path": path, "document": document, "name": name, "cases": cases,
            "network": network, "stack": stack, "recording": recording, "route": route,
            "output": output, "install": install, "upload": upload,
            "localization_launch_args": localization_launch_args,
            "gt_pose_launch_args": gt_pose_launch_args,
            "localization_safety": localization_safety,
            "ignored_case_yamls": [str(source.with_suffix(".case.yaml")) for source in sources
                                   if source.with_suffix(".case.yaml").is_file()]}
