"""캠페인 YAML, 시나리오와 목적지를 실행 전에 검증한다."""
import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml
from .layout import normalize_bar_setting


ROOT = Path(__file__).resolve().parents[6]
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
    return (path if path.is_absolute() else ROOT / path).resolve()


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


def _load_case(source, default):
    sidecar = source.with_suffix(".case.yaml")
    override = _yaml(sidecar) if sidecar.is_file() else {}
    unknown = set(override) - {"description", "goal", "waypoints", "timeout_sec", "arrival_mode"}
    if unknown:
        raise ConfigError("{}의 알 수 없는 필드: {}".format(sidecar, sorted(unknown)))
    raw = dict(default)
    raw.update(override)
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
    path = Path(path).expanduser().resolve()
    document = _yaml(path)
    if document.get("schema_version") != 1:
        raise ConfigError("schema_version은 1이어야 합니다")
    name = str(document.get("campaign_name", ""))
    if not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{1,59}", name):
        raise ConfigError("campaign_name은 영문/숫자로 시작하는 2~60자 ASCII 이름이어야 합니다")
    scenarios = document.get("scenarios", {})
    scenario_dir = resolve_path(scenarios.get("directory"))
    if not scenario_dir.is_dir():
        raise ConfigError("시나리오 폴더가 없습니다: {}".format(scenario_dir))
    pattern = str(scenarios.get("glob", "*.json"))
    if "/" in pattern or ".." in pattern:
        raise ConfigError("scenarios.glob은 현재 폴더의 파일 패턴만 지원합니다")
    sources = sorted(scenario_dir.glob(pattern))
    if not sources:
        raise ConfigError("시나리오 JSON이 없습니다: {} ({})".format(scenario_dir, pattern))
    default = document.get("default_case", {})
    cases = [_load_case(source, default) for source in sources]
    if len({case.install_stem for case in cases}) != len(cases):
        raise ConfigError("시나리오 설치 이름이 충돌합니다")
    expected_map = document.get("morai", {}).get("map_name", "R_KR_PR_K-city_2025")
    for case in cases:
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
    if not isinstance(stack.get("traffic_light", False), bool):
        raise ConfigError("stack.traffic_light는 true/false입니다")
    if stack["detector"] == "bevfusion" and (
            network["camera_count"] < 4 or not network["has_lidar"]):
        raise ConfigError("BEVFusion에는 카메라 4대와 LiDAR가 있는 sensor JSON이 필요합니다")
    if stack.get("traffic_light", False) and network["camera_count"] < 1:
        raise ConfigError("신호등 인지에는 전방 카메라가 있는 sensor JSON이 필요합니다")
    recording = document.get("recording", {})
    profile = resolve_path(recording.get("topic_profile"))
    try:
        topic_list = json.loads(profile.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ConfigError("토픽 프로파일을 읽을 수 없습니다: {}".format(profile)) from error
    if not isinstance(topic_list, list) or not topic_list or any(
            not isinstance(t, str) or not t.startswith("/") for t in topic_list):
        raise ConfigError("토픽 프로파일은 절대 토픽명 JSON 배열이어야 합니다")
    if recording.get("include_sensors", False):
        topic_list += ["/sensor/lidar1"] + [
            "/sensor/camera/{}/compressed".format(camera)
            for camera in ("front", "front_left", "front_right", "back")]
    recording["topics"] = list(dict.fromkeys(topic_list))
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
            "network": network, "stack": stack, "recording": recording,
            "output": output, "install": install, "upload": upload}
