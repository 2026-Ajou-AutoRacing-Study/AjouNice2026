#!/usr/bin/env python3
"""Import the downloaded static_final archive as an OD-GT validator campaign.

The downloaded ZIP remains immutable.  This tool validates all 200 MORAI
scenes, fills the common ego MGeo link in derived workspace copies, creates a
same-pose clear baseline for a bounded smoke set, and writes complete schema-v2
OD-GT campaign YAML files.

Weather words in the filenames are preserved only as source labels: the MORAI
JSON files do not contain a weather field, so this tool never claims or applies
Sunny/Cloudy/Foggy runtime weather.
"""

import argparse
import copy
import hashlib
import json
import math
import os
import re
import zipfile
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[5]
MISSION = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle"
DEFAULT_ARCHIVE = Path.home() / "Downloads/static_final-20261028T114117Z-1-001.zip"
DEFAULT_OUTPUT = MISSION / "workspace/static_final_od_gt/scenarios"
DEFAULT_SMOKE_OUTPUT = MISSION / "workspace/static_final_od_gt/smoke_scenarios"
DEFAULT_CAMPAIGN = MISSION / "workspace/static_final_od_gt_all.yaml"
DEFAULT_SMOKE_CAMPAIGN = MISSION / "workspace/static_final_od_gt_smoke.yaml"
DEFAULT_INSTALL = (Path.home() / "MORAI/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/"
                   "Scenario/R_KR_PR_K-city_2025")
LINK_SET = ROOT / "kcity_map/mgeo_map/link_set.json"

MAP_NAME = "R_KR_PR_K-city_2025"
EGO_LINK = "A2256W000213"
EGO_LINK_RATIO = 0.025441178348301677
EGO_LINK_PROJECTED_XYZ = (-79.00348615641492, -228.64628368637986, 28.386755644448428)
EGO_LINK_DISTANCE_M = 0.0005869524614958689
EXPECTED_COUNT = 200
EXPECTED_EGO_XY = (-79.004, -228.646)
EXPECTED_EGO_YAW = 61.178
NAME_PATTERN = re.compile(
    r"(?:^|/)static_final/scenario_(\d{3})_(sunny|cloudy|foggy)_intersection_([12])obj\.json$")
SMOKE_NAMES = (
    "scenario_001_cloudy_intersection_2obj.json",
    "scenario_003_sunny_intersection_1obj.json",
    "scenario_011_foggy_intersection_2obj.json",
    "scenario_077_sunny_intersection_1obj.json",
    "scenario_157_foggy_intersection_2obj.json",
)


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def sha256_file(path):
    return sha256_bytes(Path(path).read_bytes())


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def atomic_text(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def angle_difference_degrees(left, right):
    return abs((float(left) - float(right) + 180.0) % 360.0 - 180.0)


def project_to_link(point_xy):
    if math.dist(point_xy, EXPECTED_EGO_XY) > 0.05:
        raise ValueError("고정 link 계약과 다른 Ego 시작점입니다: {}".format(point_xy))
    pinned = {"link": EGO_LINK, "ratio": EGO_LINK_RATIO,
              "distance_m": EGO_LINK_DISTANCE_M,
              "projected_xyz": list(EGO_LINK_PROJECTED_XYZ),
              "source": "pinned_from_kcity_mgeo"}
    # MGeo is a separately prepared map asset and is not present in every Git
    # worktree.  When available, recompute and require exact agreement with the
    # pinned contract; otherwise the recorded projection remains reproducible.
    if not LINK_SET.is_file():
        return pinned
    links = json.loads(LINK_SET.read_text(encoding="utf-8"))
    link = next((row for row in links if row.get("idx") == EGO_LINK), None)
    if link is None:
        raise ValueError("MGeo link가 없습니다: {}".format(EGO_LINK))
    points = link["points"]
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
    travelled = 0.0
    candidates = []
    for first, second in zip(points, points[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        ratio = max(0.0, min(1.0, ((point_xy[0] - first[0]) * dx +
                                   (point_xy[1] - first[1]) * dy) / (length * length)))
        x, y = first[0] + ratio * dx, first[1] + ratio * dy
        z = first[2] + ratio * (second[2] - first[2])
        candidates.append((math.hypot(point_xy[0] - x, point_xy[1] - y),
                           (travelled + ratio * length) / total, (x, y, z)))
        travelled += length
    distance, ratio, projected = min(candidates)
    if distance > 0.20:
        raise ValueError("Ego가 예상 link에서 너무 멉니다: {:.3f}m".format(distance))
    if not math.isclose(ratio, EGO_LINK_RATIO, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("MGeo link ratio가 고정 계약과 다릅니다: {} != {}".format(
            ratio, EGO_LINK_RATIO))
    return {"link": EGO_LINK, "ratio": ratio, "distance_m": distance,
            "projected_xyz": list(projected), "link_length_m": total,
            "source": "verified_current_kcity_mgeo"}


def validate_scene(name, raw, match, projection):
    try:
        scene = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError("JSON 오류: {} ({})".format(name, exc)) from exc
    if scene.get("mapInfo", {}).get("mapName") != MAP_NAME:
        raise ValueError("맵 불일치: {}".format(name))
    ego = scene.get("egoVehicle", {})
    initial = ego.get("initPosition", {})
    position = initial.get("pos", {})
    ego_xy = (float(position.get("x")), float(position.get("y")))
    if math.dist(ego_xy, EXPECTED_EGO_XY) > 0.05:
        raise ValueError("Ego 시작점 불일치: {} {}".format(name, ego_xy))
    if angle_difference_degrees(initial.get("rot", {}).get("yaw"), EXPECTED_EGO_YAW) > 1.0:
        raise ValueError("Ego yaw 불일치: {}".format(name))
    objects = scene.get("objectList")
    if not isinstance(objects, list) or len(objects) != int(match.group(3)):
        raise ValueError("파일명 객체 수와 JSON이 다릅니다: {}".format(name))
    if scene.get("vehicleList") or scene.get("pedestrianList") or scene.get("aiPedstrianList"):
        raise ValueError("static_final에는 차량/보행자가 없어야 합니다: {}".format(name))
    unique_ids = [item.get("UNIQUEID") for item in objects]
    if len(unique_ids) != len(set(unique_ids)):
        raise ValueError("객체 UNIQUEID 중복: {}".format(name))
    for item in objects:
        scale = item.get("scale", {})
        if item.get("DataID") != 40100001:
            raise ValueError("CargoBox 이외 DataID: {}".format(name))
        if [float(scale.get(axis)) for axis in ("x", "y", "z")] != [0.7, 1.05, 0.7]:
            raise ValueError("예상하지 않은 CargoBox scale: {}".format(name))
        if float(item.get("moveSpeed", 0.0)) != 0.0 or item.get("waypointEventList"):
            raise ValueError("움직이는 정적 장애물: {}".format(name))
    if int(initial.get("gear", 0)) != 1 or float(ego.get("currentVelocity", 1.0)) != 0.0:
        raise ValueError("Ego가 Park/정지 저장 상태가 아닙니다: {}".format(name))

    derived = copy.deepcopy(scene)
    derived_initial = derived["egoVehicle"]["initPosition"]
    derived_initial["initLink"] = projection["link"]
    derived_initial["initLinkRatio"] = projection["ratio"]
    return derived


def campaign_text(name, scenario_directory, install_dir, display, *, include=None):
    relative = Path(scenario_directory).resolve().relative_to(ROOT).as_posix()
    selection = "  glob: 'scenario_*.json'\n"
    if include:
        selection += "  include:\n" + "".join("    - {}\n".format(item) for item in include)
    return """# static_final 다운로드 원본에서 생성한 개인 OD-GT 검증 캠페인입니다.
# 파일명의 날씨는 label일 뿐 JSON에 날씨 설정이 없으므로 날씨 성능으로 집계하지 않습니다.
schema_version: 2
campaign_name: {name}
scenarios:
  directory: {directory}
{selection}  repeat: 1
default_case:
  goal: {{x: 71.510, y: 429.608, z: 28.383, yaw_deg: 90.033}}
  waypoints: []
  timeout_sec: 90
  arrival_mode: stop
stack:
  pose_source: localization
  detector: od_gt
  traffic_light: false
recording:
  topic_profile: base_ws/src/planning_addons/planning_scenario_validator/config/internal/topic_profiles/basic.json
  include_sensors: false
  video: {{enabled: true, display: '{display}', fps: 10, focus_planviz: false, window_layout: split, bottom_bar_px: auto}}
  minimum_free_disk_gib: 10
network:
  sensor_json: configs/sensors/Sensor_competition_local.json
  ego_network_json: base_ws/src/planning_addons/planning_scenario_validator/config/internal/generated/ego_network_validator_local.json
morai:
  map_name: R_KR_PR_K-city_2025
  scenario_install_dir: {install}
  scenario_load_host: 127.0.0.1
  scenario_load_port: 9095
output_root: planner_logs/validation
resume: true
continue_on_system_error: false
readiness_timeout_sec: 60
scenario_ready_timeout_sec: 35
route_ready_timeout_sec: 75
validation:
  goal_tolerance_m: 2.0
  goal_hold_sec: 1.0
  goal_speed_max_mps: 0.5
  topic_loss_timeout_sec: 4.0
  stuck_timeout_sec: 0.0
upload: {{enabled: false}}
""".format(name=name, directory=relative, selection=selection,
           display=display, install=Path(install_dir).resolve())


def build(archive, output, smoke_output, campaign, smoke_campaign, install_dir, display):
    archive = Path(archive).resolve()
    if not archive.is_file():
        raise ValueError("static_final ZIP이 없습니다: {}".format(archive))
    projection = project_to_link(EXPECTED_EGO_XY)
    weather_counts = Counter()
    object_counts = Counter()
    cases = {}
    raw_hashes = set()
    with zipfile.ZipFile(str(archive)) as bundle:
        members = []
        for name in bundle.namelist():
            match = NAME_PATTERN.search(name)
            if match:
                members.append((int(match.group(1)), name, match))
        members.sort()
        if [number for number, _, _ in members] != list(range(1, EXPECTED_COUNT + 1)):
            raise ValueError("static_final 번호가 001~200 연속이 아닙니다")
        for number, member, match in members:
            raw = bundle.read(member)
            digest = sha256_bytes(raw)
            if digest in raw_hashes:
                raise ValueError("완전히 중복된 시나리오가 있습니다: {}".format(member))
            raw_hashes.add(digest)
            scene = validate_scene(member, raw, match, projection)
            filename = Path(member).name
            output_path = Path(output) / filename
            atomic_json(output_path, scene)
            weather_counts[match.group(2)] += 1
            object_counts[len(scene["objectList"])] += 1
            cases[filename] = {
                "source_member": member,
                "source_sha256": digest,
                "derived_sha256": sha256_file(output_path),
                "filename_weather_label_only": match.group(2),
                "object_count": len(scene["objectList"]),
                "normalization": {"initLink": projection["link"],
                                  "initLinkRatio": projection["ratio"]},
            }

    first = json.loads((Path(output) / SMOKE_NAMES[0]).read_text(encoding="utf-8"))
    clear = copy.deepcopy(first)
    clear["objectList"] = []
    clear_name = "scenario_000_clear_baseline.json"
    atomic_json(Path(smoke_output) / clear_name, clear)
    for filename in SMOKE_NAMES:
        atomic_json(Path(smoke_output) / filename,
                    json.loads((Path(output) / filename).read_text(encoding="utf-8")))

    manifest = {
        "schema_version": 1,
        "generated_by": Path(__file__).name,
        "source_archive": str(archive),
        "source_archive_sha256": sha256_file(archive),
        "source_is_immutable": True,
        "map_name": MAP_NAME,
        "scene_count": len(cases),
        "weather_label_counts": dict(sorted(weather_counts.items())),
        "weather_runtime_contract": "absent_from_json; filename label only",
        "object_counts": {str(key): value for key, value in sorted(object_counts.items())},
        "object_contract": {"DataID": 40100001, "asset": "CargoBox",
                            "scale": [0.7, 1.05, 0.7], "moveSpeed": 0.0},
        "ego_link_projection": projection,
        "clear_baseline": {"derived_from": SMOKE_NAMES[0], "filename": clear_name,
                           "objectList_removed": True},
        "smoke_cases": [clear_name] + list(SMOKE_NAMES),
        "cases": cases,
    }
    atomic_json(Path(output).parent / "manifest.json", manifest)
    atomic_json(Path(smoke_output).parent / "manifest.json", manifest)
    atomic_text(campaign, campaign_text("static_final_od_gt_all", output, install_dir, display))
    atomic_text(smoke_campaign, campaign_text(
        "static_final_od_gt_smoke", smoke_output, install_dir, display,
        include=[clear_name] + list(SMOKE_NAMES)))
    print("생성 완료: full={} smoke={} link={} ratio={:.12f}".format(
        len(cases), 1 + len(SMOKE_NAMES), projection["link"], projection["ratio"]))
    print(campaign)
    print(smoke_campaign)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--smoke-output", type=Path, default=DEFAULT_SMOKE_OUTPUT)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--smoke-campaign", type=Path, default=DEFAULT_SMOKE_CAMPAIGN)
    parser.add_argument("--scenario-install-dir", type=Path, default=DEFAULT_INSTALL)
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ":0.0"))
    args = parser.parse_args()
    build(args.archive, args.output, args.smoke_output, args.campaign,
          args.smoke_campaign, args.scenario_install_dir, args.display)


if __name__ == "__main__":
    main()
