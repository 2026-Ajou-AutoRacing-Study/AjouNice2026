#!/usr/bin/env python3
"""Build a personal 17-case campaign from saved MORAI static-obstacle scenes.

The saved ``static-startline`` scene supplies the shared ego initial state.
Each position scene keeps its complete saved world and only receives that ego
state.  ``static-obstacle-car1`` is deliberately emitted as case 17, after
the sixteen single-obstacle positions.

The generated files belong in the ignored mission workspace.  The validator
will add its own ExternalCtrl/ScenarioLoad EgoNetwork sidecar at runtime, so
the MORAI-saved ``EgoNetwork/*_MN.json`` files are intentionally not copied.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[5]
MISSION = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/missions/static_obstacle"
DEFAULT_SOURCE = (Path.home() / "Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/"
                  "Scenario/R_KR_PR_K-city_2025")
DEFAULT_OUTPUT = MISSION / "workspace/scenarios"
DEFAULT_CAMPAIGN = MISSION / "workspace/user_static_obstacle_17_gt.yaml"
DEFAULT_INSTALL = (Path.home() / "Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/"
                   "Scenario/R_KR_PR_K-city_2025")


START_LINK = "A2256W000213"
START_LINK_RATIO = 0.037013183817814564


def read_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError("MORAI 저장 시나리오가 없습니다: {}".format(path)) from exc
    except json.JSONDecodeError as exc:
        raise ValueError("유효한 JSON이 아닙니다: {} ({})".format(path, exc)) from exc


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ego_position(scene):
    try:
        return scene["egoVehicle"]["initPosition"]["pos"]
    except (KeyError, TypeError) as exc:
        raise ValueError("egoVehicle.initPosition.pos가 없습니다") from exc


def scene_entry(index, source_path, label, start_scene):
    scene = read_json(source_path)
    if scene.get("mapInfo", {}).get("mapName") != start_scene.get("mapInfo", {}).get("mapName"):
        raise ValueError("맵이 시작점과 다릅니다: {}".format(source_path.name))
    if not isinstance(scene.get("objectList"), list):
        raise ValueError("objectList가 없습니다: {}".format(source_path.name))
    target = copy.deepcopy(scene)
    target["egoVehicle"] = copy.deepcopy(start_scene["egoVehicle"])
    stem = "user_static_{:02d}_{}".format(index, label)
    object_positions = []
    for item in target["objectList"]:
        position = item.get("pos", {})
        object_positions.append({
            "data_id": item.get("DataID"),
            "unique_id": item.get("UNIQUEID"),
            "x": position.get("x"),
            "y": position.get("y"),
            "z": position.get("z"),
        })
    return stem, target, {
        "order": index,
        "source": str(source_path),
        "source_sha256": sha256(source_path),
        "description": "{} / static-startline ego 시작점 적용".format(label),
        "object_count": len(target["objectList"]),
        "objects": object_positions,
    }


def campaign_text(output, install_dir):
    relative_output = output.relative_to(ROOT).as_posix()
    return """# 자동 생성: static-startline의 ego 시작점을 16개 위치 + car1에 공통 적용합니다.
# 수정이 필요하면 이 파일을 다른 이름으로 복사해 사용하세요. 재생성하면 이 파일은 갱신됩니다.
schema_version: 2
campaign_name: user_static_obstacle_17_gt
scenarios:
  directory: {output}
  glob: 'user_static_*.json'
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
  video: {{enabled: true, display: ':1', fps: 10, focus_planviz: false, window_layout: split, bottom_bar_px: auto}}
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
upload: {{enabled: false, rclone_remote: 'ajou_drive:planning_validator', profile: basic}}
""".format(output=relative_output, install=install_dir)


def build(source_dir, output, campaign, install_dir):
    source_dir = source_dir.resolve()
    output = output.resolve()
    campaign = campaign.resolve()
    start_path = source_dir / "static-startline.json"
    start_scene = read_json(start_path)
    start_initial = start_scene["egoVehicle"]["initPosition"]
    if not str(start_initial.get("initLink", "")).strip():
        start_initial["initLink"] = START_LINK
        start_initial["initLinkRatio"] = START_LINK_RATIO
    start_pos = ego_position(start_scene)
    map_name = start_scene.get("mapInfo", {}).get("mapName")
    if map_name != "R_KR_PR_K-city_2025":
        raise ValueError("static-startline 맵이 예상과 다릅니다: {}".format(map_name))

    cases = []
    for index in range(1, 17):
        source = source_dir / "static-obstacle-pos{}.json".format(index)
        cases.append(scene_entry(index, source, "pos{:02d}".format(index), start_scene))
    cases.append(scene_entry(17, source_dir / "static-obstacle-car1.json", "car01", start_scene))

    output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "generated_by": Path(__file__).name,
        "source_directory": str(source_dir),
        "startline": {
            "source": str(start_path),
            "source_sha256": sha256(start_path),
            "applied_ego_start": {key: start_pos[key] for key in ("x", "y", "z")},
        },
        "cases": {},
    }
    for stem, scene, entry in cases:
        path = output / (stem + ".json")
        write_json(path, scene)
        manifest["cases"][stem] = entry
        print(path)
    write_json(output / "metadata/manifest.json", manifest)
    campaign.parent.mkdir(parents=True, exist_ok=True)
    temporary = campaign.with_name(campaign.name + ".tmp")
    temporary.write_text(campaign_text(output, install_dir.resolve()), encoding="utf-8")
    temporary.replace(campaign)
    print(campaign)
    print("생성 완료: 위치 16개 뒤에 car1 1개, 총 17개")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCE,
                        help="static-startline/pos1..16/car1 JSON이 있는 MORAI Scenario 폴더")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="생성된 개인 시나리오 폴더")
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN,
                        help="생성할 GT 캠페인 YAML 경로")
    parser.add_argument("--scenario-install-dir", type=Path, default=DEFAULT_INSTALL,
                        help="자동 실행 중 ScenarioLoad JSON을 설치할 실제 MORAI Scenario 폴더")
    args = parser.parse_args()
    build(args.source_dir, args.output, args.campaign, args.scenario_install_dir)


if __name__ == "__main__":
    main()
