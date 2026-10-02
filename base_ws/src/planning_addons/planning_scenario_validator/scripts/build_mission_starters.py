#!/usr/bin/env python3
"""Reproducible mission validation starters, not OD training data generation."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import yaml

from build_static_corridor_cases import _prepared_official_scene, _vector, OFFICIAL_GOAL
from mission_starter_guides import render_guide

ROOT = Path(__file__).resolve().parents[5]
PACKAGE = Path(__file__).resolve().parents[1]
OUTPUT = PACKAGE / "config/missions"
SAMPLE = ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
ACTOR_LISTS = ("vehicleList", "pedestrianList", "objectList", "spawnPointList",
               "pedspawnPointList", "aiPedstrianList", "waypointDataList")


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(scene, shade=False):
    scene = copy.deepcopy(scene)
    for key in ACTOR_LISTS:
        scene[key] = []
    if not shade:
        scene["shadedAreaList"] = []
    ego = scene["egoVehicle"]
    ego.update(currentVelocity=0.0, currentControlMode=2, beforeControlMode=2, customValue=0.0)
    ego["initPosition"]["gear"] = 1
    return scene


class Geometry:
    def __init__(self):
        self.links = {x["idx"]: x for x in read(ROOT / "kcity_map/mgeo_map/link_set.json")}
        self.route = [list(map(float, x.split()[:3])) for x in
                      (ROOT / "configs/global_path/2026_molit_comp_global_path.txt").read_text().splitlines() if x.strip()]
        self.translation = yaml.safe_load((ROOT / "configs/tf/morai_lanelet_static_tf.yaml").read_text())["map_to_morai_map"]
        if any(abs(x) > 1e-9 for x in self.translation["rpy"]):
            raise ValueError("Map rotation changed: review starter goal transformation")

    def link_pose(self, link_id, ratio, lateral=0.0):
        points = self.links[link_id]["points"]
        n = min(len(points) - 2, int(ratio * (len(points) - 1)))
        fraction = ratio * (len(points) - 1) - n
        a, b = points[n:n + 2]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        p = [a[i] + fraction * (b[i] - a[i]) for i in range(3)]
        p[0] -= math.sin(yaw) * lateral
        p[1] += math.cos(yaw) * lateral
        return p, math.degrees(yaw)

    def goal(self, point):
        # Goal uses map coordinates; scenario actors use MORAI coordinates.
        xyz = self.translation["xyz"]
        return dict(x=round(point[0] + xyz[0], 6), y=round(point[1] + xyz[1], 6),
                    z=round(point[2] + xyz[2], 6), yaw_deg=round(point[3], 6))

    def nearest_link(self, point, heading):
        candidates = []
        for link, row in self.links.items():
            pts = row["points"]
            for i, (a, b) in enumerate(zip(pts, pts[1:])):
                yaw = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
                if abs((yaw - heading + 180) % 360 - 180) > 45:
                    continue
                candidates.append((math.dist(point[:2], a[:2]), link, i / (len(pts) - 1)))
        _, link, ratio = min(candidates)
        return link, ratio

    def course_goal(self, start, distance):
        i = min(range(len(self.route)), key=lambda i: math.dist(start[:2], self.route[i][:2]))
        total = 0.0
        for j in range(i + 1, len(self.route) - 1):
            total += math.dist(self.route[j - 1], self.route[j])
            if total >= distance:
                p, q = self.route[j:j + 2]
                return self.goal(p + [math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))])
        raise ValueError("Requested mission endpoint exceeds course")


def place_ego(scene, geometry, link, ratio):
    p, yaw = geometry.link_pose(link, ratio)
    initial = scene["egoVehicle"]["initPosition"]
    initial.update(pos=_vector(p[0], p[1], p[2] + 0.34),
                   rot={"roll": "0.000", "pitch": "0.000", "yaw": "{:.3f}".format(yaw)},
                   initLink=link, initLinkRatio=ratio, initPositionMode="Absolute")
    return p


def static_vehicle(template, uid, p, yaw, link="", ratio=0):
    actor = copy.deepcopy(template)
    actor["UNIQUEID"] = uid
    actor["initPosition"].update(pos=_vector(*p), initLink=link, initLinkRatio=ratio,
        initPositionMode="Absolute", gear=1,
        rot={"roll": "0.000", "pitch": "0.000", "yaw": str(yaw)})
    # Do not transplant wheels/engine state from a moving sample NPC.
    actor["initPosition"].pop("vehicleSaveData", None)
    actor.update(currentVelocity=0.0, viewCurrentVelocity=0.0, desiredVelocity=0.0,
                 constantVelocity=0.0, linkVelocity=0.0, velocityType=2,
                 isWaypointMode=False, isCloseLoop=False, isLaneChange=False,
                 activeDistance=0.0, spawnPointUniqueID=-1, destinationUniqueID=-1,
                 wayPointInfos=[])
    for key in ("startPointLinkInfo", "endPointLinkInfo", "currentLinkInfo", "loopStartLinkInfo"):
        actor[key] = {"linkIdx": link, "pointIdx": 0}
    return actor


def build(output=OUTPUT):
    sample = read(SAMPLE)
    geom = Geometry()
    families = {}
    assets = read(ROOT / "dataset/morai_od/configs/morai_asset_catalog_260807.json")
    asset_names = {x["data_id"]: x["name"] for key in
                   ("vehicles", "pedestrians_and_related", "obstacles_and_related") for x in assets[key]}
    asset_names.update({40200020: "Scooter1", 40200021: "Scooter2"})

    def add(family, stem, scene, goal, timeout, description, tier="normal", parameters=None):
        families.setdefault(family, []).append((stem, scene, {
            "description": "[{}] {}".format(tier, description), "goal": goal,
            "waypoints": [], "timeout_sec": timeout, "arrival_mode": "stop"},
            {"name": stem, "tier": tier, "parameters": parameters or {},
             "actors": [{"container": key, "data_id": x["DataID"],
                         "asset": asset_names.get(x["DataID"], "unknown"),
                         "position_morai": x.get("pos", x.get("initPosition", {}).get("pos")),
                         "speed_config": x.get("speed", x.get("moveSpeed", x.get("desiredVelocity", 0)))}
                        for key in ("vehicleList", "pedestrianList", "objectList") for x in scene[key]],
             "goal_map": goal,
             "description": description, "live_verified": False}))

    # Stable sample anchors and original spawn start/end/destination links.
    for family, prefix, ids, duration, distance in (
            ("roundabout", "ra", {4}, 75, 100),
            ("highway", "high", {6, 8, 10}, 90, 220)):
        reference = read(ROOT / "configs/scenario/perception_benchmark" /
                         ("bench_roundabout_raw.json" if family == "roundabout" else "bench_highway_merge_raw.json"))
        base = clean(reference)
        pos = reference["egoVehicle"]["initPosition"]["pos"]
        goal = geom.course_goal([pos["x"], pos["y"]], distance)
        spawns = [s for s in sample["spawnPointList"] if s["UNIQUEID"] in ids]
        for i in range(24):
            scene = copy.deepcopy(base)
            tier = "baseline" if i < 2 else "normal" if i < 16 else "stress"
            scene["spawnPointList"] = copy.deepcopy(spawns if i else [])
            settings = []
            for n, spawn in enumerate(scene["spawnPointList"]):
                # Fixed, reproducible scenarios. RandomMode itself samples models
                # only from the sample's MiddleSize filter [2].
                speed = (20 if family == "roundabout" else (40, 50, 60)[n]) if i == 1 else (
                    (15, 20, 25, 30, 35, 40)[(i + n) % 6] if family == "roundabout" else
                    (30, 40, 50, 60, 70, 80)[(i + 2 * n) % 6])
                period = (5 if family == "roundabout" else 10) if i == 1 else (3, 5, 7, 10, 14)[i % 5]
                initial = 0 if i < 8 else min(speed, (10, 20, speed)[i % 3])
                offset = 0 if i < 16 else (-0.35, 0.35)[i % 2]
                spawn.update(MinDesiredVelocity_Custom=float(speed), MaxDesiredVelocity_Custom=float(speed),
                    MinSpawnVelocity_Custom=float(initial), MaxSpawnVelocity_Custom=float(initial),
                    MinSpawnVelocity_Link=0.0, MaxSpawnVelocity_Link=0.0,
                    SpawnVelocityType=2, DesiredVelocityType=2,
                    minSpawnPeriod=float(period), maxSpawnPeriod=float(period),
                    spawnedVehiclesCount=0, waitSpawnTime=0.0, isSpawning=True,
                    maximumSpawnVehicle=24, MinLatBias=offset, MaxLatBias=offset)
                if family == "highway":
                    spawn.update(spawnVehicleTypeID=-1, listVehicleLengthType=[2])
                else:
                    # One fixed model per scenario, unknown official model.
                    spawn["spawnVehicleTypeID"] = (30100004, 30100007, 30100014, 30200014)[i % 4] if i != 1 else 30100004
                settings.append({"spawn_id": spawn["UNIQUEID"], "goal_kmh": speed,
                    "initial_kmh": initial, "period_sec": period, "lateral_m": offset,
                    "model_id": spawn["spawnVehicleTypeID"]})
            add(family, "{}_{}_{:02d}".format(prefix, tier, i), scene, goal, duration,
                "샘플 spawn/종료 링크 유지; {}".format("무차량 기준선" if not i else "속도·간격·차종 변형"),
                tier, {"spawns": settings})

    static_base = clean(_prepared_official_scene())
    catalog = read(ROOT / "configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json")["objectList"]
    object_ids = (40100001, 40100027, 40100005, 40100047, 40100028)
    by_id = {int(x["DataID"]): x for x in catalog}
    for i in range(24):
        scene = copy.deepcopy(static_base)
        tier = "baseline" if i < 2 else "normal" if i < 16 else "stress"
        count = 0 if i == 0 else 1 if i < 10 else 2 if i < 20 else 3
        for n in range(count):
            uid = 100 + n
            # Sample static mission, not arbitrary dataset locations.
            p = [-60.61 + (0, -0.65, 0.65)[(i + n) % 3], -159.25 + n * 17, 28.4]
            if i in (15, 18, 21) and n == 0:
                link, ratio = geom.nearest_link(p, 90)
                scene["vehicleList"].append(static_vehicle(sample["vehicleList"][0], uid, p, 90, link, ratio))
            else:
                actor = copy.deepcopy(by_id[object_ids[(i + n) % len(object_ids)]])
                actor.update(UNIQUEID=uid, pos=_vector(*p), scale=_vector(1, 1, 1),
                             isWaypointMode=False, moveSpeed=0.0, activeDistance=0.0)
                scene["objectList"].append(actor)
        if i == 1:
            scene["objectList"] = [copy.deepcopy(sample["objectList"][0])]
        if i >= 22:
            actor = copy.deepcopy(sample["objectList"][0])
            actor.update(DataID=40200020 if i == 22 else 40100059, UNIQUEID=110,
                         pos=_vector(-59.8, -133.5, 28.4), scale=_vector(1, 1, 1),
                         isWaypointMode=False, moveSpeed=0.0, activeDistance=0.0)
            scene["objectList"].append(actor)
        add("static_obstacle", "soa_{}_{:02d}".format(tier, i), scene,
            dict(zip(("x", "y", "z", "yaw_deg"), OFFICIAL_GOAL)), 65,
            "일반 정적 구간 좌우/다객체/정지 차량; 01은 원본 scaled CargoBox", tier)

    # No shadow cut-in traffic. Retain the official shade plus static actors.
    shadow_ref = read(ROOT / "configs/scenario/localization_evaluation/localization_shadow_entry_clear.json")
    shadow_goal = yaml.safe_load((PACKAGE / "config/missions/gps_shadow/base/static_obstacle_bevfusion.yaml").read_text())["default_case"]["goal"]
    shadow_sources = sorted((ROOT / "configs/scenario/hidden_mission/static_2").glob("*.json"))[:9]
    shadow_sources += sorted((ROOT / "configs/scenario/hidden_mission/static_3").glob("*.json"))[:9]
    for i in range(24):
        tier = "baseline" if i < 2 else "normal" if i < 16 else "stress"
        if i < 6:
            scene = clean(shadow_ref, shade=True)
            if i:
                p, yaw = geom.link_pose("A2256W000126", 0.35, (-0.6, 0.0, 0.6)[i % 3])
                if i < 3:
                    actor = copy.deepcopy(sample["objectList"][0])
                    actor.update(pos=_vector(*p), scale=_vector(1, 1, 1), isWaypointMode=False)
                    scene["objectList"] = [actor]
                else:
                    link, ratio = geom.nearest_link(p, yaw)
                    scene["vehicleList"] = [static_vehicle(sample["vehicleList"][0], 100, p, yaw, link, ratio)]
        else:
            scene = clean(read(shadow_sources[i - 6]), shade=True)
            source = read(shadow_sources[i - 6])
            for key in ("objectList", "vehicleList", "pedestrianList"):
                scene[key] = source[key]
            scene["egoVehicle"] = copy.deepcopy(clean(shadow_ref, shade=True)["egoVehicle"])
        add("gps_shadow", "hid_soa_{}_{:02d}".format(tier, i), scene, shadow_goal, 90,
            "공식 음영 유지; 정적 객체/정지 차량 회피, 끼어들기 없음", tier,
            {"source": str(shadow_sources[i - 6].relative_to(ROOT)) if i >= 6 else "shadow_entry_clear"})

    # Dynamic mission is the sample intersection, not the GPS shadow.
    base = clean(sample)
    start = place_ego(base, geom, "A2256W000202", 0.40)
    p, yaw = geom.link_pose("A2256W000315", 0.42)
    goal = geom.goal(p + [yaw])
    path = next(x for x in sample["waypointDataList"] if x["pathName"] == "PedPath - 3")
    for i in range(24):
        scene = copy.deepcopy(base)
        tier = "baseline" if i < 2 else "normal" if i < 16 else "stress"
        if 1 <= i < 18:
            actor = copy.deepcopy(sample["pedestrianList"][0])
            way = copy.deepcopy(path)
            speed = 10 if i == 1 else (3, 5, 7, 10)[i % 4]
            trigger = 15 if i == 1 else (15, 20, 25, 30)[i % 4]
            if i % 2 == 0:
                way["waypointData"].reverse()
                q = way["waypointData"][0]["pos"]
                actor["pos"] = _vector(-q["x"], -q["z"], q["y"])
                actor["initPos"] = copy.deepcopy(q)
                actor["standardPos"] = copy.deepcopy(q)
                actor["rot"]["yaw"] = "96.738"
            actor.update(speed=float(speed), activeDistance=float(trigger), movingDistanceAmount=0.0)
            if i < 12:
                actor["DataID"] = (50100001, 50100016, 50200001, 50100077)[i % 4] if i != 1 else 50100001
                scene["pedestrianList"] = [actor]
            else:
                obj = copy.deepcopy(sample["objectList"][0])
                obj.update(DataID=(40200020, 40200021, 40100059)[i % 3], UNIQUEID=3,
                    pos=actor["pos"], rot=actor["rot"], initPos=actor["initPos"],
                    scale=_vector(1, 1, 1), isWaypointMode=True, moveSpeed=float(speed),
                    activeDistance=float(trigger), waypointEventList=[])
                scene["objectList"] = [obj]
                way["pathName"] = "ObstaclePath - 3"
            scene["waypointDataList"] = [way]
        elif i >= 18:
            # Road-constrained NPC crosses the intersection on an existing link.
            link = "A2256W001124" if i % 2 else "A2256W000636"
            p, yaw = geom.link_pose(link, 0.05)
            actor = static_vehicle(sample["vehicleList"][0], 100, p, yaw, link, 0.05)
            actor.update(DataID=(30100004, 30100014, 30200014)[i % 3],
                         desiredVelocity=float((10, 20, 30)[i % 3]),
                         constantVelocity=float((10, 20, 30)[i % 3]), activeDistance=25.0,
                         destinationMode=1)
            actor["initPosition"]["gear"] = 4
            actor["startPointLinkInfo"] = {"linkIdx": link, "pointIdx": 0}
            actor["currentLinkInfo"] = {"linkIdx": link, "pointIdx": 0}
            actor["endPointLinkInfo"] = {"linkIdx": link, "pointIdx": len(geom.links[link]["points"]) - 1}
            scene["vehicleList"] = [actor]
            if i >= 22:
                # Concurrent road vehicle plus the original crossing pedestrian.
                scene["pedestrianList"] = [copy.deepcopy(sample["pedestrianList"][0])]
                scene["waypointDataList"] = [copy.deepcopy(path)]
        add("dynamic_obstacle", "do_{}_{:02d}".format(tier, i), scene, goal, 70,
            "공식 사거리; 보행자/스쿠터 양방향 또는 도로 NPC 교차", tier,
            {"experimental_actor_motion": i >= 12})

    template = yaml.safe_load((PACKAGE / "config/missions/static_obstacle/base/static_pair_mission_bevfusion.yaml").read_text())
    expected = {}
    for family, rows in families.items():
        folder = Path(output) / family / "base"
        for stem, scene, case, meta in rows:
            validate(scene, geom)
            expected[folder / "scenarios" / (stem + ".json")] = json.dumps(scene, ensure_ascii=False, indent=2) + "\n"
        common = {k: v for k, v in rows[0][2].items() if k != "description"}
        if any({k: v for k, v in row[2].items() if k != "description"} != common for row in rows):
            raise ValueError("{}: 서로 다른 목적지/시간은 별도 캠페인 YAML로 분리하세요".format(family))
        campaign = copy.deepcopy(template)
        campaign.update(schema_version=2, campaign_name="starter_{}_bev".format(family))
        campaign["scenarios"] = {"directory": str((folder / "scenarios").relative_to(ROOT)), "glob": "*.json"}
        campaign["default_case"] = common
        campaign["recording"]["video"].update(window_layout="split", focus_planviz=False, bottom_bar_px="auto", fps=10)
        campaign["recording"]["minimum_free_disk_gib"] = 10
        campaign["upload"] = {"enabled": False, "rclone_remote": "ajou_drive:planning_validator", "profile": "basic"}
        header = ("# 미션별 공유 예제: GUI에서 열어 개인 YAML로 저장하세요.\n"
                  "# 이 YAML의 목적지/주행시간이 모든 선택 시나리오에 동일하게 적용됩니다.\n"
                  "# 다른 목적지/시간은 별도 이름의 YAML로 시험하세요.\n"
                  "# 센서는 계속 켜고, bag에서만 LiDAR/카메라를 제외합니다.\n"
                  "# GT 대조는 GUI 탐지기를 od_gt로 바꿔 저장하고 새 실험을 시작하세요.\n")
        expected[folder / "example.yaml"] = header + yaml.safe_dump(campaign, allow_unicode=True, sort_keys=False)
        expected[folder / "README.md"] = render_guide(family, rows, campaign, geom, ROOT, PACKAGE)
        expected[folder / "manifest.json"] = json.dumps({"schema_version": 1, "mission": family,
            "source_sample_sha256": digest(SAMPLE), "live_verified": False,
            "geometry_sha256": {str(p.relative_to(ROOT)): digest(p) for p in (
                ROOT / "kcity_map/mgeo_map/link_set.json", ROOT / "configs/global_path/2026_molit_comp_global_path.txt",
                ROOT / "configs/tf/morai_lanelet_static_tf.yaml")},
            "official_distribution": "unknown; illustrative validation variations, not competition guarantees",
            "cases": [x[3] for x in rows]}, ensure_ascii=False, indent=2) + "\n"
    return expected


def validate(scene, geom):
    ids = [scene["egoVehicle"]["UNIQUEID"]]
    for key in ACTOR_LISTS[:-1]:
        ids.extend(x["UNIQUEID"] for x in scene[key])
    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate actor IDs")
    for actor in scene["spawnPointList"]:
        for key in ("startLinkInfo", "endLinkInfo"):
            info = actor[key]
            if info["linkIdx"] not in geom.links or not 0 <= info["pointIdx"] < len(geom.links[info["linkIdx"]]["points"]):
                raise ValueError("Spawn link/index outside map")
    if scene["mapInfo"]["mapName"] != "R_KR_PR_K-city_2025":
        raise ValueError("Wrong map")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="생성 파일을 수정하지 않고 재현성 검사")
    args = parser.parse_args()
    expected = build()
    for path, content in expected.items():
        if args.check:
            if not path.is_file() or path.read_text(encoding="utf-8") != content:
                raise SystemExit("Regeneration mismatch: {}".format(path))
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
    obsolete = sorted(OUTPUT.glob("*/base/scenarios/*.case.yaml"))
    if obsolete:
        raise SystemExit("폐기된 보조 YAML이 {}개 남았습니다. 공유 base의 .case.yaml을 제거하세요".format(len(obsolete)))
    print("{} files / 5 missions / 120 scenarios {}".format(len(expected), "verified" if args.check else "generated"))


if __name__ == "__main__":
    main()
