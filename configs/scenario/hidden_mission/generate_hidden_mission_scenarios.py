#!/usr/bin/env python3
"""Generate GPS-shadow hidden-mission MORAI scenarios.

The user's hid_start_pos.json and hid1.json are treated as immutable references.
Generated files are split into static_2, static_3, and dynamic_1 families.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BASE_PATH = HERE / "hid_start_pos.json"
OFFICIAL_PATH = ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
HID1_PATH = HERE / "hid1.json"
MGEO_LINKS_PATH = ROOT / "kcity_map/mgeo_map/link_set.json"
MANIFEST_PATH = HERE / "manifest.json"

ROUTE_LINK_IDS = ("A2256W000446", "A2256W000448", "A2256W000126")
SHADE_DATA_ID = 110200001
EGO_WORLD = (53.080001831054688, -466.31201171875)

VEHICLES = {
    "sedan": (30100014, "2020_Kia_K5"),
    "suv": (30200014, "2023_Hyundai_Ioniq5"),
    "bus": (30300003, "DEFAULT_BUS"),
    "truck": (30400004, "2013_Hyundai_Xcient(P540)"),
}
PEDESTRIANS = {
    "adult": (50100001, "Man1"),
    "woman": (50100016, "Woman8"),
    "child": (50200001, "Boy1"),
    "worker": (50100077, "worker_m_1"),
    "police": (50100058, "police_m_2"),
    "cyclist": (50100084, "Bicycle_Female1"),
}
OBJECTS = {
    "cargo": (40100001, "CargoBox"),
    "barrier": (40200162, "concrete-barrier"),
    "bicycle_city": (40100059, "Bicycle_city"),
    "bicycle_mountain": (40100060, "Bicycle_mountain"),
    "scooter_01": (40100062, "Scooter_01"),
    "scooter_02": (40100063, "Scooter_02"),
    "sport_bike": (40200017, "SportBike"),
    "electric_scooter_1": (40200018, "ElectricScooter1"),
    "electric_scooter_2": (40200019, "ElectricScooter2"),
    "scooter_rider_1": (40200020, "Scooter1"),
    "scooter_rider_2": (40200021, "Scooter2"),
    "baby_stroller": (40200011, "BabyStroller1"),
}


STATIC_2 = [
    ("s2_01_sedan_suv_gap08", ["v:sedan", "v:suv"], 44.0, [8.0], [0.8, -0.8]),
    ("s2_02_truck_sedan_gap10", ["v:truck", "v:sedan"], 52.0, [10.0], [-0.8, 0.8]),
    ("s2_03_bus_scooter_gap18", ["v:bus", "o:scooter_01"], 61.0, [18.0], [0.6, -1.0]),
    ("s2_04_adult_child_gap08", ["p:adult", "p:child"], 47.0, [8.0], [-1.0, 0.8]),
    ("s2_05_worker_bicycle_gap12", ["p:worker", "o:bicycle_city"], 56.0, [12.0], [0.8, -0.8]),
    ("s2_06_bicycle_scooter_gap08", ["o:bicycle_mountain", "o:scooter_02"], 65.0, [8.0], [-1.0, 1.0]),
    ("s2_07_two_scooters_gap14", ["o:scooter_01", "o:scooter_02"], 42.0, [14.0], [0.7, -0.9]),
    ("s2_08_cargo_barrier_gap10", ["o:cargo", "o:barrier"], 59.0, [10.0], [0.0, 0.0]),
    ("s2_09_sedan_adult_gap12", ["v:sedan", "p:adult"], 49.0, [12.0], [0.8, -0.6]),
    ("s2_10_suv_bicycle_gap15", ["v:suv", "o:bicycle_city"], 62.0, [15.0], [-0.8, 0.7]),
    ("s2_11_suv_scooter_tight06", ["v:suv", "o:scooter_01"], 54.0, [6.0], [0.9, -0.9]),
    ("s2_12_woman_scooter_wide18", ["p:woman", "o:scooter_02"], 68.0, [18.0], [-1.0, 0.0]),
    ("s2_13_sedan_barrier_stagger12", ["v:sedan", "o:barrier"], 45.0, [12.0], [0.7, -0.7]),
    ("s2_14_truck_child_wide20", ["v:truck", "p:child"], 66.0, [20.0], [-0.6, 0.8]),
]

STATIC_3 = [
    ("s3_01_three_vehicles_gap10", ["v:sedan", "v:suv", "v:truck"], 42.0, [8.0, 12.0], [1.0, -0.9, 1.1]),
    ("s3_02_vehicle_ped_bicycle_gap10", ["v:sedan", "p:adult", "o:bicycle_city"], 47.0, [13.0, 7.0], [-1.0, 0.9, -1.1]),
    ("s3_03_three_pedestrians_gap08", ["p:adult", "p:child", "p:worker"], 52.0, [6.0, 10.0], [0.8, -1.1, 0.9]),
    ("s3_04_bicycle_two_scooters_gap08", ["o:bicycle_mountain", "o:scooter_01", "o:scooter_02"], 58.0, [11.0, 5.0], [-1.1, 0.8, -1.0]),
    ("s3_05_sedan_scooter_barrier_gap12", ["v:sedan", "o:scooter_01", "o:barrier"], 44.0, [9.0, 15.0], [0.9, -1.0, 1.1]),
    ("s3_06_bus_woman_bicycle_gap15", ["v:bus", "p:woman", "o:bicycle_city"], 42.0, [12.0, 18.0], [-0.8, 1.0, -1.1]),
    ("s3_07_three_barriers_dense06", ["o:barrier", "o:cargo", "o:barrier"], 61.0, [5.5, 7.0], [1.1, -0.8, 1.0]),
    ("s3_08_truck_bicycle_child_gap18", ["v:truck", "o:bicycle_city", "p:child"], 44.0, [14.0, 20.0], [-0.9, 1.1, -0.8]),
    ("s3_09_sedan_suv_cargo_gap08", ["v:sedan", "v:suv", "o:cargo"], 64.0, [10.0, 6.0], [1.0, -0.9, 0.8]),
    ("s3_10_alternating_lateral_gap10", ["v:sedan", "o:scooter_02", "p:adult"], 46.0, [7.0, 13.0], [-1.1, 0.8, -1.0]),
    ("s3_11_ped_bicycle_scooter_gap12", ["p:worker", "o:bicycle_mountain", "o:scooter_01"], 57.0, [16.0, 8.0], [0.8, -1.0, 1.1]),
    ("s3_12_two_cars_ped_gap20", ["v:sedan", "v:suv", "p:police"], 43.0, [14.0, 22.0], [-1.0, 0.9, -1.1]),
    ("s3_13_small_objects_dense05p5", ["o:bicycle_city", "o:scooter_01", "o:cargo"], 67.0, [4.5, 6.5], [1.1, -0.8, 0.9]),
    ("s3_14_mixed_wide22", ["v:truck", "p:adult", "o:scooter_02"], 42.0, [18.0, 23.0], [-0.8, 1.1, -0.9]),
]

DYNAMIC_1 = [
    ("d1_01_adult_ltr_nominal", "p:adult", 45.0, "ltr", 4.0, 30.0, 12.0),
    ("d1_02_adult_rtl_nominal", "p:adult", 45.0, "rtl", 4.0, 30.0, 12.0),
    ("d1_03_child_ltr_slow", "p:child", 55.0, "ltr", 3.0, 24.0, 10.0),
    ("d1_04_woman_rtl_fast", "p:woman", 55.0, "rtl", 6.0, 30.0, 12.0),
    ("d1_05_worker_ltr_late", "p:worker", 68.0, "ltr", 5.0, 18.0, 12.0),
    ("d1_06_police_rtl_early", "p:police", 68.0, "rtl", 5.0, 40.0, 14.0),
    ("d1_07_cyclist_ltr", "p:cyclist", 80.0, "ltr", 9.0, 35.0, 14.0),
    ("d1_08_cyclist_rtl", "p:cyclist", 80.0, "rtl", 7.0, 28.0, 14.0),
    ("d1_09_sportbike_ltr", "o:sport_bike", 50.0, "ltr", 10.0, 35.0, 16.0),
    ("d1_10_e_scooter1_rtl", "o:electric_scooter_1", 58.0, "rtl", 5.0, 25.0, 12.0),
    ("d1_11_e_scooter2_ltr", "o:electric_scooter_2", 66.0, "ltr", 7.0, 30.0, 12.0),
    ("d1_12_scooter1_rtl_fast", "o:scooter_rider_1", 74.0, "rtl", 10.0, 35.0, 16.0),
    ("d1_13_scooter2_ltr_late", "o:scooter_rider_2", 82.0, "ltr", 8.0, 18.0, 14.0),
    ("d1_14_stroller_rtl_slow", "o:baby_stroller", 88.0, "rtl", 3.0, 28.0, 10.0),
]


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def dump_json(data) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(x: float, y: float, z: float):
    return {"x": x, "y": y, "z": z, "_x": f"{x:.3f}", "_y": f"{y:.3f}", "_z": f"{z:.3f}"}


def rotation(yaw: float):
    return {"roll": "0.000", "pitch": "0.000", "yaw": f"{yaw:.3f}"}


def waypoint_vector(position):
    # MORAI waypoint JSON stores Unity coordinates: (-ENU x, ENU z, -ENU y).
    return vector(-position[0], position[2], -position[1])


def build_route(links):
    by_id = {row["idx"]: row for row in links}
    points = []
    point_links = []
    for link_id in ROUTE_LINK_IDS:
        link = by_id[link_id]
        for point in link["points"]:
            if points and math.dist(points[-1], point) < 1e-6:
                continue
            points.append(tuple(float(value) for value in point))
            point_links.append(link_id)
    arc = [0.0]
    for first, second in zip(points, points[1:]):
        arc.append(arc[-1] + math.dist(first, second))

    best = None
    cumulative = 0.0
    for index, (first, second) in enumerate(zip(points, points[1:])):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length_xy = math.hypot(dx, dy)
        ratio = max(0.0, min(1.0, ((EGO_WORLD[0] - first[0]) * dx + (EGO_WORLD[1] - first[1]) * dy) / (length_xy**2)))
        projected = (first[0] + ratio * dx, first[1] + ratio * dy)
        distance = math.hypot(EGO_WORLD[0] - projected[0], EGO_WORLD[1] - projected[1])
        candidate = (distance, cumulative + ratio * math.dist(first, second))
        if best is None or candidate < best:
            best = candidate
        cumulative += math.dist(first, second)
    return points, point_links, arc, best[1], best[0]


def pose_at(route, station_m: float, lateral_m: float = 0.0):
    points, point_links, arc, ego_arc, _ = route
    target = ego_arc + station_m
    for index, (first, second) in enumerate(zip(points, points[1:])):
        if arc[index + 1] < target:
            continue
        ratio = (target - arc[index]) / (arc[index + 1] - arc[index])
        x = first[0] + ratio * (second[0] - first[0])
        y = first[1] + ratio * (second[1] - first[1])
        z = first[2] + ratio * (second[2] - first[2])
        yaw = math.degrees(math.atan2(second[1] - first[1], second[0] - first[0]))
        normal = (-math.sin(math.radians(yaw)), math.cos(math.radians(yaw)))
        x += lateral_m * normal[0]
        y += lateral_m * normal[1]
        return {"position": (x, y, z), "yaw": yaw, "normal": normal, "link_id": point_links[index], "ratio": ratio}
    raise ValueError(f"station {station_m} m is outside route")


def empty_scene(base, shade):
    scene = copy.deepcopy(base)
    for key in ("vehicleList", "pedestrianList", "objectList", "spawnPointList", "pedspawnPointList", "aiPedstrianList", "waypointDataList"):
        scene[key] = []
    scene["shadedAreaList"] = [copy.deepcopy(shade)]
    return scene


def zero_vehicle_state(actor):
    for key in ("linkVelocity", "constantVelocity", "desiredVelocity", "currentVelocity", "viewCurrentVelocity"):
        if key in actor:
            actor[key] = 0.0
    actor["isCloseLoop"] = False
    actor["isLaneChange"] = False
    actor["isWaypointMode"] = False
    actor["activeDistance"] = 0.0
    actor["wayPointInfos"] = []
    save = actor.get("initPosition", {}).get("vehicleSaveData", {})
    save["velocity"] = 0.0
    for key in ("lastVelocity",):
        if key in save:
            save[key] = {"x": 0.0, "y": 0.0, "z": 0.0}
    if "vehicle" in save:
        save["vehicle"]["lastVelocity"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    for wheel in save.get("wheels", []):
        wheel["L"] = 0.0
        wheel["Tr"] = 0.0


def make_static_actor(kind, key, uid, pose, templates):
    position, yaw = pose["position"], pose["yaw"]
    if kind == "v":
        data_id, name = VEHICLES[key]
        actor = copy.deepcopy(templates["vehicle"])
        actor["DataID"] = data_id
        actor["UNIQUEID"] = uid
        actor["initPosition"]["initPositionMode"] = "Absolute"
        actor["initPosition"]["pos"] = vector(*position)
        actor["initPosition"]["rot"] = rotation(yaw)
        actor["initPosition"]["initLink"] = pose["link_id"]
        actor["initPosition"]["initLinkRatio"] = pose["ratio"]
        actor["initPosition"]["gear"] = 1
        zero_vehicle_state(actor)
        return "vehicleList", actor, data_id, name
    if kind == "p":
        data_id, name = PEDESTRIANS[key]
        actor = copy.deepcopy(templates["pedestrian"])
        actor.update({"DataID": data_id, "UNIQUEID": uid, "pos": vector(*position), "rot": rotation(yaw), "activeDistance": 0.0, "speed": 0.0, "active": True, "loop": False, "movingDistance": 0.0, "movingDistanceAmount": 0.0, "isWaypointMode": False, "waypointEventList": []})
        actor["initPos"] = waypoint_vector(position)
        actor["standardPos"] = waypoint_vector(position)
        return "pedestrianList", actor, data_id, name
    data_id, name = OBJECTS[key]
    actor = copy.deepcopy(templates["object"])
    actor.update({"DataID": data_id, "UNIQUEID": uid, "pos": vector(*position), "rot": rotation(yaw), "isWaypointMode": False, "moveSpeed": 0.0, "activeDistance": 0.0, "waypointEventList": []})
    return "objectList", actor, data_id, name


def parse_actor(spec: str):
    kind, key = spec.split(":", 1)
    return kind, key


def make_static_scene(base, shade, route, templates, definition, family):
    name, specs, start_station, gaps, offsets = definition
    scene = empty_scene(base, shade)
    actors = []
    stations = [start_station]
    for gap in gaps:
        stations.append(stations[-1] + gap)
    for index, (spec, lateral) in enumerate(zip(specs, offsets)):
        station = stations[index]
        pose = pose_at(route, station, lateral)
        container, actor, data_id, asset_name = make_static_actor(*parse_actor(spec), 100 + index, pose, templates)
        scene[container].append(actor)
        actors.append({"container": container, "data_id": data_id, "asset_name": asset_name, "station_m": station, "lateral_offset_m": lateral, "position": dict(zip(("x", "y", "z"), pose["position"])), "yaw_deg": pose["yaw"]})
    return name, scene, {"family": family, "actor_count": len(specs), "first_obstacle_station_m": start_station, "longitudinal_gaps_m": gaps, "lateral_pattern": ["left" if value > 0 else "right" if value < 0 else "center" for value in offsets], "actors": actors, "intent": "force alternating left-right avoidance through the obstacle sequence"}


def crossing_endpoints(route, station, path_length, direction):
    pose = pose_at(route, station)
    center = pose["position"]
    normal = pose["normal"]
    half = path_length / 2.0
    left = (center[0] + normal[0] * half, center[1] + normal[1] * half, center[2])
    right = (center[0] - normal[0] * half, center[1] - normal[1] * half, center[2])
    return (left, right) if direction == "ltr" else (right, left)


def make_dynamic_scene(base, shade, route, templates, definition):
    name, spec, station, direction, speed, active_distance, path_length = definition
    scene = empty_scene(base, shade)
    kind, key = parse_actor(spec)
    start, end = crossing_endpoints(route, station, path_length, direction)
    yaw = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
    uid = 200
    waypoint_template = copy.deepcopy(templates["waypoint"])
    waypoint_data = []
    for position in (start, end):
        waypoint = copy.deepcopy(waypoint_template)
        waypoint["pos"] = waypoint_vector(position)
        waypoint["rot"] = rotation(0.0)
        waypoint_data.append(waypoint)
    if kind == "p":
        data_id, asset_name = PEDESTRIANS[key]
        actor = copy.deepcopy(templates["pedestrian"])
        actor.update({"DataID": data_id, "UNIQUEID": uid, "pos": vector(*start), "rot": rotation(yaw), "activeDistance": active_distance, "speed": speed, "active": True, "loop": False, "movingDistance": path_length, "movingDistanceAmount": 0.0, "isWaypointMode": True})
        actor["initPos"] = waypoint_vector(start)
        actor["standardPos"] = waypoint_vector(start)
        container = "pedestrianList"
        path_name = f"PedPath - {uid}"
    else:
        data_id, asset_name = OBJECTS[key]
        actor = copy.deepcopy(templates["object"])
        actor.update({"DataID": data_id, "UNIQUEID": uid, "pos": vector(*start), "rot": rotation(yaw), "isWaypointMode": True, "moveSpeed": speed, "activeDistance": active_distance, "waypointEventList": []})
        container = "objectList"
        path_name = f"ObstaclePath - {uid}"
    scene[container] = [actor]
    scene["waypointDataList"] = [{"pathName": path_name, "waypointData": waypoint_data}]
    metadata = {"family": "dynamic_1", "actor_count": 1, "actor": {"container": container, "data_id": data_id, "asset_name": asset_name, "station_m": station, "direction": direction, "speed_config_value": speed, "active_distance_m": active_distance, "path_length_m": path_length, "start": dict(zip(("x", "y", "z"), start)), "end": dict(zip(("x", "y", "z"), end))}, "intent": "cross the ego road laterally and test stop-or-avoid behavior"}
    return name, scene, metadata


def validate_scene(scene, metadata):
    ids = [-1]
    for key in ("vehicleList", "pedestrianList", "objectList"):
        ids.extend(actor["UNIQUEID"] for actor in scene[key])
    if len(ids) != len(set(ids)):
        raise ValueError("duplicate UNIQUEID")
    actor_count = sum(len(scene[key]) for key in ("vehicleList", "pedestrianList", "objectList"))
    if actor_count != metadata["actor_count"]:
        raise ValueError("actor count mismatch")
    if len(scene["shadedAreaList"]) != 1 or scene["shadedAreaList"][0]["DataID"] != SHADE_DATA_ID:
        raise ValueError("official GPS shade missing")
    if metadata["family"].startswith("static"):
        if scene["waypointDataList"]:
            raise ValueError("static scene has waypoint path")
        if any(float(actor.get("currentVelocity", 0.0)) != 0.0 for actor in scene["vehicleList"]):
            raise ValueError("static vehicle velocity is nonzero")
        if any(float(actor.get("speed", 0.0)) != 0.0 for actor in scene["pedestrianList"]):
            raise ValueError("static pedestrian speed is nonzero")
        if any(float(actor.get("moveSpeed", 0.0)) != 0.0 for actor in scene["objectList"]):
            raise ValueError("static object speed is nonzero")
        stations = [actor["station_m"] for actor in metadata["actors"]]
        if not 42.0 <= stations[0] <= 68.0 or stations[-1] > 90.0:
            raise ValueError("static obstacle station is outside the intended shade depth")
        if metadata["family"] == "static_3":
            pattern = metadata["lateral_pattern"]
            if pattern not in (["left", "right", "left"], ["right", "left", "right"]):
                raise ValueError(f"static_3 is not alternating: {pattern}")
            if len(set(metadata["longitudinal_gaps_m"])) != 2:
                raise ValueError("static_3 longitudinal gaps must be irregular")
            if any(abs(actor["lateral_offset_m"]) < 0.7 for actor in metadata["actors"]):
                raise ValueError("static_3 lateral offset is too small for a slalom")
    else:
        if len(scene["waypointDataList"]) != 1 or len(scene["waypointDataList"][0]["waypointData"]) != 2:
            raise ValueError("dynamic scene must have one two-point path")
        actor = (scene["pedestrianList"] or scene["objectList"])[0]
        waypoint = scene["waypointDataList"][0]["waypointData"][0]["pos"]
        world_start = (-waypoint["x"], -waypoint["z"], waypoint["y"])
        if math.dist(world_start, (actor["pos"]["x"], actor["pos"]["y"], actor["pos"]["z"])) > 0.01:
            raise ValueError("dynamic actor does not start at first waypoint")


def expected_outputs(base, official, hid1, links):
    shade = next(item for item in official["shadedAreaList"] if item["DataID"] == SHADE_DATA_ID)
    object_template = copy.deepcopy(hid1["objectList"][0])
    pedestrian_template = copy.deepcopy(official["pedestrianList"][0])
    vehicle_template = copy.deepcopy(official["vehicleList"][0])
    waypoint_path = next(item for item in official["waypointDataList"] if item.get("waypointData"))
    waypoint_template = copy.deepcopy(waypoint_path["waypointData"][0])
    templates = {"object": object_template, "pedestrian": pedestrian_template, "vehicle": vehicle_template, "waypoint": waypoint_template}
    route = build_route(links)
    outputs = {}
    records = []
    for definition in STATIC_2:
        name, scene, metadata = make_static_scene(base, shade, route, templates, definition, "static_2")
        validate_scene(scene, metadata)
        relative = Path("static_2") / f"{name}.json"
        outputs[relative] = dump_json(scene)
        records.append({"file": relative.as_posix(), "scenario_id": name, **metadata})
    for definition in STATIC_3:
        name, scene, metadata = make_static_scene(base, shade, route, templates, definition, "static_3")
        validate_scene(scene, metadata)
        relative = Path("static_3") / f"{name}.json"
        outputs[relative] = dump_json(scene)
        records.append({"file": relative.as_posix(), "scenario_id": name, **metadata})
    for definition in DYNAMIC_1:
        name, scene, metadata = make_dynamic_scene(base, shade, route, templates, definition)
        validate_scene(scene, metadata)
        relative = Path("dynamic_1") / f"{name}.json"
        outputs[relative] = dump_json(scene)
        records.append({"file": relative.as_posix(), "scenario_id": name, **metadata})
    manifest = {
        "schema_version": 1,
        "map_name": base["mapInfo"]["mapName"],
        "source_files": {"start_pose": BASE_PATH.name, "manual_static_reference": HID1_PATH.name, "official_shade_and_actor_templates": str(OFFICIAL_PATH.relative_to(ROOT)), "mgeo_links": str(MGEO_LINKS_PATH.relative_to(ROOT))},
        "source_sha256": {"start_pose": sha256(BASE_PATH), "manual_static_reference": sha256(HID1_PATH), "official_sample": sha256(OFFICIAL_PATH), "mgeo_links": sha256(MGEO_LINKS_PATH)},
        "route_links": list(ROUTE_LINK_IDS),
        "ego_centerline_projection_error_m": route[4],
        "scenario_counts": {"static_2": len(STATIC_2), "static_3": len(STATIC_3), "dynamic_1": len(DYNAMIC_1), "total": len(records)},
        "speed_note": "speed and moveSpeed are preserved as MORAI scenario configuration values; verify the simulator version's displayed unit after import",
        "scenarios": records,
    }
    outputs[Path("manifest.json")] = dump_json(manifest)
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="verify generated files without rewriting them")
    args = parser.parse_args()
    base = load_json(BASE_PATH)
    official = load_json(OFFICIAL_PATH)
    hid1 = load_json(HID1_PATH)
    links = load_json(MGEO_LINKS_PATH)
    outputs = expected_outputs(base, official, hid1, links)
    if args.check:
        failures = []
        for relative, content in outputs.items():
            path = HERE / relative
            if not path.is_file():
                failures.append(f"missing: {relative}")
            elif path.read_text(encoding="utf-8") != content:
                failures.append(f"stale: {relative}")
        generated = {path.relative_to(HERE) for family in ("static_2", "static_3", "dynamic_1") for path in (HERE / family).glob("*.json")}
        unexpected = sorted(generated - set(outputs))
        failures.extend(f"unexpected: {path}" for path in unexpected)
        if failures:
            raise SystemExit("\n".join(failures))
        print(f"OK: {len(outputs) - 1} scenarios and manifest are current")
        return
    for relative, content in outputs.items():
        path = HERE / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    print(f"generated {len(outputs) - 1} scenarios under {HERE}")


if __name__ == "__main__":
    main()
