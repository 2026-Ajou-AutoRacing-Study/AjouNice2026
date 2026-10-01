#!/usr/bin/env python3
"""Build reproducible non-shadow static-obstacle planning scenes from the Base3 benchmark."""

import argparse
import copy
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[5]
SOURCE = ROOT / "configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json"
OFFICIAL_SOURCE = ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
DEFAULT_OUTPUT = ROOT / "planner_logs/validation_inputs/static_corridor"

# Goals are v85 map-frame centerline points at route arcs 305, 335 and 385 m.
# Source MGeo poses are transformed with configs/tf/morai_lanelet_static_tf.yaml.
CASES = (
    ("soa_corridor_00_clear", 0, (71.092, 379.631, 28.396, 84.901), 80),
    ("soa_corridor_01_cargobox", 1, (71.092, 379.631, 28.396, 84.901), 100),
    ("soa_corridor_03_mixed", 3, (71.694, 409.612, 28.384, 90.096), 130),
    ("soa_corridor_06_mixed", 6, (71.704, 459.605, 28.379, 90.131), 180),
)
EXPECTED_IDS = (40100001, 40100027, 40100005, 40200162, 40100047, 40100028)
OFFICIAL_GOAL = (71.510, 429.608, 28.383, 90.033)  # v85 map, route arc 355 m


def _write_case(output, stem, scene, goal, timeout, description):
    path = output / (stem + ".json")
    path.write_text(json.dumps(scene, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    x, y, z, yaw = goal
    (output / (stem + ".case.yaml")).write_text(
        "description: {}\n"
        "goal: {{x: {:.3f}, y: {:.3f}, z: {:.3f}, yaw_deg: {:.3f}}}\n"
        "waypoints: []\n"
        "timeout_sec: {}\n"
        "arrival_mode: stop\n".format(description, x, y, z, yaw, timeout),
        encoding="utf-8",
    )
    print(path)


def _vector(x, y, z):
    return {"x": x, "y": y, "z": z,
            "_x": "{:.3f}".format(x), "_y": "{:.3f}".format(y), "_z": "{:.3f}".format(z)}


def _official_scenes(output, source=OFFICIAL_SOURCE):
    scene = json.loads(Path(source).read_text(encoding="utf-8"))
    if len(scene["objectList"]) != 1 or int(scene["objectList"][0]["DataID"]) != 40100001:
        raise ValueError("Official sample no longer contains exactly one CargoBox")
    # MGeo route arc 285 m: 45 m before the official sample CargoBox near arc 330 m.
    # Leave the object's published sample pose/scale unchanged for the scaled case.
    ego = scene["egoVehicle"]
    ego["currentVelocity"] = 0.0
    initial = ego["initPosition"]
    initial["initPositionMode"] = "Absolute"
    initial["pos"] = _vector(-63.195, -186.079, 28.736)
    initial["rot"] = {"roll": "0.000", "pitch": "0.000", "yaw": "78.294"}
    initial["initLink"] = "A2256W000213"
    initial["initLinkRatio"] = 0.6060807181098155
    initial["gear"] = 1
    scene["vehicleList"] = []
    scene["pedestrianList"] = []
    scene["spawnPointList"] = []
    scene["pedspawnPointList"] = []
    scene["aiPedstrianList"] = []
    scene["shadedAreaList"] = []
    for stem, objects, scale in (
            ("soa_official_00_clear", 0, None),
            ("soa_official_01_cargobox_native", 1, 1.0),
            ("soa_official_01_cargobox_scaled", 1, None)):
        target = copy.deepcopy(scene)
        target["objectList"] = target["objectList"][:objects]
        if scale is not None:
            target["objectList"][0]["scale"] = _vector(scale, scale, scale)
        _write_case(output, stem, target, OFFICIAL_GOAL, 100,
                    "샘플 정적 장애물 구간 / {} / GPS 음영 없음".format(stem))


def build(source=SOURCE, output=DEFAULT_OUTPUT):
    scene = json.loads(Path(source).read_text(encoding="utf-8"))
    objects = scene["objectList"]
    if tuple(int(obj["DataID"]) for obj in objects) != EXPECTED_IDS:
        raise ValueError("Base3 benchmark object catalog/order changed; review scene selection")
    for key in ("vehicleList", "pedestrianList", "spawnPointList", "pedspawnPointList", "aiPedstrianList"):
        if scene.get(key):
            raise ValueError("Benchmark contains unexpected moving actors: {}".format(key))
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    for stem, count, goal, timeout in CASES:
        target = copy.deepcopy(scene)
        target["objectList"] = copy.deepcopy(objects[:count])
        # This corridor is far from the official GPS shadow. Remove shade
        # metadata as well, so localization never becomes a hidden variable.
        target["shadedAreaList"] = []
        _write_case(output, stem, target, goal, timeout,
                    "일반 정적 장애물 구간 / 객체 {}개 / GPS 음영 없음".format(count))
    _official_scenes(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    build(args.source, args.output)


if __name__ == "__main__":
    main()
