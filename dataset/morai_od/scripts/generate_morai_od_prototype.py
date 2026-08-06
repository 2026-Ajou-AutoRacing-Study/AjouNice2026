#!/usr/bin/env python3
"""Generate one loadable MORAI OD scenario from official route/MGeo data.

This is a prototype for validating the Internal Scenario JSON workflow.  It
clones actor records saved by MORAI itself instead of inventing undocumented
engine fields, then replaces IDs, poses, link references, and motion state.
Generated scenario JSON and reproducibility metadata are written into the
monorepo. Large sensor captures remain outside Git under AJOUNICE_DATA_ROOT.
"""

import argparse
import copy
import json
import math
import random
from pathlib import Path


DEFAULT_SEED = 260806
DEFAULT_ANCHOR_INDEX = 2516
DEFAULT_RADIUS_M = 20.0


def repository_relative_path(path):
    resolved = path.resolve()
    project_root = Path(__file__).resolve().parents[3]
    try:
        return str(resolved.relative_to(project_root))
    except ValueError:
        return str(resolved)


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    dataset_workspace = project_root / "dataset/morai_od"
    parser = argparse.ArgumentParser(
        description="Generate a prototype MORAI object-detection scenario."
    )
    parser.add_argument(
        "--template",
        type=Path,
        default=project_root
        / "configs/scenario/2026_molit_comp_sample_scene.json",
    )
    parser.add_argument(
        "--waypoints",
        type=Path,
        default=project_root
        / "configs/global_path/2026_molit_comp_global_path.txt",
    )
    parser.add_argument(
        "--mgeo-dir",
        type=Path,
        default=project_root / "kcity_map/mgeo_map",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=dataset_workspace,
        help=(
            "MORAI OD workspace containing scenarios/ and metadata/ "
            "(default: dataset/morai_od)."
        ),
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--anchor-index", type=int, default=DEFAULT_ANCHOR_INDEX)
    parser.add_argument("--radius", type=float, default=DEFAULT_RADIUS_M)
    parser.add_argument("--npc-count", type=int, default=4)
    parser.add_argument("--pedestrian-count", type=int, default=3)
    parser.add_argument("--obstacle-count", type=int, default=2)
    parser.add_argument(
        "--weather",
        choices=("sunny", "foggy"),
        default="sunny",
        help="Recorded in the sidecar only; Internal Scenario JSON has no field.",
    )
    parser.add_argument(
        "--time-hour",
        type=int,
        choices=(11, 13, 15),
        default=13,
        help="Recorded in the sidecar only; Internal Scenario JSON has no field.",
    )
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_waypoints(path):
    points = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            values = stripped.split()
            if len(values) != 3:
                raise ValueError(
                    f"{path}:{line_number}: expected x y z, got {stripped!r}"
                )
            points.append(tuple(float(value) for value in values))
    if len(points) < 3:
        raise ValueError(f"Not enough waypoints in {path}")
    return points


def wrap_degrees(angle):
    return (angle + 180.0) % 360.0 - 180.0


def heading_degrees(before, after):
    return wrap_degrees(
        math.degrees(math.atan2(after[1] - before[1], after[0] - before[0]))
    )


def make_pose_dict(x, y, z):
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "_x": f"{x:.3f}",
        "_y": f"{y:.3f}",
        "_z": f"{z:.3f}",
    }


def make_rotation_dict(yaw):
    return {
        "roll": "0.000",
        "pitch": "0.000",
        "yaw": f"{wrap_degrees(yaw):.3f}",
    }


def build_link_samples(links):
    samples = []
    for link in links:
        points = link.get("points", [])
        for point_index, point in enumerate(points):
            if len(points) == 1:
                yaw = 0.0
            else:
                before = points[max(0, point_index - 1)]
                after = points[min(len(points) - 1, point_index + 1)]
                yaw = heading_degrees(before, after)
            samples.append(
                {
                    "x": float(point[0]),
                    "y": float(point[1]),
                    "z": float(point[2]),
                    "yaw": yaw,
                    "link_idx": str(link["idx"]),
                    "point_idx": point_index,
                    "point_ratio": (
                        point_index / (len(points) - 1)
                        if len(points) > 1
                        else 0.0
                    ),
                    "max_speed": float(link.get("max_speed") or 0.0),
                }
            )
    if not samples:
        raise ValueError("MGeo contains no link points")
    return samples


def distance_xy(left, right):
    return math.hypot(left["x"] - right["x"], left["y"] - right["y"])


def nearest_sample(samples, x, y):
    return min(
        samples,
        key=lambda sample: (sample["x"] - x) ** 2
        + (sample["y"] - y) ** 2,
    )


def offset_sample(sample, lateral_offset):
    yaw_rad = math.radians(sample["yaw"])
    return {
        **sample,
        "x": sample["x"] - math.sin(yaw_rad) * lateral_offset,
        "y": sample["y"] + math.cos(yaw_rad) * lateral_offset,
    }


def zero_vector(vector):
    if not isinstance(vector, dict):
        return
    for key in ("x", "y", "z"):
        if key in vector:
            vector[key] = 0.0


def zero_vehicle_save_data(init_position):
    save_data = init_position.get("vehicleSaveData")
    if not isinstance(save_data, dict):
        return
    save_data["velocity"] = 0.0
    zero_vector(save_data.get("lastVelocity"))
    vehicle = save_data.get("vehicle")
    if isinstance(vehicle, dict):
        zero_vector(vehicle.get("lastVelocity"))


def set_vehicle_pose(
    actor,
    sample,
    unique_id,
    z_lift,
    ego=False,
):
    actor["UNIQUEID"] = unique_id
    init_position = actor["initPosition"]
    init_position["initPositionMode"] = "Absolute"
    init_position["pos"] = make_pose_dict(
        sample["x"],
        sample["y"],
        sample["z"] + z_lift,
    )
    init_position["rot"] = make_rotation_dict(sample["yaw"])
    init_position["initLink"] = sample["link_idx"]
    init_position["initLinkRatio"] = sample["point_ratio"]
    init_position["gear"] = 1 if ego else 4
    zero_vehicle_save_data(init_position)

    actor["currentVelocity"] = 0.0
    if "viewCurrentVelocity" in actor:
        actor["viewCurrentVelocity"] = 0.0
    if "constantVelocity" in actor:
        actor["constantVelocity"] = 0.0
    if "desiredVelocity" in actor:
        actor["desiredVelocity"] = 0.0
    if "linkVelocity" in actor:
        actor["linkVelocity"] = sample["max_speed"]

    link_info = {
        "linkIdx": sample["link_idx"],
        "pointIdx": sample["point_idx"],
    }
    for key in (
        "startPointLinkInfo",
        "endPointLinkInfo",
        "currentLinkInfo",
        "loopStartLinkInfo",
    ):
        if key in actor:
            actor[key] = dict(link_info)

    if not ego:
        actor["isCloseLoop"] = False
        actor["isLaneChange"] = False
        actor["isEgo"] = False
        actor["spawnPointUniqueID"] = -1
        actor["destinationUniqueID"] = -1
        actor["isWaypointMode"] = False
        actor["wayPointInfos"] = []
        actor["bias"] = 0.0


def set_pedestrian_pose(actor, pose, yaw, unique_id):
    actor["UNIQUEID"] = unique_id
    actor["pos"] = make_pose_dict(pose["x"], pose["y"], pose["z"])
    actor["rot"] = make_rotation_dict(yaw)

    # MORAI's serialized pedestrian state uses Unity's internal axis order.
    internal_pose = make_pose_dict(-pose["x"], pose["z"], -pose["y"])
    actor["initPos"] = copy.deepcopy(internal_pose)
    actor["standardPos"] = copy.deepcopy(internal_pose)
    actor["initRot"] = make_rotation_dict(0.0)

    actor["activeDistance"] = 100.0
    actor["speed"] = 0.0
    actor["active"] = True
    actor["loop"] = False
    actor["movingDistance"] = 0.0
    actor["movingDistanceAmount"] = 0.0
    actor["animSpeed"] = 0.0
    actor["isWaypointMode"] = False
    actor["waypointEventList"] = []


def set_obstacle_pose(actor, sample, unique_id, scale):
    actor["UNIQUEID"] = unique_id
    actor["pos"] = make_pose_dict(
        sample["x"],
        sample["y"],
        sample["z"] + 0.03,
    )
    actor["rot"] = make_rotation_dict(sample["yaw"])
    actor["scale"] = make_pose_dict(scale, scale, scale)
    actor["initPos"] = make_pose_dict(0.0, 0.0, 0.0)
    actor["initRot"] = make_rotation_dict(0.0)
    actor["isWaypointMode"] = False
    actor["moveSpeed"] = 0.0
    actor["activeDistance"] = 0.0
    actor["waypointEventList"] = []


def is_clear(candidate, occupied, required_clearance):
    return all(
        math.hypot(candidate["x"] - item["x"], candidate["y"] - item["y"])
        >= required_clearance + item["radius"]
        for item in occupied
    )


def select_road_samples(
    rng,
    all_samples,
    ego_sample,
    count,
    radius,
    occupied,
    actor_radius,
    clearance,
):
    candidates = []
    for raw_sample in all_samples:
        distance = distance_xy(raw_sample, ego_sample)
        if not 5.5 <= distance <= radius:
            continue
        lateral = rng.uniform(-0.35, 0.35)
        sample = offset_sample(raw_sample, lateral)
        if is_clear(sample, occupied, actor_radius + clearance):
            candidates.append(sample)

    rng.shuffle(candidates)
    selected = []
    for candidate in candidates:
        if not is_clear(candidate, occupied, actor_radius + clearance):
            continue
        selected.append(candidate)
        occupied.append(
            {
                "x": candidate["x"],
                "y": candidate["y"],
                "radius": actor_radius,
            }
        )
        if len(selected) == count:
            return selected
    raise RuntimeError(
        f"Could place only {len(selected)}/{count} road actors "
        f"inside {radius:.1f} m"
    )


def select_pedestrian_poses(
    rng,
    ego_sample,
    all_samples,
    count,
    radius,
    occupied,
):
    yaw_rad = math.radians(ego_sample["yaw"])
    forward = (math.cos(yaw_rad), math.sin(yaw_rad))
    left = (-math.sin(yaw_rad), math.cos(yaw_rad))
    selected = []

    for _ in range(300):
        if len(selected) == count:
            break
        longitudinal = rng.uniform(-16.0, 16.0)
        if abs(longitudinal) < 5.0:
            longitudinal = math.copysign(5.0, longitudinal or 1.0)
        lateral = rng.choice((-1.0, 1.0)) * rng.uniform(4.5, 7.0)
        x = (
            ego_sample["x"]
            + forward[0] * longitudinal
            + left[0] * lateral
        )
        y = (
            ego_sample["y"]
            + forward[1] * longitudinal
            + left[1] * lateral
        )
        if math.hypot(x - ego_sample["x"], y - ego_sample["y"]) > radius:
            continue
        road_sample = nearest_sample(all_samples, x, y)
        pose = {"x": x, "y": y, "z": road_sample["z"] + 0.05}
        if not is_clear(pose, occupied, required_clearance=1.0):
            continue
        occupied.append({"x": x, "y": y, "radius": 0.45})
        selected.append(
            {
                **pose,
                "yaw": wrap_degrees(
                    ego_sample["yaw"] + (90.0 if lateral > 0.0 else -90.0)
                ),
                "placement": "road_edge_prototype",
            }
        )

    if len(selected) != count:
        raise RuntimeError(
            f"Could place only {len(selected)}/{count} pedestrians"
        )
    return selected


def validate_scenario(
    scenario,
    ego_sample,
    radius,
    link_ids,
    expected_counts,
):
    problems = []
    counts = {
        "vehicleList": len(scenario["vehicleList"]),
        "pedestrianList": len(scenario["pedestrianList"]),
        "objectList": len(scenario["objectList"]),
    }
    if counts != expected_counts:
        problems.append(f"actor counts differ: {counts} != {expected_counts}")

    unique_ids = []
    for group in ("vehicleList", "pedestrianList", "objectList"):
        for actor in scenario[group]:
            unique_ids.append(actor["UNIQUEID"])
    if len(unique_ids) != len(set(unique_ids)):
        problems.append("actor UNIQUEID values are not unique")

    for actor in scenario["vehicleList"]:
        info = actor.get("currentLinkInfo", {})
        if info.get("linkIdx") not in link_ids:
            problems.append(
                f"NPC {actor['UNIQUEID']} has unknown link "
                f"{info.get('linkIdx')}"
            )

    positions = []
    for actor in scenario["vehicleList"]:
        positions.append(("npc", actor["UNIQUEID"], actor["initPosition"]["pos"]))
    for actor in scenario["pedestrianList"]:
        positions.append(("pedestrian", actor["UNIQUEID"], actor["pos"]))
    for actor in scenario["objectList"]:
        positions.append(("obstacle", actor["UNIQUEID"], actor["pos"]))

    for group, unique_id, position in positions:
        distance = math.hypot(
            position["x"] - ego_sample["x"],
            position["y"] - ego_sample["y"],
        )
        if distance > radius + 0.1:
            problems.append(
                f"{group} {unique_id} lies {distance:.2f} m from Ego"
            )

    if problems:
        raise ValueError("Scenario validation failed:\n- " + "\n- ".join(problems))
    return counts


def generate(args):
    if args.radius <= 6.0:
        raise ValueError("--radius must be greater than 6 m")
    for count_name in ("npc_count", "pedestrian_count", "obstacle_count"):
        if getattr(args, count_name) < 0:
            raise ValueError(f"--{count_name.replace('_', '-')} must be >= 0")

    template = load_json(args.template)
    waypoints = load_waypoints(args.waypoints)
    links = load_json(args.mgeo_dir / "link_set.json")
    samples = build_link_samples(links)
    link_ids = {str(link["idx"]) for link in links}

    if not 0 <= args.anchor_index < len(waypoints):
        raise IndexError(
            f"anchor index {args.anchor_index} outside "
            f"0..{len(waypoints) - 1}"
        )

    rng = random.Random(args.seed)
    anchor = waypoints[args.anchor_index]
    ego_sample = nearest_sample(samples, anchor[0], anchor[1])
    before = waypoints[(args.anchor_index - 1) % len(waypoints)]
    after = waypoints[(args.anchor_index + 1) % len(waypoints)]
    ego_sample = {
        **ego_sample,
        "x": anchor[0],
        "y": anchor[1],
        "z": anchor[2],
        "yaw": heading_degrees(before, after),
    }

    scenario = copy.deepcopy(template)
    scenario["spawnPointList"] = []
    scenario["pedspawnPointList"] = []
    scenario["aiPedstrianList"] = []
    scenario["shadedAreaList"] = []
    scenario["frictionChangeMaterialList"] = []
    scenario["waypointDataList"] = []

    set_vehicle_pose(
        scenario["egoVehicle"],
        ego_sample,
        unique_id=-1,
        z_lift=0.34,
        ego=True,
    )
    scenario["egoVehicle"]["customValue"] = 0.0
    scenario["egoVehicle"]["linkValue"] = ego_sample["max_speed"]

    occupied = [
        {
            "x": ego_sample["x"],
            "y": ego_sample["y"],
            "radius": 3.2,
        }
    ]

    npc_samples = select_road_samples(
        rng=rng,
        all_samples=samples,
        ego_sample=ego_sample,
        count=args.npc_count,
        radius=args.radius,
        occupied=occupied,
        actor_radius=2.7,
        clearance=0.8,
    )
    npc_templates = template.get("vehicleList", [])
    if args.npc_count and not npc_templates:
        raise ValueError("Template has no NPC vehicle records")
    scenario["vehicleList"] = []
    vehicle_lifts = {
        30100004: 0.30,
        30100005: 0.36,
        30100010: 0.32,
        30100014: 0.32,
        30200003: 0.33,
        30300002: 0.35,
        30400004: 0.55,
        30700003: 0.35,
    }
    for index, sample in enumerate(npc_samples):
        actor = copy.deepcopy(npc_templates[index % len(npc_templates)])
        set_vehicle_pose(
            actor,
            sample,
            unique_id=1001 + index,
            z_lift=vehicle_lifts.get(actor.get("DataID"), 0.34),
            ego=False,
        )
        scenario["vehicleList"].append(actor)

    pedestrian_poses = select_pedestrian_poses(
        rng=rng,
        ego_sample=ego_sample,
        all_samples=samples,
        count=args.pedestrian_count,
        radius=args.radius,
        occupied=occupied,
    )
    pedestrian_templates = template.get("pedestrianList", [])
    if args.pedestrian_count and not pedestrian_templates:
        raise ValueError("Template has no pedestrian records")
    scenario["pedestrianList"] = []
    for index, pose in enumerate(pedestrian_poses):
        actor = copy.deepcopy(
            pedestrian_templates[index % len(pedestrian_templates)]
        )
        set_pedestrian_pose(
            actor,
            pose,
            yaw=pose["yaw"],
            unique_id=2001 + index,
        )
        scenario["pedestrianList"].append(actor)

    obstacle_samples = select_road_samples(
        rng=rng,
        all_samples=samples,
        ego_sample=ego_sample,
        count=args.obstacle_count,
        radius=args.radius,
        occupied=occupied,
        actor_radius=1.2,
        clearance=0.7,
    )
    obstacle_templates = template.get("objectList", [])
    if args.obstacle_count and not obstacle_templates:
        raise ValueError("Template has no obstacle records")
    scenario["objectList"] = []
    for index, sample in enumerate(obstacle_samples):
        actor = copy.deepcopy(
            obstacle_templates[index % len(obstacle_templates)]
        )
        set_obstacle_pose(
            actor,
            sample,
            unique_id=3001 + index,
            scale=rng.uniform(0.9, 1.1),
        )
        scenario["objectList"].append(actor)

    expected_counts = {
        "vehicleList": args.npc_count,
        "pedestrianList": args.pedestrian_count,
        "objectList": args.obstacle_count,
    }
    counts = validate_scenario(
        scenario=scenario,
        ego_sample=ego_sample,
        radius=args.radius,
        link_ids=link_ids,
        expected_counts=expected_counts,
    )

    scenario_dir = args.output_root / "scenarios/prototypes"
    metadata_dir = args.output_root / "metadata/prototypes"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    metadata_dir.mkdir(parents=True, exist_ok=True)
    stem = (
        f"morai_od_prototype_a{args.anchor_index:04d}_"
        f"{args.weather}_t{args.time_hour:02d}_seed{args.seed}"
    )
    scenario_path = scenario_dir / f"{stem}.json"
    metadata_path = metadata_dir / f"{stem}.metadata.json"

    with scenario_path.open("w", encoding="utf-8") as stream:
        json.dump(scenario, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    metadata = {
        "prototype_version": 0,
        "scenario_file": scenario_path.name,
        "template": repository_relative_path(args.template),
        "waypoints": repository_relative_path(args.waypoints),
        "mgeo_dir": repository_relative_path(args.mgeo_dir),
        "seed": args.seed,
        "anchor_index": args.anchor_index,
        "ego": {
            "x": ego_sample["x"],
            "y": ego_sample["y"],
            "z": ego_sample["z"] + 0.34,
            "yaw": ego_sample["yaw"],
            "link_idx": ego_sample["link_idx"],
            "point_idx": ego_sample["point_idx"],
        },
        "radius_m": args.radius,
        "weather": args.weather,
        "time_hour": args.time_hour,
        "environment_note": (
            "Internal Scenario JSON does not set weather/time. "
            "Apply these values in MORAI UI or a validated Scenario Runner."
        ),
        "foggy_density": 1 if args.weather == "foggy" else None,
        "foggy_distance": 0 if args.weather == "foggy" else None,
        "counts": counts,
        "pedestrian_placement": (
            "Prototype road-edge placement. GIS/Lanelet crosswalk constraints "
            "will be added before bulk generation."
        ),
    }
    with metadata_path.open("w", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    # Re-open the file so serialization errors cannot hide behind in-memory data.
    reloaded = load_json(scenario_path)
    validate_scenario(
        scenario=reloaded,
        ego_sample=ego_sample,
        radius=args.radius,
        link_ids=link_ids,
        expected_counts=expected_counts,
    )

    print("MORAI OD prototype scenario generated")
    print(f"scenario: {scenario_path}")
    print(f"metadata: {metadata_path}")
    print(
        "ego: "
        f"anchor={args.anchor_index}, "
        f"link={ego_sample['link_idx']}, "
        f"position=({ego_sample['x']:.3f}, "
        f"{ego_sample['y']:.3f}, {ego_sample['z'] + 0.34:.3f}), "
        f"yaw={ego_sample['yaw']:.3f} deg"
    )
    print(
        "actors: "
        f"npc={counts['vehicleList']}, "
        f"pedestrian={counts['pedestrianList']}, "
        f"obstacle={counts['objectList']}"
    )
    print(
        f"environment sidecar: {args.weather}, {args.time_hour}:00 "
        "(not embedded in Internal Scenario JSON)"
    )
    print("static validation: OK")


def main():
    generate(parse_args())


if __name__ == "__main__":
    main()
