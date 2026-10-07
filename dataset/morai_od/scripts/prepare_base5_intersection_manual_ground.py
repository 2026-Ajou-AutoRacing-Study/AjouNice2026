#!/usr/bin/env python3
"""Prepare editor-grounded paths, not training scenes; never loads MR itself."""
import argparse
import copy
import math
from pathlib import Path

import generate_base5_independent as gen
from base5_map_surface import configure, ego_road_support
from base5_static_height import saved_height_candidate

ASSETS = ("Woman2", "Dog1", "Dog2", "Deer1", "Ped_Pet1")
OFFSETS = (-12., -6., 0., 6., 12.)
NETWORK = gen.geo.PROJECT_ROOT / "configs/network/ego_network_mr_base5_savesensordata_collision_local.json"
CONFIG = gen.geo.PROJECT_ROOT / "dataset/morai_od/configs/base5_independent_5k_sunny13.json"


def constant_path(crossing, z):
    """Normalize the crossing axis, NOT heights onto an invented flat surface."""
    dx, dy = crossing["axis"]
    norm = math.hypot(dx, dy)
    if norm <= 0 or not math.isfinite(norm) or not math.isfinite(z):
        raise ValueError("Finite Z and nonzero crossing axis required")
    dx, dy = dx / norm, dy / norm
    if dx < 0:
        dx, dy = -dx, -dy
    center = crossing["center"]
    points = [dict(x=center["x"] + s * dx, y=center["y"] + s * dy, z=z)
              for s in OFFSETS]
    return points, (dx, dy)


def park(ego):
    ego["initPosition"]["gear"] = 1
    ego["currentVelocity"] = ego["customValue"] = 0.
    saved = ego["initPosition"].get("vehicleSaveData", {})
    saved["velocity"] = 0.
    for vector in (saved.get("lastVelocity"), saved.get("vehicle", {}).get("lastVelocity")):
        gen.geo.zero_vector(vector)
    inputs = saved.get("inputData")
    if isinstance(inputs, list) and len(inputs) > 6:
        inputs[6] = 1


def build_scene(ctx, crossing, asset, number):
    candidate = saved_height_candidate(asset, ctx)
    offset = candidate.get("pivot_height_candidate_m", 0.) if candidate["usable"] else 0.
    z = crossing["center"]["z"] + offset
    points, axis = constant_path(crossing, z)
    pose = gen.geo.continuous_route_pose_at_arc(ctx["route"], ctx["arc"], crossing["route_arc_m"] - 20.)
    support = ego_road_support(ctx["map_surface_index"], pose)
    if not support["road_supported"]:
        raise ValueError("Calibration Ego must have mapped road support")
    pose["z"] = support["height_m"]
    scene = copy.deepcopy(ctx["template"])
    gen.geo.clear_actor_lists(scene)
    gen.geo.set_ego_pose(scene["egoVehicle"], pose, 0.)
    park(scene["egoVehicle"])
    speed = 1.2 if asset.get("ontology_class") == "Animal" else 1.1
    path = dict(policy="natural-target-v1", points=points, nominal_speed_mps=speed,
                speed_field_value=speed * 3.6)
    gen.set_pose(copy.deepcopy(asset["template"]), dict(points[0], yaw=math.degrees(math.atan2(axis[1], axis[0]))),
                 1, asset, scene, "walking", path)
    # Screen the whole swept path, not merely start/end positions.
    from shapely.geometry import LineString
    size, center_offset, _ = gen.geometry(asset, ctx)
    ego_box = gen.rectangle([pose["x"] + 1.35 * math.cos(math.radians(pose["yaw"])),
                             pose["y"] + 1.35 * math.sin(math.radians(pose["yaw"]))], [4.8, 2.], pose["yaw"])
    clearance = LineString([(p["x"], p["y"]) for p in points]).distance(ego_box)
    if clearance <= math.hypot(*size[:2]) / 2 + math.hypot(*center_offset[:2]) + 1.:
        raise ValueError("Calibration path approaches parked Ego")
    name = f"0_{number}_{asset['name']}_{crossing['crosswalk_id']}"
    row = dict(number=number, name=name, original_file=name + ".json", corrected_file=f"{number}.json",
               asset=asset["name"], data_id=asset["data_id"], unique_id=1,
               source_list=asset["source_list"], crosswalk=copy.deepcopy(crossing),
               axis=list(axis), local_offsets_m=list(OFFSETS), world_waypoints=points,
               nominal_speed_mps=speed, provisional_constant_z_m=z,
               pivot_initializer=candidate, ego_map_support=support,
               ego_path_clearance_m=clearance, calibration_only=True,
               ground_verified=False, status="WAITING_FOR_MANUAL_EDITOR_GROUNDING")
    return scene, row


def install_files(pairs):
    """Refuse even identical pre-existing files; preserve every unrelated scenario."""
    destinations = [dst for _, dst in pairs]
    if len(set(destinations)) != len(destinations):
        raise ValueError("Duplicate installation destination")
    for dst in destinations:
        if dst.exists():
            raise FileExistsError(f"Installation overwrite refused: {dst}")
    written = []
    try:
        for src, dst in pairs:
            dst.parent.mkdir(parents=True, exist_ok=True)
            with dst.open("xb") as stream:
                written.append(dst)
                stream.write(src.read_bytes())
    except BaseException:
        # Only roll back files exclusively created by this invocation.
        for dst in written:
            dst.unlink()
        raise


def prepare(output, runtime, mgeo, install=False):
    if output.exists():
        raise FileExistsError(f"Output overwrite refused: {output}")
    config = gen.geo.load_json(CONFIG)
    ctx = configure(gen.context(runtime, mgeo), config["placement_ground"])
    crossing_rows = sorted(gen.dynamic_crossing_candidates(ctx, ctx["mission_templates"]["dynamic_obstacle"][0][1],
                           config["mission_dynamic_observation"]), key=lambda c: c["route_arc_m"])
    if {c["crosswalk_id"] for c in crossing_rows} != {"CW000000", "CW000002"}:
        raise ValueError("Expected exactly the two current mission crosswalks")
    assets = {a["name"]: a for pool in ctx["pools"].values() for a in pool if a["name"] in ASSETS}
    rows, scenes = [], []
    for crossing in crossing_rows:
        for name in ASSETS:
            scene, row = build_scene(ctx, crossing, assets[name], len(rows) + 1)
            scenes.append(scene)
            rows.append(row)
    # Protect user's intended corrected names too, before creating any output.
    if install:
        for row in rows:
            for dst in (runtime / row["original_file"], runtime / row["corrected_file"],
                        runtime / "EgoNetwork" / (row["name"] + "_MN.json"),
                        runtime / "EgoNetwork" / (str(row["number"]) + "_MN.json")):
                if dst.exists():
                    raise FileExistsError(f"Manual calibration filename already occupied: {dst}")
    output.mkdir(parents=True)
    pairs = []
    network = gen.geo.load_json(NETWORK)
    for row, scene in zip(rows, scenes):
        scenario = output / row["original_file"]
        sidecar = output / "EgoNetwork" / (row["name"] + "_MN.json")
        sidecar.parent.mkdir(exist_ok=True)
        gen.geo.write_json(scenario, scene)
        gen.geo.write_json(sidecar, network)
        row.update(scenario_sha256=gen.geo.sha256(scenario), sidecar_sha256=gen.geo.sha256(sidecar))
        pairs += [(scenario, runtime / scenario.name), (sidecar, runtime / "EgoNetwork" / sidecar.name)]
    manifest = dict(schema_version=1, purpose="EDITOR_GROUND_CALIBRATION_ONLY_NOT_TRAINING",
                    status="WAITING_FOR_MANUAL_EDITOR_GROUNDING", environment="sunny13_user_session_preserved",
                    runtime=str(runtime.resolve()), config_sha256=gen.geo.sha256(CONFIG),
                    reference_sources=ctx["static_height_reference_sources"],
                    network_sha256=gen.geo.sha256(NETWORK), count=len(rows), scenes=rows,
                    installation_requested=install, live_loaded=False, contact_verified=False)
    gen.geo.write_json(output / "calibration_manifest.json", manifest)
    if install:
        install_files(pairs)
    return output / "calibration_manifest.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runtime-dir", type=Path, default=gen.geo.DEFAULT_RUNTIME_ASSET_DIR)
    parser.add_argument("--mgeo-dir", type=Path, default=gen.geo.DEFAULT_MGEO)
    parser.add_argument("--install", action="store_true", help="Install only; never calls ScenarioLoad")
    args = parser.parse_args()
    print(prepare(args.output, args.runtime_dir, args.mgeo_dir, args.install))


if __name__ == "__main__":
    main()
