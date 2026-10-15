#!/usr/bin/env python3
"""Freeze moving-Ego QA bags for the existing E24 offline inference entrypoint.

Native ROS XYZI/JPEG are retained. Catalog-corrected, reconstructed Object GT
is a diagnostic reference, not native annotation, visibility ground truth or
AP. Physical heading applies the explicit -1.347-degree MORAI convention;
recorded operational TF is retained separately, never used to redefine GT.
"""

import argparse
import bisect
from collections import Counter, defaultdict
from datetime import datetime
import hashlib
import gzip
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAMERAS = ("front", "front_left", "front_right", "back")
CLASSES = ("vehicle", "pedestrian", "obstacle")
DEFAULT_CATALOG = ROOT / "dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json"


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for blob in iter(lambda: stream.read(1048576), b""):
            h.update(blob)
    return h.hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def normalize(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def causal_camera(rows, stamp, cloud_receipt, tolerance=.10):
    """Nearest header among cameras already in the recorded receive history.

    Bag receipt ordering approximates callback availability, not its exact
    thread scheduling. A future *header* is legal when its message was already
    received; a camera received after this cloud is never borrowed offline.
    """
    candidates = [row for row in rows if row[1] <= cloud_receipt + 1.e-9
                  and abs(row[0] - stamp) <= tolerance + 1.e-9]
    return min(candidates, key=lambda row: (abs(row[0]-stamp), row[0], row[1]), default=None)


def bracket_pose(rows, timestamp, max_gap=.15):
    """Interpolate XYZ/yaw only inside a supported bracket; never extrapolate."""
    if not rows:
        return None
    right = bisect.bisect_left([row[0] for row in rows], timestamp)
    if right < len(rows) and abs(rows[right][0] - timestamp) < 1.e-9:
        return dict(rows[right][1])
    if right == 0 or right == len(rows):
        return None
    lo, before = rows[right-1]
    hi, after = rows[right]
    if not 0 < hi-lo <= max_gap:
        return None
    fraction = (timestamp-lo)/(hi-lo)
    result = {key: before[key]+fraction*(after[key]-before[key]) for key in "xyz"}
    result["yaw"] = normalize(before["yaw"] + fraction*normalize(after["yaw"]-before["yaw"]))
    return result


def pose_matrix(pose):
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    result = np.eye(4)
    result[:2, :2] = [[c, -s], [s, c]]
    result[:3, 3] = [pose[key] for key in "xyz"]
    return result


def transform_pose(pose, alignment):
    transformed = pose_matrix(alignment) @ pose_matrix(pose)
    return dict(x=float(transformed[0, 3]), y=float(transformed[1, 3]), z=float(transformed[2, 3]),
                yaw=normalize(pose["yaw"] + alignment["yaw"]))


def local_bottom_box(point, raw_ego, lidar_translation):
    c, s = math.cos(raw_ego["yaw"]), math.sin(raw_ego["yaw"])
    dx, dy = point.x-raw_ego["x"], point.y-raw_ego["y"]
    return [c*dx+s*dy-lidar_translation[0], -s*dx+c*dy-lidar_translation[1],
            point.z-raw_ego["z"]-lidar_translation[2]-point.height/2,
            point.length, point.width, point.height, normalize(point.yaw-raw_ego["yaw"])]


def world_gravity_box(bottom_box, world_from_lidar):
    box = list(bottom_box)
    center = np.asarray([box[0], box[1], box[2]+box[5]/2, 1.])
    transformed = np.asarray(world_from_lidar) @ center
    world_yaw = math.atan2(world_from_lidar[1][0], world_from_lidar[0][0])
    return [*transformed[:3].tolist(), *box[3:6], normalize(box[6]+world_yaw)]


def full_gt_case(case_name, corrected, lineage, alignment, provenance, lower_stamp=None):
    """Export full trajectories, including future points outside model ROI."""
    trajectories, tracker_gt = [], []
    for segment in corrected:
        key = case_name + "|" + segment.key
        times, objects, series = [], [], []
        for point in segment.points:
            if lower_stamp is not None and point.timestamp < lower_stamp:
                continue
            pose = transform_pose(dict(x=point.x, y=point.y, z=point.z, yaw=point.yaw), alignment)
            center = [pose[axis] for axis in "xyz"]
            times.append(point.timestamp)
            series.append([point.timestamp, center])
            objects.append(dict(key=key, reconstructed_key=segment.key, asset=segment.asset_name,
                                center=center, size=[point.length, point.width, point.height],
                                yaw=pose["yaw"], class_name=segment.class_name))
        if not times:
            continue
        actor_lineage = [row for row in lineage[segment.key] if lower_stamp is None or row["stamp"] >= lower_stamp]
        trajectories.append(dict(key=[key], reconstructed_key=segment.key, asset=segment.asset_name,
                                 class_name=segment.class_name, series=series, objects=objects,
                                 raw_uid_lineage=actor_lineage))
        tracker_gt.append([times, objects])
    return dict(gt_trajectories=trajectories, gt=tracker_gt, unresolved=[],
                provenance=dict(provenance, full_original_object_history=True, model_roi_filter_applied=False,
                                key_scope="Unique case|reconstructed actor key; raw UID/generation is separate evidence."))


def count_support(points, bottom_box):
    gravity = list(bottom_box)
    gravity[2] += gravity[5]/2
    relative = points[:, :3]-np.asarray(gravity[:3])
    c, s = math.cos(gravity[6]), math.sin(gravity[6])
    x = c*relative[:, 0]+s*relative[:, 1]
    y = -s*relative[:, 0]+c*relative[:, 1]
    return int(np.count_nonzero((abs(x) <= gravity[3]/2+.03) & (abs(y) <= gravity[4]/2+.03)
                               & (abs(relative[:, 2]) <= gravity[5]/2+.03)))


def unresolved_nearby(raw_segments, resolved_keys, raw_ego, stamp, interpolate, max_gap):
    """Do not turn unknown geometry into silent nearby negatives.

    A conservative five-meter pivot margin is used around the rectangular
    model ROI. An active but unbracketable unknown actor excludes scoring too.
    Input frames themselves are retained for continuous tracker history.
    """
    nearby = []
    for segment in raw_segments:
        if segment.key in resolved_keys or not segment.points[0].timestamp <= stamp <= segment.points[-1].timestamp:
            continue
        point = interpolate(segment.points, stamp, max_gap)
        if point is None:
            nearby.append(dict(key=segment.key, reason="unbracketed_unknown_actor"))
            continue
        dx, dy = point.x-raw_ego["x"], point.y-raw_ego["y"]
        c, s = math.cos(raw_ego["yaw"]), math.sin(raw_ego["yaw"])
        local_xy = [c*dx+s*dy, -s*dx+c*dy]
        if abs(local_xy[0]) <= 105. and abs(local_xy[1]) <= 35.:
            nearby.append(dict(key=segment.key, reason="unknown_geometry_near_model_roi", raw_pivot_base_xy=local_xy))
    return nearby


def continuity_report(timeline, sample_step):
    selected = [row["raw_lidar_index"] for row in timeline if row["selected"]]
    all_indices = [row["raw_lidar_index"] for row in timeline]
    interior = [index for index in all_indices if selected and selected[0] < index < selected[-1]
                and index not in set(selected)]
    return dict(total_original_clouds=len(timeline), selected=len(selected), sample_step=sample_step,
                continuous_for_tracker=bool(selected) and sample_step == 1 and not interior,
                initial_nonmoving_excluded=False,
                interior_unselected_raw_indices=interior,
                prefix_unselected=sum(index < selected[0] for index in all_indices) if selected else len(all_indices),
                suffix_unselected=sum(index > selected[-1] for index in all_indices) if selected else 0,
                scope="All original clouds have timeline entries; no arbitrary warmup/edge trim. Missing causal cameras/pose are explicit.")


def raw_uid_lineage(object_rows, raw_segments):
    """Link exact raw GT observations to reconstructed segments and UID epochs.

    Reconstructed actor identity is authoritative for temporal diagnostics.
    Raw UID is frame-local evidence: one raw UID may change physical owner,
    and one physical actor may use multiple raw UIDs. Generation increments
    whenever the reconstructed owner changes, not merely when a gap exists.
    """
    point_lookup = defaultdict(list)
    for segment in raw_segments:
        for point in segment.points:
            point_lookup[(point.timestamp, segment.class_name)].append((segment.key, point))
    lineage, generations, previous_owner = defaultdict(list), defaultdict(int), {}
    deduplicated = {}
    for stamp, receipt, msg in object_rows:
        count = sum(len(getattr(msg, group)) for group in ("npc_list", "pedestrian_list", "obstacle_list"))
        if stamp not in deduplicated or count > deduplicated[stamp][0]:
            deduplicated[stamp] = (count, receipt, msg)
    for stamp, (_, receipt, msg) in sorted(deduplicated.items()):
        for cls, group in zip(CLASSES, ("npc_list", "pedestrian_list", "obstacle_list")):
            remaining = list(point_lookup.get((stamp, cls), ()))
            for status in getattr(msg, group):
                xyz = [float(getattr(status.position, key)) for key in "xyz"]
                matches = [(index, key, point) for index, (key, point) in enumerate(remaining)
                           if math.dist(xyz, [point.x, point.y, point.z]) <= 1.e-6]
                if len(matches) != 1:
                    raise ValueError("Raw GT observation cannot be uniquely traced to reconstructed actor")
                index, key, _ = matches[0]
                remaining.pop(index)
                uid = int(status.unique_id)
                raw_key = (cls, uid)
                if previous_owner.get(raw_key) != key:
                    generations[raw_key] += 1
                    previous_owner[raw_key] = key
                lineage[key].append(dict(stamp=stamp, receipt=receipt, class_name=cls, raw_uid=uid,
                                         raw_uid_generation=generations[raw_key],
                                         raw_name=str(status.name), reconstructed_key=key))
            if remaining:
                raise ValueError("Reconstructed GT has observations missing from raw lineage")
    return dict(lineage)


def quaternion_yaw(rotation):
    return math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                      1-2*(rotation.y*rotation.y+rotation.z*rotation.z))


def read_tf_evidence(bag):
    import rosbag
    alignment, dynamic = None, {}
    with rosbag.Bag(str(bag)) as stream:
        for topic, msg, _ in stream.read_messages(topics=["/tf_static", "/tf"]):
            for transform in msg.transforms:
                parent, child = transform.header.frame_id.lstrip("/"), transform.child_frame_id.lstrip("/")
                translation, rotation = transform.transform.translation, transform.transform.rotation
                pose = dict(x=float(translation.x), y=float(translation.y), z=float(translation.z),
                            yaw=quaternion_yaw(rotation))
                if parent == "map" and child == "morai_map":
                    if abs(rotation.x) > 1.e-8 or abs(rotation.y) > 1.e-8:
                        raise ValueError("Nonplanar map/MORAI alignment unsupported")
                    if alignment is not None and not np.allclose(
                            [alignment[k] for k in ("x", "y", "z", "yaw")],
                            [pose[k] for k in ("x", "y", "z", "yaw")], atol=1.e-9, rtol=0):
                        raise ValueError("Recorded map alignment changed")
                    alignment = pose
                elif topic == "/tf" and child == "base_link" and parent in ("map", "morai_map"):
                    stamp = transform.header.stamp.to_sec()
                    dynamic[stamp] = (parent, pose)
    if alignment is None:
        raise ValueError("Recorded map -> morai_map TF is required; no guessed map transform")
    rows = sorted((stamp, transform_pose(pose, alignment) if parent == "morai_map" else pose)
                  for stamp, (parent, pose) in dynamic.items())
    return alignment, rows


def validate_case_source(case):
    bag = Path(case["bag"]).resolve()
    if digest(bag) != case["bag_sha256"]:
        raise ValueError("Raw source bag changed")
    source = Path(case["source"]).resolve() if case.get("source") else None
    if source is not None and case.get("source_sha256") and digest(source) != case["source_sha256"]:
        raise ValueError("Prepared scenario source changed")
    companion = Path(case["folder"]) / "scenario.json"
    if not companion.exists():
        raise ValueError("Loaded companion scenario.json required")
    return bag, companion


def read_case_events(folder):
    """Separate previous-scene/pre-load input from the loaded scene epoch."""
    path = Path(folder) / "events.jsonl"
    if not path.exists():
        raise ValueError("Scenario lifecycle events required; cannot guess reload boundary")
    events = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    def event_time(name):
        values = [datetime.fromisoformat(row["time"]).timestamp() for row in events if row["event"] == name]
        if len(values) != 1:
            raise ValueError("Expected one explicit scenario lifecycle event: " + name)
        return values[0]
    ready = event_time("SCENARIO_READY")
    controller = event_time("CONTROLLER_START")
    end = event_time("RUN_FINISHED")
    if not ready <= controller <= end:
        raise ValueError("Scenario lifecycle order is inconsistent")
    return dict(path=str(path.resolve()), sha256=digest(path), scene_ready_stamp=ready,
                controller_start_stamp=controller, run_finished_stamp=end,
                policy="Pre-load/previous-scene inputs retained but not a continuous loaded-scene tracker session.")


def prepare(args):
    if args.output.exists():
        raise FileExistsError("Fresh output directory required")
    if args.sample_step < 1 or not math.isfinite(args.heading_correction_deg):
        raise ValueError("Positive sample step and finite heading correction required")
    if not 0 < args.camera_tolerance_sec <= .10 or not 0 < args.pose_bracket_sec <= .20:
        raise ValueError("Bounded camera synchronization/pose interpolation required")
    sys.path[:0] = [str(ROOT / "docker/bevfusion"),
                   str(ROOT / "perception/detection3d/bevfusion/tools"),
                   str(ROOT / "base_ws/src/tracking_prediction_evaluation/src")]
    from app.sensor4_calibration import load_sensor4_calibration
    from prepare_base5_rosbag_quick import xyzi
    from tracking_prediction_evaluation.trajectory import read_morai_trajectories, interpolate_point
    from lidar_packet_reassembly import read_source

    progress = json.loads(args.progress.read_text())
    cases = progress["completed"]
    if not cases:
        raise ValueError("No completed raw capture cases")
    plan_path = args.progress.parent / "plan.json"
    plan = json.loads(plan_path.read_text()) if plan_path.exists() else None
    if plan and len(cases) != len(plan["cases"]) and not args.allow_partial:
        raise ValueError("Incomplete campaign; explicitly select --allow-partial for preserved completed cases")
    calibration = load_sensor4_calibration(args.sensor, ["camera_"+camera for camera in CAMERAS])
    if not np.allclose(calibration["base_from_lidar"][:3, :3], np.eye(3)):
        raise ValueError("Planar diagnostic requires the audited unrotated LiDAR mount")
    translation = calibration["base_from_lidar"][:3, 3]
    frozen = {str(path.resolve()): digest(path) for path in (args.progress, args.sensor, args.catalog)}
    if plan_path.exists():
        frozen[str(plan_path.resolve())] = digest(plan_path)
    args.output.mkdir(parents=True)
    rows, reports, gt_cases = [], [], {}
    for case_index, case in enumerate(cases):
        bag, companion = validate_case_source(case)
        events = read_case_events(case["folder"])
        source = read_source(bag)
        raw_segments = read_morai_trajectories(bag, include_classes=CLASSES, minimum_points=1,
                                              heading_correction_deg=args.heading_correction_deg)
        from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
        corrected, resolution = transform_segments_to_asset_catalog_bbox_center(raw_segments, args.catalog)
        resolved_keys = {segment.key for segment in corrected}
        lineage = raw_uid_lineage(source["messages"]["/Object_topic"], raw_segments)
        alignment, operational_tf = read_tf_evidence(bag)
        ego_by_stamp = {}
        for stamp, _, message in source["messages"]["/Ego_topic"]:
            ego_by_stamp[stamp] = dict(
                x=float(message.position.x), y=float(message.position.y), z=float(message.position.z),
                yaw=normalize(math.radians(float(message.heading)+args.heading_correction_deg)))
        ego = sorted(ego_by_stamp.items())
        if not ego:
            raise ValueError("Raw Ego GT trajectory required")
        lidar = source["messages"]["/sensor/lidar1"]
        if not lidar:
            raise ValueError("Original PointCloud2 sequence required")
        case_name = f"{case['scene']}_{case['policy']}"
        timeline, skipped = [], Counter()
        retained = 0
        for raw_index, (stamp, receipt, cloud) in enumerate(lidar):
            loaded_epoch = stamp >= events["scene_ready_stamp"] and receipt >= events["scene_ready_stamp"]
            timeline_row = dict(raw_lidar_index=raw_index, stamp=stamp, receipt=receipt, selected=False, reason=None,
                                scene_epoch_valid=loaded_epoch)
            timeline.append(timeline_row)
            raw_ego = bracket_pose(ego, stamp, args.pose_bracket_sec)
            selected = [causal_camera(source["messages"][f"/sensor/camera/{camera}/compressed"],
                                      stamp, receipt, args.camera_tolerance_sec) for camera in CAMERAS]
            if raw_ego is None or any(row is None for row in selected):
                reason = "unbracketed_ego" if raw_ego is None else "missing_causal_camera"
                skipped[reason] += 1
                timeline_row["reason"] = reason
                continue
            if raw_index % args.sample_step:
                skipped["explicit_sample_step"] += 1
                timeline_row["reason"] = "explicit_sample_step"
                continue
            token = f"case{case_index:02d}_{case['scene']}_{case['policy']}_{raw_index:05d}"
            dest = args.output / "data" / token
            dest.mkdir(parents=True)
            points = xyzi(cloud)
            finite_mask = np.isfinite(points).all(axis=1) & (points[:, :3] != 0).any(axis=1)
            finite = points[finite_mask]
            if len(finite) and (np.any(finite[:, 3] < 0) or np.any(finite[:, 3] > 255)):
                raise ValueError("Raw ROS intensity is outside the 0..255 contract")
            point_path = dest / "points.bin"
            point_path.write_bytes(points.astype("<f4").tobytes())
            images, offsets, receipts, image_stamps = {}, {}, {}, {}
            for camera, (image_stamp, image_receipt, image) in zip(CAMERAS, selected):
                data = bytes(image.data)
                if not data.startswith(b"\xff\xd8"):
                    raise ValueError("Original native JPEG bytes required")
                path = dest / (camera+".jpg")
                path.write_bytes(data)
                images["image/camera_"+camera] = str(path.resolve())
                offsets[camera] = image_stamp-stamp
                receipts[camera] = image_receipt
                image_stamps[camera] = image_stamp
            map_ego = transform_pose(raw_ego, alignment)
            world_from_lidar = pose_matrix(map_ego) @ calibration["base_from_lidar"]
            boxes, labels, assets, keys, support, world_boxes, actors = [], [], [], [], [], [], []
            gt_unbracketed = []
            nearby_unknown = unresolved_nearby(raw_segments, resolved_keys, raw_ego, stamp,
                                               interpolate_point, args.pose_bracket_sec)
            for segment in corrected:
                if not segment.points[0].timestamp <= stamp <= segment.points[-1].timestamp:
                    continue
                point = interpolate_point(segment.points, stamp, args.pose_bracket_sec)
                if point is None:
                    gt_unbracketed.append(segment.key)
                    continue
                box = local_bottom_box(point, raw_ego, translation)
                if abs(box[0]) > 100 or abs(box[1]) > 30:
                    continue
                keys.append(case_name+"|"+segment.key)
                boxes.append(box)
                labels.append(CLASSES.index(segment.class_name))
                assets.append(segment.asset_name)
                support.append(count_support(finite, box))
                world_boxes.append(world_gravity_box(box, world_from_lidar))
                actor_history = lineage[segment.key]
                before_index = bisect.bisect_right([row["stamp"] for row in actor_history], stamp)-1
                actor = actor_history[max(0, before_index)]
                actors.append(dict(actor, asset=segment.asset_name,
                                   raw_uid_scope="Frame-local UID/generation; reconstructed_key is persistent diagnostic correspondence."))
            operational = bracket_pose(operational_tf, stamp, args.pose_bracket_sec)
            discrepancy = None if operational is None else dict(
                xy_m=math.hypot(operational["x"]-map_ego["x"], operational["y"]-map_ego["y"]),
                z_m=operational["z"]-map_ego["z"],
                yaw_deg=math.degrees(normalize(operational["yaw"]-map_ego["yaw"])))
            rows.append(dict(
                token=token, source="dev", case=case_name,
                scene=case["scene"], policy=case["policy"], stamp=stamp, receipt=receipt,
                raw_lidar_index=raw_index, point_path=str(point_path.resolve()), point_sha256=digest(point_path),
                images=images, image_sha256={key: digest(path) for key, path in images.items()},
                camera_offsets_sec=offsets, camera_stamps=image_stamps, camera_receipts=receipts,
                camera_stamp_span_sec=max(image_stamps.values())-min(image_stamps.values()),
                camera_selection="Nearest header within 0.10s among messages received no later than this cloud receipt; actual callback scheduling not certified.",
                gt_boxes=boxes, gt_labels=labels, gt_points=support, gt_assets=assets, parent_uids=keys,
                gt_world_boxes=world_boxes, gt_actor_metadata=actors, gt_unbracketed_actor_keys=gt_unbracketed,
                gt_unknown_near_roi=nearby_unknown,
                scene_epoch_valid=loaded_epoch,
                tracker_session=case_name+("|loaded" if loaded_epoch else "|preload"),
                scoring_eligible=loaded_epoch and not gt_unbracketed and not nearby_unknown,
                actor_geometry_resolved=not nearby_unknown,
                ego=map_ego, raw_ego=raw_ego, operational_tf_ego=operational,
                operational_tf_vs_raw_gt=discrepancy, world_frame="map",
                lidar_translation=translation.tolist(), world_from_lidar=world_from_lidar.tolist(),
                map_from_morai=alignment, heading_correction_deg=args.heading_correction_deg,
                box_center_convention="Export bottom; catalog geometric gravity center transformed from corrected physical MORAI yaw. Operational TF is diagnostic only.",
                raw_point_count=len(points), finite_nonzero_point_count=len(finite),
            ))
            timeline_row.update(selected=True, token=token)
            retained += 1
        if not retained:
            raise ValueError("No causal synchronized frames in completed case")
        continuity = continuity_report(timeline, args.sample_step)
        loaded_rows = [row for row in rows if row["case"] == case_name and row["scene_epoch_valid"]]
        loaded_timeline = [row for row in timeline if row["scene_epoch_valid"]]
        loaded_continuity = continuity_report(loaded_timeline, args.sample_step)
        excluded_scoring = sum(not row["scoring_eligible"] for row in loaded_rows)
        gt_cases[case_name] = full_gt_case(case_name, corrected, lineage, alignment, dict(
            path=str(bag), bag_sha256=case["bag_sha256"], heading_correction_deg=args.heading_correction_deg,
            recorded_alignment=alignment, gt_resolution=resolution, excluded_scoring_frames=excluded_scoring,
            caveat="Catalog center diagnostics, not annotation/AP. GT interpolation uses only valid temporal brackets.",
            scenario_events=events), lower_stamp=events["scene_ready_stamp"])
        gt_cases[case_name]["unresolved"] = [dict(
            key=case_name+"|"+segment.key, class_name=segment.class_name, raw_uid_lineage=lineage[segment.key],
            raw_pivot_series=[[point.timestamp, [point.x, point.y, point.z]] for point in segment.points
                              if point.timestamp >= events["scene_ready_stamp"]],
            reference="Uncorrected MORAI actor pivot; not a geometric GT center")
            for segment in raw_segments if segment.key not in resolved_keys
            and segment.points[-1].timestamp >= events["scene_ready_stamp"]]
        reports.append(dict(
            scene=case["scene"], policy=case["policy"], bag=str(bag), bag_sha256=case["bag_sha256"],
            scenario=str(companion), scenario_sha256=digest(companion), selected=retained,
            skipped=dict(skipped), continuity=continuity, catalog_resolution=resolution,
            loaded_scene_continuity=loaded_continuity, scene_events=events,
            loaded_scene_frames=len(loaded_rows), preload_frames=retained-len(loaded_rows),
            map_from_morai=alignment, raw_uid_lineage=lineage, timeline=timeline,
            scoring_eligible=len(loaded_rows)-excluded_scoring,
            moving_ego_displacement_m=math.dist([ego[0][1][key] for key in "xy"], [ego[-1][1][key] for key in "xy"]),
            scope="Full raw sequence, catalog GT only; missing inputs explicit. Natural reacquisition coverage requires downstream observation.",
        ))
        print(json.dumps(dict(event="prepared_case", scene=case["scene"], policy=case["policy"], frames=retained,
                              continuous_for_tracker=continuity["continuous_for_tracker"])), flush=True)
    if frozen != {path: digest(path) for path in frozen}:
        raise ValueError("Frozen preparation contracts changed during extraction")
    save(args.output / "inputs.json", dict(
        schema_version=1, rows=rows, cases=dict(Counter(row["case"] for row in rows)),
        input_source_sha256=frozen, sensor_sha256=digest(args.sensor), catalog_sha256=digest(args.catalog),
        heading_correction_deg=args.heading_correction_deg, sample_step=args.sample_step,
        camera_tolerance_sec=args.camera_tolerance_sec, pose_bracket_sec=args.pose_bracket_sec,
        continuous_for_tracker=all(report["loaded_scene_continuity"]["continuous_for_tracker"] for report in reports),
        tracker_replay_filter="Reset per case, then retain ALL scene_epoch_valid rows including initial Park before controller start. Do not cross the scenario reload/teleport boundary.",
        scope="Moving Ego native ROS inputs and catalog-corrected diagnostic GT. Initial stationary history is retained. Not AP, exact online callback timing or closed-loop certification.",
    ))
    save(args.output / "preparation_report.json", dict(cases=reports, input_source_sha256=frozen))
    with gzip.open(args.output / "gt_pack.json.gz", "wt") as stream:
        json.dump(dict(cases=gt_cases, input_source_sha256=frozen,
                       scope="Full catalog-corrected Object trajectories, including beyond ROI and all raw UID generations."),
                  stream, allow_nan=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--progress", type=Path)
    source.add_argument("--campaign", type=Path, help="Completed campaign directory containing progress.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sensor", type=Path, default=ROOT / "configs/sensors/Sensor_competition_local.json")
    parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
    parser.add_argument("--heading-correction-deg", type=float, default=-1.347)
    parser.add_argument("--sample-step", type=int, default=1)
    parser.add_argument("--camera-tolerance-sec", type=float, default=.10)
    parser.add_argument("--pose-bracket-sec", type=float, default=.15)
    parser.add_argument("--allow-partial", action="store_true")
    args = parser.parse_args()
    if args.campaign:
        args.progress = args.campaign / "progress.json"
    prepare(args)


if __name__ == "__main__":
    main()
