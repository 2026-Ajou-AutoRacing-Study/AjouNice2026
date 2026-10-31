#!/usr/bin/env python3
"""Approximate static-MGeo housing projection for offline cause separation only.

This is a candidate/oracle ROI requiring visual verification, NOT automatic GT,
localization, a production detector, or a production map/camera modification.
Ego roll/pitch are unmeasured and assumed zero. MGeo point is treated as the
housing CENTER: z_offset is recorded but never added a second time. Occlusion,
lamp state, rendering/model-center offsets and lens distortion are not solved.

The explicitly chosen MGeo file and current fixed front-camera profile are read
only. Ego scene yaw = raw MORAI heading - 1.347 degrees, exactly once. Ego/body
axes are forward/left/up; pixels are right/down. Camera pitch follows the
specified MORAI projection equations, not a guessed optical-frame convention.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SENSOR = ROOT / "configs/sensors/Sensor_competition_local.json"
HEADING_CORRECTION_DEGREES = -1.347


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value, name):
    if isinstance(value, bool):
        raise ValueError("{} cannot be a boolean".format(name))
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(name))
    return number


def xyz(values, name):
    if isinstance(values, dict):
        values = [values[axis] for axis in "xyz"]
    if len(values) != 3:
        raise ValueError("{} needs exactly three coordinates".format(name))
    return [finite(value, name) for value in values]


def fixed_camera():
    return {"position_xyz_m": [1.9, 0., 1.2], "pitch_degrees": 2.,
            "yaw_degrees": 0., "roll_degrees": 0., "horizontal_fov_degrees": 90.,
            "width": 1280, "height": 720}


def load_fixed_camera(path=DEFAULT_SENSOR):
    """Require the specified fixed camera contract; never select another profile."""
    path = Path(path).resolve(strict=True)
    document = json.loads(path.read_text())
    matches = [camera for camera in document["cameraList"] if camera["m_SensorUniqueID"] == 1]
    if len(matches) != 1:
        raise ValueError("Expected exactly one camera with m_SensorUniqueID=1")
    camera, expected = matches[0], fixed_camera()
    parameters = camera["cc"]
    actual = {"position_xyz_m": xyz(camera["pos"], "camera position"),
              "pitch_degrees": finite(camera["rot"]["pitch"], "camera pitch"),
              "yaw_degrees": finite(camera["rot"]["yaw"], "camera yaw"),
              "roll_degrees": finite(camera["rot"]["roll"], "camera roll"),
              "horizontal_fov_degrees": finite(parameters["cameraFOV"], "camera FOV"),
              "width": parameters["cameraResWidth"], "height": parameters["cameraResHeight"]}
    for key in expected:
        left = actual[key] if isinstance(actual[key], list) else [actual[key]]
        right = expected[key] if isinstance(expected[key], list) else [expected[key]]
        if any(not math.isclose(a, b, rel_tol=0, abs_tol=1e-6) for a, b in zip(left, right)):
            raise ValueError("Sensor camera1 differs from fixed projection contract: {}".format(key))
    if any(abs(finite(value, "lens distortion")) > 1e-12 for value in parameters.get("lensDistortion", [])):
        raise ValueError("Nonzero lens distortion is unsupported by this approximate projection")
    return actual, {"sensor_path": str(path), "sensor_sha256": digest(path), "sensor_unique_id": 1,
                    "stored_focalLengthpixel_not_used": parameters.get("focalLengthpixel"),
                    "intrinsics_source": "horizontal_FOV_and_resolution_square_pixels"}


def load_signal(path, signal_id):
    path = Path(path).resolve(strict=True)
    records = json.loads(path.read_text())
    if not isinstance(records, list):
        raise ValueError("Expected an explicit MGeo traffic_light_set.json root list")
    matches = [signal for signal in records if signal.get("idx") == signal_id]
    if len(matches) != 1:
        raise ValueError("Signal ID must match exactly one MGeo record: {}".format(signal_id))
    return matches[0], {"mgeo_path": str(path), "mgeo_sha256": digest(path), "signal_id": signal_id}


def parse_ego_document(document, frame_index=None):
    """Accept a frame['ego'] snapshot, telemetry itself, or v4 frame['status']."""
    if "frames" in document:
        if frame_index is None or isinstance(frame_index, bool) or not isinstance(frame_index, int):
            raise ValueError("A capture manifest needs an explicit --frame-index")
        if not 0 <= frame_index < len(document["frames"]):
            raise ValueError("frame-index is out of range")
        document = document["frames"][frame_index]
    elif frame_index is not None:
        raise ValueError("frame-index is only valid for a document containing frames")
    ego = document.get("ego", document.get("status", document))
    if "position_xyz_m" in ego:
        position, heading = ego["position_xyz_m"], ego["heading_degrees"]
    else:
        position, heading = ego["position"], ego["heading"]
    return xyz(position, "Ego position"), finite(heading, "raw Ego heading"), {
        "frame_index": frame_index, "ego_header_stamp": ego.get("ego_header_stamp", ego.get("header")),
        "roll_pitch_assumption": "unmeasured_assumed_zero_not_full_6DoF"}


def world_to_camera(point, ego_position, raw_heading_degrees, camera=None):
    camera = fixed_camera() if camera is None else camera
    point, ego_position = xyz(point, "world point"), xyz(ego_position, "Ego position")
    yaw = math.radians(finite(raw_heading_degrees, "raw heading") + HEADING_CORRECTION_DEGREES)
    pitch = math.radians(finite(camera["pitch_degrees"], "camera pitch"))
    if camera.get("yaw_degrees", 0) != 0 or camera.get("roll_degrees", 0) != 0:
        raise ValueError("Only zero camera yaw/roll is supported")
    delta = [point[index] - ego_position[index] for index in range(3)]
    ego = [math.cos(yaw) * delta[0] + math.sin(yaw) * delta[1],
           -math.sin(yaw) * delta[0] + math.cos(yaw) * delta[1], delta[2]]
    camera_position = xyz(camera["position_xyz_m"], "camera position")
    dx, dy, dz = [ego[index] - camera_position[index] for index in range(3)]
    forward = math.cos(pitch) * dx - math.sin(pitch) * dz
    up = math.sin(pitch) * dx + math.cos(pitch) * dz
    return {"ego_xyz_forward_left_up_m": ego,
            "camera_xyz_forward_left_up_m": [forward, dy, up], "depth_m": forward}


def project_point(point, ego_position, raw_heading_degrees, camera=None):
    camera = fixed_camera() if camera is None else camera
    result = world_to_camera(point, ego_position, raw_heading_degrees, camera)
    width, height = camera["width"], camera["height"]
    fov = finite(camera["horizontal_fov_degrees"], "FOV")
    if width <= 0 or height <= 0 or not 0 < fov < 180:
        raise ValueError("Invalid camera resolution/FOV")
    focal = (width / 2) / math.tan(math.radians(fov / 2))
    forward, left, up = result["camera_xyz_forward_left_up_m"]
    result["image_uv"] = None if forward <= 1e-6 else [width / 2 - focal * left / forward,
                                                               height / 2 - focal * up / forward]
    result["in_front"] = forward > 1e-6
    return result


def project_housing(signal, ego_position, raw_heading_degrees, camera=None, margin_px=4.):
    camera = fixed_camera() if camera is None else camera
    margin_px = finite(margin_px, "margin_px")
    if margin_px < 0:
        raise ValueError("margin_px must be nonnegative")
    center = xyz(signal["point"], "MGeo signal center")
    width_m, height_m = finite(signal["width"], "housing width"), finite(signal["height"], "housing height")
    heading = finite(signal["heading"], "signal heading")
    if width_m <= 0 or height_m <= 0:
        raise ValueError("MGeo housing width/height must be positive; no guessed dimensions")
    width_angle = math.radians(heading + 90.)
    width_axis = [math.cos(width_angle), math.sin(width_angle), 0.]
    corners = []
    for width_sign, height_sign in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corner = [center[index] + width_sign * width_m / 2 * width_axis[index] for index in range(3)]
        corner[2] += height_sign * height_m / 2
        corners.append({"world_xyz_m": corner, **project_point(corner, ego_position, raw_heading_degrees, camera)})
    center_projection = project_point(center, ego_position, raw_heading_degrees, camera)
    all_in_front = all(corner["in_front"] for corner in corners)
    raw_bbox = expanded = clipped = None
    fully_in_image = outside = False
    margin_clipped, reasons = False, []
    if not all_in_front:
        reasons.append("housing_behind_or_crossing_camera_plane")
    else:
        pixels = [corner["image_uv"] for corner in corners]
        raw_bbox = [min(point[0] for point in pixels), min(point[1] for point in pixels),
                    max(point[0] for point in pixels), max(point[1] for point in pixels)]
        width, height = camera["width"], camera["height"]
        fully_in_image = all(0 <= point[0] <= width and 0 <= point[1] <= height for point in pixels)
        outside = raw_bbox[2] <= 0 or raw_bbox[0] >= width or raw_bbox[3] <= 0 or raw_bbox[1] >= height
        expanded = [math.floor(raw_bbox[0] - margin_px), math.floor(raw_bbox[1] - margin_px),
                    math.ceil(raw_bbox[2] + margin_px), math.ceil(raw_bbox[3] + margin_px)]
        clipped = [min(width, max(0, expanded[0])), min(height, max(0, expanded[1])),
                   min(width, max(0, expanded[2])), min(height, max(0, expanded[3]))]
        margin_clipped = clipped != expanded
        if not fully_in_image:
            reasons.append("housing_completely_out_of_screen" if outside else "housing_partly_out_of_screen")
        if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
            clipped = None
    focal = (camera["width"] / 2) / math.tan(math.radians(camera["horizontal_fov_degrees"] / 2))
    return {
        "schema_version": 1, "scope": "approximate_static_MGeo_oracle_candidate_only_not_production",
        "requires_visual_verification": True, "ground_truth": False, "production_ready": False,
        "valid": all_in_front and fully_in_image and clipped is not None,
        "invalid_reasons": reasons, "all_corners_in_front": all_in_front,
        "out_of_screen": all_in_front and not fully_in_image, "completely_out_of_screen": outside,
        "margin_clipped": margin_clipped, "margin_px": margin_px,
        "bbox_xyxy_float_no_margin": raw_bbox, "bbox_xyxy_integer_unclipped": expanded,
        "bbox_xyxy_integer_clipped": clipped,
        "projected_center_uv": center_projection["image_uv"], "center_projection": center_projection,
        "corners": corners, "signal_id": signal.get("idx"), "signal_center_world_xyz_m": center,
        "signal_heading_degrees": heading, "housing_dimensions_width_height_m": [width_m, height_m],
        "housing_width_axis_world_xyz": width_axis, "mgeo_z_offset_not_added": signal.get("z_offset"),
        "ego_position_world_xyz_m": xyz(ego_position, "Ego position"),
        "raw_ego_heading_degrees": finite(raw_heading_degrees, "raw heading"),
        "corrected_scene_yaw_degrees": finite(raw_heading_degrees, "raw heading") + HEADING_CORRECTION_DEGREES,
        "camera": camera, "intrinsics": {"fx": focal, "fy": focal,
                                           "cx": camera["width"] / 2, "cy": camera["height"] / 2},
        "coordinate_contract": {"world": "explicit_static_MGeo_local_same_as_native_Ego_xyz",
                                "ego_axes": "forward_left_up", "pixels": "right_down",
                                "heading_correction_degrees_applied_once": HEADING_CORRECTION_DEGREES,
                                "ego_roll_pitch": "unmeasured_assumed_zero",
                                "camera_forward": "cos(pitch)*dx-sin(pitch)*dz",
                                "camera_up": "sin(pitch)*dx+cos(pitch)*dz"},
        "limitations": ["No occlusion test", "No lamp/state ground truth", "Ego roll/pitch assumed zero",
                        "Static housing center/size/orientation and rendered model may differ",
                        "No calibrated distortion or production localization transform"],
    }


def write_json(report, output=None):
    if output is None:
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    else:
        with Path(output).open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mgeo", required=True, type=Path, help="Explicit traffic_light_set.json asset")
    parser.add_argument("--signal-id", required=True, help="Exact native MGeo idx; no historical aliases")
    pose = parser.add_mutually_exclusive_group(required=True)
    pose.add_argument("--ego", nargs=4, type=float, metavar=("X", "Y", "Z", "RAW_HEADING_DEG"))
    pose.add_argument("--ego-json", type=Path)
    parser.add_argument("--frame-index", type=int)
    parser.add_argument("--sensor-json", type=Path, default=DEFAULT_SENSOR)
    parser.add_argument("--margin-px", type=float, default=4.)
    parser.add_argument("--output", type=Path, help="Fresh JSON file only; default stdout")
    args = parser.parse_args(argv)
    camera, camera_source = load_fixed_camera(args.sensor_json)
    signal, signal_source = load_signal(args.mgeo, args.signal_id)
    if args.ego is not None:
        if args.frame_index is not None:
            parser.error("--frame-index requires --ego-json")
        position, heading = args.ego[:3], args.ego[3]
        ego_source = {"source": "explicit_CLI_native_Ego_xyz_and_raw_heading"}
    else:
        path = args.ego_json.resolve(strict=True)
        position, heading, ego_source = parse_ego_document(json.loads(path.read_text()), args.frame_index)
        ego_source.update(path=str(path), sha256=digest(path))
    report = project_housing(signal, position, heading, camera, args.margin_px)
    report["provenance"] = {"sensor": camera_source, "signal": signal_source, "ego": ego_source,
                            "projection_source_sha256": digest(__file__)}
    write_json(report, args.output)


if __name__ == "__main__":
    main()
