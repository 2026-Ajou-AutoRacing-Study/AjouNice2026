#!/usr/bin/env python3
"""Validate a timed-grouped SaveSensorData dynamic capture directory."""

import argparse
import hashlib
import json
import math
import statistics
from collections import Counter
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_jsonl(path):
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def validate_clip(capture_dir, clip_path):
    clip = load_json(clip_path)
    frames_path = capture_dir / clip["frames_jsonl"]
    errors = []
    warnings = []
    if not frames_path.is_file():
        return {"clip_id": clip.get("clip_id"), "errors": ["frames JSONL missing"]}
    frames = load_jsonl(frames_path)
    if len(frames) != int(clip["frame_count"]):
        errors.append("clip frame_count differs from frames JSONL")
    indices = [int(frame["frame_index"]) for frame in frames]
    if indices != list(range(len(frames))):
        errors.append("frame indices are not contiguous")
    trigger_times = [float(frame["triggered_unix"]) for frame in frames]
    if any(right <= left for left, right in zip(trigger_times, trigger_times[1:])):
        errors.append("trigger timestamps are not strictly increasing")
    stems = [frame.get("morai_timestamp_stem") for frame in frames]
    if len(stems) != len(set(stems)):
        errors.append("MORAI timestamp stems are not unique")

    dataset_id = str(clip.get("dataset_id", ""))
    static_classcentric = dataset_id.startswith(
        "morai_static_classcentric_base3"
    )
    sequence_contract = clip.get("sequence_contract") or {}
    if dataset_id.startswith("morai_temporal_sequence_v2"):
        if len(frames) != 8:
            errors.append("temporal V2 clip does not contain 8 frames")
        if int(sequence_contract.get("clip_length", 0)) != 8:
            errors.append("temporal V2 sequence contract has invalid clip length")
        target_period = float(
            sequence_contract.get(
                "nominal_period_sec", clip.get("capture_period_sec", 0.0)
            )
        )
        sequence_times = [
            frame.get("sequence_time_sec") for frame in frames
        ]
        if any(value is None for value in sequence_times):
            errors.append("temporal V2 frame misses sequence_time_sec")
            sequence_times = trigger_times
        else:
            sequence_times = [float(value) for value in sequence_times]
            if any(
                right <= left
                for left, right in zip(sequence_times, sequence_times[1:])
            ):
                errors.append("sequence timestamps are not strictly increasing")
        intervals = [
            right - left
            for left, right in zip(sequence_times, sequence_times[1:])
        ]
        if intervals and abs(statistics.median(intervals) - target_period) > 0.04:
            errors.append(
                "temporal V2 median sequence period differs from contract: "
                "{:.3f}s vs {:.3f}s".format(
                    statistics.median(intervals), target_period
                )
            )
        if sequence_contract.get("split_group") != clip.get("clip_id"):
            errors.append("temporal V2 split group must equal clip id")
        for frame in frames:
            if frame.get("sequence_contract") != sequence_contract:
                errors.append(
                    "frame {} sequence contract differs from clip".format(
                        frame.get("frame_index")
                    )
                )
            expected_prev = int(frame.get("frame_index", 0)) > 0
            if bool(frame.get("prev_exists")) != expected_prev:
                errors.append(
                    "frame {} has invalid prev_exists".format(
                        frame.get("frame_index")
                    )
                )
            expected_prev_id = (
                "{}_f{:06d}".format(
                    clip.get("clip_id"), int(frame["frame_index"]) - 1
                )
                if expected_prev
                else None
            )
            if frame.get("prev_frame_id") != expected_prev_id:
                errors.append(
                    "frame {} has invalid prev_frame_id".format(
                        frame.get("frame_index")
                    )
                )

    timing_stream_counts = {}
    timing_relative = clip.get("timing_sidecar")
    timing_samples = []
    if timing_relative:
        timing_path = capture_dir / timing_relative
        if not timing_path.is_file():
            errors.append("timing sidecar is missing")
        else:
            timing_samples = load_jsonl(timing_path)
            sample_ids = {
                sample.get("sample_id") for sample in timing_samples
            }
            timing_stream_counts = dict(
                Counter(sample.get("stream") for sample in timing_samples)
            )
            for required_stream in ("ego", "objects"):
                if timing_stream_counts.get(required_stream, 0) == 0:
                    errors.append(
                        "timing sidecar has no {} samples".format(
                            required_stream
                        )
                    )
            if timing_stream_counts.get("clock", 0) == 0:
                warnings.append(
                    "timing sidecar has no /clock samples; message header "
                    "timestamps remain available"
                )
            for frame in frames:
                references = frame.get("timing_samples")
                if not references:
                    errors.append("frame has no timing sample references")
                    continue
                for required_stream in ("ego", "objects"):
                    reference = references.get(required_stream)
                    if not reference:
                        errors.append(
                            "frame {} has no {} timing reference".format(
                                frame.get("frame_index"), required_stream
                            )
                        )
                    elif reference.get("sample_id") not in sample_ids:
                        errors.append(
                            "frame {} references missing timing sample {}".format(
                                frame.get("frame_index"),
                                reference.get("sample_id"),
                            )
                        )
    elif int(clip.get("schema_version", 1)) >= 2:
        errors.append("schema v2 clip has no timing sidecar")

    timed_grouped = (
        clip.get("capture_format")
        == "morai_save_sensor_data_timed_grouped"
    )
    motion_arm_m = float(
        clip.get("capture_motion_arm_m", 0.30 if timed_grouped else 0.0)
    )
    minimum_progress_m = float(
        clip.get(
            "minimum_capture_progress_m",
            0.20 if timed_grouped else 0.0,
        )
    )
    if frames and motion_arm_m > 0.0:
        first_distance = float(frames[0].get("trigger_distance_m", 0.0))
        if first_distance + 1e-6 < motion_arm_m:
            errors.append(
                "first frame was captured before Ego motion arm: "
                "{:.3f}m < {:.3f}m".format(first_distance, motion_arm_m)
            )
    if minimum_progress_m > 0.0:
        for frame in frames[1:]:
            progress = float(
                frame.get("capture_progress_since_previous_m", 0.0)
            )
            if progress + 1e-6 < minimum_progress_m:
                errors.append(
                    "frame {} lacks Ego progress: {:.3f}m < {:.3f}m".format(
                        frame.get("frame_index"),
                        progress,
                        minimum_progress_m,
                    )
                )

    class_counts = Counter()
    phase_counts = Counter()
    stable_tracks = Counter()
    track_positions = {}
    static_motion_violations = 0
    for frame in frames:
        if not frame.get("verified"):
            errors.append("unverified frame present")
        if (
            not static_classcentric
            and not frame.get("quality", {}).get("actor_present")
        ):
            warnings.append("frame without primary dynamic actor")
        # Temporal V2 stored ``sensor_files``. Base3 rolling-grouped uses the
        # more explicit ``new_files`` plus per-sensor artifact inventories.
        # Both schemas refer to files copied under the capture directory.
        files = frame.get("sensor_files") or frame.get("new_files") or []
        if not files:
            errors.append("frame has no sensor_files")
        for relative in files:
            path = capture_dir / relative
            if not path.is_file():
                errors.append("missing sensor file: {}".format(relative))
        observed_artifacts = frame.get("observed_artifacts") or {}
        required_artifacts = frame.get("required_artifacts") or {}
        missing_artifacts = frame.get("missing_artifacts") or {}
        if missing_artifacts:
            errors.append(
                "frame reports missing required artifacts: {}".format(
                    missing_artifacts
                )
            )
        for sensor, required_suffixes in required_artifacts.items():
            observed_suffixes = set(observed_artifacts.get(sensor, []))
            missing_suffixes = sorted(set(required_suffixes) - observed_suffixes)
            if missing_suffixes:
                errors.append(
                    "frame artifact inventory misses {}: {}".format(
                        sensor, missing_suffixes
                    )
                )
        objects = frame.get("objects") or {}
        for actor in objects.get("actors", []):
            class_counts[actor["class"]] += 1
            stable_tracks[(actor["class"], int(actor["unique_id"]))] += 1
            track_positions.setdefault(
                (actor["class"], int(actor["unique_id"])), []
            ).append(actor["position_m"])
            phase_counts[actor.get("motion_phase", "missing")] += 1
            if actor["class"] == "obstacle":
                velocity = actor.get("derived_velocity_mps") or {}
                if (
                    float(velocity.get("x", 0.0)) ** 2
                    + float(velocity.get("y", 0.0)) ** 2
                ) ** 0.5 > 0.15:
                    static_motion_violations += 1
            for key in (
                "morai_velocity_raw",
                "morai_acceleration_raw",
                "relative_to_ego",
            ):
                if actor.get(key) is None:
                    errors.append("actor metadata misses {}".format(key))
    if static_motion_violations:
        errors.append(
            "{} obstacle observations have non-static derived velocity".format(
                static_motion_violations
            )
        )
    if clip.get("mission_family") == "crosswalk_pedestrian":
        pedestrian_displacements = []
        for (class_name, _unique_id), positions in track_positions.items():
            if class_name != "pedestrian" or len(positions) < 2:
                continue
            pedestrian_displacements.append(
                math.hypot(
                    float(positions[-1]["x"]) - float(positions[0]["x"]),
                    float(positions[-1]["y"]) - float(positions[0]["y"]),
                )
            )
        if not pedestrian_displacements or max(pedestrian_displacements) < 0.2:
            errors.append(
                "crosswalk pedestrian moved less than 0.2 m during clip"
            )
    expected_scene = clip.get("source_scene_expected") or {}
    if int(expected_scene.get("spawn_points", 0)) > 0:
        vehicle_displacements = []
        for (class_name, _unique_id), positions in track_positions.items():
            if class_name != "vehicle" or len(positions) < 2:
                continue
            vehicle_displacements.append(
                math.hypot(
                    float(positions[-1]["x"]) - float(positions[0]["x"]),
                    float(positions[-1]["y"]) - float(positions[0]["y"]),
                )
            )
        if not vehicle_displacements:
            errors.append("spawned-vehicle clip contains no vehicle track")
        elif max(vehicle_displacements) < 0.2:
            errors.append("spawned vehicles moved less than 0.2 m during clip")

    sensor_config = clip.get("sensor_config")
    if sensor_config:
        sensor_path = Path(sensor_config)
        if not sensor_path.is_absolute():
            sensor_path = PROJECT_ROOT / sensor_path
        if not sensor_path.is_file():
            errors.append("sensor config is missing")
        elif clip.get("sensor_config_sha256") != sha256(sensor_path):
            errors.append("sensor config SHA256 differs")
    return {
        "clip_id": clip.get("clip_id"),
        "frame_count": len(frames),
        "class_observations": dict(class_counts),
        "motion_phases": dict(phase_counts),
        "stable_track_count": len(stable_tracks),
        "tracks_visible_at_least_6_frames": sum(
            count >= 6 for count in stable_tracks.values()
        ),
        "timing_stream_counts": timing_stream_counts,
        "timing_streams": clip.get("timing_streams", {}),
        "error_count": len(errors),
        "warning_count": len(warnings),
        "errors": errors,
        "warnings": warnings[:20],
    }


def main():
    args = parse_args()
    capture_dir = args.capture_dir.expanduser().resolve()
    metadata_dir = capture_dir / "dynamic_metadata"
    clips = sorted(metadata_dir.glob("*.clip.json"))
    if not clips:
        raise SystemExit("no dynamic_metadata/*.clip.json found")
    results = [validate_clip(capture_dir, path) for path in clips]
    report = {
        "schema_version": 1,
        "capture_dir": str(capture_dir),
        "clip_count": len(results),
        "frame_count": sum(item.get("frame_count", 0) for item in results),
        "error_count": sum(item.get("error_count", 0) for item in results),
        "warning_count": sum(item.get("warning_count", 0) for item in results),
        "clips": results,
    }
    report["status"] = "pass" if report["error_count"] == 0 else "fail"
    report_path = args.report or capture_dir / "dynamic_validation.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("clips: {}".format(report["clip_count"]))
    print("frames: {}".format(report["frame_count"]))
    print("errors: {}".format(report["error_count"]))
    print("warnings: {}".format(report["warning_count"]))
    print("status: {}".format(report["status"]))
    print("report: {}".format(report_path))
    if report["error_count"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
