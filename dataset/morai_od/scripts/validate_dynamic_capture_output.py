#!/usr/bin/env python3
"""Validate a timed-grouped SaveSensorData dynamic capture directory."""

import argparse
import hashlib
import json
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
    for frame in frames:
        if not frame.get("verified"):
            errors.append("unverified frame present")
        if not frame.get("quality", {}).get("actor_present"):
            warnings.append("frame without primary dynamic actor")
        files = frame.get("sensor_files", [])
        if not files:
            errors.append("frame has no sensor_files")
        for relative in files:
            path = capture_dir / relative
            if not path.is_file():
                errors.append("missing sensor file: {}".format(relative))
        objects = frame.get("objects") or {}
        for actor in objects.get("actors", []):
            class_counts[actor["class"]] += 1
            stable_tracks[(actor["class"], int(actor["unique_id"]))] += 1
            phase_counts[actor.get("motion_phase", "missing")] += 1
            for key in (
                "morai_velocity_raw",
                "morai_acceleration_raw",
                "relative_to_ego",
            ):
                if actor.get(key) is None:
                    errors.append("actor metadata misses {}".format(key))

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
