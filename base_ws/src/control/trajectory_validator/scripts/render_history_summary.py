#!/usr/bin/env python3

"""Backfill or regenerate the static summary image for one history run."""

import argparse
import json
import os
import re
import tempfile
from pathlib import Path

from trajectory_validator.summary_svg import (
    read_source_trajectory_xy,
    render_run_summary_svg,
    sha256_file,
)


SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "base_ws" / "src").is_dir() and (parent / "kcity_map").is_dir():
            return parent
    raise RuntimeError("cannot locate AjouNice2026 repository root")


def resolve_repo_file(root, value, label):
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("{} must stay inside repository".format(label)) from error
    if not path.is_file():
        raise FileNotFoundError("{} does not exist: {}".format(label, path))
    return path


def map_version_from_path(path):
    match = re.search(r"(?:^|_)v([0-9]+)\.osm$", Path(path).name, re.IGNORECASE)
    return "v{}".format(match.group(1)) if match else None


def atomic_json(path, document):
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--traj-sample", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--map-file")
    parser.add_argument("--projector-file")
    parser.add_argument("--filename", default="summary.svg")
    parser.add_argument("--map-margin-m", type=float, default=20.0)
    arguments = parser.parse_args()
    if not SAFE_NAME.fullmatch(arguments.traj_sample) or not SAFE_NAME.fullmatch(arguments.run_id):
        raise ValueError("traj sample and run ID must match [A-Za-z0-9_-]+")

    root = repository_root()
    run_directory = (
        root
        / "base_ws/src/control/trajectory_validator/history"
        / arguments.traj_sample
        / arguments.run_id
    )
    metadata_path = run_directory / "metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError("history run requires metadata.json")
    with metadata_path.open() as stream:
        metadata = json.load(stream)
    if metadata.get("traj_sample") != arguments.traj_sample or metadata.get("run_id") != arguments.run_id:
        raise ValueError("history metadata identity does not match requested run")
    gt_points = None
    if metadata.get("schema_version") == 5:
        tracking_path = run_directory / "tracking_samples.csv"
        if not tracking_path.is_file():
            raise FileNotFoundError("schema-v5 history requires tracking_samples.csv")
        trajectory_metadata = metadata.get("reference_trajectory") or metadata.get(
            "input_trajectory", {}
        )
        trajectory_file = trajectory_metadata.get("file")
        trajectory_path = resolve_repo_file(root, trajectory_file, "input trajectory")
        if trajectory_metadata.get("sha256") != sha256_file(trajectory_path):
            raise ValueError("input trajectory SHA-256 does not match metadata")
        gt_points = read_source_trajectory_xy(trajectory_path)
    elif not (run_directory / "gt.csv").is_file() or not (
        run_directory / "control_cycles.csv"
    ).is_file():
        raise FileNotFoundError(
            "legacy history requires gt.csv and control_cycles.csv"
        )

    map_metadata = metadata.get("map", {})
    map_file = (
        arguments.map_file
        or map_metadata.get("file")
        or "kcity_map/lanelet_map/lanelet2_map_v44.osm"
    )
    projector_file = (
        arguments.projector_file
        or map_metadata.get("projector_file")
        or "kcity_map/lanelet_map/map_projector_info_v1.yaml"
    )
    map_path = resolve_repo_file(root, map_file, "map file")
    projector_path = resolve_repo_file(root, projector_file, "projector file")
    artifact = render_run_summary_svg(
        run_directory,
        map_path,
        projector_path,
        filename=arguments.filename,
        margin_m=arguments.map_margin_m,
        gt_points=gt_points,
    )
    metadata["map"] = {
        **map_metadata,
        "version": map_version_from_path(map_path) or map_metadata.get("version"),
        "file": str(map_path.relative_to(root)),
        "sha256": sha256_file(map_path),
        "projector_file": str(projector_path.relative_to(root)),
        "projector_sha256": sha256_file(projector_path),
    }
    artifacts = metadata.setdefault("artifacts", {})
    artifacts.pop("summary_image", None)
    artifacts["summary_svg"] = artifact
    atomic_json(metadata_path, metadata)
    print("saved {}".format(run_directory / arguments.filename))


if __name__ == "__main__":
    main()
