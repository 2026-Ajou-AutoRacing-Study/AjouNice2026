#!/usr/bin/env python3
"""Build matched raw/ego-deskew Base4 highway experiment datasets.

The input deck combines previously validated hard-case and continuous-speed
captures with one or more new gated-bag directories.  Both outputs use the
same bag list and conversion contract so a raw-versus-deskew comparison does
not accidentally change the scene distribution.
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_AUDITS = (
    REPO_ROOT
    / "artifacts/datasets/morai_base4_velocity_hardcase_v2_raw16/conversion_audit.json",
    REPO_ROOT
    / "artifacts/datasets/morai_base4_highspeed_jitter_raw_v1/conversion_audit.json",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--new-bag-dir",
        action="append",
        type=Path,
        default=[],
        help="directory containing newly accepted .bag files; repeatable",
    )
    parser.add_argument(
        "--source-audit",
        action="append",
        type=Path,
        default=[],
        help="conversion_audit.json whose source bags are reused; repeatable",
    )
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument(
        "--raw-output",
        type=Path,
        default=REPO_ROOT
        / "artifacts/datasets/morai_base4_highway_first_experiment_v1_raw",
    )
    parser.add_argument(
        "--deskew-output",
        type=Path,
        default=REPO_ROOT
        / "artifacts/datasets/morai_base4_highway_first_experiment_v1_ego_deskew",
    )
    parser.add_argument(
        "--raw-serialized-root",
        default="/datasets/base4_highway_first_experiment_v1_raw",
    )
    parser.add_argument(
        "--deskew-serialized-root",
        default="/datasets/base4_highway_first_experiment_v1_ego_deskew",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def audit_bags(path):
    payload = json.loads(path.read_text())
    bags = payload.get("bags")
    if not isinstance(bags, list) or not bags:
        raise RuntimeError(f"audit has no source bags: {path}")
    return [Path(item) for item in bags]


def unique_existing_bags(paths):
    unique = []
    seen = set()
    missing = []
    for path in paths:
        resolved = path.expanduser().resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        if not resolved.is_file():
            missing.append(resolved)
        else:
            unique.append(resolved)
    if missing:
        sample = "\n".join(f"  {path}" for path in missing[:10])
        raise FileNotFoundError(f"{len(missing)} source bags are missing:\n{sample}")
    if not unique:
        raise RuntimeError("no input bags selected")
    return unique


def converter_command(bags, calibration, output, serialized_root, deskew):
    command = [
        sys.executable,
        str(
            REPO_ROOT
            / "perception/detection3d/bevfusion/tools/build_base3_online_bevfusion_infos.py"
        ),
        *[str(path) for path in bags],
        "--calibration",
        str(calibration),
        "--asset-offsets",
        str(
            REPO_ROOT
            / "dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json"
        ),
        "--output-root",
        str(output),
        "--serialized-root",
        serialized_root,
        "--samples-per-scene",
        "16",
        "--window-start-event",
        "sequence_start",
        "--with-velocity-targets",
        "--causal-sweep-count",
        "3",
        "--minimum-gt-points",
        "5",
        "--state-alignment",
        "interpolate",
        "--ego-heading-correction-deg",
        "-1.347",
        "--state-invalid-policy",
        "drop",
        "--max-camera-age-sec",
        "0.1",
        "--require-positive-class",
        "vehicle",
        "--distance-filter",
        "box-range",
    ]
    if deskew:
        command.extend(
            [
                "--lidar-deskew",
                "ego-pose-2d",
                "--lidar-deskew-reference",
                "end",
                "--deskew-max-point-time-span-sec",
                "0.15",
                "--deskew-invalid-policy",
                "drop",
            ]
        )
    else:
        command.extend(["--lidar-deskew", "off"])
    return command


def main():
    args = parse_args()
    audits = args.source_audit or list(DEFAULT_AUDITS)
    candidates = []
    for audit in audits:
        candidates.extend(audit_bags(audit.expanduser().resolve()))
    for directory in args.new_bag_dir:
        candidates.extend(sorted(directory.expanduser().resolve().rglob("*.bag")))
    bags = unique_existing_bags(candidates)
    calibration = args.calibration.expanduser().resolve()
    if not calibration.is_file():
        raise FileNotFoundError(calibration)

    print(f"selected bags: {len(bags)}")
    print(f"raw output: {args.raw_output}")
    print(f"deskew output: {args.deskew_output}")
    commands = (
        converter_command(
            bags,
            calibration,
            args.raw_output.expanduser().resolve(),
            args.raw_serialized_root,
            False,
        ),
        converter_command(
            bags,
            calibration,
            args.deskew_output.expanduser().resolve(),
            args.deskew_serialized_root,
            True,
        ),
    )
    if args.dry_run:
        for command in commands:
            print(" ".join(command))
        return
    for output in (args.raw_output, args.deskew_output):
        if output.exists():
            raise FileExistsError(f"refusing to overwrite dataset: {output}")
    for command in commands:
        subprocess.run(command, cwd=REPO_ROOT, check=True)


if __name__ == "__main__":
    main()
