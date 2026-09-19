#!/usr/bin/env python3
"""Run SaveSensorData collection for selected components of one OD batch."""

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"
CAPTURE_SCRIPT = DATASET_ROOT / "scripts/ros_scenario_autocapture_poc.py"
PRIMARY_COMPONENTS = ("mission", "static", "hard_dynamic", "hard_static")
RESERVE_COMPONENTS = ("mission_reserve", "static_reserve", "hard_dynamic_reserve")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-index", type=int, default=1)
    parser.add_argument("--capture-root", type=Path, default=Path.home() / "morai_cap")
    parser.add_argument("--component", action="append", choices=PRIMARY_COMPONENTS + RESERVE_COMPONENTS)
    parser.add_argument("--include-reserve", action="store_true")
    # Production batches retry transient MORAI actor omissions. Smoke tests
    # may still request one attempt explicitly when diagnosing a scene.
    parser.add_argument("--scenario-load-attempts", type=int, default=3)
    return parser.parse_args()


def manifest_for(tag, component):
    group = f"dynamic_dataset_{tag}_{component}"
    suffix = (
        "_sunny_t13.json"
        if component in {"static", "static_reserve", "hard_static"}
        else "_runtime.json"
    )
    return DATASET_ROOT / "manifests" / f"{group}{suffix}"


def motion_mode(component):
    return "rolling-grouped" if component in {"static", "static_reserve", "hard_static"} else "timed-grouped"


def main():
    args = parse_args()
    tag = f"batch{args.batch_index:03d}"
    components = list(args.component or PRIMARY_COMPONENTS)
    if args.include_reserve and args.component is None:
        components.extend(RESERVE_COMPONENTS)
    run_tag = datetime.now().strftime("%y%m%d_%H%M%S")
    for component in components:
        manifest = manifest_for(tag, component)
        if not manifest.is_file():
            raise FileNotFoundError(manifest)
        capture_dir = args.capture_root / f"{tag}_{component}_{run_tag}"
        command = [
            sys.executable,
            CAPTURE_SCRIPT,
            "--manifest", manifest,
            "--capture-dir", capture_dir,
            "--motion-mode", motion_mode(component),
            "--scenario-load-attempts", str(args.scenario_load_attempts),
            "--continue-on-scene-error",
        ]
        print("+", " ".join(str(item) for item in command), flush=True)
        subprocess.run([str(item) for item in command], cwd=PROJECT_ROOT, check=True)


if __name__ == "__main__":
    main()
