#!/usr/bin/env python3
"""Build a tiny scene-contiguous info subset for Q1 recurrent smoke tests."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--frames-per-split", type=int, default=4)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.frames_per_split < 2:
        parser.error("Q1 smoke requires at least two consecutive frames")
    args.output_root.mkdir(parents=True, exist_ok=True)
    report = {}
    for split in ("train", "val", "test"):
        source = args.input_root / f"morai_base3_infos_{split}.pkl"
        if not source.is_file():
            if split == "test":
                source = args.input_root / "morai_base3_infos_val.pkl"
            else:
                raise FileNotFoundError(source)
        target = args.output_root / f"morai_base3_infos_{split}.pkl"
        if target.exists() and not args.force:
            raise FileExistsError(target)
        with source.open("rb") as stream:
            payload = pickle.load(stream)
        rows = payload["data_list"]
        if not rows and split == "test":
            source = args.input_root / "morai_base3_infos_val.pkl"
            with source.open("rb") as stream:
                payload = pickle.load(stream)
            rows = payload["data_list"]
        if not rows:
            raise RuntimeError(f"{split}: no rows in {source}")
        first_scene = str(rows[0]["scene_id"])
        selected = [
            row for row in rows if str(row["scene_id"]) == first_scene
        ][:args.frames_per_split]
        if len(selected) < 2:
            raise RuntimeError(f"{split}: first scene has fewer than two frames")
        output = dict(payload)
        output["data_list"] = selected
        with target.open("wb") as stream:
            pickle.dump(output, stream, protocol=pickle.HIGHEST_PROTOCOL)
        report[split] = {
            "scene_id": first_scene,
            "frames": len(selected),
            "frame_indices": [row["sequence_frame_index"] for row in selected],
        }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
