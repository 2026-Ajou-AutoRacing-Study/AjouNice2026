#!/usr/bin/env python3
"""Dependency-light schema/image/projection smoke test for MORAI Fast-BEV infos."""

import argparse
import pickle
from pathlib import Path

import numpy as np
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_root", type=Path)
    parser.add_argument("--split", choices=("train", "val", "test"), default="train")
    parser.add_argument("--samples", type=int, default=20)
    args = parser.parse_args()
    info_path = args.dataset_root / f"fastbev_infos/morai_infos_{args.split}.pkl"
    with info_path.open("rb") as stream:
        payload = pickle.load(stream)
    infos = payload["infos"][: args.samples]
    failures = []
    box_count = 0
    for info in infos:
        if list(info["cams"]) != ["front", "front_left", "front_right", "back"]:
            failures.append(f"{info['token']}: camera order")
        if np.asarray(info["gt_boxes"]).ndim != 2 or np.asarray(info["gt_boxes"]).shape[1] != 7:
            failures.append(f"{info['token']}: gt_boxes shape")
        box_count += len(info["gt_boxes"])
        for name, camera in info["cams"].items():
            path = Path(camera["data_path"])
            if not path.is_file():
                failures.append(f"{info['token']}: missing {name} image")
                continue
            with Image.open(path) as image:
                image.verify()
            intrinsic = np.asarray(camera["cam_intrinsic"])
            rotation = np.asarray(camera["sensor2lidar_rotation"])
            if intrinsic.shape != (3, 3) or rotation.shape != (3, 3):
                failures.append(f"{info['token']}: malformed {name} calibration")
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
                failures.append(f"{info['token']}: non-orthonormal {name} rotation")
    if failures:
        for failure in failures[:20]:
            print("FAIL:", failure)
        raise SystemExit(f"smoke failed: {len(failures)} issue(s)")
    print(f"smoke passed: split={args.split}, samples={len(infos)}, cameras={len(infos) * 4}, boxes={box_count}")


if __name__ == "__main__":
    main()
