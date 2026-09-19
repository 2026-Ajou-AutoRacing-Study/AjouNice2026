#!/usr/bin/env python3
"""Validate MORAI Fast-BEV image and 3D geometry augmentation contracts."""

import argparse
import copy
import json
from pathlib import Path

import numpy as np
import torch
from mmcv import Config
from mmcv.utils import import_modules_from_strings
from mmdet3d.core.bbox import LiDARInstance3DBoxes
from mmdet3d.core.points import LiDARPoints
from mmdet3d.datasets import build_dataset
from mmdet3d.datasets.pipelines.transforms_3d import (
    GlobalRotScaleTrans,
    RandomFlip3D,
)
from PIL import Image

from perception.detection3d.fastbev.plugin.pipelines import (
    MoraiAugImageMultiViewImage,
)


def _data(value):
    return getattr(value, "data", value)


def _camera_metadata():
    return {
        "lidar2img": {
            "lidar2img_aug": [
                {
                    "rot": np.eye(3, dtype=np.float32),
                    "tran": np.zeros(3, dtype=np.float32),
                }
                for _ in range(4)
            ]
        }
    }


def validate_flip_sampling(data_config, samples, seed):
    transform = MoraiAugImageMultiViewImage(
        data_config=copy.deepcopy(data_config), backend="opencv", is_train=True)
    rng_state = np.random.get_state()
    np.random.seed(seed)
    flips = sum(
        bool(transform.sample_augmentation(720, 1280)[3])
        for _ in range(samples))
    np.random.set_state(rng_state)
    observed = flips / samples
    expected = float(data_config["flip_prob"])
    # Six sigma remains tight enough to catch an accidental 50% fallback but
    # avoids flaky failures from random sampling.
    tolerance = max(0.01, 6.0 * np.sqrt(expected * (1.0 - expected) / samples))
    if abs(observed - expected) > tolerance:
        raise AssertionError(
            f"image flip rate {observed:.4f} != {expected:.4f} "
            f"within {tolerance:.4f}")
    return observed


def validate_image_geometry(data_config, seed):
    image = np.random.default_rng(seed).integers(
        0, 256, size=(480, 640, 3), dtype=np.uint8)
    pil_transform = MoraiAugImageMultiViewImage(
        data_config=copy.deepcopy(data_config), backend="pil", is_train=True)
    cv_transform = MoraiAugImageMultiViewImage(
        data_config=copy.deepcopy(data_config), backend="opencv", is_train=True)

    np.random.seed(seed)
    pil_params = pil_transform.sample_augmentation(480, 640)
    np.random.seed(seed)
    cv_params = cv_transform.sample_augmentation(480, 640)
    if pil_params != cv_params:
        raise AssertionError("PIL/OpenCV sampled different image geometry")

    post_rot = torch.eye(2)
    post_tran = torch.zeros(2)
    pil_image, pil_rot, pil_tran = pil_transform.img_transform(
        Image.fromarray(image, mode="RGB"), post_rot.clone(), post_tran.clone(),
        resize=pil_params[0], resize_dims=pil_params[1], crop=pil_params[2],
        flip=pil_params[3], rotate=pil_params[4], pad=pil_params[5])
    cv_image, cv_rot, cv_tran = cv_transform.img_transform(
        Image.fromarray(image, mode="RGB"), post_rot.clone(), post_tran.clone(),
        resize=cv_params[0], resize_dims=cv_params[1], crop=cv_params[2],
        flip=cv_params[3], rotate=cv_params[4], pad=cv_params[5])
    if np.asarray(pil_image).shape != np.asarray(cv_image).shape:
        raise AssertionError("PIL/OpenCV image shapes differ")
    if not np.allclose(pil_rot, cv_rot) or not np.allclose(
            pil_tran, cv_tran):
        raise AssertionError("PIL/OpenCV projection geometry differs")
    return {
        "output_shape": list(np.asarray(cv_image).shape),
        "pixel_mae": float(np.mean(np.abs(
            np.asarray(pil_image, dtype=np.float32)
            - np.asarray(cv_image, dtype=np.float32)))),
        "sampled_flip": bool(cv_params[3]),
        "sampled_rotation_deg": float(cv_params[4]),
    }


def validate_3d_annotation_updates():
    original = torch.tensor(
        [[2.0, 1.0, 0.2, 4.5, 1.9, 1.7, 0.3, 0.0, 0.0]],
        dtype=torch.float32)
    sample = _camera_metadata()
    sample.update({
        "gt_bboxes_3d": LiDARInstance3DBoxes(original.clone(), box_dim=9),
        "bbox3d_fields": ["gt_bboxes_3d"],
        "points": LiDARPoints(
            torch.tensor([[2.0, 1.0, 0.2, 0.0, 0.0]], dtype=torch.float32),
            points_dim=5),
    })

    flip = RandomFlip3D(
        flip_2d=False,
        sync_2d=False,
        flip_ratio_bev_horizontal=1.0,
        flip_ratio_bev_vertical=0.0,
        update_img2lidar=True)
    flipped = flip(sample)
    after_flip = flipped["gt_bboxes_3d"].tensor.clone()
    if torch.allclose(original, after_flip):
        raise AssertionError("RandomFlip3D did not update the 3D annotation")

    before_global = after_flip.clone()
    before_projection = np.asarray(
        flipped["lidar2img"]["lidar2img_aug"][0]["rot"]).copy()
    global_transform = GlobalRotScaleTrans(
        rot_range=[0.10, 0.10],
        scale_ratio_range=[1.05, 1.05],
        translation_std=[0.01, 0.01, 0.01],
        update_img2lidar=True)
    np.random.seed(123)
    transformed = global_transform(flipped)
    after_global = transformed["gt_bboxes_3d"].tensor
    after_projection = np.asarray(
        transformed["lidar2img"]["lidar2img_aug"][0]["rot"])
    if torch.allclose(before_global, after_global):
        raise AssertionError("GlobalRotScaleTrans did not update annotations")
    if np.allclose(before_projection, after_projection):
        raise AssertionError("GlobalRotScaleTrans did not update projection")
    if not torch.isfinite(after_global).all() or not np.isfinite(
            after_projection).all():
        raise AssertionError("3D augmentation produced non-finite values")
    if transformed.get("transformation_3d_flow") != ["HF", "R", "S", "T"]:
        raise AssertionError("Unexpected 3D transformation flow")
    return {
        "flow": transformed["transformation_3d_flow"],
        "box_before": original[0, :7].tolist(),
        "box_after": after_global[0, :7].tolist(),
    }


def validate_dataset(config, frames):
    dataset = build_dataset(config.data.train)
    checked = []
    for index in range(min(frames, len(dataset))):
        np.random.seed(1000 + index)
        sample = dataset[index]
        image = _data(sample["img"])
        boxes = _data(sample["gt_bboxes_3d"])
        labels = _data(sample["gt_labels_3d"])
        if tuple(image.shape) != (4, 3, 384, 704):
            raise AssertionError(f"Unexpected image tensor shape: {image.shape}")
        if not torch.isfinite(image).all() or not torch.isfinite(
                boxes.tensor).all():
            raise AssertionError(f"Non-finite dataset sample at index {index}")
        if len(boxes) != len(labels):
            raise AssertionError(f"Box/label mismatch at index {index}")
        checked.append({"index": index, "boxes": len(boxes)})
    return checked


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path,
        default=Path(
            "perception/detection3d/fastbev/configs/"
            "fastbev_m0_morai_4cam_s384x704.py"))
    parser.add_argument("--flip-samples", type=int, default=20000)
    parser.add_argument("--dataset-frames", type=int, default=8)
    parser.add_argument("--seed", type=int, default=260814)
    args = parser.parse_args()

    config = Config.fromfile(str(args.config.resolve()))
    import_modules_from_strings(**config.custom_imports)
    data_config = dict(config.data_config)
    report = {
        "backend": config.image_transform_backend,
        "configured_bev_flip_probability": float(
            config.bev_horizontal_flip_prob),
        "configured_image_flip_probability": float(
            config.image_horizontal_flip_prob),
        "observed_image_flip_probability": validate_flip_sampling(
            data_config, args.flip_samples, args.seed),
        "image_geometry": validate_image_geometry(data_config, args.seed),
        "annotation_geometry": validate_3d_annotation_updates(),
        "dataset_samples": validate_dataset(config, args.dataset_frames),
    }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
