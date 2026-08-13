#!/usr/bin/env python3
"""Compare the reference MMDetection and optimized fixed-camera preprocessors."""

import argparse
import copy
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np
import torch
from mmcv import Config
from mmcv.utils import import_modules_from_strings
from mmdet.datasets.pipelines import Compose
from mmdet3d.core.bbox import get_box_type

CAMERA_NAMES = ("front", "front_left", "front_right", "back")


def load_node_module(repo_root):
    path = repo_root / "base_ws/src/morai_fastbev_ros/scripts/fastbev_online_node.py"
    spec = importlib.util.spec_from_file_location("fastbev_online_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_images(image_dir):
    images = []
    for name in CAMERA_NAMES:
        candidates = list(image_dir.glob(f"{name}.*"))
        if len(candidates) != 1:
            raise RuntimeError(
                f"Expected one {name}.* in {image_dir}, got {candidates}")
        image = cv2.imread(str(candidates[0]), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Cannot decode {candidates[0]}")
        images.append(image)
    return images


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path.home() / "AjouNice2026")
    parser.add_argument("--image-dir", type=Path, required=True)
    parser.add_argument(
        "--config", type=Path,
        default=Path("perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704.py"))
    parser.add_argument(
        "--sensor-config", type=Path, default=Path("configs/sensors/Sensor_6.json"))
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--backend", choices=("fixed", "opencv"), default="fixed")
    args = parser.parse_args()
    repo_root = args.repo_root.expanduser().resolve()
    config_path = args.config if args.config.is_absolute() else repo_root / args.config
    sensor_path = (
        args.sensor_config if args.sensor_config.is_absolute()
        else repo_root / args.sensor_config)
    module = load_node_module(repo_root)

    node = module.FastBEVOnlineNode.__new__(module.FastBEVOnlineNode)
    node.cfg = Config.fromfile(str(config_path))
    import_modules_from_strings(**node.cfg.custom_imports)
    node.box_type_3d, node.box_mode_3d = get_box_type("LiDAR")
    pipeline_cfg = copy.deepcopy(node.cfg.data.test.pipeline)
    pipeline_cfg[0] = dict(type="MoraiMultiViewArrayPipeline", n_images=4)
    node.pipeline = Compose(pipeline_cfg)
    node.calibration = module._camera_calibration(sensor_path)
    node.projection_metadata = module._projection_metadata(node.calibration)
    node.pin_memory = False
    node._initialize_online_preprocessor()

    images = load_images(args.image_dir.expanduser().resolve())
    reference, reference_metas = node._prepare_reference(images, "validation")
    if args.backend == "opencv":
        optimized, optimized_metas = node._prepare_opencv(images)
    else:
        optimized, optimized_metas = node._prepare_fixed(images)
    difference = (reference.float() - optimized.float()).abs()
    reference_scale = reference.float().abs().mean().item()
    report = {
        "shape": list(reference.shape),
        "backend": args.backend,
        "mean_abs_error": difference.mean().item(),
        "p95_abs_error": torch.quantile(difference, 0.95).item(),
        "max_abs_error": difference.max().item(),
        "relative_mean_abs_error": difference.mean().item() / max(reference_scale, 1e-12),
        "metadata_extrinsic_max_abs_error": max(
            float(np.max(np.abs(left - right)))
            for left, right in zip(
                reference_metas[0]["lidar2img"]["extrinsic"],
                optimized_metas[0]["lidar2img"]["extrinsic"])),
    }
    print(json.dumps(report, indent=2, sort_keys=True))

    if args.checkpoint:
        checkpoint_path = (
            args.checkpoint if args.checkpoint.is_absolute()
            else repo_root / args.checkpoint)
        node.repo_root = repo_root
        node.config_path = config_path
        node.checkpoint_path = checkpoint_path
        node.use_fp16 = True
        node.cudnn_benchmark = True
        node.class_names = ["vehicle", "pedestrian", "obstacle"]
        node.score_thresholds = {name: 0.2 for name in node.class_names}
        node.max_detection_range = float("inf")
        node._load_model_and_pipeline()
        node._initialize_online_preprocessor()
        with torch.no_grad():
            ref_boxes, ref_scores, ref_labels = node._infer(
                reference.unsqueeze(0).cuda(), reference_metas)
            opt_boxes, opt_scores, opt_labels = node._infer(
                optimized.unsqueeze(0).cuda(), optimized_metas)
        comparable = (
            len(ref_scores) == len(opt_scores)
            and np.array_equal(ref_labels, opt_labels))
        model_report = {
            "reference_detections": len(ref_scores),
            "optimized_detections": len(opt_scores),
            "ordered_labels_equal": bool(comparable),
        }
        if comparable and len(ref_scores):
            model_report.update({
                "score_mean_abs_error": float(
                    np.mean(np.abs(ref_scores - opt_scores))),
                "score_max_abs_error": float(
                    np.max(np.abs(ref_scores - opt_scores))),
                "box_mean_abs_error": float(
                    np.mean(np.abs(ref_boxes - opt_boxes))),
                "box_max_abs_error": float(
                    np.max(np.abs(ref_boxes - opt_boxes))),
            })
        print(json.dumps({"model_output": model_report}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
