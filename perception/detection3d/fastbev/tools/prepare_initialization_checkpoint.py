#!/usr/bin/env python3
"""Create a shape-safe Fast-BEV initialization checkpoint and audit report."""

import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import torch
from mmcv import Config
from mmcv.utils import import_modules_from_strings
from mmdet3d.models import build_model


ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = (
    ROOT
    / "perception/detection3d/fastbev/configs/"
    "fastbev_m0_morai_4cam_s384x704.py"
)
MODE_PREFIXES = {
    "backbone": ("backbone.",),
    "image": ("backbone.", "neck."),
    "image_bev": ("backbone.", "neck.", "neck_fuse_0.", "neck_3d."),
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--mode", choices=sorted(MODE_PREFIXES), required=True)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--label", required=True)
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalize_state_dict(checkpoint):
    state_dict = checkpoint.get("state_dict", checkpoint)
    if not isinstance(state_dict, dict):
        raise TypeError("checkpoint state_dict must be a mapping")
    return {
        key[7:] if key.startswith("module.") else key: value
        for key, value in state_dict.items()
    }


def parameter_count(tensors):
    return sum(value.numel() for value in tensors.values())


def family(key):
    return key.split(".", 1)[0]


def main():
    args = parse_args()
    source_path = Path(args.source).resolve()
    output_path = Path(args.output).resolve()
    report_path = Path(args.report).resolve()
    config_path = Path(args.config).resolve()
    if not source_path.is_file():
        raise FileNotFoundError(source_path)

    cfg = Config.fromfile(str(config_path))
    import_modules_from_strings(**cfg.custom_imports)
    model = build_model(
        cfg.model,
        train_cfg=cfg.get("train_cfg"),
        test_cfg=cfg.get("test_cfg"),
    )
    target_state = model.state_dict()

    checkpoint = torch.load(source_path, map_location="cpu", weights_only=False)
    source_state = normalize_state_dict(checkpoint)
    allowed_prefixes = MODE_PREFIXES[args.mode]

    selected = {}
    excluded_policy = []
    excluded_missing = []
    excluded_shape = []
    for key, value in source_state.items():
        if not key.startswith(allowed_prefixes):
            excluded_policy.append(key)
            continue
        if key not in target_state:
            excluded_missing.append(key)
            continue
        if tuple(value.shape) != tuple(target_state[key].shape):
            excluded_shape.append(
                {
                    "key": key,
                    "source_shape": list(value.shape),
                    "target_shape": list(target_state[key].shape),
                }
            )
            continue
        selected[key] = value

    # Keep parameterized modules atomic.  Loading only a bias when the matching
    # weight has an incompatible shape is technically accepted by PyTorch but
    # produces a hybrid layer that was never trained as a unit.
    mismatched_modules = {
        item["key"].rsplit(".", 1)[0] for item in excluded_shape
    }
    excluded_atomic = []
    for key in list(selected):
        if key.rsplit(".", 1)[0] in mismatched_modules:
            excluded_atomic.append(key)
            del selected[key]

    if not selected:
        raise RuntimeError("No compatible tensors were selected")

    selected_by_family = defaultdict(dict)
    target_by_family = defaultdict(dict)
    for key, value in selected.items():
        selected_by_family[family(key)][key] = value
    for key, value in target_state.items():
        target_by_family[family(key)][key] = value

    family_report = {}
    for name in sorted(set(target_by_family) | set(selected_by_family)):
        selected_params = parameter_count(selected_by_family[name])
        target_params = parameter_count(target_by_family[name])
        family_report[name] = {
            "selected_tensors": len(selected_by_family[name]),
            "target_tensors": len(target_by_family[name]),
            "selected_parameters": selected_params,
            "target_parameters": target_params,
            "parameter_coverage": (
                selected_params / target_params if target_params else 0.0
            ),
        }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "meta": {
            "initialization_label": args.label,
            "selection_mode": args.mode,
            "allowed_prefixes": list(allowed_prefixes),
            "source_path": str(source_path),
            "source_sha256": sha256(source_path),
            "target_config": str(config_path),
        },
        "state_dict": selected,
    }
    torch.save(payload, output_path)

    selected_parameters = parameter_count(selected)
    target_parameters = parameter_count(target_state)
    report = {
        "label": args.label,
        "mode": args.mode,
        "source": str(source_path),
        "source_sha256": payload["meta"]["source_sha256"],
        "output": str(output_path),
        "output_sha256": sha256(output_path),
        "target_config": str(config_path),
        "allowed_prefixes": list(allowed_prefixes),
        "source_tensor_count": len(source_state),
        "target_tensor_count": len(target_state),
        "selected_tensor_count": len(selected),
        "selected_parameter_count": selected_parameters,
        "target_parameter_count": target_parameters,
        "target_parameter_coverage": selected_parameters / target_parameters,
        "families": family_report,
        "excluded": {
            "policy_count": len(excluded_policy),
            "missing_count": len(excluded_missing),
            "shape_count": len(excluded_shape),
            "atomic_count": len(excluded_atomic),
            "missing_keys": excluded_missing,
            "shape_mismatches": excluded_shape,
            "atomic_keys": excluded_atomic,
            "atomic_modules": sorted(mismatched_modules),
        },
    }
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print("label:", args.label)
    print("mode:", args.mode)
    print("selected tensors:", len(selected), "/", len(target_state))
    print(
        "target parameter coverage:",
        f"{100.0 * report['target_parameter_coverage']:.2f}%",
    )
    for name, values in family_report.items():
        if values["selected_tensors"]:
            print(
                f"{name}: tensors={values['selected_tensors']}/"
                f"{values['target_tensors']}, parameter coverage="
                f"{100.0 * values['parameter_coverage']:.2f}%"
            )
    print("shape mismatches:", len(excluded_shape))
    print("atomic companion exclusions:", len(excluded_atomic))
    print("output:", output_path)
    print("report:", report_path)


if __name__ == "__main__":
    main()
