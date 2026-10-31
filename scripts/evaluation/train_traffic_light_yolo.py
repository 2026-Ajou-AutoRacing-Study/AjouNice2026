#!/usr/bin/env python3
"""Isolated research YOLO26 fine-tuning from an explicit local official COCO checkpoint."""
import argparse
from contextlib import contextmanager
import importlib.metadata
import json
import os
from pathlib import Path
import tempfile

from traffic_light_yolo_dataset import digest, load_dataset, write_data_yaml

BASELINE_SHA256 = "2bb73720568a975c94d79a54cc097a0a4b70ec7c8de23552b4d0e71112160818"
ULTRALYTICS_VERSION = "8.4.143"


@contextmanager
def offline_runtime():
    keys = {key: os.environ.get(key) for key in ("YOLO_OFFLINE", "YOLO_CONFIG_DIR", "MPLCONFIGDIR", "YOLO_AUTOINSTALL", "WANDB_MODE")}
    with tempfile.TemporaryDirectory(prefix="traffic-yolo-") as directory:
        for name in ("settings", "matplotlib"):
            (Path(directory) / name).mkdir()
        os.environ.update(YOLO_OFFLINE="true", YOLO_CONFIG_DIR=str(Path(directory)/"settings"),
                          MPLCONFIGDIR=str(Path(directory)/"matplotlib"), YOLO_AUTOINSTALL="false", WANDB_MODE="disabled")
        try:
            yield
        finally:
            for key, value in keys.items():
                if value is None:os.environ.pop(key, None)
                else:os.environ[key] = value


def training_arguments(data, output, *, imgsz=960, epochs=80, batch=16, device="0", seed=0, max_hours=None,
                       patience=20, save_period=-1):
    if imgsz not in (640, 960, 1280) or type(epochs) is not int or epochs < 1 or type(batch) is not int or batch < 1:
        raise ValueError("Explicit supported resolution/positive epoch/batch required")
    if type(patience) is not int or patience<0 or type(save_period) is not int or save_period==0 or save_period< -1:
        raise ValueError("Patience must be nonnegative; save period is -1 or a positive epoch interval")
    return {"data": str(Path(data).resolve()), "project": str(Path(output).resolve()), "name": "fit", "exist_ok": False,
            "imgsz": imgsz, "epochs": epochs, "batch": batch, "device": device, "seed": seed,
            "deterministic": True, "workers": 2, "cache": False, "amp": False, "compile": False,
            "optimizer": "AdamW", "lr0": .001, "lrf": .01, "weight_decay": .0005,
            "warmup_epochs": 2., "cos_lr": True, "patience": patience, "time": max_hours,
            "rect": False, "mosaic": 0., "close_mosaic": 0, "mixup": 0., "copy_paste": 0.,
            "degrees": 0., "perspective": 0., "translate": .05, "scale": .1,
            "fliplr": 0., "flipud": 0., "hsv_h": 0., "hsv_s": .1, "hsv_v": .1,
            "plots": True, "save": True, "save_period":save_period, "nms":None,
            "val": True, "split": "val", "verbose": True}


def run(dataset, pretrained, output, *, development_pilot=False, **configuration):
    output, pretrained = Path(output).resolve(), Path(pretrained).resolve(strict=True)
    if output.exists() or output.is_symlink():raise FileExistsError("Research output must be fresh")
    if pretrained.name not in ("yolo26n.pt", "yolo26s.pt") or digest(pretrained) == BASELINE_SHA256:
        raise ValueError("Use explicit official pretrained YOLO26n/s, never overwrite/fine-tune operational best.pt")
    _, metadata = load_dataset(dataset, development_pilot=development_pilot)
    if importlib.metadata.version("ultralytics") != ULTRALYTICS_VERSION:
        raise ValueError("Pin ultralytics==" + ULTRALYTICS_VERSION)
    output.mkdir(parents=True, exist_ok=False)
    data_yaml = output / "resolved_data.yaml"
    write_data_yaml(data_yaml, Path(dataset).resolve().parent, include_test=metadata["split_image_counts"].get("test", 0) > 0)
    arguments = training_arguments(data_yaml, output, **configuration)
    request = {"research_only": True, "not_production": True, "dataset": metadata,
               "runtime_versions": {package: importlib.metadata.version(package) for package in
                                    ("ultralytics", "torch", "torchvision", "numpy", "polars", "pillow")},
               "pretrained": {"path": str(pretrained), "sha256": digest(pretrained)},
               "script_sha256": digest(__file__), "dataset_tool_sha256": digest(Path(__file__).with_name("traffic_light_yolo_dataset.py")),
               "arguments": arguments, "test_used_for_model_selection": False,
               "candidate_selection_policy":"Validation mAP plus fixed-confidence precision/recall/FP and score separation; default best.pt mAP alone is not an operating-threshold qualification",
               "inference_head":"default one-to-many with NMS; train/val/predict nms=None"}
    (output / "train_request.json").write_text(json.dumps(request, indent=2))
    # Preserve the exact trainer/label verifier used, independent of later workspace edits.
    source_dir=output/"source_snapshot";source_dir.mkdir()
    for source in (Path(__file__),Path(__file__).with_name("traffic_light_yolo_dataset.py")):
        (source_dir/source.name).write_bytes(source.read_bytes())
    status = {"status": "failed", "research_only": True}
    try:
        with offline_runtime():
            from ultralytics import YOLO, settings
            settings.update({key: False for key in ("sync", "wandb", "mlflow", "tensorboard", "hub") if key in settings})
            model = YOLO(str(pretrained))
            if len(model.names) != 80:
                raise ValueError("Official COCO 80-class initialization required")
            result = model.train(**arguments)
            status.update(status="complete", best=str(model.trainer.best),
                          best_sha256=digest(model.trainer.best), metrics=result.results_dict,
                          ultralytics=ULTRALYTICS_VERSION)
    except BaseException as error:
        status["error"] = type(error).__name__ + ": " + str(error)
        raise
    finally:
        (output / "train_status.json").write_text(json.dumps(status, indent=2, default=str))
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path, help="Exported YOLO manifest.json")
    parser.add_argument("--pretrained", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="0")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-hours", type=float)
    parser.add_argument("--patience",type=int,default=20,help="0 disables early stopping for fixed-epoch experiments")
    parser.add_argument("--save-period",type=int,default=-1,help="Positive epoch interval for validation-only candidate selection")
    parser.add_argument("--development-pilot", action="store_true")
    print(json.dumps(run(**vars(parser.parse_args())), default=str))


if __name__ == "__main__":main()
