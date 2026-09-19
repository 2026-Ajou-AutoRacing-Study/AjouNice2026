#!/usr/bin/env python3
"""Re-evaluate BEVFusion checkpoints and log one corrected W&B val run.

The public entry point runs on the host and starts the team BEVFusion Docker
image.  A private worker mode runs inside that container so evaluation always
uses the same MMDetection3D/CUDA environment as training.

Example:

    python3 perception/detection3d/bevfusion/tools/\
evaluate_corrected_checkpoint_sweep.py \
      --run-dir artifacts/detection3d/bevfusion/team_training/sw/\
bf-sw-fresh-e40-rtx5060ti-260918-005536 \
      --epochs 1-40 \
      --wandb-run-name bf-sw-fresh-e40-range30-corrected-val
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Dict, Iterable, List


DEFAULT_IMAGE = "ajounice/bevfusion:team-cu128-v1"
DEFAULT_CONFIG = (
    "perception/detection3d/bevfusion/configs/"
    "bevfusion_base3_b0_4cam_lidar.py"
)
DEFAULT_INFO_ROOT = "artifacts/datasets/morai_base3_bevfusion_portable_v1"
DEFAULT_CAPTURE_ROOT = "/home/luv/morai_cap/260829_034057_base3_static_10k"
DEFAULT_EVAL_BATCH_SIZE = 4
EXPECTED_VAL_GT = {
    "val/center/vehicle_GT": 1100.0,
    "val/center/pedestrian_GT": 1005.0,
    "val/center/obstacle_GT": 2185.0,
}


def fail(message: str) -> "NoReturn":
    raise SystemExit("[ERROR] " + message)


def parse_epochs(specification: str) -> List[int]:
    """Parse comma-separated epochs and inclusive ranges."""
    epochs = set()
    for token in specification.split(","):
        token = token.strip()
        if not token:
            continue
        if "-" in token:
            first_text, last_text = token.split("-", 1)
            first, last = int(first_text), int(last_text)
            if first > last:
                fail("invalid descending epoch range: {}".format(token))
            epochs.update(range(first, last + 1))
        else:
            epochs.add(int(token))
    if not epochs or min(epochs) <= 0:
        fail("--epochs must contain positive epoch numbers")
    return sorted(epochs)


def resolve_path(value: str, base: Path) -> Path:
    path = Path(os.path.expandvars(os.path.expanduser(value)))
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def find_mmengine_metrics(output_dir: Path, prefix: str) -> Dict[str, float]:
    candidates = sorted(
        path for path in output_dir.glob("*/*.json")
        if path.name != "metrics.json"
    )
    for path in reversed(candidates):
        for line in reversed(path.read_text().splitlines()):
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict) and prefix + "/center/mAP" in value:
                return value
    fail("MMEngine metric JSON was not found under {}".format(output_dir))


def validate_contract(metrics: Dict[str, float], split: str) -> None:
    if split != "val":
        return
    mismatches = []
    for key, expected in EXPECTED_VAL_GT.items():
        actual = metrics.get(key)
        if actual != expected:
            mismatches.append("{}={!r} (expected {})".format(key, actual, expected))
    if mismatches:
        fail("corrected range contract mismatch: " + ", ".join(mismatches))


def load_or_evaluate(
        epoch: int, run_dir: Path, output_root: Path, config: Path,
        info_root: Path, split: str, batch_size: int,
        force: bool) -> Dict[str, float]:
    output_dir = output_root / "e{}".format(epoch)
    normalized_path = output_dir / "metrics.json"
    if normalized_path.is_file() and not force:
        payload = json.loads(normalized_path.read_text())
        metrics = payload["metrics"]
        validate_contract(metrics, split)
        print("[cache] E{} {}".format(epoch, normalized_path), flush=True)
        return metrics

    checkpoint = run_dir / "epoch_{}.pth".format(epoch)
    if not checkpoint.is_file():
        fail("checkpoint is missing: {}".format(checkpoint))
    info = info_root / "morai_base3_infos_{}.pkl".format(split)
    if not info.is_file():
        fail("dataset info is missing: {}".format(info))

    output_dir.mkdir(parents=True, exist_ok=True)
    console_path = output_dir / "console.log"
    command = [
        sys.executable,
        "/opt/mmdetection3d/tools/test.py",
        str(config),
        str(checkpoint),
        "--work-dir", str(output_dir),
        "--cfg-options",
        "test_dataloader.dataset.ann_file={}".format(info),
        "test_dataloader.batch_size={}".format(batch_size),
        "test_dataloader.num_workers=2",
        "test_dataloader.persistent_workers=true",
        "test_evaluator.prefix={}".format(split),
    ]
    print("[start] E{} {} evaluation".format(epoch, split), flush=True)
    child_environment = os.environ.copy()
    # The outer worker owns the single corrected W&B run.  Disable the W&B
    # visualizer instantiated by each independent MMEngine test subprocess.
    child_environment["WANDB_MODE"] = "disabled"
    with console_path.open("w", buffering=1) as console:
        process = subprocess.run(
            command, stdout=console, stderr=subprocess.STDOUT,
            env=child_environment, check=False)
    if process.returncode:
        tail = console_path.read_text(errors="replace").splitlines()[-80:]
        fail("E{} evaluation failed:\n{}".format(epoch, "\n".join(tail)))

    metrics = find_mmengine_metrics(output_dir, split)
    validate_contract(metrics, split)
    atomic_json(normalized_path, {
        "schema_version": 1,
        "checkpoint_epoch": epoch,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": sha256(checkpoint),
        "split": split,
        "evaluation_contract": {
            "point_cloud_range": [-30.0, -30.0, -3.0, 30.0, 30.0, 2.0],
            "filter": "axis_aligned_box_center_for_gt_and_prediction",
            "evaluation_batch_size": batch_size,
        },
        "metrics": metrics,
    })
    print(
        "[done] E{} center_mAP={:.6f} IoU_mAP@0.50={:.6f}"
        .format(epoch, metrics[split + "/center/mAP"],
                metrics[split + "/iou/mAP_0.50"]),
        flush=True,
    )
    return metrics


def run_worker(args: argparse.Namespace) -> None:
    run_dir = Path(args.run_dir)
    output_root = Path(args.output_root)
    config = Path(args.config)
    info_root = Path(args.info_root)
    epochs = parse_epochs(args.epochs)

    wandb_run = None
    if not args.no_wandb:
        import wandb

        wandb_run = wandb.init(
            entity=args.wandb_entity,
            project=args.wandb_project,
            name=args.wandb_run_name,
            id=args.wandb_run_id,
            resume="allow",
            group=args.wandb_group,
            job_type="evaluation",
            tags=["bevfusion", "base3", "corrected-range30", "validation"],
            config={
                "source_training_run": run_dir.name,
                "checkpoint_epochs": epochs,
                "split": args.split,
                "point_cloud_range": [-30, -30, -3, 30, 30, 2],
                "evaluation_batch_size": args.batch_size,
                "evaluation_contract":
                    "axis-aligned box-center filter on GT and predictions",
            },
        )
        wandb.define_metric("epoch")
        wandb.define_metric("checkpoint_epoch", step_metric="epoch")
        wandb.define_metric("val/*", step_metric="epoch")
        wandb.define_metric("test/*", step_metric="epoch")

    try:
        for epoch in epochs:
            metrics = load_or_evaluate(
                epoch, run_dir, output_root, config, info_root,
                args.split, args.batch_size, args.force)
            if wandb_run is not None:
                numeric_metrics = {
                    key: float(value) for key, value in metrics.items()
                    if isinstance(value, (int, float))
                }
                wandb_run.log({
                    "epoch": epoch,
                    "checkpoint_epoch": epoch,
                    **numeric_metrics,
                }, step=epoch)
                print("[wandb] logged E{}".format(epoch), flush=True)
    finally:
        if wandb_run is not None:
            wandb_run.finish()


def host_command(args: argparse.Namespace) -> List[str]:
    repo_root = Path(__file__).resolve().parents[4]
    run_dir = resolve_path(args.run_dir, repo_root)
    info_root = resolve_path(args.info_root, repo_root)
    capture_root = resolve_path(args.capture_root, repo_root)
    config = resolve_path(args.config, repo_root)
    epochs = parse_epochs(args.epochs)

    for path, label in ((run_dir, "run directory"),
                        (info_root, "info root"),
                        (capture_root, "capture root")):
        if not path.is_dir():
            fail("{} is missing: {}".format(label, path))
    if not config.is_file():
        fail("config is missing: {}".format(config))
    for epoch in epochs:
        checkpoint = run_dir / "epoch_{}.pth".format(epoch)
        if not checkpoint.is_file():
            fail("checkpoint is missing: {}".format(checkpoint))

    output_root = run_dir / args.output_subdir
    output_root.mkdir(parents=True, exist_ok=True)
    container_home = output_root / ".container_home"
    container_home.mkdir(exist_ok=True)
    wandb_dir = output_root / "wandb"
    wandb_dir.mkdir(exist_ok=True)

    uid, gid = os.getuid(), os.getgid()
    command = [
        "docker", "run", "--rm", "--gpus", "all",
        "--shm-size={}".format(args.shm_size),
        "--name", args.container_name,
        "--user", "{}:{}".format(uid, gid),
        "-v", "{}:/workspace/AjouNice2026:ro".format(repo_root),
        "-v", "{}:/datasets/base3:ro".format(capture_root),
        "-v", "{}:/datasets/infos:ro".format(info_root),
        "-v", "{}:/evaluation_run".format(run_dir),
        "-w", "/workspace/AjouNice2026/perception/detection3d/bevfusion",
        "-e", "HOME=/evaluation_run/{}/.container_home".format(
            args.output_subdir),
        "-e", "PYTHONPATH=/workspace/AjouNice2026:/opt/mmdetection3d",
        "-e", "TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1",
        "-e", "WANDB_MODE={}".format("disabled" if args.no_wandb else "online"),
        "-e", "WANDB_DISABLE_CODE=true",
        "-e", "WANDB_DIR=/evaluation_run/{}/wandb".format(
            args.output_subdir),
    ]
    netrc = Path.home() / ".netrc"
    if not args.no_wandb:
        if not netrc.is_file():
            fail("~/.netrc is missing; run `wandb login` first or use --no-wandb")
        command.extend([
            "-v", "{}:/evaluation_run/{}/.container_home/.netrc:ro".format(
                netrc, args.output_subdir),
        ])
    command.extend([
        args.image,
        "python",
        "/workspace/AjouNice2026/" + DEFAULT_CONFIG.replace(
            "configs/bevfusion_base3_b0_4cam_lidar.py",
            "tools/evaluate_corrected_checkpoint_sweep.py"),
        "--inside-container",
        "--run-dir", "/evaluation_run",
        "--output-root", "/evaluation_run/{}".format(args.output_subdir),
        "--config", "/workspace/AjouNice2026/{}".format(
            config.relative_to(repo_root)),
        "--info-root", "/datasets/infos",
        "--epochs", args.epochs,
        "--split", args.split,
        "--batch-size", str(args.batch_size),
        "--wandb-entity", args.wandb_entity,
        "--wandb-project", args.wandb_project,
        "--wandb-group", args.wandb_group,
        "--wandb-run-name", args.wandb_run_name,
        "--wandb-run-id", args.wandb_run_id,
    ])
    if args.no_wandb:
        command.append("--no-wandb")
    if args.force:
        command.append("--force")
    return command


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True,
                        help="training output containing epoch_N.pth files")
    parser.add_argument("--epochs", default="1-40",
                        help="comma-separated inclusive ranges, e.g. 1-40 or 1,10,26")
    parser.add_argument("--split", choices=("val", "test"), default="val")
    parser.add_argument("--batch-size", type=int,
                        default=DEFAULT_EVAL_BATCH_SIZE,
                        help="evaluation batch size (default: 4)")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--info-root", default=DEFAULT_INFO_ROOT)
    parser.add_argument("--capture-root", default=DEFAULT_CAPTURE_ROOT)
    parser.add_argument("--output-subdir", default="corrected_range30_eval")
    parser.add_argument("--output-root", help=argparse.SUPPRESS)
    parser.add_argument("--image", default=DEFAULT_IMAGE)
    parser.add_argument("--shm-size", default="12g")
    parser.add_argument("--container-name", default="ajounice-bf-corrected-eval")
    parser.add_argument("--wandb-entity", default="AjouNice2026")
    parser.add_argument("--wandb-project", default="morai-3d-object-detection")
    parser.add_argument("--wandb-group", default="bevfusion-base3-corrected-range30")
    parser.add_argument(
        "--wandb-run-name", default="bf-base3-e40-range30-corrected-val")
    parser.add_argument(
        "--wandb-run-id", default="bf-base3-e40-range30-corrected-val",
        help="stable W&B ID used for safe resume")
    parser.add_argument("--no-wandb", action="store_true")
    parser.add_argument("--force", action="store_true",
                        help="ignore cached metrics and rerun inference")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--inside-container", action="store_true",
                        help=argparse.SUPPRESS)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.batch_size <= 0:
        fail("--batch-size must be positive")
    if args.inside_container:
        if not args.output_root:
            fail("internal worker requires --output-root")
        run_worker(args)
        return

    command = host_command(args)
    print("Corrected BEVFusion checkpoint sweep", flush=True)
    print("  epochs: {}".format(args.epochs), flush=True)
    print("  split:  {}".format(args.split), flush=True)
    print("  batch:  {}".format(args.batch_size), flush=True)
    print("  W&B:    {}".format(
        "disabled" if args.no_wandb else args.wandb_run_name), flush=True)
    print("  command: {}".format(shlex.join(command)), flush=True)
    if args.dry_run:
        return
    raise SystemExit(subprocess.run(command, check=False).returncode)


if __name__ == "__main__":
    main()
