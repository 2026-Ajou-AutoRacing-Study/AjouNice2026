#!/usr/bin/env python3
"""Resolve, snapshot, and execute one portable team BEVFusion experiment."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
from typing import Any, Dict, Iterable, List

import yaml


VALID_OPERATORS = {"sw", "yj", "jh"}
VALID_MODES = {"fresh", "finetune", "resume", "random"}
ROOT_KEYS = {
    "version", "name", "mode", "epochs", "learning_rate", "checkpoint",
    "base_config", "dataset", "gpu", "schedule", "wandb", "output",
    "checkpointing",
}
NESTED_KEYS = {
    "dataset": {
        "capture_root", "info_root", "container_root", "overfit_samples",
    },
    "gpu": {
        "profile", "physical_batch_size", "gradient_accumulation", "workers",
        "shm_size",
    },
    "wandb": {"enabled", "entity", "project", "group", "tags"},
    "output": {"root"},
    "checkpointing": {"interval", "max_keep_ckpts"},
    "schedule": {"warmup_optimizer_updates", "validation_interval"},
}


def fail(message: str) -> "NoReturn":
    raise SystemExit("[ERROR] " + message)


def run_text(args: Iterable[str], *, check: bool = True) -> str:
    result = subprocess.run(
        list(args), text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        check=False,
    )
    if check and result.returncode:
        fail("command failed: {}\n{}".format(shlex.join(list(args)), result.stdout.strip()))
    return result.stdout.strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def scalar_auto(value: Any, fallback: Any, converter, label: str):
    if value in (None, "auto"):
        return fallback
    try:
        converted = converter(value)
    except (TypeError, ValueError):
        fail("{} must be a number or auto; got {!r}".format(label, value))
    return converted


def ensure_keys(data: Dict[str, Any]) -> None:
    unknown = set(data) - ROOT_KEYS
    if unknown:
        fail("unknown experiment keys: {}".format(", ".join(sorted(unknown))))
    for key, allowed in NESTED_KEYS.items():
        value = data.get(key, {}) or {}
        if not isinstance(value, dict):
            fail("{} must be a mapping".format(key))
        extra = set(value) - allowed
        if extra:
            fail("unknown {} keys: {}".format(key, ", ".join(sorted(extra))))


def resolve_path(value: str, repo_root: Path) -> Path:
    path = Path(os.path.expandvars(os.path.expanduser(value)))
    return path if path.is_absolute() else repo_root / path


def detect_gpu_name() -> str:
    output = run_text([
        "nvidia-smi", "--query-gpu=name", "--format=csv,noheader",
    ])
    names = [line.strip() for line in output.splitlines() if line.strip()]
    if not names:
        fail("nvidia-smi returned no GPU")
    return names[0]


def select_gpu_profile(repo_root: Path, requested: str, gpu_name: str) -> Dict[str, Any]:
    profile_dir = repo_root / "perception/detection3d/bevfusion/configs/team/gpu"
    profiles = []
    for path in sorted(profile_dir.glob("*.yaml")):
        profile = yaml.safe_load(path.read_text())
        profile["_path"] = str(path)
        profiles.append(profile)
    if requested != "auto":
        for profile in profiles:
            if profile.get("id") == requested:
                return profile
        fail("unknown GPU profile: {}".format(requested))
    for profile in profiles:
        if str(profile.get("match", "")).lower() in gpu_name.lower():
            return profile
    fail("no team GPU profile matches {!r}; select one explicitly".format(gpu_name))


def copy_source_snapshot(repo_root: Path, destination: Path) -> None:
    detection3d_root = destination / "perception/detection3d"
    detection3d_root.mkdir(parents=True, exist_ok=True)
    ignore = shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache")
    for component in ("bevfusion", "common"):
        source = repo_root / "perception/detection3d" / component
        target = detection3d_root / component
        shutil.copytree(source, target, ignore=ignore)
    target = detection3d_root / "bevfusion"
    experiment_root = target / "configs/team/experiments"
    for operator in VALID_OPERATORS:
        shutil.rmtree(experiment_root / operator, ignore_errors=True)
    third_party = destination / "third_party"
    third_party.mkdir(parents=True, exist_ok=True)
    (third_party / "mmdetection3d").symlink_to("/opt/mmdetection3d")


def atomic_json(path: Path, value: Dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", help="local experiment name or YAML path")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="1 epoch, 16 samples per split")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[4]
    operator = os.environ.get("BEVFUSION_OPERATOR", "").strip()
    if operator not in VALID_OPERATORS:
        fail("run configure_team_machine.sh first; BEVFUSION_OPERATOR must be sw, yj, or jh")

    experiment_arg = Path(args.experiment)
    if experiment_arg.suffix in {".yaml", ".yml"} or "/" in args.experiment:
        experiment_path = resolve_path(args.experiment, Path.cwd())
    else:
        experiment_path = (
            repo_root / "perception/detection3d/bevfusion/configs/team/experiments"
            / operator / (args.experiment + ".yaml")
        )
    if not experiment_path.is_file():
        fail("experiment YAML not found: {}".format(experiment_path))
    raw = yaml.safe_load(experiment_path.read_text())
    if not isinstance(raw, dict):
        fail("experiment YAML root must be a mapping")
    ensure_keys(raw)
    if raw.get("version") != 1:
        fail("experiment version must be 1")

    name = str(raw.get("name", "")).strip()
    if not re.fullmatch(r"[a-z0-9][a-z0-9._-]*", name):
        fail("name must match [a-z0-9][a-z0-9._-]*")
    mode = str(raw.get("mode", "")).strip()
    if mode not in VALID_MODES:
        fail("mode must be one of {}".format(sorted(VALID_MODES)))
    epochs = int(raw.get("epochs", 12 if mode == "fresh" else 6))
    lr = float(raw.get("learning_rate", 1e-4 if mode in {"fresh", "random"} else 2e-5))
    if epochs <= 0 or lr <= 0:
        fail("epochs and learning_rate must be positive")

    schedule_cfg = raw.get("schedule", {}) or {}
    warmup_optimizer_updates = int(
        schedule_cfg.get("warmup_optimizer_updates", 500)
    )
    validation_interval = int(schedule_cfg.get("validation_interval", 1))
    if warmup_optimizer_updates < 0:
        fail("schedule.warmup_optimizer_updates must be nonnegative")
    if validation_interval <= 0:
        fail("schedule.validation_interval must be positive")

    checkpointing_cfg = raw.get("checkpointing", {}) or {}
    checkpoint_interval = int(checkpointing_cfg.get("interval", 1))
    max_keep_ckpts = int(checkpointing_cfg.get("max_keep_ckpts", 3))
    if checkpoint_interval <= 0 or max_keep_ckpts <= 0:
        fail("checkpointing.interval and checkpointing.max_keep_ckpts must be positive")

    base_config = resolve_path(
        str(raw.get("base_config", "perception/detection3d/bevfusion/configs/bevfusion_base3_b0_4cam_lidar.py")),
        repo_root,
    )
    if not base_config.is_file() or repo_root not in base_config.parents:
        fail("base_config must be an existing file inside the repository")

    dataset_cfg = raw.get("dataset", {}) or {}
    overfit_samples = int(dataset_cfg.get("overfit_samples", 0))
    if overfit_samples < 0:
        fail("dataset.overfit_samples must be nonnegative")
    info_value = str(dataset_cfg.get("info_root") or os.environ.get("BASE3_INFO_ROOT", ""))
    capture_value = str(
        dataset_cfg.get("capture_root") or os.environ.get("BASE3_CAPTURE_ROOT", "")
    )
    container_root = str(dataset_cfg.get("container_root") or "/datasets/base3")
    if not info_value or not capture_value:
        fail(
            "dataset.info_root/capture_root or BASE3_INFO_ROOT/"
            "BASE3_CAPTURE_ROOT must be configured"
        )
    if not container_root.startswith("/datasets/") or container_root in {"/datasets", "/datasets/"}:
        fail("dataset.container_root must be an absolute child of /datasets")
    info_root = resolve_path(info_value, repo_root).resolve()
    capture_root = resolve_path(capture_value, repo_root).resolve()
    for split in ("train", "val", "test"):
        info = info_root / "morai_base3_infos_{}.pkl".format(split)
        if not info.is_file() or not info.stat().st_size:
            fail("missing dataset info: {}".format(info))
    if not capture_root.is_dir():
        fail("Base3 capture root is missing: {}".format(capture_root))

    gpu_name = detect_gpu_name()
    gpu_cfg = raw.get("gpu", {}) or {}
    requested_profile = str(gpu_cfg.get("profile", "auto"))
    profile = select_gpu_profile(repo_root, requested_profile, gpu_name)
    batch = scalar_auto(gpu_cfg.get("physical_batch_size"), profile["physical_batch_size"], int, "batch size")
    accumulation = scalar_auto(gpu_cfg.get("gradient_accumulation"), profile["gradient_accumulation"], int, "gradient accumulation")
    workers = scalar_auto(gpu_cfg.get("workers"), profile["workers"], int, "workers")
    shm_size = str(profile["shm_size"] if gpu_cfg.get("shm_size") in (None, "auto") else gpu_cfg["shm_size"])
    if batch <= 0 or accumulation <= 0 or workers < 0:
        fail("batch and accumulation must be positive; workers must be nonnegative")
    effective_batch = batch * accumulation
    if effective_batch != 4:
        fail("team baseline requires physical_batch_size * gradient_accumulation == 4; got {}".format(effective_batch))

    checkpoint_value = str(raw.get("checkpoint") or "")
    if not checkpoint_value:
        env_key = {
            "fresh": "BEVFUSION_NUSCENES_ADAPTED_CHECKPOINT",
            "finetune": "BEVFUSION_BASE3_E10_FULL_CHECKPOINT",
            "resume": "BEVFUSION_BASE3_E10_FULL_CHECKPOINT",
            "random": "",
        }[mode]
        checkpoint_value = os.environ.get(env_key, "") if env_key else ""
    checkpoint = resolve_path(checkpoint_value, repo_root).resolve() if checkpoint_value else None
    if mode != "random" and (checkpoint is None or not checkpoint.is_file()):
        fail("{} mode checkpoint is missing: {}".format(mode, checkpoint_value or "<unset>"))

    image = os.environ.get("BEVFUSION_IMAGE", "ajounice/bevfusion:team-cu128-v1")
    output_cfg = raw.get("output", {}) or {}
    output_value = str(output_cfg.get("root") or os.environ.get("BEVFUSION_OUTPUT_ROOT", ""))
    output_root = resolve_path(output_value, repo_root).resolve() if output_value else (
        repo_root / "artifacts/detection3d/bevfusion/team_training"
    )
    timestamp = dt.datetime.now().strftime("%y%m%d-%H%M%S")
    gpu_slug = profile["id"]
    run_name = "bf-{}-{}-{}-{}".format(operator, name, gpu_slug, timestamp)
    if args.smoke:
        run_name += "-smoke"
    work_dir = output_root / operator / run_name
    if work_dir.exists():
        fail("refusing to overwrite output: {}".format(work_dir))
    work_dir.mkdir(parents=True)
    for local_dir in (".container_home", ".cache", ".matplotlib"):
        (work_dir / local_dir).mkdir()
    source_snapshot = work_dir / "source_snapshot" / "AjouNice2026"
    copy_source_snapshot(repo_root, source_snapshot)

    wandb_cfg = raw.get("wandb", {}) or {}
    wandb_enabled = bool(wandb_cfg.get("enabled", True))
    wandb_entity = str(wandb_cfg.get("entity") or os.environ.get("WANDB_ENTITY", "AjouNice2026"))
    wandb_project = str(wandb_cfg.get("project") or os.environ.get("WANDB_PROJECT", "morai-3d-object-detection"))
    wandb_group = str(wandb_cfg.get("group") or "bevfusion-base3-team")
    tags = [str(tag) for tag in wandb_cfg.get("tags", [])]
    tags.extend([operator, profile["id"], mode])
    tags = list(dict.fromkeys(tags))

    # MMEngine advances an iteration scheduler every dataloader iteration, even
    # when gradient accumulation delays optimizer.step().  Keep the public
    # recipe value in optimizer updates so the same experiment remains
    # portable across physical batch/accumulation GPU profiles.
    warmup_iterations = max(1, warmup_optimizer_updates * accumulation)
    persistent = "true" if workers > 0 else "false"
    cfg_options = [
        "train_dataloader.batch_size={}".format(batch),
        "train_dataloader.num_workers={}".format(workers),
        "train_dataloader.persistent_workers={}".format(persistent),
        "val_dataloader.num_workers={}".format(max(0, min(2, workers))),
        "val_dataloader.persistent_workers={}".format(persistent),
        "test_dataloader.num_workers={}".format(max(0, min(2, workers))),
        "test_dataloader.persistent_workers={}".format(persistent),
        "optim_wrapper.accumulative_counts={}".format(accumulation),
        "optim_wrapper.optimizer.lr={}".format(lr),
        "param_scheduler.0.end={}".format(warmup_iterations),
        "train_cfg.max_epochs={}".format(epochs),
        "train_cfg.val_interval={}".format(validation_interval),
        "param_scheduler.1.end={}".format(epochs),
        "param_scheduler.1.T_max={}".format(epochs),
        "default_hooks.checkpoint.interval={}".format(checkpoint_interval),
        "default_hooks.checkpoint.max_keep_ckpts={}".format(max_keep_ckpts),
        "train_dataloader.dataset.ann_file=/datasets/infos/morai_base3_infos_train.pkl",
        "val_dataloader.dataset.ann_file=/datasets/infos/morai_base3_infos_val.pkl",
        "test_dataloader.dataset.ann_file=/datasets/infos/morai_base3_infos_test.pkl",
        "visualizer.vis_backends.1.init_kwargs.name={}".format(run_name),
        "visualizer.vis_backends.1.init_kwargs.entity={}".format(wandb_entity),
        "visualizer.vis_backends.1.init_kwargs.project={}".format(wandb_project),
        "visualizer.vis_backends.1.init_kwargs.group={}".format(wandb_group),
        "visualizer.vis_backends.1.init_kwargs.job_type={}".format("smoke-test" if args.smoke else "train"),
        "visualizer.vis_backends.1.init_kwargs.tags={}".format(json.dumps(tags, separators=(",", ":"))),
    ]
    if mode in {"fresh", "finetune"}:
        cfg_options.append("load_from=/checkpoints/init.pth")
    elif mode == "random":
        cfg_options.append("load_from=None")
    if overfit_samples:
        # Deliberately evaluate the exact same examples used for training.  This
        # is a learnability diagnostic, never a generalization metric.
        cfg_options.extend([
            "train_dataloader.dataset.indices={}".format(overfit_samples),
            "val_dataloader.dataset.ann_file=/datasets/infos/morai_base3_infos_train.pkl",
            "val_dataloader.dataset.indices={}".format(overfit_samples),
            "test_dataloader.dataset.ann_file=/datasets/infos/morai_base3_infos_train.pkl",
            "test_dataloader.dataset.indices={}".format(overfit_samples),
        ])
    if args.smoke:
        cfg_options.extend([
            "train_cfg.max_epochs=1",
            "param_scheduler.1.end=1",
            "param_scheduler.1.T_max=1",
            "train_dataloader.dataset.indices=16",
            "val_dataloader.dataset.indices=16",
            "test_dataloader.dataset.indices=16",
            "default_hooks.checkpoint.max_keep_ckpts=1",
        ])

    uid, gid = os.getuid(), os.getgid()
    container_config = "/workspace/AjouNice2026/{}".format(base_config.relative_to(repo_root))
    docker_command: List[str] = [
        "docker", "run", "--rm", "--gpus", "all", "--shm-size={}".format(shm_size),
        "--name", "ajounice-{}".format(run_name), "--user", "{}:{}".format(uid, gid),
        "-v", "{}:/workspace/AjouNice2026:ro".format(source_snapshot),
        "-v", "{}:{}:ro".format(capture_root, container_root),
        "-v", "{}:/datasets/infos:ro".format(info_root),
        "-v", "{}:/outputs".format(work_dir),
        "-w", "/workspace/AjouNice2026/perception/detection3d/bevfusion",
        "-e", "HOME=/outputs/.container_home",
        "-e", "XDG_CACHE_HOME=/outputs/.cache",
        "-e", "MPLCONFIGDIR=/outputs/.matplotlib",
        "-e", "PYTHONPATH=/workspace/AjouNice2026:/opt/mmdetection3d",
        "-e", "TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1",
        "-e", "WANDB_MODE={}".format("online" if wandb_enabled else "disabled"),
        "-e", "WANDB_DISABLE_CODE=true",
        "-e", "WANDB_RUN_ID={}".format(run_name),
    ]
    if checkpoint is not None:
        docker_command.extend(["-v", "{}:/checkpoints/init.pth:ro".format(checkpoint)])
    netrc = Path.home() / ".netrc"
    if wandb_enabled and netrc.is_file():
        docker_command.extend(["-v", "{}:/outputs/.container_home/.netrc:ro".format(netrc)])
    docker_command.extend([
        image, "python", "/opt/mmdetection3d/tools/train.py", container_config,
        "--work-dir", "/outputs",
    ])
    if mode == "resume":
        docker_command.extend(["--resume", "/checkpoints/init.pth"])
    docker_command.extend(["--cfg-options"] + cfg_options)

    git_sha = run_text(["git", "-C", str(repo_root), "rev-parse", "HEAD"])
    git_branch = run_text(["git", "-C", str(repo_root), "branch", "--show-current"], check=False)
    git_status = run_text(["git", "-C", str(repo_root), "status", "--short"], check=False)
    (work_dir / "experiment_input.yaml").write_text(experiment_path.read_text())
    shutil.copy2(base_config, work_dir / "mmengine_config_input.py")
    (work_dir / "command.txt").write_text(shlex.join(docker_command) + "\n")
    (work_dir / "git_status.txt").write_text(git_status + "\n")
    relevant_diff = run_text([
        "git", "-C", str(repo_root), "diff", "--", "docker/bevfusion",
        "perception/detection3d/bevfusion",
    ], check=False)
    (work_dir / "source_changes.patch").write_text(relevant_diff + "\n")

    resolved = {
        "version": 1, "run_name": run_name, "operator": operator,
        "mode": mode, "epochs": 1 if args.smoke else epochs,
        "learning_rate": lr, "image": image, "gpu_name": gpu_name,
        "gpu_profile": profile["id"], "physical_batch_size": batch,
        "gradient_accumulation": accumulation, "effective_batch_size": effective_batch,
        "warmup_optimizer_updates": warmup_optimizer_updates,
        "warmup_iterations": warmup_iterations,
        "validation_interval": validation_interval,
        "overfit_samples": overfit_samples,
        "workers": workers, "shm_size": shm_size, "capture_root": str(capture_root),
        "dataset_container_root": container_root,
        "info_root": str(info_root), "checkpoint": str(checkpoint) if checkpoint else None,
        "checkpointing": {
            "interval": 1 if args.smoke else checkpoint_interval,
            "max_keep_ckpts": 1 if args.smoke else max_keep_ckpts,
        },
        "base_config": str(base_config.relative_to(repo_root)), "wandb": {
            "enabled": wandb_enabled, "entity": wandb_entity, "project": wandb_project,
            "group": wandb_group, "tags": tags,
        }, "smoke": args.smoke,
    }
    (work_dir / "resolved_experiment.yaml").write_text(yaml.safe_dump(resolved, sort_keys=False))
    manifest = {
        **resolved, "status": "dry-run" if args.dry_run else "running",
        "started_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "git_sha": git_sha, "git_branch": git_branch,
        "checkpoint_sha256": sha256(checkpoint) if checkpoint else None,
        "dataset_info_sha256": {
            split: sha256(info_root / "morai_base3_infos_{}.pkl".format(split))
            for split in ("train", "val", "test")
        },
    }
    atomic_json(work_dir / "run_manifest.json", manifest)

    print("Team BEVFusion experiment")
    print("  run:        {}".format(run_name))
    print("  GPU:        {} ({})".format(gpu_name, profile["id"]))
    print("  mode:       {}{}".format(mode, " / smoke" if args.smoke else ""))
    print("  batch:      {} x accumulation {} = effective {}".format(batch, accumulation, effective_batch))
    print("  epochs/lr:  {} / {}".format(1 if args.smoke else epochs, lr))
    print("  output:     {}".format(work_dir))
    print("  source:     immutable run snapshot")
    print("  command:    {}".format(shlex.join(docker_command)))
    if args.dry_run:
        return

    inspect = subprocess.run(
        ["docker", "image", "inspect", image], stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, check=False,
    )
    if inspect.returncode:
        fail("Docker image is missing: {}".format(image))
    log_path = work_dir / "train.log"
    with log_path.open("w", buffering=1) as log:
        process = subprocess.Popen(
            docker_command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        assert process.stdout is not None
        for line in process.stdout:
            sys.stdout.write(line)
            log.write(line)
        return_code = process.wait()
    manifest["finished_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    manifest["return_code"] = return_code
    manifest["status"] = "completed" if return_code == 0 else "failed"
    atomic_json(work_dir / "run_manifest.json", manifest)
    if return_code:
        raise SystemExit(return_code)


if __name__ == "__main__":
    main()
