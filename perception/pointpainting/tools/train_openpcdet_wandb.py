#!/usr/bin/env python3
"""Run an upstream OpenPCDet training job with W&B TensorBoard sync.

The wrapper deliberately leaves ``~/OpenPCDet/tools/train.py`` untouched.  It
removes only the W&B-specific arguments, initializes one run, patches
TensorBoardX before OpenPCDet creates its SummaryWriter, and then delegates to
the original training entry point.
"""

import argparse
import os
from pathlib import Path
import runpy
import shlex
import sys


def parse_wrapper_args(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--wandb-entity", default=os.getenv("WANDB_ENTITY", "AjouNice2026"))
    parser.add_argument("--wandb-project", default=os.getenv("WANDB_PROJECT", "pointpainting-morai"))
    parser.add_argument("--wandb-run-name", required=True)
    parser.add_argument("--wandb-group", default=None)
    parser.add_argument("--wandb-job-type", default="train")
    parser.add_argument("--wandb-tags", default="")
    parser.add_argument(
        "--wandb-mode",
        choices=("online", "offline", "disabled"),
        default=os.getenv("WANDB_MODE", "online"),
    )
    parser.add_argument(
        "--openpcdet-root",
        type=Path,
        default=Path(os.getenv("OPENPCDET_ROOT", "~/OpenPCDet")).expanduser(),
    )
    return parser.parse_known_args(argv)


def option_value(arguments, option, default=None):
    try:
        return arguments[arguments.index(option) + 1]
    except (ValueError, IndexError):
        return default


def openpcdet_output_dir(openpcdet_root, train_args):
    cfg_file = Path(option_value(train_args, "--cfg_file"))
    parts = cfg_file.parts
    group = Path(*parts[1:-1]) if len(parts) > 2 else Path()
    return (
        openpcdet_root
        / "output"
        / group
        / cfg_file.stem
        / option_value(train_args, "--extra_tag", "default")
    )


def upload_tensorboard_summaries(run, output_dir, configured_epochs):
    """Upload epoch summaries and validation scalars missed by the watcher."""
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

    train_events = sorted((output_dir / "tensorboard").glob("events.out.tfevents.*"))
    if train_events:
        accumulator = EventAccumulator(str(train_events[-1]))
        accumulator.Reload()
        losses = accumulator.Scalars("train/loss")
        learning_rates = accumulator.Scalars("meta_data/learning_rate")
        if losses:
            epoch_count = max(1, int(configured_epochs or 1))
            steps_per_epoch = max(1, (len(losses) + epoch_count - 1) // epoch_count)
            lr_by_step = {event.step: event.value for event in learning_rates}
            for epoch_index in range(epoch_count):
                chunk = losses[
                    epoch_index * steps_per_epoch : (epoch_index + 1) * steps_per_epoch
                ]
                if not chunk:
                    continue
                last_step = chunk[-1].step
                run.log(
                    {
                        "epoch": epoch_index + 1,
                        "epoch/train_loss_mean": sum(x.value for x in chunk) / len(chunk),
                        "epoch/learning_rate_last": lr_by_step.get(last_step),
                    }
                )

    validation_events = sorted(
        (output_dir / "eval/eval_with_train/tensorboard_val").glob(
            "events.out.tfevents.*"
        )
    )
    if validation_events:
        accumulator = EventAccumulator(str(validation_events[-1]))
        accumulator.Reload()
        values_by_epoch = {}
        for tag in accumulator.Tags().get("scalars", []):
            events = accumulator.Scalars(tag)
            for event in events:
                values_by_epoch.setdefault(int(event.step), {})[
                    f"val/{tag}"
                ] = event.value
        for epoch, values in sorted(values_by_epoch.items()):
            run.log({"epoch": epoch, **values})


def register_tensorboard_writers():
    """Track OpenPCDet writers so validation events can be flushed on demand."""
    import tensorboardX

    original_writer = tensorboardX.SummaryWriter
    writers = []

    class RegisteredSummaryWriter(original_writer):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            writers.append(self)

    tensorboardX.SummaryWriter = RegisteredSummaryWriter
    return writers


def main():
    wrapper, train_args = parse_wrapper_args(sys.argv[1:])
    openpcdet_root = wrapper.openpcdet_root.resolve()
    train_script = openpcdet_root / "tools/train.py"
    if not train_script.is_file():
        raise FileNotFoundError(f"OpenPCDet train.py not found: {train_script}")

    try:
        import wandb
    except ImportError as exc:
        raise RuntimeError(
            "wandb is not installed in the active environment; run "
            "`python -m pip install wandb` first"
        ) from exc

    repo_root = Path(__file__).resolve().parents[3]
    wandb_dir = repo_root / "artifacts/wandb"
    wandb_dir.mkdir(parents=True, exist_ok=True)
    tags = [tag.strip() for tag in wrapper.wandb_tags.split(",") if tag.strip()]
    config = {
        "runner": "OpenPCDet",
        "openpcdet_root": str(openpcdet_root),
        "command": shlex.join([str(train_script), *train_args]),
        "cfg_file": option_value(train_args, "--cfg_file"),
        "pretrained_model": option_value(train_args, "--pretrained_model"),
        "epochs": option_value(train_args, "--epochs"),
        "workers": option_value(train_args, "--workers"),
        "extra_tag": option_value(train_args, "--extra_tag"),
        "cuda_visible_devices": os.getenv("CUDA_VISIBLE_DEVICES", "ALL"),
    }

    # OpenPCDet uses tensorboardX.  Patch it before train.py imports and creates
    # SummaryWriter so train/loss, component losses and learning rate are sent
    # to W&B with their original global iteration.
    wandb.tensorboard.patch(save=True, tensorboard_x=True)
    tensorboard_writers = register_tensorboard_writers()
    run = wandb.init(
        entity=wrapper.wandb_entity,
        project=wrapper.wandb_project,
        name=wrapper.wandb_run_name,
        group=wrapper.wandb_group,
        job_type=wrapper.wandb_job_type,
        tags=tags,
        mode=wrapper.wandb_mode,
        dir=str(wandb_dir),
        config=config,
        save_code=False,
    )
    run.define_metric("epoch")
    run.define_metric("epoch/*", step_metric="epoch")
    run.define_metric("val/*", step_metric="epoch")

    tools_dir = str(openpcdet_root / "tools")
    sys.path.insert(0, tools_dir)
    sys.path.insert(0, str(openpcdet_root))
    sys.argv = [str(train_script), *train_args]
    exit_code = 0
    try:
        runpy.run_path(str(train_script), run_name="__main__")
        for writer in tensorboard_writers:
            writer.flush()
        upload_tensorboard_summaries(
            run,
            openpcdet_output_dir(openpcdet_root, train_args),
            option_value(train_args, "--epochs", "1"),
        )
    except BaseException:
        exit_code = 1
        raise
    finally:
        for writer in tensorboard_writers:
            writer.close()
        run.finish(exit_code=exit_code)


if __name__ == "__main__":
    main()
