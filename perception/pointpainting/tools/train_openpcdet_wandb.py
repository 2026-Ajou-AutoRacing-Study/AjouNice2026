#!/usr/bin/env python3
"""Run an upstream OpenPCDet training job with W&B TensorBoard sync.

The wrapper deliberately leaves ``~/OpenPCDet/tools/train.py`` untouched.  It
removes only the W&B-specific arguments, initializes one run, patches
TensorBoardX before OpenPCDet creates its SummaryWriter, and then delegates to
the original training entry point.
"""

import argparse
import glob
import os
from pathlib import Path
import runpy
import shlex
import shutil
import sys


def parse_wrapper_args(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--wandb-entity", default=os.getenv("WANDB_ENTITY", "AjouNice2026"))
    parser.add_argument(
        "--wandb-project",
        default=os.getenv("WANDB_PROJECT", "morai-3d-object-detection"),
    )
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
        "--validation-mode",
        choices=("interleaved", "post-training"),
        default="interleaved",
        help="Run validation after every selected epoch or use upstream post-training eval.",
    )
    parser.add_argument("--validation-interval", type=int, default=1)
    parser.add_argument("--validation-start-epoch", type=int, default=1)
    parser.add_argument("--selection-metric", default="center_mAP")
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


def upload_tensorboard_summaries(
        run, output_dir, configured_epochs, upload_validation=True):
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
    if upload_validation and validation_events:
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


def define_canonical_metrics(run):
    """Use one model-neutral W&B schema across Fast-BEV and PointPainting."""
    classes = ("vehicle", "pedestrian", "obstacle")
    center_thresholds = ("0.5", "1", "2", "4")
    distance_ranges = ("0_10", "10_20", "20_30", "30_50")
    iou_thresholds = ("0.25", "0.30", "0.50")

    run.define_metric("epoch", hidden=True)
    run.define_metric("epoch/*", step_metric="epoch")
    # Keep every validation scalar available for manually curated panels, but
    # avoid W&B creating dozens of automatic panels for low-level diagnostics.
    run.define_metric("val/*", step_metric="epoch", hidden=True)
    run.define_metric("eval/*", hidden=True)

    maximize = ["val/center_mAP", "val/center/mAP"]
    minimize = []
    for class_name in classes:
        maximize.append(f"val/center/{class_name}_mean_AP")
        for threshold in center_thresholds:
            maximize.append(f"val/center/{class_name}_AP_{threshold}m")
        for metric in ("F1", "precision", "recall"):
            maximize.append(f"val/operating/{class_name}_1m_{metric}")
        for distance_range in distance_ranges:
            maximize.append(
                f"val/range/{distance_range}m/{class_name}_mean_AP")
        for threshold in iou_thresholds:
            maximize.extend((
                f"val/iou/{class_name}_AP_{threshold}",
                f"val/iou/{class_name}_rec_{threshold}",
            ))
        for metric in ("mATE", "mASE", "mAOE"):
            minimize.append(f"val/tp/{class_name}_{metric}")
        for metric in (
                "mAVE", "speed_MAE", "longitudinal_MAE", "lateral_MAE"):
            minimize.append(f"val/velocity/{class_name}_{metric}")

    for threshold in iou_thresholds:
        maximize.extend((
            f"val/iou/mAP_{threshold}",
            f"val/iou/mAR_{threshold}",
        ))
    minimize.append("val/velocity/mAVE")

    for name in maximize:
        run.define_metric(
            name, step_metric="epoch", hidden=False,
            summary="max", overwrite=True)
    for name in minimize:
        run.define_metric(
            name, step_metric="epoch", hidden=False,
            summary="min", overwrite=True)


def install_interleaved_validation(openpcdet_root, train_args, wrapper, run):
    """Patch OpenPCDet's epoch loop with Train N -> Val N semantics."""
    import torch
    import tqdm
    from tensorboardX import SummaryWriter

    from pcdet.datasets import build_dataloader
    from train_utils import train_utils
    from eval_utils.eval_utils import eval_one_epoch

    output_dir = openpcdet_output_dir(openpcdet_root, train_args)
    eval_output_dir = output_dir / "eval" / "eval_with_train"
    val_tb = SummaryWriter(log_dir=str(eval_output_dir / "tensorboard_val"))
    validation_interval = max(1, int(wrapper.validation_interval))
    validation_start_epoch = max(1, int(wrapper.validation_start_epoch))
    selection_metric = str(wrapper.selection_metric)
    best_value = float("-inf")
    best_path = None

    def train_model_interleaved(
            model, optimizer, train_loader, model_func, lr_scheduler,
            optim_cfg, start_epoch, total_epochs, start_iter, rank, tb_log,
            ckpt_save_dir, train_sampler=None, lr_warmup_scheduler=None,
            ckpt_save_interval=1, max_ckpt_save_num=50,
            merge_all_iters_to_one_epoch=False, use_amp=False,
            use_logger_to_record=False, logger=None,
            logger_iter_interval=None, ckpt_save_time_interval=None,
            show_gpu_stat=False, cfg=None):
        nonlocal best_value, best_path
        accumulated_iter = start_iter
        hook_config = cfg.get("HOOK", None)
        augment_disable_flag = False
        dist_train = isinstance(
            model, torch.nn.parallel.DistributedDataParallel)
        eval_model = model.module if dist_train else model

        _, val_loader, _ = build_dataloader(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=cfg.CLASS_NAMES,
            batch_size=int(option_value(train_args, "--batch_size", "1")),
            dist=dist_train,
            workers=int(option_value(train_args, "--workers", "4")),
            logger=logger,
            training=False,
        )
        eval_args = argparse.Namespace(
            save_to_file="--save_to_file" in train_args,
            infer_time=False,
        )

        with tqdm.trange(
                start_epoch, total_epochs, desc="epochs", dynamic_ncols=True,
                leave=(rank == 0)) as tbar:
            total_it_each_epoch = len(train_loader)
            if merge_all_iters_to_one_epoch:
                train_loader.dataset.merge_all_iters_to_one_epoch(
                    merge=True, epochs=total_epochs)
                total_it_each_epoch = len(train_loader) // max(total_epochs, 1)

            dataloader_iter = iter(train_loader)
            for cur_epoch in tbar:
                if train_sampler is not None:
                    train_sampler.set_epoch(cur_epoch)
                if (lr_warmup_scheduler is not None and
                        cur_epoch < optim_cfg.WARMUP_EPOCH):
                    cur_scheduler = lr_warmup_scheduler
                else:
                    cur_scheduler = lr_scheduler

                augment_disable_flag = train_utils.disable_augmentation_hook(
                    hook_config, dataloader_iter, total_epochs, cur_epoch, cfg,
                    augment_disable_flag, logger)
                accumulated_iter = train_utils.train_one_epoch(
                    model, optimizer, train_loader, model_func,
                    lr_scheduler=cur_scheduler,
                    accumulated_iter=accumulated_iter,
                    optim_cfg=optim_cfg,
                    rank=rank,
                    tbar=tbar,
                    tb_log=tb_log,
                    leave_pbar=(cur_epoch + 1 == total_epochs),
                    total_it_each_epoch=total_it_each_epoch,
                    dataloader_iter=dataloader_iter,
                    cur_epoch=cur_epoch,
                    total_epochs=total_epochs,
                    use_logger_to_record=use_logger_to_record,
                    logger=logger,
                    logger_iter_interval=logger_iter_interval,
                    ckpt_save_dir=ckpt_save_dir,
                    ckpt_save_time_interval=ckpt_save_time_interval,
                    show_gpu_stat=show_gpu_stat,
                    use_amp=use_amp,
                )

                trained_epoch = cur_epoch + 1
                checkpoint_path = None
                if trained_epoch % ckpt_save_interval == 0 and rank == 0:
                    checkpoints = glob.glob(
                        str(ckpt_save_dir / "checkpoint_epoch_*.pth"))
                    checkpoints.sort(key=os.path.getmtime)
                    if len(checkpoints) >= max_ckpt_save_num:
                        delete_count = len(checkpoints) - max_ckpt_save_num + 1
                        for path in checkpoints[:delete_count]:
                            os.remove(path)
                    checkpoint_stem = ckpt_save_dir / (
                        f"checkpoint_epoch_{trained_epoch}")
                    train_utils.save_checkpoint(
                        train_utils.checkpoint_state(
                            model, optimizer, trained_epoch, accumulated_iter),
                        filename=checkpoint_stem,
                    )
                    checkpoint_path = Path(str(checkpoint_stem) + ".pth")

                should_validate = (
                    trained_epoch >= validation_start_epoch and
                    (trained_epoch - validation_start_epoch) %
                    validation_interval == 0
                )
                if not should_validate:
                    continue

                result_dir = (
                    eval_output_dir / f"epoch_{trained_epoch}" / "val")
                metrics = eval_one_epoch(
                    cfg, eval_args, eval_model, val_loader, trained_epoch,
                    logger, dist_test=dist_train, result_dir=result_dir)
                if rank != 0:
                    continue

                scalar_metrics = {
                    key: float(value) for key, value in metrics.items()
                    if isinstance(value, (int, float))
                }
                for key, value in scalar_metrics.items():
                    val_tb.add_scalar(key, value, trained_epoch)
                val_tb.flush()
                run.log({
                    "epoch": trained_epoch,
                    **{f"val/{key}": value
                       for key, value in scalar_metrics.items()},
                })

                selected = scalar_metrics.get(selection_metric)
                if (selected is not None and selected > best_value and
                        checkpoint_path is not None):
                    if best_path is not None and best_path.exists():
                        best_path.unlink()
                    best_value = selected
                    best_path = ckpt_save_dir / (
                        f"best_{selection_metric}_epoch_{trained_epoch}.pth")
                    shutil.copy2(checkpoint_path, best_path)
                    logger.info(
                        "New best %s=%.6f at epoch %d -> %s",
                        selection_metric, best_value, trained_epoch, best_path)

                model.train()

        val_tb.flush()

    train_utils.train_model = train_model_interleaved

    # Upstream train.py always performs a second checkpoint sweep after the
    # training loop. Interleaved mode has already evaluated every requested
    # epoch, so replace only that imported function with an explicit no-op.
    import test

    def skip_repeat_eval(*args, **kwargs):
        return None

    test.repeat_eval_ckpt = skip_repeat_eval


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
        "validation_mode": wrapper.validation_mode,
        "validation_interval": wrapper.validation_interval,
        "validation_start_epoch": wrapper.validation_start_epoch,
        "selection_metric": wrapper.selection_metric,
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
    define_canonical_metrics(run)

    tools_dir = str(openpcdet_root / "tools")
    sys.path.insert(0, tools_dir)
    sys.path.insert(0, str(openpcdet_root))
    if wrapper.validation_mode == "interleaved":
        install_interleaved_validation(
            openpcdet_root, train_args, wrapper, run)
    # OpenPCDet resolves _BASE_CONFIG_ relative to its tools directory rather
    # than relative to the parent YAML. Match the native `cd tools && train.py`
    # invocation so both relative and absolute --cfg_file values work.
    os.chdir(tools_dir)
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
            upload_validation=(wrapper.validation_mode != "interleaved"),
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
