#!/usr/bin/env python3
"""Plot OpenPCDet's fastai-style OneCycle LR before a training run."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--steps-per-epoch", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--observed-event-dir",
        type=Path,
        help="Optional TensorBoard train event directory to overlay.",
    )
    return parser.parse_args()


def annealing_cos(start, end, fraction):
    return end + (start - end) * (np.cos(np.pi * fraction) + 1.0) / 2.0


def planned_schedule(epochs, steps_per_epoch, lr_max, div_factor, pct_start):
    total_steps = epochs * steps_per_epoch
    phase_one_steps = int(total_steps * pct_start)
    steps = np.arange(total_steps, dtype=np.int64)
    learning_rates = np.empty(total_steps, dtype=np.float64)
    low_lr = lr_max / div_factor
    final_lr = low_lr / 1.0e4

    first = steps < phase_one_steps
    learning_rates[first] = annealing_cos(
        low_lr,
        lr_max,
        steps[first] / max(phase_one_steps, 1),
    )
    learning_rates[~first] = annealing_cos(
        lr_max,
        final_lr,
        (steps[~first] - phase_one_steps) / max(total_steps - phase_one_steps, 1),
    )
    epoch_position = steps / steps_per_epoch
    return epoch_position, learning_rates, phase_one_steps / steps_per_epoch


def observed_schedule(event_dir):
    if event_dir is None:
        return None
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

    event_files = sorted(event_dir.glob("events.out.tfevents.*"))
    if not event_files:
        raise FileNotFoundError(f"No TensorBoard events in {event_dir}")
    accumulator = EventAccumulator(str(event_files[-1]))
    accumulator.Reload()
    events = accumulator.Scalars("meta_data/learning_rate")
    return (
        np.asarray([event.step for event in events], dtype=np.int64),
        np.asarray([event.value for event in events], dtype=np.float64),
    )


def main():
    args = parse_args()
    config = yaml.safe_load(args.config.read_text())
    optimization = config["OPTIMIZATION"]
    epochs = int(optimization["NUM_EPOCHS"])
    lr_max = float(optimization["LR"])
    div_factor = float(optimization["DIV_FACTOR"])
    pct_start = float(optimization["PCT_START"])

    epoch_x, planned_lr, peak_epoch = planned_schedule(
        epochs, args.steps_per_epoch, lr_max, div_factor, pct_start)
    observed = observed_schedule(args.observed_event_dir)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for axis in axes:
        axis.plot(epoch_x, planned_lr, color="#2563eb", linewidth=2.2,
                  label="Planned OneCycle LR")
        axis.axvline(peak_epoch, color="#dc2626", linestyle="--", linewidth=1.4,
                     label=f"Peak LR at epoch {peak_epoch:.1f}")
        axis.grid(True, alpha=0.25)
        axis.set_ylabel("Learning rate")

        if observed is not None:
            observed_steps, observed_lr = observed
            observed_epochs = observed_steps / args.steps_per_epoch
            axis.plot(observed_epochs, observed_lr, color="#16a34a", linewidth=1.1,
                      alpha=0.9, label="Observed training LR")

    axes[0].set_title(
        f"OpenPCDet Adam OneCycle: {epochs} epochs, "
        f"{args.steps_per_epoch} steps/epoch, max LR={lr_max:g}"
    )
    axes[0].legend(loc="upper right")
    axes[1].set_yscale("log")
    axes[1].set_xlabel("Epoch progress")
    axes[1].legend(loc="upper right")
    axes[1].set_xlim(0, epochs)

    low_lr = lr_max / div_factor
    final_lr = low_lr / 1.0e4
    summary = (
        f"start={low_lr:.2e}   peak={lr_max:.2e}   "
        f"final≈{final_lr:.2e}   pct_start={pct_start:.2f}"
    )
    fig.text(0.5, 0.01, summary, ha="center", fontsize=10)
    fig.tight_layout(rect=(0, 0.035, 1, 1))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight")
    plt.close(fig)

    print(f"output: {args.output}")
    print(f"start_lr: {low_lr:.10g}")
    print(f"peak_lr: {lr_max:.10g} at epoch {peak_epoch:.3f}")
    print(f"final_lr: {final_lr:.10g}")


if __name__ == "__main__":
    main()
