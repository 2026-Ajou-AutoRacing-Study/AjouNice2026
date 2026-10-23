"""Fixed-cycle warm-up/cosine policy for bounded Base5 scheduler comparisons.

Reference: katsura-jp/pytorch-cosine-annealing-with-warmup (MIT),
revision 12d03c07553aedd3d9e9155e2b3e31ce8c64081a. This is not a vendored copy:
MMEngine owns stepping/state; parameter-group ratios are preserved.
"""
import math


def restart_factor(step, cycle_steps, warmup_steps, start_factor=.1, min_factor=.01):
    if cycle_steps < 2 or not 0 < warmup_steps < cycle_steps:
        raise ValueError('Require 0 < warmup_steps < cycle_steps')
    if not 0 < min_factor <= start_factor <= 1:
        raise ValueError('Require 0 < min_factor <= start_factor <= 1')
    if step < 0:
        raise ValueError('Step must be nonnegative')
    phase = step % cycle_steps
    if phase < warmup_steps:
        return start_factor + (1 - start_factor) * phase / warmup_steps
    progress = (phase - warmup_steps) / (cycle_steps - warmup_steps)
    return min_factor + (1 - min_factor) * (1 + math.cos(math.pi * progress)) / 2
