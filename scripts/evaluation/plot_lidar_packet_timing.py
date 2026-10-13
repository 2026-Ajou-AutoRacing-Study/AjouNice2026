#!/usr/bin/env python3
"""Plot recorded host stamps/source footer/groups, without guessing ray time."""
import argparse
from pathlib import Path

import numpy as np

from lidar_packet_reassembly import footer_nearest_hour, group_indices, read_source, rotations


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Fresh plot path required')
    source = read_source(args.bag)
    packets = source['packets']
    angles = np.asarray([rotations(p)[0] for p in packets])
    stamps = np.asarray([p.stamp.to_sec() for p in packets])
    footers = np.asarray([int.from_bytes(bytes(p.data)[1200:1204], 'little') for p in packets])
    footer_times = np.asarray([footer_nearest_hour(t, f) for t, f in zip(stamps, footers)])
    starts = np.r_[0, np.flatnonzero(np.diff(footers) != 0)+1]
    if len(starts) < 6:
        raise ValueError('Need at least six source batches')
    begin, end = starts[2], starts[6]
    x = np.arange(begin, end)
    first = starts[(starts >= begin) & (starts < end)]
    reference = footer_times[begin]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, (footer_times[x]-reference)*1000, label='native footer (source batch)', color='tab:orange')
    axes[0].scatter(x, (stamps[x]-reference)*1000, s=8, label='host read midpoint stamp')
    axes[0].scatter(first, (stamps[first]-reference)*1000, s=55, marker='x', color='red', label='first packet after wait')
    axes[0].set_ylabel('Relative time (ms)')
    axes[0].set_title('Recorded source bursts and host stamp bias; NOT exact ray acquisition time')
    axes[0].legend(loc='upper left', fontsize=8)
    axes[1].scatter(x, angles[x]/100., s=8)
    axes[1].set_ylabel('First block azimuth (deg)')
    colors = ('tab:blue', 'tab:green', 'tab:purple', 'tab:brown')
    for row, (policy, color) in enumerate(zip(('fixed51', 'fixed50', 'cut0', 'cut180'), colors)):
        for a, b in group_indices(angles, policy):
            if b <= begin or a >= end:
                continue
            axes[2].plot([max(a, begin), min(b-1, end-1)], [row, row], color=color, linewidth=8, alpha=.65)
            if begin <= a < end:
                axes[2].axvline(a, color=color, alpha=.2, linewidth=.7)
    axes[2].set_yticks(range(4)); axes[2].set_yticklabels(['51 packets', '50 packets', 'cut 0', 'cut 180'])
    axes[2].set_xlabel('Original packet index')
    for axis in axes:
        for index in first:
            axis.axvline(index, color='tab:orange', linestyle=':', alpha=.6)
        axis.grid(alpha=.2)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
