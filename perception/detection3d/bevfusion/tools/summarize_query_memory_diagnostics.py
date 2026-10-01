#!/usr/bin/env python3
"""Summarize Q1 query-memory diagnostics captured by the ROS bridge."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    values: dict[str, list[float]] = {}
    frames = 0
    with args.input.open(encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            diagnostics = record.get("diagnostics", {})
            if not diagnostics:
                continue
            frames += 1
            for name, value in diagnostics.items():
                values.setdefault(name, []).append(float(value))

    summary = {"frames": frames, "metrics": {}}
    for name, samples in sorted(values.items()):
        array = np.asarray(samples, dtype=np.float64)
        summary["metrics"][name] = {
            "mean": float(np.mean(array)),
            "p50": float(np.percentile(array, 50)),
            "p95": float(np.percentile(array, 95)),
            "max": float(np.max(array)),
        }
    text = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
