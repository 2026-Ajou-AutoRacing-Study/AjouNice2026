#!/usr/bin/env python3
"""Overlay opt-in LC_GEOMETRY rosout records and the exact tested boundary.

Input is either a rosbag or replay text log. Outputs are generated artifacts.
"""
import argparse
import collections
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def records(path):
    if path.suffix == ".bag":
        import rosbag
        with rosbag.Bag(str(path)) as bag:
            start = bag.get_start_time()
            geometry_topic = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/lane_change_geometry"
            for topic, msg, stamp in bag.read_messages(topics=["/rosout", geometry_topic]):
                if topic == geometry_topic:
                    yield stamp.to_sec() - start, json.loads(msg.data)
                elif "LC_GEOMETRY " in msg.msg:
                    yield stamp.to_sec() - start, json.loads(msg.msg.split("LC_GEOMETRY ", 1)[1])
    else:
        for line in path.read_text().splitlines():
            if line.startswith("GEOMETRY "):
                yield float(line.split()[1]), json.loads(line.split("LC_GEOMETRY ", 1)[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--time", type=float, default=112.0)
    parser.add_argument("--map", type=Path, help="optional lanelet OSM (UTM projector)")
    parser.add_argument("--origin", nargs=2, type=float, metavar=("LAT", "LON"))
    args = parser.parse_args()
    rows = list(records(args.input))
    if not rows:
        raise SystemExit("No LC_GEOMETRY records: enable AUTOWARE_LC_GEOMETRY_AUDIT on BPP")
    args.output.mkdir(parents=True, exist_ok=True)
    selected_time = min(rows, key=lambda row: abs(row[0] - args.time))[0]
    selected = [d for t, d in rows if t == selected_time]
    fig, ax = plt.subplots(figsize=(9, 11))
    if args.map:
        if not args.origin:
            parser.error("--map requires --origin LAT LON")
        import ctypes
        import lanelet2
        ctypes.CDLL("libautoware_lanelet2_extension_lib.so", mode=ctypes.RTLD_GLOBAL)
        lane_map = lanelet2.io.load(str(args.map), lanelet2.projection.UtmProjector(
            lanelet2.io.Origin(*args.origin)))
        ids = set(i for d in selected for i in d["current_ids"] + d["target_ids"])
        for lane_id in sorted(ids):
            lane = lane_map.laneletLayer[lane_id]
            xy = [(p.x, p.y) for p in lane.polygon2d()]
            ax.fill(*zip(*xy), alpha=0.06, facecolor="gray", edgecolor="gray")
            p = lane.centerline[len(lane.centerline)//2]
            ax.text(p.x, p.y, str(lane_id), fontsize=7, color="gray", clip_on=True)
    for i, d in enumerate(selected):
        color = plt.cm.tab10(i % 10)
        for key, style, label in [("boundary", "k--", "tested boundary"),
                                   ("reference", "g:", "target reference")]:
            if key in d:
                xy = d[key]
                ax.plot(*zip(*xy), style, alpha=0.6, label=label if i == 0 else None)
        xy = d["poses"]
        if not xy:
            continue
        ax.plot(*zip(*xy), "o-", color=color, markersize=4,
                label=f'{i}: {d["reason"]}, N={len(xy)}, prep={d["prepare_length"]:.1f}m')
        ax.scatter(*d["prepare_end"], marker="x", color=color, s=90)
        ax.scatter(*xy[0], marker="s", facecolors="none", edgecolors=[color], s=90)
        ax.scatter(*xy[-1], marker="*", color=color, s=100)
        offset = (d["width"] / 2 + 0.1) * (1 if d["side"] == "left" else -1)
        edge = []
        # Reproduce sliding(2) | drop(1), including its sparse-polyline behavior.
        for p, q in list(zip(xy, xy[1:]))[1:]:
            dx, dy = p[0] - q[0], p[1] - q[1]
            length = math.hypot(dx, dy)
            if length:
                edge.append((q[0] - dy / length * offset, q[1] + dx / length * offset))
        edge = d.get("tested_edge", edge)
        if edge:
            ax.plot(*zip(*edge), ".:", color=color, alpha=0.8)
    all_xy = [p for d in selected for p in d["poses"] + [d["prepare_end"]]]
    ax.set_xlim(min(p[0] for p in all_xy)-8, max(p[0] for p in all_xy)+8)
    ax.set_ylim(min(p[1] for p in all_xy)-8, max(p[1] for p in all_xy)+8)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_xlabel("map x [m]"); ax.set_ylabel("map y [m]")
    ax.set_title(f"LC geometry t={selected_time:.3f}s\nx: prepare end / square: LC start / star: LC end")
    ax.legend(loc="best", fontsize=8)
    fig.savefig(args.output / "candidate_overlay.png", dpi=160, bbox_inches="tight")
    summary = {"source": str(args.input), "count": len(rows),
               "reasons": dict(collections.Counter(d["reason"] for _, d in rows)),
               "selected_time": selected_time, "selected": selected}
    (args.output / "geometry.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: v for k, v in summary.items() if k != "selected"}, indent=2))


if __name__ == "__main__":
    main()
