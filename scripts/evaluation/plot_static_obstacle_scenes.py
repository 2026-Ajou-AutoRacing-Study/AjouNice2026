"""Draw three exact-stamp BPP scene / PO geometry examples from a completed replay."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch
from shapely.geometry import Polygon

parser = argparse.ArgumentParser()
parser.add_argument("run", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
fig, axes = plt.subplots(1, 3, figsize=(15, 7), constrained_layout=True)
snapshots = []
for ax, (name, elapsed) in zip(axes, [("204201", 4.0), ("204005", 5.5), ("200049", 7.5)]):
    replay = json.loads((args.run / f"planning_rviz_20260922_{name}" / "result.json").read_text())
    stamp = replay["bag_start"] + elapsed
    trajectory = min(replay["trajectories"], key=lambda x: abs(x["time"] - stamp))
    stamp = trajectory["time"]
    scene = next(s for s in replay["scenes"] if abs(s["time"] - stamp) < 1e-6)
    road = next(s for s in replay["records"] if abs(s["time"] - stamp) < 1e-6)
    for side in ["left", "right"]:
        ax.plot(*zip(*road[side]), color="#333333", lw=2,
                label="Road exterior" if side == "left" else None)
    for i, obstacle in enumerate(scene["obstacles"]):
        polygon = Polygon(obstacle["polygon"])
        for padding, color, label in [
            (obstacle["hard"] + obstacle["soft"], "#e8c354", "Soft preference"),
            (obstacle["hard"], "#e07854", "Hard clearance"),
            (0, "#444444", "SOA retained envelope"),
        ]:
            ax.add_patch(Patch(list(polygon.buffer(padding).exterior.coords), color=color,
                               alpha=.55, label=label if i == 0 else None))
    pts = trajectory["points"]
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#1768ac", lw=2,
            label="Selected PO path")
    ego = min(replay["planning_odometry"], key=lambda e: abs(e["time"] - stamp))
    ax.plot(ego["x"], ego["y"], "o", color="#8b3299", label="Recorded ego")
    ax.set(xlim=(63, 86), ylim=(387, 415), xlabel="Map X [m]", ylabel="Map Y [m]",
           title=f"{name[:2]}:{name[2:4]}:{name[4:]} / +{stamp-replay['bag_start']:.3f}s")
    ax.set_aspect("equal")
    ax.grid(alpha=.2)
    snapshots.append({"bag": replay["bag"], "stamp": stamp, "scene": scene,
                      "road": road, "trajectory": trajectory})
axes[0].legend(fontsize=8, loc="lower left")
fig.suptitle("Road exterior + independent obstacle holes / selected optimized path")
args.output.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(args.output.with_suffix(".svg"))
fig.savefig(args.output.with_suffix(".png"), dpi=140)
args.output.with_suffix(".json").write_text(json.dumps(snapshots, indent=2))
print(args.output.with_suffix(".svg"))
