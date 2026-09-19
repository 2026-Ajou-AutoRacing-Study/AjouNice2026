# Base3 static-obstacle OD benchmark

`bench_static_obstacle_base3_assets_raw.json` is a clean, human-driven MORAI
scenario for recording an online-sensor OD evaluation bag in the competition
static-obstacle corridor.

- Ego starts about 41 m before the first target in Keyboard/manual mode and P.
- Moving vehicles, pedestrians, and spawn points are removed.
- Six assets retained in the final Base3-Static training allowlist are placed
  every 18 m and alternate slightly left/right within the driven lane.
- Scale is fixed to the catalog-native 1.0 so this benchmark measures class and
  box generalization without the official sample's `CargoBox` scale-2 special
  case.
- The audit sidecar records exact DataID, route arc, offset, pose, and link.

The target order is `CargoBox`, `PE_Drum`, `TrafficBarrel`,
`concrete-barrier`, `Concrete_Barricade01`, and `PE_Firewall_Orange`.

Rebuild with:

```bash
cd ~/AjouNice2026
python3 dataset/morai_od/scripts/build_static_obstacle_od_benchmark.py
```
