# MORAI raw-sensor benchmark scenarios

These scenarios are deterministic derivatives of
`configs/scenario/2026_molit_comp_sample_scene.json` for reusable rosbag
recording. Regenerate them with:

```bash
python3 dataset/morai_od/scripts/build_raw_benchmark_scenarios.py
```

- `bench_roundabout_raw.json`: Ego is in Keyboard mode and parked at the
  roundabout approach. Only official roundabout spawn point `4` is retained.
- `bench_highway_merge_raw.json`: Ego is in Keyboard mode and parked at the
  highway merge approach. Only official highway spawn points `6`, `8`, and
  `10` are retained.

Both scenarios remove saved NPC instances, pedestrians, obstacles, pedestrian
spawners, and actor waypoint data. Traffic-light state is retained because it
is map infrastructure rather than a perception actor.
