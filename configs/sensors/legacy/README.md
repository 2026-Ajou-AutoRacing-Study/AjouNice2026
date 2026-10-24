# Historical Sensor catalog

These are byte-preserved inputs, not current calibration or runtime defaults.
Use [the current role table](../README.md). The first batch had no active reader;
the second pass retired every non-production choice, including required old mirrors.

| Archived files | Reason |
|---|---|
| `Sensor_1.json`, `Sensor_2.json`, `Sensor_3.json` | Superseded sensor layouts; number alone is not a version-selection policy |
| `Sensor_4_noise_v1.json` | Old noise experiment; actual Sensor4 trial input remains in the parent |
| `Sensor_local.json`, `Sensor_lan.json` | Old unreferenced local/LAN variants; canonical competition profiles remain in the parent |
| `Sensor_competition_local_lidar150_test.json`, `Sensor_competition_local_lidar_freq1_probe.json` | Opt-in LiDAR period/frequency probes, not production |
| `Sensor_woodbox_lidar_only_local.json`, `2026_molit_comp_cam_set.json` | Standalone LiDAR/camera subsets, not the current complete sensor contract |
| `sensor_layout.yaml`, `sensor_layout_example.yaml` | Unused descriptive/example layouts, not the loaded MORAI calibration |
| `261024_README.before.md` | Original router before role-based classification |
| `Sensor.json`, `Sensor_6.json` | Byte-identical aliases of canonical local; active callers now use the canonical file |
| `Sensor_4.json`, `Sensor_5.json` | Historical 20Hz layouts; trial/replay callers explicitly select these archived inputs |
| `Sensor_localization_gt_lightweight.json` | Opt-in reduced-load experiment only, not the production sensor contract |
| `261024_README.before_canonical_only.md` | Router before removing the remaining compatibility choices |

JSON/YAML bytes, endpoint values, extrinsics and periods were not normalized.
Reproduce only with the original software, sensor profile and scenario provenance;
do not automatically restore these paths into a live stack or frozen release.
