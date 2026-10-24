# Historical Network catalog

The first batch had no active reader. The second pass retired all non-production
choices instead of retaining root-level compatibility files. Current selection is
in [../README.md](../README.md); optional consumers explicitly select archived inputs.

| Archived file | Reason |
|---|---|
| `ego_network_1_lan_9088_9099.json` | Old unreferenced port variant, not the canonical LAN adapter |
| `ego_network_mr_calibration_hybrid.json` | Old calibration/SaveSensorData variant; no current reader |
| `ego_network_mr_base5_savesensordata_local.json` | Old SaveSensorData variant without the required Collision contract; the collision-enabled frozen input remains in the parent |
| `ego_network_mrp_minimal_hybrid.json` | Unreferenced MRP-named copy; the actually referenced MR minimal hybrid remains in the parent |
| `261024_README.before.md` | Original router before dependency audit |
| `ego_network_1.json` | Duplicate of canonical local; active callers were migrated |
| `ego_network_local.json`, `ego_network_1_lan.json` | Historical ScenarioLoad sources used explicitly by the validator generator |
| `ego_network_mr_base5_savesensordata_collision_local.json`, `ego_network_mr_minimal_hybrid.json` | Historical capture contracts. Existing frozen bundles remain intact; source reproduction uses the original tree |
| `261024_README.before_canonical_only.md` | Router before canonical-only consolidation |

Original JSON values and bytes are preserved. A similarly named ROS/hybrid profile
is not a UDP driving profile. Do not replace the canonical or pinned collection
configuration merely because a file is available here.
