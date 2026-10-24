# Historical Network catalog

These original files had no active source/test/launch or inspected frozen
runtime/release/portable reader at the 2026-10-24 audit. Current role selection and
the intentionally retained validator/collection dependencies are in [../README.md](../README.md).

| Archived file | Reason |
|---|---|
| `ego_network_1_lan_9088_9099.json` | Old unreferenced port variant, not the canonical LAN adapter |
| `ego_network_mr_calibration_hybrid.json` | Old calibration/SaveSensorData variant; no current reader |
| `ego_network_mr_base5_savesensordata_local.json` | Old SaveSensorData variant without the required Collision contract; the collision-enabled frozen input remains in the parent |
| `ego_network_mrp_minimal_hybrid.json` | Unreferenced MRP-named copy; the actually referenced MR minimal hybrid remains in the parent |
| `261024_README.before.md` | Original router before dependency audit |

Original JSON values and bytes are preserved. A similarly named ROS/hybrid profile
is not a UDP driving profile. Do not replace the canonical or pinned collection
configuration merely because a file is available here.
