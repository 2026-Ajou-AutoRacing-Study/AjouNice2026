# AW149 ROS1 planning workspace

149 boundary packages + 5 support packages, imported from `aw149-ros1-source-r003`, revision `r003-source-002`. The original provenance is retained verbatim; its historical scope labels do not certify this relocated workspace. Current evidence belongs in `logs/` and the integration handoff.

```bash
planning_ws/tools/build.sh --providers
planning_ws/tools/build_addons.sh
/usr/bin/python3 planning_ws/tools/audit_source.py
```

Build order: ROS Noetic → planning_ws → base_ws. Dependencies OSQP/proxsuite/acados are rebuilt under `planning_ws/external`; source archives/locks and pinned renderer are included. System dependencies (Noetic, compiler, Eigen, PCL, Lanelet2 system dependencies, Python build tooling) must also exist on a new machine. Provider preparation currently needs network access for locked Python requirements; this is not an offline image.

Use a clean shell, then `source planning_ws/tools/env.sh`. Do not mix with old `base_ws/devel`, old Autoware2ros1 installs, or a ROS2 environment. Build outputs are prefix-specific and must be rebuilt after moving the repository, not copied.

`src` is the frozen ported stack; AjouNice-specific glue lives in `base_ws/src/planning_addons`. Of 4,001 checkpoint entries, 3,997 are source files checked byte-for-byte; four Python cache entries are intentionally excluded. Source and provider licenses remain in their source trees. Nested vendor `.git` metadata is intentionally excluded; source-hash checks still apply.

See [developer handoff](../docs/planning/INTEGRATION-HANDOFF.md) for pending base stack changes and driving gates. A successful build does not mean all packages run concurrently without runtime errors. Do not enable a controller until those gates pass.

## Publication note

Vendor repositories retain their original `.gitignore` files. These hide 998 pinned provider files (including configured headers and JSON test fixtures) despite those files belonging to the frozen source manifest. Before a reviewed commit, the publisher must explicitly stage the exact `provenance/force-add-vendor-paths.txt` allowlist using `git add -f --pathspec-from-file=planning_ws/provenance/force-add-vendor-paths.txt` from the repository root, in addition to reviewing/staging the ordinary import. Do not use `git add -A`; unrelated user work exists. No staging or push is performed by build/audit tools.

Historical per-package oracle scripts under `src` may still mention old Autoware2ros1 test locations. They are retained as frozen provenance, not portable runtime/build entry points. Use this workspace's `tools` scripts; rebuilding the stack does not rerun all historical external harnesses.
