> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Modules

Planning package and runtime-unit navigation.

## Files

- `inventory.csv`
  - Retained release-workspace planning packages.
  - Start here when you know the package/module name.

- `runtime_surfaces.csv`
  - ROS 2 components, pluginlib plugins, and executable entry points detected in planning packages.
  - Use this to split a package into actual implementation units.

- `specs/`
  - One source-derived package summary per retained planning package.
  - Includes package identity, runtime surfaces, detected inputs/outputs/services/parameters, config defaults, and evidence paths.

## Version Policy

These module files are release-workspace source-derived evidence. Main-head duplicate specs were removed to reduce ambiguity.

Duplicate Markdown table renders of `inventory.csv` and `runtime_surfaces.csv` are intentionally not kept.
