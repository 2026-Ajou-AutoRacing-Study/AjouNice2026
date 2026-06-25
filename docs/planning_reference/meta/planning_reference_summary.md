> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning Reference Summary

This corpus was pruned to keep planning-stack evidence only.

## Kept

- Planning-related rendered Autoware documentation Markdown:
  - `autoware_universe`: 84 pages
  - `autoware_documentation`: 21 pages
  - `autoware_core`: 15 pages
- Official planning docs: `docs/official/`
- Planning package inventory: `modules/inventory.csv`
- Planning runtime surfaces: `modules/runtime_surfaces.csv`
- Planning package specs: `modules/specs/`
- Planning topic/service/parameter contracts: `interfaces/contracts.csv`
- Planning-used Autoware message/service/action definitions: `interfaces/messages.csv`
- Planning interface coverage notes: `interfaces/coverage.md`

## Removed

- Broad source Markdown corpus: `corpus/all_markdown/`
- Broad source Markdown folder: `docs/source/`
- Non-planning rendered documentation pages
- Non-planning rendered documentation extracted assets
- Main-head duplicate source-derived module specs and contracts
- Duplicate Markdown renders of CSV data:
  - `interfaces/contracts.md`
  - `interfaces/messages.md`
  - `modules/inventory.md`
  - `modules/runtime_surfaces.md`
- Acquisition-only source input files already represented by provenance manifests
- Broad package inventory
- Broad message type crosswalk
- Full-corpus validation/review reports that no longer describe the pruned planning-only bundle

## Interface Policy

Planning topics, services, parameters, runtime components, plugins, executables, and Autoware message definitions are retained because they are needed to rebuild or compare the planning stack.

Standard ROS message types and helper/template types are listed in `interfaces/coverage.md` when they are mentioned but not expanded locally.
