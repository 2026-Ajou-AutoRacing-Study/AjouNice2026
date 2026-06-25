> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Autoware Planning Reference

This is a compact, planning-only Autoware reference bundle for Codex-led development.

It keeps the information needed to understand and rebuild the Autoware planning stack: planning modules, runtime surfaces, topics, parameters, message/service/action definitions, and selected upstream planning docs used as local intent/algorithm evidence.

It is not a finalized ROS 1 porting plan.

## Start Here

1. Read `CODEX_REFERENCE_GUIDE.md` when asking Codex to work from this reference.
2. Use `modules/inventory.csv` to find the planning package or module.
3. Use `modules/runtime_surfaces.csv` to see actual ROS 2 components, plugins, and executables.
4. Use `modules/specs/` for package-level source-derived summaries.
5. Use `interfaces/contracts.csv` and `interfaces/messages.csv` for topics, parameters, services, and message fields.
6. Use `docs/official/` only when the package spec is not enough to understand upstream intent, algorithms, diagrams, or documented behavior.

## Folder Layout

```text
reference/autoware_planning_porting/
  modules/       planning package inventory, runtime surfaces, package specs
  interfaces/    topics, parameters, services, and planning message definitions
  docs/          selected upstream planning docs and extracted diagram assets
  meta/          source/version provenance and pruning summary
```

## What Was Kept

- Release-workspace planning packages: 93 packages
- Runtime surfaces: 129 components/plugins/executables
- Planning contracts: 5,233 topic/service/parameter/config rows
- Planning-used Autoware msg/srv/action definitions: 149 types
- Planning package specs: 93 files
- Upstream planning docs snapshot: 120 pages

## What Was Removed

To keep this usable and GitHub-friendly, broad raw/reference material was removed:

- Full upstream source clones
- Raw HTTrack HTML mirror
- Non-planning docs
- Main-head duplicate source-derived indices
- Human-readable duplicate renders of CSV data
- Broad all-repo Markdown corpus
- Broad package/message inventories unrelated to planning
- Acquisition input files already captured by provenance manifests
- Premature ROS 1 porting plan files

If a future task needs non-planning evidence, fetch that upstream source or doc explicitly for that task.

## Baseline Policy

- `modules/` and `interfaces/` are source-derived from the release workspace snapshot.
- `docs/official/` is a local upstream documentation snapshot for intent and algorithm context. It is secondary to source-derived contracts when implementing runtime behavior.
- `meta/source_repositories.md` records the repository URLs, versions, and commits used as provenance.
- Generated specs and CSVs are navigation aids. CSV files are the canonical local structured data for inventories, runtime surfaces, contracts, and message definitions. For final implementation decisions, verify behavior against upstream source if ambiguity remains.
