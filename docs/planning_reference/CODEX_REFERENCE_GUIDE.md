> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Codex Reference Guide

Use this file as the first instruction when Codex works on Autoware planning code.

## Rule

Treat `reference/autoware_planning_porting/` as a planning-only Autoware evidence bundle.

It is not a ROS 1 porting plan. It is the reference material to inspect before deciding how to port or rebuild a module.

## Read Order

1. `README.md`
2. `modules/inventory.csv`
3. `modules/runtime_surfaces.csv`
4. The relevant package spec under `modules/specs/`
5. `interfaces/contracts.csv`
6. `interfaces/messages.csv`
7. `interfaces/coverage.md`
8. Matching pages under `docs/official/` when algorithm intent, design rationale, diagrams, or documented behavior are needed
9. `meta/source_repositories.md` if exact upstream version evidence is needed

## Prompt To Use

```text
Before changing code, read reference/autoware_planning_porting/CODEX_REFERENCE_GUIDE.md.

Use reference/autoware_planning_porting as a planning-only Autoware evidence bundle.
Start from modules/inventory.csv, modules/runtime_surfaces.csv, and modules/specs/ to identify planning packages, runtime components, plugins, and package specs.
Use interfaces/contracts.csv and interfaces/messages.csv for planning topics, services, parameters, and message fields.
Use docs/official/ as secondary upstream intent/algorithm evidence when the source-derived package spec is not enough.
Do not treat this as a finalized ROS 1 porting plan.
Preserve Autoware planning topic names, message names, fields, parameters, and behavior unless we explicitly decide otherwise.
```

## Module Workflow

1. Find the package in `modules/inventory.csv`.
2. Check runtime units in `modules/runtime_surfaces.csv`.
3. Open the package spec in `modules/specs/`.
4. Check topic/service/parameter contracts in `interfaces/contracts.csv` or the package spec.
5. Check message/service/action fields in `interfaces/messages.csv`.
6. Read matching upstream docs in `docs/official/` only when you need the module's documented purpose, diagrams, algorithm notes, or design rationale.
7. If anything is ambiguous, use `meta/source_repositories.md` to verify against upstream source.

## Limits

- Full upstream source clones are not committed.
- Raw mirrored HTML is not committed.
- Non-planning docs were pruned.
- Main-head duplicate source-derived indices were pruned.
- Human-readable duplicate renders of CSV data were pruned; use the CSVs as canonical local structured data.
- Package specs are navigation aids, not authoritative rewrites of Autoware behavior.
- `docs/official/` is not the primary working index. It is retained so Codex can inspect upstream planning documentation locally without depending on live web access.
