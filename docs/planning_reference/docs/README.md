> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Upstream Docs Snapshot

Selected upstream Autoware planning documentation retained as local Markdown for Codex.

This is not the primary working index for porting. Start from `../modules/` and `../interfaces/` when implementing or reviewing code. Use this folder when the source-derived package specs do not explain the documented purpose, algorithm, diagrams, or design rationale.

## `official/`

Official rendered Autoware planning documentation converted to Markdown.

Use this as a local upstream docs snapshot, not as a replacement for source-derived contracts.

## `assets/`

Extracted diagram/image assets used by `official/`.

Most normal development work should not start from this folder.

## Why There Is No Broad Source Markdown Folder

The previous source Markdown corpus contained many duplicated or adjacent non-planning README files. It was removed to keep this reference readable.

Source-derived planning details are available through:

- `../modules/specs/`
- `../interfaces/contracts.csv`
- `../interfaces/messages.csv`
