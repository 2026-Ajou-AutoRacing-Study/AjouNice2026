> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Meta

Provenance and acquisition inputs for this planning reference bundle.

## Files

- `source_repositories.md`
  - Upstream repository URLs, requested versions, actual commits, and tags.

- `source_repositories.csv`
  - CSV form of the source repository manifest.

- `official_docs_manifest.csv`
  - Mapping from retained official docs canonical URLs to local Markdown files.

- `planning_reference_summary.md`
  - Summary of what was kept and what was pruned.

This folder is for traceability. Most development work should start from `../docs/`, `../modules/`, or `../interfaces/`.

Acquisition-only input files such as `.repos` and planning-doc URL lists are not retained separately. Their repository/version rows are captured in `source_repositories.csv`, and retained documentation URL mappings are captured in `official_docs_manifest.csv`.
