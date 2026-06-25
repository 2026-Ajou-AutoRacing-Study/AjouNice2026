> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning Autoware Rendered Docs Markdown

This directory contains planning-related Markdown converted from the mirrored rendered Autoware documentation HTML snapshots.

These files are documentation references only. They do not define a ROS 1 porting plan or reinterpret Autoware behavior.

Inline `data:image` diagrams were extracted into sibling files under `../assets/` so the Markdown remains readable in GitHub and editors. Relative media links were rewritten to the original Autoware documentation URLs when local media files are not committed in the lean bundle.

## Counts

- `autoware_universe`: 84
- `autoware_documentation`: 21
- `autoware_core`: 15

## Path Rule

Markdown paths are derived from each page canonical URL, not from HTTrack temporary file names.
For example, a canonical Autoware Universe page under `/autoware_universe/main/planning/.../` is stored under `autoware_universe/main/planning/...md`.

Only planning-related rendered docs are retained in this GitHub-friendly corpus. Non-planning Autoware docs were pruned.

## Manifest

See `../../meta/official_docs_manifest.csv` for the mapping from canonical URL and mirrored HTML file to Markdown output.

## Known Original-Docs Artifacts

Some pages include PlantUML diagrams that were already rendered as error images in the original documentation output. These are preserved as extracted SVG assets for traceability.

Some math-heavy pages may need comparison with the upstream raw Markdown because the rendered HTML snapshots do not always preserve LaTeX source perfectly when converted back to Markdown. When this is found, keep the official page path but prefer the upstream raw Markdown expression and record the raw source URL in the page front matter.
