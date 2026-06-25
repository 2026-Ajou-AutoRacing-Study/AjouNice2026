> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Interfaces

Planning topics, services, parameters, and message definitions.

## Files

- `contracts.csv`
  - Full extracted planning contract matrix.
  - Contains publishers, subscriptions, services/clients, runtime parameters, and config defaults.

- `messages.csv`
  - Autoware msg/srv/action definitions used by planning interfaces.
  - Includes nested Autoware field types where available.

- `coverage.md`
  - Notes on which external/helper types are mentioned by planning but not expanded locally.

Duplicate Markdown renders of `contracts.csv` and `messages.csv` are intentionally not kept. The CSVs are the canonical local structured data.

## How To Use

Start with the relevant package spec in `../modules/specs/`, filter `contracts.csv` by package/topic/parameter, then use `messages.csv` for exact message fields.
