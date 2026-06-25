---
source_type: rendered_autoware_docs
site: autoware_documentation
title: /api/planning/cooperation/set_commands
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/list/api/planning/cooperation/set_commands/
rendered_html: rendered/autoware_documentation_html/index-167.html
generated: 2026-06-24T14:59:50.068942+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# /api/planning/cooperation/set\_commands

## Status

- Latest Version: not released
- Method: function call
- Data Type: [autoware\_adapi\_v1\_msgs/srv/SetCooperationCommands](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/types/autoware_adapi_v1_msgs/srv/SetCooperationCommands/)

## Description

Set the operator's decision for cooperation.
For details, see the [cooperation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/cooperation/).

## Request

| Name | Type | Description |
| --- | --- | --- |
| commands.uuid | unique\_identifier\_msgs/msg/UUID | The ID in the cooperation status. |
| commands.cooperator | autoware\_adapi\_v1\_msgs/msg/CooperationDecision | The operator's decision. |

## Response

| Name | Type | Description |
| --- | --- | --- |
| status | autoware\_adapi\_v1\_msgs/msg/ResponseStatus | response status |
