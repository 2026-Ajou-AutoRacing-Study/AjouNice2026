---
source_type: rendered_autoware_docs
site: autoware_documentation
title: /api/planning/cooperation/get_policies
canonical_url: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/list/api/planning/cooperation/get_policies/
rendered_html: rendered/autoware_documentation_html/index-166.html
generated: 2026-06-24T14:59:50.040426+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# /api/planning/cooperation/get\_policies

## Status

- Latest Version: not released
- Method: function call
- Data Type: [autoware\_adapi\_v1\_msgs/srv/GetCooperationPolicies](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/types/autoware_adapi_v1_msgs/srv/GetCooperationPolicies/)

## Description

Get the default decision that is used instead when the operator's decision is undecided.
For details, see the [cooperation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/cooperation/).

## Request

None

## Response

| Name | Type | Description |
| --- | --- | --- |
| status | autoware\_adapi\_v1\_msgs/msg/ResponseStatus | response status |
| policies.behavior | string | The type of the target behavior. |
| policies.sequence | string | The type of the target sequence. |
| policies.policy | uint8 | The type of the cooperation policy. |
