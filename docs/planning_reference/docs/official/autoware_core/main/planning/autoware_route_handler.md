---
source_type: rendered_autoware_docs
site: autoware_core
title: route handler
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/autoware_route_handler/
rendered_html: rendered/autoware_core_html/index-62.html
generated: 2026-06-24T15:00:04.971903+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# route handler

`route_handler` is a library for calculating driving route on the lanelet map.

## Unit Testing

The unit testing depends on `autoware_test_utils` package.
`autoware_test_utils` is a library that provides several common functions to simplify unit test creating.

![route_handler_test](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_route_handler/images/route_handler_test.svg)

By default, route file is necessary to create tests. The following illustrates the route that are used in the unit test

### Lane change test route

![lane_change_test_route](https://autowarefoundation.github.io/autoware_core/main/planning/autoware_route_handler/images/lane_change_test_route.svg)

- The route is based on map that can be obtained from `autoware_test_utils\test_map`
