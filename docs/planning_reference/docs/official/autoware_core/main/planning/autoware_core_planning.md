---
source_type: rendered_autoware_docs
site: autoware_core
title: Autoware Core Planning
canonical_url: https://autowarefoundation.github.io/autoware_core/main/planning/autoware_core_planning/
rendered_html: rendered/autoware_core_html/index-56.html
generated: 2026-06-24T15:00:04.782540+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Autoware Core Planning

## Overview

The `autoware_core_planning` is a meta-package within the Autoware project, designed to manage and organize various planning-related modules essential for autonomous vehicle operations. This package serves as a central point for integrating different planning functionalities, such as path planning, path optimization, and trajectory generation.

## Usage

The `autoware_core_planning` meta-package is used to launch and manage planning modules as part of the Autoware system. You can start the system using:

```
ros2 launch autoware_launch planning_launch.xml
```
