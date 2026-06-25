---
source_type: rendered_autoware_docs
site: autoware_universe
title: Path Smoothing
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_smoother/
rendered_html: rendered/autoware_universe_html/index-165.html
generated: 2026-06-24T14:59:32.202591+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Path Smoothing

## Purpose

This package contains code to smooth a path or trajectory.

## Features

### Elastic Band

More details about the elastic band can be found [here](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_smoother/docs/eb/).

## Parameter

| Name | Type | Description | Default | Range |
| --- | --- | --- | --- | --- |
| common.output\_delta\_arc\_length | float | Delta arc length for output trajectory [m]. | 0.5 | N/A |
| common.output\_backward\_traj\_length | float | Backward length for backward trajectory from base\_link [m]. | 5 | N/A |
| option.enable\_warm\_start | boolean | Enable warm start for elastic band optimization. | 1 | N/A |
| option.enable\_optimization\_validation | boolean | Enable optimization validation. | 0 | N/A |
| common.num\_points | integer | Number of points for optimization. | 100 | N/A |
| common.delta\_arc\_length | float | Delta arc length for optimization [m]. | 1 | N/A |
| clearance.num\_joint\_points | integer | Number of joint points (joint means connecting fixing and smoothing). | 3 | N/A |
| clearance.clearance\_for\_fix | float | Maximum optimizing range when applying fixing. | 0 | N/A |
| clearance.clearance\_for\_joint | float | Maximum optimizing range when applying jointing. | 0.1 | N/A |
| clearance.clearance\_for\_smooth | float | Maximum optimizing range when applying smoothing. | 0.1 | N/A |
| weight.smooth\_weight | float | Weight for smoothing. | 1 | N/A |
| weight.lat\_error\_weight | float | Weight for lateral error. | 0.001 | N/A |
| qp.max\_iteration | integer | Maximum iteration when solving QP. | 10000 | N/A |
| qp.eps\_abs | float | Absolute epsilon when solving OSQP. | 1e-07 | N/A |
| qp.eps\_rel | float | Relative epsilon when solving OSQP. | 1e-07 | N/A |
| validation.max\_error | float | Maximum error for validation [m]. | 3 | N/A |
| ego\_nearest\_dist\_threshold | float | Distance threshold for nearest ego search [m]. | 3 | N/A |
| ego\_nearest\_yaw\_threshold | float | Yaw threshold for nearest ego search [rad]. | 1.046 | N/A |
| replan.enable | boolean | If true, only perform smoothing when the input changes significantly. | 1 | N/A |
| replan.max\_path\_shape\_around\_ego\_lat\_dist | float | Threshold of path shape change around ego [m]. | 2 | N/A |
| replan.max\_path\_shape\_forward\_lon\_dist | float | Forward point to check lateral distance difference [m]. | 100 | N/A |
| replan.max\_path\_shape\_forward\_lat\_dist | float | Threshold of path shape change around forward point [m]. | 0.2 | N/A |
| replan.max\_ego\_moving\_dist | float | Threshold of ego's moving distance for replan [m]. | 5 | N/A |
| replan.max\_goal\_moving\_dist | float | Threshold of goal's moving distance for replan [m]. | 15 | N/A |
| replan.max\_delta\_time\_sec | float | Threshold of delta time for replan [second]. | 0 | N/A |
