---
source_type: rendered_autoware_docs
site: autoware_universe
title: Path Generation Design
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_path_generation_design/
rendered_html: rendered/autoware_universe_html/index-197.html
generated: 2026-06-24T14:59:34.306590+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Path Generation Design

This document explains how the path is generated for lane change and avoidance, etc. The implementation can be found in [path\_shifter.hpp](https://github.com/autowarefoundation/autoware_universe/blob/fcd1c7bdbb7bef749b57e8c13ba255fa4e7de2ae/planning/behavior_path_planner/autoware_behavior_path_planner_common/include/autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp).

## Overview

The base idea of the path generation in lane change and avoidance is to smoothly shift the reference path, such as the center line, in the lateral direction. This is achieved by using a constant-jerk profile as in the figure below. More details on how it is used can be found in [README](https://github.com/autowarefoundation/autoware_universe/blob/fcd1c7bdbb7bef749b57e8c13ba255fa4e7de2ae/planning/behavior_path_planner/autoware_behavior_path_planner/README.md). It is assumed that the reference path is smooth enough for this algorithm.

The figure below explains how the application of a constant lateral jerk \(l^{'''}(s)\) can be used to induce lateral shifting. In order to comply with the limits on lateral acceleration and velocity, zero-jerk time is employed in the figure ( \(T\_a\) and \(T\_v\) ). In each interval where constant jerk is applied, the shift position \(l(s)\) can be characterized by a third-degree polynomial. Therefore the shift length from the reference path can then be calculated by combining spline curves.

![path-shifter](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/images/path_shifter/path_shifter.png)

Note that, due to the rarity of the \(T\_v\) in almost all cases of lane change and avoidance, \(T\_v\) is not considered in the current implementation.

## Mathematical derivation

With initial longitudinal velocity \(v\_0^{\rm lon}\) and longitudinal acceleration \(a^{\rm lon}\), longitudinal position \(s(t)\) and longitudinal velocity at each time \(v^{\rm lon}(t)\) can be derived as:

\[
\begin{align}
s\_1&= v^{\rm lon}\_0 T\_j + \frac{1}{2} a^{\rm lon} T\_j^2 \\
v\_1&= v^{\rm lon}\_0 + a^{\rm lon} T\_j \\
s\_2&= v^{\rm lon}\_1 T\_a + \frac{1}{2} a^{\rm lon} T\_a^2 \\
v\_2&= v^{\rm lon}\_1 + a^{\rm lon} T\_a \\
s\_3&= v^{\rm lon}\_2 T\_j + \frac{1}{2} a^{\rm lon} T\_j^2 \\
v\_3&= v^{\rm lon}\_2 + a^{\rm lon} T\_j \\
s\_4&= v^{\rm lon}\_3 T\_v + \frac{1}{2} a^{\rm lon} T\_v^2 \\
v\_4&= v^{\rm lon}\_3 + a^{\rm lon} T\_v \\
s\_5&= v^{\rm lon}\_4 T\_j + \frac{1}{2} a^{\rm lon} T\_j^2 \\
v\_5&= v^{\rm lon}\_4 + a^{\rm lon} T\_j \\
s\_6&= v^{\rm lon}\_5 T\_a + \frac{1}{2} a^{\rm lon} T\_a^2 \\
v\_6&= v^{\rm lon}\_5 + a^{\rm lon} T\_a \\
s\_7&= v^{\rm lon}\_6 T\_j + \frac{1}{2} a^{\rm lon} T\_j^2 \\
v\_7&= v^{\rm lon}\_6 + a^{\rm lon} T\_j
\end{align}
\]

By applying simple integral operations, the following analytical equations can be derived to describe the shift distance \(l(t)\) at each time under lateral jerk, lateral acceleration, and velocity constraints.

\[
\begin{align}
l\_1&= \frac{1}{6}jT\_j^3\\[10pt]
l\_2&= \frac{1}{6}j T\_j^3 + \frac{1}{2} j T\_a T\_j^2 + \frac{1}{2} j T\_a^2 T\_j\\[10pt]
l\_3&= j T\_j^3 + \frac{3}{2} j T\_a T\_j^2 + \frac{1}{2} j T\_a^2 T\_j\\[10pt]
l\_4&= j T\_j^3 + \frac{3}{2} j T\_a T\_j^2 + \frac{1}{2} j T\_a^2 T\_j + j(T\_a + T\_j)T\_j T\_v\\[10pt]
l\_5&= \frac{11}{6} j T\_j^3 + \frac{5}{2} j T\_a T\_j^2 + \frac{1}{2} j T\_a^2 T\_j + j(T\_a + T\_j)T\_j T\_v \\[10pt]
l\_6&= \frac{11}{6} j T\_j^3 + 3 j T\_a T\_j^2 + j T\_a^2 T\_j + j(T\_a + T\_j)T\_j T\_v\\[10pt]
l\_7&= 2 j T\_j^3 + 3 j T\_a T\_j^2 + j T\_a^2 T\_j + j(T\_a + T\_j)T\_j T\_v
\end{align}
\]

These equations are used to determine the shape of a path. Additionally, by applying further mathematical operations to these basic equations, the expressions of the following subsections can be derived.

### Calculation of maximum acceleration from transition time and final shift length

In cases where there are no limitations on lateral velocity and lateral acceleration, the maximum lateral acceleration during the shifting can be calculated as follows. The constant-jerk time is given by \(T\_j = T\_{\rm total}/4\) because of its symmetric property. Since \(T\_a=T\_v=0\), the final shift length \(L=l\_7=2jT\_j^3\) can be determined using the above equation. The maximum lateral acceleration is then given by \(a\_{\rm max} =jT\_j\). This results in the following expression for the maximum lateral acceleration:

\[
\begin{align}
a\_{\rm max}^{\rm lat} = \frac{8L}{T\_{\rm total}^2}
\end{align}
\]

### Calculation of Ta, Tj and jerk from acceleration limit

In cases where there are no limitations on lateral velocity, the constant-jerk and acceleration times, as well as the required jerk can be calculated from the acceleration limit, total time, and final shift length as follows. Since \(T\_v=0\), the final shift length is given by:

\[
\begin{align}
L = l\_7 = 2 j T\_j^3 + 3 j T\_a T\_j^2 + j T\_a^2 T\_j
\end{align}
\]

Additionally, the velocity profile reveals the following relations:

\[
\begin{align}
a\_{\rm lim}^{\rm lat} &= j T\_j\\
T\_{\rm total} &= 4T\_j + 2T\_a
\end{align}
\]

By solving these three equations, the following can be obtained:

\[
\begin{align}
T\_j&=\frac{T\_{\rm total}}{2} - \frac{2L}{a\_{\rm lim}^{\rm lat} T\_{\rm total}}\\[10pt]
T\_a&=\frac{4L}{a\_{\rm lim}^{\rm lat} T\_{\rm total}} - \frac{T\_{\rm total}}{2}\\[10pt]
jerk&=\frac{2a\_{\rm lim} ^2T\_{\rm total}}{a\_{\rm lim}^{\rm lat} T\_{\rm total}^2-4L}
\end{align}
\]

Where \(T\_j\) is the constant-jerk time, \(T\_a\) is the constant acceleration time, \(j\) is the required jerk, \(a\_{\rm lim}^{\rm lat}\) is the lateral acceleration limit, and \(L\) is the final shift length.

### Calculation of required time from jerk and acceleration constraint

In cases where there are no limitations on lateral velocity, the total time required for shifting can be calculated from the lateral jerk and lateral acceleration limits and the final shift length as follows. By solving the two equations given above:

\[
L = l\_7 = 2 j T\_j^3 + 3 j T\_a T\_j^2 + j T\_a^2 T\_j,\quad a\_{\rm lim}^{\rm lat} = j T\_j
\]

We obtain the following expressions:

\[
\begin{align}
T\_j &= \frac{a\_{\rm lim}^{\rm lat}}{j}\\[10pt]
T\_a &= \frac{1}{2}\sqrt{\frac{a\_{\rm lim}^{\rm lat}}{j}^2 + \frac{4L}{a\_{\rm lim}^{\rm lat}}} - \frac{3a\_{\rm lim}^{\rm lat}}{2j}
\end{align}
\]

The total time required for shifting can then be calculated as \(T\_{\rm total}=4T\_j+2T\_a\).

## Limitation

- Since \(T\_v\) is zero in almost all cases of lane change and avoidance, \(T\_v\) is not considered in the current implementation.
