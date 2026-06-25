---
source_type: rendered_autoware_docs
site: autoware_universe
title: Model predictive trajectory
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/docs/mpt/
rendered_html: rendered/autoware_universe_html/index-164.html
generated: 2026-06-24T14:59:32.158428+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Model predictive trajectory

## Abstract

Model Predictive Trajectory (MPT) calculates the trajectory that meets the following conditions.

- Kinematically feasible for linear vehicle kinematics model
- Collision free with obstacles and road boundaries

Conditions for collision free is considered to be not hard constraints but soft constraints.
When the optimization failed or the optimized trajectory is not collision free, the output trajectory will be previously generated trajectory.

Trajectory near the ego must be stable, therefore the condition where trajectory points near the ego are the same as previously generated trajectory is considered, and this is the only hard constraints in MPT.

## Flowchart

![uml diagram](../../../../../../assets/autoware_universe/main/planning/autoware_path_optimizer/docs/mpt/image-01-1bf8eb2764.svg)

## Vehicle kinematics

As the following figure, we consider the bicycle kinematics model in the frenet frame to track the reference path.
At time step \(k\), we define lateral distance to the reference path, heading angle against the reference path, and steer angle as \(y\_k\), \(\theta\_k\), and \(\delta\_k\) respectively.

![vehicle_error_kinematics](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/vehicle_error_kinematics.png)

Assuming that the commanded steer angle is \(\delta\_{des, k}\), the kinematics model in the frenet frame is formulated as follows.
We also assume that the steer angle \(\delta\_k\) is first-order lag to the commanded one.

\[
\begin{align}
y\_{k+1} & = y\_{k} + v \sin \theta\_k dt \\\
\theta\_{k+1} & = \theta\_k + \frac{v \tan \delta\_k}{L}dt - \kappa\_k v \cos \theta\_k dt \\\
\delta\_{k+1} & = \delta\_k - \frac{\delta\_k - \delta\_{des,k}}{\tau}dt
\end{align}
\]

### Linearization

Then we linearize these equations.
\(y\_k\) and \(\theta\_k\) are tracking errors, so we assume that those are small enough.
Therefore \(\sin \theta\_k \approx \theta\_k\).

Since \(\delta\_k\) is a steer angle, it is not always small.
By using a reference steer angle \(\delta\_{\mathrm{ref}, k}\) calculated by the reference path curvature \(\kappa\_k\), we express \(\delta\_k\) with a small value \(\Delta \delta\_k\).

Note that the steer angle \(\delta\_k\) is within the steer angle limitation \(\delta\_{\max}\).
When the reference steer angle \(\delta\_{\mathrm{ref}, k}\) is larger than the steer angle limitation \(\delta\_{\max}\), and \(\delta\_{\mathrm{ref}, k}\) is used to linearize the steer angle, \(\Delta \delta\_k\) is \(\Delta \delta\_k = \delta - \delta\_{\mathrm{ref}, k} = \delta\_{\max} - \delta\_{\mathrm{ref}, k}\), and the absolute \(\Delta \delta\_k\) gets larger.
Therefore, we have to apply the steer angle limitation to \(\delta\_{\mathrm{ref}, k}\) as well.

\[
\begin{align}
\delta\_{\mathrm{ref}, k} & = \mathrm{clamp}(\arctan(L \kappa\_k), -\delta\_{\max}, \delta\_{\max}) \\\
\delta\_k & = \delta\_{\mathrm{ref}, k} + \Delta \delta\_k, \ \Delta \delta\_k \ll 1 \\\
\end{align}
\]

\(\mathrm{clamp}(v, v\_{\min}, v\_{\max})\) is a function to convert \(v\) to be larger than \(v\_{\min}\) and smaller than \(v\_{\max}\).

Using this \(\delta\_{\mathrm{ref}, k}\), \(\tan \delta\_k\) is linearized as follows.

\[
\begin{align}
\tan \delta\_k & \approx \tan \delta\_{\mathrm{ref}, k} + \lbrace\frac{d \tan \delta}{d \delta}\rbrace|\_{\delta = \delta\_{\mathrm{ref}, k}} \Delta \delta\_k \\\
& = \tan \delta\_{\mathrm{ref}, k} + \lbrace\frac{d \tan \delta}{d \delta}\rbrace|\_{\delta = \delta\_{\mathrm{ref}, k}} (\delta\_{\mathrm{ref}, k} - \delta\_k) \\\
& = \tan \delta\_{\mathrm{ref}, k} - \frac{\delta\_{\mathrm{ref}, k}}{\cos^2 \delta\_{\mathrm{ref}, k}} + \frac{1}{\cos^2 \delta\_{\mathrm{ref}, k}} \delta\_k
\end{align}
\]

### One-step state equation

Based on the linearization, the error kinematics is formulated with the following linear equations,

\[
\begin{align}
\begin{pmatrix}
y\_{k+1} \\\
\theta\_{k+1}
\end{pmatrix}
=
\begin{pmatrix}
1 & v dt \\\
0 & 1
\end{pmatrix}
\begin{pmatrix}
y\_k \\\
\theta\_k
\end{pmatrix}
+
\begin{pmatrix}
0 \\\
\frac{v dt}{L \cos^{2} \delta\_{\mathrm{ref}, k}}
\end{pmatrix}
\delta\_{k}
+
\begin{pmatrix}
0 \\\
\frac{v \tan(\delta\_{\mathrm{ref}, k}) dt}{L} - \frac{v \delta\_{\mathrm{ref}, k} dt}{L \cos^{2} \delta\_{\mathrm{ref}, k}} - \kappa\_k v dt
\end{pmatrix}
\end{align}
\]

which can be formulated as follows with the state \(\mathbf{x}\), control input \(u\) and some matrices, where \(\mathbf{x} = (y\_k, \theta\_k)\)

\[
\begin{align}
\mathbf{x}\_{k+1} = A\_k \mathbf{x}\_k + \mathbf{b}\_k u\_k + \mathbf{w}\_k
\end{align}
\]

### Time-series state equation

Then, we formulate time-series state equation by concatenating states, control inputs and matrices respectively as

\[
\begin{align}
\mathbf{x} = A \mathbf{x}\_0 + B \mathbf{u} + \mathbf{w}
\end{align}
\]

where

\[
\begin{align}
\mathbf{x} = (\mathbf{x}^T\_1, \mathbf{x}^T\_2, \mathbf{x}^T\_3, \dots, \mathbf{x}^T\_{n-1})^T \\\
\mathbf{u} = (u\_0, u\_1, u\_2, \dots, u\_{n-2})^T \\\
\mathbf{w} = (\mathbf{w}^T\_0, \mathbf{w}^T\_1, \mathbf{w}^T\_2, \dots, \mathbf{w}^T\_{n-1})^T. \\\
\end{align}
\]

In detail, each matrices are constructed as follows.

\[
\begin{align}
\begin{pmatrix}
\mathbf{x}\_1 \\\
\mathbf{x}\_2 \\\
\mathbf{x}\_3 \\\
\vdots \\\
\mathbf{x}\_{n-1}
\end{pmatrix}
=
\begin{pmatrix}
A\_0 \\\
A\_1 A\_0 \\\
A\_2 A\_1 A\_0\\\
\vdots \\\
\prod\limits\_{k=0}^{n-1} A\_{k}
\end{pmatrix}
\mathbf{x}\_0
+
\begin{pmatrix}
B\_0 & 0 & & \dots & 0 \\\
A\_0 B\_0 & B\_1 & 0 & \dots & 0 \\\
A\_1 A\_0 B\_0 & A\_0 B\_1 & B\_2 & \dots & 0 \\\
\vdots & \vdots & & \ddots & 0 \\\
\prod\limits\_{k=0}^{n-3} A\_k B\_0 & \prod\limits\_{k=0}^{n-4} A\_k B\_1 & \dots & A\_0 B\_{n-3} & B\_{n-2}
\end{pmatrix}
\begin{pmatrix}
u\_0 \\\
u\_1 \\\
u\_2 \\\
\vdots \\\
u\_{n-2}
\end{pmatrix}
+
\begin{pmatrix}
I & 0 & & \dots & 0 \\\
A\_0 & I & 0 & \dots & 0 \\\
A\_1 A\_0 & A\_0 & I & \dots & 0 \\\
\vdots & \vdots & & \ddots & 0 \\\
\prod\limits\_{k=0}^{n-3} A\_k & \prod\limits\_{k=0}^{n-4} A\_k & \dots & A\_0 & I
\end{pmatrix}
\begin{pmatrix}
\mathbf{w}\_0 \\\
\mathbf{w}\_1 \\\
\mathbf{w}\_2 \\\
\vdots \\\
\mathbf{w}\_{n-2}
\end{pmatrix}
\end{align}
\]

### Free-boundary-conditioned time-series state equation

For path planning which does not start from the current ego pose, \(\mathbf{x}\_0\) should be the design variable of optimization.
Therefore, we make \(\mathbf{u}'\) by concatenating \(\mathbf{x}\_0\) and \(\mathbf{u}\), and redefine \(\mathbf{x}\) as follows.

\[
\begin{align}
\mathbf{u}' & = (\mathbf{x}^T\_0, \mathbf{u}^T)^T \\\
\mathbf{x} & = (\mathbf{x}^T\_0, \mathbf{x}^T\_1, \mathbf{x}^T\_2, \dots, \mathbf{x}^T\_{n-1})^T
\end{align}
\]

Then we get the following state equation

\[
\begin{align}
\mathbf{x}' = B \mathbf{u}' + \mathbf{w},
\end{align}
\]

which is in detail

\[
\begin{align}
\begin{pmatrix}
\mathbf{x}\_0 \\\
\mathbf{x}\_1 \\\
\mathbf{x}\_2 \\\
\mathbf{x}\_3 \\\
\vdots \\\
\mathbf{x}\_{n-1}
\end{pmatrix}
=
\begin{pmatrix}
I & 0 & \dots & & & 0 \\\
A\_0 & B\_0 & 0 & & \dots & 0 \\\
A\_1 A\_0 & A\_0 B\_0 & B\_1 & 0 & \dots & 0 \\\
A\_2 A\_1 A\_0 & A\_1 A\_0 B\_0 & A\_0 B\_1 & B\_2 & \dots & 0 \\\
\vdots & \vdots & \vdots & & \ddots & 0 \\\
\prod\limits\_{k=0}^{n-1} A\_k & \prod\limits\_{k=0}^{n-3} A\_k B\_0 & \prod\limits\_{k=0}^{n-4} A\_k B\_1 & \dots & A\_0 B\_{n-3} & B\_{n-2}
\end{pmatrix}
\begin{pmatrix}
\mathbf{x}\_0 \\\
u\_0 \\\
u\_1 \\\
u\_2 \\\
\vdots \\\
u\_{n-2}
\end{pmatrix}
+
\begin{pmatrix}
0 & \dots & & & 0 \\\
I & 0 & & \dots & 0 \\\
A\_0 & I & 0 & \dots & 0 \\\
A\_1 A\_0 & A\_0 & I & \dots & 0 \\\
\vdots & \vdots & & \ddots & 0 \\\
\prod\limits\_{k=0}^{n-3} A\_k & \prod\limits\_{k=0}^{n-4} A\_k & \dots & A\_0 & I
\end{pmatrix}
\begin{pmatrix}
\mathbf{w}\_0 \\\
\mathbf{w}\_1 \\\
\mathbf{w}\_2 \\\
\vdots \\\
\mathbf{w}\_{n-2}
\end{pmatrix}.
\end{align}
\]

## Objective function

The objective function for smoothing and tracking is shown as follows, which can be formulated with value function matrices \(Q, R\).

\[
\begin{align}
J\_1 (\mathbf{x}', \mathbf{u}') & = w\_y \sum\_{k} y\_k^2 + w\_{\theta} \sum\_{k} \theta\_k^2 + w\_{\delta} \sum\_k \delta\_k^2 + w\_{\dot \delta} \sum\_k \dot \delta\_k^2 + w\_{\ddot \delta} \sum\_k \ddot \delta\_k^2 \\\
& = \mathbf{x}'^T Q \mathbf{x}' + \mathbf{u}'^T R \mathbf{u}' \\\
& = \mathbf{u}'^T H \mathbf{u}' + \mathbf{u}'^T \mathbf{f}
\end{align}
\]

As mentioned before, the constraints to be collision free with obstacles and road boundaries are formulated to be soft constraints.
Assuming that the lateral distance to the road boundaries or obstacles from the back wheel center, front wheel center, and the point between them are \(y\_{\mathrm{base}, k}, y\_{\mathrm{top}, k}, y\_{\mathrm{mid}, k}\) respectively, and slack variables for each point are \(\lambda\_{\mathrm{base}}, \lambda\_{\mathrm{top}}, \lambda\_{\mathrm{mid}}\), the soft constraints can be formulated as follows.

\[
y\_{\mathrm{base}, k, \min} - \lambda\_{\mathrm{base}, k} \leq y\_{\mathrm{base}, k} (y\_k) \leq y\_{\mathrm{base}, k, \max} + \lambda\_{\mathrm{base}, k}\\\
y\_{\mathrm{top}, k, \min} - \lambda\_{\mathrm{top}, k} \leq y\_{\mathrm{top}, k} (y\_k) \leq y\_{\mathrm{top}, k, \max} + \lambda\_{\mathrm{top}, k}\\\
y\_{\mathrm{mid}, k, \min} - \lambda\_{\mathrm{mid}, k} \leq y\_{\mathrm{mid}, k} (y\_k) \leq y\_{\mathrm{mid}, k, \max} + \lambda\_{\mathrm{mid}, k} \\\
0 \leq \lambda\_{\mathrm{base}, k} \\\
0 \leq \lambda\_{\mathrm{top}, k} \\\
0 \leq \lambda\_{\mathrm{mid}, k}
\]

Since \(y\_{\mathrm{base}, k}, y\_{\mathrm{top}, k}, y\_{\mathrm{mid}, k}\) is formulated as a linear function of \(y\_k\), the objective function for soft constraints is formulated as follows.

\[
\begin{align}
J\_2 & (\mathbf{\lambda}\_\mathrm{base}, \mathbf{\lambda}\_\mathrm{top}, \mathbf {\lambda}\_\mathrm{mid})\\\
& = w\_{\mathrm{base}} \sum\_{k} \lambda\_{\mathrm{base}, k} + w\_{\mathrm{mid}} \sum\_k \lambda\_{\mathrm{mid}, k} + w\_{\mathrm{top}} \sum\_k \lambda\_{\mathrm{top}, k}
\end{align}
\]

Slack variables are also design variables for optimization.
We define a vector \(\mathbf{v}\), that concatenates all the design variables.

\[
\begin{align}
\mathbf{v} =
\begin{pmatrix}
\mathbf{u}'^T & \mathbf{\lambda}\_\mathrm{base}^T & \mathbf{\lambda}\_\mathrm{top}^T & \mathbf{\lambda}\_\mathrm{mid}^T
\end{pmatrix}^T
\end{align}
\]

The summation of these two objective functions is the objective function for the optimization problem.

\[
\begin{align}
\min\_{\mathbf{v}} J (\mathbf{v}) = \min\_{\mathbf{v}} J\_1 (\mathbf{u}') + J\_2 (\mathbf{\lambda}\_\mathrm{base}, \mathbf{\lambda}\_\mathrm{top}, \mathbf{\lambda}\_\mathrm{mid})
\end{align}
\]

As mentioned before, we use hard constraints where some trajectory points in front of the ego are the same as the previously generated trajectory points.
This hard constraints is formulated as follows.

\[
\begin{align}
\delta\_k = \delta\_{k}^{\mathrm{prev}} (0 \leq i \leq N\_{\mathrm{fix}})
\end{align}
\]

Finally we transform those objective functions to the following QP problem, and solve it.

\[
\begin{align}
\min\_{\mathbf{v}} \ & \frac{1}{2} \mathbf{v}^T \mathbf{H} \mathbf{v} + \mathbf{f} \mathbf{v} \\\
\mathrm{s.t.} \ & \mathbf{b}\_{lower} \leq \mathbf{A} \mathbf{v} \leq \mathbf{b}\_{upper}
\end{align}
\]

## Constraints

### Steer angle limitation

Steer angle has a limitation \(\delta\_{max}\) and \(\delta\_{min}\).
Therefore we add linear inequality equations.

\[
\begin{align}
\delta\_{min} \leq \delta\_i \leq \delta\_{max}
\end{align}
\]

### Collision free

To realize collision-free trajectory planning, we have to formulate constraints that the vehicle is inside the road and also does not collide with obstacles in linear equations.
For linearity, we implemented some methods to approximate the vehicle shape with a set of circles, that is reliable and easy to implement.

1. Bicycle Model
2. Uniform Circles
3. Fitting Uniform Circles

![vehicle_circles](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/vehicle_circles.svg)

Now we formulate the linear constraints where a set of circles on each trajectory point is collision-free.
By using the drivable area, we calculate upper and lower boundaries along reference points, which will be interpolated on any position on the trajectory.
NOTE that upper and lower boundary is left and right, respectively.

Assuming that upper and lower boundaries are \(b\_l\), \(b\_u\) respectively, and \(r\) is a radius of a circle, lateral deviation of the circle center \(y'\) has to be

\[
b\_l + r \leq y' \leq b\_u - r.
\]

![bounds](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/bounds.svg)

Based on the following figure, \(y'\) can be formulated as follows.

\[
\begin{align}
y' & = L \sin(\theta + \beta) + y \cos \beta - l \sin(\gamma - \phi\_a) \\\
& = L \sin \theta \cos \beta + L \cos \theta \sin \beta + y \cos \beta - l \sin(\gamma - \phi\_a) \\\
& \approx L \theta \cos \beta + L \sin \beta + y \cos \beta - l \sin(\gamma - \phi\_a)
\end{align}
\]

\[
b\_l + r - \lambda \leq y' \leq b\_u - r + \lambda.
\]

\[
\begin{align}
y' & = C\_1 \mathbf{x} + C\_2 \\\
& = C\_1 (B \mathbf{v} + \mathbf{w}) + C\_2 \\\
& = C\_1 B \mathbf{v} + \mathbf{w} + C\_2
\end{align}
\]

Note that longitudinal position of the circle center and the trajectory point to calculate boundaries are different.
But each boundaries are vertical against the trajectory, resulting in less distortion by the longitudinal position difference since road boundaries does not change so much.
For example, if the boundaries are not vertical against the trajectory and there is a certain difference of longitudinal position between the circe center and the trajectory point, we can easily guess that there is much more distortion when comparing lateral deviation and boundaries.

\[
\begin{align}
A\_{blk} & =
\begin{pmatrix}
C\_1 B & O & \dots & O & I\_{N\_{ref} \times N\_{ref}} & O \dots & O\\\
-C\_1 B & O & \dots & O & I & O \dots & O\\\
O & O & \dots & O & I & O \dots & O
\end{pmatrix}
\in \mathbf{R}^{3 N\_{ref} \times D\_v + N\_{circle} N\_{ref}} \\\
\mathbf{b}\_{lower, blk} & =
\begin{pmatrix}
\mathbf{b}\_{lower} - C\_1 \mathbf{w} - C\_2 \\\
-\mathbf{b}\_{upper} + C\_1 \mathbf{w} + C\_2 \\\
O
\end{pmatrix}
\in \mathbf{R}^{3 N\_{ref}} \\\
\mathbf{b}\_{upper, blk} & = \mathbf{\infty}
\in \mathbf{R}^{3 N\_{ref}}
\end{align}
\]

We will explain options for optimization.

#### L-infinity optimization

The above formulation is called L2 norm for slack variables.
Instead, if we use L-infinity norm where slack variables are shared by enabling `l_inf_norm`.

\[
\begin{align}
A\_{blk} =
\begin{pmatrix}
C\_1 B & I\_{N\_{ref} \times N\_{ref}} \\\
-C\_1 B & I \\\
O & I
\end{pmatrix}
\in \mathbf{R}^{3N\_{ref} \times D\_v + N\_{ref}}
\end{align}
\]

## Tips for stable trajectory planning

In order to make the trajectory optimization problem stabler to solve, the boundary constraint which the trajectory footprints should be inside and optimization weights are modified.

### Keep minimum boundary width

The drivable area's width is sometimes smaller than the vehicle width since the behavior module does not consider the width.
To realize the stable trajectory optimization, the drivable area's width is guaranteed to be larger than the vehicle width and an additional margin in a rule-based way.

We cannot distinguish the boundary by roads from the boundary by obstacles for avoidance in the motion planner, the drivable area is modified in the following multi steps assuming that \(l\_{width}\) is the vehicle width and \(l\_{margin}\) is an additional margin.

![keep_minimum_boundary_width](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/keep_minimum_boundary_width.drawio.svg)

### Extend violated boundary

### Avoid sudden steering

When the obstacle suddenly appears which is determined to avoid by the behavior module, the drivable area's shape just in front of the ego will change, resulting in the sudden steering.
To prevent this, the drivable area's shape close to the ego is fixed as previous drivable area's shape.

Assume that \(v\_{ego}\) is the ego velocity, and \(t\_{fix}\) is the time to fix the forward drivable area's shape.

![avoid_sudden_steering](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/avoid_sudden_steering.drawio.svg)

### Calculate avoidance cost

![avoidance_cost](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_path_optimizer/media/avoidance_cost.drawio.svg)

### Change optimization weights

\[
\begin{align}
r & = \mathrm{lerp}(w^{\mathrm{steer}}\_{\mathrm{normal}}, w^{\mathrm{steer}}\_{\mathrm{avoidance}}, c) \\\
w^{\mathrm{lat}} & = \mathrm{lerp}(w^{\mathrm{lat}}\_{\mathrm{normal}}, w^{\mathrm{lat}}\_{\mathrm{avoidance}}, r) \\\
w^{\mathrm{yaw}} & = \mathrm{lerp}(w^{\mathrm{yaw}}\_{\mathrm{normal}}, w^{\mathrm{yaw}}\_{\mathrm{avoidance}}, r)
\end{align}
\]

Assume that \(c\) is the normalized avoidance cost, \(w^{\mathrm{lat}}\) is the weight for lateral error, \(w^{\mathrm{yaw}}\) is the weight for yaw error, and other variables are as follows.

| Parameter | Type | Description |
| --- | --- | --- |
| \(w^{\mathrm{steer}}\_{\mathrm{normal}}\) | double | weight for steering minimization in normal cases |
| \(w^{\mathrm{steer}}\_{\mathrm{avoidance}}\) | double | weight for steering minimization in avoidance cases |
| \(w^{\mathrm{lat}}\_{\mathrm{normal}}\) | double | weight for lateral error minimization in normal cases |
| \(w^{\mathrm{lat}}\_{\mathrm{avoidance}}\) | double | weight for lateral error minimization in avoidance cases |
| \(w^{\mathrm{yaw}}\_{\mathrm{normal}}\) | double | weight for yaw error minimization in normal cases |
| \(w^{\mathrm{yaw}}\_{\mathrm{avoidance}}\) | double | weight for yaw error minimization in avoidance cases |
