---
source_type: rendered_autoware_docs
site: autoware_universe
title: Continuous Jerk Filter
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_trajectory_optimizer/docs/continuous_jerk_filter/
rendered_html: rendered/autoware_universe_html/index-176.html
generated: 2026-06-24T14:59:32.850302+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Continuous Jerk Filter

## 1. Overview Introduction

The **Continuous Jerk Smoother** provides jerk-constrained velocity smoothing for neural network output trajectories, ensuring kinematic feasibility while maintaining smooth motion profiles.

### Key Features

- **Jerk-constrained optimization**: Enforces jerk limits for comfortable passenger experience
- **Stop point detection**: Automatically detects and handles zero-velocity points in trajectories
- **Reference tracking**: Tracks input trajectory velocities and accelerations as soft constraints
- **Hard constraint enforcement**: Enforces velocity upper bounds (including lateral acceleration limits)
- **QP-based formulation**: Uses quadratic programming for efficient optimization

### Design Philosophy

The smoother is adapted from `JerkFilteredSmoother` but specifically designed for **neural network output paths**. It optimizes velocity profiles while respecting acceleration and jerk constraints, making it suitable for autonomous driving applications where smooth motion is critical.

## 2. Mathematical Formulation

### 2.1 Optimization Variables

The optimization uses the following variables for N trajectory points:

\[
x = \begin{bmatrix}
b[0], b[1], ..., b[N-1], \\
a[0], a[1], ..., a[N-1], \\
\delta[0], ..., \delta[N-1], \\
\sigma[0], \sigma[1], ..., \sigma[N-1], \\
\gamma[0], \gamma[1], ..., \gamma[N-1]
\end{bmatrix}
\]

Where:

- \(b[i] = v[i]^2\) (velocity squared)
- \(a[i]\) = acceleration at point i
- \(\delta[i]\), \(\sigma[i]\), \(\gamma[i]\) = slack variables for soft constraints

### 2.2 Objective Function

The optimization minimizes a weighted sum of several objectives:

#### 2.2.1 Jerk Minimization

Minimizes the squared jerk along the trajectory:

\[
J\_{jerk} = \sum\_{i} w\_{jerk} \cdot \left(\frac{a[i+1] - a[i]}{\Delta s} \cdot v\_{ref}[i]\right)^2 \cdot \Delta s
\]

Where:

- \(w\_{jerk}\) = jerk weight parameter
- \(\Delta s\) = distance between consecutive points
- \(v\_{ref}[i]\) = reference velocity at segment i

#### 2.2.2 Velocity Tracking

Tracks reference velocities from input trajectory:

\[
J\_{vel} = \sum\_{i} w\_{vel} \cdot (b[i] - v\_{ref}[i]^2)^2
\]

#### 2.2.3 Acceleration Tracking

Tracks reference accelerations from input trajectory:

\[
J\_{accel} = \sum\_{i} w\_{accel} \cdot (a[i] - a\_{ref}[i])^2
\]

#### 2.2.4 Slack Variable Penalties

Penalizes constraint violations:

\[
J\_{slack} = \sum\_{i} (w\_v \cdot \delta[i]^2 + w\_a \cdot \sigma[i]^2 + w\_j \cdot \gamma[i]^2)
\]

### 2.3 Constraints

#### 2.3.1 Velocity Upper Bound (Soft)

\[
0 \leq b[i] - \delta[i] \leq v\_{max}[i]^2
\]

Where \(\(v\_{max}[i]\)\) includes lateral acceleration limits.

#### 2.3.2 Acceleration Limits (Soft)

\[
a\_{min} \leq a[i] - \sigma[i] \leq a\_{max}
\]

#### 2.3.3 Jerk Limits (Soft)

\[
j\_{min} \cdot \Delta s \leq (a[i+1] - a[i]) \cdot v\_{ref}[i] - \gamma[i] \cdot \Delta s \leq j\_{max} \cdot \Delta s
\]

#### 2.3.4 Dynamics Constraint (Hard)

Relates velocity squared to acceleration:

\[
\frac{b[i+1] - b[i]}{\Delta s} = 2 \cdot a[i]
\]

Derived from kinematic relation: \(\frac{d(v^2)}{ds} = 2a\)

#### 2.3.5 Initial Conditions (Hard)

\[
b[0] = v\_0^2
\]

\[
a[0] = a\_0
\]

### 2.4 Stop Point Detection

The optimizer automatically detects stop points (zero velocity) and:

1. Optimizes only up to the stop point
2. Sets velocities to zero for points after the stop
3. Reduces computational complexity

## 3. Class Structures

### 3.1 `ContinuousJerkSmootherParams`

Configuration structure containing all tunable parameters:

```cpp
struct ContinuousJerkSmootherParams {
// QP optimization weights
double jerk\_weight{10.0}; // Weight for jerk minimization
double over\_v\_weight{10000.0}; // Weight for velocity limit violation
double over\_a\_weight{5000.0}; // Weight for acceleration limit violation
double over\_j\_weight{200.0}; // Weight for jerk limit violation
double velocity\_tracking\_weight{100.0}; // Weight for tracking reference velocity
double accel\_tracking\_weight{10.0}; // Weight for tracking reference acceleration

// Kinematic limits
double max\_accel{2.0}; // Maximum acceleration [m/s²]
double min\_decel{-3.0}; // Minimum deceleration [m/s²]
double max\_jerk{1.5}; // Maximum jerk [m/s³]
double min\_jerk{-1.5}; // Minimum jerk [m/s³]
};
```

### 3.2 `ContinuousJerkSmoother` Class

#### 3.2.1 Public Interface

```cpp
class ContinuousJerkSmoother {
public:
// Constructor
explicit ContinuousJerkSmoother(const ContinuousJerkSmootherParams & params);

// Main smoothing function
bool apply(
const double v0, // Initial velocity [m/s]
const double a0, // Initial acceleration [m/s²]
const TrajectoryPoints & input, // Input trajectory points
TrajectoryPoints & output, // Output smoothed trajectory points
const std::vector & max\_velocity\_per\_point = {} // Per-point velocity upper bounds
);

// Parameter management
void set\_params(const ContinuousJerkSmootherParams & params);
ContinuousJerkSmootherParams get\_params() const;

~ContinuousJerkSmoother() = default;
};
```

## 4. Implementation Details

### 4.1 QP Problem Setup

The optimization problem is formulated as:

\[
\begin{align}
\text{minimize:} & \quad \frac{1}{2} x^T P x + q^T x \\
\text{subject to:} & \quad l \leq Ax \leq u
\end{align}
\]

Where:

- \(P\) = Hessian matrix (quadratic terms)
- \(q\) = Linear term vector
- \(A\) = Constraint matrix
- \(l\), \(\(u\)\) = Lower and upper bounds

### 4.2 Variable Indexing

```
IDX_B0 = 0;      // Start of b variables (velocity squared)
IDX_A0 = N;      // Start of a variables (acceleration)
IDX_DELTA0 = 2N; // Start of delta variables (velocity slack)
IDX_SIGMA0 = 3N; // Start of sigma variables (acceleration slack)
IDX_GAMMA0 = 4N; // Start of gamma variables (jerk slack)
```

### 4.3 Error Handling

- Returns `false` if optimization fails
- Logs warnings for empty trajectories or optimization failures
- Handles NaN values in optimization results
- Provides fallback behavior for edge cases

## 5. Usage Example

```cpp
// Create smoother with default parameters
ContinuousJerkSmootherParams params;
params.max\_accel = 2.5;
params.min\_decel = -4.0;
params.max\_jerk = 2.0;
params.min\_jerk = -2.0;

ContinuousJerkSmoother smoother(params);

// Apply smoothing
double v0 = 1.0; // Initial velocity [m/s]
double a0 = 0.0; // Initial acceleration [m/s²]
TrajectoryPoints input\_trajectory = ...;
TrajectoryPoints output\_trajectory;

std::vector max\_velocities(input\_trajectory.size(), 10.0); // 10 m/s limit

bool success = smoother.apply(v0, a0, input\_trajectory, output\_trajectory, max\_velocities);
```

## 6. Integration with Trajectory Optimizer

The continuous jerk smoother is typically used within the trajectory velocity optimizer pipeline:

1. **Input**: Raw trajectory with reference velocities
2. **Preprocessing**: Calculate per-point velocity limits (considering lateral acceleration)
3. **Optimization**: Apply continuous jerk smoothing
4. **Output**: Kinematically feasible trajectory with smooth velocity profile

## 7. Performance Considerations

- **Computational Complexity**: \(\(O(N^3)\)\) for QP solving, but N is typically small (≤ 100 points)
- **Memory Usage**: Matrices of size \(\((5N \times 5N)\)\) for P and \(\((4N \times 5N)\)\) for A
- **Real-time Feasibility**: Suitable for real-time operation with typical trajectory lengths
