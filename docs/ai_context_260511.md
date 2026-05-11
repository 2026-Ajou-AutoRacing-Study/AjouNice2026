# AI Development Context: AjouNice Autonomous Driving Stack

This file is intended for AI assistants that will help with actual software development. It is not a presentation document. Treat it as the technical context and source-of-truth summary for the planned autonomous driving stack.

## 0. Source-of-Truth Policy

Use the final research proposal as the primary source of truth. Earlier brainstorming ideas may have been discarded and should not be assumed unless explicitly present here.

When there is uncertainty, prioritize the following order:

1. Final research proposal architecture.
2. Competition constraints and simulator availability.
3. Practical ROS1 implementation feasibility.
4. Previous brainstorming only as optional background, not as a requirement.

The stack is Autoware-inspired, but the target runtime is ROS1. Do not assume that Autoware Universe ROS2 packages can be used directly. If an Autoware Universe module is referenced, interpret it as an algorithmic reference or a module to be adapted/reimplemented for ROS1 unless the team explicitly confirms ROS2 bridging or porting.

---

## 1. Project Summary

Project: 2026 University AI/SW Mobility Competition, AI-converged Autonomous Driving division.

Team objective: Build a simulator-based autonomous driving stack using multi-sensor perception, LiDAR-inertial localization, Autoware-inspired planning, MPC/PI control, and diagnostics monitoring.

High-level pipeline:

```text
Sensors
  -> Sensor preprocessing / point cloud concatenation
  -> Localization: FAST-LIO2 / multi-sensor fusion
  -> Perception: BEVFusion
  -> Tracking: SimpleTrack
  -> Trajectory prediction: Autoware map_based_prediction-inspired module
  -> Planning: Autoware-inspired lane-driving / parking planners
  -> Control: lateral MPC + longitudinal PI
  -> Vehicle control command
```

Parallel diagnostics monitor all major modules and can trigger speed limiting, controlled stop, or emergency stop.

---

## 2. Target Runtime and Network Context

Runtime environment:

- ROS1-based system.
- LiDAR packets are received through UDP.
- Other sensor and module interfaces are handled through ROS1 topics.
- The simulator provides vehicle/sensor streams according to the competition environment.

Important integration note:

- Use ROS1-compatible packages or wrappers where possible.
- If a referenced stack is ROS2-native, do not assume direct use. Plan for ROS1 adaptation, message conversion, or algorithmic reimplementation.
- The final stack should be robust and simple enough to run during competition without manual intervention after launch.

---

## 3. Sensor Configuration

Planned sensors:

```text
GPS:    1 unit
IMU:    1 unit
LiDAR:  HDL32 x 2
Camera: 6 cameras, each approximately FOV 55 degrees
```

LiDAR layout:

- Two HDL32 LiDARs.
- One forward-facing / forward-coverage LiDAR.
- One backward-facing / backward-coverage LiDAR.
- Both point clouds are transformed into the `base_link` coordinate frame.
- The two point clouds are concatenated into a unified point cloud before downstream perception/localization when required.

Camera layout:

- Six-camera setup intended to resemble the nuScenes-style 360-degree camera configuration.
- Proposed views:
  - Front
  - Front Right
  - Back Right
  - Back
  - Back Left
  - Left Right / side view as represented in the proposal diagram
- The purpose of the six-camera configuration is to support BEVFusion-style multi-view camera fusion.

Network:

```text
LiDAR: UDP
Other sensors and modules: ROS1
```

Coordinate frame convention:

- `base_link` is the main vehicle body frame.
- All LiDAR point clouds should be transformed into `base_link` before concatenation.
- Camera extrinsics should be defined relative to `base_link` for BEVFusion projection.
- GPS/IMU should also be linked to `base_link` through static or calibrated transforms.

---

## 4. Sensor Preprocessing and Calibration

### 4.1 Dual-LiDAR Point Cloud Concatenation

Purpose:

- Combine front and rear HDL32 point clouds into a single vehicle-frame point cloud.
- Increase spatial coverage and reduce blind spots.

Inputs:

```text
/front_lidar/points
/back_lidar/points
TF: front_lidar_frame -> base_link
TF: back_lidar_frame  -> base_link
```

Output:

```text
/points_concat or /concatenated_pointcloud
```

Required behavior:

- Transform each cloud to `base_link`.
- Time-align as much as possible.
- Concatenate into one `sensor_msgs/PointCloud2`.
- Preserve timestamps and frame IDs consistently.
- Monitor point cloud latency and packet drops.

Potential failure modes:

- Incorrect extrinsic calibration causing duplicated or shifted objects.
- Unsynchronized point clouds causing moving objects to appear distorted.
- Excessive CPU load due to large point clouds.

### 4.2 Camera-LiDAR Calibration

BEVFusion requires accurate camera-LiDAR calibration.

Required calibration data:

- Camera intrinsics.
- Camera extrinsics relative to `base_link`.
- LiDAR extrinsics relative to `base_link`.
- Image resolution and FOV.
- Timestamp alignment between images and LiDAR.

If calibration is inaccurate, camera features and LiDAR geometry will not align correctly in BEV space.

---

## 5. Localization Stack: FAST-LIO2 / Multi-Sensor Fusion

Reference:

- Xu et al., "FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter," 2021.

Objective:

- Estimate real-time 6-DOF ego vehicle pose using LiDAR-inertial localization.
- Use concatenated LiDAR point cloud and IMU in a tightly coupled manner.
- Use GNSS and wheel odometry as auxiliary information when available to reduce accumulated drift.
- Produce ego pose and twist for perception, prediction, planning, and control.

Primary algorithmic basis:

- FAST-LIO2-style LiDAR-Inertial Odometry.
- Tightly coupled LiDAR + IMU fusion.
- Iterated Kalman Filter / ESKF-style state estimation.
- Incremental local LiDAR map for scan-to-map matching.

Planned inputs:

```text
/concatenated_pointcloud     # sensor_msgs/PointCloud2, dual HDL32 cloud in base_link or LiDAR frame with TF
/imu                         # IMU, 6-axis, target 500 Hz if available
/gnss                        # GNSS absolute positioning, if available
/wheel_odom                  # Vehicle wheel odometry, if available and allowed by simulator/competition setup
```

Important note about wheel odometry:

- The proposal includes wheel odometry as an auxiliary localization input.
- During actual implementation, verify whether wheel odometry is available and allowed in the competition simulator.
- If wheel odometry is unavailable, the localization stack should still operate with LiDAR + IMU, optionally with GNSS drift correction.

Expected outputs:

```text
/localization/odom           # nav_msgs/Odometry-style output; contains pose and twist
/localization/pose           # optional pose-only output
/localization/twist          # optional twist-only output
/local_map                   # incremental local LiDAR map, if exposed by FAST-LIO2
```

The odometry output should include:

```text
Position:    x, y, z
Orientation: roll, pitch, yaw or quaternion
Linear velocity:  vx, vy, vz
Angular velocity: wx, wy, wz
```

Target output rate:

```text
~50 Hz nominal ego odometry output, if computationally feasible
```

Performance target from proposal:

- FAST-LIO-style localization is expected to be low-latency.
- The proposal cites approximately 1.82 ms per frame on Intel CPU and 5.23 ms per frame on ARM CPU as reference performance.

### 5.1 ZUPT: Zero Velocity Update

Planned purpose:

- Suppress IMU integration error and pose drift while the vehicle is stationary.
- Reduce drift during stop phases.

Trigger strategy:

```text
Stationary detection using wheel odometry if available.
If wheel odometry is unavailable, consider fallback stationary detection using:
  - low linear acceleration magnitude variation
  - low angular velocity magnitude
  - minimal LiDAR scan-to-map motion
  - simulator vehicle status if available and allowed
```

ZUPT update concept:

- When the vehicle is confidently stationary, impose a zero-velocity constraint.
- Correct velocity estimate and optionally refine IMU bias.
- Avoid applying ZUPT during slow but nonzero motion.

Do not overuse ZUPT. Incorrect stationary detection can corrupt the state estimator.

---

## 6. Perception Stack: BEVFusion

Reference:

- Liu et al., "BEVFusion: Multi-Task Multi-Sensor Fusion with Unified Bird's-Eye View Representation," 2022.

One-line definition:

- BEVFusion is a multi-modal perception framework that projects camera and LiDAR features into a unified bird's-eye-view representation and performs 3D object detection and BEV semantic understanding in that shared space.

Objective:

- Use camera + LiDAR unified BEV representation for multi-modal 3D object detection.
- Use semantic information from six cameras and geometric information from HDL32 LiDAR point clouds.
- Produce object-level 3D detections for tracking/prediction and BEV semantic map outputs for planning support.

Planned inputs:

```text
Images from 6 monocular cameras, approximately 360-degree coverage
Concatenated point cloud from 32-channel LiDAR setup
Camera intrinsics/extrinsics
LiDAR extrinsics
Timestamp alignment information
```

Training dataset strategy:

- Use nuScenes as the reference training/pretraining dataset because its sensor configuration is similar to the planned six-camera 360-degree setup.
- Because the competition simulator sensor configuration and domain will differ from nuScenes, create a simulator-specific dataset when possible.
- Use task-specific fine-tuning on simulator data.
- Apply augmentation to reduce the domain gap.

Possible augmentation categories:

```text
Camera:
  - brightness / contrast / gamma shift
  - blur / noise
  - random crop / resize consistent with calibration assumptions
  - synthetic weather or lighting variation if simulator supports it

LiDAR:
  - point dropout
  - range noise
  - intensity perturbation
  - random object occlusion

BEV / object-level:
  - rotation / translation within valid coordinate assumptions
  - class balancing if needed
```

Important caution:

- Fine-tuning is only realistic if synchronized camera + LiDAR data and labels are available.
- If no 3D ground-truth labels are available from the simulator, training BEVFusion fully may require manual labeling, automatic simulator annotations, or a simplified detector path.
- Do not assume nuScenes-pretrained weights will work perfectly without adaptation.

Expected BEVFusion outputs:

```text
3D Bounding Boxes:
  x, y, z
  w, l, h
  yaw
  vx
  score
  class

BEV Semantic Map:
  drivable area
  lane
  sidewalk
```

Downstream usage:

- 3D bounding boxes go to tracking.
- BEV semantic map can support drivable area, occupancy grid, and collision checking in planning.
- Dynamic objects should preferably remain object-level through tracking and prediction instead of being flattened directly into an occupancy grid only.

---

## 7. Tracking Stack: SimpleTrack

Reference:

- Pang et al., "SimpleTrack: Understanding and Rethinking 3D Multi-object Tracking," 2021.

Objective:

- Assign and maintain consistent object IDs across frames using the 3D object detection results from BEVFusion.
- Provide temporally consistent tracked objects to the trajectory prediction module.

Rationale:

- SimpleTrack is lightweight and suitable for real-time competition environments.
- It avoids heavy learned components.
- It combines 3D GIoU-based association and Kalman Filter-based motion estimation.
- This is appropriate when the input is already 3D bounding boxes from BEVFusion.

Input:

```text
3D bounding boxes from BEVFusion
No tracking ID included at input
```

Output:

```text
Tracked 3D objects
Each object includes a persistent tracking ID
Object state may include pose, size, velocity, class, and confidence
```

Required behavior:

- Maintain stable IDs through short occlusions when possible.
- Reduce frame-to-frame detection flicker.
- Estimate object velocity for prediction.
- Drop stale tracks after a timeout.
- Initialize new tracks from high-confidence detections.

---

## 8. Trajectory Prediction Stack: Autoware map_based_prediction-Inspired Module

Reference:

- Autoware `map_based_prediction` concept/documentation.

Objective:

- Predict future trajectories for tracked objects.
- Use tracked object states and HD map constraints to generate map-aware predicted trajectories.
- Support planning with dynamic obstacle future motion, not just instantaneous occupancy.

Inputs:

```text
Tracked 3D objects from SimpleTrack
HD map / lanelet2 map
Ego pose from localization
```

Output:

```text
Predicted trajectories for surrounding objects
Each trajectory contains future waypoints and associated object ID
```

Expected method:

- Baseline constant-velocity model for short-term prediction.
- HD map / lane constraints to keep predictions lane-consistent where applicable.
- For vehicles, prefer lane-following predictions when a map lane association exists.
- For uncertain objects, produce conservative short-horizon predictions.

Planning interface:

- Planning should consume tracked objects plus predicted trajectories.
- Dynamic obstacle collision checking should use future trajectory information when available.
- Occupancy grid should be treated as a complementary representation, especially for static obstacles or free-space planning.

ROS1 caution:

- Autoware Universe `map_based_prediction` may not be directly available in ROS1.
- Implement a ROS1-compatible prediction node or adapt the concept.

---

## 9. Planning Stack: Autoware-Inspired Planning System

Planning is inspired by the Autoware planning system and uses both object-level dynamic information and BEV/occupancy information.

Planning inputs:

```text
Ego status:
  - pose
  - twist

Perception:
  - tracked 3D bounding boxes
  - predicted future trajectories

BEV map:
  - drivable area
  - occupancy grid map

Map / route:
  - HD map / lanelet2-style map if available
  - mission goal and checkpoints
```

Planning output:

```text
Final ego trajectory
```

The planning system is divided into two high-level scenario families:

1. Lane-driving planner.
2. Free-space / parking planner.

### 9.1 Lane-Driving Planning Pipeline

Nodes and responsibilities:

```text
Mission Planner:
  - Generate global route using goal, checkpoints, and map.

Behavior Path Planner:
  - Generate route-based path.
  - Define drivable area.
  - Handle lane following, lane change, static obstacle avoidance, pull out, pull over, etc.

Behavior Velocity Planner:
  - Apply rule-based velocity constraints.
  - Generate stop points for stop lines, traffic lights, intersections, etc.

Path Optimizer:
  - Generate collision-free trajectory from path and drivable area.
  - Maintain appropriate path velocity.

Motion Velocity Planner:
  - Refine final trajectory velocity profile using obstacle, curvature, acceleration, deceleration, and jerk constraints.
  - Ensure the trajectory is trackable by the control module.

Scenario Selector:
  - Select between lane-driving trajectory and parking/free-space trajectory depending on current scenario.
  - Send selected final trajectory to control.
```

### 9.2 Parking / Free-Space Planning Pipeline

Nodes and responsibilities:

```text
Parking Goal Planner:
  - Set parking target pose and ROI when entering parking scenario.
  - Generate free-space planning conditions from current ego pose and goal pose.

Occupancy Grid:
  - Represent surrounding obstacles and available parking space as grid/costmap.
  - Use LiDAR / BEV perception output to generate collision-checking map.

Freespace Planner:
  - Use Hybrid A* over OGM to generate a parking trajectory that respects vehicle kinematics.
  - Consider vehicle footprint, minimum turning radius, and obstacle regions.
  - Generate a collision-free path.

Direction Handler:
  - Split a parking trajectory containing forward and reverse segments into control-friendly sections.
  - Include gear and direction information for the control module.
```

### 9.3 Object-Level vs Occupancy-Level Planning

Use both representations correctly:

```text
Object-level representation:
  - 3D bounding boxes
  - tracking IDs
  - predicted future trajectories
  - best for dynamic obstacles such as vehicles and pedestrians

Occupancy representation:
  - BEV grid / costmap
  - static obstacles
  - free-space and parking collision checking
  - best for local obstacle occupancy and freespace planning
```

Do not reduce all dynamic objects to occupancy only if future trajectory prediction is available. For dynamic objects, object-level tracking and prediction are important for planning safety.

---

## 10. Control Stack: Lateral MPC + Longitudinal PI

Control objective:

- Track the final trajectory from planning.
- Output vehicle control commands for steering, acceleration, and braking.

Control structure:

```text
Reference trajectory -> Lateral MPC -> Steering command
Target/current speed -> Longitudinal PI -> Accel/brake command
Steering + accel/brake -> vehicle control command
```

### 10.1 Lateral Control: MPC

Reference:

- Autoware trajectory follower concept.

Method:

- Kinematic Bicycle Model-based future vehicle behavior prediction.
- Optimization target includes:
  - lateral error
  - heading error
  - steering effort / steering smoothness

Output:

```text
Steering command
```

### 10.2 Longitudinal Control: PI

Method:

- Compute velocity error between target speed and current speed.
- Apply PI controller.
- Convert PI output to acceleration and brake commands.
- Limit rapid acceleration/deceleration to reduce wheel slip and unstable behavior.

Output:

```text
Acceleration command
Brake command
```

Final vehicle command:

```text
/vehicle_ctrl_cmd or simulator-specific control command topic/interface
```

---

## 11. Diagnostics and Fail-Safe Layer

Diagnostics objective:

- Monitor Sensor, Localization, Perception, Planning, and Control status.
- Help debugging during development.
- Trigger safety actions based on fault level.

Module-level monitoring:

```text
Sensors:
  - Hz
  - timestamp delay
  - packet drop

Localization:
  - pose freshness
  - pose jump

Perception:
  - inference latency
  - detection confidence

Planning:
  - trajectory validity
  - collision-free check

Control:
  - tracking error
  - command saturation
```

Diagnostic aggregator:

- Integrates module statuses.
- Computes system risk level.
- Produces fault-level decision.

Fault levels:

```text
OK:
  - normal driving

WARN:
  - speed limitation

ERROR:
  - controlled stop

FATAL:
  - emergency stop
```

Fail-safe behavior should be conservative. If localization pose is stale, perception latency is too high, or planned trajectory is invalid, the stack should slow down or stop rather than continue blindly.

---

## 12. Development Priorities

Highest-priority implementation path:

1. Bring up simulator sensors in ROS1.
2. Validate LiDAR UDP reception and ROS Velodyne processing.
3. Build static TF tree for GPS, IMU, dual LiDAR, and six cameras relative to `base_link`.
4. Implement point cloud concatenation.
5. Stabilize localization with FAST-LIO2 using LiDAR + IMU first.
6. Add GNSS and wheel odometry fusion only after base LIO is stable and legally/technically available.
7. Implement BEVFusion inference or a simplified BEVFusion-compatible perception path.
8. Add SimpleTrack for tracked 3D objects.
9. Add map-aware short-term prediction.
10. Integrate planning and control.
11. Add diagnostics and fail-safe logic.

Do not start with the most complex full stack. Build a minimal reliable stack first, then add advanced modules.

---

## 13. Key Interface Contracts

### Localization -> Planning / Control

```text
Message style: nav_msgs/Odometry-like
Content:
  - ego position
  - ego orientation
  - linear velocity
  - angular velocity
Rate target: ~50 Hz
```

### Perception -> Tracking

```text
Input to tracking:
  - list of 3D bounding boxes
  - each bbox: x, y, z, w, l, h, yaw, vx, score, class
```

### Tracking -> Prediction

```text
Input to prediction:
  - tracked objects
  - each object has persistent ID
  - pose, velocity, size, class, confidence
```

### Prediction -> Planning

```text
Input to planning:
  - tracked object list
  - predicted future trajectories
  - object IDs and time-indexed future waypoints
```

### BEV Semantic Map / Occupancy -> Planning

```text
Input to planning:
  - drivable area
  - lane / sidewalk semantic map
  - occupancy grid map for collision checking and freespace planning
```

### Planning -> Control

```text
Input to control:
  - final ego trajectory
  - position waypoints
  - target velocity profile
  - optional gear/direction for parking/free-space trajectories
```

### Control -> Simulator

```text
Output:
  - steering
  - accel
  - brake
  - gear/direction if required by simulator
```

---

## 14. Things the AI Assistant Should Avoid Assuming

- Do not assume Autoware Universe ROS2 modules are directly usable in the target ROS1 runtime.
- Do not assume wheel odometry is definitely available until confirmed.
- Do not assume pretrained BEVFusion weights will work without simulator-specific calibration and fine-tuning.
- Do not flatten all perception outputs into occupancy grid only; dynamic objects should go through tracking and prediction if possible.
- Do not ignore TF calibration. Dual LiDAR and camera-LiDAR extrinsics are critical.
- Do not optimize only for SOTA performance. Competition reliability and runtime stability are more important.

---

## 15. Preferred Development Style for AI Assistance

When helping with implementation, the AI assistant should:

- Ask which simulator topics and message types are actually available before writing final integration code.
- Prefer ROS1-compatible C++/Python nodes.
- Provide launch files, topic names, TF assumptions, and message definitions when possible.
- Keep modules individually testable.
- Include fallback behavior and diagnostics.
- Separate algorithmic concept from exact package dependency.
- Prioritize robust, debuggable code over overly complex research implementations.

