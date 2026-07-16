# Out Of Lane Detector

This module detects when the ego is out of the provided reference path laterally.

Inputs:
- `/odometry` : `nav_msgs/Odometry` (ego pose)
- `/input/path_with_lane_id` : `autoware_internal_planning_msgs/PathWithLaneId` (reference path)

Outputs:
- `~output/out_of_lane` : `std_msgs/Bool` (true = out of lane)

Parameters:
- `~lateral_threshold` (float, default 0.8 m): lateral distance threshold to consider ego out of lane
- `~check_rate` (float, default 10.0 Hz): how often to evaluate

Behavior:
- Computes minimum Euclidean distance between ego pose and reference path points; publishes true if greater than `lateral_threshold`.

Notes:
- This is a simple detector for initial integration; for lanelet-aware checking, the module should use lanelet map projection and lateral offset computations.
