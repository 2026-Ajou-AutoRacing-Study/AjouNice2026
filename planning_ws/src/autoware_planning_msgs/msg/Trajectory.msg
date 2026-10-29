std_msgs/Header header
autoware_planning_msgs/TrajectoryPoint[] points
# Permission belongs to this exact geometry, never to all trajectories while a
# node happens to be recovering. Propagate unchanged through velocity planning.
bool boundary_recovery
