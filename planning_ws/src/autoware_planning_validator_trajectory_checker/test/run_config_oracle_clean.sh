#!/usr/bin/env bash
set -euo pipefail

# The provider DSO pulls the parent validator's motion-utils closure.  Catkin's
# env_cached.sh only exports this package and the parent ABI, so it cannot
# distinguish a missing DSO from a checker failure.  Keep the full closure,
# but receive this configure's installed provider prefix explicitly rather
# than silently validating the historical trajectory-checker installation.
test_binary="$1"
candidate_prefix="$2"
validator_prefix="$3"
# Resolve the configured installation closure, never historical work directories.
set +u
source "$validator_prefix/setup.bash"
source "$candidate_prefix/setup.bash"
set -u
rospack profile >/dev/null
for package in autoware_planning_validator autoware_planning_validator_trajectory_checker \
  autoware_planning_validator_latency_checker autoware_planning_validator_intersection_collision_checker \
  autoware_planning_validator_rear_collision_checker; do
  rospack find "$package" >/dev/null
done
rosparam load "$candidate_prefix/share/autoware_planning_validator_trajectory_checker/config/trajectory_checker.ros1.yaml" /aw149_full_ingress
exec "$test_binary"
