#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp>

#include <gtest/gtest.h>
#include <limits>
#include <autoware_utils/geometry/boost_geometry.hpp>

TEST(SafetyCheckSourceAdapter, IntegralPolygonKeepsUnsafeDebugAndFirstCollision)
{
  namespace check = autoware::behavior_path_planner::utils::path_safety_checker;
  geometry_msgs::Pose ego_pose;
  ego_pose.orientation.w = 1.0;
  const std::vector<check::PoseWithVelocityStamped> ego{{0.0, ego_pose, 0.0}, {1.0, ego_pose, 0.0}};
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.max_longitudinal_offset_m = 2.0;
  vehicle.rear_overhang_m = 1.0;
  vehicle.vehicle_width_m = 2.0;
  check::IntegralPredictedPolygonParams params;
  params.time_horizon = 2.0;
  params.forward_margin = params.backward_margin = params.lat_margin = 0.0;
  const auto object = [&](uint8_t id, double y) {
    check::ExtendedPredictedObject value;
    value.uuid.uuid[0] = id;
    value.initial_pose = ego_pose;
    value.initial_pose.position.y = y;
    const auto poly = autoware_utils::to_footprint(value.initial_pose, 1.0, 1.0, 1.0);
    value.initial_polygon = poly;
    check::PredictedPathWithPolygon path;
    path.confidence = 1.0;
    path.path.emplace_back(0.0, value.initial_pose, 0.0, poly);
    path.path.emplace_back(1.0, value.initial_pose, 0.0, poly);
    value.predicted_paths.push_back(path);
    return value;
  };
  check::CollisionCheckDebugMap debug;
  const auto clear = object(1, 5.0), colliding = object(2, 0.0), unvisited = object(3, 0.0);
  EXPECT_FALSE(check::checkSafetyWithIntegralPredictedPolygon(
    ego, vehicle, {clear, colliding, unvisited}, true, params, debug));
  ASSERT_EQ(debug.size(), 2U);
  EXPECT_TRUE(debug.at(clear.uuid.uuid).is_safe);
  EXPECT_FALSE(debug.at(colliding.uuid.uuid).is_safe);
  EXPECT_EQ(debug.at(colliding.uuid.uuid).ego_predicted_path.size(), ego.size());
  EXPECT_FALSE(debug.at(colliding.uuid.uuid).extended_ego_polygon.outer().empty());
  debug.clear();
  EXPECT_TRUE(check::checkSafetyWithIntegralPredictedPolygon(
    ego, vehicle, {clear}, true, params, debug));
  ASSERT_EQ(debug.size(), 1U);
  EXPECT_TRUE(debug.begin()->second.is_safe);
}

TEST(SafetyCheckSourceAdapter, PreciseFootprintDistanceUsesOffsetPose)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const double x : {0.0, 10.0}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.orientation.w = 1.0;
    path.points.push_back(point);
  }
  autoware::behavior_path_planner::BehaviorPathPlannerParameters parameters;
  parameters.base_link2front = 1.0;
  parameters.base_link2rear = 1.0;
  parameters.vehicle_width = 2.0;
  autoware_perception_msgs::PredictedObjects objects;
  auto & object = objects.objects.emplace_back();
  object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 4.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.0;
  const auto distance = [&](bool offset) {
    return autoware::behavior_path_planner::utils::path_safety_checker::
      shortest_distance_from_ego_footprint_to_objects_on_path(path, objects, parameters, offset);
  };
  EXPECT_NEAR(distance(true), 2.0, 1e-9);
  EXPECT_GT(distance(false), distance(true));
  object.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  EXPECT_NEAR(distance(true), 0.0, 1e-9);
  objects.objects.clear();
  EXPECT_EQ(distance(true), std::numeric_limits<double>::max());
}

namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;

TEST(SafetyCheckDiagnostics, RecordsMarginCollisionWithoutChangingTheDecision)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.max_longitudinal_offset_m = 2.0;
  vehicle.rear_overhang_m = 1.0;
  vehicle.vehicle_width_m = 2.0;
  vehicle.wheel_base_m = 1.0;
  vehicle.front_overhang_m = 1.0;
  vehicle.wheel_tread_m = 2.0;
  const std::vector<safety::PoseWithVelocityStamped> ego{{0.0, pose, 2.0}, {1.0, pose, 2.0}};
  safety::ExtendedPredictedObject object;
  object.uuid.uuid[0] = 7;
  object.initial_pose = pose;
  object.initial_pose.position.x = 8.0;
  safety::PredictedPathWithPolygon prediction;
  prediction.confidence = 0.7;
  prediction.path.emplace_back(0.0, object.initial_pose, 2.0,
    autoware_utils::to_footprint(object.initial_pose, 1.0, 1.0, 1.0));
  object.predicted_paths.push_back(prediction);
  safety::RSSparams params;
  params.front_vehicle_deceleration = params.rear_vehicle_deceleration = -6.0;
  params.rear_vehicle_reaction_time = 3.0;
  params.longitudinal_distance_min_threshold = 3.0;
  safety::CollisionCheckDebugMap baseline, observed;
  safety::RSSCollisionTrace trace;
  autoware_internal_planning_msgs::PathWithLaneId planned;
  const bool old_result = safety::check_safety_with_rss(
    planned, ego, vehicle, {object}, baseline, params, true, 1.0, 3.1416);
  const bool observed_result = safety::check_safety_with_rss_diagnostics(
    planned, ego, vehicle, {object}, observed, params, true, 1.0, 3.1416,
    &trace, 0.0, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(old_result);
  EXPECT_EQ(old_result, observed_result);
  EXPECT_EQ(observed.at(object.uuid.uuid).unsafe_reason, "overlap_extended_polygon");
  EXPECT_EQ(baseline.at(object.uuid.uuid).unsafe_reason,
    observed.at(object.uuid.uuid).unsafe_reason);
  EXPECT_DOUBLE_EQ(trace.time, 0.0);
  EXPECT_DOUBLE_EQ(trace.longitudinal_extension, 6.0);
  EXPECT_DOUBLE_EQ(trace.ego_velocity, 2.0);
  EXPECT_TRUE(trace.object_is_front);
  EXPECT_NEAR(trace.path_confidence, 0.7, 1e-6);
}

TEST(SafetyCheckDiagnostics, PhaseWindowExcludesLaterCollisionAndIncludesBoundary)
{
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.max_longitudinal_offset_m = 2.0;
  vehicle.rear_overhang_m = 1.0;
  vehicle.vehicle_width_m = 2.0;
  vehicle.wheel_base_m = 1.0;
  vehicle.front_overhang_m = 1.0;
  vehicle.wheel_tread_m = 2.0;
  const std::vector<safety::PoseWithVelocityStamped> ego{{0.0, pose, 0.0}, {1.0, pose, 0.0}};
  safety::ExtendedPredictedObject object;
  safety::PredictedPathWithPolygon prediction;
  auto far = pose;
  far.position.y = 10.0;
  prediction.path.emplace_back(0.0, far, 0.0, autoware_utils::to_footprint(far, 1.0, 1.0, 1.0));
  prediction.path.emplace_back(1.0, pose, 0.0, autoware_utils::to_footprint(pose, 1.0, 1.0, 1.0));
  object.predicted_paths.push_back(prediction);
  safety::CollisionCheckDebugMap debug;
  safety::RSSCollisionTrace trace;
  safety::RSSparams params;
  const auto check = [&](double begin, double end) {
    return safety::check_safety_with_rss_diagnostics({}, ego, vehicle, {object}, debug,
      params, true, 1.0, 3.1416, &trace, begin, end);
  };
  EXPECT_FALSE(check(0.0, 2.0));
  EXPECT_DOUBLE_EQ(trace.time, 1.0);
  EXPECT_EQ(debug.begin()->second.unsafe_reason, "overlap_polygon");
  EXPECT_TRUE(check(0.0, 1.0));
  EXPECT_FALSE(check(1.0, 2.0));
  EXPECT_TRUE(check(0.0, 0.0));
}

TEST(SafetyCheckSourceAdapter, PreservesRssAndPathTimeSurface)
{
  safety::RSSparams params;
  EXPECT_DOUBLE_EQ(safety::calcRssDistance(4.0, 6.0, params), 10.0);

  geometry_msgs::Pose pose0;
  pose0.orientation.w = 1.0;
  geometry_msgs::Pose pose1 = pose0;
  pose1.position.x = 2.0;
  const std::vector<safety::PoseWithVelocityStamped> path{
    {0.0, pose0, 1.0}, {2.0, pose1, 3.0}};
  const auto filtered = safety::filterPredictedPathByTimeHorizon(path, 1.0);
  ASSERT_FALSE(filtered.empty());
  EXPECT_LE(filtered.back().time, 1.0);

  geometry_msgs::Pose opposing = pose0;
  opposing.orientation.z = 1.0;
  opposing.orientation.w = 0.0;
  EXPECT_TRUE(safety::isTargetObjectOncoming(pose0, opposing));
}
