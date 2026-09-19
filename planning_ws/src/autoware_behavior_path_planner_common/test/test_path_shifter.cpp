#include <autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp>
#include <autoware/behavior_path_planner_common/interface/data_readiness.hpp>
#include <autoware/behavior_path_planner_common/interface/bpp_input_core.hpp>
#include <autoware/behavior_path_planner_common/parameters.hpp>
#include <autoware/behavior_path_planner_common/data_types.hpp>
#include <autoware/behavior_path_planner_common/planner_data.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_lifecycle.hpp>
#include <autoware/behavior_path_planner_common/turn_signal_resolver.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_core.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_interface.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_manager_core.hpp>
#include <autoware/behavior_path_planner_common/interface/scene_module_manager_ros1.hpp>
#include <autoware/behavior_path_planner_common/interface/planner_manager_slot_core.hpp>
#include <autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp>
#include <autoware/behavior_path_planner_common/utils/object_distance.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/velocity_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/map_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/dynamic_area_core.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/path_projection.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering_core.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>
#include <autoware/behavior_path_planner_common/marker_utils/utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>

#include <gtest/gtest.h>
#include <boost/geometry/algorithms/correct.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <cmath>

namespace
{
autoware_internal_planning_msgs::PathWithLaneId makePath()
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  for (int index = 0; index < 10; ++index) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(index);
    point.point.pose.orientation.w = 1.0;
    path.points.push_back(point);
  }
  return path;
}

lanelet::Lanelet makeLanelet(
  const lanelet::Id id, const double first_x, const double last_x,
  const std::string & left_bound_type = "none")
{
  lanelet::LineString3d left(
    id * 10 + 1,
    {lanelet::Point3d(id * 10 + 2, first_x, 1.0, 0.0),
     lanelet::Point3d(id * 10 + 3, last_x, 1.0, 0.0)});
  left.attributes()[lanelet::AttributeName::Type] = left_bound_type;
  const lanelet::LineString3d right(
    id * 10 + 4,
    {lanelet::Point3d(id * 10 + 5, first_x, -1.0, 0.0),
     lanelet::Point3d(id * 10 + 6, last_x, -1.0, 0.0)});
  return lanelet::Lanelet(id, left, right);
}

builtin_interfaces::Time toBuiltinTime(const ros::Time & time)
{
  builtin_interfaces::Time stamp;
  stamp.sec = static_cast<int32_t>(time.sec);
  stamp.nanosec = time.nsec;
  return stamp;
}
}  // namespace

TEST(PathShifter, RejectsMissingAndTooShortReference)
{
  autoware::behavior_path_planner::PathShifter shifter;
  autoware::behavior_path_planner::ShiftedPath output;
  EXPECT_FALSE(shifter.generate(&output));
  shifter.setPath(makePath());
  autoware::behavior_path_planner::ShiftLine line;
  line.start.position.x = 1.0;
  line.end.position.x = 2.0;
  line.end_shift_length = 1.0;
  shifter.setShiftLines({line});
  EXPECT_FALSE(shifter.generate(&output));
}

TEST(PathShifter, SignedBrakingChangesSplineAndRejectsStoppingBeforeEnd)
{
  using namespace autoware::behavior_path_planner;
  auto path = makePath();
  for (auto & point : path.points) point.point.pose.position.x *= 5;
  PathShifter shifter;
  shifter.setPath(path);
  ShiftLine line;
  line.start = path.points.front().point.pose;
  line.end = path.points.back().point.pose;
  line.end_shift_length = 3.5;
  shifter.setShiftLines({line});
  shifter.setVelocity(20);
  shifter.setLateralAccelerationLimit(6.5);
  ShiftedPath coasting, braking;
  shifter.setLongitudinalAcceleration(0);
  ASSERT_TRUE(shifter.generate(&coasting));
  shifter.setLongitudinalAcceleration(-3);
  ASSERT_TRUE(shifter.generate(&braking));
  EXPECT_GT(std::abs(braking.path.points.at(3).point.pose.position.y -
                     coasting.path.points.at(3).point.pose.position.y), 0.01);
  EXPECT_NEAR(braking.path.points.back().point.pose.position.y, 3.5, 1e-6);
  shifter.setLongitudinalAcceleration(-6);
  EXPECT_THROW(shifter.generate(&braking), std::logic_error);
}

TEST(StartCommonUtilities, PreservesVectorNormAndStopLaneIdIntersection)
{
  namespace utils = autoware::behavior_path_planner::utils;
  geometry_msgs::Vector3 vector;
  vector.x = 3.0;
  vector.y = 4.0;
  vector.z = 12.0;
  EXPECT_DOUBLE_EQ(utils::l2Norm(vector), 13.0);
  auto path = makePath();
  for (auto & point : path.points) {
    point.point.longitudinal_velocity_mps = 5.0;
    point.lane_ids = {10, 20};
  }
  path.points[4].lane_ids = {20, 30};
  const auto stop = utils::insertStopPoint(3.5, path);
  EXPECT_DOUBLE_EQ(stop.point.pose.position.x, 3.5);
  EXPECT_EQ(stop.lane_ids, std::vector<int64_t>({20}));
  ASSERT_EQ(path.points.size(), 11U);
  for (size_t i = 4; i < path.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(path.points[i].point.longitudinal_velocity_mps, 0.0);
  }
}

TEST(StartCommonUtilities, PreservesDecelerationSignAndBelowTargetPolicies)
{
  namespace utils = autoware::behavior_path_planner::utils;
  auto data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  auto odom = boost::make_shared<nav_msgs::Odometry>();
  auto acceleration = boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  data->self_odometry = odom;
  data->self_acceleration = acceleration;
  odom->twist.twist.linear.x = 1.0;
  EXPECT_THROW(utils::calc_feasible_decel_distance(data, 1.0, -1.0, 0.0), std::invalid_argument);
  EXPECT_FALSE(utils::calc_feasible_decel_distance(data, -1.0, -1.0, 2.0));
  const auto departure_distance = utils::parking_departure::calcFeasibleDecelDistance(
    data, 1.0, -1.0, 2.0);
  ASSERT_TRUE(departure_distance);
  EXPECT_DOUBLE_EQ(*departure_distance, 0.0);
  autoware_internal_planning_msgs::PathWithLaneId empty;
  EXPECT_FALSE(utils::insert_feasible_stop_point(empty, data, -1.0, -1.0, "obstacle"));
}

TEST(StartCommonUtilities, ShoulderAdjacencyUsesSharedBoundaryIds)
{
  namespace utils = autoware::behavior_path_planner::utils;
  auto road = makeLanelet(10, 0.0, 10.0);
  auto remote = makeLanelet(20, 0.0, 10.0);
  const lanelet::Lanelet right(30, road.rightBound(), remote.rightBound());
  const lanelet::Lanelet left(40, remote.leftBound(), road.leftBound());
  EXPECT_FALSE(utils::getRightLanelet(road, {remote}));
  const auto drivable = utils::generateDrivableLanesWithShoulderLanes({road}, {right, left});
  ASSERT_EQ(drivable.size(), 1U);
  EXPECT_EQ(drivable.front().left_lane.id(), 40);
  EXPECT_EQ(drivable.front().right_lane.id(), 30);
  ASSERT_EQ(drivable.front().middle_lanes.size(), 1U);
  EXPECT_EQ(drivable.front().middle_lanes.front().id(), 10);
}

TEST(BppTrafficSignal, MissingStateOrGreenSignalDoesNotRequestStop)
{
  using autoware::behavior_path_planner::PlannerData;
  using autoware::behavior_path_planner::utils::traffic_light::isTrafficSignalStop;
  ros::Time::init();

  const lanelet::LineString3d left(
    1, {lanelet::Point3d(2, 0.0, 1.0, 0.0), lanelet::Point3d(3, 10.0, 1.0, 0.0)});
  const lanelet::LineString3d right(
    4, {lanelet::Point3d(5, 0.0, -1.0, 0.0), lanelet::Point3d(6, 10.0, -1.0, 0.0)});
  lanelet::Lanelet lane(7, left, right);
  const lanelet::LineString3d stop_line(
    8, {lanelet::Point3d(9, 5.0, 1.0, 0.0), lanelet::Point3d(10, 5.0, -1.0, 0.0)});
  const lanelet::LineString3d traffic_light(
    21, {lanelet::Point3d(22, 5.0, 1.0, 3.0), lanelet::Point3d(23, 5.0, -1.0, 3.0)});
  lane.addRegulatoryElement(lanelet::TrafficLight::make(42, {}, {traffic_light}, stop_line));
  const lanelet::ConstLanelets lanes{lane};

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->parameters.traffic_light_signal_timeout = 10.0;
  EXPECT_FALSE(isTrafficSignalStop(lanes, planner_data));

  autoware_perception_msgs::TrafficLightGroupArray green;
  green.stamp = toBuiltinTime(ros::Time::now());
  autoware_perception_msgs::TrafficLightGroup group;
  group.traffic_light_group_id = 42;
  autoware_perception_msgs::TrafficLightElement lamp;
  lamp.shape = autoware_perception_msgs::TrafficLightElement::CIRCLE;
  lamp.color = autoware_perception_msgs::TrafficLightElement::GREEN;
  group.elements.push_back(lamp);
  green.traffic_light_groups.push_back(group);
  planner_data->updateTrafficSignals(green);
  EXPECT_FALSE(isTrafficSignalStop(lanes, planner_data));
}

TEST(BppInputCore, AcceptsFirstAndMaterialLateralOffsetChangesOnly)
{
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  auto first = boost::make_shared<tier4_planning_msgs::LateralOffset>();
  first->stamp = toBuiltinTime(ros::Time(1.0));
  first->lateral_offset = 0.5F;
  EXPECT_TRUE(autoware::behavior_path_planner::BppInputCore::updateLateralOffset(planner_data, first));
  EXPECT_EQ(planner_data->lateral_offset, first);
  auto same = boost::make_shared<tier4_planning_msgs::LateralOffset>(*first);
  same->stamp = toBuiltinTime(ros::Time(2.0));
  same->lateral_offset = 0.50001F;
  EXPECT_FALSE(autoware::behavior_path_planner::BppInputCore::updateLateralOffset(planner_data, same));
  EXPECT_EQ(planner_data->lateral_offset, first);
  auto changed = boost::make_shared<tier4_planning_msgs::LateralOffset>(*first);
  changed->stamp = toBuiltinTime(ros::Time(3.0));
  changed->lateral_offset = -0.25F;
  EXPECT_TRUE(autoware::behavior_path_planner::BppInputCore::updateLateralOffset(planner_data, changed));
  EXPECT_EQ(planner_data->lateral_offset, changed);
  EXPECT_FALSE(autoware::behavior_path_planner::BppInputCore::updateLateralOffset({}, changed));
}

TEST(BppTrafficSignal, FreshRedSignalRequestsStopAndStaleSignalFailsClosedToNoDecision)
{
  using autoware::behavior_path_planner::PlannerData;
  using autoware::behavior_path_planner::utils::traffic_light::isTrafficSignalStop;
  ros::Time::init();

  const lanelet::LineString3d left(
    11, {lanelet::Point3d(12, 0.0, 1.0, 0.0), lanelet::Point3d(13, 10.0, 1.0, 0.0)});
  const lanelet::LineString3d right(
    14, {lanelet::Point3d(15, 0.0, -1.0, 0.0), lanelet::Point3d(16, 10.0, -1.0, 0.0)});
  lanelet::Lanelet lane(17, left, right);
  const lanelet::LineString3d stop_line(
    18, {lanelet::Point3d(19, 5.0, 1.0, 0.0), lanelet::Point3d(20, 5.0, -1.0, 0.0)});
  const lanelet::LineString3d traffic_light(
    24, {lanelet::Point3d(25, 5.0, 1.0, 3.0), lanelet::Point3d(26, 5.0, -1.0, 3.0)});
  lane.addRegulatoryElement(lanelet::TrafficLight::make(43, {}, {traffic_light}, stop_line));
  const lanelet::ConstLanelets lanes{lane};

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->parameters.traffic_light_signal_timeout = 10.0;
  autoware_perception_msgs::TrafficLightGroupArray red;
  red.stamp = toBuiltinTime(ros::Time::now());
  autoware_perception_msgs::TrafficLightGroup group;
  group.traffic_light_group_id = 43;
  autoware_perception_msgs::TrafficLightElement lamp;
  lamp.shape = autoware_perception_msgs::TrafficLightElement::CIRCLE;
  lamp.color = autoware_perception_msgs::TrafficLightElement::RED;
  group.elements.push_back(lamp);
  red.traffic_light_groups.push_back(group);
  planner_data->updateTrafficSignals(red);
  EXPECT_TRUE(isTrafficSignalStop(lanes, planner_data));

  planner_data->parameters.traffic_light_signal_timeout = 0.0;
  EXPECT_FALSE(isTrafficSignalStop(lanes, planner_data));
}

TEST(BppRouteUtils, PreservesSignedDistanceAlongLaneletCenterline)
{
  const lanelet::LineString3d left(
    101, {lanelet::Point3d(102, 0.0, 1.0, 0.0), lanelet::Point3d(103, 10.0, 1.0, 0.0)});
  const lanelet::LineString3d right(
    104, {lanelet::Point3d(105, 0.0, -1.0, 0.0), lanelet::Point3d(106, 10.0, -1.0, 0.0)});
  const lanelet::ConstLanelets lanes{lanelet::Lanelet(107, left, right)};
  geometry_msgs::Pose current;
  geometry_msgs::Pose goal;
  current.position.x = 2.0;
  goal.position.x = 8.0;
  EXPECT_NEAR(
    autoware::behavior_path_planner::utils::getSignedDistance(current, goal, lanes), 6.0, 1e-6);
  EXPECT_NEAR(
    autoware::behavior_path_planner::utils::getSignedDistance(goal, current, lanes), -6.0, 1e-6);
}

TEST(DrivableAreaMapUtils, FiltersTypedSegmentsWithinStrictRange)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  lanelet::LaneletMap map;
  lanelet::LineString3d curb(
    501, {lanelet::Point3d(502, 0.0, 2.0, 0.0), lanelet::Point3d(503, 5.0, 2.0, 0.0)});
  curb.attributes()[lanelet::AttributeName::Type] = "curbstone";
  curb.attributes()[lanelet::AttributeName::Subtype] = "low";
  lanelet::LineString3d far_curb(
    504, {lanelet::Point3d(505, 0.0, 30.0, 0.0), lanelet::Point3d(506, 5.0, 30.0, 0.0)});
  far_curb.attributes()[lanelet::AttributeName::Type] = "curbstone";
  lanelet::LineString3d ignored(
    507, {lanelet::Point3d(508, 0.0, 1.0, 0.0), lanelet::Point3d(509, 5.0, 1.0, 0.0)});
  ignored.attributes()[lanelet::AttributeName::Type] = "road_border";
  map.add(curb);
  map.add(far_curb);
  map.add(ignored);

  expansion::DrivableAreaExpansionParameters params;
  params.max_path_arc_length = 3.0;
  params.avoid_linestring_types.emplace_back("curbstone.low");
  geometry_msgs::Point ego;
  const auto result = expansion::extract_uncrossable_segments(map, ego, params);

  EXPECT_TRUE(expansion::has_types(curb, params.avoid_linestring_types));
  EXPECT_FALSE(expansion::has_types(far_curb, {expansion::DrivableAreaExpansionParameters::LinestringType{"curbstone.low"}}));
  EXPECT_EQ(result.size(), 1U);
}

TEST(DrivableAreaFootprints, PreservesStaticVelocityBoundaryAndPoseFootprint)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  geometry_msgs::Twist stopped;
  EXPECT_TRUE(autoware::behavior_path_planner::utils::path_safety_checker::filter::velocity_filter(stopped, -1e-6, 0.5));
  EXPECT_FALSE(autoware::behavior_path_planner::utils::path_safety_checker::filter::velocity_filter(stopped, 0.0, 0.5));
  autoware_perception_msgs::PredictedObjects objects;
  autoware_perception_msgs::PredictedObject object;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  objects.objects.push_back(object);
  expansion::DrivableAreaExpansionParameters params;
  params.object_exclusion.exclude_static = true;
  params.object_exclusion.stopped_obj_vel_th = 0.5;
  params.object_exclusion.front_offset = 1.0;
  const auto footprints = expansion::create_object_footprints(objects, params);
  ASSERT_EQ(footprints.size(), 1U);
  EXPECT_NEAR(footprints.front().outer().front().x(), 8.0, 1e-9);
  EXPECT_NEAR(footprints.front().outer().front().y(), 1.0, 1e-9);
}

TEST(BppObjectFilteringCore, PreservesVelocityPositionRadiusAndVehicleClassificationBoundaries)
{
  namespace filter = autoware::behavior_path_planner::utils::path_safety_checker::filter;
  namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;
  geometry_msgs::Twist twist; twist.linear.x = 2.0;
  EXPECT_TRUE(filter::velocity_filter(twist, 1.0, 3.0)); EXPECT_FALSE(filter::velocity_filter(twist, 2.0, 3.0));
  geometry_msgs::Point origin, near; near.x = 1.0;
  EXPECT_TRUE(filter::is_within_circle(near, origin, 2.0)); EXPECT_FALSE(filter::is_within_circle(near, origin, 1.0));
  autoware_perception_msgs::ObjectClassification car, pedestrian;
  car.label = autoware_perception_msgs::ObjectClassification::CAR; pedestrian.label = autoware_perception_msgs::ObjectClassification::PEDESTRIAN;
  EXPECT_TRUE(filter::is_vehicle(car)); EXPECT_FALSE(filter::is_vehicle(pedestrian));
  autoware_perception_msgs::PredictedObjects objects; objects.objects.resize(2);
  objects.objects[0].kinematics.initial_twist_with_covariance.twist.linear.x = 0.5;
  objects.objects[1].kinematics.initial_twist_with_covariance.twist.linear.x = 2.0;
  EXPECT_EQ(safety::filter_objects_by_velocity(objects, 1.0, true).objects.size(), 1U);
  objects.objects[0].kinematics.initial_pose_with_covariance.pose.position.x = 1.0;
  objects.objects[1].kinematics.initial_pose_with_covariance.pose.position.x = 3.0;
  safety::filter_objects_within_radius(objects, origin, 2.0);
  ASSERT_EQ(objects.objects.size(), 1U);
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> path(3);
  for (size_t index = 0; index < path.size(); ++index) path[index].point.pose.position.x = static_cast<double>(index);
  EXPECT_TRUE(filter::position_filter(objects.objects.front(), path, origin, 2.0, -1.0));
  autoware_perception_msgs::PredictedObjects classified; classified.objects.resize(2);
  classified.objects[0].classification.push_back(car); classified.objects[1].classification.push_back(pedestrian);
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck types;
  types.check_car = true; types.check_pedestrian = false;
  EXPECT_TRUE(safety::is_target_object_type(classified.objects[0], types));
  EXPECT_FALSE(safety::is_target_object_type(classified.objects[1], types));
  safety::filter_objects_by_class(classified, types);
  EXPECT_EQ(classified.objects.size(), 1U);
  const auto lane = makeLanelet(411, 0.0, 2.0);
  classified.objects.front().kinematics.initial_pose_with_covariance.pose.position.x = 1.0;
  classified.objects.front().kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  EXPECT_TRUE(safety::is_centroid_within_lanelets(classified.objects.front(), {lane}));
  classified.objects.front().kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  EXPECT_TRUE(safety::is_centroid_within_lanelet(classified.objects.front(), lane, M_PI_2));
  classified.objects.front().kinematics.initial_pose_with_covariance.pose.orientation.w = 0.0;
  classified.objects.front().kinematics.initial_pose_with_covariance.pose.orientation.z = 1.0;
  EXPECT_FALSE(safety::is_centroid_within_lanelet(classified.objects.front(), lane, M_PI_2));
  classified.objects.front().kinematics.initial_pose_with_covariance.pose.position.y = 3.0;
  EXPECT_FALSE(safety::is_centroid_within_lanelets(classified.objects.front(), {lane}));
}

TEST(BppObjectFilteringCore, AppliesSourceOrderVelocityClassAndPathPositionFilters)
{
  namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> path(3);
  path[0].point.pose.position.x = 0.0;
  path[1].point.pose.position.x = 10.0;
  path[2].point.pose.position.x = 20.0;
  geometry_msgs::Point origin;

  const auto make_object = [](const double x, const double velocity, const uint8_t label) {
    autoware_perception_msgs::PredictedObject object;
    object.kinematics.initial_pose_with_covariance.pose.position.x = x;
    object.kinematics.initial_twist_with_covariance.twist.linear.x = velocity;
    autoware_perception_msgs::ObjectClassification classification;
    classification.label = label;
    classification.probability = 1.0F;
    object.classification.push_back(classification);
    return object;
  };

  autoware_perception_msgs::PredictedObjects objects;
  using Classification = autoware_perception_msgs::ObjectClassification;
  objects.objects.push_back(make_object(5.0, 2.0, Classification::CAR));       // keep
  objects.objects.push_back(make_object(5.0, 0.5, Classification::CAR));       // velocity
  objects.objects.push_back(make_object(5.0, 2.0, Classification::BICYCLE));   // class
  objects.objects.push_back(make_object(15.0, 2.0, Classification::CAR));      // forward
  objects.objects.push_back(make_object(-5.0, 2.0, Classification::CAR));      // backward

  safety::ObjectsFilteringCoreParams params;
  params.ignore_object_velocity_threshold = 1.0;
  params.object_check_forward_distance = 10.0;
  params.object_check_backward_distance = 3.0;
  params.object_types_to_check.check_bicycle = false;

  const auto filtered = safety::filter_objects_for_path(objects, path, origin, params);
  ASSERT_EQ(filtered.objects.size(), 1U);
  EXPECT_DOUBLE_EQ(filtered.objects.front().kinematics.initial_pose_with_covariance.pose.position.x, 5.0);
  EXPECT_TRUE(safety::filter_objects_for_path({}, path, origin, params).objects.empty());
  EXPECT_TRUE(safety::filter_objects_for_path(objects, {}, origin, params).objects.empty());
}

TEST(BppObjectFilteringCore, SeparatesObjectsBySourceLaneletOverlapContract)
{
  namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;
  const auto lane = makeLanelet(412, 0.0, 10.0);
  const auto make_box = [](const double x) {
    autoware_perception_msgs::PredictedObject object;
    object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 2.0;
    object.shape.dimensions.y = 1.0;
    object.kinematics.initial_pose_with_covariance.pose.position.x = x;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
    return object;
  };
  autoware_perception_msgs::PredictedObjects objects;
  objects.objects = {make_box(5.0), make_box(13.0)};
  EXPECT_TRUE(safety::is_polygon_overlap_lanelet(objects.objects.front(), lane));
  EXPECT_FALSE(safety::is_polygon_overlap_lanelet(objects.objects.back(), lane));
  const auto condition = [](const auto & object, const auto & target_lane, const double) {
    return safety::is_polygon_overlap_lanelet(object, target_lane);
  };
  const auto [target_indices, other_indices] =
    safety::separate_object_indices_by_lanelets(objects, {lane}, condition);
  ASSERT_EQ(target_indices.size(), 1U);
  ASSERT_EQ(other_indices.size(), 1U);
  EXPECT_EQ(target_indices.front(), 0U);
  EXPECT_EQ(other_indices.front(), 1U);
  const auto [target_objects, other_objects] =
    safety::separate_objects_by_lanelets(objects, {lane}, condition);
  EXPECT_EQ(target_objects.objects.size(), 1U);
  EXPECT_EQ(other_objects.objects.size(), 1U);
  const auto [empty_target, empty_other] =
    safety::separate_objects_by_lanelets(objects, {}, condition);
  EXPECT_TRUE(empty_target.objects.empty());
  EXPECT_TRUE(empty_other.objects.empty());
}

TEST(BppObjectFilteringCore, AllocatesCurrentLaneObjectThroughRouteHandler)
{
  namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map); ASSERT_FALSE(map->laneletLayer.empty());
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  const auto handler = std::make_shared<autoware::route_handler::RouteHandler>(map_message);
  const auto lane = *map->laneletLayer.begin();
  const auto centerline = lane.centerline(); ASSERT_FALSE(centerline.empty());
  autoware_perception_msgs::PredictedObjects objects;
  autoware_perception_msgs::PredictedObject object;
  const auto & point = centerline[centerline.size() / 2];
  object.kinematics.initial_pose_with_covariance.pose.position.x = point.x();
  object.kinematics.initial_pose_with_covariance.pose.position.y = point.y();
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  objects.objects.push_back(object);
  safety::ObjectLaneConfiguration configuration; configuration.check_current_lane = true;
  const auto targets = safety::create_target_objects_on_lane(
    {lane}, handler, objects, configuration, false, false, 1.0, 0.5);
  EXPECT_EQ(targets.on_current_lane.size(), 1U);
  EXPECT_TRUE(targets.on_left_lane.empty()); EXPECT_TRUE(targets.on_right_lane.empty());
}

TEST(BppSafetyCore, PreservesRssStoppingFallbackAndMinimumLengthFormula)
{
  namespace safety = autoware::behavior_path_planner::utils::path_safety_checker;
  safety::RSSparams params;
  params.rear_vehicle_reaction_time = 1.0; params.rear_vehicle_safety_time_margin = 0.5;
  params.front_vehicle_deceleration = -2.0; params.rear_vehicle_deceleration = -4.0;
  EXPECT_NEAR(safety::calc_rss_distance(4.0, 6.0, params), 9.5, 1e-9);
  params.front_vehicle_deceleration = 1.0; params.rear_vehicle_deceleration = 0.0;
  EXPECT_NEAR(safety::calc_rss_distance(2.0, 3.0, params), 7.0, 1e-9);
  params.longitudinal_velocity_delta_time = 0.2; params.longitudinal_distance_min_threshold = 3.0;
  EXPECT_NEAR(safety::calc_minimum_longitudinal_length(-5.0, 4.0, params), 3.8, 1e-9);
  autoware_perception_msgs::Shape shape; shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX; shape.dimensions.x = 4.0; shape.dimensions.y = 6.0;
  EXPECT_NEAR(safety::calc_obstacle_min_length(shape), 2.0, 1e-9); EXPECT_NEAR(safety::calc_obstacle_max_length(shape), std::sqrt(13.0), 1e-9);
  shape.type = autoware_perception_msgs::Shape::POLYGON; shape.footprint.points.resize(2); shape.footprint.points[0].x = 1.0; shape.footprint.points[1].x = 3.0;
  EXPECT_NEAR(safety::calc_obstacle_min_length(shape), 1.0, 1e-9); EXPECT_NEAR(safety::calc_obstacle_max_length(shape), 3.0, 1e-9);
  geometry_msgs::Pose ego, oncoming; ego.orientation.w = 1.0; oncoming.orientation.z = 1.0;
  EXPECT_TRUE(safety::is_target_object_oncoming(ego, oncoming));
  EXPECT_FALSE(safety::is_target_object_oncoming(ego, ego));
  std::vector<safety::PoseWithVelocityStamped> predicted{{0.0, ego, 1.0}, {2.0, oncoming, 3.0}};
  const auto interpolated = safety::calc_interpolated_pose_with_velocity(predicted, 1.0);
  ASSERT_TRUE(interpolated); EXPECT_NEAR(interpolated->velocity, 2.0, 1e-9); EXPECT_NEAR(interpolated->time, 1.0, 1e-9);
  EXPECT_FALSE(safety::calc_interpolated_pose_with_velocity(predicted, 3.0));
  const auto cropped_prediction = safety::filter_predicted_path_by_time_horizon(predicted, 1.0);
  ASSERT_EQ(cropped_prediction.size(), 2U); EXPECT_NEAR(cropped_prediction.back().time, 1.0, 1e-9);
  std::vector<safety::PoseWithVelocityStamped> target_path(4);
  for (size_t index = 0; index < target_path.size(); ++index) {
    target_path[index].time = static_cast<double>(index);
    target_path[index].pose.position.x = static_cast<double>(index) * 10.0;
  }
  geometry_msgs::Pose target_pose; target_pose.position.x = 18.0;
  const auto after_target = safety::filter_predicted_path_after_target_pose(target_path, target_pose);
  ASSERT_EQ(after_target.size(), 2U); EXPECT_DOUBLE_EQ(after_target.front().time, 2.0);
  EXPECT_TRUE(safety::filter_predicted_path_after_target_pose({}, target_pose).empty());
  autoware_internal_planning_msgs::PathWithLaneId rough_path;
  rough_path.points.resize(2);
  rough_path.points[1].point.pose.position.x = 10.0;
  autoware_perception_msgs::PredictedObjects rough_objects;
  autoware_perception_msgs::PredictedObject rough_object;
  rough_object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  rough_object.shape.dimensions.x = 2.0; rough_object.shape.dimensions.y = 2.0;
  rough_object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  rough_objects.objects.push_back(rough_object);
  autoware::behavior_path_planner::BehaviorPathPlannerParameters rough_parameters;
  rough_parameters.vehicle_width = 2.0; rough_parameters.front_overhang = 2.0; rough_parameters.rear_overhang = 2.0;
  EXPECT_EQ(safety::check_objects_collision_rough(rough_path, rough_objects, 0.0, 4.0, rough_parameters, true), std::make_pair(true, true));
  rough_objects.objects.front().kinematics.initial_pose_with_covariance.pose.position.x = 50.0;
  EXPECT_EQ(safety::check_objects_collision_rough(rough_path, rough_objects, 0.0, 4.0, rough_parameters, false), std::make_pair(false, false));
  autoware_utils_geometry::Polygon2d first_polygon, overlap_polygon, separate_polygon;
  first_polygon.outer() = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}};
  overlap_polygon.outer() = {{1.0, 1.0}, {3.0, 1.0}, {3.0, 3.0}, {1.0, 3.0}, {1.0, 1.0}};
  separate_polygon.outer() = {{3.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {3.0, 4.0}, {3.0, 3.0}};
  EXPECT_TRUE(safety::check_polygons_intersect({first_polygon}, {overlap_polygon}));
  EXPECT_FALSE(safety::check_polygons_intersect({first_polygon}, {separate_polygon}));
  EXPECT_TRUE(safety::is_target_object_front(ego, overlap_polygon, 0.0));
  autoware_utils_geometry::Polygon2d behind_polygon; behind_polygon.outer() = {{-3.0, 0.0}, {-1.0, 0.0}, {-1.0, 1.0}, {-3.0, 1.0}, {-3.0, 0.0}};
  EXPECT_FALSE(safety::is_target_object_front(ego, behind_polygon, 0.0));
  autoware::vehicle_info_utils::VehicleInfo vehicle; vehicle.max_longitudinal_offset_m = 2.0; vehicle.rear_overhang_m = 1.0; vehicle.front_overhang_m = 1.0; vehicle.wheel_base_m = 1.0; vehicle.wheel_tread_m = 1.0; vehicle.left_overhang_m = 0.5; vehicle.right_overhang_m = 0.5; vehicle.vehicle_width_m = 2.0; vehicle.min_lateral_offset_m = -1.0; vehicle.max_lateral_offset_m = 1.0; vehicle.min_longitudinal_offset_m = -1.0;
  const auto moving_ego_polygon = safety::create_extended_ego_polygon(ego, vehicle, 4.0, 0.5, false);
  const auto stopped_ego_polygon = safety::create_extended_ego_polygon(ego, vehicle, 4.0, 0.5, true);
  EXPECT_FALSE(moving_ego_polygon.outer().empty()); EXPECT_FALSE(stopped_ego_polygon.outer().empty());
  const auto min_x = [](const auto & polygon) {
    return std::min_element(polygon.outer().begin(), polygon.outer().end(), [](const auto & lhs, const auto & rhs) { return lhs.x() < rhs.x(); })->x();
  };
  const auto max_x = [](const auto & polygon) {
    return std::max_element(polygon.outer().begin(), polygon.outer().end(), [](const auto & lhs, const auto & rhs) { return lhs.x() < rhs.x(); })->x();
  };
  EXPECT_GT(max_x(moving_ego_polygon), max_x(stopped_ego_polygon));
  EXPECT_LT(min_x(stopped_ego_polygon), min_x(moving_ego_polygon));
  const auto along_path_polygon = safety::create_extended_ego_polygon_along_path(
    makePath(), ego, vehicle, 4.0, 0.5, false);
  EXPECT_GT(along_path_polygon.outer().size(), moving_ego_polygon.outer().size());
  EXPECT_NEAR(max_x(along_path_polygon), 6.0, 1e-9);
  autoware_internal_planning_msgs::PathWithLaneId empty_planned_path;
  const auto fallback_polygon = safety::create_extended_ego_polygon_along_path(
    empty_planned_path, ego, vehicle, 4.0, 0.5, false);
  EXPECT_EQ(fallback_polygon.outer().size(), moving_ego_polygon.outer().size());
  autoware_perception_msgs::PredictedPath object_path; object_path.time_step.sec = 1; object_path.time_step.nanosec = 0; object_path.path.resize(2); object_path.path[0].orientation.w = object_path.path[1].orientation.w = 1.0; object_path.path[1].position.x = 2.0;
  const auto object_pose = safety::calc_object_predicted_pose(object_path, 0.5);
  ASSERT_TRUE(object_pose); EXPECT_NEAR(object_pose->position.x, 1.0, 1e-9);
  EXPECT_FALSE(safety::calc_object_predicted_pose(object_path, 3.0));
  autoware_perception_msgs::PredictedObject predicted_object; predicted_object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX; predicted_object.shape.dimensions.x = 2.0; predicted_object.shape.dimensions.y = 1.0; predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x = 2.0; predicted_object.kinematics.predicted_paths = {object_path};
  const auto transformed = safety::transform_object_predicted_paths(predicted_object, 1.0, 0.5);
  ASSERT_EQ(transformed.size(), 1U); ASSERT_EQ(transformed.front().path.size(), 3U);
  EXPECT_NEAR(transformed.front().path[1].velocity, 2.0, 1e-9); EXPECT_FALSE(transformed.front().path[1].polygon.outer().empty());
  predicted_object.object_id.uuid[0] = 17U;
  autoware_perception_msgs::ObjectClassification predicted_classification;
  predicted_classification.label = autoware_perception_msgs::ObjectClassification::CAR;
  predicted_classification.probability = 1.0F;
  predicted_object.classification = {predicted_classification};
  const auto extended = safety::transform_extended_predicted_object(predicted_object, 1.0, 0.5);
  EXPECT_EQ(extended.uuid.uuid[0], 17U); EXPECT_EQ(extended.classification.label, predicted_classification.label);
  ASSERT_EQ(extended.predicted_paths.size(), 1U); EXPECT_EQ(extended.predicted_paths.front().path.size(), 3U);
  auto candidates = extended;
  candidates.predicted_paths.push_back(extended.predicted_paths.front());
  candidates.predicted_paths[0].confidence = 0.2F;
  candidates.predicted_paths[1].confidence = 0.8F;
  EXPECT_EQ(safety::get_object_predicted_paths(candidates, false).size(), 1U);
  EXPECT_EQ(safety::get_object_predicted_paths(candidates, true).size(), 2U);
  safety::IntegralPredictedPolygonParams integral_params;
  integral_params.time_horizon = 1.0;
  std::vector<safety::PoseWithVelocityStamped> integral_ego{{0.0, ego, 0.0}, {1.0, ego, 0.0}};
  EXPECT_TRUE(safety::check_safety_with_integral_predicted_polygons(
    integral_ego, vehicle, {}, true, integral_params));
  EXPECT_FALSE(safety::check_safety_with_integral_predicted_polygons(
    integral_ego, vehicle, {extended}, true, integral_params));
  safety::PoseWithVelocityAndPolygonStamped rear_object;
  rear_object.pose.orientation.w = 1.0;
  rear_object.pose.position.x = -5.0;
  rear_object.velocity = 0.0;
  rear_object.polygon.outer() = {{-5.5, -0.5}, {-4.5, -0.5}, {-4.5, 0.5}, {-5.5, 0.5}, {-5.5, -0.5}};
  const auto extended_rear = safety::create_extended_object_polygon(rear_object, 4.0, 0.5, false);
  EXPECT_GT(max_x(extended_rear), max_x(rear_object.polygon));
  safety::ExtendedPredictedObject rss_object;
  rss_object.predicted_paths.push_back({1.0F, {{0.0, geometry_msgs::Pose{}, 0.0,
    autoware_utils_geometry::Polygon2d{}}}});
  auto & rss_pose = rss_object.predicted_paths.front().path.front();
  rss_pose.pose.orientation.w = 1.0;
  rss_pose.pose.position.x = 5.0;
  rss_pose.polygon.outer() = {{4.5, -0.5}, {5.5, -0.5}, {5.5, 0.5}, {4.5, 0.5}, {4.5, -0.5}};
  boost::geometry::correct(rss_pose.polygon);
  safety::RSSparams rss_params;
  rss_params.rear_vehicle_reaction_time = 1.0;
  rss_params.rear_vehicle_deceleration = -2.0;
  rss_params.front_vehicle_deceleration = -2.0;
  rss_params.longitudinal_distance_min_threshold = 1.0;
  rss_params.lateral_distance_max_threshold = 0.1;
  std::vector<safety::PoseWithVelocityStamped> rss_ego{{0.0, ego, 3.0}, {1.0, ego, 3.0}};
  EXPECT_FALSE(safety::check_safety_with_rss_rectangle(
    rss_ego, vehicle, {rss_object}, rss_params, true, 1.0, M_PI_2));
  rss_params.extended_polygon_policy = "along_path";
  EXPECT_FALSE(safety::check_safety_with_rss_rectangle(
    rss_ego, vehicle, {rss_object}, rss_params, true, 1.0, M_PI_2));
  EXPECT_FALSE(safety::check_safety_with_rss(
    makePath(), rss_ego, vehicle, {rss_object}, rss_params, true, 1.0, M_PI_2));
  safety::CollisionCheckDebugMap debug_map;
  EXPECT_FALSE(safety::check_safety_with_rss(
    makePath(), rss_ego, vehicle, {rss_object}, debug_map, rss_params, true, 1.0, M_PI_2));
  ASSERT_EQ(debug_map.size(), 1U);
  const auto & collision_debug = debug_map.begin()->second;
  EXPECT_FALSE(collision_debug.is_safe);
  EXPECT_EQ(collision_debug.unsafe_reason, "overlap_extended_polygon");
  EXPECT_EQ(collision_debug.ego_predicted_path.size(), rss_ego.size());
  EXPECT_EQ(collision_debug.object_predicted_path.size(), 1U);
  EXPECT_FALSE(collision_debug.extended_ego_polygon.outer().empty());
  EXPECT_FALSE(collision_debug.extended_object_polygon.outer().empty());
}

TEST(DynamicAreaCore, PreservesWidthFormulaAndBidirectionalRateLimit)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  expansion::DrivableAreaExpansionParameters params;
  params.vehicle_info.front_overhang_m = 1.0; params.vehicle_info.vehicle_width_m = 2.0; params.vehicle_info.wheel_base_m = 3.0;
  EXPECT_NEAR(expansion::calculate_minimum_lane_width(10.0, params), 60.0 / 22.0, 1e-9);
  std::vector<geometry_msgs::Point> bound(3); bound[1].x = 1.0; bound[2].x = 2.0;
  std::vector<double> distances{2.0, 0.0, 0.0};
  expansion::apply_bound_change_rate_limit(distances, bound, 0.5);
  EXPECT_NEAR(distances[1], 1.5, 1e-9); EXPECT_NEAR(distances[2], 1.0, 1e-9);
}

TEST(DynamicAreaCore, FinalExpansionFailsClosedWithoutRuntimeInputs)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  expansion::PathWithLaneId path; path.points.resize(3); path.left_bound.resize(2); path.right_bound.resize(2);
  for (size_t index = 0; index < path.points.size(); ++index) path.points[index].point.pose.position.x = static_cast<double>(index);
  path.left_bound[0].y = path.left_bound[1].y = 2.0; path.right_bound[0].y = path.right_bound[1].y = -2.0;
  const auto original_left = path.left_bound;
  expansion::expand_drivable_area(path, std::make_shared<autoware::behavior_path_planner::PlannerData>());
  EXPECT_EQ(path.left_bound, original_left);
}

TEST(DynamicAreaCore, FinalExpansionConsumesMapObjectsAndUpdatesPoseCache)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_message);
  const auto lane = *map->laneletLayer.begin();
  const auto centerline = lane.centerline();
  ASSERT_GE(centerline.size(), 3U);
  expansion::PathWithLaneId path;
  for (const size_t index : {0UL, centerline.size() / 2, centerline.size() - 1}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x = centerline[index].x(); point.point.pose.position.y = centerline[index].y(); point.point.pose.position.z = centerline[index].z(); point.point.pose.orientation.w = 1.0;
    path.points.push_back(point);
    geometry_msgs::Point left = point.point.pose.position, right = point.point.pose.position;
    left.y += 3.0; right.y -= 3.0; path.left_bound.push_back(left); path.right_bound.push_back(right);
  }
  auto odometry = boost::make_shared<nav_msgs::Odometry>(); odometry->pose.pose = path.points.front().point.pose;
  planner_data->self_odometry = odometry;
  planner_data->dynamic_object = boost::make_shared<autoware_perception_msgs::PredictedObjects>();
  auto & params = planner_data->drivable_area_expansion_parameters;
  params.resample_interval = 1.0; params.max_reuse_deviation = 1.0; params.min_bound_interval = 0.1; params.curvature_average_window = 1; params.max_bound_rate = 1.0; params.arc_length_range = 1.0; params.max_expansion_distance = 1.0;
  params.vehicle_info.front_overhang_m = 1.0; params.vehicle_info.vehicle_width_m = 2.0; params.vehicle_info.wheel_base_m = 3.0;
  expansion::expand_drivable_area(path, planner_data);
  EXPECT_FALSE(planner_data->drivable_area_expansion_prev_path_poses.empty());
  EXPECT_EQ(planner_data->drivable_area_expansion_prev_path_poses.size(), planner_data->drivable_area_expansion_prev_curvatures.size());
  EXPECT_GE(path.left_bound.size(), 3U); EXPECT_GE(path.right_bound.size(), 3U);
}

TEST(DynamicAreaCore, ResamplesAndCropsFreshPathPoseCache)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  expansion::PathWithLaneId path; path.points.resize(3);
  for (size_t index = 0; index < path.points.size(); ++index) {
    path.points[index].point.pose.position.x = static_cast<double>(index);
    path.points[index].point.pose.orientation.w = 1.0;
  }
  expansion::DrivableAreaExpansionParameters params; params.resample_interval = 0.5; params.max_path_arc_length = 1.2;
  std::vector<geometry_msgs::Pose> poses; std::vector<double> curvatures{1.0};
  expansion::reuse_previous_poses(path, poses, curvatures, geometry_msgs::Point{}, params);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_NEAR(poses.front().position.x, 0.0, 1e-9);
  EXPECT_NEAR(poses.back().position.x, 1.0, 1e-9);
  EXPECT_TRUE(curvatures.empty());
}

TEST(DynamicAreaCore, ReusesCompatiblePathPoseCacheAndCurvatures)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  expansion::PathWithLaneId path; path.points.resize(3); path.left_bound.resize(2); path.right_bound.resize(2);
  for (size_t index = 0; index < path.points.size(); ++index) {
    path.points[index].point.pose.position.x = static_cast<double>(index);
    path.points[index].point.pose.orientation.w = 1.0;
  }
  path.left_bound[0].y = path.left_bound[1].y = 2.0; path.left_bound[1].x = 2.0;
  path.right_bound[0].y = path.right_bound[1].y = -2.0; path.right_bound[1].x = 2.0;
  std::vector<geometry_msgs::Pose> poses(3);
  for (size_t index = 0; index < poses.size(); ++index) { poses[index].position.x = static_cast<double>(index); poses[index].orientation.w = 1.0; }
  std::vector<double> curvatures{0.1, 0.2, 0.3};
  expansion::DrivableAreaExpansionParameters params; params.resample_interval = 0.5; params.max_reuse_deviation = 0.5;
  expansion::reuse_previous_poses(path, poses, curvatures, geometry_msgs::Point{}, params);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_NEAR(poses.back().position.x, 2.0, 1e-9);
  EXPECT_EQ(curvatures, (std::vector<double>{0.1, 0.2, 0.3}));
}

TEST(DynamicAreaCore, DistributesExpansionAndCompensatesLimitedSide)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  expansion::Expansion e; e.min_lane_widths = {6.0}; e.left_bound_indexes = {0}; e.right_bound_indexes = {0};
  e.left_projections = {{{}, 2.0}}; e.right_projections = {{{}, 2.0}};
  expansion::calculate_expansion_distances(e, {2.0, 2.0}, {2.0, 2.0});
  EXPECT_EQ(e.left_distances, (std::vector<double>{1.0, 1.0})); EXPECT_EQ(e.right_distances, (std::vector<double>{1.0, 1.0}));
  expansion::calculate_expansion_distances(e, {0.25, 0.25}, {3.0, 3.0});
  EXPECT_EQ(e.left_distances, (std::vector<double>{0.25, 0.25})); EXPECT_EQ(e.right_distances, (std::vector<double>{1.75, 1.75}));
}

TEST(DynamicAreaCore, CalculatesCurvatureDrivenExpansionAndBothBoundMappings)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Pose> poses(2); poses[1].position.x = 10.0;
  std::vector<geometry_msgs::Point> left(2), right(2); left[1].x = right[1].x = 10.0; left[0].y = left[1].y = 2.0; right[0].y = right[1].y = -2.0;
  expansion::DrivableAreaExpansionParameters params;
  params.vehicle_info.front_overhang_m = 1.0; params.vehicle_info.vehicle_width_m = 2.0; params.vehicle_info.wheel_base_m = 3.0;
  const auto e = expansion::calculate_expansion(poses, left, right, {0.0, 0.1}, params);
  ASSERT_EQ(e.min_lane_widths.size(), 2U);
  EXPECT_DOUBLE_EQ(e.min_lane_widths[0], 0.0);
  EXPECT_NEAR(e.min_lane_widths[1], expansion::calculate_minimum_lane_width(10.0, params), 1e-9);
  EXPECT_EQ(e.left_bound_indexes, (std::vector<size_t>{0, 0}));
  EXPECT_EQ(e.right_bound_indexes, (std::vector<size_t>{0, 0}));
}

TEST(DrivableAreaPathProjection, PreservesSegmentLineAndLinestringProjection)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  const expansion::Point2d point{2.0, 3.0}, first{0.0, 0.0}, second{4.0, 0.0};
  const auto segment = expansion::point_to_segment_projection(point, first, second);
  EXPECT_NEAR(segment.point.x(), 2.0, 1e-9); EXPECT_NEAR(segment.distance, 3.0, 1e-9);
  const auto line = expansion::point_to_line_projection(expansion::Point2d{6.0, 1.0}, first, second);
  EXPECT_NEAR(line.point.x(), 6.0, 1e-9);
  expansion::LineString2d path; path.push_back(first); path.push_back(second); path.push_back(expansion::Point2d{8.0, 0.0});
  const auto projected = expansion::point_to_linestring_projection(point, path);
  EXPECT_NEAR(projected.arc_length, 2.0, 1e-9); EXPECT_NEAR(projected.distance, 3.0, 1e-9);
}

TEST(DynamicAreaCore, InsertsBoundsAndCalculatesSmoothedCurvature)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Point> left(2), right(2); left[1].x = right[1].x = 10.0; left[0].y = left[1].y = 2.0; right[0].y = right[1].y = -2.0;
  geometry_msgs::Pose pose; pose.position.x = 5.0;
  expansion::add_bound_points(left, right, {pose}, 0.1);
  EXPECT_EQ(left.size(), 3U); EXPECT_EQ(right.size(), 3U);
  std::vector<geometry_msgs::Pose> curve(3); curve[1].position.x = 1.0; curve[2].position.x = 1.0; curve[2].position.y = 1.0;
  const auto curvatures = expansion::calculate_smoothed_curvatures(curve, 0);
  EXPECT_NEAR(curvatures[1], std::sqrt(2.0), 1e-9);
}

TEST(DynamicAreaCore, MapsBoundsAndPropagatesWithinArcLengthRange)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Pose> poses(2); poses[0].position.x = 1.0; poses[1].position.x = 9.0;
  std::vector<geometry_msgs::Point> bound(3); bound[1].x = 5.0; bound[2].x = 10.0;
  expansion::Expansion e;
  expansion::calculate_bound_index_mappings(e, poses, bound, expansion::LEFT);
  ASSERT_EQ(e.left_bound_indexes, (std::vector<size_t>{0, 1}));
  e.left_distances = {2.0, 0.0, 0.0};
  expansion::apply_arc_length_range_smoothing(e, bound, 6.0, expansion::LEFT);
  EXPECT_EQ(e.left_distances, (std::vector<double>{2.0, 0.0, 2.0}));
}

TEST(DynamicAreaCore, ExpandsBoundAwayFromProjectedPath)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Pose> poses(2); poses[1].position.x = 10.0;
  std::vector<geometry_msgs::Point> bound(2); bound[0].y = bound[1].y = 2.0; bound[1].x = 10.0;
  expansion::expand_bound(bound, poses, {1.0, 1.0});
  EXPECT_NEAR(bound[0].y, 3.0, 1e-9); EXPECT_NEAR(bound[1].y, 3.0, 1e-9);
}

TEST(DynamicAreaCore, LimitsExpansionByCorrectSideUncrossableSegment)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Point> bound(2); bound[1].x = 10.0;
  expansion::SegmentRtree segments; segments.insert({expansion::Point2d{0.0, 3.0}, expansion::Point2d{10.0, 3.0}});
  expansion::DrivableAreaExpansionParameters params; params.avoid_linestring_dist = 1.0; params.max_expansion_distance = 5.0;
  const auto limits = expansion::calculate_maximum_distance_from_segments(bound, segments, params, expansion::LEFT);
  EXPECT_EQ(limits, (std::vector<double>{2.0, 2.0}));
}

TEST(DynamicAreaCore, TreatsIntersectingUncrossableSegmentAsZeroClearance)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Point> bound(2); bound[1].x = 10.0;
  expansion::SegmentRtree segments;
  segments.insert({expansion::Point2d{5.0, 0.0}, expansion::Point2d{5.0, 2.0}});
  expansion::DrivableAreaExpansionParameters params; params.avoid_linestring_dist = 1.0; params.max_expansion_distance = 5.0;
  const auto limits = expansion::calculate_maximum_distance_from_segments(bound, segments, params, expansion::LEFT);
  EXPECT_EQ(limits, (std::vector<double>{0.0, 0.0}));
}

TEST(DynamicAreaCore, LimitsExpansionByPolygonOnCorrectSideAndByIntersection)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Point> bound(2); bound[1].x = 10.0;
  expansion::DrivableAreaExpansionParameters params; params.max_expansion_distance = 5.0;
  expansion::Polygon2d above;
  above.outer() = {{2.0, 3.0}, {4.0, 3.0}, {4.0, 5.0}, {2.0, 5.0}, {2.0, 3.0}};
  const expansion::SegmentRtree no_segments;
  EXPECT_EQ(expansion::calculate_maximum_distance(bound, no_segments, {above}, params, expansion::LEFT), (std::vector<double>{3.0, 3.0}));
  expansion::Polygon2d crossing;
  crossing.outer() = {{4.0, -1.0}, {6.0, -1.0}, {6.0, 1.0}, {4.0, 1.0}, {4.0, -1.0}};
  EXPECT_EQ(expansion::calculate_maximum_distance(bound, no_segments, {crossing}, params, expansion::LEFT), (std::vector<double>{0.0, 0.0}));
}

TEST(DynamicAreaCore, RemovesSelfIntersectingBoundLoop)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Pose> poses(2); poses[1].position.x = 1.0;
  std::vector<geometry_msgs::Point> bound(4); bound[1].x=2.0; bound[1].y=2.0; bound[2].y=2.0; bound[3].x=2.0;
  expansion::expand_bound(bound, poses, {0.0,0.0,0.0,0.0});
  EXPECT_EQ(bound.size(), 1U);  // ROS2 removes the loop including its last segment endpoint.
}

TEST(DynamicAreaCore, PreservesAdjacentCurveSegmentsWithoutExpansion)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  std::vector<geometry_msgs::Pose> poses(2);
  poses[1].position.x = 10.0;
  // Both turning directions must retain shared endpoints (not self intersections).
  for (const double direction : {-1.0, 1.0}) {
    std::vector<geometry_msgs::Point> bound(13);
    for (size_t i = 0; i < bound.size(); ++i) {
      bound[i].x = 12.0 * std::sin(0.1 * i);
      bound[i].y = direction * 12.0 * (1.0 - std::cos(0.1 * i));
    }
    const auto original = bound;
    expansion::expand_bound(bound, poses, std::vector<double>(bound.size(), 0.0));
    ASSERT_EQ(bound.size(), original.size());
    for (size_t i = 0; i < bound.size(); ++i) {
      EXPECT_DOUBLE_EQ(bound[i].x, original[i].x);
      EXPECT_DOUBLE_EQ(bound[i].y, original[i].y);
    }
  }
}

TEST(DynamicAreaCore, PreservesV61Lanelet200392LeftBoundary)
{
  namespace expansion = autoware::behavior_path_planner::drivable_area_expansion;
  // v61 way 100323, local UTM coordinates. This previously collapsed to 3 points.
  const std::vector<std::pair<double, double>> xy = {
    {33.565799,181.961885}, {34.044499,183.470884}, {34.492199,184.616585},
    {34.897399,186.367584}, {35.014599,188.177985}, {34.834899,189.983284},
    {34.363198,191.734985}, {33.612099,193.386485}, {32.601699,194.893185},
    {31.359099,196.215084}, {29.929899,197.302784}, {29.282099,197.681784},
    {28.613799,198.449484}};
  std::vector<geometry_msgs::Point> bound;
  for (const auto & p : xy) {
    geometry_msgs::Point point; point.x = p.first; point.y = p.second;
    bound.push_back(point);
  }
  std::vector<geometry_msgs::Pose> poses(2);
  poses.front().position = bound.front(); poses.back().position = bound.back();
  for (size_t repeat = 0; repeat < 3; ++repeat) {
    expansion::expand_bound(bound, poses, std::vector<double>(bound.size(), 0.0));
    ASSERT_EQ(bound.size(), xy.size());
    for (size_t i = 0; i < xy.size(); ++i) {
      EXPECT_DOUBLE_EQ(bound[i].x, xy[i].first);
      EXPECT_DOUBLE_EQ(bound[i].y, xy[i].second);
    }
  }
}

TEST(BppRouteUtils, FailsClosedWithoutReadyRouteMapAndPlannerInputs)
{
  using autoware::behavior_path_planner::PlannerData;
  using autoware::behavior_path_planner::utils::getCurrentLanes;
  EXPECT_TRUE(getCurrentLanes(std::make_shared<PlannerData>()).empty());

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->ensureRouteHandler();
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.position.x = 5.0;
  odometry->pose.pose.orientation.w = 1.0;
  planner_data->self_odometry = odometry;
  EXPECT_TRUE(getCurrentLanes(planner_data, 5.0, 5.0).empty());
}

TEST(BppRouteUtils, SelectsCurrentLaneFromAReadyMapAndRoute)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  ASSERT_FALSE(map->laneletLayer.empty());
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_message);
  ASSERT_TRUE(route_handler->isMapMsgReady());

  lanelet::ConstLanelets route_lanes;
  geometry_msgs::Pose start;
  geometry_msgs::Pose goal;
  for (const auto & candidate : map->laneletLayer) {
    const auto centerline = candidate.centerline();
    if (centerline.size() < 2) continue;
    const auto & first = centerline.front();
    const auto & last = centerline.back();
    start.position.x = first.x();
    start.position.y = first.y();
    start.position.z = first.z();
    goal.position.x = (first.x() + last.x()) * 0.5;
    goal.position.y = (first.y() + last.y()) * 0.5;
    goal.position.z = (first.z() + last.z()) * 0.5;
    const auto yaw = std::atan2(last.y() - first.y(), last.x() - first.x());
    start.orientation.z = std::sin(yaw * 0.5);
    start.orientation.w = std::cos(yaw * 0.5);
    goal.orientation = start.orientation;
    if (route_handler->planPathLaneletsBetweenCheckpoints(start, goal, &route_lanes)) break;
  }
  ASSERT_FALSE(route_lanes.empty());
  route_handler->setRouteLanelets(route_lanes);
  ASSERT_TRUE(route_handler->isHandlerReady());

  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->route_handler = route_handler;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = start;
  planner_data->self_odometry = odometry;
  const auto selected = autoware::behavior_path_planner::utils::getCurrentLanes(
    planner_data, 5.0, 5.0);
  ASSERT_FALSE(selected.empty());
  EXPECT_EQ(selected.front().id(), route_lanes.front().id());
}

TEST(BppRouteUtils, GeneratesCenterLinePathAndReusesPreviousRoutePath)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(
    AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  const auto map_message = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_message);

  lanelet::ConstLanelets route_lanes;
  geometry_msgs::Pose start;
  geometry_msgs::Pose goal;
  for (const auto & candidate : map->laneletLayer) {
    const auto centerline = candidate.centerline();
    if (centerline.size() < 2) continue;
    const auto & first = centerline.front();
    const auto & last = centerline.back();
    start.position.x = first.x();
    start.position.y = first.y();
    start.position.z = first.z();
    goal.position.x = (first.x() + last.x()) * 0.5;
    goal.position.y = (first.y() + last.y()) * 0.5;
    goal.position.z = (first.z() + last.z()) * 0.5;
    const auto yaw = std::atan2(last.y() - first.y(), last.x() - first.x());
    start.orientation.z = std::sin(yaw * 0.5);
    start.orientation.w = std::cos(yaw * 0.5);
    goal.orientation = start.orientation;
    if (route_handler->planPathLaneletsBetweenCheckpoints(start, goal, &route_lanes)) break;
  }
  ASSERT_FALSE(route_lanes.empty());
  route_handler->setRouteLanelets(route_lanes);

  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->route_handler = route_handler;
  planner_data->parameters.backward_path_length = 5.0;
  planner_data->parameters.forward_path_length = 5.0;
  planner_data->parameters.input_path_interval = 1.0;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = start;
  planner_data->self_odometry = odometry;

  const auto generated = autoware::behavior_path_planner::utils::calcCenterLinePath(
    planner_data, start, 0.0);
  ASSERT_FALSE(generated.points.empty());
  EXPECT_FALSE(generated.points.front().lane_ids.empty());

  const auto reused = autoware::behavior_path_planner::utils::calcCenterLinePath(
    planner_data, start, 0.0, generated);
  ASSERT_FALSE(reused.points.empty());
  EXPECT_EQ(reused.header.frame_id, route_handler->getRouteHeader().frame_id);
}

TEST(BppStaticDrivableArea, GeneratesCutsAndExpandsRouteLanes)
{
  using autoware::behavior_path_planner::utils::cutOverlappedLanes;
  using autoware::behavior_path_planner::utils::expandLanelets;
  using autoware::behavior_path_planner::utils::generateDrivableLanes;
  using autoware::behavior_path_planner::utils::getOverlappedLaneletId;

  const auto lane_one = makeLanelet(301, 0.0, 2.0, "curb");
  const auto lane_two = makeLanelet(302, 2.0, 4.0);
  const auto lane_three = makeLanelet(303, 0.0, 2.0);
  const auto lanes = generateDrivableLanes({lane_one, lane_two, lane_three});
  ASSERT_EQ(lanes.size(), 3U);
  EXPECT_EQ(lanes.front().left_lane.id(), lane_one.id());
  EXPECT_EQ(lanes.front().right_lane.id(), lane_one.id());
  ASSERT_TRUE(getOverlappedLaneletId(lanes));
  EXPECT_EQ(*getOverlappedLaneletId(lanes), 2U);

  autoware_internal_planning_msgs::PathWithLaneId path;
  for (const auto lane_id : {lane_one.id(), lane_two.id(), lane_three.id()}) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.lane_ids.push_back(lane_id);
    path.points.push_back(point);
  }
  const auto shortened = cutOverlappedLanes(path, lanes);
  ASSERT_EQ(shortened.size(), 2U);
  ASSERT_EQ(path.points.size(), 2U);
  EXPECT_EQ(path.points.front().lane_ids.front(), lane_one.id());
  EXPECT_EQ(path.points.back().lane_ids.front(), lane_two.id());

  const auto expanded = expandLanelets(shortened, 0.5, 0.5, {"curb"});
  ASSERT_EQ(expanded.size(), shortened.size());
  EXPECT_NEAR(
    expanded.front().left_lane.leftBound().front().y(),
    shortened.front().left_lane.leftBound().front().y(), 1e-9);
  EXPECT_LT(
    expanded.front().right_lane.rightBound().front().y(),
    shortened.front().right_lane.rightBound().front().y());
}

TEST(BppMarkerUtils, PreservesShiftLineMarkerIdsOffsetsAndEmptyFallback)
{
  ros::Time::init();
  autoware::behavior_path_planner::ShiftLine line;
  line.start.position.x = 1.0;
  line.start.orientation.w = 1.0;
  line.end.position.x = 5.0;
  line.end.orientation.w = 1.0;
  line.end_shift_length = 2.0;
  const auto markers = marker_utils::createShiftLineMarkerArray(
    {line}, 1.0, "side_shift_test", 0.1F, 0.2F, 0.3F, 0.4F);
  ASSERT_EQ(markers.markers.size(), 3U);
  EXPECT_EQ(markers.markers.at(0).id, 1);
  EXPECT_EQ(markers.markers.at(1).id, 2);
  EXPECT_EQ(markers.markers.at(2).id, 3);
  EXPECT_EQ(markers.markers.at(2).type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers.at(0).ns, "side_shift_test");
  EXPECT_NEAR(markers.markers.at(0).pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(markers.markers.at(1).pose.position.y, 2.0, 1e-9);
  EXPECT_NEAR(markers.markers.at(2).scale.x, 0.4, 1e-6);
  ASSERT_EQ(markers.markers.at(2).points.size(), 2U);
  EXPECT_NEAR(markers.markers.at(2).points.front().y, 1.0, 1e-9);
  EXPECT_NEAR(markers.markers.at(2).points.back().y, 2.0, 1e-9);

  const auto empty = marker_utils::createShiftLineMarkerArray(
    {}, 0.0, "empty", 1.0F, 1.0F, 1.0F, 0.1F);
  EXPECT_EQ(empty.markers.size(), 3U);
}

TEST(PathShifter, AppliesSplineShiftAndRetainsPathInvariants)
{
  autoware::behavior_path_planner::PathShifter shifter;
  shifter.setPath(makePath());
  autoware::behavior_path_planner::ShiftLine line;
  line.start.position.x = 1.0;
  line.end.position.x = 8.0;
  line.end_shift_length = 1.0;
  shifter.setShiftLines({line});
  autoware::behavior_path_planner::ShiftedPath output;
  ASSERT_TRUE(shifter.generate(&output));
  ASSERT_EQ(output.path.points.size(), 10U);
  ASSERT_EQ(output.shift_length.size(), 10U);
  EXPECT_DOUBLE_EQ(output.shift_length.front(), 0.0);
  EXPECT_DOUBLE_EQ(output.shift_length.at(8), 1.0);
  EXPECT_GT(output.path.points.at(4).point.pose.position.y, 0.0);
}

TEST(PathShifter, RetainsAccumulatedBaseOffsetAfterConsumedShift)
{
  autoware::behavior_path_planner::PathShifter shifter;
  shifter.setPath(makePath());
  autoware::behavior_path_planner::ShiftLine line;
  line.start.position.x = 1.0;
  line.end.position.x = 5.0;
  line.end_shift_length = 0.5;
  shifter.setShiftLines({line});
  shifter.removeBehindShiftLineAndSetBaseOffset(5);
  EXPECT_DOUBLE_EQ(shifter.getBaseOffset(), 0.5);
  autoware::behavior_path_planner::ShiftedPath output;
  ASSERT_TRUE(shifter.generate(&output));
  EXPECT_DOUBLE_EQ(output.shift_length.front(), 0.5);
}

TEST(PathUtils, ResamplesClipsAndPreservesStopSegments)
{
  auto path = makePath();
  const auto resampled = autoware::behavior_path_planner::utils::resamplePathWithSpline(path, 0.5);
  ASSERT_GE(resampled.points.size(), path.points.size());
  EXPECT_DOUBLE_EQ(resampled.points.front().point.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(resampled.points.back().point.pose.position.x, 9.0);
  autoware::behavior_path_planner::utils::clipPathLength(path, 5, 2.0, 2.0);
  ASSERT_EQ(path.points.size(), 7U);
  EXPECT_DOUBLE_EQ(path.points.front().point.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(path.points.back().point.pose.position.x, 8.0);
}

TEST(PathUtils, ResamplingRetainsLaneBoundaryAndForwardStop)
{
  auto path = makePath();
  for (size_t index = 0; index < path.points.size(); ++index) {
    path.points.at(index).lane_ids = {index < 5 ? 100 : 200};
    path.points.at(index).point.longitudinal_velocity_mps = index < 6 ? 3.0F : 0.0F;
  }
  const auto resampled = autoware::behavior_path_planner::utils::resamplePathWithSpline(path, 2.5);
  const auto find_x = [&](const double x) {
    return std::find_if(resampled.points.begin(), resampled.points.end(), [&](const auto & point) {
      return std::abs(point.point.pose.position.x - x) < 1e-6;
    });
  };
  const auto lane_boundary = find_x(5.0);
  ASSERT_NE(lane_boundary, resampled.points.end());
  EXPECT_EQ(lane_boundary->lane_ids, std::vector<int64_t>({200}));
  const auto stop = find_x(6.0);
  ASSERT_NE(stop, resampled.points.end());
  EXPECT_FLOAT_EQ(stop->point.longitudinal_velocity_mps, 0.0F);
}

TEST(PathUtils, SplitsDirectionChangeAndRetainsTerminalStop)
{
  auto path = makePath();
  for (size_t index = 0; index < path.points.size(); ++index) {
    path.points.at(index).point.longitudinal_velocity_mps = index < 5 ? 2.0F : -2.0F;
    if (index >= 4) path.points.at(index).point.pose.orientation.z = 1.0;
    if (index >= 4) path.points.at(index).point.pose.orientation.w = 0.0;
  }
  const auto indices = autoware::behavior_path_planner::utils::getReversingIndices(path);
  ASSERT_EQ(indices, std::vector<size_t>({4}));
  auto parts = autoware::behavior_path_planner::utils::dividePath(path, indices);
  ASSERT_EQ(parts.size(), 2U);
  autoware::behavior_path_planner::utils::correctDividedPathVelocity(parts);
  EXPECT_EQ(parts.front().points.back().point.longitudinal_velocity_mps, 0.0F);
  EXPECT_EQ(parts.back().points.back().point.longitudinal_velocity_mps, 0.0F);
  EXPECT_GT(parts.front().points.front().point.longitudinal_velocity_mps, 0.0F);
  EXPECT_LT(parts.back().points.front().point.longitudinal_velocity_mps, 0.0F);
}

TEST(PathUtils, InterpolatesAndCombinesWithoutDuplicateJoin)
{
  geometry_msgs::Pose start, end;
  start.orientation.w = 1.0; end.orientation.w = 1.0; end.position.x = 4.0;
  const auto poses = autoware::behavior_path_planner::utils::interpolatePose(start, end, 1.0);
  ASSERT_FALSE(poses.empty());
  EXPECT_GT(poses.front().position.x, 0.0);
  EXPECT_LT(poses.back().position.x, 4.0);
  auto first = makePath(); first.points.resize(5);
  auto second = makePath(); second.points.erase(second.points.begin(), second.points.begin() + 4);
  const auto combined = autoware::behavior_path_planner::utils::combinePath(first, second);
  EXPECT_EQ(combined.points.size(), 10U);
  EXPECT_DOUBLE_EQ(combined.points.at(4).point.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(combined.points.at(5).point.pose.position.x, 5.0);
}

TEST(PathUtils, RestoresUnshiftedEgoPoseFromPreviousShiftedPath)
{
  autoware::behavior_path_planner::ShiftedPath shifted;
  shifted.path = makePath();
  shifted.shift_length.assign(shifted.path.points.size(), 1.0);
  for (auto & point : shifted.path.points) point.point.pose.position.y = 1.0;
  geometry_msgs::Pose ego;
  ego.position.x = 4.0; ego.position.y = 1.0; ego.orientation.w = 1.0;
  const auto unshifted = autoware::behavior_path_planner::utils::getUnshiftedEgoPose(ego, shifted);
  EXPECT_DOUBLE_EQ(unshifted.position.x, 4.0);
  EXPECT_DOUBLE_EQ(unshifted.position.y, 0.0);
  EXPECT_DOUBLE_EQ(unshifted.orientation.w, ego.orientation.w);
  shifted.shift_length.pop_back();
  const auto fail_closed = autoware::behavior_path_planner::utils::getUnshiftedEgoPose(ego, shifted);
  EXPECT_DOUBLE_EQ(fail_closed.position.y, ego.position.y);
}

namespace
{
autoware::behavior_path_planner::BppInputState makeReadyBppInputState()
{
  autoware::behavior_path_planner::BppInputState state;
  state.route_received = true;
  state.vector_map_received = true;
  state.scenario_received = true;
  state.acceleration_received = true;
  state.operation_mode_received = true;
  state.perception_received_at_seconds = 9.5;
  state.odometry_received_at_seconds = 9.5;
  state.occupancy_grid_received_at_seconds = 9.5;
  return state;
}
}  // namespace

TEST(BppIngressReadiness, MissingMandatoryInputsPreventPlannerExecution)
{
  auto state = makeReadyBppInputState();
  state.route_received = false;
  state.perception_received_at_seconds.reset();
  const auto report = autoware::behavior_path_planner::evaluateBppInputReadiness(
    state, 10.0, 1.0, false);
  EXPECT_EQ(report.status, autoware::behavior_path_planner::DataReadyStatus::NOT_RECEIVED);
  EXPECT_FALSE(report.canRunPlanner());
  EXPECT_NE(std::find(report.not_received.begin(), report.not_received.end(), "route"),
            report.not_received.end());
  EXPECT_NE(std::find(report.not_received.begin(), report.not_received.end(), "perception_objects"),
            report.not_received.end());
}

TEST(BppIngressReadiness, CyclicTimeoutIsDiagnosedButDoesNotBlockExecution)
{
  auto state = makeReadyBppInputState();
  state.odometry_received_at_seconds = 7.0;
  const auto report = autoware::behavior_path_planner::evaluateBppInputReadiness(
    state, 10.0, 1.0, false);
  EXPECT_EQ(report.status, autoware::behavior_path_planner::DataReadyStatus::TIMEOUT);
  EXPECT_TRUE(report.canRunPlanner());
  EXPECT_EQ(report.timed_out, std::vector<std::string>({"odometry"}));
}

TEST(BppIngressReadiness, TrafficSignalIsCyclicOnlyWhenEnabled)
{
  const auto state = makeReadyBppInputState();
  const auto disabled = autoware::behavior_path_planner::evaluateBppInputReadiness(
    state, 10.0, 1.0, false);
  EXPECT_EQ(disabled.status, autoware::behavior_path_planner::DataReadyStatus::SUCCESS);
  const auto enabled = autoware::behavior_path_planner::evaluateBppInputReadiness(
    state, 10.0, 1.0, true);
  EXPECT_EQ(enabled.status, autoware::behavior_path_planner::DataReadyStatus::NOT_RECEIVED);
  EXPECT_FALSE(enabled.canRunPlanner());
  EXPECT_EQ(enabled.not_received, std::vector<std::string>({"traffic_signal"}));
}

TEST(BppParameters, PreservesPlannerAndModuleConfigurationSurface)
{
  autoware::behavior_path_planner::BehaviorPathPlannerParameters parameters;
  EXPECT_DOUBLE_EQ(parameters.traffic_light_signal_timeout, 1.0);
  parameters.vehicle_info.wheel_base_m = 2.7;
  parameters.wheel_base = parameters.vehicle_info.wheel_base_m;
  parameters.turn_signal_roundabout_on_entry = "enable";
  EXPECT_DOUBLE_EQ(parameters.wheel_base, 2.7);
  EXPECT_EQ(parameters.turn_signal_roundabout_on_entry, "enable");
  autoware::behavior_path_planner::ModuleConfigParameters module_config;
  EXPECT_FALSE(module_config.enable_module);
  EXPECT_FALSE(module_config.enable_rtc);
}

TEST(BppOccupancyCollision, PreservesGridTransformsAndUnknownCellPolicy)
{
  nav_msgs::OccupancyGrid grid;
  grid.info.width = 40; grid.info.height = 40; grid.info.resolution = 0.25;
  grid.info.origin.position.x = 1.0; grid.info.origin.position.y = 1.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(1600, 0);
  grid.data.at(28 * grid.info.width + 28) = 100;
  autoware::behavior_path_planner::OccupancyGridMapParam param;
  param.theta_size = 8; param.obstacle_threshold = 1;
  param.vehicle_shape = {4.0, 2.0, 0.0};
  autoware::behavior_path_planner::OccupancyGridBasedCollisionDetector detector;
  detector.setParam(param);
  const auto local = autoware::behavior_path_planner::global2local(grid, [] {
    geometry_msgs::Pose pose; pose.position.x = 2.0; pose.position.y = 2.0; pose.orientation.w = 1.0; return pose;
  }());
  EXPECT_DOUBLE_EQ(local.position.x, 1.0);
  EXPECT_DOUBLE_EQ(local.position.y, 1.0);
  EXPECT_EQ(autoware::behavior_path_planner::discretize_angle(M_PI * 0.25, 8), 1);
  detector.setMap(grid);
  EXPECT_TRUE(detector.detectCollision({24, 24, 0}, true));
  EXPECT_TRUE(detector.detectCollision({-100, -100, 0}, true));
  EXPECT_FALSE(detector.detectCollision({-100, -100, 0}, false));
}

TEST(BppOccupancyCollision, DetectsObstacleAlongPath)
{
  nav_msgs::OccupancyGrid grid;
  grid.info.width = 40; grid.info.height = 40; grid.info.resolution = 0.25;
  grid.info.origin.orientation.w = 1.0; grid.data.assign(1600, 0);
  grid.data.at(28 * grid.info.width + 28) = 100;
  autoware::behavior_path_planner::OccupancyGridMapParam param;
  param.theta_size = 8; param.obstacle_threshold = 1; param.vehicle_shape = {0.2, 0.2, 0.0};
  autoware::behavior_path_planner::OccupancyGridBasedCollisionDetector detector;
  detector.setParam(param); detector.setMap(grid);
  autoware_internal_planning_msgs::PathWithLaneId path;
  autoware_internal_planning_msgs::PathPointWithLaneId point;
  point.point.pose.position.x = 7.0; point.point.pose.position.y = 7.0; point.point.pose.orientation.w = 1.0;
  path.points.push_back(point);
  EXPECT_TRUE(detector.hasObstacleOnPath(path, false));
  path.points.front().point.pose.position.x = 1.0;
  EXPECT_FALSE(detector.hasObstacleOnPath(path, false));
}

namespace
{
autoware_perception_msgs::PredictedObject makeBoxObject(const double x, const double y)
{
  autoware_perception_msgs::PredictedObject object;
  object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  return object;
}

autoware_utils_geometry::LinearRing2d makeFootprint()
{
  return {{1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}};
}
}  // namespace

TEST(BppObjectDistance, PreservesNormAndEmptyObjectSentinel)
{
  geometry_msgs::Vector3 vector;
  vector.x = 1.0; vector.y = 2.0; vector.z = 2.0;
  EXPECT_DOUBLE_EQ(autoware::behavior_path_planner::utils::l2Norm(vector), 3.0);
  geometry_msgs::Pose ego; ego.orientation.w = 1.0;
  autoware_perception_msgs::PredictedObjects objects;
  EXPECT_EQ(
    autoware::behavior_path_planner::utils::calcLongitudinalDistanceFromEgoToObjects(
      ego, 1.0, -1.0, objects), std::numeric_limits<double>::max());
}

TEST(BppObjectDistance, DetectsFootprintCollisionAndKeepsFarObjectClear)
{
  geometry_msgs::Pose ego; ego.orientation.w = 1.0;
  autoware_perception_msgs::PredictedObjects objects;
  objects.objects.push_back(makeBoxObject(5.0, 0.0));
  EXPECT_FALSE(autoware::behavior_path_planner::utils::checkCollisionBetweenFootprintAndObjects(
    makeFootprint(), ego, objects, 0.1));
  objects.objects.front() = makeBoxObject(0.0, 0.0);
  EXPECT_TRUE(autoware::behavior_path_planner::utils::checkCollisionBetweenFootprintAndObjects(
    makeFootprint(), ego, objects, 0.1));
  auto path = makePath();
  path.points.resize(1);
  EXPECT_TRUE(autoware::behavior_path_planner::utils::checkCollisionBetweenPathFootprintsAndObjects(
    makeFootprint(), path, objects, 0.1));
}

TEST(BppObjectDistance, PreservesLateralAndLongitudinalEnvelopeDistances)
{
  geometry_msgs::Pose ego; ego.orientation.w = 1.0;
  const auto centered = makeBoxObject(0.0, 0.0);
  EXPECT_DOUBLE_EQ(
    autoware::behavior_path_planner::utils::calcLateralDistanceFromEgoToObject(ego, 2.0, centered),
    0.0);
  EXPECT_DOUBLE_EQ(
    autoware::behavior_path_planner::utils::calcLongitudinalDistanceFromEgoToObject(
      ego, 1.0, -1.0, centered), 0.0);
  const auto ahead = makeBoxObject(5.0, 0.0);
  EXPECT_DOUBLE_EQ(
    autoware::behavior_path_planner::utils::calcLongitudinalDistanceFromEgoToObject(
      ego, 1.0, -1.0, ahead), 3.0);
}

TEST(BppParkingDepartureVelocity, PreservesDirectionSignTerminalStopAndShortPathGuard)
{
  auto forward = makePath();
  forward.points.resize(3);
  for (auto & point : forward.points) point.point.longitudinal_velocity_mps = -2.0F;
  auto reverse = makePath();
  reverse.points.resize(3);
  reverse.points.at(0).point.pose.orientation.z = 1.0;
  reverse.points.at(0).point.pose.orientation.w = 0.0;
  reverse.points.at(1).point.pose.orientation.z = 1.0;
  reverse.points.at(1).point.pose.orientation.w = 0.0;
  reverse.points.at(2).point.pose.orientation.z = 1.0;
  reverse.points.at(2).point.pose.orientation.w = 0.0;
  for (auto & point : reverse.points) point.point.longitudinal_velocity_mps = 2.0F;
  autoware_internal_planning_msgs::PathWithLaneId short_path;
  short_path.points.resize(1);
  std::vector<autoware_internal_planning_msgs::PathWithLaneId> paths{
    forward, reverse, short_path};
  std::vector<std::pair<double, double>> terminal{{9.0, 9.0}, {9.0, 9.0}, {9.0, 9.0}};
  autoware::behavior_path_planner::utils::parking_departure::modifyVelocityByDirection(
    paths, terminal, 3.0, 1.0);
  EXPECT_FLOAT_EQ(paths.at(0).points.at(0).point.longitudinal_velocity_mps, 2.0F);
  EXPECT_FLOAT_EQ(paths.at(0).points.back().point.longitudinal_velocity_mps, 0.0F);
  EXPECT_EQ(terminal.at(0), std::make_pair(3.0, 1.0));
  EXPECT_FLOAT_EQ(paths.at(1).points.at(0).point.longitudinal_velocity_mps, -2.0F);
  EXPECT_FLOAT_EQ(paths.at(1).points.back().point.longitudinal_velocity_mps, 0.0F);
  EXPECT_EQ(terminal.at(1), std::make_pair(-3.0, -1.0));
  EXPECT_EQ(terminal.at(2), std::make_pair(0.0, 0.0));
  EXPECT_EQ(
    autoware::behavior_path_planner::utils::parking_departure::getPairsTerminalVelocityAndAccel(
      terminal, 99), std::make_pair(0.0, 0.0));
}

TEST(BppFreespacePathConversion, AssignsContainedLaneInheritsItAndPreservesDirection)
{
  autoware::freespace_planning_algorithms::PlannerWaypoints waypoints;
  waypoints.header.frame_id = "map";
  autoware::freespace_planning_algorithms::PlannerWaypoint first;
  first.pose.pose.position.x = 0.5;
  first.pose.pose.orientation.w = 1.0;
  autoware::freespace_planning_algorithms::PlannerWaypoint second = first;
  second.pose.pose.position.x = 3.0;
  autoware::freespace_planning_algorithms::PlannerWaypoint reverse = second;
  reverse.pose.pose.position.x = 4.0;
  reverse.is_back = true;
  waypoints.waypoints = {first, second, reverse};
  const auto path = autoware::behavior_path_planner::utils::convertWayPointsToPathWithLaneId(
    waypoints, 2.5, {makeLanelet(101, 0.0, 1.0)});
  ASSERT_EQ(path.points.size(), 3U);
  EXPECT_EQ(path.header.frame_id, "map");
  EXPECT_EQ(path.points.at(0).lane_ids, std::vector<int64_t>({101}));
  EXPECT_EQ(path.points.at(1).lane_ids, std::vector<int64_t>({101}));
  EXPECT_EQ(path.points.at(2).lane_ids, std::vector<int64_t>({101}));
  EXPECT_FLOAT_EQ(path.points.at(0).point.longitudinal_velocity_mps, 2.5F);
  EXPECT_FLOAT_EQ(path.points.at(2).point.longitudinal_velocity_mps, -2.5F);
}

TEST(BppManagerDataTypes, PreserveDefaultSignalsAndCandidateSentinels)
{
  geometry_msgs::Pose start, end;
  start.position.x = 1.0; end.position.x = 2.0;
  const autoware::behavior_path_planner::TurnSignalInfo signal(start, end);
  EXPECT_EQ(signal.turn_signal.command, autoware_vehicle_msgs::TurnIndicatorsCommand::NO_COMMAND);
  EXPECT_EQ(signal.hazard_signal.command, autoware_vehicle_msgs::HazardLightsCommand::NO_COMMAND);
  EXPECT_DOUBLE_EQ(signal.required_start_point.position.x, 1.0);
  autoware_internal_planning_msgs::PathWithLaneId path = makePath();
  const autoware::behavior_path_planner::CandidateOutput candidate(path);
  EXPECT_EQ(candidate.path_candidate.points.size(), path.points.size());
  EXPECT_EQ(candidate.start_distance_to_path_change, std::numeric_limits<double>::lowest());
  autoware::behavior_path_planner::BehaviorModuleOutput output;
  output.path = path;
  EXPECT_EQ(output.path.points.size(), path.points.size());
}

TEST(BppPlannerData, StoresInputsAndFailsClosedForStaleOrMissingTrafficSignals)
{
  autoware::behavior_path_planner::PlannerData data;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.position.x = 3.0;
  data.self_odometry = odometry;
  EXPECT_DOUBLE_EQ(data.self_odometry->pose.pose.position.x, 3.0);
  autoware_perception_msgs::TrafficLightGroupArray signals;
  signals.stamp = toBuiltinTime(ros::Time(10.0));
  signals.traffic_light_groups.resize(1);
  signals.traffic_light_groups.front().traffic_light_group_id = 42;
  data.updateTrafficSignals(signals);
  ASSERT_TRUE(data.getTrafficSignal(42, ros::Time(10.5)));
  EXPECT_FALSE(data.getTrafficSignal(99, ros::Time(10.5)));
  data.parameters.traffic_light_signal_timeout = 0.9;
  EXPECT_FALSE(data.getTrafficSignal(42, ros::Time(11.0)));
  data.ensureRouteHandler();
  EXPECT_TRUE(data.route_handler);
}

TEST(BppSceneLifecycle, PreservesApprovalTransitionPriorityAndTerminalStates)
{
  using autoware::behavior_path_planner::ApprovalState;
  using autoware::behavior_path_planner::ModuleStatus;
  using autoware::behavior_path_planner::SceneModuleLifecycle;

  SceneModuleLifecycle module("side_shift");
  EXPECT_EQ(module.update(false, false), ModuleStatus::RUNNING);
  module.setApprovalState(ApprovalState{true, false, false});
  EXPECT_EQ(module.update(false, false), ModuleStatus::WAITING_APPROVAL);
  EXPECT_TRUE(module.isWaitingApproval());
  module.setApprovalState(ApprovalState{true, true, false});
  EXPECT_EQ(module.update(false, false), ModuleStatus::RUNNING);
  module.setApprovalState(ApprovalState{true, false, false});
  EXPECT_EQ(module.update(true, true), ModuleStatus::SUCCESS);
  EXPECT_EQ(module.update(false, true), ModuleStatus::SUCCESS);

  SceneModuleLifecycle failure("avoidance");
  EXPECT_EQ(failure.update(false, false), ModuleStatus::RUNNING);
  EXPECT_EQ(failure.update(false, true), ModuleStatus::FAILURE);
  EXPECT_EQ(failure.update(true, false), ModuleStatus::FAILURE);
}

TEST(BppSceneLifecycle, TerminatesRequestsAndExitUnlocksAllLocalState)
{
  using autoware::behavior_path_planner::ApprovalState;
  using autoware::behavior_path_planner::ModuleStatus;
  using autoware::behavior_path_planner::SceneModuleLifecycle;

  SceneModuleLifecycle module("goal_planner");
  EXPECT_EQ(module.update(false, false), ModuleStatus::RUNNING);
  module.setApprovalState(ApprovalState{true, false, true});
  EXPECT_EQ(module.update(false, false), ModuleStatus::RUNNING);
  module.waitApproval();
  module.lockNewModuleLaunch();
  module.lockOutputPath();
  module.onExit();
  EXPECT_FALSE(module.isWaitingApproval());
  EXPECT_FALSE(module.isNewModuleLaunchLocked());
  EXPECT_FALSE(module.isOutputPathLocked());
}

namespace
{
autoware::behavior_path_planner::TurnSignalInfo makeTurnSignal(
  const uint8_t command, const double desired_start, const double desired_end,
  const double required_start, const double required_end)
{
  geometry_msgs::Pose start, end;
  start.orientation.w = 1.0;
  end.orientation.w = 1.0;
  start.position.x = desired_start;
  end.position.x = desired_end;
  autoware::behavior_path_planner::TurnSignalInfo signal(start, end);
  signal.turn_signal.command = command;
  signal.required_start_point.position.x = required_start;
  signal.required_start_point.orientation.w = 1.0;
  signal.required_end_point.position.x = required_end;
  signal.required_end_point.orientation.w = 1.0;
  return signal;
}
}  // namespace

TEST(BppTurnSignalResolver, ArbitratesCandidatesAndPreservesSourcePriority)
{
  using autoware::behavior_path_planner::TurnSignalInfo;
  using autoware::behavior_path_planner::TurnSignalResolver;
  const auto path = makePath();
  geometry_msgs::Pose ego;
  ego.position.x = 5.0;
  ego.orientation.w = 1.0;
  const auto intersection = makeTurnSignal(
    autoware_vehicle_msgs::TurnIndicatorsCommand::ENABLE_RIGHT, 0.0, 9.0, 3.0, 8.0);
  auto behavior = makeTurnSignal(
    autoware_vehicle_msgs::TurnIndicatorsCommand::ENABLE_LEFT, 0.0, 9.0, 3.0, 8.0);
  TurnSignalResolver resolver;
  EXPECT_EQ(
    resolver.resolve(path, ego, 5, intersection, TurnSignalInfo{}, behavior, 3.0, 1.0).command,
    autoware_vehicle_msgs::TurnIndicatorsCommand::ENABLE_RIGHT);
}

TEST(BppTurnSignalResolver, UsesBaseLinkOffsetAndDisablesInvalidCandidates)
{
  using autoware::behavior_path_planner::TurnSignalInfo;
  using autoware::behavior_path_planner::TurnSignalResolver;
  const auto path = makePath();
  geometry_msgs::Pose ego;
  ego.orientation.w = 1.0;
  auto behavior = makeTurnSignal(
    autoware_vehicle_msgs::TurnIndicatorsCommand::ENABLE_LEFT, 1.0, 6.0, 2.0, 5.0);
  TurnSignalResolver resolver(2.0);
  EXPECT_EQ(
    resolver.resolve(path, ego, 0, TurnSignalInfo{}, TurnSignalInfo{}, behavior, 3.0, 1.0).command,
    autoware_vehicle_msgs::TurnIndicatorsCommand::ENABLE_LEFT);
  behavior.turn_signal.command = autoware_vehicle_msgs::TurnIndicatorsCommand::NO_COMMAND;
  EXPECT_EQ(
    resolver.resolve(path, ego, 0, TurnSignalInfo{}, TurnSignalInfo{}, behavior, 3.0, 1.0).command,
    autoware_vehicle_msgs::TurnIndicatorsCommand::DISABLE);
}

namespace
{
class TestSceneModuleCore final : public autoware::behavior_path_planner::SceneModuleCore
{
public:
  TestSceneModuleCore() : SceneModuleCore("test_module") {}
  using SceneModuleCore::getEgoPose;
  using SceneModuleCore::getEgoPosition;
  using SceneModuleCore::getEgoSpeed;
  using SceneModuleCore::getEgoTwist;
  bool requested{true};
  bool ready{true};
  bool success{false};
  bool failure{false};
  int update_count{0};
  int entry_count{0};
  int plan_count{0};
  int exit_count{0};
  int post_process_count{0};
  bool return_empty{false};
  int run_count{0};
  autoware::behavior_path_planner::BehaviorModuleOutput run() override
  {
    ++run_count;
    return SceneModuleCore::run();
  }

private:
  bool isExecutionRequested() const override { return requested; }
  bool isExecutionReady() const override { return ready; }
  bool canTransitSuccessState() override { return success; }
  bool canTransitFailureState() override { return failure; }
  void updateData() override { ++update_count; }
  void processOnEntry() override { ++entry_count; }
  void processOnExit() override { ++exit_count; }
  void postProcess() override { ++post_process_count; }
  autoware::behavior_path_planner::CandidateOutput planCandidate() const override
  {
    return autoware::behavior_path_planner::CandidateOutput(makePath());
  }
  autoware::behavior_path_planner::BehaviorModuleOutput plan() override
  {
    ++plan_count;
    autoware::behavior_path_planner::BehaviorModuleOutput output;
    if (return_empty) return output;
    output.path = makePath();
    return output;
  }
};

TEST(SceneModuleCoreDispatch, PreservesDerivedRunThroughBasePointer)
{
  TestSceneModuleCore module;
  autoware::behavior_path_planner::SceneModuleCore * base = &module;
  EXPECT_FALSE(base->run().path.points.empty());
  EXPECT_EQ(module.run_count, 1);
  module.return_empty = true;
  EXPECT_THROW(base->run(), std::invalid_argument);
  EXPECT_EQ(module.run_count, 2);
}

class TestSceneModuleManagerCore final
  : public autoware::behavior_path_planner::SceneModuleManagerCore
{
public:
  TestSceneModuleManagerCore() : SceneModuleManagerCore("test_manager") {}
  TestSceneModuleCore * created{nullptr};

private:
  std::unique_ptr<autoware::behavior_path_planner::SceneModuleCore>
  createNewSceneModuleInstance() override
  {
    auto module = std::make_unique<TestSceneModuleCore>();
    created = module.get();
    return module;
  }
};

class TestSceneModuleProvider final : public autoware::behavior_path_planner::SceneModuleInterface
{
public:
  TestSceneModuleProvider() : SceneModuleInterface("test_provider") {}
  explicit TestSceneModuleProvider(MarkerInterfaceMap marker_interfaces)
  : SceneModuleInterface("test_provider", {}, std::move(marker_interfaces))
  {
  }
  explicit TestSceneModuleProvider(RTCInterfaceMap rtc_interfaces)
  : SceneModuleInterface("test_provider", std::move(rtc_interfaces))
  {
  }
  bool success{false};

  void setMarkerIds(const int info_id, const int debug_id, const int drivable_id)
  {
    info_marker_.markers.resize(1);
    info_marker_.markers.front().id = info_id;
    debug_marker_.markers.resize(1);
    debug_marker_.markers.front().id = debug_id;
    drivable_lanes_marker_.markers.resize(1);
    drivable_lanes_marker_.markers.front().id = drivable_id;
  }

private:
  bool isExecutionRequested() const override { return false; }
  bool isExecutionReady() const override { return true; }
  bool canTransitSuccessState() override { return success; }
  bool canTransitFailureState() override { return false; }
  autoware::behavior_path_planner::CandidateOutput planCandidate() const override
  {
    return autoware::behavior_path_planner::CandidateOutput(makePath());
  }
  autoware::behavior_path_planner::BehaviorModuleOutput plan() override
  {
    autoware::behavior_path_planner::BehaviorModuleOutput output;
    output.path = makePath();
    return output;
  }
};

class TestSceneModuleManagerRos1 final
  : public autoware::behavior_path_planner::SceneModuleManagerRos1
{
public:
  explicit TestSceneModuleManagerRos1(ros::NodeHandle * node)
  : SceneModuleManagerRos1(node, "test_manager_ros1")
  {
  }

private:
  std::unique_ptr<autoware::behavior_path_planner::SceneModuleCore>
  createNewSceneModuleInstance() override
  {
    return std::make_unique<TestSceneModuleCore>();
  }
};

class TestSceneModuleProviderManager final
  : public autoware::behavior_path_planner::SceneModuleManagerRos1
{
public:
  TestSceneModuleProviderManager() : SceneModuleManagerRos1(nullptr, "test_provider_manager") {}

private:
  std::unique_ptr<autoware::behavior_path_planner::SceneModuleCore>
  createNewSceneModuleInstance() override
  {
    return std::make_unique<TestSceneModuleProvider>();
  }
};
}  // namespace

TEST(BppSceneModuleCore, RunsNormalAndApprovalPathsWithNonemptyInvariant)
{
  TestSceneModuleCore module;
  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), autoware::behavior_path_planner::ModuleStatus::RUNNING);
  const auto normal = module.run();
  EXPECT_EQ(normal.path.points.size(), 10U);
  EXPECT_EQ(module.update_count, 1);
  EXPECT_EQ(module.plan_count, 1);

  autoware::behavior_path_planner::BehaviorModuleOutput previous;
  previous.path = makePath();
  previous.reference_path = makePath();
  module.setPreviousModuleOutput(previous);
  module.waitApproval();
  const auto waiting = module.run();
  EXPECT_EQ(waiting.path.points.size(), previous.path.points.size());
  EXPECT_EQ(module.plan_count, 1);
  module.lockNewModuleLaunch();
  module.lockOutputPath();
  module.onExit();
  EXPECT_EQ(module.exit_count, 1);
  EXPECT_FALSE(module.isLockedNewModuleLaunch());
  EXPECT_FALSE(module.isOutputPathLocked());

  TestSceneModuleCore empty_module;
  empty_module.updateCurrentState();
  empty_module.return_empty = true;
  EXPECT_THROW(empty_module.run(), std::invalid_argument);
}

TEST(BppSceneModuleManagerCore, PreservesIdleObserverEntryAndExitOwnership)
{
  TestSceneModuleManagerCore manager;
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  manager.setData(planner_data);
  autoware::behavior_path_planner::BehaviorModuleOutput previous;
  previous.path = makePath();

  manager.updateIdleModuleInstance();
  ASSERT_NE(manager.created, nullptr);
  EXPECT_EQ(manager.created->entry_count, 0);
  manager.updateIdleModuleInstance();
  EXPECT_EQ(manager.created->entry_count, 1);
  EXPECT_TRUE(manager.isExecutionRequested(previous));
  EXPECT_EQ(manager.created->update_count, 1);
  EXPECT_TRUE(manager.canLaunchNewModule());

  auto active = std::make_shared<TestSceneModuleCore>();
  manager.registerNewModule(active, previous);
  EXPECT_EQ(manager.observerCount(), 1U);
  EXPECT_TRUE(manager.exist(active));
  EXPECT_FALSE(manager.canLaunchNewModule());
  manager.reset();
  EXPECT_EQ(active->exit_count, 1);
  EXPECT_EQ(manager.observerCount(), 0U);
}

TEST(BppSceneModuleInterface, OwnsTimeKeeperForManagerProcessingTimeWiring)
{
  TestSceneModuleProvider module;
  EXPECT_NE(module.getTimeKeeper(), nullptr);
}

TEST(BppSceneModuleInterface, PublishesItsObjectsOfInterestProviders)
{
  ros::NodeHandle node;
  TestSceneModuleProvider::MarkerInterfaceMap interfaces;
  interfaces.emplace(
    "avoidance", std::make_shared<autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface>(&node, "test_provider_avoidance"));
  TestSceneModuleProvider module(std::move(interfaces));
  EXPECT_NO_THROW(module.publishObjectsOfInterestMarker());
}

TEST(BppSceneModuleInterface, UpdatesAndLocksItsRTCProviders)
{
  ros::NodeHandle node;
  TestSceneModuleProvider::RTCInterfaceMap interfaces;
  interfaces.emplace(
    "avoidance", std::make_shared<autoware::rtc_interface::RTCInterface>(&node, "test_provider_avoidance", true));
  TestSceneModuleProvider module(std::move(interfaces));
  EXPECT_FALSE(module.hasRegisteredRTCStatus());
  module.lockRTCCommand();
  EXPECT_NO_THROW(module.updateRTCStatus(1.0, 2.0));
  module.unlockRTCCommand();
  EXPECT_TRUE(module.hasRegisteredRTCStatus());
  module.updateCurrentState();
  module.updateRTCStatus(1.0, 2.0);
  module.success = true;
  module.updateCurrentState();
  EXPECT_EQ(module.getCurrentStatus(), autoware::behavior_path_planner::ModuleStatus::SUCCESS);
  EXPECT_NO_THROW(module.onExit());
  module.removeRTCStatus();
  EXPECT_FALSE(module.hasRegisteredRTCStatus());
}

TEST(BppPlannerData, ResolvesEgoPointAndSegmentWithConfiguredSoftConstraints)
{
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.position.x = 4.2;
  odometry->pose.pose.orientation.w = 1.0;
  planner_data->self_odometry = odometry;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = M_PI_2;
  const auto path = makePath();
  EXPECT_EQ(planner_data->findEgoIndex(path.points), 4U);
  EXPECT_EQ(planner_data->findEgoSegmentIndex(path.points), 4U);
}

TEST(BppSceneModuleCore, ExposesOriginalEgoKinematicsToSceneModules)
{
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose.position.x = 3.5;
  odometry->pose.pose.orientation.w = 1.0;
  odometry->twist.twist.linear.x = -2.5;
  planner_data->self_odometry = odometry;
  TestSceneModuleCore module;
  module.setData(planner_data);
  EXPECT_DOUBLE_EQ(module.getEgoPosition().x, 3.5);
  EXPECT_DOUBLE_EQ(module.getEgoPose().position.x, 3.5);
  EXPECT_DOUBLE_EQ(module.getEgoTwist().linear.x, -2.5);
  EXPECT_DOUBLE_EQ(module.getEgoSpeed(), 2.5);
}

TEST(BppSceneModuleCore, OwnsAndResetsCandidateAndReferencePathCaches)
{
  TestSceneModuleCore module;
  autoware::behavior_path_planner::BehaviorModuleOutput previous;
  previous.path = makePath();
  previous.reference_path = makePath();
  module.setPreviousModuleOutput(previous);
  module.waitApproval();
  const auto output = module.run();
  EXPECT_EQ(output.path.points.size(), 10U);
  EXPECT_EQ(module.getPathCandidate().points.size(), 10U);
  EXPECT_EQ(module.getPathReference().points.size(), 10U);
  module.resetPathCandidate();
  module.resetPathReference();
  EXPECT_TRUE(module.getPathCandidate().points.empty());
  EXPECT_TRUE(module.getPathReference().points.empty());
}

TEST(BppSceneModuleManagerRos1, AggregatesProviderMarkersWithSourceIdOffsets)
{
  TestSceneModuleProviderManager manager;
  manager.updateIdleModuleInstance();
  const auto module = manager.takeIdleModule();
  const auto provider = std::dynamic_pointer_cast<TestSceneModuleProvider>(module);
  ASSERT_TRUE(provider);
  provider->setMarkerIds(1, 2, 3);
  manager.registerNewModule(module, {});
  const auto markers = manager.aggregateMarkers();
  ASSERT_EQ(markers.info.markers.size(), 1U);
  ASSERT_EQ(markers.debug.markers.size(), 1U);
  ASSERT_EQ(markers.drivable_lanes.markers.size(), 1U);
  EXPECT_EQ(markers.info.markers.front().id, 256);
  EXPECT_EQ(markers.debug.markers.front().id, 257);
  EXPECT_EQ(markers.drivable_lanes.markers.front().id, 258);
}

TEST(BppSceneModuleManagerRos1, BindsAndPublishesStagedProviders)
{
  ros::NodeHandle node;
  TestSceneModuleManagerRos1 manager(&node);
  EXPECT_TRUE(manager.initInterface({"lane_change", "avoidance"}));
  EXPECT_TRUE(manager.hasPlanningFactorInterface());
  EXPECT_EQ(manager.rtcInterfaceCount(), 2U);
  EXPECT_EQ(manager.markerInterfaceCount(), 2U);
  manager.publishRTCStatus();
  manager.publish_planning_factors();
  manager.publishObjectsOfInterest();

  TestSceneModuleManagerRos1 null_manager(nullptr);
  EXPECT_FALSE(null_manager.initInterface({"lane_change"}));
}

TEST(BppSceneModuleManagerRos1, WiresModuleTimeKeeperWhenRegisteringProviderModule)
{
  ros::NodeHandle node;
  TestSceneModuleManagerRos1 manager(&node);
  ASSERT_TRUE(manager.initInterface({"avoidance"}));
  auto module = std::make_shared<TestSceneModuleProvider>();
  autoware::behavior_path_planner::BehaviorModuleOutput previous;
  previous.path = makePath();
  EXPECT_NO_THROW(manager.registerNewModule(module, previous));
  EXPECT_TRUE(manager.exist(module));
}

TEST(BppPlannerManagerSlotCore, RunsTheFirstRequestingManagerAndOwnsActiveModule)
{
  auto first = std::make_shared<TestSceneModuleManagerCore>();
  auto second = std::make_shared<TestSceneModuleManagerCore>();
  autoware::behavior_path_planner::PlannerManagerSlotCore slot({first, second});
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  autoware::behavior_path_planner::BehaviorModuleOutput input;
  input.path = makePath();

  const auto output = slot.run(planner_data, input);
  ASSERT_NE(first->created, nullptr);
  EXPECT_EQ(output.path.points.size(), 10U);
  EXPECT_EQ(first->created->plan_count, 1);
  EXPECT_EQ(first->created->post_process_count, 1);
  EXPECT_EQ(second->created, nullptr);
  EXPECT_EQ(slot.activeModuleCount(), 1U);

  slot.run(planner_data, input);
  EXPECT_EQ(first->created->plan_count, 2);
  EXPECT_EQ(first->created->post_process_count, 2);

  slot.reset();
  EXPECT_EQ(slot.activeModuleCount(), 0U);
  first->updateObserver();
  EXPECT_EQ(first->observerCount(), 0U);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "bpp_common_test", ros::init_options::AnonymousName);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
