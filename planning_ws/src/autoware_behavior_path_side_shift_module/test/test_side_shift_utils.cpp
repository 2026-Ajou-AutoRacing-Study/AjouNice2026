#include <autoware/behavior_path_side_shift_module/utils.hpp>
#include <autoware/behavior_path_side_shift_module/scene_core.hpp>
#include <autoware/behavior_path_side_shift_module/legacy_manager.hpp>
#include <autoware/behavior_path_planner_common/interface/planner_manager_slot_core.hpp>
#include <autoware/behavior_path_planner_common/interface/bpp_input_core.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>

#include <gtest/gtest.h>
#include <tf2/utils.h>

namespace
{
autoware::behavior_path_planner::PathWithLaneId makePath()
{
  autoware::behavior_path_planner::PathWithLaneId path;
  for (int index = 0; index < 3; ++index) {
    autoware::behavior_path_planner::PathPointWithLaneId point;
    point.point.pose.position.x = index < 2 ? static_cast<double>(index) : 1.0;
    point.point.pose.position.y = index < 2 ? 0.0 : 1.0;
    path.points.push_back(point);
  }
  return path;
}

autoware::behavior_path_planner::PathWithLaneId makeStraightPath()
{
  autoware::behavior_path_planner::PathWithLaneId path;
  for (int index = 0; index < 16; ++index) {
    autoware::behavior_path_planner::PathPointWithLaneId point;
    point.point.pose.position.x = index;
    point.point.pose.orientation.w = 1.0;
    point.lane_ids.push_back(101);
    path.points.push_back(point);
  }
  return path;
}

lanelet::ConstLanelet makeLanelet()
{
  lanelet::LineString3d left(11, {lanelet::Point3d(12, 0.0, 1.0, 0.0), lanelet::Point3d(13, 15.0, 1.0, 0.0)});
  lanelet::LineString3d right(14, {lanelet::Point3d(15, 0.0, -1.0, 0.0), lanelet::Point3d(16, 15.0, -1.0, 0.0)});
  return lanelet::Lanelet(101, left, right);
}
}  // namespace

TEST(SideShiftUtils, SetsForwardAndTerminalOrientations)
{
  auto path = makePath();
  autoware::behavior_path_planner::setOrientation(&path);
  EXPECT_NEAR(tf2::getYaw(path.points.at(0).point.pose.orientation), 0.0, 1e-6);
  EXPECT_NEAR(tf2::getYaw(path.points.at(1).point.pose.orientation), M_PI_2, 1e-6);
  EXPECT_NEAR(tf2::getYaw(path.points.at(2).point.pose.orientation), M_PI_2, 1e-6);
  autoware::behavior_path_planner::setOrientation(nullptr);
}

TEST(SideShiftUtils, UsesNearestPathPointAndHandlesEmptyShift)
{
  autoware::behavior_path_planner::ShiftedPath shifted;
  shifted.path = makePath();
  shifted.shift_length = {0.0, 0.5, 1.0};
  geometry_msgs::Point ego;
  ego.x = 1.05;
  ego.y = 0.85;
  EXPECT_DOUBLE_EQ(autoware::behavior_path_planner::getClosestShiftLength(shifted, ego), 1.0);
  shifted.shift_length.clear();
  EXPECT_DOUBLE_EQ(autoware::behavior_path_planner::getClosestShiftLength(shifted, ego), 0.0);
}

TEST(SideShiftSceneCore, ProducesShiftedPathAndExpandedDrivableLanes)
{
  autoware::behavior_path_planner::LegacySideShiftParameters parameters;
  parameters.min_distance_to_start_shifting = 1.0;
  parameters.time_to_start_shifting = 0.0;
  parameters.shifting_lateral_jerk = 1.0;
  parameters.min_shifting_distance = 2.0;
  parameters.min_shifting_speed = 1.0;
  autoware::behavior_path_planner::SideShiftSceneCore scene(parameters);
  const auto reference = makeStraightPath();
  scene.setReferencePath(reference);
  scene.setCurrentLanelets({makeLanelet()});
  scene.requestLateralOffset(1.0);
  const auto output = scene.plan(0, 2.0);
  ASSERT_EQ(output.path.points.size(), reference.points.size());
  ASSERT_EQ(output.reference_path.points.size(), reference.points.size());
  ASSERT_EQ(output.drivable_area_info.drivable_lanes.size(), 1U);
  EXPECT_TRUE(output.drivable_area_info.is_already_expanded);
  EXPECT_GT(output.path.points.back().point.pose.position.y, 0.9);
  ASSERT_EQ(scene.shiftLines().size(), 1U);
  EXPECT_EQ(scene.shiftLines().front().start_idx, 1U);
  EXPECT_GT(scene.shiftLines().front().end_idx, scene.shiftLines().front().start_idx);
}

TEST(SideShiftSceneCore, FailsClosedWithoutReferenceOrLanesAndSupportsRightShift)
{
  autoware::behavior_path_planner::SideShiftSceneCore scene;
  scene.requestLateralOffset(-1.0);
  EXPECT_TRUE(scene.plan(0, 1.0).path.points.empty());
  const auto reference = makeStraightPath();
  scene.setReferencePath(reference);
  EXPECT_TRUE(scene.plan(0, 1.0).path.points.empty());
  scene.setCurrentLanelets({makeLanelet()});
  const auto output = scene.plan(0, 1.0);
  ASSERT_FALSE(output.path.points.empty());
  EXPECT_LT(output.path.points.back().point.pose.position.y, -0.9);
  EXPECT_TRUE(scene.plan(reference.points.size(), 1.0).path.points.empty());
}

TEST(SideShiftRegistry, RunsThroughBppSlotWithReadyRouteInputs)
{
  const auto map = autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(AW149_ROUTE_FIXTURE_PATH);
  ASSERT_TRUE(map);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(
    autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
  lanelet::ConstLanelets route_lanes;
  geometry_msgs::Pose start;
  geometry_msgs::Pose goal;
  for (const auto & lane : map->laneletLayer) {
    const auto centerline = lane.centerline();
    if (centerline.size() < 2) continue;
    const auto & first = centerline.front();
    const auto & last = centerline.back();
    start.position.x = first.x(); start.position.y = first.y(); start.position.z = first.z();
    goal.position.x = (first.x() + last.x()) * 0.5;
    goal.position.y = (first.y() + last.y()) * 0.5;
    const auto yaw = std::atan2(last.y() - first.y(), last.x() - first.x());
    start.orientation.z = std::sin(yaw * 0.5); start.orientation.w = std::cos(yaw * 0.5);
    goal.orientation = start.orientation;
    if (route_handler->planPathLaneletsBetweenCheckpoints(start, goal, &route_lanes)) break;
  }
  ASSERT_FALSE(route_lanes.empty());
  route_handler->setRouteLanelets(route_lanes);
  const auto reference = route_handler->getCenterLinePath(route_lanes, 0.0, 20.0, true);
  ASSERT_GT(reference.points.size(), 2U);
  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->route_handler = route_handler;
  planner_data->parameters.backward_path_length = 5.0;
  planner_data->parameters.forward_path_length = 10.0;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = start;
  odometry->twist.twist.linear.x = 2.0;
  planner_data->self_odometry = odometry;
  const auto current_lanes = autoware::behavior_path_planner::utils::getCurrentLanes(planner_data, 5.0, 10.0);
  ASSERT_FALSE(current_lanes.empty());
  autoware::behavior_path_planner::BehaviorModuleOutput input;
  input.path = reference;
  input.reference_path = reference;
  autoware::behavior_path_planner::LegacySideShiftParameters parameters;
  parameters.min_distance_to_start_shifting = 1.0;
  parameters.min_shifting_distance = 2.0;
  parameters.min_shifting_speed = 1.0;
  autoware::behavior_path_planner::SideShiftSceneCore direct(parameters);
  direct.setReferencePath(reference);
  direct.setCurrentLanelets(current_lanes);
  direct.requestLateralOffset(0.5);
  ASSERT_FALSE(direct.plan(0, 2.0).path.points.empty());
  auto lateral_offset = boost::make_shared<tier4_planning_msgs::LateralOffset>();
  lateral_offset->stamp.sec = 10;
  lateral_offset->stamp.nanosec = 0;
  lateral_offset->lateral_offset = 0.5F;
  ASSERT_TRUE(autoware::behavior_path_planner::BppInputCore::updateLateralOffset(
    planner_data, lateral_offset));
  auto manager = std::make_shared<autoware::behavior_path_planner::SideShiftSceneManager>(parameters);
  autoware::behavior_path_planner::PlannerManagerSlotCore slot({manager});
  const auto output = slot.run(planner_data, input);
  ASSERT_FALSE(output.path.points.empty());
  EXPECT_TRUE(output.drivable_area_info.is_already_expanded);
  EXPECT_EQ(slot.activeModuleCount(), 1U);

  // The module must keep the first generated path while the ego is within its
  // active shift interval, even when a new request arrives.  Once the end is
  // reached, the pending request becomes the next shift line.
  autoware::behavior_path_planner::SideShiftSceneModule module(parameters);
  module.setData(planner_data);
  module.setPreviousModuleOutput(input);
  module.setLateralOffset(0.5);
  const auto first_shift = module.run();
  ASSERT_FALSE(first_shift.path.points.empty());
  ASSERT_TRUE(module.latestShiftLine());
  const auto first_line = *module.latestShiftLine();
  ASSERT_LT(first_line.start_idx, first_line.end_idx);
  odometry->pose.pose = reference.points.at(first_line.start_idx).point.pose;
  module.setLateralOffset(-0.5);
  const auto mid_shift = module.run();
  ASSERT_FALSE(mid_shift.path.points.empty());
  EXPECT_DOUBLE_EQ(
    mid_shift.path.points.back().point.pose.position.x,
    first_shift.path.points.back().point.pose.position.x);
  EXPECT_DOUBLE_EQ(
    mid_shift.path.points.back().point.pose.position.y,
    first_shift.path.points.back().point.pose.position.y);
  odometry->pose.pose = reference.points.at(first_line.end_idx).point.pose;
  const auto next_shift = module.run();
  ASSERT_FALSE(next_shift.path.points.empty());
  ASSERT_TRUE(module.latestShiftLine());
  EXPECT_DOUBLE_EQ(module.latestShiftLine()->end_shift_length, -0.5);
}

TEST(SideShiftRegistry, PreservesManagerProvidersInFactoryModule)
{
  ros::NodeHandle node("side_shift_provider_factory_test");
  autoware::behavior_path_planner::SideShiftSceneManager manager(&node);
  ASSERT_TRUE(manager.initProviders({"side_shift"}, false));
  EXPECT_EQ(manager.rtcInterfaceCount(), 1U);
  EXPECT_EQ(manager.markerInterfaceCount(), 1U);
  EXPECT_TRUE(manager.hasPlanningFactorInterface());

  manager.updateIdleModuleInstance();
  const auto module = std::dynamic_pointer_cast<autoware::behavior_path_planner::SideShiftSceneModule>(
    manager.takeIdleModule());
  ASSERT_TRUE(module);
  EXPECT_EQ(module->rtcInterfaceCount(), 1U);
  EXPECT_EQ(module->markerInterfaceCount(), 1U);
  EXPECT_TRUE(module->hasPlanningFactorInterface());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "aw149_side_shift_provider_factory_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
