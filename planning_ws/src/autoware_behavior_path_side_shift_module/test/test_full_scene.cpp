#include <autoware/behavior_path_side_shift_module/manager.hpp>
#include <autoware/behavior_path_planner_common/interface/planner_manager_slot_core.hpp>
#include <autoware/behavior_path_planner_common/interface/bpp_input_core.hpp>
#include <autoware/behavior_path_planner_common/utils/route_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <pluginlib/class_loader.hpp>
#include <gtest/gtest.h>
using namespace autoware::behavior_path_planner;
TEST(FullSideShift, NativePluginAndRequestThrottle) {
  ros::NodeHandle node("/full_side_shift_test");
  pluginlib::ClassLoader<SceneModuleManagerInterface> loader("autoware_behavior_path_planner_common","autoware::behavior_path_planner::SceneModuleManagerInterface");
  auto manager=loader.createUniqueInstance("autoware::behavior_path_planner::SideShiftModuleManager");
  ASSERT_TRUE(manager->initialize(&node));manager->updateIdleModuleInstance();
  auto module=std::dynamic_pointer_cast<SideShiftModule>(manager->takeIdleModule());
  ASSERT_TRUE(module);EXPECT_TRUE(module->hasPlanningFactorInterface());
  EXPECT_FALSE(module->isExecutionRequested());EXPECT_TRUE(module->isExecutionReady());
  EXPECT_FALSE(module->isReadyForNextRequest(100.0));
  EXPECT_FALSE(module->isReadyForNextRequest(0.0,true));
  EXPECT_TRUE(module->isReadyForNextRequest(0.0,false));
  module->processOnExit();EXPECT_FALSE(module->isExecutionRequested());
}
TEST(FullSideShift, MissingConfigurationFailsInsteadOfZeroDefaults) {
  ros::NodeHandle node("/full_side_shift_missing");SideShiftModuleManager manager;
  EXPECT_ANY_THROW(manager.initialize(&node));
}
class FullSideShiftDirection:public testing::TestWithParam<double> {};
TEST_P(FullSideShiftDirection, NativeManagerRunsOriginalSceneThroughBppSlot) {
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
  planner_data->parameters.forward_path_length = 20.0;
  planner_data->parameters.input_path_interval=1.0;
  planner_data->parameters.ego_nearest_dist_threshold=3.0;
  planner_data->parameters.ego_nearest_yaw_threshold=1.57;
  auto odometry = boost::make_shared<nav_msgs::Odometry>();
  odometry->pose.pose = start;
  odometry->twist.twist.linear.x = 2.0;
  planner_data->self_odometry = odometry;
  const auto current_lanes = autoware::behavior_path_planner::utils::getCurrentLanes(planner_data, 5.0, 10.0);
  ASSERT_FALSE(current_lanes.empty());
  autoware::behavior_path_planner::BehaviorModuleOutput input;
  input.path = reference;
  input.reference_path = reference;

  ros::NodeHandle node("/full_side_shift_test");
  node.setParam("side_shift/shift_request_time_limit",0.0);
  node.setParam("side_shift/min_distance_to_start_shifting",1.0);
  node.setParam("side_shift/min_shifting_distance",2.0);
  node.setParam("side_shift/min_shifting_speed",1.0);
  node.setParam("side_shift/shifting_lateral_jerk",1.0);
  node.setParam("side_shift/publish_debug_marker",true);
  auto offset=boost::make_shared<tier4_planning_msgs::LateralOffset>();
  offset->stamp.sec=10;offset->lateral_offset=GetParam();planner_data->lateral_offset=offset;
  auto manager=std::make_shared<SideShiftModuleManager>();ASSERT_TRUE(manager->initialize(&node));
  PlannerManagerSlotCore slot({manager});
  ASSERT_FALSE(slot.run(planner_data,input).path.points.empty());
  const auto output=slot.run(planner_data,input);
  ASSERT_FALSE(output.path.points.empty());EXPECT_TRUE(output.drivable_area_info.is_already_expanded);
  EXPECT_EQ(slot.activeModuleCount(),1U);
  double max_signed=0.0;
  for(const auto & point:output.path.points) {
    const double offset=autoware::motion_utils::calcLateralOffset(reference.points,point.point.pose.position);
    max_signed=std::max(max_signed,offset*(GetParam()>0?1.0:-1.0));
    EXPECT_TRUE(std::isfinite(point.point.pose.position.x));
  }
  EXPECT_GT(max_signed,0.1);
}
INSTANTIATE_TEST_SUITE_P(LeftAndRight,FullSideShiftDirection,testing::Values(0.5,-0.5));
int main(int argc,char ** argv) {ros::init(argc,argv,"full_side_shift_tests");ros::NodeHandle owner;testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
