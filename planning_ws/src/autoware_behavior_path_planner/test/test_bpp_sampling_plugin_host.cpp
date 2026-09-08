#include <autoware/behavior_path_planner/behavior_path_planner_core.hpp>

#include "ros1_side_shift_fixture.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>

namespace
{
void setSamplingParameters(ros::NodeHandle & node)
{
  node.setParam("constraints/hard/max_curvature", 3.0);
  node.setParam("constraints/hard/min_curvature", -3.0);
  node.setParam("constraints/soft/lateral_deviation_weight", 1.0);
  node.setParam("constraints/soft/length_weight", 1.0);
  node.setParam("constraints/soft/curvature_weight", 2000.0);
  node.setParam("constraints/soft/weights", std::vector<double>{0.5, 1.0, 20.0});
  node.setParam("sampling/enable_frenet", true);
  node.setParam("sampling/enable_bezier", false);
  node.setParam("sampling/resolution", 0.5);
  node.setParam("sampling/previous_path_reuse_points_nb", 2);
  node.setParam("sampling/nb_target_lateral_positions", 0);
  node.setParam("sampling/target_lengths", std::vector<double>{20.0, 40.0});
  node.setParam("sampling/target_lateral_positions", std::vector<double>{-4.5, -2.5, 0.0, 2.5, 4.5});
  node.setParam("sampling/frenet/target_lateral_velocities", std::vector<double>{-0.5, 0.0, 0.5});
  node.setParam("sampling/frenet/target_lateral_accelerations", std::vector<double>{0.0});
  node.setParam("preprocessing/force_zero_initial_deviation", true);
  node.setParam("preprocessing/force_zero_initial_heading", true);
  node.setParam("preprocessing/smooth_reference_trajectory", false);
  node.setParam("wheel_radius", 0.5);
  node.setParam("wheel_width", 0.3);
  node.setParam("wheel_base", 2.7);
  node.setParam("wheel_tread", 1.6);
  node.setParam("front_overhang", 1.0);
  node.setParam("rear_overhang", 1.1);
  node.setParam("left_overhang", 0.2);
  node.setParam("right_overhang", 0.2);
  node.setParam("vehicle_height", 1.8);
  node.setParam("max_steer_angle", 0.6);
}
}  // namespace

TEST(BehaviorPathPlannerCore, RequiresExplicitSlotForRequestedSamplingManager)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char ** argv = nullptr;
    ros::init(argc, argv, "bpp_sampling_plugin_host");
  }
  ros::NodeHandle node("bpp_sampling_plugin_host");
  aw149::test::loadSideShiftFixture(node);
  setSamplingParameters(node);
  node.setParam("launch_modules", std::vector<std::string>{
    "autoware::behavior_path_planner::SamplingPlannerModuleManager"});

  node.deleteParam("slots");
  EXPECT_THROW(autoware::behavior_path_planner::BehaviorPathPlannerCore{&node}, std::runtime_error);
  node.setParam("slots", std::vector<std::string>{"slot1"});
  node.setParam("slot1", std::vector<std::string>{"not_sampling_planner"});
  EXPECT_THROW(autoware::behavior_path_planner::BehaviorPathPlannerCore{&node}, std::runtime_error);
  node.setParam("slot1", std::vector<std::string>{"sampling_planner"});
  autoware::behavior_path_planner::BehaviorPathPlannerCore core(&node);
  ASSERT_TRUE(core.isInitialized());
  ASSERT_EQ(core.loadedPluginManagerNames().size(), 1U);
  EXPECT_EQ(core.loadedPluginManagerNames().front(), "sampling_planner");
  EXPECT_TRUE(core.hasConfiguredPluginSlot());
  // Explicit configuration grants execution eligibility, not automatic
  // activation without map/input data and the module's request predicate.
  EXPECT_EQ(core.activeModuleCount(), 0U);
}
