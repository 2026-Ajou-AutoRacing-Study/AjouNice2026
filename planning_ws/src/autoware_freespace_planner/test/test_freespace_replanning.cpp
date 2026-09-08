#include <autoware_freespace_planner/freespace_planner_node.hpp>
#include <gtest/gtest.h>

#include <cstdlib>

// Friend named by the original node, used to exercise the original state
// transitions with the real Astar collision checker (no planning mock).
class TestFreespacePlanner : public testing::Test {
protected:
  void SetUp() override {
    ros::NodeHandle node("/freespace_transport_oracle");
    ASSERT_EQ(
      std::system(
        "rosparam load \"$(rospack find autoware_freespace_planner)/config/"
        "freespace_planner.ros1.yaml\" /freespace_transport_oracle"),
      0);
    for (const auto & entry : {std::pair{"wheel_radius", 0.3}, {"wheel_width", 0.2},
         {"wheel_base", 2.7}, {"wheel_tread", 1.5}, {"front_overhang", 0.9},
         {"rear_overhang", 1.0}, {"left_overhang", 0.2}, {"right_overhang", 0.2},
         {"vehicle_height", 1.6}, {"max_steer_angle", 0.6}})
      node.setParam(entry.first, entry.second);
    planner = std::make_unique<autoware::freespace_planner::FreespacePlannerNode>(node, node);
    grid = boost::make_shared<nav_msgs::OccupancyGrid>();
    grid->header.frame_id = "map";
    grid->info.resolution = 0.2;
    grid->info.width = grid->info.height = 100;
    grid->info.origin.position.x = grid->info.origin.position.y = -10;
    grid->info.origin.orientation.w = 1;
    grid->data.assign(10000, 0);
    planner->occupancy_grid_ = grid;
    planner->current_pose_.header.frame_id = "map";
    planner->current_pose_.pose.orientation.w = 1;
    planner->partial_trajectory_.points.resize(5);
    for (size_t i = 0; i < 5; ++i) {
      auto & point = planner->partial_trajectory_.points[i];
      point.pose.position.x = i;
      point.pose.orientation.w = 1;
      point.longitudinal_velocity_mps = 1;
    }
    planner->trajectory_ = planner->partial_trajectory_;
    planner->node_param_.th_obstacle_time_sec = 0.05;
    planner->node_param_.replan_when_obstacle_found = true;
    planner->node_param_.replan_when_course_out = false;
  }
  bool collision() { return planner->checkCurrentTrajectoryCollision(); }
  bool requiresPlan() { return planner->isPlanRequired(); }
  void obstacle(bool present) { grid->data[5050] = present ? 100 : 0; }
  void disableObstacleReplanning() { planner->node_param_.replan_when_obstacle_found = false; }
  void expectMovingReplanWaits() {
    planner->node_param_.replan_when_obstacle_found = false;
    planner->node_param_.replan_when_course_out = true;
    planner->node_param_.th_course_out_distance_m = 0.5;
    planner->is_new_parking_cycle_ = false;
    auto scenario = boost::make_shared<autoware_internal_planning_msgs::Scenario>();
    scenario->activating_scenarios = {autoware_internal_planning_msgs::Scenario::PARKING};
    planner->pending_scenario_ = scenario;
    planner->pending_grid_ = grid;
    planner->route_ = boost::make_shared<autoware_planning_msgs::LaneletRoute>();
    auto odometry = boost::make_shared<nav_msgs::Odometry>();
    odometry->header.frame_id = "map";
    odometry->header.stamp = ros::Time::now();
    odometry->pose.pose.orientation.w = 1;
    odometry->pose.pose.position.y = 3;
    odometry->twist.twist.linear.x = 1;
    planner->pending_odometry_.push_back(odometry);
    planner->onTimer();
    EXPECT_TRUE(planner->reset_in_progress_);
    EXPECT_TRUE(planner->trajectory_.points.empty());
    EXPECT_FALSE(planner->is_completed_);
  }
  std::unique_ptr<autoware::freespace_planner::FreespacePlannerNode> planner;
  nav_msgs::OccupancyGridPtr grid;
};
TEST_F(TestFreespacePlanner, PersistentObstacleTriggersAndClearResetsTimer) {
  EXPECT_FALSE(collision());
  obstacle(true);
  EXPECT_FALSE(collision());
  ros::WallDuration(0.08).sleep();
  EXPECT_TRUE(collision());
  EXPECT_TRUE(requiresPlan());
  disableObstacleReplanning();
  EXPECT_FALSE(requiresPlan());
  obstacle(false);
  EXPECT_FALSE(collision());
  obstacle(true);
  EXPECT_FALSE(collision());
}
TEST_F(TestFreespacePlanner, MovingVehicleWaitsBeforeCourseOutReplan) {
  expectMovingReplanWaits();
}
TEST(FreespaceUtilities, ArrivalYawWrapAcrossPiPreservesNearbyOrientation) {
  geometry_msgs::Pose target, current;
  const double target_yaw = M_PI - 0.01, current_yaw = -M_PI + 0.01;
  target.orientation.z = std::sin(target_yaw / 2);
  target.orientation.w = std::cos(target_yaw / 2);
  current.orientation.z = std::sin(current_yaw / 2);
  current.orientation.w = std::cos(current_yaw / 2);
  EXPECT_TRUE(autoware::freespace_planner::utils::is_near_target(target, current, 0.5));
  current.position.x = 1;
  EXPECT_FALSE(autoware::freespace_planner::utils::is_near_target(target, current, 0.5));
}
TEST(FreespaceUtilities, EmptyPathHasNoReversingIndices) {
  EXPECT_TRUE(autoware::freespace_planner::utils::get_reversing_indices(
    autoware_planning_msgs::Trajectory{}).empty());
}
