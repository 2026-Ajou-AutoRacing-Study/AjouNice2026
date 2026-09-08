#include <aw149_surround_obstacle_checker/parameters.hpp>

#include <gtest/gtest.h>

TEST(SurroundObstacleCheckerRos1Parameters, LoadsCompleteSourceConfiguration)
{
  ros::NodeHandle node("/aw149_surround_obstacle_checker");
  const auto parameters = aw149::surround_obstacle_checker::Parameters::init(node);
  EXPECT_EQ(parameters.obstacle_types.size(), 9U);
  EXPECT_TRUE(parameters.obstacle_types.at("car").enable_check);
  EXPECT_DOUBLE_EQ(parameters.obstacle_types.at("car").surround_check_front_distance, 0.5);
  EXPECT_FALSE(parameters.obstacle_types.at("pointcloud").enable_check);
  EXPECT_DOUBLE_EQ(parameters.surround_check_hysteresis_distance, 0.1);
  EXPECT_DOUBLE_EQ(parameters.state_clear_time, 0.2);
  EXPECT_DOUBLE_EQ(parameters.stop_state_ego_speed, 0.1);
  EXPECT_EQ(parameters.debug_footprint_label, "car");
}

TEST(SurroundObstacleCheckerRos1Parameters, FailsClosedForMissingRequiredDistance)
{
  ros::NodeHandle node("/aw149_surround_obstacle_checker_missing");
  node.setParam("car/enable_check", true);
  EXPECT_THROW(aw149::surround_obstacle_checker::Parameters::init(node), std::runtime_error);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aw149_surround_obstacle_checker_parameter_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
