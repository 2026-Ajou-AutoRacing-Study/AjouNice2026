#include <autoware/planning_validator_intersection_collision_checker/parameters_ros1.hpp>

#include <gtest/gtest.h>

namespace icc = autoware::planning_validator::intersection_collision_checker_ros1;

TEST(ParametersRos1, LoadsCompleteRenderedTree)
{
  ros::NodeHandle nh("/icc_r004");
  icc::Params parameters{};
  std::string error;
  ASSERT_TRUE(icc::load(nh, parameters, error)) << error;
  EXPECT_TRUE(parameters.enable);
  EXPECT_DOUBLE_EQ(parameters.detection_range, 50.0);
  EXPECT_EQ(parameters.pointcloud.velocity_estimation.buffer_size, 10);
  EXPECT_TRUE(parameters.right_turn.check_traffic_signal);
}

TEST(ParametersRos1, MissingTreeFailsClosed)
{
  ros::NodeHandle nh("/icc_r004_missing");
  icc::Params parameters{};
  std::string error;
  EXPECT_FALSE(icc::load(nh, parameters, error));
  EXPECT_NE(error.find("missing required ICC parameter"), std::string::npos);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "icc_r004_parameters_oracle");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
