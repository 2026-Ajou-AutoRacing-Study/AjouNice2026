#include "ros1_compat.hpp"

#include <gtest/gtest.h>

namespace
{
using autoware::motion_velocity_planner::ParameterMap;

TEST(Ros1ParameterMap, AppliesTypedValuesAndLeavesMissingOrMismatchedValuesUntouched)
{
  ParameterMap parameters;
  parameters.emplace("gain", XmlRpc::XmlRpcValue(2.5));
  parameters.emplace("enabled", XmlRpc::XmlRpcValue(true));
  parameters.emplace("wrong_type", XmlRpc::XmlRpcValue("not-a-double"));

  double gain = 0.0;
  bool enabled = false;
  double preserved = 3.0;
  EXPECT_TRUE(autoware_utils::update_param(parameters, "gain", gain));
  EXPECT_TRUE(autoware_utils::update_param(parameters, "enabled", enabled));
  EXPECT_FALSE(autoware_utils::update_param(parameters, "missing", preserved));
  EXPECT_FALSE(autoware_utils::update_param(parameters, "wrong_type", preserved));
  EXPECT_DOUBLE_EQ(gain, 2.5);
  EXPECT_TRUE(enabled);
  EXPECT_DOUBLE_EQ(preserved, 3.0);
}
}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
