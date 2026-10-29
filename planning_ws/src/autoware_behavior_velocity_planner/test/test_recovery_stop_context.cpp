#include <autoware/behavior_velocity_planner/recovery_stop_context.hpp>
#include <gtest/gtest.h>

using autoware::behavior_velocity_planner::hasNewBehaviorStop;
namespace {
auto path() {
  std::vector<autoware_internal_planning_msgs::PathPointWithLaneId> points;
  for (int i=0;i<=30;++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=i;p.point.longitudinal_velocity_mps=i==30?0:5;
    points.push_back(p);
  }
  return points;
}
}
TEST(RecoveryStopContext, ExistingTerminalStopDoesNotBlockRecovery) {
  auto input=path();input[8].point.longitudinal_velocity_mps=0;
  EXPECT_FALSE(hasNewBehaviorStop(input,input,geometry_msgs::Point{}));
}
TEST(RecoveryStopContext, NewlyAddedTrafficStopBlocksRecovery) {
  const auto input=path();auto output=input;
  output[8].point.longitudinal_velocity_mps=0;
  EXPECT_TRUE(hasNewBehaviorStop(input,output,geometry_msgs::Point{}));
}
TEST(RecoveryStopContext, FarStopDoesNotExplainCurrentStandstill) {
  const auto input=path();auto output=input;
  output[25].point.longitudinal_velocity_mps=0;
  EXPECT_FALSE(hasNewBehaviorStop(input,output,geometry_msgs::Point{}));
  EXPECT_TRUE(hasNewBehaviorStop(input,{},geometry_msgs::Point{}));
}
int main(int argc,char ** argv) {
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
