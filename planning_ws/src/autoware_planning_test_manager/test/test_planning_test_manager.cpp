#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <nav_msgs/Odometry.h>
#include <gtest/gtest.h>
TEST(PlanningTestManager, Communication){ros::NodeHandle n; autoware::planning_test_manager::PlanningInterfaceTestManager m; m.subscribeOutput<autoware_planning_msgs::Trajectory>("trajectory");m.normalTrajectory(n,"trajectory");m.abnormalTrajectory(n,"trajectory");m.subscribeOutput<autoware_planning_msgs::LaneletRoute>("route");m.normalRoute(n,"route");m.abnormalRoute(n,"route");m.behaviorNormalRoute(n,"route");m.behaviorGoalOnLeftRoute(n,"route");m.subscribeOutput<autoware_internal_planning_msgs::PathWithLaneId>("lane_path");m.normalPathWithLaneId(n,"lane_path");m.abnormalPathWithLaneId(n,"lane_path");m.subscribeOutput<autoware_planning_msgs::Path>("path");m.normalPath(n,"path");m.abnormalPath(n,"path");m.subscribeOutput<nav_msgs::Odometry>("odom");m.offTrackInitialPoses(n,"odom");m.offTrackOdometry(n,"odom");EXPECT_GE(m.getReceivedTopicNum(),15u);}
int main(int argc,char**argv){ros::init(argc,argv,"planning_test_manager_test");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
