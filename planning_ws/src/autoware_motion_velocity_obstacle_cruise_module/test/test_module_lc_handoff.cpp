#include "obstacle_cruise_module.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
namespace autoware::motion_velocity_planner {
class LaneChangeHandoffTestPeer {
public:
 static void approve(ObstacleCruiseModule & module,const std::vector<TrajectoryPoint> & raw,bool old_limit) {
  module.lc_commitment_.header.frame_id="map";
  module.lc_commitment_.header.stamp=ros::Time::now();
  module.lc_commitment_received_=ros::Time::now();
  for(const auto & p:raw) {
   autoware_internal_planning_msgs::PathPointWithLaneId point;
   point.point.pose=p.pose;point.point.longitudinal_velocity_mps=p.longitudinal_velocity_mps;
   module.lc_commitment_.points.push_back(point);
  }
  module.need_to_clear_velocity_limit_=old_limit;
 }
 static bool owns(const ObstacleCruiseModule & module) {return module.lc_owns_longitudinal_;}
 static bool hasOldLimit(const ObstacleCruiseModule & module) {return module.need_to_clear_velocity_limit_;}
};
TEST(LaneChangeCruiseModuleHandoff, ClearsOnlyCruiseAndSuppressesItsNormalOutput) {
 ros::NodeHandle node("/ownership_module_test");
 // Select the planner explicitly so this fixture also runs on an empty ROS master.
 node.setParam("obstacle_cruise/option/planning_algorithm", "pid_base");
 node.setParam("obstacle_cruise/cruise_planning/pid_based_planner/error_function_type", "quadratic");
 ObstacleCruiseModule module;module.init(node,"obstacle_cruise");
 std::vector<TrajectoryPoint> raw;
 for(int i=0;i<100;++i) {
  TrajectoryPoint p;p.pose.position.x=i;p.pose.orientation.w=1;p.longitudinal_velocity_mps=15;raw.push_back(p);
 }
 auto data=std::make_shared<PlannerData>();data->current_odometry.pose.pose.orientation.w=1;
 LaneChangeHandoffTestPeer::approve(module,raw,true);
 const auto handoff=module.plan(raw,raw,data);
 EXPECT_TRUE(LaneChangeHandoffTestPeer::owns(module));
 EXPECT_FALSE(LaneChangeHandoffTestPeer::hasOldLimit(module));
 EXPECT_FALSE(handoff.velocity_limit);
 ASSERT_TRUE(handoff.velocity_limit_clear_command);
 EXPECT_EQ(handoff.velocity_limit_clear_command->sender,"obstacle_cruise");
 EXPECT_TRUE(handoff.velocity_limit_clear_command->command);
 const auto maintained=module.plan(raw,raw,data);
 EXPECT_FALSE(maintained.velocity_limit);
 EXPECT_FALSE(maintained.velocity_limit_clear_command);
}
}
int main(int argc,char **argv) {
 ros::init(argc,argv,"module_lc_handoff_test");ros::Time::setNow(ros::Time(100));
 testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
