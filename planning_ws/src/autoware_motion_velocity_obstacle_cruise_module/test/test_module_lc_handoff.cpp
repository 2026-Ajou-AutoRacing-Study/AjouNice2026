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
class LaneChangeFrontProtection : public testing::Test {
protected:
 ros::NodeHandle node{"/lc_front_protection"};
 ObstacleCruiseModule module;
 std::shared_ptr<PlannerData> data=std::make_shared<PlannerData>();
 std::vector<TrajectoryPoint> raw;
 void SetUp() override {
  ros::Time::setNow(ros::Time(100));
  node.setParam("obstacle_cruise/option/planning_algorithm","pid_base");
  node.setParam("obstacle_cruise/cruise_planning/pid_based_planner/error_function_type","quadratic");
  node.setParam("obstacle_cruise/cruise_planning/pid_based_planner/use_velocity_limit_based_planner",true);
  module.init(node,"obstacle_cruise");
  data->vehicle_info_=autoware::vehicle_info_utils::createVehicleInfo(
    .39,.23,3,1.651,.845,.790,.1195,.1195,1.8,.6);
  data->current_odometry.pose.pose.orientation.w=1;
  data->current_odometry.twist.twist.linear.x=25;
  data->ego_nearest_dist_threshold=3;data->ego_nearest_yaw_threshold=1;
  data->trajectory_polygon_collision_check.decimate_trajectory_step_length=2;
  for(int i=0;i<151;++i) {
   TrajectoryPoint point;point.pose.position.x=i;point.pose.orientation.w=1;
   point.longitudinal_velocity_mps=25;raw.push_back(point);
  }
  LaneChangeHandoffTestPeer::approve(module,raw,false);
 }
 void object(double x,double y,double speed) {
  autoware_perception_msgs::PredictedObjects message;
  message.header.frame_id="map";message.header.stamp=ros::Time::now();
  autoware_perception_msgs::PredictedObject object;object.object_id.uuid[0]=1;
  object.shape.type=autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x=4.5;object.shape.dimensions.y=2;object.shape.dimensions.z=1.8;
  autoware_perception_msgs::ObjectClassification cls;cls.label=cls.CAR;cls.probability=1;
  object.classification.push_back(cls);
  object.kinematics.initial_pose_with_covariance.pose.position.x=x;
  object.kinematics.initial_pose_with_covariance.pose.position.y=y;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w=1;
  object.kinematics.initial_twist_with_covariance.twist.linear.x=speed;
  autoware_perception_msgs::PredictedPath prediction;prediction.confidence=1;
  prediction.time_step.nanosec=100000000;
  for(int i=0;i<=80;++i) {auto at=object.kinematics.initial_pose_with_covariance.pose;
   at.position.x+=speed*i*.1;prediction.path.push_back(at);}
  object.kinematics.predicted_paths.push_back(prediction);message.objects.push_back(object);
  data->process_predicted_objects(message);
 }
};
TEST_F(LaneChangeFrontProtection, ApprovedLcStillBoundsSpeedForAnAlreadyAlignedSlowerLeader) {
 object(20,0,10);
 const auto result=module.plan(raw,raw,data);
 ASSERT_TRUE(result.velocity_limit);
 EXPECT_LT(result.velocity_limit->max_velocity,25);
 EXPECT_EQ(result.velocity_limit->sender,"obstacle_cruise");
 EXPECT_TRUE(LaneChangeHandoffTestPeer::owns(module));
}
TEST_F(LaneChangeFrontProtection, AdjacentRearTrafficDoesNotReinstateOrdinaryFollowing) {
 object(-10,3.5,28);
 const auto result=module.plan(raw,raw,data);
 EXPECT_FALSE(result.velocity_limit);
 EXPECT_TRUE(LaneChangeHandoffTestPeer::owns(module));
}
TEST_F(LaneChangeFrontProtection, FrontProtectionLimitClearsAfterTheFrontBodyCorridorIsFree) {
 object(20,0,10);
 const auto guarded=module.plan(raw,raw,data);
 ASSERT_TRUE(guarded.velocity_limit);
 data->objects.clear();
 const auto clear=module.plan(raw,raw,data);
 EXPECT_FALSE(clear.velocity_limit);
 ASSERT_TRUE(clear.velocity_limit_clear_command);
 EXPECT_EQ(clear.velocity_limit_clear_command->sender,"obstacle_cruise");
 EXPECT_TRUE(clear.velocity_limit_clear_command->command);
}

}
int main(int argc,char **argv) {
 ros::init(argc,argv,"module_lc_handoff_test");ros::Time::setNow(ros::Time(100));
 testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
