#include <autoware/path_optimizer/recovery_state.hpp>
#include <autoware/motion_utils/trajectory/recovery_collision.hpp>
#include <autoware_perception_msgs/PredictedObjects.h>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gtest/gtest.h>

using autoware::path_optimizer::RecoveryState;
namespace {
void cycle(RecoveryState & state, double time, double speed=0, bool success=false,
           bool optimizer_stop=true, bool permitted=true, bool valid=true) {
  state.observe(time,speed,valid,permitted);
  state.normalResult(time,success,optimizer_stop,permitted);
}
void activate(RecoveryState & state) {
  for(int i=0;i<=21;++i)cycle(state,i*0.1);
  ASSERT_TRUE(state.active());
}
}
TEST(RecoveryContract, RequiresStoppedAndFailedContinuously) {
  RecoveryState state;
  for(int i=0;i<50;++i)cycle(state,i*0.1,1.0);
  EXPECT_FALSE(state.active());
  for(int i=50;i<69;++i)cycle(state,i*0.1);
  EXPECT_FALSE(state.active());
  cycle(state,6.9);cycle(state,7.1);
  EXPECT_TRUE(state.active());
}
TEST(RecoveryContract, TransientFailuresAndIntentionalStopsNeverEnter) {
  RecoveryState transient,waiting;
  for(int i=0;i<100;++i) {
    cycle(transient,i*0.1,0,i%4==0);
    cycle(waiting,i*0.1,0,false,false);
  }
  EXPECT_FALSE(transient.active());EXPECT_FALSE(waiting.active());
}
TEST(RecoveryContract, StaleInputManualModeAndTimeResetBreakEntryEvidence) {
  for(int reason=0;reason<4;++reason) {
    RecoveryState state;
    for(int i=0;i<19;++i)cycle(state,i*0.1);
    if(reason==0)cycle(state,1.9,0,false,true,true,false);
    if(reason==1)cycle(state,1.9,0,false,true,false);
    if(reason==2)cycle(state,3.0);
    if(reason==3)cycle(state,0.2);
    EXPECT_FALSE(state.active());
  }
}
TEST(RecoveryContract, ActiveHasNoSpeedTimeoutOrContextExit) {
  RecoveryState state;activate(state);
  cycle(state,3,10);cycle(state,1000,0,false,true,false,false);
  state.clearEvidence();EXPECT_TRUE(state.active());
  cycle(state,1000.05,0,true,false,true,false);EXPECT_TRUE(state.active());
  cycle(state,1000.1,0,true,false,true,true);
  EXPECT_FALSE(state.active()); // A fresh normal solution hands over immediately.
}
TEST(RecoveryContract, NormalSuccessExitsEvenWhileEntryContextIsInhibited) {
  RecoveryState state;activate(state);
  // Entry authorization and current data validity have different purposes.
  // A safe normal solution hands over even during a legitimate traffic stop.
  cycle(state,20.0,0,true,false,false,true);
  EXPECT_FALSE(state.active());
}
TEST(RecoveryContract, LongGapDoesNotReplaceMinimumFailedSamples) {
  RecoveryState::Parameters p;p.maximum_sample_gap=10;p.minimum_failed_cycles=5;
  RecoveryState state(p);
  cycle(state,0);cycle(state,3);EXPECT_FALSE(state.active());
  cycle(state,3.1);cycle(state,3.2);cycle(state,3.3);EXPECT_TRUE(state.active());
}

TEST(RecoveryCollision, ChecksCrossingPredictionAgeAndMissingPrediction) {
  std::vector<autoware_planning_msgs::TrajectoryPoint> points;
  for(int i=0;i<=40;++i) {
    autoware_planning_msgs::TrajectoryPoint p;p.pose.orientation.w=1;
    p.pose.position.x=i*0.5;p.longitudinal_velocity_mps=2;points.push_back(p);
  }
  autoware::vehicle_info_utils::VehicleInfo vehicle{};
  vehicle.wheel_base_m=2.5;vehicle.front_overhang_m=0.5;vehicle.rear_overhang_m=0.5;
  vehicle.vehicle_width_m=1.8;vehicle.vehicle_height_m=1.5;
  autoware_perception_msgs::PredictedObjects objects;
  auto check=[&](double age=0) {return autoware::motion_utils::recoveryCollisionFree(
    points,points.front().pose,2.0,objects,vehicle,age);};
  EXPECT_TRUE(check());
  autoware_perception_msgs::PredictedObject object;
  object.shape.dimensions.x=2;object.shape.dimensions.y=1;object.shape.dimensions.z=1;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w=1;
  object.kinematics.initial_pose_with_covariance.pose.position.x=8;
  object.kinematics.initial_pose_with_covariance.pose.position.y=10;
  autoware_perception_msgs::PredictedPath path;path.time_step.sec=1;
  for(int i=0;i<=8;++i) {
    auto pose=object.kinematics.initial_pose_with_covariance.pose;
    pose.position.y=10-i*2.5;path.path.push_back(pose);
  }
  object.kinematics.predicted_paths.push_back(path);objects.objects.push_back(object);
  EXPECT_FALSE(check());EXPECT_FALSE(check(0.4));
  // A one-pose prediction still occupies space throughout the future horizon.
  auto & live=objects.objects.front();
  live.kinematics.initial_pose_with_covariance.pose.position.y=0;
  live.kinematics.predicted_paths.front().path={live.kinematics.initial_pose_with_covariance.pose};
  EXPECT_FALSE(check());
  // Prediction timestamps must be advanced to the current planning time.
  live.kinematics.initial_pose_with_covariance.pose.position.x=0;
  live.kinematics.initial_pose_with_covariance.pose.position.y=10;
  live.kinematics.predicted_paths.front().path.clear();
  for(int i=0;i<=5;++i) {
    auto pose=live.kinematics.initial_pose_with_covariance.pose;
    if(i==5)pose.position.y=0;
    live.kinematics.predicted_paths.front().path.push_back(pose);
  }
  EXPECT_TRUE(check());EXPECT_FALSE(check(5));
  live.kinematics.initial_pose_with_covariance.pose.position.x=8;
  objects.objects.front().kinematics.predicted_paths.clear();
  EXPECT_TRUE(check());
  objects.objects.front().kinematics.initial_twist_with_covariance.twist.linear.y=2;
  EXPECT_FALSE(check());
  objects.objects.front().kinematics.initial_twist_with_covariance.twist.linear.y=0;
  objects.objects.front().kinematics.initial_pose_with_covariance.pose.position.y=0;
  EXPECT_FALSE(check());
  live.shape.type=1;live.shape.dimensions.y=0;
  live.kinematics.initial_pose_with_covariance.pose.position.y=10;
  EXPECT_TRUE(check());
  live.kinematics.initial_pose_with_covariance.pose.position.y=0;
  EXPECT_FALSE(check());
  live.shape.type=2;live.shape.dimensions.x=0;
  for(const auto & xy:std::vector<std::pair<float,float>>{{-1,-0.5},{1,-0.5},{1,0.5},{-1,0.5}}) {
    geometry_msgs::Point32 p;p.x=xy.first;p.y=xy.second;live.shape.footprint.points.push_back(p);
  }
  EXPECT_FALSE(check());
  live.kinematics.initial_pose_with_covariance.pose.position.y=10;
  EXPECT_TRUE(check());
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
