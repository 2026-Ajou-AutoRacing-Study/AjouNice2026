// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_lane_change_module/scene.hpp>
#include <autoware/behavior_path_lane_change_module/utils/overtake.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>

namespace bpp = autoware::behavior_path_planner;
namespace lc = bpp::utils::lane_change;
namespace safety = bpp::utils::path_safety_checker;

namespace
{
geometry_msgs::Pose pose(double x, double y = 0, double yaw = 0)
{
  geometry_msgs::Pose p;
  p.position.x=x; p.position.y=y;
  p.orientation.z=std::sin(yaw/2); p.orientation.w=std::cos(yaw/2);
  return p;
}
autoware_internal_planning_msgs::PathWithLaneId road(double y=0, double velocity=25)
{
  autoware_internal_planning_msgs::PathWithLaneId path;
  path.header.frame_id="map";
  for (double x=-100; x<=400; x+=1) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose=pose(x,y);
    p.point.longitudinal_velocity_mps=velocity;
    path.points.push_back(p);
  }
  return path;
}
auto vehicle()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.39,0.23,3.0,1.651,0.845,0.790,0.1195,0.1195,1.8,0.6);
}
safety::ExtendedPredictedObject object(double x, double y=3.5, double speed=15, int id=1)
{
  autoware_perception_msgs::PredictedObject raw;
  raw.object_id.uuid[0]=id;
  raw.shape.type=autoware_perception_msgs::Shape::BOUNDING_BOX;
  raw.shape.dimensions.x=4.5; raw.shape.dimensions.y=2; raw.shape.dimensions.z=1.8;
  autoware_perception_msgs::ObjectClassification c;
  c.label=autoware_perception_msgs::ObjectClassification::CAR; c.probability=1;
  raw.classification.push_back(c);
  raw.kinematics.initial_pose_with_covariance.pose=pose(x,y);
  raw.kinematics.initial_twist_with_covariance.twist.linear.x=speed;
  safety::ExtendedPredictedObject obj(raw);
  obj.dist_from_ego=x;
  safety::PredictedPathWithPolygon p;
  p.confidence=1;
  for (int i=0; i<=120; ++i) {
    const double t=i*.1;
    auto at=pose(x+speed*t,y);
    p.path.emplace_back(t,at,speed,autoware_utils::to_polygon2d(at,obj.shape));
  }
  obj.predicted_paths.push_back(p);
  return obj;
}
auto prediction(double speed=20, double duration=4, double acceleration=0)
{
  std::vector<safety::PoseWithVelocityStamped> p;
  for (int i=0; i<=int(duration*10); ++i) {
    double t=i*.1;
    p.emplace_back(t,pose(speed*t+.5*acceleration*t*t,3.5),speed+acceleration*t);
  }
  return p;
}
auto policy()
{
  bpp::lane_change::OvertakeParameters p;
  p.enable=true; p.assume_cooperative_npc=true;
  return p;
}
bool cooperative(const safety::ExtendedPredictedObject & obj,
  const std::vector<safety::PoseWithVelocityStamped> & ego,
  const bpp::lane_change::OvertakeParameters & p=policy())
{
  safety::CollisionCheckDebugMap debug;
  return lc::check_cooperative_rear(road(),ego,obj,vehicle(),p,1,debug);
}
class Harness : public bpp::NormalLaneChange
{
public:
  explicit Harness(const std::shared_ptr<bpp::LaneChangeParameters> & p, bool subscribe=false)
  : NormalLaneChange(p,subscribe ? bpp::LaneChangeModuleType::NORMAL :
      bpp::LaneChangeModuleType::EXTERNAL_REQUEST,autoware::route_handler::Direction::LEFT)
  { type_=bpp::LaneChangeModuleType::NORMAL; }
  using NormalLaneChange::OvertakePhase;
  using NormalLaneChange::overtake_;
  using NormalLaneChange::overtake_prepare_path_;
  using NormalLaneChange::updateOvertake;
  using NormalLaneChange::common_data_ptr_;
  using NormalLaneChange::filtered_objects_;
  using NormalLaneChange::prev_module_output_;
  using NormalLaneChange::receiveFinalTrajectory;
  using NormalLaneChange::finalVelocityPrediction;
  using NormalLaneChange::overtake_feedback_reason_;
  using NormalLaneChange::final_trajectory_sub_;
  using NormalLaneChange::activeSafetyPath;
  using NormalLaneChange::status_;
  using NormalLaneChange::abort_path_;
  using NormalLaneChange::is_colliding;
  using NormalLaneChange::find_colliding_object_if_all_paths_collide;
  using NormalLaneChange::usesCooperativePolicy;
};
struct Scene : testing::Test
{
  std::shared_ptr<bpp::LaneChangeParameters> parameters=std::make_shared<bpp::LaneChangeParameters>();
  std::shared_ptr<bpp::PlannerData> data=std::make_shared<bpp::PlannerData>();
  std::unique_ptr<Harness> module;
  void SetUp() override
  {
    ros::Time::setNow(ros::Time(100));
    parameters->overtake=policy();
    parameters->cancel.enable_on_lane_changing_phase=true;
    parameters->safety.rss_params_for_prepare.extended_polygon_policy="along_path";
    parameters->safety.rss_params.extended_polygon_policy="rectangle";
    parameters->safety.rss_params_for_prepare.lateral_distance_max_threshold=.5;
    parameters->safety.rss_params.lateral_distance_max_threshold=1;
    parameters->safety.rss_params_for_prepare.front_vehicle_deceleration=-6;
    parameters->safety.rss_params_for_prepare.rear_vehicle_deceleration=-6;
    parameters->safety.rss_params.front_vehicle_deceleration=-6;
    parameters->safety.rss_params.rear_vehicle_deceleration=-6;
    parameters->safety.rss_params.rear_vehicle_reaction_time=2;
    parameters->trajectory.max_longitudinal_acc=3.7;
    data->parameters.vehicle_info=vehicle();
    data->parameters.vehicle_width=1.89; data->parameters.vehicle_length=4.635;
    data->parameters.base_link2front=3.845; data->parameters.base_link2rear=.790;
    data->parameters.max_acc=3.7; data->parameters.min_acc=-6; data->parameters.max_vel=25;
    data->parameters.ego_nearest_dist_threshold=3;
    data->parameters.ego_nearest_yaw_threshold=1;
    auto odom=boost::make_shared<nav_msgs::Odometry>();
    odom->pose.pose=pose(0); odom->twist.twist.linear.x=15;
    data->self_odometry=odom;
    data->self_acceleration=boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
    module=std::make_unique<Harness>(parameters);
    module->setData(data);
    module->common_data_ptr_->current_lanes_path=road();
    module->common_data_ptr_->target_lanes_path=road(3.5);
    module->common_data_ptr_->lanes_polygon_ptr->current = {
      {-101,-1.75}, {401,-1.75}, {401,1.75}, {-101,1.75}};
    module->common_data_ptr_->transient_data.dist_to_terminal_start=1000;
    module->prev_module_output_.path=road();
    module->filtered_objects_.target_lane_leading.moving.push_back(object(5));
  }
  bpp::LaneChangePath candidate()
  {
    bpp::LaneChangePath path;
    path.type=bpp::lane_change::PathType::SpatialSpeedProfile;
    path.path=road(0,20);
    path.info.lane_changing_start=pose(0);
    path.info.lane_changing_end=pose(80);
    path.info.duration={0,4};
    return path;
  }
  void feedback(double speed=20)
  {
    auto message=boost::make_shared<autoware_planning_msgs::Trajectory>();
    message->header.frame_id="map"; message->header.stamp=ros::Time::now();
    for(double x=-5;x<=150;x+=1) {
      autoware_planning_msgs::TrajectoryPoint point;
      point.pose=pose(x); point.longitudinal_velocity_mps=speed;
      message->points.push_back(point);
    }
    module->receiveFinalTrajectory(message);
  }
};
}

TEST(CooperativeRear, TenMetresAreBetweenBodies)
{
  EXPECT_NEAR(lc::rear_body_gap(road(),pose(0),object(-10).initial_polygon,vehicle()),6.96,1e-6);
  EXPECT_FALSE(cooperative(object(-10),prediction()));
  EXPECT_TRUE(cooperative(object(-14),prediction()));
}
TEST(CooperativeRear, ClosingSpeedIncreasesRequiredDistance)
{
  EXPECT_DOUBLE_EQ(lc::cooperative_rear_distance(20,15,policy()),10);
  EXPECT_GT(lc::cooperative_rear_distance(60/3.6,80/3.6,policy()),20);
  EXPECT_FALSE(cooperative(object(-14,3.5,22),prediction(16.7)));
}
TEST(CooperativeRear, PredictedEgoDecelerationCanInvalidateEntry)
{
  EXPECT_TRUE(cooperative(object(-20,3.5,15),prediction(20)));
  EXPECT_FALSE(cooperative(object(-20,3.5,15),prediction(20,4,-3)));
}
TEST(CooperativeRear, BodyOverlapIsAlwaysRejected)
{
  EXPECT_FALSE(cooperative(object(0),prediction()));
}
TEST(CooperativeRear, MissingFutureCoverageIsRejected)
{
  auto obj=object(-20);
  obj.predicted_paths.front().path.resize(2);
  EXPECT_FALSE(cooperative(obj,prediction()));
}
TEST(CooperativeRear, EveryObjectPredictionMustPass)
{
  auto obj=object(-20);
  obj.predicted_paths.push_back(object(-5).predicted_paths.front());
  EXPECT_FALSE(cooperative(obj,prediction()));
}
TEST(CooperativeRear, RequiresExplicitNpcAssumption)
{
  auto p=policy(); p.assume_cooperative_npc=false;
  EXPECT_FALSE(cooperative(object(-20),prediction(),p));
}
TEST(CooperativeRear, TruckYawAndFullLengthAffectGap)
{
  auto truck=object(-15);
  truck.shape.dimensions.x=20;
  const auto aligned=autoware_utils::to_polygon2d(pose(-15),truck.shape);
  const auto angled=autoware_utils::to_polygon2d(pose(-15,3.5,M_PI/2),truck.shape);
  EXPECT_LT(lc::rear_body_gap(road(),pose(0),aligned,vehicle()),10);
  EXPECT_GT(lc::rear_body_gap(road(),pose(0),angled,vehicle()),10);
}
TEST_F(Scene, AcceleratesInCurrentLaneAndWaitsForMeasuredGap)
{
  module->updateOvertake(false);
  ASSERT_EQ(module->overtake_.phase,Harness::OvertakePhase::Accelerating);
  ASSERT_TRUE(module->overtake_prepare_path_);
  for(const auto & p:module->overtake_prepare_path_->points) EXPECT_DOUBLE_EQ(p.point.pose.position.y,0);
  module->updateLaneChangeStatus();
  EXPECT_FALSE(module->getLaneChangeStatus().is_safe);
}
TEST_F(Scene, TargetPersistsAndBecomesReadyOnlyAfterActualPass)
{
  module->updateOvertake(false);
  ASSERT_TRUE(module->overtake_.target);
  const auto id=*module->overtake_.target;
  module->filtered_objects_.target_lane_leading.moving.push_back(object(0,3.5,15,2));
  module->updateOvertake(false);
  ASSERT_TRUE(module->overtake_.target);
  EXPECT_EQ(*module->overtake_.target,id);
  module->filtered_objects_.target_lane_leading.moving.pop_back();
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(40); odom->twist.twist.linear.x=20;
  data->self_odometry=odom; module->setData(data);
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Ready);
  EXPECT_GT(module->overtake_.gap,10);
}
TEST_F(Scene, BlockedFrontSpacePreventsPreparation)
{
  module->filtered_objects_.current_lane.push_back(object(12,0,5,2));
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Blocked);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, BlockedTargetSlotPreventsPreparation)
{
  module->filtered_objects_.target_lane_leading.moving.push_back(object(20,3.5,15,2));
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Blocked);
}
TEST_F(Scene, PreparationCannotCrossTheLaneBoundary)
{
  for (auto & point: module->prev_module_output_.path.points) {
    if (point.point.pose.position.x>20) point.point.pose.position.y=3.5;
  }
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Blocked);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, PassingMustLeaveRoomForTheLateralManeuver)
{
  module->common_data_ptr_->transient_data.dist_to_terminal_start=60;
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Blocked);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, EgoMustBeInsideCurrentLaneBeforeAccelerating)
{
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(0,1.5);
  data->self_odometry=odom; module->setData(data);
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Blocked);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, PreparationHasATimeLimit)
{
  module->updateOvertake(false);
  ros::Time::setNow(ros::Time(109));
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Blocked);
  EXPECT_EQ(module->overtake_.reason,"prepare_timeout");
}
TEST_F(Scene, FeedbackIsRequiredAndMustBeFresh)
{
  EXPECT_FALSE(module->finalVelocityPrediction(candidate()));
  feedback();
  EXPECT_TRUE(module->finalVelocityPrediction(candidate()));
  ros::Time::setNow(ros::Time(101));
  EXPECT_FALSE(module->finalVelocityPrediction(candidate()));
}
TEST_F(Scene, FeedbackSubscribesToTheRos1FinalOutput)
{
  Harness wired(parameters,true);
  EXPECT_EQ(wired.final_trajectory_sub_.getTopic(),"/planning/trajectory");
}
TEST_F(Scene, FeedbackTopicCanBeRemappedByConfiguration)
{
  ros::NodeHandle node("~");
  node.setParam("lane_change/overtake/final_trajectory_topic","/test/final_trajectory");
  Harness wired(parameters,true);
  EXPECT_EQ(wired.final_trajectory_sub_.getTopic(),"/test/final_trajectory");
  node.deleteParam("lane_change/overtake/final_trajectory_topic");
}
TEST_F(Scene, FinalSpeedLimitIsIncludedInPrediction)
{
  feedback(10);
  const auto p=module->finalVelocityPrediction(candidate());
  ASSERT_TRUE(p);
  for(const auto & point:*p) EXPECT_LE(point.velocity,10.001);
}
TEST_F(Scene, AbortChecksThePathThatIsActuallyOutput)
{
  module->status_.lane_change_path=candidate();
  auto abort=candidate(); abort.info.lane_changing_end=pose(60,-1);
  module->abort_path_=std::make_shared<bpp::LaneChangePath>(abort);
  EXPECT_EQ(&module->activeSafetyPath(),&module->status_.lane_change_path);
  module->toAbortState();
  EXPECT_EQ(&module->activeSafetyPath(),module->abort_path_.get());
}
TEST_F(Scene, RepeatedCollisionAcrossHypothesesIsNotDeduplicatedIntoSafety)
{
  safety::CollisionCheckDebugMap debug;
  const auto path=candidate();
  const auto ego=prediction();
  const auto obj=object(0);
  EXPECT_TRUE(module->find_colliding_object_if_all_paths_collide(
    path,{ego,ego},{obj},parameters->safety.rss_params,debug,false));
}
TEST_F(Scene, OnlyTrackedNpcReceivesCooperativePolicy)
{
  const auto target=object(-20);
  module->overtake_.target=target.uuid;
  module->overtake_.phase=Harness::OvertakePhase::Ready;
  EXPECT_TRUE(module->usesCooperativePolicy(target));
  EXPECT_FALSE(module->usesCooperativePolicy(object(-20,3.5,15,2)));
  module->abort_path_=std::make_shared<bpp::LaneChangePath>(candidate());
  module->toAbortState();
  EXPECT_FALSE(module->usesCooperativePolicy(target));
}
TEST_F(Scene, DifferentFrontVehicleRetainsNormalRssCheck)
{
  auto path=candidate();
  path.cooperative_overtake_target=object(-20).uuid;
  module->overtake_.target=path.cooperative_overtake_target;
  module->overtake_.phase=Harness::OvertakePhase::Ready;
  safety::CollisionCheckDebugMap debug;
  const auto front=object(10,3.5,15,2);
  EXPECT_TRUE(module->is_colliding(path,front,prediction(),
    parameters->safety.rss_params,debug,false));
  ASSERT_TRUE(debug.count(front.uuid.uuid));
  EXPECT_EQ(debug.at(front.uuid.uuid).unsafe_reason,"overlap_extended_polygon");
}

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"lane_change_overtake_test",ros::init_options::NoSigintHandler);
  ros::Time::init();
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
