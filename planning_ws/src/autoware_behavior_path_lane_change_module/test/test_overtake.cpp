// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_lane_change_module/scene.hpp>
#include <autoware/behavior_path_lane_change_module/utils/overtake.hpp>
#include <autoware/behavior_path_lane_change_module/utils/speed_profile.hpp>
#include <autoware/behavior_path_lane_change_module/utils/candidate_sampling.hpp>
#include <autoware/behavior_path_lane_change_module/utils/calculation.hpp>
#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <autoware/behavior_path_lane_change_module/utils/safety_check.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <chrono>
#include <iostream>

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
  using NormalLaneChange::executionProfile;
  using NormalLaneChange::execution_replan_requested_;
  using NormalLaneChange::replacementIsContinuous;
  using NormalLaneChange::candidateRank;
  using NormalLaneChange::preparationSpeedMismatch;
  using NormalLaneChange::isBeforeLateralMotion;
  using NormalLaneChange::overtake_feedback_reason_;
  using NormalLaneChange::final_trajectory_sub_;
  using NormalLaneChange::activeSafetyPath;
  using NormalLaneChange::status_;
  using NormalLaneChange::abort_path_;
  using NormalLaneChange::is_colliding;
  using NormalLaneChange::check_candidate_path_safety;
  using NormalLaneChange::find_colliding_object_if_all_paths_collide;
  using NormalLaneChange::usesCooperativePolicy;
  using NormalLaneChange::get_target_objects;
  using NormalLaneChange::hard_collision_detected_;
  using NormalLaneChange::CandidateSearchMode;
  using NormalLaneChange::prepareCandidateForSearch;
  using NormalLaneChange::ahead_candidates_checked_;
  using NormalLaneChange::other_candidates_checked_;
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
    parameters->trajectory.lat_acc_map.add(0.0,0.2,1.0);
    parameters->trajectory.lat_acc_map.add(40.0,0.2,1.0);
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
    module->common_data_ptr_->lanes_polygon_ptr->target = {
      {-101,1.75}, {401,1.75}, {401,5.25}, {-101,5.25}};
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
TEST(MergeSlot, UsesPredictedBodyPositionInsteadOfAccelerationSign)
{
  const auto target=object(5);
  EXPECT_TRUE(lc::merges_ahead_of_target(road(),pose(100,3.5),4,target,vehicle()));
  EXPECT_FALSE(lc::merges_ahead_of_target(road(),pose(50,3.5),4,target,vehicle()));
  EXPECT_FALSE(lc::merges_ahead_of_target(road(),pose(65,3.5),4,target,vehicle()));
}
TEST(MergeSlot, AllTargetPredictionsNeedCoverageAndAnAheadGap)
{
  auto target=object(5);
  target.predicted_paths.push_back(object(50).predicted_paths.front());
  EXPECT_FALSE(lc::merges_ahead_of_target(road(),pose(100,3.5),4,target,vehicle()));
  target.predicted_paths.resize(1);
  target.predicted_paths.front().path.resize(2);
  EXPECT_FALSE(lc::merges_ahead_of_target(road(),pose(100,3.5),4,target,vehicle()));
}
TEST(MergeSlot, SeparatesRearSlotFromMissingCoverageAndBodyOverlap)
{
  auto target=object(5);
  EXPECT_EQ(lc::predicted_merge_slot(road(),pose(50,3.5),4,target,vehicle()),
    lc::MergeSlot::Behind);
  EXPECT_EQ(lc::predicted_merge_slot(road(),pose(65,3.5),4,target,vehicle()),
    lc::MergeSlot::Unavailable);
  target.predicted_paths.front().path.resize(31);
  EXPECT_EQ(lc::predicted_merge_slot(road(),pose(50,3.5),4,target,vehicle()),
    lc::MergeSlot::Unavailable);
}
TEST_F(Scene, PreparationStaysInsideCurrentLane)
{
  module->updateOvertake(false);
  ASSERT_EQ(module->overtake_.phase,Harness::OvertakePhase::Accelerating);
  ASSERT_TRUE(module->overtake_prepare_path_);
  for(const auto & p:module->overtake_prepare_path_->points) EXPECT_DOUBLE_EQ(p.point.pose.position.y,0);
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
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Unavailable);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, BlockedTargetSlotPreventsPreparation)
{
  module->filtered_objects_.target_lane_leading.moving.push_back(object(20,3.5,15,2));
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Unavailable);
}
TEST_F(Scene, PreparationCannotCrossTheLaneBoundary)
{
  for (auto & point: module->prev_module_output_.path.points) {
    if (point.point.pose.position.x>20) point.point.pose.position.y=3.5;
  }
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Unavailable);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, PassingMustLeaveRoomForTheLateralManeuver)
{
  module->common_data_ptr_->transient_data.dist_to_terminal_start=60;
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Unavailable);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, EgoMustBeInsideCurrentLaneBeforeAccelerating)
{
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(0,1.5);
  data->self_odometry=odom; module->setData(data);
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Unavailable);
  EXPECT_FALSE(module->overtake_prepare_path_);
}
TEST_F(Scene, PreparationIsReevaluatedWithoutAnElapsedTimeLimit)
{
  module->updateOvertake(false);
  ros::Time::setNow(ros::Time(109));
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Accelerating);
  EXPECT_TRUE(module->overtake_prepare_path_);
}
TEST_F(Scene, FailedPreparationCanRecoverOnTheVeryNextCycle)
{
  module->filtered_objects_.current_lane.push_back(object(12,0,5,2));
  module->updateOvertake(false);
  ASSERT_EQ(module->overtake_.phase,Harness::OvertakePhase::Unavailable);
  module->filtered_objects_.current_lane.clear();
  ros::Time::setNow(ros::Time(100,100000000));
  module->updateOvertake(false);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Accelerating);
  EXPECT_TRUE(module->overtake_prepare_path_);
}
TEST_F(Scene, AccelerationPreparationDoesNotExcludeRearCandidates)
{
  module->updateOvertake(false);
  ASSERT_EQ(module->overtake_.phase,Harness::OvertakePhase::Accelerating);
  auto rear=candidate(); rear.info.lane_changing_end=pose(50,3.5);
  EXPECT_FALSE(module->prepareCandidateForSearch(rear,Harness::CandidateSearchMode::Ahead));
  EXPECT_TRUE(module->prepareCandidateForSearch(rear,Harness::CandidateSearchMode::Other));
  EXPECT_FALSE(rear.cooperative_overtake_target);
  EXPECT_EQ(module->other_candidates_checked_,1U);
}
TEST_F(Scene, FrontPreferenceCannotLeakCooperationIntoRearCandidate)
{
  module->overtake_.target=object(5).uuid;
  module->overtake_.phase=Harness::OvertakePhase::Ready;
  auto front=candidate(); front.info.lane_changing_end=pose(100,3.5);
  EXPECT_TRUE(module->prepareCandidateForSearch(front,Harness::CandidateSearchMode::Ahead));
  ASSERT_TRUE(front.cooperative_overtake_target);
  EXPECT_FALSE(module->prepareCandidateForSearch(front,Harness::CandidateSearchMode::Other));
  auto rear=front; rear.info.lane_changing_end=pose(50,3.5);
  EXPECT_TRUE(module->prepareCandidateForSearch(rear,Harness::CandidateSearchMode::Other));
  EXPECT_FALSE(rear.cooperative_overtake_target);
}
TEST_F(Scene, ShortPredictionCannotBypassFrontGateAsAnOrdinaryCandidate)
{
  module->overtake_.target=object(5).uuid;
  module->overtake_.phase=Harness::OvertakePhase::Ready;
  module->filtered_objects_.target_lane_leading.moving.front().predicted_paths.front().path.resize(31);
  auto path=candidate(); path.info.lane_changing_end=pose(100,3.5);
  EXPECT_FALSE(module->prepareCandidateForSearch(path,Harness::CandidateSearchMode::Ahead));
  EXPECT_FALSE(module->prepareCandidateForSearch(path,Harness::CandidateSearchMode::Other));
}
TEST_F(Scene, MissingTargetDoesNotLatchAWaitingState)
{
  module->updateOvertake(false);
  module->filtered_objects_.target_lane_leading.moving.clear();
  module->updateOvertake(false);
  EXPECT_FALSE(module->overtake_.target);
  EXPECT_EQ(module->overtake_.phase,Harness::OvertakePhase::Idle);
}
TEST_F(Scene, FeedbackIsRequiredAndMustBeFresh)
{
  EXPECT_FALSE(module->finalVelocityPrediction(candidate()));
  feedback();
  EXPECT_TRUE(module->finalVelocityPrediction(candidate()));
  ros::Time::setNow(ros::Time(101));
  EXPECT_FALSE(module->finalVelocityPrediction(candidate()));
}
TEST_F(Scene, DifferentOrMissingFramesStillRejectFeedback)
{
  feedback();
  auto path=candidate(); path.path.header.frame_id.clear();
  EXPECT_FALSE(module->finalVelocityPrediction(path));
  EXPECT_EQ(module->overtake_feedback_reason_,"feedback_frame_missing");
  path.path.header.frame_id="odom";
  EXPECT_FALSE(module->finalVelocityPrediction(path));
  EXPECT_EQ(module->overtake_feedback_reason_,"feedback_frame_mismatch");
  path.path.header.frame_id="map";
  EXPECT_TRUE(module->finalVelocityPrediction(path));
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

TEST_F(Scene, OrdinaryLaneChangeIncludesCurrentLaneEvenWhenLegacySwitchIsFalse)
{
  parameters->safety.collision_check.check_current_lane=false;
  module->filtered_objects_.current_lane.push_back(object(12,0,5,2));
  const auto targets=module->get_target_objects(module->filtered_objects_,{});
  EXPECT_TRUE(std::any_of(targets.leading.begin(),targets.leading.end(),[](const auto & o) {
    return o.uuid.uuid[0]==2;
  }));
}
TEST_F(Scene, ActualMotionHypothesisExistsBeforeApproval)
{
  const auto paths=lc::convert_to_predicted_paths(module->common_data_ptr_,candidate(),1,false);
  ASSERT_EQ(paths.size(),2u);
  EXPECT_FALSE(paths[0].empty()); EXPECT_FALSE(paths[1].empty());
}
TEST_F(Scene, PersistentFeedbackDelayRequestsReplanningWithoutInventingCollision)
{
  feedback(); auto path=candidate();
  for(auto & p:path.path.points) p.point.pose.position.y+=1.0;
  EXPECT_TRUE(module->finalVelocityPrediction(path));
  EXPECT_EQ(module->overtake_feedback_reason_,"feedback_recovery_pending");
  ros::Time::setNow(ros::Time(100.6)); feedback();
  EXPECT_TRUE(module->finalVelocityPrediction(path));
  ros::Time::setNow(ros::Time(101.01)); feedback();
  EXPECT_TRUE(module->finalVelocityPrediction(path));
  EXPECT_EQ(module->overtake_feedback_reason_,"feedback_progress_replan_requested");
  EXPECT_TRUE(module->execution_replan_requested_);
  // A valid same-cycle correction clears the failure with no extra wait.
  EXPECT_TRUE(module->finalVelocityPrediction(candidate()));
}
TEST_F(Scene, LaterPhysicalCollisionBypassesEarlierRssOnlyHysteresis)
{
  auto obj=object(15,3.5,10);
  safety::CollisionCheckDebugMap debug;
  EXPECT_TRUE(module->is_colliding(candidate(),obj,prediction(20),
    parameters->safety.rss_params,debug,true));
  EXPECT_TRUE(module->hard_collision_detected_);
}
TEST(LaneChangeExecutableSpeed, RampAndPropulsionLimitsAreAppliedTogether)
{
  std::vector<double> s,upper;
  for(int i=0;i<=100;++i) { s.push_back(i); upper.push_back(std::sqrt(100+2*3.25*i)); }
  auto v=lc::limit_speed_profile_jerk(s,upper,0.0,1.0,-1.0,3.25);
  EXPECT_LT(v[1],upper[1]);
  for(size_t i=1;i<v.size();++i) {
    EXPECT_LE(v[i],upper[i]+1e-6);
    EXPECT_LE((v[i]*v[i]-v[i-1]*v[i-1])/(2*(s[i]-s[i-1])),3.250001);
  }
}
TEST(LaneChangeExecutableSpeed, InfeasibleBrakeJerkIsRejected)
{
  EXPECT_THROW(lc::limit_speed_profile_jerk({0,1},{20,1},0,1,-1,3.25),std::logic_error);
}

TEST_F(Scene, MissingPredictionTailCannotBeApprovedAsSafe)
{
  auto obj=object(100,3.5,15);
  obj.predicted_paths.front().path.resize(31); // only three seconds
  safety::CollisionCheckDebugMap debug;
  EXPECT_TRUE(module->is_colliding(candidate(),obj,prediction(20,4),
    parameters->safety.rss_params,debug,false));
  EXPECT_EQ(debug.at(obj.uuid.uuid).unsafe_reason,"prediction_coverage_short");
}
TEST_F(Scene, CompletePredictionCoverageStillAllowsSafeGap)
{
  auto obj=object(100,3.5,15);
  safety::CollisionCheckDebugMap debug;
  EXPECT_FALSE(module->is_colliding(candidate(),obj,prediction(20,4),
    parameters->safety.rss_params,debug,false));
}

TEST(LaneChangeCandidateBudget, AccelerationCruiseAndBrakingAreVisitedBeforeFinePositiveGrid)
{
  std::vector<bpp::LaneChangePhaseMetrics> input;
  for(double a:{3.0,2.0,1.0,0.0,-1.0,-2.0}) input.emplace_back(0,0,15,a,0,0);
  for(double a:{3.0,0.0,-1.0}) input.emplace_back(0.5,7.5,15,a,a,0);
  const auto ordered=lc::diversify_prepare_metrics(input);
  ASSERT_EQ(ordered.size(),input.size());
  EXPECT_EQ(ordered[0].sampled_lon_accel,3.0);
  EXPECT_EQ(ordered[1].sampled_lon_accel,0.0);
  EXPECT_EQ(ordered[2].sampled_lon_accel,-1.0);
  EXPECT_EQ(ordered[6].duration,0.5);
}

TEST_F(Scene, FinalFeedbackChecksActualLateralGeometry)
{
  auto message=boost::make_shared<autoware_planning_msgs::Trajectory>();
  message->header.frame_id="map"; message->header.stamp=ros::Time::now();
  for(double x=-5;x<=150;x+=1) {
    autoware_planning_msgs::TrajectoryPoint p;p.pose=pose(x,0.25);p.longitudinal_velocity_mps=20;
    message->points.push_back(p);
  }
  module->receiveFinalTrajectory(message);
  const auto checked=module->finalVelocityPrediction(candidate());
  ASSERT_TRUE(checked);ASSERT_FALSE(checked->empty());
  EXPECT_NEAR(checked->front().pose.position.y,0.25,1e-9);
}
TEST_F(Scene, FasterFinalProfileIsAlsoCheckedWithoutClippingToCandidate)
{
  feedback(25);
  const auto checked=module->finalVelocityPrediction(candidate());
  ASSERT_TRUE(checked);ASSERT_FALSE(checked->empty());
  EXPECT_NEAR(checked->front().velocity,25.0,1e-6);
}

TEST_F(Scene, NewCandidateDoesNotRequireTheOldFinalGeometryToAlreadyExecuteIt)
{
  feedback();  // The car still follows y=0 before approval.
  auto path=candidate();
  for (auto & point:path.path.points) point.point.pose.position.y=0.8;
  path.info.lane_changing_start=pose(0,0.8);
  path.info.lane_changing_end=pose(80,0.8);
  path.cooperative_overtake_target=module->filtered_objects_.target_lane_leading.moving.front().uuid;
  bpp::lane_change::TargetObjects objects{{},{}};
  objects.leading.push_back(object(200,0.8,20,2));
  EXPECT_TRUE(module->check_candidate_path_safety(path,objects));
  // After approval the actual final motion is still collision-checked while
  // progress is pending, rather than being misclassified as a rear object.
  EXPECT_TRUE(module->finalVelocityPrediction(path));
  EXPECT_EQ(module->overtake_feedback_reason_,"feedback_recovery_pending");
}
TEST_F(Scene, NewCandidateStillRejectsAFrontCollisionBeforeApproval)
{
  feedback();
  auto path=candidate();
  path.cooperative_overtake_target=module->filtered_objects_.target_lane_leading.moving.front().uuid;
  bpp::lane_change::TargetObjects objects{{},{}};
  objects.leading.push_back(object(10,0,0,2));
  EXPECT_FALSE(module->check_candidate_path_safety(path,objects));
}

TEST_F(Scene, StoppedFinalMotionIsCheckedAtTheStopPoseWithoutTeleporting)
{
  feedback(0);
  const auto checked=module->finalVelocityPrediction(candidate());
  ASSERT_TRUE(checked);
  ASSERT_FALSE(checked->empty());
  EXPECT_GE(checked->back().time,4.0);
  for(const auto & point:*checked) {
    EXPECT_NEAR(point.pose.position.x,0,1e-6);
    EXPECT_NEAR(point.velocity,0,1e-6);
  }
  safety::CollisionCheckDebugMap debug;
  EXPECT_TRUE(module->is_colliding(candidate(),object(-10,0,10),*checked,
    parameters->safety.rss_params,debug,true));
}

TEST(LaneChangeFinalObservation, BrakingThenHoldingNeverJumpsToTheManeuverEnd)
{
  const std::vector<double> s{0,10,20,30}, v{10,0,0,0};
  EXPECT_THROW(lc::speed_profile_duration(s,v),std::logic_error);
  const auto duration=lc::observed_speed_profile_duration(s,v,4.0);
  EXPECT_DOUBLE_EQ(duration,4.0);
  const auto moving=lc::sample_speed_profile(s,v,1.0);
  EXPECT_NEAR(moving.first,7.5,1e-9);
  EXPECT_NEAR(moving.second,5.0,1e-9);
  const auto stopped=lc::sample_speed_profile(s,v,duration);
  EXPECT_DOUBLE_EQ(stopped.first,10.0);
  EXPECT_DOUBLE_EQ(stopped.second,0.0);
}

TEST(LaneChangeExecutionProgress, SameManeuverHalfSecondLateAtDifferentSpeeds)
{
  for (double speed : {10.0,25.0,35.0}) {
    std::vector<lc::ExecutionProgressPoint> profile;
    const double yaw = std::atan2(3.5,4.0*speed);
    for(int i=0;i<=40;++i) {
      const double time=i*.1;
      profile.push_back({speed*time,3.5*time/4.0,time,yaw});
    }
    EXPECT_NEAR(lc::execution_recovery_time(profile,speed*2.0,3.5*1.5/4.0,
      yaw,speed,0.0,1.0,.5),.5,1e-6);
  }
}
TEST(LaneChangeExecutionProgress, DifferentGeometryAndWrongHeadingCannotHideAsTimeLag)
{
  const std::vector<lc::ExecutionProgressPoint> profile{{0,0,0,0},{100,0,4,0}};
  EXPECT_GT(lc::execution_recovery_time(profile,50,1.0,0,25,.2,1,.5),1.0);
  EXPECT_GT(lc::execution_recovery_time(profile,50,0,3.14,25,.2,1,.5),1.0);
}
TEST(LaneChangeExecutionProgress, DurationUsesSecondsAndResetsOnRecoveryOrClockReset)
{
  lc::ExecutionRecoveryMonitor monitor;
  EXPECT_FALSE(monitor.expired(100,false,1));
  EXPECT_FALSE(monitor.expired(100.9,false,1));
  EXPECT_TRUE(monitor.expired(101,false,1));
  EXPECT_FALSE(monitor.expired(101.01,true,1));
  EXPECT_FALSE(monitor.expired(101.02,false,1));
  EXPECT_FALSE(monitor.expired(1,false,1));
}
TEST_F(Scene, ActualFootprintLeavingCorridorIsRejectedWithoutProgressWait)
{
  auto message=boost::make_shared<autoware_planning_msgs::Trajectory>();
  message->header.frame_id="map";message->header.stamp=ros::Time::now();
  for(double x=-5;x<=150;x+=1) {
    autoware_planning_msgs::TrajectoryPoint p;
    p.pose=pose(x,std::min(8.0,std::max(0.0,x*.1)));
    p.longitudinal_velocity_mps=20;
    message->points.push_back(p);
  }
  module->receiveFinalTrajectory(message);
  EXPECT_FALSE(module->finalVelocityPrediction(candidate()));
  EXPECT_EQ(module->overtake_feedback_reason_,"feedback_outside_corridor");
  EXPECT_TRUE(module->hard_collision_detected_);
}
TEST_F(Scene, DistantStoppedFrontObjectAllowsSafeAdjacentLaneCandidate)
{
  auto path=candidate();
  for(auto & p:path.path.points) {
    const double x=p.point.pose.position.x;
    p.point.pose.position.y=3.5*std::clamp(x/50.0,0.0,1.0);
    p.point.pose.orientation=pose(0,0,(x>0&&x<50)?std::atan2(3.5,50):0).orientation;
  }
  path.info.lane_changing_end=pose(80,3.5);
  bpp::lane_change::TargetObjects objects{{},{}};
  objects.leading.push_back(object(80,0,0,2));
  EXPECT_TRUE(module->check_candidate_path_safety(path,objects));
  objects.leading.clear();objects.leading.push_back(object(10,0,0,2));
  EXPECT_FALSE(module->check_candidate_path_safety(path,objects));
}

TEST_F(Scene, EgoDeviationRequestsReplanWithoutAutomaticCancel)
{
  parameters->enable_path_miss_detection=true;
  lanelet::LineString3d left(11, {lanelet::Point3d(12,-100,5.25,0),lanelet::Point3d(13,400,5.25,0)});
  lanelet::LineString3d right(14, {lanelet::Point3d(15,-100,1.75,0),lanelet::Point3d(16,400,1.75,0)});
  module->common_data_ptr_->lanes_ptr->target.push_back(lanelet::Lanelet(17,left,right));
  auto path=candidate();
  for(auto & p:path.path.points) p.point.pose.position.y=1.0;
  module->status_.lane_change_path=path;
  EXPECT_FALSE(module->hasMissedLaneChangePath());
  ros::Time::setNow(ros::Time(100.9));
  EXPECT_FALSE(module->hasMissedLaneChangePath());
  ros::Time::setNow(ros::Time(101.01));
  EXPECT_FALSE(module->hasMissedLaneChangePath());
  EXPECT_TRUE(module->execution_replan_requested_);
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(0,1.0);data->self_odometry=odom;
  EXPECT_FALSE(module->hasMissedLaneChangePath());
}
TEST_F(Scene, PhaseClockUsesSelectedDurationInsteadOfMapSpeedCap)
{
  auto path=candidate();
  for(auto & p:path.path.points) p.point.longitudinal_velocity_mps=41.6667;
  const auto profile=module->executionProfile(path);
  const auto end=std::find_if(profile.begin(),profile.end(),[](const auto & p) {return p.s==180.0;});
  ASSERT_NE(end,profile.end());
  EXPECT_NEAR(end->time,4.0,1e-6);
}
TEST_F(Scene, ExecutionMonitorCostForDenseFeedback)
{
  auto message=boost::make_shared<autoware_planning_msgs::Trajectory>();
  message->header.frame_id="map";message->header.stamp=ros::Time::now();
  for(double x=-5;x<=200;x+=.2) {
    autoware_planning_msgs::TrajectoryPoint p;p.pose=pose(x);p.longitudinal_velocity_mps=20;
    message->points.push_back(p);
  }
  module->receiveFinalTrajectory(message);
  const auto begin=std::chrono::steady_clock::now();
  for(int i=0;i<20;++i) ASSERT_TRUE(module->finalVelocityPrediction(candidate()));
  const double mean_ms=std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-begin).count()/20.0;
  std::cout << "Dense feedback monitor mean_ms=" << mean_ms << std::endl;
  EXPECT_LT(mean_ms,parameters->time_limit);
}


TEST(LaneChangeCompactSampling, SevenDurationsThreeStrategiesSixExplicitLimits)
{
  std::vector<bpp::LaneChangePhaseMetrics> dense;
  for (int i=0;i<=6;++i) for(int j=0;j<=20;++j) {
    const double a=3.7-j*11.7/20;
    dense.emplace_back(i*.5,10,20,a,a,0);
  }
  const auto reduced=lc::compact_prepare_metrics(dense,3.7,-2.0);
  ASSERT_EQ(reduced.size(),21u);
  for(int i=0;i<=6;++i) {
    EXPECT_DOUBLE_EQ(reduced[i*3].duration,i*.5);
    EXPECT_GT(reduced[i*3].sampled_lon_accel,0);
    EXPECT_NEAR(reduced[i*3+1].sampled_lon_accel,0,.3);
    EXPECT_LT(reduced[i*3+2].sampled_lon_accel,0);
  }
  bpp::lane_change::TrajectoryParameters p;
  p.lat_acc_map.add(0,1,6.5);
  p.lateral_acceleration_samples={2,3,4,5,6,6.5};
  EXPECT_EQ(lc::lateral_acceleration_samples(p,20),p.lateral_acceleration_samples);
  EXPECT_EQ(reduced.size()*lc::lateral_acceleration_samples(p,20).size(),126u);
  p.lat_acc_map={};p.lat_acc_map.add(0,.2,1);
  EXPECT_EQ(lc::lateral_acceleration_samples(p,20),std::vector<double>({1}));
}
TEST(LaneChangePredictionEndpoint, KeepsOriginalEndIncludingNonGridHorizon)
{
  for (const double horizon : {0.0,3.0,3.2,8.0}) {
    autoware_perception_msgs::PredictedPath raw;
    raw.confidence=1; raw.time_step.sec=0; raw.time_step.nanosec=100000000;
    for(int i=0;i<=int(std::round(horizon*10));++i) raw.path.push_back(pose(i*.1*15));
    autoware_perception_msgs::Shape shape;
    shape.type=shape.BOUNDING_BOX;shape.dimensions.x=4.5;shape.dimensions.y=2;
    const auto converted=lc::transform_predicted_path(raw,shape,15,.5);
    ASSERT_TRUE(converted);
    EXPECT_NEAR(converted->path.back().time,horizon,1e-6);
    EXPECT_NEAR(converted->path.back().pose.position.x,horizon*15,1e-6);
    for(size_t i=1;i<converted->path.size();++i) EXPECT_GT(converted->path[i].time,converted->path[i-1].time);
    EXPECT_FALSE(lc::transform_predicted_path(raw,shape,15,0));
  }
}
TEST_F(Scene, DisabledOvertakeDoesNotPromoteUnrelatedOtherLaneObject)
{
  parameters->overtake.enable=false;
  parameters->safety.collision_check.check_other_lanes=false;
  module->overtake_.target=object(93,-4,15,8).uuid; // stale prior target is not authority
  module->filtered_objects_.others.push_back(object(93,-4,15,8));
  module->filtered_objects_.current_lane.push_back(object(40,0,15,9));
  const auto selected=module->get_target_objects(module->filtered_objects_,{});
  ASSERT_EQ(selected.leading.size(),2u);
  for(const auto & o:selected.leading) EXPECT_NE(o.uuid,object(93,-4,15,8).uuid);
}
TEST_F(Scene, ReplacementMustJoinMeasuredPoseHeadingAndSpeed)
{
  auto path=candidate();
  for(auto & p:path.path.points) p.point.longitudinal_velocity_mps=15;
  EXPECT_TRUE(module->replacementIsContinuous(path));
  auto shifted=path;
  for(auto & p:shifted.path.points) p.point.pose.position.y=1;
  EXPECT_FALSE(module->replacementIsContinuous(shifted));
  auto rotated=path;
  for(auto & p:rotated.path.points) p.point.pose.orientation=pose(0,0,.2).orientation;
  EXPECT_FALSE(module->replacementIsContinuous(rotated));
  for(auto & p:path.path.points) p.point.longitudinal_velocity_mps=20;
  EXPECT_FALSE(module->replacementIsContinuous(path));
}
class RollingHarness : public Harness
{
public:
  using Harness::Harness;
  bpp::LaneChangePath proposed;
  bool safe{true};
  std::pair<bool,bool> getSafePath(bpp::LaneChangePath & out) const override
  { out=proposed;return {true,safe}; }
};
TEST_F(Scene, PreparationTakesNewImmediateOpportunityAndKeepsApprovalOnFailedSearch)
{
  RollingHarness rolling(parameters);
  data->route_handler=std::make_shared<autoware::route_handler::RouteHandler>();
  rolling.setData(data);
  auto active=candidate();
  for(auto & p:active.path.points) p.point.longitudinal_velocity_mps=15;
  active.info.shift_line.start=pose(45);active.info.duration.prepare=3;
  rolling.status_.lane_change_path=active;
  rolling.status_.is_valid_path=true;rolling.status_.is_safe=true;
  rolling.proposed=active;rolling.proposed.info.shift_line.start=pose(0);
  rolling.proposed.info.duration.prepare=0;
  rolling.refreshApprovedPlan();
  EXPECT_DOUBLE_EQ(rolling.getLaneChangePath().info.duration.prepare,0);
  // A rejected next search must not replace this safe maneuver with fallback.
  rolling.execution_replan_requested_=true;rolling.safe=false;
  rolling.proposed.info.duration.prepare=2;
  rolling.refreshApprovedPlan();
  EXPECT_DOUBLE_EQ(rolling.getLaneChangePath().info.duration.prepare,0);
  EXPECT_TRUE(rolling.execution_replan_requested_);
}
TEST_F(Scene, SelectionPrefersAccelerationThenShortPreparationThenGentleMotion)
{
  module->filtered_objects_={};
  auto immediate=candidate();immediate.info.lateral_acceleration=3;
  auto delayed=immediate;delayed.info.duration.prepare=3;delayed.info.lateral_acceleration=2;
  EXPECT_LT(module->candidateRank(immediate),module->candidateRank(delayed));
  auto same_gentle=delayed;same_gentle.info.duration.prepare=0;
  EXPECT_LT(module->candidateRank(same_gentle),module->candidateRank(delayed));
  auto accelerating=delayed;accelerating.info.longitudinal_acceleration.lane_changing=.5;
  auto braking=same_gentle;braking.info.longitudinal_acceleration.lane_changing=-.5;
  EXPECT_LT(module->candidateRank(accelerating),module->candidateRank(same_gentle));
  EXPECT_LT(module->candidateRank(same_gentle),module->candidateRank(braking));
  auto harsh=immediate;harsh.info.lateral_acceleration=6.5;
  EXPECT_LT(module->candidateRank(immediate),module->candidateRank(harsh));
}
TEST_F(Scene, VerifiedFrontGapPrecedesAccelerationAndUnknownIsNotAFrontGap)
{
  module->filtered_objects_={};
  module->filtered_objects_.target_lane_leading.moving.push_back(object(5,3.5,15));
  auto ahead=candidate();ahead.info.lane_changing_end=pose(150);
  ahead.info.duration.prepare=3; ahead.info.longitudinal_acceleration.lane_changing=-1;
  auto behind=candidate();behind.info.lane_changing_end=pose(0);
  behind.info.longitudinal_acceleration.lane_changing=1;
  EXPECT_LT(module->candidateRank(ahead),module->candidateRank(behind));
  auto unavailable=behind;unavailable.info.duration.prepare=20;
  EXPECT_EQ(std::get<0>(module->candidateRank(ahead)),0);
  EXPECT_NE(std::get<0>(module->candidateRank(unavailable)),0);
}
TEST(LaneChangeCompactSampling, ExplicitPreparationTimesAreAbsoluteAndRespectSpeedReadiness)
{
  bpp::lane_change::TrajectoryParameters p;p.max_prepare_duration=7;
  p.preparation_duration_samples={0,.5,1,1.5,2,3,4.5,7};
  EXPECT_EQ(lc::preparation_duration_samples(p,0),p.preparation_duration_samples);
  EXPECT_EQ(lc::preparation_duration_samples(p,1.2),std::vector<double>({1.5,2,3,4.5,7}));
  EXPECT_TRUE(lc::preparation_duration_samples(p,7.1).empty());
  EXPECT_TRUE(lc::preparation_duration_samples(p,-1).empty());
  EXPECT_TRUE(lc::preparation_duration_samples(p,std::numeric_limits<double>::quiet_NaN()).empty());
  // Configuration order and duplicates must not multiply work.
  p.preparation_duration_samples={7,1,1,0,8};
  EXPECT_EQ(lc::preparation_duration_samples(p,0),std::vector<double>({0,1,7}));
}
TEST_F(Scene, ProductionDurationGeneratorUsesExplicitGridWithoutShiftingOrAddingTimes)
{
  parameters->trajectory.min_prepare_duration=0;
  parameters->trajectory.max_prepare_duration=7;
  parameters->trajectory.preparation_duration_samples={0,.5,1,1.5,2,3,4.5,7};
  auto & ready=module->common_data_ptr_->transient_data.lane_change_prepare_duration;
  ready=0;
  EXPECT_EQ(lc::calculation::calc_prepare_durations(module->common_data_ptr_),
    parameters->trajectory.preparation_duration_samples);
  ready=2.2;
  EXPECT_EQ(lc::calculation::calc_prepare_durations(module->common_data_ptr_),
    std::vector<double>({3,4.5,7}));
  // Empty optional grid preserves the existing uniform half-second policy.
  parameters->trajectory.preparation_duration_samples.clear();ready=0;
  EXPECT_EQ(lc::calculation::calc_prepare_durations(module->common_data_ptr_).size(),15u);
}
TEST(LaneChangeCompactSampling, PreparationCapDoesNotDeleteAccelerationDuringExecution)
{
  std::vector<bpp::LaneChangePhaseMetrics> capped;
  capped.emplace_back(2.0,30.0,15.0,0.0,0.0,0.0);
  capped.emplace_back(2.0,26.0,11.0,-2.0,-2.0,0.0);
  const auto reduced=lc::compact_prepare_metrics(capped,3.0,-2.0);
  ASSERT_EQ(reduced.size(),3u);
  EXPECT_DOUBLE_EQ(reduced[0].actual_lon_accel,0.0);
  EXPECT_DOUBLE_EQ(reduced[0].velocity,15.0);
  EXPECT_DOUBLE_EQ(reduced[0].sampled_lon_accel,3.0);
  EXPECT_DOUBLE_EQ(reduced[1].sampled_lon_accel,0.0);
  EXPECT_DOUBLE_EQ(reduced[2].sampled_lon_accel,-2.0);
}
TEST(LaneChangeCompactSampling, EightDurationsThreeStrategiesSixLateralLimits)
{
  bpp::lane_change::TrajectoryParameters p;p.max_prepare_duration=7;
  p.preparation_duration_samples={0,.5,1,1.5,2,3,4.5,7};
  std::vector<bpp::LaneChangePhaseMetrics> dense;
  for(double t:lc::preparation_duration_samples(p,0)) for(double a:{3.0,0.0,-2.0})
    dense.emplace_back(t,10.0,15.0,a,a,0.0);
  const auto reduced=lc::compact_prepare_metrics(dense,3.0,-2.0);
  p.lat_acc_map.add(0.0,1.0,6.5);p.lateral_acceleration_samples={1,2,3,4,5,6.5};
  ASSERT_EQ(reduced.size(),24u);
  EXPECT_EQ(reduced.size()*lc::lateral_acceleration_samples(p,15).size(),144u);
}
TEST_F(Scene, ReplacementContinuityDoesNotDependOnLongitudinalSampleSpacing)
{
  auto path=candidate();
  for(auto & p:path.path.points) p.point.longitudinal_velocity_mps=15;
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  path.path.points.erase(std::remove_if(path.path.points.begin(),path.path.points.end(),
    [](const auto & p) { return static_cast<int>(p.point.pose.position.x)%2 != 0; }),
    path.path.points.end());
  odom->pose.pose=pose(1.0);data->self_odometry=odom;
  EXPECT_TRUE(module->replacementIsContinuous(path));
}
TEST_F(Scene, PassingAnOldStartWithWrongSpeedRequestsFreshPlanBeforeLateralMotion)
{
  parameters->th_finish_judge_lateral_diff=.1;
  auto path=candidate();path.info.shift_line.start=pose(-1);
  for(auto & p:path.path.points) p.point.longitudinal_velocity_mps=20;
  module->status_.lane_change_path=path;module->status_.is_valid_path=true;
  EXPECT_FALSE(module->isEgoOnPreparePhase());
  EXPECT_TRUE(module->isBeforeLateralMotion());
  EXPECT_TRUE(module->preparationSpeedMismatch());
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(0,.3);data->self_odometry=odom;
  EXPECT_FALSE(module->preparationSpeedMismatch()); // active motion uses RSS/feasible return
}
TEST_F(Scene, StalePreparationSpeedCannotKeepAnUnsafeOldPlan)
{
  RollingHarness rolling(parameters);
  data->route_handler=std::make_shared<autoware::route_handler::RouteHandler>();
  rolling.setData(data);
  auto active=candidate();active.info.shift_line.start=pose(45);active.info.duration.prepare=3;
  for(auto & p:active.path.points) p.point.longitudinal_velocity_mps=20;
  rolling.status_.lane_change_path=active;rolling.status_.is_valid_path=true;
  rolling.status_.is_safe=true;rolling.proposed=active;rolling.safe=false;
  rolling.refreshApprovedPlan();
  EXPECT_FALSE(rolling.status_.is_valid_path);
  EXPECT_TRUE(rolling.execution_replan_requested_);
}
TEST_F(Scene, AnOutboundEndpointCannotFinishAnAbort)
{
  module->status_.lane_change_path=candidate();
  module->abort_path_=std::make_shared<bpp::LaneChangePath>(candidate());
  module->toAbortState();
  EXPECT_FALSE(module->hasFinishedLaneChange());
}
TEST_F(Scene, AbortCompletionRequiresReturnBodyLateralPositionAndHeading)
{
  parameters->th_finish_judge_lateral_diff=.1;parameters->th_finish_judge_yaw_diff=.1;
  module->abort_path_=std::make_shared<bpp::LaneChangePath>(candidate());
  module->abort_path_->info.shift_line.end=pose(80);
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(90,1);data->self_odometry=odom;
  EXPECT_FALSE(module->hasFinishedAbort());
  odom->pose.pose=pose(90,0,.2);
  EXPECT_FALSE(module->hasFinishedAbort());
  odom->pose.pose=pose(90,0,0);
  EXPECT_TRUE(module->hasFinishedAbort());
}
TEST_F(Scene, ZeroPrepareFrenetPredictionIncludesItsTimeOrigin)
{
  parameters->trajectory.min_prepare_duration=0;
  bpp::lane_change::TrajectoryGroup group;
  group.prepare=road(0,15);group.prepare_metric.duration=0;
  group.prepare_metric.velocity=15;
  group.lane_changing.times={0,.5,1};
  group.lane_changing.poses={pose(0),pose(7.5),pose(15)};
  group.lane_changing.longitudinal_velocities={15,15,15};
  const auto prediction=lc::convert_to_predicted_path(module->common_data_ptr_,group,1);
  ASSERT_EQ(prediction.size(),3u);
  EXPECT_DOUBLE_EQ(prediction.front().time,0.0);
  ASSERT_TRUE(lc::remaining_prepare_time(group.prepare,prediction,pose(0)));
  // Reproduce the previous implementation's missing origin and its rejection.
  const std::vector<safety::PoseWithVelocityStamped> old(prediction.begin()+1,prediction.end());
  EXPECT_FALSE(lc::remaining_prepare_time(group.prepare,old,pose(0)));
}
TEST_F(Scene, ApprovedFrenetPredictionRebasesToCurrentEgoRatherThanOriginalTimeTable)
{
  parameters->trajectory.min_prepare_duration=0;
  auto path=candidate();path.type=bpp::lane_change::PathType::FrenetPlanner;
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(25);data->self_odometry=odom;
  module->common_data_ptr_->self_odometry_ptr=odom;
  const auto prediction=lc::convert_to_predicted_paths(module->common_data_ptr_,path,1,true);
  ASSERT_FALSE(prediction.empty());ASSERT_FALSE(prediction.front().empty());
  EXPECT_DOUBLE_EQ(prediction.front().front().time,0.0);
  EXPECT_NEAR(prediction.front().front().pose.position.x,25,1e-6);
  EXPECT_GT(prediction.front().back().pose.position.x,25);
}
TEST_F(Scene, FinalFeedbackRemainsNonemptyInTheApprovedFinishBuffer)
{
  parameters->lane_change_finish_judge_buffer=3;
  auto odom=boost::make_shared<nav_msgs::Odometry>(*data->self_odometry);
  odom->pose.pose=pose(81);data->self_odometry=odom;
  module->common_data_ptr_->self_odometry_ptr=odom;
  feedback(15);
  const auto prediction=module->finalVelocityPrediction(candidate());
  ASSERT_TRUE(prediction);ASSERT_FALSE(prediction->empty());
  EXPECT_DOUBLE_EQ(prediction->front().time,0.0);
  EXPECT_NEAR(prediction->front().pose.position.x,81,1e-6);
}
TEST_F(Scene, GentleFrenetPreferenceUsesTheWholePeakNotOnlyItsInitialAcceleration)
{
  auto gentle=candidate(),abrupt=candidate();
  gentle.type=abrupt.type=bpp::lane_change::PathType::FrenetPlanner;
  gentle.info.lateral_acceleration=.2;abrupt.info.lateral_acceleration=0;
  gentle.frenet_path.lane_changing.lateral_accelerations={.2,1.0,-.8,0};
  abrupt.frenet_path.lane_changing.lateral_accelerations={0,4.0,-3.0,0};
  EXPECT_LT(module->candidateRank(gentle),module->candidateRank(abrupt));
  EXPECT_DOUBLE_EQ(lc::candidate_lateral_acceleration(gentle),1.0);
}
TEST(LaneChangeCompactSampling, FrenetPeakCannotExceedTheVelocityDependentLateralEnvelope)
{
  bpp::lane_change::TrajectoryParameters p;p.lat_acc_map.add(0,1,6.5);
  autoware::frenet_planner::Trajectory trajectory;
  trajectory.longitudinal_velocities={15,15,15};
  trajectory.lateral_accelerations={0,6.5,0};
  EXPECT_TRUE(lc::frenet_lateral_envelope_satisfied(trajectory,p));
  trajectory.lateral_accelerations[1]=6.6;
  EXPECT_FALSE(lc::frenet_lateral_envelope_satisfied(trajectory,p));
}
TEST_F(Scene, AdmittedGeometryDoesNotDependOnAnAmbiguousNearestRouteId)
{
  EXPECT_TRUE(lc::footprint_within_admitted_lane_union(pose(0),vehicle(),
    *module->common_data_ptr_->lanes_polygon_ptr));
  // A body straddling the admitted current/target shared boundary is included
  // in their union; checking either polygon alone would reject a valid LC.
  EXPECT_TRUE(lc::footprint_within_admitted_lane_union(pose(0,1.75),vehicle(),
    *module->common_data_ptr_->lanes_polygon_ptr));
  EXPECT_FALSE(lc::footprint_within_admitted_lane_union(pose(0,6),vehicle(),
    *module->common_data_ptr_->lanes_polygon_ptr));
}
TEST_F(Scene, CorridorUnionDoesNotBridgeAnUnadmittedGapOrAcceptOnlyTheCentre)
{
  auto lanes=*module->common_data_ptr_->lanes_polygon_ptr;
  lanes.target={{-101,3.5},{401,3.5},{401,7},{-101,7}};
  EXPECT_FALSE(lc::footprint_within_admitted_lane_union(pose(0,2.5),vehicle(),lanes));
  EXPECT_FALSE(lc::footprint_within_admitted_lane_union(pose(0,-1.5),vehicle(),lanes));
}
int main(int argc, char ** argv)
{
  ros::init(argc,argv,"lane_change_overtake_test",ros::init_options::NoSigintHandler);
  ros::Time::init();
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
