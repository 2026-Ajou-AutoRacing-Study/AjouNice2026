// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_path_lane_change_module/scene.hpp>
#include <autoware/behavior_path_lane_change_module/utils/overtake.hpp>
#include <autoware/behavior_path_lane_change_module/utils/speed_profile.hpp>
#include <autoware/behavior_path_lane_change_module/utils/candidate_sampling.hpp>
#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
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
TEST_F(Scene, FinalFeedbackFromDifferentLateralManeuverIsRejected)
{
  feedback(); auto path=candidate();
  for(auto & p:path.path.points) p.point.pose.position.y+=1.0;
  EXPECT_FALSE(module->finalVelocityPrediction(path));
  EXPECT_EQ(module->overtake_feedback_reason_,"feedback_maneuver_geometry_mismatch");
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

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"lane_change_overtake_test",ros::init_options::NoSigintHandler);
  ros::Time::init();
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
