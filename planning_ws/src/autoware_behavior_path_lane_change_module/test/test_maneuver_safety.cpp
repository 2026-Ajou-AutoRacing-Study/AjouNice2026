#include "autoware/behavior_path_lane_change_module/utils/maneuver_safety.hpp"
#include "autoware/behavior_path_lane_change_module/utils/prepared_safety_world.hpp"
#include <gtest/gtest.h>

namespace lc = autoware::behavior_path_planner::utils::lane_change;

TEST(PreparedSafetyWorld, ExactAgeHorizonAndMalformedInputs) {
  autoware_perception_msgs::PredictedObjects raw;
  raw.header.frame_id="map";raw.header.stamp=ros::Time(100);
  nav_msgs::Odometry odom;odom.header=raw.header;
  autoware_perception_msgs::PredictedObject object;
  object.shape.type=autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x=4;object.shape.dimensions.y=2;object.shape.dimensions.z=1;
  autoware_perception_msgs::PredictedPath prediction;prediction.time_step.nanosec=100000000;
  for(int i=0;i<=30;++i){geometry_msgs::Pose p;p.position.x=i;p.orientation.w=1;prediction.path.push_back(p);}
  object.kinematics.predicted_paths.push_back(prediction);raw.objects.push_back(object);
  for(double age:{.05,-.005}) {
    odom.header.stamp=raw.header.stamp+ros::Duration(age);
    const auto world=lc::prepare_safety_observations(raw,odom,.5);
    EXPECT_NEAR(world.second,3-std::max(0.,age),1e-10);
    EXPECT_NEAR(world.first[0].predicted_paths[0].path.front().pose.position.x,
      std::max(0.,age)*10,1e-10);
    EXPECT_DOUBLE_EQ(world.first[0].predicted_paths[0].path.front().time,0);
    EXPECT_NEAR(world.first[0].predicted_paths[0].path.back().time,world.second,1e-10);
    EXPECT_NEAR(world.second+std::max(0.,-age),age<0?3.005:2.95,1e-10);
  }
  odom.header.stamp=raw.header.stamp-ros::Duration(.011);
  EXPECT_THROW(lc::prepare_safety_observations(raw,odom,.5),std::logic_error);
  odom.header.stamp=raw.header.stamp+ros::Duration(.501);
  EXPECT_THROW(lc::prepare_safety_observations(raw,odom,.5),std::logic_error);
  odom.header=raw.header;odom.header.frame_id="odom";
  EXPECT_THROW(lc::prepare_safety_observations(raw,odom,.5),std::logic_error);
  odom.header=raw.header;
  EXPECT_THROW(lc::prepare_safety_observations(raw,odom,std::numeric_limits<double>::quiet_NaN()),std::logic_error);
  raw.objects[0].kinematics.predicted_paths[0].time_step.nanosec=0;
  EXPECT_THROW(lc::prepare_safety_observations(raw,odom,.5),std::logic_error);
  raw.objects[0].kinematics.predicted_paths.clear();
  EXPECT_THROW(lc::prepare_safety_observations(raw,odom,.5),std::logic_error);
}
namespace {
lc::SafetyPath line(double velocity=10.0) {
  lc::SafetyPath path;
  for (int i=0;i<=150;++i) {
    autoware_internal_planning_msgs::PathPointWithLaneId point;
    point.point.pose.position.x=i;
    point.point.pose.orientation.w=1.0;
    point.point.longitudinal_velocity_mps=velocity;
    point.lane_ids={42};
    path.points.push_back(point);
  }
  return path;
}
}

TEST(ManeuverSafety, ActualAndPlannedAreDistinct) {
  const auto path=line(10);
  const auto hypotheses=lc::maneuver_motion_hypotheses(path,0,30,5,0,30,0,.5);
  ASSERT_EQ(hypotheses.size(),2u);
  EXPECT_DOUBLE_EQ(hypotheses[0].back().time,3);
  EXPECT_NEAR(hypotheses[0].back().pose.position.x,30,1e-6);
  EXPECT_NEAR(hypotheses[1].back().pose.position.x,15,1e-6);
}

TEST(ManeuverSafety, StoppedPlanRetainsLateRearExposure) {
  const auto path=line(0);
  const auto hypotheses=lc::maneuver_motion_hypotheses(path,0,30,0,0,30,3,.5);
  ASSERT_GT(hypotheses[0].size(),2u);
  EXPECT_DOUBLE_EQ(hypotheses[0].back().time,3);
  EXPECT_DOUBLE_EQ(hypotheses[0].back().velocity,0);
  EXPECT_DOUBLE_EQ(hypotheses[0].back().pose.position.x,0);
}

TEST(ManeuverSafety, NoMovingPoseClampOrMissingActualCoverage) {
  const auto path=line(10);
  EXPECT_THROW(lc::maneuver_motion_hypotheses(path,0,10,10,0,30,3,.5),std::logic_error);
  EXPECT_THROW(lc::maneuver_motion_hypotheses(path,0,100,30,0,30,0,.5),std::logic_error);
}

TEST(ManeuverSafety, BrakingPreservesGeometryAndStopsWithinLimits) {
  auto path=line(30);
  for(size_t i=80;i<path.points.size();++i)path.points[i].point.longitudinal_velocity_mps=0;
  const auto proposal=lc::make_maneuver_braking_proposal(path,0,10,0,-3,-2,2);
  ASSERT_EQ(path.points.size(),proposal.points.size());
  for(size_t i=0;i<path.points.size();++i) {
    EXPECT_EQ(path.points[i].point.pose,proposal.points[i].point.pose);
    EXPECT_EQ(path.points[i].lane_ids,proposal.points[i].lane_ids);
    EXPECT_LE(proposal.points[i].point.longitudinal_velocity_mps,path.points[i].point.longitudinal_velocity_mps);
    if(i) {
      const double v0=proposal.points[i-1].point.longitudinal_velocity_mps;
      const double v1=proposal.points[i].point.longitudinal_velocity_mps;
      EXPECT_GE((v1*v1-v0*v0)/2,-3.01);
    }
  }
  EXPECT_EQ(proposal.points[80].point.longitudinal_velocity_mps,0);
}

TEST(ManeuverSafety, BrakingRejectsUnreachableUpstreamStop) {
  auto path=line(30);
  path.points[3].point.longitudinal_velocity_mps=0;
  EXPECT_THROW(lc::make_maneuver_braking_proposal(path,0,10,0,-3,-2,2),std::logic_error);
}

TEST(ManeuverSafety, StrongMeasuredBrakingIsPreservedAndReleasedNotClamped) {
  auto path=line(30);
  // Spatial probes inside the first integration step reveal whether a(0) was
  // wrongly replaced by the command limit before integrating.
  path.points[1].point.pose.position.x=.01;
  const auto proposal=lc::make_maneuver_braking_proposal(path,0,10,-4.54,-4,-10,2);
  EXPECT_FLOAT_EQ(proposal.points[0].point.longitudinal_velocity_mps,10);
  const double first_v=proposal.points[1].point.longitudinal_velocity_mps;
  const double first_mean_acc=(first_v*first_v-100)/.02;
  EXPECT_NEAR(first_mean_acc,-4.53,.01);  // -4.54 -> -4.52 over dt=.01.
  EXPECT_LT(first_mean_acc,-4.4);         // Not an instantaneous clamp to -4.
  for(size_t i=0;i<proposal.points.size();++i) {
    EXPECT_TRUE(std::isfinite(proposal.points[i].point.longitudinal_velocity_mps));
    EXPECT_GE(proposal.points[i].point.longitudinal_velocity_mps,0);
    EXPECT_EQ(proposal.points[i].point.pose,path.points[i].point.pose);
  }
  EXPECT_EQ(proposal.points.back().point.longitudinal_velocity_mps,0);
}

TEST(ManeuverSafety, PositiveReleaseJerkAndR18MeasuredState) {
  const auto path=line(30);
  const auto slow=lc::make_maneuver_braking_proposal(path,0,10,-6,-4,-10,1);
  const auto fast=lc::make_maneuver_braking_proposal(path,0,10,-6,-4,-10,10);
  EXPECT_LT(slow.points[5].point.longitudinal_velocity_mps,
            fast.points[5].point.longitudinal_velocity_mps);
  // R18/002 accepted failure's actual state must not be rejected solely for
  // being below the nominal -4 command acceleration.
  const auto observed=lc::make_maneuver_braking_proposal(
    path,0,10.85082102624088,-4.539554431026597,-4,-10,100);
  EXPECT_NEAR(observed.points.front().point.longitudinal_velocity_mps,10.85082102624088,1e-5);
  EXPECT_EQ(observed.points.back().point.longitudinal_velocity_mps,0);
}

TEST(ManeuverSafety, StrongBrakingStopNeverReversesOrViolatesUpstreamCap) {
  auto path=line(30);
  for(size_t i=10;i<path.points.size();++i)path.points[i].point.longitudinal_velocity_mps=0;
  const auto stopped=lc::make_maneuver_braking_proposal(path,0,0,-8,-4,-10,100);
  for(const auto & point:stopped.points)EXPECT_EQ(point.point.longitudinal_velocity_mps,0);
  const auto tiny=lc::make_maneuver_braking_proposal(path,0,.001,-8,-4,-10,100);
  for(size_t i=0;i<tiny.points.size();++i) {
    EXPECT_GE(tiny.points[i].point.longitudinal_velocity_mps,0);
    EXPECT_TRUE(std::isfinite(tiny.points[i].point.longitudinal_velocity_mps));
    EXPECT_LE(tiny.points[i].point.longitudinal_velocity_mps,path.points[i].point.longitudinal_velocity_mps);
  }
  path.points[1].point.longitudinal_velocity_mps=0;
  EXPECT_THROW(lc::make_maneuver_braking_proposal(path,0,10,-6,-4,-10,2),std::logic_error);
}

TEST(ManeuverSafety, MissingOrInvalidPositiveReleaseLimitFailsClosed) {
  for(double jerk:{0.0,-1.0,std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(lc::make_maneuver_braking_proposal(line(30),0,10,-6,-4,-10,jerk),std::logic_error);
  }
}

TEST(ManeuverSafety, NominalDecelerationBoundaryHasContinuousSpeedProfile) {
  const auto path=line(30);
  const auto below=lc::make_maneuver_braking_proposal(path,0,10,-4.000001,-4,-10,100);
  const auto exact=lc::make_maneuver_braking_proposal(path,0,10,-4,-4,-10,100);
  const auto above=lc::make_maneuver_braking_proposal(path,0,10,-3.999999,-4,-10,100);
  for(size_t i=0;i<path.points.size();++i) {
    EXPECT_NEAR(below.points[i].point.longitudinal_velocity_mps,
                exact.points[i].point.longitudinal_velocity_mps,1e-5);
    EXPECT_NEAR(above.points[i].point.longitudinal_velocity_mps,
                exact.points[i].point.longitudinal_velocity_mps,1e-5);
  }
}

TEST(ManeuverSafety, RejectionDoesNotCommit) {
  auto output=line(10);
  auto proposal=line(5);
  EXPECT_FALSE(lc::accept_checked_maneuver(output,proposal,[](const auto&){return false;}));
  EXPECT_EQ(output.points[5].point.longitudinal_velocity_mps,10);
  EXPECT_TRUE(lc::accept_checked_maneuver(output,proposal,[](const auto&){return true;}));
  EXPECT_EQ(output.points[5].point.longitudinal_velocity_mps,5);
}

TEST(ManeuverSafety, EgoProjectionIsNotNearestVertex) {
  const auto path=line(10);
  geometry_msgs::Point ego;
  ego.x=.7;
  const auto anchored=lc::anchor_maneuver_at_ego(path,0,ego);
  EXPECT_NEAR(anchored.first.points[anchored.second].point.pose.position.x,.7,1e-6);
  const auto predictions=lc::maneuver_motion_hypotheses(
    anchored.first,anchored.second,31,5,0,30,0,.5);
  EXPECT_NEAR(predictions[1].front().pose.position.x,.7,1e-6);
}

TEST(ManeuverSafety, BoundedHorizonDoesNotRequireEntireLongManeuverPrediction) {
  const auto path=line(10);
  const auto p=lc::maneuver_motion_hypotheses(path,0,80,10,0,30,2.5,.5,2.9);
  EXPECT_DOUBLE_EQ(p[0].back().time,2.9);
  EXPECT_NEAR(p[0].back().pose.position.x,29,1e-6);
  EXPECT_THROW(lc::maneuver_motion_hypotheses(path,0,80,10,0,30,2.5,.5,2.49),std::logic_error);
  EXPECT_DOUBLE_EQ(lc::maneuver_motion_hypotheses(path,0,80,10,0,30,2.5,.5,2.5)[0].back().time,2.5);
}

TEST(ManeuverSafety, BoundedObservationHoldsRealStopEvenIfLaterPathHasPositiveSpeed) {
  auto path=line(10);
  path.points[10].point.longitudinal_velocity_mps=0;
  const auto p=lc::maneuver_motion_hypotheses(path,0,80,10,0,30,2.5,.5,2.8);
  EXPECT_DOUBLE_EQ(p[0].back().time,2.8);
  EXPECT_NEAR(p[0].back().pose.position.x,10,1e-6);
  EXPECT_DOUBLE_EQ(p[0].back().velocity,0);
  EXPECT_NEAR(p[1].back().pose.position.x,28,1e-6);
}

TEST(ManeuverSafety, BoundedObservationContinuesPastShortLcWithoutPoseClamp) {
  const auto path=line(10);
  const auto p=lc::maneuver_motion_hypotheses(path,0,5,10,0,30,2.5,.5,3.0);
  EXPECT_NEAR(p[0].back().pose.position.x,30,1e-6);
  EXPECT_DOUBLE_EQ(p[0].back().velocity,10);
}

TEST(ManeuverSafety, ExactlyThreeSecondObjectEndpointIsPreserved) {
  autoware_perception_msgs::PredictedObject object;
  object.shape.type=autoware_perception_msgs::Shape::BOUNDING_BOX;
  object.shape.dimensions.x=4;object.shape.dimensions.y=2;object.shape.dimensions.z=1;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w=1;
  autoware_perception_msgs::PredictedPath prediction;
  prediction.time_step.nanosec=100000000;
  for(int i=0;i<=30;++i){geometry_msgs::Pose p;p.position.x=i;p.orientation.w=1;prediction.path.push_back(p);}
  object.kinematics.predicted_paths.push_back(prediction);
  const auto observed=lc::observed_maneuver_object(object,.5);
  ASSERT_EQ(observed.predicted_paths.size(),1u);
  EXPECT_EQ(observed.predicted_paths[0].path.size(),7u);
  EXPECT_DOUBLE_EQ(observed.predicted_paths[0].path.back().time,3.0);
  EXPECT_DOUBLE_EQ(observed.predicted_paths[0].path.back().pose.position.x,30.0);
}

TEST(ManeuverSafety, SpatialTailRoadCeilingDoesNotBecomeInstant150Kph) {
  auto path=line(41.667);
  for(size_t i=0;i<=10;++i)path.points[i].point.longitudinal_velocity_mps=10;
  const auto model=lc::maneuver_prediction_profile(path,0,10,0,true,10,0,0,2,4);
  for(size_t i=0;i<=10;++i)EXPECT_DOUBLE_EQ(model.points[i].point.longitudinal_velocity_mps,10);
  const auto prediction=lc::maneuver_motion_hypotheses(model,0,10,10,0,41.667,2.5,.5,3.0);
  EXPECT_LE(prediction[0].back().velocity,14.01);
  EXPECT_LT(prediction[0].back().pose.position.x,35);
  EXPECT_FLOAT_EQ(path.points[11].point.longitudinal_velocity_mps,41.667f);
}

TEST(ManeuverSafety, LegacyPrepareRoadCeilingUsesSelectedAcceleration) {
  const auto path=line(41.667);
  const auto model=lc::maneuver_prediction_profile(path,0,60,20,false,10,0,1,2,4);
  for(size_t i=0;i<=20;++i)EXPECT_DOUBLE_EQ(model.points[i].point.longitudinal_velocity_mps,10);
  const auto prediction=lc::maneuver_motion_hypotheses(model,0,60,10,0,41.667,2.5,.5,3.0);
  EXPECT_NEAR(prediction[0].back().velocity,11,1e-3);
  EXPECT_NEAR(prediction[0].back().pose.position.x,30.5,1e-3);
}

TEST(ManeuverSafety, CompletedLcActiveTickChecksReachableTailInsteadOfFailure) {
  const auto path=line(41.667);
  const auto model=lc::maneuver_prediction_profile(path,10,5,0,true,10,0,0,2,4);
  const auto prediction=lc::maneuver_motion_hypotheses(model,10,10,10,0,41.667,2.5,.5,3.0);
  EXPECT_NEAR(prediction[0].front().velocity,10,1e-6);
  EXPECT_NEAR(prediction[0].back().velocity,16,1e-3);
  EXPECT_NEAR(prediction[0].back().pose.position.x,49,1e-3);
}

TEST(ManeuverSafety, AbortUsesItsOwnCoastingModelNotOldPositiveLcAcceleration) {
  autoware::behavior_path_planner::LaneChangePath original;
  original.path=line(41.667);
  original.type=autoware::behavior_path_planner::lane_change::PathType::SpatialSpeedProfile;
  original.info.longitudinal_acceleration.prepare=2;
  original.info.longitudinal_acceleration.lane_changing=3;
  original.info.duration.prepare=3;
  const auto abort=lc::abort_prediction_metadata(original);
  EXPECT_EQ(abort.path,original.path);
  EXPECT_DOUBLE_EQ(abort.info.duration.prepare,0);
  EXPECT_DOUBLE_EQ(original.info.duration.prepare,3);
  EXPECT_EQ(original.info.longitudinal_acceleration.lane_changing,3);
  const auto model=lc::maneuver_prediction_profile(abort.path,0,60,10,false,10,
    abort.info.longitudinal_acceleration.prepare,abort.info.longitudinal_acceleration.lane_changing,2,4);
  const auto p=lc::maneuver_motion_hypotheses(model,0,60,10,1,41.667,2.5,.5,3.0);
  EXPECT_NEAR(p[0].back().velocity,10,1e-6);
  EXPECT_NEAR(p[1].back().velocity,13,1e-6);
}

TEST(ManeuverSafety, FailureReasonJsonEscapesQuotesBackslashesAndControlBytes) {
  EXPECT_EQ(lc::safety_json_string("a\"\\\n\t"),"\"a\\\"\\\\\\u000a\\u0009\"");
}

TEST(ManeuverSafety, PlanarStationsPreserveGradeWithoutMovingAcrossSegments) {
  auto path=line(10);
  for(size_t i=0;i<path.points.size();++i)path.points[i].point.pose.position.z=28.4+.001*i;
  const auto point=lc::safety_pose_at_arc(path,5.25);
  EXPECT_NEAR(point.position.x,5.25,1e-12);
  EXPECT_NEAR(point.position.z,28.40525,1e-12);
  EXPECT_THROW(lc::safety_pose_at_arc(path,std::numeric_limits<double>::quiet_NaN()),std::logic_error);
}

TEST(ManeuverSafety, ExactVertexAndRepeatedAnchoringWorkOnGradedPath) {
  auto path=line(10);
  for(size_t i=0;i<path.points.size();++i)path.points[i].point.pose.position.z=28.4+.001*i;
  const auto ego=path.points[5].point.pose.position;
  // The old 2-D station -> 3-D interpolation shifts this vertex micrometres
  // behind segment 5 and insertTargetPoint rejects it as a sharp angle.
  const auto anchored=lc::anchor_maneuver_at_ego(path,5,ego);
  EXPECT_EQ(anchored.second,5u);
  EXPECT_EQ(anchored.first.points.size(),path.points.size());
  const auto again=lc::anchor_maneuver_at_ego(anchored.first,anchored.second,ego);
  EXPECT_EQ(again.first.points.size(),path.points.size());
  EXPECT_DOUBLE_EQ(again.first.points[again.second].point.pose.position.x,5);
}

TEST(ManeuverSafety, BothMotionHypothesesUseTheSamePlanarMetricOnGrade) {
  auto path=line(10);
  for(size_t i=0;i<path.points.size();++i)path.points[i].point.pose.position.z=.5*i;
  const auto predictions=lc::maneuver_motion_hypotheses(path,0,80,10,0,30,2.5,.5,3.0);
  for(const auto & prediction:predictions) {
    EXPECT_NEAR(prediction.back().pose.position.x,30,1e-9);
    EXPECT_NEAR(prediction.back().pose.position.z,15,1e-9);
  }
}

TEST(ManeuverSafety, RisingAnchorIsOptInAndDoesNotChangeExistingVertices) {
  auto path=line(19.257547);
  path.points[1].point.longitudinal_velocity_mps=19.424622;
  geometry_msgs::Point ego; ego.x=.561264;
  const auto old=lc::anchor_maneuver_at_ego(path,0,ego);
  const auto anchored=lc::anchor_maneuver_at_ego(path,0,ego,true);
  EXPECT_FLOAT_EQ(old.first.points[old.second].point.longitudinal_velocity_mps,19.257547f);
  EXPECT_NEAR(anchored.first.points[anchored.second].point.longitudinal_velocity_mps,19.351498,2e-6);
  const auto again=lc::anchor_maneuver_at_ego(anchored.first,anchored.second,ego,true);
  EXPECT_EQ(again.first.points.size(),anchored.first.points.size());
  EXPECT_FLOAT_EQ(again.first.points[again.second].point.longitudinal_velocity_mps,
    anchored.first.points[anchored.second].point.longitudinal_velocity_mps);
  ego.x=1;
  const auto vertex=lc::anchor_maneuver_at_ego(path,0,ego,true);
  EXPECT_EQ(vertex.first.points.size(),path.points.size());
  EXPECT_FLOAT_EQ(vertex.first.points[vertex.second].point.longitudinal_velocity_mps,19.424622f);
}

TEST(ManeuverSafety, RisingAnchorPreservesStopsAndDescendingCeilings) {
  geometry_msgs::Point ego;ego.x=.5;
  for(const auto speeds: {std::pair<double,double>{0,10},{10,0},{10,5}}) {
    auto path=line(speeds.first);path.points[1].point.longitudinal_velocity_mps=speeds.second;
    const auto anchored=lc::anchor_maneuver_at_ego(path,0,ego,true);
    EXPECT_FLOAT_EQ(anchored.first.points[anchored.second].point.longitudinal_velocity_mps,speeds.first);
    EXPECT_FLOAT_EQ(anchored.first.points.back().point.longitudinal_velocity_mps,speeds.first);
  }
}

TEST(ManeuverSafety, FreshBrakingEnvelopeDoesNotReuseOldDynamicSpeeds) {
  auto old=line(5);const auto immutable=old;
  EXPECT_THROW(lc::make_maneuver_braking_proposal(old,0,10,0,-4,-10,100),std::logic_error);
  const auto fresh=lc::maneuver_braking_hard_caps(old,0,std::vector<double>(old.points.size(),20));
  const auto braking=lc::make_maneuver_braking_proposal(fresh,0,10,0,-4,-10,100);
  EXPECT_FLOAT_EQ(braking.points.front().point.longitudinal_velocity_mps,10);
  EXPECT_EQ(old,immutable);
  for(size_t i=0;i<old.points.size();++i) {
    EXPECT_EQ(braking.points[i].point.pose,old.points[i].point.pose);
    EXPECT_EQ(braking.points[i].lane_ids,old.points[i].lane_ids);
  }
}

TEST(ManeuverSafety, FreshBrakingEnvelopeRetainsOriginalAndNewStops) {
  auto old=line(5);old.points[60].point.longitudinal_velocity_mps=0;
  auto caps=std::vector<double>(old.points.size()-2,20);caps[3]=0;
  const auto fresh=lc::maneuver_braking_hard_caps(old,2,caps);
  EXPECT_EQ(fresh.points[0],old.points[0]);EXPECT_EQ(fresh.points[1],old.points[1]);
  EXPECT_FLOAT_EQ(fresh.points[5].point.longitudinal_velocity_mps,0);
  EXPECT_FLOAT_EQ(fresh.points[60].point.longitudinal_velocity_mps,0);
  EXPECT_THROW(lc::make_maneuver_braking_proposal(fresh,2,10,0,-4,-10,100),std::logic_error);
}

TEST(ManeuverSafety, FreshBrakingEnvelopeRejectsInvalidCapsAtomically) {
  const auto old=line(5);const auto immutable=old;
  EXPECT_THROW(lc::maneuver_braking_hard_caps(old,0,{20}),std::logic_error);
  for(double bad:{-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    auto caps=std::vector<double>(old.points.size(),20);caps.back()=bad;
    EXPECT_THROW(lc::maneuver_braking_hard_caps(old,0,caps),std::logic_error);
    EXPECT_EQ(old,immutable);
  }
}
