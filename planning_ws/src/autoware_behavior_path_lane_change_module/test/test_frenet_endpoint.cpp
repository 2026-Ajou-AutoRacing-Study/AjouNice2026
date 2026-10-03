#include "autoware/behavior_path_lane_change_module/utils/frenet_endpoint.hpp"
#include <gtest/gtest.h>

namespace ep=autoware::behavior_path_planner::utils::lane_change::frenet_endpoint;
namespace fp=autoware::frenet_planner;
using Spline=autoware::sampler_common::transform::Spline2D;
namespace {
Spline reference(){return Spline({0.,50.,100.,150.},{0.,0.,0.,0.});}
fp::Trajectory sample(double duration=3.4473348451214987,double resolution=.5) {
  fp::FrenetState initial,target;initial.position={0.,3.317346872438649};
  initial.longitudinal_velocity=target.longitudinal_velocity=28.227679518594158;
  target.position={initial.longitudinal_velocity*duration,0.};
  target.lateral_velocity=-initial.position.d/duration;
  auto out=fp::generateCandidate(initial,target,duration,resolution);
  out.sampling_parameter.target_duration=duration;out.sampling_parameter.target_state=target;
  fp::calculateCartesian(reference(),out);return out;
}
}
TEST(FrenetEndpoint, NonIntegralTerminalPreservesPolynomialAndOriginalXY) {
  auto value=sample();const auto before=value;ASSERT_EQ(value.times.back(),3.);
  ASSERT_TRUE(ep::complete(reference(),value));ASSERT_EQ(value.times.size(),8U);
  EXPECT_DOUBLE_EQ(value.times.back(),before.sampling_parameter.target_duration);
  for(size_t i=0;i<before.times.size();++i) {
    EXPECT_DOUBLE_EQ(value.times[i],before.times[i]);EXPECT_EQ(value.points[i],before.points[i]);
    EXPECT_DOUBLE_EQ(value.frenet_points[i].s,before.frenet_points[i].s);
    EXPECT_DOUBLE_EQ(value.frenet_points[i].d,before.frenet_points[i].d);
  }
  EXPECT_NEAR(value.frenet_points.back().d,0.,1e-12);
  EXPECT_NEAR(value.lateral_polynomial->velocity(value.times.back()),-.96229319792742984,1e-12);
  EXPECT_GT(value.lengths.back(),before.lengths.back());
  EXPECT_EQ(value.poses.size(),value.times.size());EXPECT_EQ(value.jerks.size(),value.times.size());
  EXPECT_FALSE(ep::complete(reference(),value));
}
TEST(FrenetEndpoint, ExactGridIsUnchangedAndIdempotent) {
  auto value=sample(3.);const auto before=value;
  EXPECT_FALSE(ep::complete(reference(),value));EXPECT_EQ(value.times,before.times);
  EXPECT_EQ(value.poses,before.poses);EXPECT_EQ(value.points,before.points);
  EXPECT_EQ(value.longitudinal_velocities,before.longitudinal_velocities);
}
TEST(FrenetEndpoint, FloatingGridDoesNotCreateTinyExtraSegment) {
  auto value=sample(.7,.1);const auto count=value.times.size();
  ep::complete(reference(),value);EXPECT_EQ(value.times.size(),count);
  EXPECT_DOUBLE_EQ(value.times.back(),.7);EXPECT_FALSE(ep::complete(reference(),value));
}
TEST(FrenetEndpoint, NativeCartesianDerivedArraysAreRecomputedExactly) {
  auto value=sample(),expected=value;const double T=value.sampling_parameter.target_duration;
  expected.times.push_back(T);expected.frenet_points.emplace_back(
    expected.longitudinal_polynomial->position(T),expected.lateral_polynomial->position(T));
  expected.points.clear();expected.poses.clear();expected.yaws.clear();expected.lengths.clear();
  expected.curvatures.clear();expected.longitudinal_velocities.clear();expected.lateral_velocities.clear();
  expected.longitudinal_accelerations.clear();expected.lateral_accelerations.clear();expected.jerks.clear();
  fp::calculateCartesian(reference(),expected);ASSERT_TRUE(ep::complete(reference(),value));
  EXPECT_EQ(value.poses,expected.poses);EXPECT_EQ(value.curvatures,expected.curvatures);
  EXPECT_EQ(value.longitudinal_velocities,expected.longitudinal_velocities);
  EXPECT_EQ(value.lateral_accelerations,expected.lateral_accelerations);
}
TEST(FrenetEndpoint, MissingPolynomialAndSizeMismatchRejectAtomically) {
  auto value=sample();value.lateral_polynomial.reset();const auto times=value.times;
  EXPECT_THROW(ep::complete(reference(),value),std::logic_error);EXPECT_EQ(value.times,times);
  value=sample();value.jerks.pop_back();const auto poses=value.poses;
  EXPECT_THROW(ep::complete(reference(),value),std::logic_error);EXPECT_EQ(value.poses,poses);
}
TEST(FrenetEndpoint, NonfiniteAndNonmonotoneReject) {
  auto value=sample();value.times[2]=value.times[1];
  EXPECT_THROW(ep::complete(reference(),value),std::logic_error);
  value=sample();value.frenet_points[2].s=value.frenet_points[1].s;
  EXPECT_THROW(ep::complete(reference(),value),std::logic_error);
  value=sample();value.poses[2].position.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ep::complete(reference(),value),std::logic_error);
  value=sample();value.sampling_parameter.target_duration=std::numeric_limits<double>::infinity();
  EXPECT_THROW(ep::complete(reference(),value),std::logic_error);
  value=sample();value.sampling_parameter.target_duration=2.;
  EXPECT_THROW(ep::complete(reference(),value),std::logic_error);
}
TEST(FrenetEndpoint, MissingReferenceCoverageAndDifferentGeometryRejectAtomically) {
  auto value=sample();const auto before=value;
  EXPECT_THROW(ep::complete(Spline({0.,20.,40.},{0.,0.,0.}),value),std::logic_error);
  EXPECT_EQ(value.points,before.points);
  EXPECT_THROW(ep::complete(Spline({0.,50.,100.,150.},{1.,1.,1.,1.}),value),std::logic_error);
  EXPECT_EQ(value.times,before.times);EXPECT_EQ(value.poses,before.poses);
}
TEST(FrenetEndpoint, OptInRequiresEveryNormalSafetyScope) {
  EXPECT_TRUE(ep::enabled(true,true,true,true,true,true));
  for(size_t disabled=0;disabled<6;++disabled) {
    bool values[6]={true,true,true,true,true,true};values[disabled]=false;
    EXPECT_FALSE(ep::enabled(values[0],values[1],values[2],values[3],values[4],values[5]));
  }
}
