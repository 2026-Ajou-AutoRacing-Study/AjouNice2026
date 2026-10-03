#include "pid_based_planner/lead_identity_reset.hpp"
#include "pid_based_planner/pid_controller.hpp"
#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <gtest/gtest.h>
using namespace autoware::motion_velocity_planner;
using autoware::signal_processing::LowpassFilter1d;
TEST(LeadIdentityReset, OffAndSameIdentityPreserveNativeFiltering)
{
  std::optional<int> previous;int resets=0;
  auto apply=[&](bool enabled,std::optional<int> id){return reset_lead_history_if_changed(
    enabled,id,previous,[&]{++resets;});};
  EXPECT_FALSE(apply(false,31));EXPECT_EQ(resets,0);
  EXPECT_TRUE(apply(true,31));EXPECT_FALSE(apply(true,31));EXPECT_EQ(resets,1);
  EXPECT_TRUE(apply(true,36));EXPECT_EQ(resets,2);
}
TEST(LeadIdentityReset, NoObjectGapAndOffOnClearIdentity)
{
  std::optional<int> previous=31;int resets=0;
  auto reset=[&]{++resets;};
  EXPECT_FALSE(reset_lead_history_if_changed(true,std::optional<int>{},previous,reset));
  EXPECT_FALSE(previous);
  EXPECT_TRUE(reset_lead_history_if_changed(true,std::optional<int>{31},previous,reset));
  EXPECT_FALSE(reset_lead_history_if_changed(false,std::optional<int>{31},previous,reset));
  EXPECT_FALSE(previous);
  EXPECT_TRUE(reset_lead_history_if_changed(true,std::optional<int>{31},previous,reset));
  EXPECT_EQ(resets,2);
}
TEST(LeadIdentityReset, ActualR48RawStepHasNoForeignLeadHistory)
{
  LowpassFilter1d distance(.5),error(.5),velocity(.5),normalized(.2),output(.5);
  distance.filter(83.007);error.filter(30.492);velocity.filter(16.646);
  normalized.filter(.36);output.filter(27.582);
  std::optional<int> previous=31;
  PIDController pid(1.,1.,1.);pid.calc(30.492);
  double previous_acc=-2.;LowpassFilter1d output_acc(.5),output_jerk(.5);
  output_acc.filter(-2.);output_jerk.filter(-1.);
  ASSERT_TRUE(reset_lead_history_if_changed(true,std::optional<int>{36},previous,[&]{
    distance.reset();error.reset();velocity.reset();normalized.reset();output.reset();pid.reset();
  }));
  EXPECT_DOUBLE_EQ(distance.filter(11.675),11.675);
  EXPECT_DOUBLE_EQ(error.filter(-39.271),-39.271);
  EXPECT_DOUBLE_EQ(velocity.filter(16.602),16.602);
  const double n=-39.271/11.675;
  EXPECT_DOUBLE_EQ(normalized.filter(n),n);
  EXPECT_DOUBLE_EQ(output.filter(0.),0.);
  EXPECT_DOUBLE_EQ(pid.calc(-1.),-2.); // no prior integral/derivative
  EXPECT_DOUBLE_EQ(previous_acc,-2.);
  EXPECT_DOUBLE_EQ(output_acc.filter(-4.),-3.);
  EXPECT_DOUBLE_EQ(output_jerk.filter(-3.),-2.);
}
TEST(LeadIdentityReset, SameIdentityDangerousStepIsUnchangedNotClaimedSolved)
{
  LowpassFilter1d distance(.5);distance.filter(83.007);std::optional<int> previous=31;
  EXPECT_FALSE(reset_lead_history_if_changed(true,std::optional<int>{31},previous,[&]{distance.reset();}));
  EXPECT_NEAR(distance.filter(11.675),47.341,1e-12);
}
TEST(LeadIdentityReset, R48ConfiguredQuadraticPidImmediateNewLeadResponse)
{
  // Recorded raw inputs at 1790988105.756746; deployed kp10/ki0/kd2,
  // quadratic error, min target0. This is a scalar replay, not a collision proof.
  LowpassFilter1d distance(.5),error(.5),normalized(.2),output(.5);
  distance.filter(83.007);error.filter(30.492);normalized.filter(.36);output.filter(27.582);
  PIDController pid(10.,0.,2.);pid.calc(.36*.36);
  std::optional<int> previous=31;
  ASSERT_TRUE(reset_lead_history_if_changed(true,std::optional<int>{36},previous,[&]{
    distance.reset();error.reset();normalized.reset();output.reset();pid.reset();
  }));
  const double d=distance.filter(11.675),e=error.filter(-39.271);
  const double n=normalized.filter(e/d);
  const double command=pid.calc(-n*n);
  const double target=output.filter(std::max(0.,26.325+command));
  EXPECT_DOUBLE_EQ(target,0.);
  EXPECT_LT(target,26.871); // actual old-lead-smoothed published cap
}
