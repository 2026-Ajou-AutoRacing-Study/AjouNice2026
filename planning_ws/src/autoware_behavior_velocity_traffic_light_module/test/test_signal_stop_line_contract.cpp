// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
// Pure geometry tests: no ROS master, simulator, control or native API.
#include <autoware/behavior_velocity_traffic_light_module/signal_stop_line_contract.hpp>
#include <gtest/gtest.h>
#include <limits>
namespace sc=autoware::behavior_velocity_planner::signal_compliance;
namespace {
sc::StopLineContract contract()
{
  sc::StopLineContract c;
  c.regulatory_id=201365;c.expected_stop_line={{10.,-3.},{10.,3.}};
  c.crossing_line={{17.,-3.},{17.,3.}};c.additional_stop_margin_m=.75;
  return c;
}
}
TEST(SignalStopLineContract, ValidExplicitContract)
{
  EXPECT_TRUE(sc::validStopLineContract(contract()));
}
TEST(SignalStopLineContract, AnchorIsEndpointOrderInvariant)
{
  const auto c=contract();
  EXPECT_TRUE(sc::stopLineAnchorMatches(c,{{10.,-3.},{10.,3.}}));
  EXPECT_TRUE(sc::stopLineAnchorMatches(c,{{10.,3.},{10.,-3.}}));
}
TEST(SignalStopLineContract, WrongMapOrShiftedAnchorDoesNotMatch)
{
  EXPECT_FALSE(sc::stopLineAnchorMatches(contract(),{{10.3,-3.},{10.3,3.}}));
  EXPECT_FALSE(sc::stopLineAnchorMatches(contract(),{}));
}
TEST(SignalStopLineContract, InvalidContractDoesNotQualify)
{
  auto c=contract();c.valid=false;EXPECT_FALSE(sc::validStopLineContract(c));
  c=contract();c.additional_stop_margin_m=-.1;EXPECT_FALSE(sc::validStopLineContract(c));
  c=contract();c.max_anchor_error_m=std::numeric_limits<double>::infinity();EXPECT_FALSE(sc::validStopLineContract(c));
  c=contract();c.crossing_line[1].x=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(sc::validStopLineContract(c));
}
TEST(SignalStopLineContract, CoincidentVirtualAndPhysicalDoesNotInventDirection)
{
  auto c=contract();c.crossing_line=c.expected_stop_line;
  EXPECT_FALSE(sc::validStopLineContract(c));
}
TEST(SignalStopLineContract, PassingVirtualLineDoesNotPassPhysicalPaint)
{
  auto d=sc::finiteCrossingDistance(contract(),{11.,0.});ASSERT_TRUE(d);EXPECT_DOUBLE_EQ(*d,6.);
}
TEST(SignalStopLineContract, PhysicalPaintSideSurvivesHorizonLoss)
{
  const auto c=contract();
  auto d=sc::finiteCrossingDistance(c,{17.1,0.});ASSERT_TRUE(d);EXPECT_NEAR(*d,-.1,1e-12);
  EXPECT_FALSE(sc::pathCrossingFrontDistance(c,{{11.,0.},{14.,0.}},{11.,0.},3.));
}
TEST(SignalStopLineContract, OffFiniteSegmentIsNotCrossingEvidence)
{
  EXPECT_FALSE(sc::finiteCrossingDistance(contract(),{18.,8.}));
}
TEST(SignalStopLineContract, LineEndpointOrderDoesNotChangeCrossingSign)
{
  auto c=contract();std::reverse(c.crossing_line.begin(),c.crossing_line.end());
  auto d=sc::finiteCrossingDistance(c,{11.,0.});ASSERT_TRUE(d);EXPECT_DOUBLE_EQ(*d,6.);
}
TEST(SignalStopLineContract, YellowEtaUsesPhysicalPathDistanceNotVirtualStopTarget)
{
  auto d=sc::pathCrossingFrontDistance(contract(),{{-10.,0.},{0.,0.},{10.,0.},{20.,0.}},{0.,0.},3.);
  ASSERT_TRUE(d);EXPECT_DOUBLE_EQ(*d,14.);
}
TEST(SignalStopLineContract, ShortPathCannotSubstituteVirtualLineForYellowEta)
{
  EXPECT_FALSE(sc::pathCrossingFrontDistance(contract(),{{-10.,0.},{10.,0.},{14.,0.}},{0.,0.},3.));
}
TEST(SignalStopLineContract, ThreePointPaintPolylineAndFarSegment)
{
  auto c=contract();c.crossing_line={{17.,3.},{17.,-3.},{17.,-10.}};
  EXPECT_TRUE(sc::validStopLineContract(c));
  auto d=sc::finiteCrossingDistance(c,{16.,-8.});ASSERT_TRUE(d);EXPECT_DOUBLE_EQ(*d,1.);
  d=sc::pathCrossingFrontDistance(c,{{0.,-8.},{20.,-8.}},{0.,-8.},3.);
  ASSERT_TRUE(d);EXPECT_DOUBLE_EQ(*d,14.);
}
