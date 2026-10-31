// Copyright 2026 AjouNice2026 contributors
// Licensed under the Apache License, Version 2.0.
#include <autoware/behavior_velocity_traffic_light_module/signal_judging_line.hpp>
#include <autoware/behavior_velocity_traffic_light_module/signal_judging_line_ros.hpp>
#include <gtest/gtest.h>
#include <limits>
namespace sc=autoware::behavior_velocity_planner::signal_compliance;
namespace {
sc::SignalJudgingLine line()
{ return {201365,{{17.,-3.},{17.,3.}},{1.,0.},true}; }
}
TEST(SignalJudgingLine, ValidOriginalGeometryDoesNotDependOnVirtualStopTarget)
{
  const auto original=line();ASSERT_TRUE(sc::validJudgingLine(original));
  // Neither shifted virtual stop10 nor an even earlier stop7 is an input.
  EXPECT_DOUBLE_EQ(*sc::finiteJudgingLineDistance(original,{11.,0.}),6.);
}
TEST(SignalJudgingLine, FrontPointCrossingAndEndpointOrder)
{
  auto original=line();
  EXPECT_NEAR(*sc::finiteJudgingLineDistance(original,{17.1,0.}),-.1,1e-12);
  std::reverse(original.points.begin(),original.points.end());
  EXPECT_NEAR(*sc::finiteJudgingLineDistance(original,{17.1,0.}),-.1,1e-12);
  EXPECT_DOUBLE_EQ(*sc::finiteJudgingLineDistance(original,{11.,0.}),6.);
}
TEST(SignalJudgingLine, FiniteLineDoesNotClaimOffLaneCrossing)
{
  EXPECT_FALSE(sc::finiteJudgingLineDistance(line(),{18.,8.}));
}
TEST(SignalJudgingLine, YellowEtaUsesActualFrontPointWithoutDoubleOffset)
{
  auto d=sc::pathJudgingLineDistance(line(),{{-10.,0.},{0.,0.},{10.,0.},{20.,0.}},{3.,0.});
  ASSERT_TRUE(d);EXPECT_DOUBLE_EQ(*d,14.);
}
TEST(SignalJudgingLine, ShortHorizonCannotSubstituteEarlierMapLine)
{
  EXPECT_FALSE(sc::pathJudgingLineDistance(line(),{{-10.,0.},{10.,0.},{14.,0.}},{3.,0.}));
}
TEST(SignalJudgingLine, RouteArcIncludesCurveRatherThanEuclideanShortcut)
{
  const sc::SignalJudgingLine original{201365,{{10.,7.},{10.,13.}},{1.,0.},true};
  auto d=sc::pathJudgingLineDistance(original,{{0.,0.},{0.,10.},{20.,10.}},{0.,3.});
  ASSERT_TRUE(d);EXPECT_DOUBLE_EQ(*d,17.);
}
TEST(SignalJudgingLine, FarPathProjectionAndNonfiniteValuesCannotCreateEta)
{
  EXPECT_FALSE(sc::pathJudgingLineDistance(line(),{{0.,0.},{20.,0.}},{0.,4.}));
  EXPECT_FALSE(sc::pathJudgingLineDistance(line(),{{0.,0.},{20.,0.}},
    {std::numeric_limits<double>::quiet_NaN(),0.}));
}
TEST(SignalJudgingLine, OriginalThreePointDPaintUsesBothSegments)
{
  auto original=line();original.points={{17.,3.},{17.,-3.},{17.,-10.}};
  EXPECT_DOUBLE_EQ(*sc::finiteJudgingLineDistance(original,{16.,-8.}),1.);
  EXPECT_DOUBLE_EQ(*sc::pathJudgingLineDistance(original,{{0.,-8.},{20.,-8.}},{3.,-8.}),14.);
}
TEST(SignalJudgingLine, InvalidOrDuplicateProfilesNeverAuthorize)
{
  auto original=line();
  EXPECT_EQ(sc::findSignalJudgingLine({original},201351),nullptr);
  EXPECT_EQ(sc::findSignalJudgingLine({original,original},201365),nullptr);
  original.valid=false;EXPECT_EQ(sc::findSignalJudgingLine({original},201365),nullptr);
  original=line();original.approach_direction={0.,0.};EXPECT_FALSE(sc::validJudgingLine(original));
  original=line();original.approach_direction={0.,1.};EXPECT_FALSE(sc::validJudgingLine(original));
}

namespace {
XmlRpc::XmlRpcValue xmlProfile()
{
  XmlRpc::XmlRpcValue rows;rows.setSize(1);
  rows[0]["regulatory_id"]=201365;
  rows[0]["line_xy"].setSize(4);
  rows[0]["line_xy"][0]=17.;rows[0]["line_xy"][1]=-3.;
  rows[0]["line_xy"][2]=17.;rows[0]["line_xy"][3]=3.;
  rows[0]["approach_xy"].setSize(2);
  rows[0]["approach_xy"][0]=1.;rows[0]["approach_xy"][1]=0.;
  return rows;
}
}
TEST(SignalJudgingLineXml, AcceptsOnlyExplicitNumericOriginalGeometry)
{
  auto rows=xmlProfile();
  const auto lines=sc::parseSignalJudgingLines(rows);
  ASSERT_EQ(lines.size(),1U);
  ASSERT_NE(sc::findSignalJudgingLine(lines,201365),nullptr);
}
TEST(SignalJudgingLineXml, WrongRootAndMissingOrBoolFieldsNeverRefuseStartup)
{
  XmlRpc::XmlRpcValue malformed;malformed["not_a_list"]=1;
  EXPECT_TRUE(sc::parseSignalJudgingLines(malformed).empty());
  for(int invalid=0;invalid<5;++invalid) {
    auto rows=xmlProfile();
    if(invalid==0) rows[0]["regulatory_id"]=true;
    if(invalid==1) rows[0]["line_xy"][0]=true;
    if(invalid==2) { rows[0]["line_xy"]=XmlRpc::XmlRpcValue();rows[0]["line_xy"].setSize(3); }
    if(invalid==3) rows[0]["approach_xy"]=std::string("1,0");
    if(invalid==4) { rows[0]=XmlRpc::XmlRpcValue();rows[0]["regulatory_id"]=201365; }
    std::vector<sc::SignalJudgingLine> lines;
    EXPECT_NO_THROW(lines=sc::parseSignalJudgingLines(rows));
    EXPECT_EQ(sc::findSignalJudgingLine(lines,201365),nullptr);
  }
}
TEST(SignalJudgingLineXml, DuplicateAndNonfiniteRowsCannotAuthorize)
{
  auto duplicate=xmlProfile();const auto first=duplicate[0];duplicate.setSize(2);duplicate[1]=first;
  EXPECT_EQ(sc::findSignalJudgingLine(sc::parseSignalJudgingLines(duplicate),201365),nullptr);
  auto nonfinite=xmlProfile();nonfinite[0]["line_xy"][0]=std::numeric_limits<double>::infinity();
  EXPECT_EQ(sc::findSignalJudgingLine(sc::parseSignalJudgingLines(nonfinite),201365),nullptr);
}
