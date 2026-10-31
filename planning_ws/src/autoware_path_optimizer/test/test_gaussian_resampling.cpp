// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include "autoware/path_optimizer/utils/gaussian_resampling.hpp"
#include "autoware/path_optimizer/utils/trajectory_utils.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace autoware::path_optimizer
{
namespace
{
TrajectoryPoint point(double x, double y = 0.0, double speed = 8.0)
{
  TrajectoryPoint p;
  p.pose.position.x = x; p.pose.position.y = y; p.pose.position.z = 0.1*x;
  p.pose.orientation.w = 1.0;
  p.longitudinal_velocity_mps = speed;
  return p;
}

double peakCurvature(
  const std::vector<TrajectoryPoint> & points,
  double min_x = -std::numeric_limits<double>::infinity(),
  double max_x = std::numeric_limits<double>::infinity())
{
  double peak = 0.0;
  for (size_t i=1;i+1<points.size();++i) {
    const auto & a=points[i-1].pose.position;
    const auto & b=points[i].pose.position;
    const auto & c=points[i+1].pose.position;
    if (b.x < min_x || b.x > max_x) continue;
    const double denominator=std::hypot(b.x-a.x,b.y-a.y)*std::hypot(c.x-b.x,c.y-b.y)*
      std::hypot(c.x-a.x,c.y-a.y);
    if (denominator>1e-8) peak=std::max(peak,
      2*std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/denominator);
  }
  return peak;
}

TEST(GaussianResampling, StraightPathKeepsGeometryEndpointsAndState)
{
  std::vector<TrajectoryPoint> input;
  for(int i=0;i<=40;++i) input.push_back(point(i*0.5));
  input.back().longitudinal_velocity_mps=0.0;
  const auto output=trajectory_utils::resampleTrajectoryPointsGaussian(input,0.2,0.2,point(5).pose);
  ASSERT_GT(output.size(),input.size());
  EXPECT_DOUBLE_EQ(output.front().pose.position.x,0.0);
  EXPECT_NEAR(output.back().pose.position.x,20.0,1e-10);
  for(const auto & p:output) {
    EXPECT_NEAR(p.pose.position.y,0.0,1e-12);
    EXPECT_NEAR(p.pose.position.z,0.1*p.pose.position.x,1e-9);
  }
  EXPECT_DOUBLE_EQ(output.front().longitudinal_velocity_mps,8.0);
  EXPECT_DOUBLE_EQ(output.back().longitudinal_velocity_mps,0.0);
  EXPECT_LT(peakCurvature(output),1e-10);
}

TEST(GaussianResampling, ReducesShortWavelengthJitterAndAnchorsMeasuredState)
{
  std::vector<TrajectoryPoint> input;
  for(int i=0;i<=70;++i) input.push_back(point(i*0.5,0.01*std::sin(i*1.7)));
  const auto ego=input[10].pose;
  const auto old=trajectory_utils::resampleTrajectoryPoints(input,0.2,true);
  const auto output=trajectory_utils::resampleTrajectoryPointsGaussian(input,0.2,0.2,ego);
  ASSERT_EQ(old.size(),output.size());
  EXPECT_LT(peakCurvature(output,10.0,30.0),peakCurvature(old,10.0,30.0)*0.85);
  size_t anchored=0;
  for(size_t i=0;i<output.size();++i) {
    if(std::hypot(old[i].pose.position.x-ego.position.x,
                  old[i].pose.position.y-ego.position.y)<0.7) {
      ++anchored;
      EXPECT_NEAR(old[i].pose.position.x,output[i].pose.position.x,1e-8);
      EXPECT_NEAR(old[i].pose.position.y,output[i].pose.position.y,1e-8);
    }
  }
  EXPECT_GT(anchored,3U);
  EXPECT_NEAR(output.front().pose.position.x,input.front().pose.position.x,1e-10);
  EXPECT_NEAR(output.back().pose.position.y,input.back().pose.position.y,1e-10);
}

TEST(GaussianResampling, KeepsOffGridStopPositionAndNeverReleasesStop)
{
  std::vector<TrajectoryPoint> input;
  for(int i=0;i<=10;++i) input.push_back(point(i*0.5,0.02*std::sin(i*0.5)));
  input.push_back(point(5.35,0.02*std::sin(5.35),0.0));
  const auto stop=input.back().pose.position;
  for(int i=12;i<=30;++i) input.push_back(point(i*0.5,0.02*std::sin(i*0.5),0.0));
  const auto output=trajectory_utils::resampleTrajectoryPointsGaussian(input,0.2,0.2,input[4].pose);
  bool found=false;
  for(const auto & p:output) {
    if(!found && p.longitudinal_velocity_mps==0.0) {
      found=true;
      EXPECT_NEAR(p.pose.position.x,stop.x,1e-9);
      EXPECT_NEAR(p.pose.position.y,stop.y,1e-9);
      EXPECT_NEAR(p.pose.position.z,stop.z,1e-9);
    }
    if(found) EXPECT_DOUBLE_EQ(p.longitudinal_velocity_mps,0.0);
  }
  EXPECT_TRUE(found);
}

TEST(GaussianResampling, HandlesVeryShortAndRepeatedPoints)
{
  const std::vector<TrajectoryPoint> input{point(0),point(0),point(0.05,0,0)};
  const auto output=trajectory_utils::resampleTrajectoryPointsGaussian(input,0.2,0.2,input.front().pose);
  ASSERT_EQ(output.size(),2U);
  EXPECT_NEAR(output.front().pose.position.x,0.0,1e-12);
  EXPECT_NEAR(output.back().pose.position.x,0.05,1e-12);
  EXPECT_DOUBLE_EQ(output.back().longitudinal_velocity_mps,0.0);
}

TEST(GaussianResampling, RejectsInvalidSpatialParameters)
{
  const std::vector<TrajectoryPoint> input{point(0),point(1)};
  for(double sigma:{0.0,-1.0,std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(trajectory_utils::resampleTrajectoryPointsGaussian(input,0.2,sigma,input[0].pose),std::invalid_argument);
  EXPECT_THROW(trajectory_utils::resampleTrajectoryPointsGaussian(input,0.0,0.2,input[0].pose),std::invalid_argument);
}
}
}

int main(int argc,char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
