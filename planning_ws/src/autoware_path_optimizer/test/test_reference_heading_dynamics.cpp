// Copyright 2026 AjouNice2026
// Licensed under the Apache License, Version 2.0.
#include "autoware/path_optimizer/mpt_optimizer.hpp"
#include "autoware/path_optimizer/state_equation_generator.hpp"

#include <gtest/gtest.h>
#include <cmath>

namespace autoware::path_optimizer
{
namespace
{
ReferencePoint reference(double yaw, double curvature, double distance)
{
  ReferencePoint p;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.curvature = curvature;
  p.delta_arc_length = distance;
  return p;
}

double nextWorldHeading(const std::vector<ReferencePoint> & refs, double steer)
{
  StateEquationGenerator generator(3.0, 40.0 * M_PI / 180.0,
    std::make_shared<autoware_utils::TimeKeeper>());
  const auto m = generator.calcMatrix(refs);
  // Initial heading error is zero. Reconstruct world heading at the next point.
  return refs[1].getYaw() + m.B(3, 0) * steer + m.W(3);
}

TEST(ReferenceHeadingDynamics, StraightReferenceKeepsHeading)
{
  const auto p = reference(0.7, 0.0, 0.5);
  EXPECT_NEAR(nextWorldHeading({p, p}, 0.0), 0.7, 1e-12);
}

TEST(ReferenceHeadingDynamics, ConstantCurvatureTracksReferenceWithoutHeadingError)
{
  const double curvature = 0.1;
  const auto a = reference(0.7, curvature, 0.5);
  const auto b = reference(0.75, curvature, 0.5);
  EXPECT_NEAR(nextWorldHeading({a, b}, std::atan(3.0 * curvature)), b.getYaw(), 1e-12);
}

TEST(ReferenceHeadingDynamics, ChangingReferenceFrameDoesNotChangePredictedWorldHeading)
{
  // The reference frame's sampled rotation must not create a vehicle rotation.
  const auto a = reference(0.7, 0.1, 0.5);
  const auto b = reference(0.75, 0.1, 0.5);
  const auto differently_sampled_b = reference(0.72, 0.1, 0.5);
  EXPECT_NEAR(nextWorldHeading({a, b}, 0.04),
    nextWorldHeading({a, differently_sampled_b}, 0.04), 1e-12);
}

TEST(ReferenceHeadingDynamics, WrapAtPiUsesShortReferenceRotation)
{
  const auto a = reference(179.0 * M_PI / 180.0, 0.0, 0.5);
  const auto b = reference(-179.0 * M_PI / 180.0, 0.0, 0.5);
  const double predicted = nextWorldHeading({a, b}, 0.0);
  // A stationary steering input retains world heading across the angle wrap.
  EXPECT_NEAR(std::atan2(std::sin(predicted - a.getYaw()),
    std::cos(predicted - a.getYaw())), 0.0, 1e-12);
}
}  // namespace
}  // namespace autoware::path_optimizer

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
