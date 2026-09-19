#include <gtest/gtest.h>

#include <autoware_frenet_planner/frenet_planner.hpp>
#include <autoware_frenet_planner/polynomials.hpp>

namespace fp = autoware::frenet_planner;

TEST(FrenetSemantics, QuinticEndpointAndDerivatives)
{
  const fp::Polynomial p(1.0, 2.0, -0.5, 9.0, -1.0, 0.25, 2.0);
  EXPECT_DOUBLE_EQ(p.position(0.0), 1.0);
  EXPECT_DOUBLE_EQ(p.velocity(0.0), 2.0);
  EXPECT_DOUBLE_EQ(p.acceleration(0.0), -0.5);
  EXPECT_NEAR(p.position(2.0), 9.0, 1e-12);
  EXPECT_NEAR(p.velocity(2.0), -1.0, 1e-12);
  EXPECT_NEAR(p.acceleration(2.0), 0.25, 1e-12);
}

TEST(FrenetSemantics, TimeAndArcLengthSamplingRemainInclusive)
{
  fp::FrenetState initial;
  initial.position = {0.0, 0.0};
  fp::FrenetState target;
  target.position = {1.0, 1.0};
  const auto trajectory = fp::generateCandidate(initial, target, 1.0, 0.25);
  ASSERT_EQ(trajectory.times.size(), 5U);
  ASSERT_EQ(trajectory.frenet_points.size(), 5U);
  EXPECT_DOUBLE_EQ(trajectory.times.front(), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.times.back(), 1.0);
  EXPECT_NEAR(trajectory.frenet_points.front().s, 0.0, 1e-12);
  EXPECT_NEAR(trajectory.frenet_points.back().s, 1.0, 1e-12);
  const auto path = fp::generateCandidate(initial, target, 0.5);
  ASSERT_EQ(path.frenet_points.size(), 2U);
  EXPECT_DOUBLE_EQ(path.frenet_points.front().s, 0.5);
  EXPECT_DOUBLE_EQ(path.frenet_points.back().s, 1.0);
}
