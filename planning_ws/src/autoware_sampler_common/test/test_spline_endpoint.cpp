#include <autoware_sampler_common/transform/spline_transform.hpp>
#include <gtest/gtest.h>

#include <cmath>

using autoware::sampler_common::transform::Spline;
using autoware::sampler_common::transform::Spline2D;

TEST(SplineEndpoint, DerivativesAtFirstKnot)
{
  const std::vector<double> s{0.0, 1.0, 2.0};
  const Spline spline(s, {4.0, 7.0, 10.0});
  EXPECT_NEAR(spline.velocity(0.0, s), 3.0, 1e-9);
  EXPECT_NEAR(spline.acceleration(0.0, s), 0.0, 1e-9);
}

TEST(SplineEndpoint, LateralRoundTripAtFirstKnotBothSidesAndHeadings)
{
  for (const double heading : {0.0, 0.4, -1.45, 2.7}) {
    const double dx = std::cos(heading), dy = std::sin(heading);
    const Spline2D spline({20.0, 20.0 + 10 * dx, 20.0 + 20 * dx},
                          {30.0, 30.0 + 10 * dy, 30.0 + 20 * dy});
    EXPECT_NEAR(spline.yaw(0.0), heading, 1e-9);
    EXPECT_NEAR(spline.curvature(0.0), 0.0, 1e-9);
    for (const double d : {-3.0, -0.2, 0.2, 3.0}) {
      const autoware::sampler_common::Point2d point{20.0 - d * dy, 30.0 + d * dx};
      const auto frenet = spline.frenet(point);
      const auto reconstructed = spline.cartesian(frenet);
      EXPECT_NEAR(reconstructed.x(), point.x(), 0.01);
      EXPECT_NEAR(reconstructed.y(), point.y(), 0.01);
    }
  }
}
