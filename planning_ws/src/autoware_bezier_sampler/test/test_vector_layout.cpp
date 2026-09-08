#include <gtest/gtest.h>

#include <autoware_bezier_sampler/bezier.hpp>

#include <Eigen/Core>

#include <vector>

TEST(BezierVectorConstructor, PreservesObservedUpstreamStorageLayout)
{
  const std::vector<Eigen::Vector2d> control_points{
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}, {5.0, 0.0}};
  const autoware::bezier_sampler::Bezier bezier(control_points);
  const auto midpoint = bezier.value(0.5);
  EXPECT_DOUBLE_EQ(midpoint.x(), 1.25);
  EXPECT_DOUBLE_EQ(midpoint.y(), 1.75);
}
