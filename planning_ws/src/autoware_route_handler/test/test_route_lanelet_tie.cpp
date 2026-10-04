#include <autoware/route_handler/route_lanelet_tie.hpp>
#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{
struct Point
{
  double px, py;
  double x() const { return px; }
  double y() const { return py; }
};
using Points = std::vector<Point>;
using autoware::route_handler::detail::routeLaneletTie;
using autoware::route_handler::detail::routeLaneletTieLess;

TEST(RouteLaneletTie, HeadingRejectsWrongDirectionWithinOverlap)
{
  const auto forward = routeLaneletTie(Points{{0, 0}, {10, 0}}, 5, 0.8, 0, 2);
  const auto reverse = routeLaneletTie(Points{{10, 0.8}, {0, 0.8}}, 5, 0.8, 0, 1);
  EXPECT_TRUE(routeLaneletTieLess(forward, reverse));
  EXPECT_FALSE(routeLaneletTieLess(reverse, forward));
}

TEST(RouteLaneletTie, SegmentProjectionNotVertexDistance)
{
  const auto sparse = routeLaneletTie(Points{{0, 0}, {100, 0}}, 50, 0.1, 0, 2);
  const auto sampled = routeLaneletTie(Points{{49, 1}, {50, 1}, {51, 1}}, 50, 0.1, 0, 1);
  EXPECT_NEAR(sparse.squared_distance, 0.01, 1e-12);
  EXPECT_TRUE(routeLaneletTieLess(sparse, sampled));
}

TEST(RouteLaneletTie, StableFinalIdIndependentOfInputOrder)
{
  const auto low = routeLaneletTie(Points{{0, 0}, {10, 0}}, 5, 1, 0, 1);
  const auto high = routeLaneletTie(Points{{0, 0}, {10, 0}}, 5, 1, 0, 2);
  EXPECT_TRUE(routeLaneletTieLess(low, high));
  EXPECT_FALSE(routeLaneletTieLess(high, low));
}

TEST(RouteLaneletTie, NearForwardCenterWinsDespiteSmallHeadingDifference)
{
  for (const double yaw : {-0.01, 0.0, 0.01, 0.02}) {
    const auto near = routeLaneletTie(Points{{0, 0}, {10, 0.1}}, 5, 0.1, yaw, 2);
    const auto far = routeLaneletTie(Points{{0, 1.1}, {10, 1.1}}, 5, 0.1, yaw, 1);
    EXPECT_TRUE(routeLaneletTieLess(near, far));
  }
}

TEST(RouteLaneletTie, WrappedNegativeYawPreservesDirection)
{
  const auto same = routeLaneletTie(Points{{10, 0}, {0, 0}}, 5, 0, -3.141592653589793, 2);
  const auto opposite = routeLaneletTie(Points{{0, 0}, {10, 0}}, 5, 0, -3.141592653589793, 1);
  EXPECT_NEAR(same.heading_error, 0, 1e-12);
  EXPECT_TRUE(routeLaneletTieLess(same, opposite));
}

TEST(RouteLaneletTie, MissingDegenerateAndNonfiniteCenterline)
{
  const auto valid = routeLaneletTie(Points{{0, 0}, {10, 0}}, 5, 0, 0, 4);
  for (const Points points : {Points{}, Points{{0, 0}}, Points{{0, 0}, {0, 0}},
       Points{{0, 0}, {std::numeric_limits<double>::quiet_NaN(), 1}}}) {
    const auto invalid = routeLaneletTie(points, 5, 0, 0, 1);
    EXPECT_FALSE(invalid.valid);
    EXPECT_TRUE(routeLaneletTieLess(valid, invalid));
  }
}

TEST(RouteLaneletTie, R66SamePoseProjectedReferenceDirections)
{
  // Same ego and measured BPP projections, not nearest-vertex distances.
  const double x = 198.50586111424096, y = 652.2157028351824;
  const auto make_line = [](double px, double py, double yaw) {
    return Points{{px - 10 * std::cos(yaw), py - 10 * std::sin(yaw)},
                  {px + 10 * std::cos(yaw), py + 10 * std::sin(yaw)}};
  };
  const auto target = routeLaneletTie(
    make_line(198.45122579773903, 652.2148595815274, -1.5553633292418605),
    x, y, -1.5651897506329175, 201467);
  const auto old_current = routeLaneletTie(
    make_line(197.48814725574002, 652.1448617291436, -1.5013003463489054),
    x, y, -1.5651897506329175, 201468);
  EXPECT_NEAR(std::sqrt(target.squared_distance), 0.05464182359688235, 1e-9);
  EXPECT_NEAR(std::sqrt(old_current.squared_distance), 1.0201764357647576, 1e-9);
  EXPECT_TRUE(routeLaneletTieLess(target, old_current));
}
}  // namespace
