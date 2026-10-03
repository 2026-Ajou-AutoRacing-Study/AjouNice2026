#include "prediction_time.hpp"
#include <gtest/gtest.h>
#include <limits>

namespace bvp = autoware::behavior_velocity_planner;
namespace
{
autoware_perception_msgs::PredictedPath prediction()
{
  autoware_perception_msgs::PredictedPath p;
  p.time_step.nanosec = 100000000;
  for (int i = 0; i <= 30; ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = i * 0.1;
    pose.orientation.w = 1.0;
    p.path.push_back(pose);
  }
  return p;
}
}

TEST(RoundaboutPredictionTime, AlignsMeasuredHeaderWithoutExtendingCoverage)
{
  const auto p = bvp::alignRoundaboutPrediction(prediction(), .08, 10.0);
  ASSERT_TRUE(p);
  EXPECT_DOUBLE_EQ(p->times.front(), 0.0);
  EXPECT_NEAR(p->path.path.front().position.x, .08, 1e-10);
  EXPECT_NEAR(p->times.back(), 2.92, 1e-10);
  EXPECT_NEAR(p->path.path.back().position.x, 3.0, 1e-10);
  for (size_t i = 1; i < p->times.size(); ++i) {
    EXPECT_GT(p->times[i], p->times[i-1]);
    EXPECT_LE(p->times[i]-p->times[i-1], .100000001);
  }
}

TEST(RoundaboutPredictionTime, PreservesIrregularLastSampleAndExactCut)
{
  auto input = prediction();
  input.time_step.nanosec = 95000000;
  const auto p = bvp::alignRoundaboutPrediction(input, .081, 10.0);
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->times.back(), 2.769, 1e-10);
  EXPECT_DOUBLE_EQ(p->path.path.back().position.x, input.path.back().position.x);
  const auto cut = bvp::alignRoundaboutPrediction(input, .081, 1.237);
  ASSERT_TRUE(cut);
  EXPECT_NEAR(cut->times.back(), 1.237, 1e-10);
}

TEST(RoundaboutPredictionTime, UncoveredInputsAreUnknownNotAlignmentSuccess)
{
  auto input = prediction();
  EXPECT_FALSE(bvp::alignRoundaboutPrediction(input, -.001, 3.0));
  EXPECT_FALSE(bvp::alignRoundaboutPrediction(input, 3.0, 3.0));
  EXPECT_FALSE(bvp::alignRoundaboutPrediction(input, 3.1, 3.0));
  EXPECT_FALSE(bvp::alignRoundaboutPrediction(input, 0.0, 0.0));
  input.path[2].position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(bvp::alignRoundaboutPrediction(input, 0.0, 3.0));
  input = prediction(); input.path[2].orientation.w = 0.0;
  EXPECT_FALSE(bvp::alignRoundaboutPrediction(input, 0.0, 3.0));
  input = prediction(); input.path.clear();
  EXPECT_FALSE(bvp::alignRoundaboutPrediction(input, 0.0, 3.0));
}

TEST(RoundaboutPredictionTime, CensoredExitDoesNotInvalidateProvenEarlyClear)
{
  EXPECT_FALSE(bvp::unresolvedRoundaboutExit(true, 1.0, 2.0, .5));
  EXPECT_TRUE(bvp::unresolvedRoundaboutExit(true, 1.5, 2.0, .5));
  EXPECT_TRUE(bvp::unresolvedRoundaboutExit(true, 4.0, 2.0, .5));
  EXPECT_FALSE(bvp::unresolvedRoundaboutExit(false, 4.0, 2.0, .5));
}

TEST(RoundaboutPredictionTime, R55FiniteEndIsNotAnExitAndClockShiftIsNotMargin)
{
  const auto p = bvp::alignRoundaboutPrediction(prediction(), .111, 6.0);
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->times.back(), 2.889, 1e-10);
  // Observed path ends inside the corridor; no assumed stationary extension.
  EXPECT_TRUE(bvp::unresolvedRoundaboutExit(true, 4.0, 2.7-.111, 0.0));
  EXPECT_FALSE(bvp::unresolvedRoundaboutExit(true, 2.0, 2.7-.111, 0.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
