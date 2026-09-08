#include <autoware/behavior_path_planner_common/marker_utils/colors.hpp>

#include <gtest/gtest.h>

namespace
{
TEST(MarkerColors, PreservesRgbAndDefaultAlpha)
{
  const auto red = marker_utils::colors::red();
  EXPECT_FLOAT_EQ(red.r, 1.0F);
  EXPECT_FLOAT_EQ(red.g, 0.0F);
  EXPECT_FLOAT_EQ(red.b, 0.0F);
  EXPECT_FLOAT_EQ(red.a, 0.99F);

  const auto orchid = marker_utils::colors::medium_orchid(0.25F);
  EXPECT_FLOAT_EQ(orchid.r, 0.729F);
  EXPECT_FLOAT_EQ(orchid.g, 0.333F);
  EXPECT_FLOAT_EQ(orchid.b, 0.827F);
  EXPECT_FLOAT_EQ(orchid.a, 0.25F);
}

TEST(MarkerColors, PreservesPaletteOrderAndAlpha)
{
  const auto colors = marker_utils::colors::colors_list(0.4F);
  ASSERT_EQ(colors.size(), 10U);
  EXPECT_FLOAT_EQ(colors.front().r, 1.0F);
  EXPECT_FLOAT_EQ(colors.front().g, 0.0F);
  EXPECT_FLOAT_EQ(colors.front().b, 0.0F);
  EXPECT_FLOAT_EQ(colors[2].r, 0.0F);
  EXPECT_FLOAT_EQ(colors[2].g, 0.0F);
  EXPECT_FLOAT_EQ(colors[2].b, 1.0F);
  EXPECT_FLOAT_EQ(colors.back().r, 0.690F);
  EXPECT_FLOAT_EQ(colors.back().g, 0.768F);
  EXPECT_FLOAT_EQ(colors.back().b, 0.870F);
  for (const auto & color : colors) EXPECT_FLOAT_EQ(color.a, 0.4F);
}
}  // namespace
