#include "autoware/behavior_path_lane_change_module/utils/pre_adjacency_policy.hpp"
#include <gtest/gtest.h>
#include <limits>
namespace lc=autoware::behavior_path_planner::utils::lane_change;
namespace {
lc::PreAdjacencyContext ready() {
  lc::PreAdjacencyContext c;
  c.enabled=c.normal=c.topology_verified=c.fresh_inputs=c.chain_valid=c.distances_valid=true;
  c.approved=c.near_terminal=false;c.target_start=48.4;c.remaining=460;c.required_lc=90;
  c.speed=16.34;c.acceleration=.1;c.deceleration=3;c.jerk=10;c.reaction=.5;c.front_margin=3.1;
  return c;
}
}
TEST(PreAdjacencyPolicy, DefaultsKeepLegacyAndVerifiedEntryCanDefer) {
  EXPECT_FALSE(lc::defer_pre_adjacency({}).defer);
  auto c=ready();EXPECT_TRUE(lc::defer_pre_adjacency(c).defer);
  c.enabled=false;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
}
TEST(PreAdjacencyPolicy, LegalAdjacencyImmediatelyRestoresRequest) {
  auto c=ready();c.target_start=0;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c.target_start=-1;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
}
TEST(PreAdjacencyPolicy, ExternalAndApprovedManeuversAreUntouched) {
  auto c=ready();c.normal=false;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.approved=true;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
}
TEST(PreAdjacencyPolicy, MandatoryReserveHasPriority) {
  auto c=ready();const auto d=lc::defer_pre_adjacency(c);
  EXPECT_GT(d.reserve,c.target_start+c.required_lc);
  c.remaining=d.reserve;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.near_terminal=true;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
}
TEST(PreAdjacencyPolicy, UnknownStaleOrBrokenChainKeepsLegacy) {
  auto c=ready();c.fresh_inputs=false;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.chain_valid=false;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.topology_verified=false;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.distances_valid=false;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
}
TEST(PreAdjacencyPolicy, InvalidMotionAndReserveNeverSuppress) {
  auto c=ready();c.acceleration=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.deceleration=0;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.jerk=-1;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
  c=ready();c.speed=1e308;EXPECT_FALSE(lc::defer_pre_adjacency(c).defer);
}
