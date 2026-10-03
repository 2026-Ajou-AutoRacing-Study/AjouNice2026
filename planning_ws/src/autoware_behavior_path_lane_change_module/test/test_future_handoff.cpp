#include "autoware/behavior_path_lane_change_module/utils/future_handoff.hpp"
#include <gtest/gtest.h>
namespace h=autoware::behavior_path_planner::utils::lane_change::future_handoff;
const std::vector<h::Id> proof{201474,201441,200360,201477,201479};
TEST(FutureHandoff, ActualR32NominalEndRejected)
{EXPECT_EQ(h::check(proof,201477,200185,{200185},{200360},{200185,201453,201476,201448,201478}).reason,"end_still_overlaps_current");}
TEST(FutureHandoff, LaterActualTargetCenterTopologyAdmitted)
{EXPECT_TRUE(h::check(proof,201477,200360,{}, {200360},{200360,201477,201479}).allowed);}
TEST(FutureHandoff, WrongNativeBranchRejectedWithoutCurrentMembership)
{EXPECT_FALSE(h::check(proof,201477,200185,{}, {200360},{200185,201453}).allowed);}
TEST(FutureHandoff, ForkDivergenceRejected)
{EXPECT_EQ(h::check(proof,201477,201441,{}, {201441},{201441,200185,201453}).reason,"native_following_diverges");}
TEST(FutureHandoff, TruncatedFutureRejected)
{EXPECT_EQ(h::check(proof,201477,200360,{}, {200360},{200360}).reason,"native_sequence_incomplete");}
TEST(FutureHandoff, ConcordantSharedTargetBoundary)
{EXPECT_TRUE(h::check(proof,201477,201477,{}, {200360,201477},{201477,201479}).allowed);}
TEST(FutureHandoff, UnknownTargetOverlapRejected)
{EXPECT_FALSE(h::check(proof,201477,200360,{}, {200360,999},{200360,201477}).allowed);}
TEST(FutureHandoff, NoProofOrWrongFutureRejected)
{EXPECT_FALSE(h::check({},201477,200360,{}, {200360},{200360,201477}).allowed);EXPECT_FALSE(h::check(proof,999,200360,{}, {200360},{200360,201477}).allowed);}
TEST(FutureHandoff, CandidateAlreadyBeyondFutureDoesNotRequireBacktracking)
{EXPECT_TRUE(h::check(proof,201477,201479,{}, {201479},{201479}).allowed);}
