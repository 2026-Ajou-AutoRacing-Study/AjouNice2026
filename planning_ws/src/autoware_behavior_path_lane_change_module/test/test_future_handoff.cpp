#include "autoware/behavior_path_lane_change_module/utils/future_handoff.hpp"
#include <gtest/gtest.h>
#include <map>
namespace h=autoware::behavior_path_planner::utils::lane_change::future_handoff;
const std::vector<h::Id> proof{201474,201441,200360,201477,201479};
TEST(FutureHandoff, ActualR32NominalEndRejected)
{EXPECT_EQ(h::check(proof,201477,200185,{200185},{200360},{200185,201453,201476,201448,201478}).reason,"end_still_overlaps_current");}
TEST(FutureHandoff, LaterActualTargetCenterTopologyAdmitted)
{EXPECT_TRUE(h::check(proof,201477,200360,{}, {200360},{200360,201477,201479}).allowed);}
TEST(FutureHandoff, SelectedTargetOwnershipSurvivesDifferentCurrentPolygonOverlap)
{EXPECT_TRUE(h::check(proof,201477,200360,{200185}, {200360},{200360,201477,201479}).allowed);}
TEST(FutureHandoff, SharedSuffixMembershipIsNotWrongBranch)
{EXPECT_TRUE(h::check(proof,201477,201477,{201477}, {201477},{201477,201479}).allowed);}
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

namespace {
lanelet::Lanelet ownerLane(lanelet::Id id,double center_x,double tilt=0.0,bool reverse=false)
{
  lanelet::LineString3d left(id*10+1,{lanelet::Point3d(id*10+2,-2,-10,0),lanelet::Point3d(id*10+3,-2,10,0)});
  lanelet::LineString3d right(id*10+4,{lanelet::Point3d(id*10+5,2,-10,0),lanelet::Point3d(id*10+6,2,10,0)});
  lanelet::Lanelet lane(id,left,right);
  lanelet::Point3d a(id*10+7,center_x-tilt,-10,0),b(id*10+8,center_x+tilt,10,0);
  lane.setCenterline(lanelet::LineString3d(id*10+9,reverse?lanelet::Points3d{b,a}:lanelet::Points3d{a,b}));
  return lane;
}
geometry_msgs::Pose ownerPose(double yaw)
{geometry_msgs::Pose p;p.orientation.z=std::sin(yaw/2);p.orientation.w=std::cos(yaw/2);return p;}
}
TEST(FutureHandoff, SharedSelectorNearCenterWinsTinyHeadingNoiseBothOrders)
{
  const auto near=ownerLane(11,.05,.1),far=ownerLane(12,1.0);
  for(double noise:{-.001,0.,.001})for(bool reverse:{false,true}) {
    lanelet::ConstLanelets lanes=reverse?lanelet::ConstLanelets{far,near}:lanelet::ConstLanelets{near,far};
    const auto selected=autoware::behavior_path_planner::utils::getPathLaneletOwner(lanes,ownerPose(M_PI/2+noise));
    ASSERT_TRUE(selected);EXPECT_EQ(selected->id(),11);
  }
}
TEST(FutureHandoff, SharedSelectorRejectsOppositeDirectionTiePreference)
{
  const auto reverse=ownerLane(11,0,0,true),forward=ownerLane(12,.2);
  const auto selected=autoware::behavior_path_planner::utils::getPathLaneletOwner({reverse,forward},ownerPose(M_PI/2));
  ASSERT_TRUE(selected);EXPECT_EQ(selected->id(),12);
}

namespace {
struct HandoffPathPoint {std::vector<lanelet::Id> lane_ids;};
struct HandoffPath {std::vector<HandoffPathPoint> points;};
struct HandoffHandler {
  std::map<lanelet::Id,lanelet::ConstLanelet> lanes;
  std::map<lanelet::Id,lanelet::Id> next;
  lanelet::ConstLanelet getLaneletsFromId(lanelet::Id id) const {return lanes.at(id);}
  bool isRouteLanelet(const lanelet::ConstLanelet&lane) const {return lanes.count(lane.id());}
  bool getNextLaneletWithinRoute(const lanelet::ConstLanelet&lane,lanelet::ConstLanelet*out) const {
    const auto it=next.find(lane.id());if(it==next.end())return false;
    *out=lanes.at(it->second);return true;
  }
};
}
TEST(FutureHandoff, EvaluateTargetOwnerWithCurrentPolygonOverlap)
{
  const auto current=ownerLane(10,1),target=ownerLane(20,.01),future=ownerLane(30,.5);
  const HandoffHandler handler{{{10,current},{20,target},{30,future}},{{20,30}}};
  const HandoffPath path{{{{10,20}}}};
  const auto result=h::evaluate(path,ownerPose(M_PI/2),{current},{target,future},30,handler);
  EXPECT_TRUE(result.allowed)<<result.reason;
  EXPECT_EQ(result.reason,"native_handoff_preserves_proved_suffix");
}
TEST(FutureHandoff, EvaluateCurrentOnlySelectedOwnerStillRejected)
{
  const auto current=ownerLane(10,.01),target=ownerLane(20,1),future=ownerLane(30,.5);
  const HandoffHandler handler{{{10,current},{20,target},{30,future}},{{20,30}}};
  const HandoffPath path{{{{10,20}}}};
  const auto result=h::evaluate(path,ownerPose(M_PI/2),{current},{target,future},30,handler);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.reason,"owner_outside_proved_suffix");
}
TEST(FutureHandoff, EvaluatePathEligibleOppositeTargetDirectionRejected)
{
  const auto current=ownerLane(10,1),target=ownerLane(20,.01,0,true),future=ownerLane(30,.5);
  const HandoffHandler handler{{{10,current},{20,target},{30,future}},{{20,30}}};
  // Ineligible current lane cannot silently replace the path's sole owner.
  const HandoffPath path{{{{20}}}};
  const auto result=h::evaluate(path,ownerPose(M_PI/2),{current},{target,future},30,handler);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.reason,"native_owner_direction_incompatible");
}
TEST(FutureHandoff, EvaluateSelectedTargetNativeFollowingDiverges)
{
  const auto current=ownerLane(10,1),target=ownerLane(20,.01),future=ownerLane(30,.5),wrong=ownerLane(40,1.5);
  const HandoffHandler handler{{{10,current},{20,target},{30,future},{40,wrong}},{{20,40},{40,30}}};
  const HandoffPath path{{{{10,20}}}};
  const auto result=h::evaluate(path,ownerPose(M_PI/2),{current},{target,future},30,handler);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.reason,"native_following_diverges");
}
