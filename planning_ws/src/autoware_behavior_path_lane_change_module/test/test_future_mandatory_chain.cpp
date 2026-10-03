#include "autoware/behavior_path_lane_change_module/utils/future_mandatory_chain.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <memory>
namespace fc=autoware::behavior_path_planner::utils::lane_change::future_chain;
namespace {
std::map<fc::Id,fc::Node> graph()
{
  return {{1,{10,1,true,true,{2,4}}},{2,{10,2,true,true,{3}}},
    {3,{10,3,true,true,{}}},{4,{10,2,true,true,{5}}},{5,{10,3,true,true,{}}}};
}
fc::Gate gate() {return {true,true,false,false,false,true,true,true,true,250,150,75,25};}
}
TEST(FutureMandatoryChain, UniquePreservesEveryBranchVertex)
{EXPECT_EQ(fc::prove(graph(),1,3).chain,std::vector<fc::Id>({1,2,3}));}
TEST(FutureMandatoryChain, AmbiguousDestinationRejected)
{auto g=graph();g[4].following={3};EXPECT_EQ(fc::prove(g,1,3).reason,"ambiguous_route_chain");}
TEST(FutureMandatoryChain, OutOfRouteRejected)
{auto g=graph();g[2].route=false;EXPECT_FALSE(fc::prove(g,1,3).valid);}
TEST(FutureMandatoryChain, OppositeOrCurrentBranchRejected)
{auto g=graph();g[2].forward=false;EXPECT_FALSE(fc::prove(g,1,3).valid);}
TEST(FutureMandatoryChain, RouteBacktrackingRejected)
{auto g=graph();g[2].route_order=0;EXPECT_FALSE(fc::prove(g,1,3).valid);}
TEST(FutureMandatoryChain, LoopDoesNotProveDestination)
{auto g=graph();g[2].following={1};EXPECT_FALSE(fc::prove(g,1,3).valid);}
TEST(FutureMandatoryChain, ExpansionBoundFailsClosed)
{EXPECT_EQ(fc::prove(graph(),1,3,fc::Limits{1,64,1000}).reason,"proof_budget_exhausted");}
TEST(FutureMandatoryChain, DepthBoundFailsClosed)
{EXPECT_EQ(fc::prove(graph(),1,3,fc::Limits{512,1,1000}).reason,"proof_budget_exhausted");}
TEST(FutureMandatoryChain, DistanceBoundFailsClosed)
{EXPECT_EQ(fc::prove(graph(),1,3,fc::Limits{512,64,15}).reason,"proof_budget_exhausted");}
TEST(FutureMandatoryChain, DynamicReserveBoundary)
{auto g=gate();EXPECT_TRUE(fc::applicable(g));g.target_distance=251;EXPECT_FALSE(fc::applicable(g));}
TEST(FutureMandatoryChain, DefaultOffAndFreshness)
{auto g=gate();g.enabled=false;EXPECT_FALSE(fc::applicable(g));g.enabled=true;g.fresh_route=false;EXPECT_FALSE(fc::applicable(g));}
TEST(FutureMandatoryChain, ApprovalAndOtherLifecyclesRejected)
{auto g=gate();g.approved=true;EXPECT_FALSE(fc::applicable(g));g.approved=false;g.aborting=true;EXPECT_FALSE(fc::applicable(g));g.aborting=false;g.stopping=true;EXPECT_FALSE(fc::applicable(g));}
TEST(FutureMandatoryChain, IllegalNeighborAndInvalidMetricRejected)
{auto g=gate();g.legal_adjacent=false;EXPECT_FALSE(fc::applicable(g));g.legal_adjacent=true;g.required_lc=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(fc::applicable(g));}
TEST(FutureMandatoryChain, RealRecordedRouteChainFixture)
{
  // Actual v85 vehicle graph + R30r2 RouteB edges; integration loader check is separate.
  const std::vector<fc::Id> ids{201470,200130,201474,201441,200360,201477,201479};
  std::map<fc::Id,fc::Node> g;
  for(size_t i=0;i<ids.size();++i)g.emplace(ids[i],fc::Node{10,int(i),true,true,
    i+1<ids.size()?std::vector<fc::Id>{ids[i+1]}:std::vector<fc::Id>{}});
  g[201441].following.push_back(200185);
  g[200185]={10,4,true,false,{}}; // current branch must not replace proved successor
  EXPECT_EQ(fc::prove(g,201470,201479).chain,ids);
  EXPECT_EQ(fc::prove(g,201474,201479).chain,std::vector<fc::Id>(ids.begin()+2,ids.end()));
}
TEST(FutureMandatoryChain, RawRouteEpochAndGenerationFailClosed)
{
  struct Message {int uuid;struct Header {int stamp;} header;};
  std::shared_ptr<const Message> input;
  EXPECT_FALSE(fc::same_route_epoch(input,1,2));
  input=std::make_shared<Message>(Message{1,{2}});
  EXPECT_TRUE(fc::same_route_epoch(input,1,2));
  EXPECT_FALSE(fc::same_route_epoch(input,2,2));
  EXPECT_FALSE(fc::same_route_epoch(input,1,3));
  EXPECT_TRUE(fc::same_route_generation(input,input));
  const std::shared_ptr<const Message> replacement=std::make_shared<Message>(Message{1,{2}});
  EXPECT_FALSE(fc::same_route_generation(input,replacement));
}
