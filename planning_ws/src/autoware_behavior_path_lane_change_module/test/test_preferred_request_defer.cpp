#include "autoware/behavior_path_lane_change_module/utils/preferred_request_defer.hpp"
#include <gtest/gtest.h>
#include <autoware/motion_utils/trajectory/acceleration_limits.hpp>
#include "autoware/behavior_path_lane_change_module/utils/passing_intent.hpp"
#include <map>
namespace p=autoware::behavior_path_planner::lane_change::preferred_request_defer;
TEST(PreferredRequestDefer,SparseDenseCenterlineKeepsPhysicalShift) {
 const std::vector<std::pair<double,double>> sparse{{0,0},{100,0}},dense{{0,0},{10,0},{20,0},{30,0},{100,0}};
 for(double x:{1.,19.,33.,80.,99.}) {
   EXPECT_DOUBLE_EQ(p::lateral_shift(sparse,x,3.7),3.7);
   EXPECT_DOUBLE_EQ(p::lateral_shift(dense,x,3.7),3.7);
 }
}
TEST(PreferredRequestDefer,NoExtrapolationOrDegenerateShiftProof) {
 EXPECT_TRUE(std::isnan(p::lateral_shift({{0,0},{100,0}},-1,3.7)));
 EXPECT_TRUE(std::isnan(p::lateral_shift({{0,0},{100,0}},101,3.7)));
 EXPECT_TRUE(std::isnan(p::lateral_shift({{0,0},{0,0}},0,3.7)));
 EXPECT_TRUE(std::isnan(p::lateral_shift({},0,3.7)));
 EXPECT_TRUE(std::isnan(p::lateral_shift({{0,0},{100,0}},std::numeric_limits<double>::quiet_NaN(),3.7)));
 EXPECT_DOUBLE_EQ(p::lateral_shift({{0,0},{100,0}},0,3.7),3.7);
}
p::Context good(){p::Context c;c.enabled=c.normal=c.fresh=c.topology=c.target_lead=true;
 c.approved=c.near_terminal=false;c.speed=20;c.target_speed=11;c.target_gap=25;
 c.horizon=4;c.remaining=400;c.future_reserve=150;c.max_acc=3;return c;}
TEST(PreferredRequestDefer,OnlyInitialKnownOptionalRequest){auto c=good();EXPECT_TRUE(p::evaluate(c));
 c.approved=true;EXPECT_FALSE(p::evaluate(c));c=good();c.normal=false;EXPECT_FALSE(p::evaluate(c));
 c=good();c.fresh=false;EXPECT_FALSE(p::evaluate(c));c=good();c.topology=false;EXPECT_FALSE(p::evaluate(c));}
TEST(PreferredRequestDefer,PreservesFutureSpeedReserve){auto c=good();c.remaining=254;EXPECT_FALSE(p::evaluate(c));
 c=good();c.future_reserve=300;EXPECT_FALSE(p::evaluate(c));c=good();c.near_terminal=true;EXPECT_FALSE(p::evaluate(c));}
TEST(PreferredRequestDefer,CurrentLeaderMustBeMoreFavorable){auto c=good();c.current_lead=true;c.current_gap=50;c.current_speed=15;
 EXPECT_TRUE(p::evaluate(c));c.current_gap=20;EXPECT_FALSE(p::evaluate(c));c.current_gap=50;c.current_speed=5;EXPECT_FALSE(p::evaluate(c));}
TEST(PreferredRequestDefer,NoSlowerTargetDoesNotSuppress){auto c=good();c.target_speed=21;EXPECT_FALSE(p::evaluate(c));
 c=good();c.target_gap=100;EXPECT_FALSE(p::evaluate(c));c=good();c.target_lead=false;EXPECT_FALSE(p::evaluate(c));}
TEST(PreferredRequestDefer,ExistingRssSpacingComparedOnBothLanes){auto c=good();
 c.max_acc=0; // Original coasting contract remains exact when acceleration is zero.
 c.speed=16.5;c.target_speed=11.1;c.target_gap=24;c.horizon=3;c.target_rss_gap=18;
 EXPECT_TRUE(p::evaluate(c));c.target_rss_gap=0;EXPECT_FALSE(p::evaluate(c));
 c.target_rss_gap=18;c.current_lead=true;c.current_gap=25;c.current_speed=12;c.current_rss_gap=15;
 EXPECT_FALSE(p::evaluate(c));}
TEST(PreferredRequestDefer,RecordedEntrySeparatesOptionalPreferenceFromStoppingReserve) {
 auto c=good();c.speed=16.945246874538284;c.target_gap=17.038;c.target_speed=12.334;
 c.horizon=2.903;c.remaining=473.543;c.max_acc=3.7;
 c.target_rss_gap=.5*c.speed+(c.speed*c.speed-c.target_speed*c.target_speed)/12.;
 c.future_reserve=880.901;EXPECT_FALSE(p::evaluate(c));
 // Log-rounded H and nearest recorded ego stamp: context regression, not an
 // exact callback or proof of closed-loop behavior.
 c.future_reserve=134.1;EXPECT_TRUE(p::evaluate(c));
 c.near_terminal=true;EXPECT_FALSE(p::evaluate(c));c.near_terminal=false;
 c.fresh=false;EXPECT_FALSE(p::evaluate(c));c.fresh=true;
 c.topology=false;EXPECT_FALSE(p::evaluate(c));c.topology=true;
 c.remaining=150;EXPECT_FALSE(p::evaluate(c));
 c.remaining=-1;EXPECT_FALSE(p::evaluate(c));
}
TEST(PreferredRequestDefer,UniqueRoutePrefixStopsAtSharedFork) {
 std::map<int64_t,std::vector<int64_t>> graph{{1,{2}},{2,{5,6}},{3,{4}},{4,{6,5}},{5,{}},{6,{}}};
 const auto allowed=[&](int64_t id){return graph.count(id)!=0;};
 const auto next=[&](int64_t id){return graph.at(id);};
 const auto a=p::unique_route_prefix(1,allowed,next),b=p::unique_route_prefix(3,allowed,next);
 EXPECT_TRUE(p::same_route_alternatives(a,b));
 EXPECT_EQ(a.lanes,(std::vector<int64_t>{1,2}));
 EXPECT_EQ(b.lanes,(std::vector<int64_t>{3,4}));
 graph[4]={5};EXPECT_FALSE(p::same_route_alternatives(a,p::unique_route_prefix(3,allowed,next)));
 graph[4]={5,7};EXPECT_FALSE(p::unique_route_prefix(3,allowed,next).valid);
 graph[4]={3};EXPECT_FALSE(p::unique_route_prefix(3,allowed,next).valid);
 graph[4]={5,5};EXPECT_FALSE(p::unique_route_prefix(3,allowed,next).valid);
 EXPECT_FALSE(p::unique_route_prefix(99,allowed,next).valid);
}
TEST(PreferredRequestDefer,ObservedCoverageNeverExtendsOrAcceptsMalformedTime) {
 EXPECT_DOUBLE_EQ(p::prediction_horizon(.02,{{0,1,3},{0,.5,2.5}}),2.48);
 EXPECT_DOUBLE_EQ(p::prediction_horizon(0,{{0,1,10}}),3.);
 for(const auto & paths:std::vector<std::vector<std::vector<double>>>{
   {},{{0}},{{0,1,1}},{{0,2,1}},{{.1,1,3}},{{0,std::numeric_limits<double>::quiet_NaN()}},{{0,.01}}})
   EXPECT_TRUE(std::isnan(p::prediction_horizon(.02,paths)));
 EXPECT_TRUE(std::isnan(p::prediction_horizon(-.001,{{0,1,3}})));
 EXPECT_TRUE(std::isnan(p::prediction_horizon(.501,{{0,1,3}})));
}
TEST(PreferredRequestDefer,SlowerFarCurrentLeadDoesNotImplyWorseProgress) {
 auto c=good();c.current_lead=true;c.current_speed=10;c.current_gap=100;c.current_rss_gap=20;
 EXPECT_TRUE(p::evaluate(c));
 c.current_gap=59;EXPECT_FALSE(p::evaluate(c));
 c.current_gap=100;c.near_terminal=true;EXPECT_FALSE(p::evaluate(c));
 c.near_terminal=false;c.normal=false;EXPECT_FALSE(p::evaluate(c));
 c.normal=true;c.topology=false;EXPECT_FALSE(p::evaluate(c));
 c.topology=true;c.approved=true;EXPECT_FALSE(p::evaluate(c));
}
TEST(PreferredRequestDefer,CalibratedUpperBarrierRetainsObservedAcceleration) {
 EXPECT_DOUBLE_EQ(p::acceleration_upper_bound(.1,3.7,3.329),3.329);
 EXPECT_DOUBLE_EQ(p::acceleration_upper_bound(4.,3.7,3.329),4.);
 EXPECT_DOUBLE_EQ(p::acceleration_upper_bound(-3.,3.7,3.329),3.329);
 EXPECT_TRUE(std::isnan(p::acceleration_upper_bound(0,3.7,3.8)));
 EXPECT_TRUE(std::isnan(p::acceleration_upper_bound(std::numeric_limits<double>::quiet_NaN(),3.7,3.3)));
}
TEST(PreferredRequestDefer,RecordedEarlyBracketStoppingReserveNotAtomicApproval) {
 // Exact bag odom/raw received before 43.538 s, NOT the unavailable atomic
 // request callback (input-path hazard/native buffers are not reconstructed).
 const autoware::motion_utils::AccelerationLimits map({{0,3.7},{2.777777777777778,3.7},
   {5.555555555555556,3.5},{16.666666666666668,3.4},{33.333333333333336,3.25}});
 auto c=good();c.speed=24.50186703625584;c.remaining=285.384338;
 c.horizon=p::prediction_horizon(.01695537567138672,{{0,1,2,3}});
 c.max_acc=p::acceleration_upper_bound(.01854012803897267,3.7,map.limit(c.speed,3.7));
 const double future=c.speed+c.max_acc*c.horizon;
 c.future_reserve=autoware::behavior_path_planner::passing_intent::stop_distance(future,c.max_acc,4.,10.)+future*.5+3.845;
 const double total=c.future_reserve+c.speed*c.horizon+.5*c.max_acc*c.horizon*c.horizon;
 EXPECT_NEAR(total,280.545792677,1e-6);
 EXPECT_GT(c.remaining,total);
 // Rounded leader inputs derived from this raw bracket; these assertions
 // exercise the policy, not exact lane-union/footprint adapter reconstruction.
 c.target_speed=17.933;c.target_gap=20.;c.current_lead=true;
 c.current_speed=12.614;c.current_gap=88.;
 const auto rss=[&](double lead){return .5*c.speed+(c.speed*c.speed-lead*lead)/12.;};
 c.target_rss_gap=rss(c.target_speed);c.current_rss_gap=rss(c.current_speed);
 // R60 uses accelerating travel on BOTH lanes. The old coasting comparison
 // passed 88 m, but that does not establish enough accelerating current gap.
 EXPECT_FALSE(p::evaluate(c));
 c.current_gap=110.;EXPECT_TRUE(p::evaluate(c));
 c.current_gap=80.;EXPECT_FALSE(p::evaluate(c));c.current_gap=110.;
 c.approved=true;EXPECT_FALSE(p::evaluate(c));c.approved=false;
 c.remaining=279.507410; // Later ego cannot reuse the early available reserve.
 EXPECT_LE(c.remaining,total);
 EXPECT_FALSE(p::evaluate(c));
}
TEST(PreferredRequestDefer,ExplicitImmediatePreferredForkExtendsBothLegalPrefixes) {
 const std::vector<p::OrderedRouteSegment> route{{2,{1,2}},{4,{3,4}},{5,{5,6}},{7,{7}}};
 const std::map<int64_t,std::vector<int64_t>> graph{{1,{3}},{2,{4}},{3,{5,6}},{4,{6,5}},{5,{7}},{6,{7}},{7,{}}};
 auto next=[&](int64_t id){return graph.at(id);};auto direction=[](auto,auto){return true;};
 const auto a=p::preferred_route_prefix(1,route,next,direction),b=p::preferred_route_prefix(2,route,next,direction);
 EXPECT_TRUE(p::same_preferred_continuation(a,b));
 EXPECT_EQ(a.prefix.lanes,(std::vector<int64_t>{1,3,5,7}));
 EXPECT_EQ(b.prefix.lanes,(std::vector<int64_t>{2,4,5,7}));
 EXPECT_EQ(a.resolved_forks,(std::vector<std::pair<size_t,int64_t>>{{2,5}}));
 EXPECT_TRUE(a.terminal);EXPECT_EQ(a.last_segment,3u);
}
TEST(PreferredRequestDefer,NoGlobalPreferredSetSkippingOrDuplicateRouteMembership) {
 std::vector<p::OrderedRouteSegment> route{{1,{1}},{2,{2,3}},{4,{4}}};
 std::map<int64_t,std::vector<int64_t>> graph{{1,{2,4}},{2,{4}},{3,{4}},{4,{}}};
 auto next=[&](int64_t id){return graph.at(id);};auto direction=[](auto,auto){return true;};
 EXPECT_FALSE(p::preferred_route_prefix(1,route,next,direction).prefix.valid); // skips segment1
 graph[1]={2,3};route[2].primitives.push_back(2);
 EXPECT_FALSE(p::preferred_route_prefix(1,route,next,direction).prefix.valid);
 route[2].primitives={4};route[1].preferred=4;
 EXPECT_FALSE(p::preferred_route_prefix(1,route,next,direction).prefix.valid); // not member
 route[1].preferred=2;graph[2]={1};
 EXPECT_FALSE(p::preferred_route_prefix(1,route,next,direction).prefix.valid); // cycle/order
}
TEST(PreferredRequestDefer,MissingPreferredFollowingStopsWithoutInventingEdge) {
 const std::vector<p::OrderedRouteSegment> route{{1,{1}},{2,{2,3,4}},{5,{5}}};
 auto next=[](int64_t id){return id==1?std::vector<int64_t>{3,4}:std::vector<int64_t>{};};
 const auto result=p::preferred_route_prefix(1,route,next,[](auto,auto){return true;});
 EXPECT_TRUE(result.prefix.valid);EXPECT_FALSE(result.terminal);
 EXPECT_EQ(result.prefix.lanes,(std::vector<int64_t>{1}));
 EXPECT_EQ(result.prefix.following,(std::vector<int64_t>{3,4}));
 EXPECT_TRUE(result.resolved_forks.empty());
}
TEST(PreferredRequestDefer,DirectionAndTerminalMismatchDoNotCertifyContinuation) {
 const std::vector<p::OrderedRouteSegment> route{{2,{1,2}},{3,{3,4}}};
 auto next=[](int64_t id){return id==1?std::vector<int64_t>{3}:id==2?std::vector<int64_t>{4}:std::vector<int64_t>{};};
 auto yes=[](auto,auto){return true;};
 const auto a=p::preferred_route_prefix(1,route,next,yes),b=p::preferred_route_prefix(2,route,next,yes);
 EXPECT_FALSE(p::same_preferred_continuation(a,b));
 EXPECT_FALSE(p::preferred_route_prefix(1,route,next,[](auto,auto){return false;}).prefix.valid);
 EXPECT_FALSE(p::preferred_route_prefix(99,route,next,yes).prefix.valid);
 EXPECT_FALSE(p::preferred_route_prefix(1,{},next,yes).prefix.valid);
}
TEST(PreferredRequestDefer,ExtensionJoinRejectionRetainsExactLegacyPrefix) {
 const std::vector<p::OrderedRouteSegment> route{{2,{1,2}},{4,{3,4}},{5,{5,6}},{7,{7}}};
 const std::map<int64_t,std::vector<int64_t>> graph{{1,{3}},{2,{4}},{3,{5,6}},{4,{5,6}},{5,{7}},{6,{7}},{7,{}}};
 auto next=[&](int64_t id){return graph.at(id);};auto allowed=[&](int64_t id){return graph.count(id);};
 const auto a=p::unique_route_prefix(1,allowed,next),b=p::unique_route_prefix(2,allowed,next);
 const auto bad=[](int64_t from,int64_t){return from!=3&&from!=4;};
 auto result=p::compose_route_prefixes(a,b,p::preferred_route_prefix(1,route,next,bad),p::preferred_route_prefix(2,route,next,bad));
 EXPECT_TRUE(result.valid);EXPECT_FALSE(result.extended);
 EXPECT_EQ(result.current.lanes,a.lanes);EXPECT_EQ(result.target.lanes,b.lanes);
 EXPECT_EQ(result.current.following,a.following); // End/distance domain is unchanged.
 auto yes=[](auto,auto){return true;};
 result=p::compose_route_prefixes(a,b,p::preferred_route_prefix(1,route,next,yes),p::preferred_route_prefix(2,route,next,yes));
 EXPECT_TRUE(result.valid);EXPECT_TRUE(result.extended);EXPECT_EQ(result.current.lanes.back(),7);
}
TEST(PreferredRequestDefer,MalformedOrderedRouteCannotGainExtensionDistance) {
 std::vector<p::OrderedRouteSegment> route{{2,{1,2}},{4,{3,4}},{5,{5,6}},{7,{7}}};
 const std::map<int64_t,std::vector<int64_t>> graph{{1,{3}},{2,{4}},{3,{5,6}},{4,{5,6}},{5,{7}},{6,{7}},{7,{}}};
 auto next=[&](int64_t id){return graph.at(id);};auto allowed=[&](int64_t id){return graph.count(id);};auto yes=[](auto,auto){return true;};
 const auto a=p::unique_route_prefix(1,allowed,next),b=p::unique_route_prefix(2,allowed,next);
 route[3].primitives.push_back(1);
 const auto result=p::compose_route_prefixes(a,b,p::preferred_route_prefix(1,route,next,yes),p::preferred_route_prefix(2,route,next,yes));
 EXPECT_TRUE(result.valid);EXPECT_FALSE(result.extended);EXPECT_EQ(result.current.lanes,a.lanes);
}
TEST(PreferredRequestDefer,ExtensionNeverCreatesUnknownOrMismatchedBasePermission) {
 p::RoutePrefix a{{1,3},{5,6},true},b{{2,4},{5,6},true};
 p::PreferredRoutePrefix x,y;x.prefix={{1,3,5,7},{},true};y.prefix={{2,4,5,7},{},true};
 x.terminal=y.terminal=true;x.last_segment=y.last_segment=3;
 EXPECT_FALSE(p::compose_route_prefixes({},b,x,y).valid);
 auto wrong=b;wrong.following={5,9};EXPECT_FALSE(p::compose_route_prefixes(a,wrong,x,y).valid);
 x.prefix.lanes={99,3,5,7};const auto result=p::compose_route_prefixes(a,b,x,y);
 EXPECT_TRUE(result.valid);EXPECT_FALSE(result.extended);EXPECT_EQ(result.current.lanes,a.lanes);
}

TEST(PreferredRequestDefer,RecordedR59RequestProjectsSameAccelerationOnBothLanes) {
 auto c=good();c.speed=22.089889456680776;c.target_speed=17.891655925033067;
 c.horizon=2.9747424125671387;c.target_gap=39.32121526246198;
 c.target_rss_gap=25.0326001007242;c.remaining=414.19092392209456;
 c.future_reserve=0; // Set below from the recorded total and tested acceleration.
 const autoware::motion_utils::AccelerationLimits map({{0,3.7},{2.777777777777778,3.7},
   {5.555555555555556,3.5},{16.666666666666668,3.4},{33.333333333333336,3.25}});
 c.max_acc=map.limit(c.speed,3.7); // Actual helper/map; observed a not captured by old diagnostic.
 c.future_reserve=250.79086777672086-c.speed*c.horizon-.5*c.max_acc*c.horizon*c.horizon;
 EXPECT_NEAR((c.speed-c.target_speed)*c.horizon+c.target_rss_gap,37.52126344517816,1e-9);
 EXPECT_TRUE(p::evaluate(c));
 c.current_lead=true;c.current_speed=c.target_speed;c.current_gap=40.;c.current_rss_gap=c.target_rss_gap;
 EXPECT_FALSE(p::evaluate(c)); // Same acceleration must not call this current lane clear.
 c.current_gap=100.;EXPECT_TRUE(p::evaluate(c));
 c.remaining=250.;EXPECT_FALSE(p::evaluate(c));
}

TEST(PreferredRequestDefer,FasterTargetOnlyDefersWhenProjectedClosurePositive) {
 auto c=good();c.speed=20;c.target_speed=22;c.horizon=3;c.max_acc=3.25;
 c.target_gap=25;c.target_rss_gap=20;
 EXPECT_TRUE(p::evaluate(c));
 c.target_speed=30;EXPECT_FALSE(p::evaluate(c));
 c.target_speed=22;c.max_acc=0;EXPECT_FALSE(p::evaluate(c));
 c.target_speed=18;c.target_gap=25;EXPECT_TRUE(p::evaluate(c)); // 6+20 coasting.
 c.current_lead=true;c.current_speed=18;c.current_rss_gap=20;c.current_gap=26;
 EXPECT_FALSE(p::evaluate(c)); // Exact equality remains a rejection.
 c.current_gap=26.001;EXPECT_TRUE(p::evaluate(c));
 c.current_gap=24;EXPECT_FALSE(p::evaluate(c));
}
TEST(PreferredRequestDefer,FreshRequestEpochKeepsActualOrderingAndFrames) {
 EXPECT_FALSE(p::request_epoch_valid(100.14,100.,100.0010797,100.,"map","map","map"));
 EXPECT_TRUE(p::request_epoch_valid(100.14,100.02,100.0010797,100.02,"map","map","map"));
 EXPECT_FALSE(p::request_epoch_valid(100.6,100.02,100.0010797,100.02,"map","map","map"));
 EXPECT_FALSE(p::request_epoch_valid(100.14,100.02,100.0010797,100.02,"map","odom","map"));
 EXPECT_FALSE(p::request_epoch_valid(100.14,100.02,100.0010797,100.02,"map","map","other"));
 EXPECT_FALSE(p::request_epoch_valid(100.14,100.02,100.0010797,99.,"map","map","map"));
 EXPECT_FALSE(p::request_epoch_valid(std::numeric_limits<double>::quiet_NaN(),100.,100.,100.,"map","map","map"));
}
