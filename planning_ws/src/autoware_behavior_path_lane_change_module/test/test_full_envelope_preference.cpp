#include "autoware/behavior_path_lane_change_module/utils/full_envelope_preference.hpp"
#include "autoware/behavior_path_lane_change_module/utils/rss_positive_grid.hpp"
#include <gtest/gtest.h>
namespace f=autoware::behavior_path_planner::lane_change::full_envelope;
TEST(FullEnvelopePreference, CommonDistanceAvoidsUnequalTailBias) {
 EXPECT_NEAR(f::travel_time({0,100,200},{10,10,10},150),15,1e-9);
 EXPECT_NEAR(f::travel_time({0,100,400},{10,10,10},150),15,1e-9);
 EXPECT_THROW(f::travel_time({0,100},{10,10},150),std::logic_error);
}
TEST(FullEnvelopePreference, StopCannotBeCrossedEvenIfPositiveResumes) {
 EXPECT_THROW(f::travel_time({0,50,100,200},{10,0,10,10},150),std::logic_error);
 EXPECT_NEAR(f::travel_time({0,50},{10,0},50),10,1e-9);
}
TEST(FullEnvelopePreference, UnsafeOrInfeasibleNeverRanks) {
 f::Selection pick;EXPECT_FALSE(pick.consider(0,false,{true,1}));
 EXPECT_FALSE(pick.consider(1,true,{}));std::vector<int> candidates{3,4};
 EXPECT_FALSE(pick.append_selected(candidates));EXPECT_EQ(candidates.size(),2);
 EXPECT_TRUE(pick.consider(0,true,{true,20}));EXPECT_FALSE(pick.consider(1,true,{}));
 EXPECT_TRUE(pick.append_selected(candidates));EXPECT_EQ(candidates.back(),3);
}
TEST(FullEnvelopePreference, FeasibleWholeEnvelopeBeatsFastPrefixOnly) {
 const std::vector<double> arc{0,20,40,80,150,200};
 const auto limit=[](double){return 1.;};const auto budget=[](){return true;};
 const auto bad=f::evaluate(arc,{20,20,2,20,20,20},15,0,3,-4,-2,2,150,limit,budget);
 const auto good=f::evaluate(arc,{20,20,20,20,20,20},15,0,3,-4,-2,2,150,limit,budget);
 EXPECT_FALSE(bad.feasible);ASSERT_TRUE(good.feasible);
 f::Selection pick;EXPECT_FALSE(pick.consider(0,true,bad));EXPECT_TRUE(pick.consider(1,true,good));
}
TEST(FullEnvelopePreference, InitialCapAndExistingZeroRetained) {
 const std::vector<double> arc{0,50,100,150,200};const auto limit=[](double){return 1.;};
 EXPECT_FALSE(f::evaluate(arc,{9,20,20,20,20},10,0,3,-4,-2,2,150,limit,[](){return true;}).feasible);
 EXPECT_FALSE(f::evaluate(arc,{20,20,0,20,20},10,0,3,-4,-2,2,150,limit,[](){return true;}).feasible);
}
TEST(FullEnvelopePreference, CalibratedLimitChangesReachableScore) {
 const std::vector<double> arc{0,50,100,150,200},caps(5,30);
 auto slow=f::evaluate(arc,caps,10,0,3,-4,-2,2,150,[](double){return .1;},[](){return true;});
 auto fast=f::evaluate(arc,caps,10,0,3,-4,-2,2,150,[](double){return 2.;},[](){return true;});
 ASSERT_TRUE(slow.feasible);ASSERT_TRUE(fast.feasible);EXPECT_LT(fast.travel_time,slow.travel_time);
}
TEST(FullEnvelopePreference, ExhaustedBudgetKeepsExistingSelection) {
 f::Selection pick;ASSERT_TRUE(pick.consider(0,true,{true,12}));
 auto skipped=f::evaluate({0,200},{20,20},10,0,3,-4,-2,2,150,[](double){return 1.;},[](){return false;});
 EXPECT_FALSE(pick.consider(1,true,skipped));std::vector<int> c{7,8};ASSERT_TRUE(pick.append_selected(c));EXPECT_EQ(c.back(),7);
}
TEST(FullEnvelopePreference, FiniteStrictGeometryRequired) {
 EXPECT_THROW(f::travel_time({0,0,200},{10,10,10},150),std::logic_error);
 EXPECT_THROW(f::travel_time({0,200},{10,-1},150),std::logic_error);
}

TEST(R46Envelope, LegacyWrapperMatchesExplicitThreeExactly) {
 const std::vector<double>s{0,50,100,150,200},c(5,30);
 auto l=[](double){return 3.;};auto b=[](){return true;};
 auto old=f::evaluate(s,c,10,0,3,-4,-10,100,150,l,b);
 auto explicit3=f::evaluate(s,c,10,0,3,-4,-10,100,150,l,b,{3,1.5,0});
 EXPECT_EQ(old.feasible,explicit3.feasible);EXPECT_EQ(old.travel_time,explicit3.travel_time);
}
TEST(R46Envelope, FamilyBoundsAndGlobalDeadline) {
 namespace lc=autoware::behavior_path_planner::lane_change;
 auto family=lc::rss_positive_grid::augment(bounded_speed_grid::acceleration_targets(3.4,-4),3.4,true);
 ASSERT_EQ(family.size(),12u);EXPECT_NE(std::find(family.begin(),family.end(),-.5),family.end());
 size_t attempts=0;auto budget=[&](){return ++attempts<=1;};
 auto score=f::evaluate({0,200},{30,30},10,0,3,-4,-10,100,150,[](double){return 3.4;},budget,family);
 EXPECT_TRUE(score.feasible);EXPECT_EQ(attempts,2u);
 EXPECT_EQ(f::scoring_budget_ms(100),35);EXPECT_EQ(f::scoring_budget_ms(50),17.5);
 EXPECT_EQ(f::scoring_budget_ms(1000),35);EXPECT_EQ(f::scoring_budget_ms(NAN),0);
}
TEST(R46Envelope, InvalidFamilyAndStopsCannotBecomeFeasible) {
 auto l=[](double){return 3.;};auto b=[](){return true;};
 for(const auto&targets:std::vector<std::vector<double>>{{NAN},{-5},{4},{}}) {
 EXPECT_FALSE(f::evaluate({0,200},{30,30},10,0,3,-4,-10,100,150,l,b,targets).feasible);
 }
 EXPECT_FALSE(f::evaluate({0,50,100,200},{30,0,30,30},10,0,3,-4,-10,100,150,l,b,{0,-.5,-1,-4}).feasible);
}
TEST(R46Envelope, RemoteCurveNegativeFamilyDoesNotPreferMaximumBraking) {
 // Exact R44r1 recorded shape/caps scalar fixture; full raw safety replay is separate.
 const std::vector<double>s{0,1.2136312206296136,3.2142751480035212,5.2149614889981724,7.2156841812424251,9.2164369578146701,11.217213384109522,13.218006894696186,15.218810830170538,17.219618474002068,19.22042308937877,21.221217956050303,23.221996407172014,25.222751866152308,27.223477883503818,29.224168173700733,31.224816652045135,33.225417471541512,35.225965059782474,40.323615332060434,45.4352798499838,50.553390053577758,55.678708851091152,60.81259703998095,65.955028624801756,71.103933756638511,76.256254047357928,81.402473370657333,82.633204122173765,86.567839563594617,89.798342329747257,91.665613322265415,96.793429174109178,101.91650884050473,107.03588119369341,112.15359783817121,117.27080345753879,122.08452749200086,141.21375318791252,223.7865749213868,328.42430155841106,373.30271338637596,385.84188132779104,414.68440206771993,448.65256181228182,471.88771630613672,474.77895739594834,476.7102513815239,483.46005609248505,483.66104992895669,484.16104919712291,484.66104955370349,485.16104993156625,485.6610491775524,486.16104955403318,486.66104990884247,487.1610491747208,487.66104953041707,488.16104990459917,488.66104914396118,489.1610494926029,489.66104984170465,490.16104908060657,490.66104942878309,491.16104903507994,491.66104863900637,492.1610482439533,492.66104673932915,493.16104634525743,493.6610467653046,494.16104610256787,494.66104541368435,495.16104586046907,495.66104516932666,496.16104491424755,496.66104465831398,497.16104440140947,497.66176099999512};
 const std::vector<double>c{33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.196472260504088,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,20.391656376719769,30.348220285115303,16.91877584614484,33.333333333333336,33.333333333333336,33.046169805705382,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 const auto l=[](double){return 3.363606367;};const auto b=[](){return true;};
 const auto run=[&](const std::vector<double>&targets){return f::evaluate(s,c,20.7104037,3.076056727,2.916551113,-4,-10,100,150,l,b,targets);};
 EXPECT_FALSE(run({3.363606367,1.6818031835,0}).feasible);
 const auto family=autoware::behavior_path_planner::lane_change::rss_positive_grid::augment(bounded_speed_grid::acceleration_targets(l(0),-4),l(0),true);
 auto all=run(family),gentle=run({-.5}),strong=run({-4});
 ASSERT_TRUE(all.feasible);ASSERT_TRUE(gentle.feasible);ASSERT_TRUE(strong.feasible);
 EXPECT_EQ(all.travel_time,gentle.travel_time);EXPECT_LT(all.travel_time,strong.travel_time);
}
