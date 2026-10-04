#include "autoware/behavior_path_lane_change_module/utils/reachable_admission.hpp"
#include <gtest/gtest.h>
namespace r=autoware::behavior_path_planner::lane_change::reachable_admission;
namespace {
const std::vector<double> arc{0,1,5,10,20,40,80,150,200};
const std::vector<double> caps(arc.size(),30.);
auto limit=[](double){return 2.;};
auto budget=[](){return true;};
}
TEST(ReachableAdmission, NativeUnsafeCannotBeRescued) {
 unsigned called=0;
 auto result=r::evaluate(false,arc,caps,10,0,3,-4,-2,2,limit,
   [&](const auto&,std::string&){++called;return true;},budget);
 EXPECT_FALSE(result.accepted);EXPECT_EQ(called,0u);EXPECT_EQ(result.reason,"native_unsafe");
}
TEST(ReachableAdmission, ProvisionalPhaseRequiresMandatoryActualWitnessMode) {
 EXPECT_TRUE(r::use_actual_start_provisional_phase(true,true,true,true,true,true));
 for(size_t disabled=0;disabled<6;++disabled) {
   std::array<bool,6> flags{{true,true,true,true,true,true}};flags[disabled]=false;
   EXPECT_FALSE(r::use_actual_start_provisional_phase(
     flags[0],flags[1],flags[2],flags[3],flags[4],flags[5]));
 }
 // Provisional/native safety cannot substitute for the final actual witness.
 EXPECT_FALSE(r::evaluate(true,arc,caps,25.79912252002062,1.9748379816591568,
   3.,-4.,-20.,100.,limit,[](const auto&,std::string&){return false;},budget,true).accepted);
}
TEST(ReachableAdmission, ExpandedFamilyPreservesOldTargetsAndAddsPositiveIntervals) {
 namespace grid=bounded_speed_grid;
 namespace positive=autoware::behavior_path_planner::lane_change::rss_positive_grid;
 const auto old=grid::acceleration_targets(3.4,-4.);
 EXPECT_EQ(positive::augment(old,3.4,false),old);
 const auto expanded=positive::augment(old,3.4,true);
 EXPECT_LE(expanded.size(),15u);
 for(double a:old)EXPECT_NE(std::find(expanded.begin(),expanded.end(),a),expanded.end());
 for(double f:{.75,.25,.1})EXPECT_NE(std::find(expanded.begin(),expanded.end(),3.4*f),expanded.end());
}
TEST(ReachableAdmission, ExpandedFamilyRequiresCheckedNegativeWitness) {
 unsigned calls=0;
 auto check=[&](const auto&p,std::string&){++calls;return p.samples.back().v<9.9;};
 const auto result=r::evaluate(true,arc,caps,10,0,3,-4,-2,2,limit,check,budget,true);
 EXPECT_TRUE(result.accepted);EXPECT_LT(result.target,0.);EXPECT_GT(calls,3u);
 EXPECT_FALSE(r::evaluate(true,arc,caps,10,0,3,-4,-2,2,limit,check,budget,false).accepted);
 EXPECT_FALSE(r::evaluate(false,arc,caps,10,0,3,-4,-2,2,limit,check,budget,true).accepted);
 EXPECT_FALSE(r::evaluate(true,arc,caps,10,0,3,-4,-2,2,limit,check,[](){return false;},true).accepted);
}
TEST(ReachableAdmission, ExpandedFamilyReservesOnlyRemainingAdaptiveBudget) {
 namespace positive=autoware::behavior_path_planner::lane_change::rss_positive_grid;
 auto attempted=positive::augment(bounded_speed_grid::acceleration_targets(3.4,-4.),3.4,true);
 ASSERT_EQ(attempted.size(),12u);
 const auto probes=bounded_speed_grid::probe_small_positive(true,true,.34,
   [&](double a){EXPECT_EQ(std::count(attempted.begin(),attempted.end(),a),0);
     attempted.push_back(a);return false;},15-attempted.size());
 EXPECT_EQ(probes.size(),3u);EXPECT_EQ(attempted.size(),15u);
 unsigned calls=0;
 bounded_speed_grid::probe_small_positive(true,true,.34,[&](double){++calls;return false;},0);
 EXPECT_EQ(calls,0u);
}
TEST(ReachableAdmission, OneActualWitnessPreservesImmutableInputs) {
 const auto original_arc=arc,original_caps=caps;
 unsigned checked=0;
 const auto result=r::evaluate(true,arc,caps,10,.5,3,-4,-2,2,limit,
   [&](const auto&p,std::string&){++checked;EXPECT_DOUBLE_EQ(p.speeds.front(),10.);
     const double a=(p.speeds[1]*p.speeds[1]-100.)/2.;
     EXPECT_GE(a,.5);EXPECT_LT(a,1.0);return true;},budget);
 EXPECT_TRUE(result.accepted);EXPECT_EQ(checked,1u);EXPECT_EQ(result.attempted,1u);
 EXPECT_EQ(arc,original_arc);EXPECT_EQ(caps,original_caps);
}
TEST(ReachableAdmission, NoSafeWitnessNeverApprovesNominal) {
 auto result=r::evaluate(true,arc,caps,10,0,3,-4,-2,2,limit,
   [](const auto&,std::string&why){why="all_raw_RSS_rejected";return false;},budget);
 EXPECT_FALSE(result.accepted);EXPECT_EQ(result.attempted,3u);
}
TEST(ReachableAdmission, ExistingPhysicalCapAndStopAreNotRaised) {
 auto lower=caps;lower[0]=9.;unsigned checked=0;
 EXPECT_FALSE(r::evaluate(true,arc,lower,10,0,3,-4,-2,2,limit,
   [&](const auto&,std::string&){++checked;return true;},budget).accepted);
 EXPECT_EQ(checked,0u);
 auto stop=caps;stop[6]=0.;
 auto result=r::evaluate(true,arc,stop,10,0,3,-4,-2,2,limit,
   [](const auto&p,std::string&){EXPECT_DOUBLE_EQ(p.speeds[6],0.);return true;},budget);
 EXPECT_TRUE(result.accepted);
}
TEST(ReachableAdmission, ExpiredBudgetDoesNotUseNativeFallback) {
 EXPECT_FALSE(r::evaluate(true,arc,caps,10,0,3,-4,-2,2,limit,
   [](const auto&,std::string&){return true;},[](){return false;}).accepted);
 unsigned calls=0;
 auto result=r::evaluate(true,arc,caps,10,0,3,-4,-2,2,limit,
   [](const auto&,std::string&){return true;},[&](){return ++calls<3;});
 EXPECT_FALSE(result.accepted);EXPECT_EQ(result.reason,"admission_budget_exhausted");
}
TEST(ReachableAdmission, AppendedTailStopIsVisibleToAdmissionBeforeSelection) {
 std::vector<double> path{10,10};unsigned checked=0;
 const bool result=r::finalize_then_admit(path,
   [](auto&p){p.push_back(0.);p.push_back(10.);},
   [&](const auto&p){++checked;EXPECT_EQ(p.size(),4u);EXPECT_DOUBLE_EQ(p[2],0.);return false;});
 EXPECT_FALSE(result);EXPECT_EQ(checked,1u);EXPECT_DOUBLE_EQ(path[2],0.);
}
TEST(ReachableAdmission, FinalizedNativeAndAdmissionSeeIdenticalTailAndZeros) {
 std::vector<double> p{12,0,12};std::string order;unsigned finalized=0;
 EXPECT_TRUE(r::finalize_and_check(true,p,[&](auto&v){++finalized;order+='F';v.push_back(0);},
   [&](const auto&v){order+='N';EXPECT_EQ(&v,&p);EXPECT_EQ(v,(std::vector<double>{12,0,12,0}));return true;},
   [&](const auto&v){order+='A';EXPECT_EQ(&v,&p);EXPECT_EQ(v,(std::vector<double>{12,0,12,0}));return true;}));
 EXPECT_EQ(order,"FNA");EXPECT_EQ(finalized,1u);
}
TEST(ReachableAdmission, DisabledRetainsNativeThenFinalizeThenAdmissionOrder) {
 std::vector<double> p{10};std::string order;
 EXPECT_TRUE(r::finalize_and_check(false,p,[&](auto&v){order+='F';v.push_back(0);},
   [&](const auto&v){order+='N';EXPECT_EQ(v.size(),1u);return true;},
   [&](const auto&v){order+='A';EXPECT_EQ(v.size(),2u);return true;}));
 EXPECT_EQ(order,"NFA");
}
TEST(ReachableAdmission, FinalizationFailureNeverRunsEitherSafetyCheck) {
 std::vector<double> p{10};unsigned checks=0;
 EXPECT_THROW(r::finalize_and_check(true,p,[](auto&){throw std::logic_error("invalid_tail");},
   [&](const auto&){++checks;return true;},[&](const auto&){++checks;return true;}),std::logic_error);
 EXPECT_EQ(checks,0u);
}
TEST(ReachableAdmission, FinalizedNativeFailureCannotReachAdmission) {
 std::vector<double> p{10};unsigned admission=0;
 EXPECT_FALSE(r::finalize_and_check(true,p,[](auto&v){v.push_back(0);},
   [](const auto&){return false;},[&](const auto&){++admission;return true;}));
 EXPECT_EQ(admission,0u);
}
TEST(ReachableAdmission, AdmissionFailureRejectsFinalizedNativeSafeCandidate) {
 std::vector<double> p{10};unsigned native=0,admission=0;
 EXPECT_FALSE(r::finalize_and_check(true,p,[](auto&v){v.push_back(0);},
   [&](const auto&){++native;return true;},[&](const auto&){++admission;return false;}));
 EXPECT_EQ(native,1u);EXPECT_EQ(admission,1u);
}
