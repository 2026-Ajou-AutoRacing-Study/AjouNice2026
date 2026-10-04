#include "autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp"
#include <gtest/gtest.h>
#define WITH_ROS_DISTANCE
#include <cassert>
#include <iostream>
#include <limits>
#ifdef WITH_ROS_DISTANCE
#include <autoware/motion_utils/distance/distance.hpp>
#endif
using namespace bounded_speed_grid;

TEST(BoundedSpeedGrid, R66EntryReleaseUsesExplicitSharedJerkWithoutChangingRoadCaps)
{
  // Recorded scalar state, R66 official001 +34.540602 s. This is a
  // generation regression, NOT a closed-loop collision or vehicle calibration claim.
  const std::vector<double> arc{0,0.5666316029586278,1.2998355130582926,3.299828415265983,5.299815721318985,7.299799907032544,9.299783121536295,11.299767204099453,13.299753700960057,15.299743882155074,17.29973875835407,19.299739097692747,21.2997454426087,23.29975812667648,25.29977729144389,27.85924312644239,30.421490861715657,32.9842784462936,35.54342850435491,38.089156845055705,40.62927393708149,43.17700002758628,45.742117287281204,48.32638740329952,50.92198335025854,53.51316438693101,56.09303106305603,58.67264865082287,61.25300953427883,63.833647968507385,66.41408126403508,68.99383440120735,71.57246464714166,74.14958623771159,76.72489517794864,79.29819419724005,81.86941787043479,84.43866647142549,87.00635582974155,89.57301107947512,92.1390300924224,94.7046952186909,97.27019908221294,99.83566555241785,102.4021642890002,108.37730713812005,115.56645325473126,166.95798898803974,249.53081072151406,354.1685373585384,399.0469491865033,411.5861171279184,440.42863786784727,457.65626140691523,474.3967976124091,483.4303061007736};
  const std::vector<double> caps{16.38888931274414,16.38888931274414,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,29.345584183025963,27.24716195742836,30.057718758467768,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,21.323308613577574,24.699190821695705,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,33.333333333333336,0,0,0};
  const double v=16.375975890159655, a=0.6464172774369804, H=3.00219988822937;
  const auto limit=[](double){ return 3.7; }; // release stays below 0.65 m/s^2
  EXPECT_THROW(make(arc,caps,v,a,0,H,-4,-10,100,limit),std::logic_error);
  const auto coast=make(arc,caps,v,a,0,H,-4,-20,100,limit);
  ASSERT_EQ(coast.speeds.size(),caps.size());
  EXPECT_DOUBLE_EQ(coast.speeds.front(),v);
  EXPECT_GT(coast.progress,49.0);
  EXPECT_EQ(coast.speeds.back(),caps.back());
  for(size_t i=0;i<caps.size();++i) EXPECT_LE(coast.speeds[i],caps[i]+1e-5);
  for(size_t i=1;i<coast.samples.size();++i) {
    const auto & before=coast.samples[i-1]; const auto & after=coast.samples[i];
    const double jerk=(after.a-before.a)/(after.t-before.t);
    EXPECT_GE(jerk,-20-1e-6); EXPECT_LE(jerk,100+1e-6);
    EXPECT_GE(after.a,-4-1e-6); EXPECT_LE(after.a,3.7+1e-6);
  }
}


TEST(BoundedSpeedGrid, AccelerationFamilyIncludesModerateBraking)
{
  const std::vector<double> expected{3.4, 1.7, 0, -.5, -1, -1.5, -2, -3, -4};
  EXPECT_EQ(acceleration_targets(3.4, -4), expected);
}
TEST(BoundedSpeedGrid, AccelerationFamilyClampsAndDeduplicates)
{
  EXPECT_EQ(acceleration_targets(0, -.75), (std::vector<double>{0, -.5, -.75}));
  EXPECT_EQ(acceleration_targets(2, -2), (std::vector<double>{2, 1, 0, -.5, -1, -1.5, -2}));
  for (const double minimum : {-.01, -.5, -1., -1.5, -2., -3., -4., -8.}) {
    const auto targets = acceleration_targets(3.7, minimum);
    EXPECT_LE(targets.size(), 9U);
    EXPECT_EQ(std::count(targets.begin(), targets.end(), minimum), 1);
    for (const double target : targets) {
      EXPECT_GE(target, minimum);
      EXPECT_LE(target, 3.7);
    }
  }
}
TEST(BoundedSpeedGrid, AccelerationFamilyRejectsInvalidBounds)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double active : {-1., nan, inf})
    EXPECT_THROW(acceleration_targets(active, -4), std::logic_error);
  for (const double minimum : {0., 1., nan, -inf})
    EXPECT_THROW(acceleration_targets(3.4, minimum), std::logic_error);
}

TEST(BoundedSpeedGrid, ExactR22Run2BeforeFullStop)
{
  // Exact scalar fixture: round022/official002/planning.bag,
  // checked_braking_selected[0], stamp 1790956874.277879477 (36.086017 s).
  // Full-object/core replay is separately required; this tests proposal/caps.
  const std::vector<double> arc{0,2.1393639877031694,4.2813352988102427,6.4268534218225248,8.5821806852230829,10.753655047036485,12.942669492999491,15.143711337140392,17.34830870946244,19.547252303021335,21.732682687783218,23.90065088264787,26.05466312916052,28.201388308926436,30.346095830202962,31.84018317932221,104.32478192683341,111.51392804344462,162.90546377675309,245.47828551022741,350.11601214725169,394.99442397521659,406.06351144064303,407.53359191663168,431.83755613450143};
  const std::vector<double> caps{10.720796585083008,11.385623931884766,12.013699531555176,12.610907554626465,13.183103561401367,13.735000610351562,14.269279479980469,14.786559104919434,15.285624504089355,14.698938369750977,14.091675758361816,13.462227821350098,13.998623847961426,14.513030052185059,15.008899688720703,15.344650268554688,26.743341445922852,27.616580963134766,33.119754791259766,33.333332061767578,21.156085968017578,9.4102439880371094,0,0,0};
  const auto proposed=make(arc,caps,10.720797035579977,-2.2523685058559288,-.5,2.8944528100000002,-4,-10,100,
    [](double speed){return speed<=16.6666666667?3.5-(speed-5.55555555556)*.009:3.4-(speed-16.6666666667)*.009;});
  ASSERT_NEAR(proposed.progress,28.892025165,1e-6);
  ASSERT_EQ(proposed.speeds.size(),caps.size());
  for(size_t i=0;i<caps.size();++i)ASSERT_LE(proposed.speeds[i],caps[i]+1e-6);
  ASSERT_EQ(proposed.speeds.back(),0.0);
}
TEST(BoundedSpeedGrid, ScalarInvariants) {
  std::vector<double> s,cap;for(int i=0;i<=200;++i){s.push_back(i*.5);cap.push_back(40);}
  auto limit=[](double){return 2.;};
  auto coast=make(s,cap,10,.5,0,3,-4,-10,100,limit);
  ASSERT_TRUE(coast.speeds.front()==10 && coast.progress>29 && coast.progress<31);
  auto partial=make(s,cap,10,.5,-1,3,-4,-10,100,limit);
  auto accel=make(s,cap,10,.5,1,3,-4,-10,100,limit);
  ASSERT_TRUE(partial.progress<coast.progress && coast.progress<accel.progress);
  ASSERT_TRUE(accel.speeds.back()>10 && accel.speeds.size()==s.size());
  ASSERT_TRUE(std::abs(accel.samples.back().a)<1e-6);
  ASSERT_TRUE(coast.speeds.back()>9.9 && partial.speeds.back()>6);
  for(size_t i=1;i<accel.samples.size();++i){const auto&a=accel.samples[i-1];const auto&b=accel.samples[i];
    const double j=(b.a-a.a)/(b.t-a.t);ASSERT_TRUE(j<=100+1e-6 && j>=-10-1e-6);ASSERT_TRUE(b.s>=a.s && b.v>=0);}
  auto strong=make(s,cap,10,-7,-1,3,-4,-10,2,limit);ASSERT_TRUE(strong.samples.front().a==-7);
  ASSERT_TRUE(strong.samples[1].a>-7 && strong.samples[1].a< -6.9);
  auto stopped=make(s,cap,0,-7,0,3,-4,-10,100,limit);for(double v:stopped.speeds)ASSERT_TRUE(v==0);
  auto tiny=make(s,cap,.0001,-7,0,3,-4,-10,100,limit);ASSERT_TRUE(tiny.samples.back().s>=0 && tiny.samples.back().s<1e-6);
  auto reject=[&](auto fn){bool threw=false;try{fn();}catch(const std::logic_error&){threw=true;}ASSERT_TRUE(threw);};
  auto zero=cap;zero[10]=0;reject([&]{make(s,zero,10,0,0,3,-4,-10,100,limit);});
  auto low=cap;low[0]=9;reject([&]{make(s,low,10,0,0,3,-4,-10,100,limit);});
  reject([&]{make(s,cap,10,0,0,3,-4,-10,0,limit);});
  reject([&]{make(s,cap,10,0,0,3,-4,-10,100,[](double){return std::numeric_limits<double>::infinity();});});
  auto capped=make(s,cap,10,0,10,3,-4,-10,100,limit);
  for(const auto&p:capped.samples)ASSERT_TRUE(p.a<=2+1e-9);
  // Existing distant stop gets a reachable tail, never an early artificial stop.
  auto far_stop=cap;far_stop.back()=0;
  auto stopped_at_real_line=make(s,far_stop,10,0,0,3,-4,-10,100,limit);
  auto prefix_reference=make(s,cap,10,0,0,3,-4,-10,100,limit);
  ASSERT_TRUE(stopped_at_real_line.speeds.back()==0 && stopped_at_real_line.speeds[100]==10);
  for(size_t i=0;i<s.size()&&s[i]<=31;++i)
    ASSERT_TRUE(std::abs(stopped_at_real_line.speeds[i]-prefix_reference.speeds[i])<1e-10);
  ASSERT_TRUE(std::abs(stopped_at_real_line.samples.back().s-s.back())<1e-8);
  auto step_caps=cap;for(size_t i=120;i<step_caps.size();++i)step_caps[i]=6;step_caps.back()=0;
  auto stepped=make(s,step_caps,10,0,0,3,-4,-10,100,limit);
  ASSERT_TRUE(std::abs(stepped.speeds[120]-6)<1e-7 && stepped.speeds.back()==0);
  for(size_t i=1;i<stepped.samples.size();++i){const auto&a=stepped.samples[i-1];const auto&b=stepped.samples[i];
    const double jerk=(b.a-a.a)/(b.t-a.t);ASSERT_TRUE(jerk>=-10-1e-5 && jerk<=100+1e-5);}
  std::vector<double> long_s{0,1,1000},long_caps{40,40,40};
  auto low_coast=make(long_s,long_caps,.01,0,0,3,-4,-10,100,limit);
  ASSERT_TRUE(low_coast.samples.size()<310 && low_coast.speeds.back()==.01);
#ifdef WITH_ROS_DISTANCE
  for(double v:{2.,5.,10.,20.,30.})for(double ratio:{0.,.5,.95}) {
    const auto reference=autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(v,v*ratio,0,-4,100,-10);
    ASSERT_TRUE(reference);ASSERT_TRUE(std::abs(*reference-brake_plan(v,v*ratio,-4,-10,100).distance)<1e-7);
  }
#endif
  std::cout<<"bounded speed grid scalar invariants PASS (offline only)\n";
}

TEST(BoundedSpeedGrid, RemoteIntermediateCapRetryPreservesPrefixAndPhysicalBounds) {
  std::vector<double> arc,caps;
  for(int i=0;i<=410;i+=10){arc.push_back(i);caps.push_back(40);}
  caps[38]=15;caps.back()=0;
  const auto limit=[](double){return 3.4;};
  EXPECT_THROW(make(arc,caps,20,0,0,3,-4,-10,100,limit,false),RemoteTailConnectionFailure);
  const auto recovered=make(arc,caps,20,0,0,3,-4,-10,100,limit);
  EXPECT_GT(recovered.tail_attempts,1U);EXPECT_LE(recovered.tail_attempts,5U);
  EXPECT_LT(recovered.tail_factor,1.0);EXPECT_GE(recovered.tail_factor,.5);
  const auto free_prefix=make(arc,std::vector<double>(arc.size(),40),20,0,0,3,-4,-10,100,limit,false);
  EXPECT_DOUBLE_EQ(recovered.progress,free_prefix.progress);
  for(size_t i=0;i<arc.size();++i) {
    EXPECT_LE(recovered.speeds[i],caps[i]+1e-6);
    if(arc[i]<=70)EXPECT_DOUBLE_EQ(recovered.speeds[i],free_prefix.speeds[i]);
  }
  for(size_t i=0;i<free_prefix.samples.size() && free_prefix.samples[i].t<=3;++i) {
    EXPECT_DOUBLE_EQ(recovered.samples[i].t,free_prefix.samples[i].t);
    EXPECT_DOUBLE_EQ(recovered.samples[i].s,free_prefix.samples[i].s);
    EXPECT_DOUBLE_EQ(recovered.samples[i].v,free_prefix.samples[i].v);
    EXPECT_DOUBLE_EQ(recovered.samples[i].a,free_prefix.samples[i].a);
  }
  for(size_t i=1;i<recovered.samples.size();++i) {
    const auto & before=recovered.samples[i-1];const auto & after=recovered.samples[i];
    const double jerk=(after.a-before.a)/(after.t-before.t);
    EXPECT_GE(jerk,-10-1e-5);EXPECT_LE(jerk,100+1e-5);
    EXPECT_GE(after.a,-4-1e-6);EXPECT_GE(after.v,0);EXPECT_GE(after.s,before.s);
  }
  EXPECT_EQ(recovered.speeds.back(),0);
  EXPECT_NEAR(recovered.samples.back().s,arc.back(),1e-7);
}

TEST(BoundedSpeedGrid, InvalidPrefixIsNotClassifiedAsRetryableRemoteFailure) {
  const auto limit=[](double){return 3.;};
  for(bool retry:{false,true}) {
    try {make({0,1,2},{30,0,0},20,0,0,3,-4,-10,100,limit,retry);FAIL();}
    catch(const RemoteTailConnectionFailure &){FAIL()<<"near stop must not trigger remote retry";}
    catch(const std::logic_error &){SUCCEED();}
    try {make({0,10,20},{10,40,40},20,0,0,3,-4,-10,100,limit,retry);FAIL();}
    catch(const RemoteTailConnectionFailure &){FAIL()<<"initial state must not trigger remote retry";}
    catch(const std::logic_error &){SUCCEED();}
  }
}

TEST(BoundedSpeedGrid, AdaptivePositiveRetainsEverySuccessfulProbeWithinBudget) {
  std::vector<double> accepted;
  const auto attempted=probe_small_positive(true,true,1.7,[&](double target) {
    if(target>.18)return false;
    accepted.push_back(target);return true;
  });
  EXPECT_EQ(attempted.size(),6U);
  ASSERT_EQ(accepted.size(),2U);
  EXPECT_DOUBLE_EQ(accepted[0],attempted[3]);
  EXPECT_DOUBLE_EQ(accepted[1],attempted[4]);
  EXPECT_NEAR(accepted[0],.10625,1e-12);
  EXPECT_NEAR(accepted[1],.159375,1e-12);
  for(double target:attempted){EXPECT_GT(target,0);EXPECT_LT(target,1.7);}
}
TEST(BoundedSpeedGrid, AdaptivePositiveRequiresValidCoastAndFailedPositive) {
  int calls=0;auto check=[&](double){++calls;return true;};
  EXPECT_TRUE(probe_small_positive(false,true,1.,check).empty());
  EXPECT_TRUE(probe_small_positive(true,false,1.,check).empty());
  EXPECT_TRUE(probe_small_positive(true,true,0.,check).empty());
  EXPECT_TRUE(probe_small_positive(true,true,std::nan(""),check).empty());
  EXPECT_EQ(calls,0);
}
TEST(BoundedSpeedGrid, AdaptivePositiveDoesNotSwallowInvalidInputsOrClaimMonotonicity) {
  EXPECT_THROW(probe_small_positive(true,true,1.,[](double)->bool {
    throw std::invalid_argument("malformed profile");
  }),std::invalid_argument);
  std::vector<double> accepted;
  const auto attempted=probe_small_positive(true,true,1.,[&](double x) {
    const bool okay=x<.15 || (x>.3 && x<.4);
    if(okay)accepted.push_back(x);return okay;
  });
  EXPECT_EQ(attempted.size(),6U);
  EXPECT_FALSE(accepted.empty());
  // Disconnected feasible regions are not exhaustively searched or declared absent.
}
