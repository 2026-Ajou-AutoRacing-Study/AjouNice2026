#include <gtest/gtest.h>
#include "autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp"
#include "autoware/motion_utils/trajectory/acceleration_limits.hpp"

namespace {
const std::vector<double> arc{0,2,6,10,30,60,95,125.64055146183378,180,360.2308123150218};
constexpr double ceiling=33.333333333333336;
constexpr double v0=25.902146970435,a0=1.1494142818617155,H=2.974976539;
const autoware::motion_utils::AccelerationLimits calibration({
  {0,3.7},{2.777777777777778,3.7},{5.555555555555556,3.5},
  {16.666666666666668,3.4},{33.333333333333336,3.25}});
double limit(double v){return calibration.limit(v,3.7);}
auto make(const std::vector<double>&s,const std::vector<double>&caps,double v=v0,double a=a0){
  return bounded_speed_grid::make_uniform_cap_coast(s,caps,v,a,limit(v),H,-10,100,limit);
}
}

TEST(BoundedCapCoast, ActualStateHorizonAndPhysicalBounds){
  const auto p=make(arc,std::vector<double>(arc.size(),ceiling));
  EXPECT_DOUBLE_EQ(p.samples.front().v,v0);EXPECT_DOUBLE_EQ(p.samples.front().a,a0);
  bool exact_h=false;double peak_v=0;
  for(size_t i=1;i<p.samples.size();++i){const auto&x=p.samples[i-1];const auto&y=p.samples[i];
    const double dt=y.t-x.t,j=(y.a-x.a)/dt;
    ASSERT_GT(dt,0);EXPECT_GE(j,-10-1e-7);EXPECT_LE(j,100+1e-7);
    EXPECT_GE(y.a,0);EXPECT_LE(y.a,limit(y.v)+1e-7);EXPECT_LE(y.v,ceiling+1e-8);
    EXPECT_NEAR(y.s-x.s,x.v*dt+.5*x.a*dt*dt+j*dt*dt*dt/6,1e-7);
    EXPECT_NEAR(y.v-x.v,x.a*dt+.5*j*dt*dt,1e-7);
    if(std::abs(y.t-H)<1e-10){exact_h=true;EXPECT_DOUBLE_EQ(p.progress,y.s);}
    peak_v=std::max(peak_v,y.v);
  }
  EXPECT_TRUE(exact_h);EXPECT_NEAR(p.progress,90.7164552422,1e-6);
  EXPECT_NEAR(peak_v,ceiling,1e-8);EXPECT_NEAR(p.samples.back().s,arc.back(),1e-7);
  ASSERT_EQ(p.speeds.size(),arc.size());for(double v:p.speeds)EXPECT_LE(v,ceiling+1e-8);
}

TEST(BoundedCapCoast, DoesNotChangeLegacyConstantAccelerationRejection){
  const std::vector<double>caps(arc.size(),ceiling);
  EXPECT_THROW(bounded_speed_grid::make(arc,caps,v0,a0,limit(v0),H,-4,-10,100,limit),std::logic_error);
  EXPECT_NO_THROW(make(arc,caps));
}

TEST(BoundedCapCoast, RejectsVariableZeroAndNonfiniteCeilings){
  for(double last:{0.,30.,std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::infinity()}){
    auto caps=std::vector<double>(arc.size(),ceiling);caps.back()=last;
    EXPECT_THROW(make(arc,caps),std::logic_error);
  }
}

TEST(BoundedCapCoast, RejectsInvalidOrInfeasibleInitialState){
  const std::vector<double>caps(arc.size(),ceiling);
  EXPECT_THROW(make(arc,caps,33.3,2),std::logic_error);
  EXPECT_THROW(make(arc,caps,v0,4),std::logic_error);
  EXPECT_THROW(make(arc,caps,v0,-1),std::logic_error);
  EXPECT_THROW(make(arc,caps,ceiling+1,0),std::logic_error);
}

TEST(BoundedCapCoast, RejectsMissingCoverageAndMalformedGeometry){
  EXPECT_THROW(make({},{}),std::logic_error);
  EXPECT_THROW(make({0,1},{ceiling,ceiling}),std::logic_error);
  EXPECT_THROW(make({0,0,100},{ceiling,ceiling,ceiling}),std::logic_error);
  EXPECT_THROW(bounded_speed_grid::make_uniform_cap_coast(arc,
    std::vector<double>(arc.size(),ceiling),v0,a0,3,H,-10,100,{}),std::logic_error);
}

TEST(BoundedCapCoast, RejectsSharpValidCalibrationWithoutRelaxingJerk){
  const autoware::motion_utils::AccelerationLimits sharp({
    {0,1},{10,1},{10.001,.01},{100,.01}});
  EXPECT_THROW(bounded_speed_grid::make_uniform_cap_coast({0,50,100},{100,100,100},
    10,1,1,3,-10,100,[&](double v){return sharp.limit(v,1);}),std::logic_error);
}

TEST(BoundedCapCoast, ExhaustedBudgetNeverReturnsPartialProposal){
  unsigned calls=0;
  EXPECT_THROW(bounded_speed_grid::make_uniform_cap_coast(arc,
    std::vector<double>(arc.size(),ceiling),v0,a0,limit(v0),H,-10,100,limit,
    [&](){return ++calls<3;}),std::logic_error);
  EXPECT_EQ(calls,3u);
}
