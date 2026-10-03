#include <autoware/path_optimizer/c2_bridge.hpp>
#include <autoware/path_optimizer/obstacle_collision.hpp>
#include <autoware/path_optimizer/utils/geometry_utils.hpp>
#include <autoware/path_optimizer/utils/trajectory_utils.hpp>
#include <gtest/gtest.h>
#include <limits>

using namespace autoware::path_optimizer;
namespace
{
const autoware::motion_utils::AccelerationLimits calibration({{0.,3.7},{40.,3.25}});
std::vector<TrajectoryPoint> line(double offset=0.0)
{
  std::vector<TrajectoryPoint> p(61);
  for (size_t i=0;i<p.size();++i) {
    p[i].pose.position.x=i*0.5;p[i].pose.position.y=offset;
    p[i].pose.orientation.w=1.0;p[i].longitudinal_velocity_mps=10.0;
  }
  return p;
}
auto prefix()
{
  auto p=line(.15);p.resize(11);return p;
}
}
TEST(C2Bridge, PrefixSuffixAndEndpointsPreserved)
{
  const auto before=prefix(), input=line();
  const auto r=detail::makeC2Bridge(before,input,40,.28);
  for(size_t i=0;i<before.size();++i)EXPECT_EQ(r.points[i],before[i]);
  EXPECT_EQ(r.points[r.end].pose,input[40].pose);
  for(size_t i=41;i<input.size();++i)EXPECT_EQ(r.points[r.end+i-40],input[i]);
  for(size_t i=r.begin;i<=r.end;++i) {
    EXPECT_GE(r.points[i].pose.position.y,-1e-8);
    EXPECT_LE(r.points[i].pose.position.y,.15+1e-8);
    EXPECT_LE(std::abs(tf2::getYaw(r.points[i].pose.orientation)),.03);
  }
  EXPECT_FALSE(firstCurvatureViolation(r.points,1,r.end+1,.28));
}
TEST(C2Bridge, AtomicRejectionPreservesExactBaselineIncludingStops)
{
  auto baseline=line();
  baseline[17].longitudinal_velocity_mps=0.0;
  const auto original=baseline;
  bool adopted=true;std::string rejection;
  const auto result=detail::adoptValidatedBridgeOrBaseline(baseline,[&]() {
    auto candidate=baseline;
    candidate[17].longitudinal_velocity_mps=20.0;
    candidate[2].pose.position.y=4;
    throw std::logic_error("road certificate rejected");
    return candidate;
  },adopted,rejection);
  EXPECT_FALSE(adopted);EXPECT_EQ(result,original);EXPECT_EQ(baseline,original);
  EXPECT_EQ(rejection,"road certificate rejected");
  // Rollback is not a safety decision: downstream still sees (and must check)
  // even an unsafe legacy geometry rather than silently approving it here.
  baseline[0].pose.position.y=100;
  const auto unsafe=detail::adoptValidatedBridgeOrBaseline(baseline,[]() {
    return std::vector<TrajectoryPoint>{};
  },adopted,rejection);
  EXPECT_FALSE(adopted);EXPECT_EQ(unsafe,baseline);
}
TEST(C2Bridge, AtomicAdoptionOnlyAfterCompletedValidation)
{
  const auto baseline=line();bool checked=false,adopted=false;std::string rejection="old";
  auto candidate=prefix();
  const auto result=detail::adoptValidatedBridgeOrBaseline(baseline,[&]() {
    checked=true;return candidate;
  },adopted,rejection);
  EXPECT_TRUE(checked);EXPECT_TRUE(adopted);EXPECT_TRUE(rejection.empty());
  EXPECT_EQ(result,candidate);
  EXPECT_THROW(detail::adoptValidatedBridgeOrBaseline(baseline,[]() {
    throw std::runtime_error("unexpected failure");return std::vector<TrajectoryPoint>{};
  },adopted,rejection),std::runtime_error);
}
TEST(C2Bridge, AdaptiveSweptCertificatePreservesRealRadius)
{
  geometry_msgs::Pose a,b;a.orientation.w=b.orientation.w=1;
  b.position.x=.2;
  size_t calls=0;
  const auto narrow=[&](const auto &pose,const auto &r,const auto &) {
    ++calls;return std::abs(pose.position.y)+r.at(0)>=1.01;
  };
  EXPECT_TRUE(detail::certifySweptCircleSegment(a,b,{1.0},{0.0},narrow));
  EXPECT_GT(calls,1u); // Coarse 1.1m certificate failed, true radius unchanged.
  a.position.y=b.position.y=.02;
  EXPECT_FALSE(detail::certifySweptCircleSegment(a,b,{1.0},{0.0},narrow));
  a.position.y=b.position.y=0;
  const auto interior_cutout=[](const auto &pose,const auto &r,const auto &) {
    return r.at(0)>((pose.position.x>.045 && pose.position.x<.055)? .9:1.01);
  };
  EXPECT_FALSE(detail::certifySweptCircleSegment(a,b,{1.0},{0.0},interior_cutout));
  const auto budget=[](const auto &,const auto &,const auto &)->bool {
    throw std::logic_error("budget");
  };
  EXPECT_THROW(detail::certifySweptCircleSegment(a,b,{1.0},{0.0},budget),std::logic_error);
  EXPECT_FALSE(detail::certifySweptCircleSegment(a,b,{}, {},narrow));
}
TEST(C2Bridge, InvalidInputsFailClosed)
{
  auto a=prefix(), b=line();
  EXPECT_THROW(detail::makeC2Bridge({},b,40,.28),std::logic_error);
  EXPECT_THROW(detail::makeC2Bridge(a,b,0,.28),std::logic_error);
  EXPECT_THROW(detail::makeC2Bridge(a,b,b.size()-1,.28),std::logic_error);
  EXPECT_THROW(detail::makeC2Bridge(a,b,40,0),std::logic_error);
  a.back().pose.position.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(detail::makeC2Bridge(a,b,40,.28),std::logic_error);
  a=prefix();a.back().pose.orientation.w=0;
  EXPECT_THROW(detail::makeC2Bridge(a,b,40,.28),std::logic_error);
  a=prefix();a[a.size()-2]=a.back();
  EXPECT_THROW(detail::makeC2Bridge(a,b,40,.28),std::logic_error);
}
TEST(C2Bridge, StopAndInputCapPreservation)
{
  auto a=prefix(), b=line();
  b[25].longitudinal_velocity_mps=3.0;
  for(size_t i=30;i<b.size();++i)b[i].longitudinal_velocity_mps=0.0;
  auto r=detail::makeC2Bridge(a,b,40,.28);
  // Same stop-pose insertion as extendTrajectory. Geometry prototype alone is
  // not allowed to declare interior stop preservation.
  const size_t segment=autoware::motion_utils::findNearestSegmentIndex(r.points,b[30].pose.position);
  trajectory_utils::insertStopPoint(r.points,b[30].pose,segment);
  auto stop=std::find_if(r.points.begin(),r.points.end(),[](const auto &p){return p.longitudinal_velocity_mps==0;});
  ASSERT_NE(stop,r.points.end());EXPECT_NEAR(stop->pose.position.x,15.0,.1);
  // insertStopPoint inserts the first zero; onPath's unchanged
  // setZeroVelocityAfterStopPoint performs the subsequent zero propagation.
  EXPECT_EQ(r.points.back().longitudinal_velocity_mps,0.0);
  const auto near=autoware::motion_utils::findNearestIndex(r.points,b[25].pose.position);
  EXPECT_LE(r.points[near].longitudinal_velocity_mps,3.0);
  a.back().longitudinal_velocity_mps=0.0;
  r=detail::makeC2Bridge(a,b,40,.28);
  for(size_t i=r.begin;i<=r.end;++i)EXPECT_EQ(r.points[i].longitudinal_velocity_mps,0.0);
}
TEST(C2Bridge, BridgeEntireRoadAndObstacleChecks)
{
  const auto r=detail::makeC2Bridge(prefix(),line(),40,.28);
  std::vector<geometry_msgs::Point> left(2),right(2);
  left[0].x=-10;left[1].x=50;left[0].y=left[1].y=3;
  right[0].x=-10;right[1].x=50;right[0].y=right[1].y=-3;
  geometry_utils::CircleFootprintRoadChecker wide(left,right);
  for(size_t i=r.begin;i<=r.end;++i)EXPECT_FALSE(wide.isOutside(r.points[i].pose,{1.0},{0.0}));
  left[0].y=left[1].y=.5;
  geometry_utils::CircleFootprintRoadChecker narrow(left,right);
  EXPECT_TRUE(narrow.isOutside(r.points[r.begin].pose,{1.0},{0.0}));
  // The first half is clear but the bridge's far end is not: checking only
  // the legacy short optimization horizon must not approve the whole bridge.
  left.resize(4);
  left[0].x=-10;left[0].y=3;left[1].x=16;left[1].y=3;
  left[2].x=18;left[2].y=.5;left[3].x=50;left[3].y=.5;
  geometry_utils::CircleFootprintRoadChecker far_cutout(left,right);
  EXPECT_FALSE(far_cutout.isOutside(r.points[r.begin].pose,{1.0},{0.0}));
  EXPECT_TRUE(far_cutout.isOutside(r.points[r.end].pose,{1.0},{0.0}));
  autoware_planning_msgs::StaticObstacle obstacle;
  for(auto xy:std::vector<std::pair<float,float>>{{17,-1},{18,-1},{18,1},{17,1}}) {
    geometry_msgs::Point32 p;p.x=xy.first;p.y=xy.second;obstacle.polygon.points.push_back(p);
  }
  autoware::vehicle_info_utils::VehicleInfo vehicle;
  vehicle.wheel_base_m=3;vehicle.front_overhang_m=.845;vehicle.rear_overhang_m=.79;vehicle.vehicle_width_m=1.89;
  EXPECT_TRUE(firstObstacleCollision(r.points,r.begin,{obstacle},vehicle));
  EXPECT_FALSE(firstObstacleCollision(r.points,r.begin,{},vehicle));
}

TEST(C2Bridge, RisingTransportIsOptInAndPreservesGeometryPrefixSuffix)
{
  auto a=prefix(),b=line();
  for(auto &p:b)p.longitudinal_velocity_mps=std::sqrt(100+4*p.pose.position.x);
  a.back().longitudinal_velocity_mps=std::sqrt(120.);
  const auto legacy=detail::makeC2Bridge(a,b,40,.28);
  const auto off=detail::makeC2Bridge(a,b,40,.28,false);
  const auto on=detail::makeC2Bridge(a,b,40,.28,true,&calibration);
  EXPECT_EQ(legacy.points,off.points);
  ASSERT_EQ(on.points.size(),off.points.size());
  for(size_t i=0;i<on.points.size();++i) {
    EXPECT_EQ(on.points[i].pose,off.points[i].pose);
    if(i<=on.begin||i>on.end)EXPECT_EQ(on.points[i],off.points[i]);
  }
  EXPECT_GT(on.points[on.end-1].longitudinal_velocity_mps,
    off.points[off.end-1].longitudinal_velocity_mps+1.0);
  for(size_t i=on.begin+1;i<=on.end;++i) {
    const auto&p=on.points[i];const auto&q=on.points[i-1];
    const double ds=std::hypot(p.pose.position.x-q.pose.position.x,p.pose.position.y-q.pose.position.y);
    const double a_arc=(double(p.longitudinal_velocity_mps)*p.longitudinal_velocity_mps-
      double(q.longitudinal_velocity_mps)*q.longitudinal_velocity_mps)/(2*ds);
    EXPECT_LE(a_arc,2.001); // Input v^2 gradient: a=2, including float storage.
    EXPECT_LE(p.longitudinal_velocity_mps,std::sqrt(100+4*p.pose.position.x)+1e-5);
  }
}

TEST(C2Bridge, RisingTransportAnchorsLowerPrefixWithoutInstantJump)
{
  auto a=prefix(),b=line();
  for(auto&p:b)p.longitudinal_velocity_mps=std::sqrt(144+4*p.pose.position.x);
  const auto on=detail::makeC2Bridge(a,b,40,.28,true,&calibration);
  EXPECT_EQ(on.points[on.begin],a.back());
  EXPECT_LT(on.points[on.begin+1].longitudinal_velocity_mps,10.02);
  EXPECT_GT(on.points[on.end].longitudinal_velocity_mps,12.);
  EXPECT_LT(on.points[on.end].longitudinal_velocity_mps,b[40].longitudinal_velocity_mps);
}

TEST(C2Bridge, FlatFallingAndZeroTransportKeepOriginalLimits)
{
  for(int mode:{0,1,2}) {
    auto a=prefix(),b=line();
    if(mode==1)for(auto&p:b)p.longitudinal_velocity_mps=10-.1*p.pose.position.x;
    if(mode==2)b[25].longitudinal_velocity_mps=0;
    const auto old=detail::makeC2Bridge(a,b,40,.28);
    const auto on=detail::makeC2Bridge(a,b,40,.28,true,&calibration);
    EXPECT_EQ(on.points,old.points);
  }
  auto a=prefix(),b=line();a.back().longitudinal_velocity_mps=0;
  for(auto&p:b)p.longitudinal_velocity_mps=20;
  const auto on=detail::makeC2Bridge(a,b,40,.28,true,&calibration);
  for(size_t i=on.begin;i<=on.end;++i)EXPECT_EQ(on.points[i].longitudinal_velocity_mps,0);
}

TEST(C2Bridge, InvalidRisingInputRollsBackAtomically)
{
  auto a=prefix(),b=line();b[20].longitudinal_velocity_mps=std::numeric_limits<float>::quiet_NaN();
  bool adopted=true;std::string reason;
  const auto output=detail::adoptValidatedBridgeOrBaseline(a,[&]{
    return detail::makeC2Bridge(a,b,40,.28,true,&calibration).points;
  },adopted,reason);
  EXPECT_FALSE(adopted);EXPECT_EQ(output,a);EXPECT_FALSE(reason.empty());
}

TEST(C2Bridge, StaircaseTransportUsesFasterEndCalibrationAndRequiresMap)
{
  auto a=prefix(),b=line();
  for(size_t i=0;i<b.size();++i)b[i].longitudinal_velocity_mps=std::sqrt(100.+10.*(i/2));
  a.back().longitudinal_velocity_mps=b[10].longitudinal_velocity_mps;
  EXPECT_THROW(detail::makeC2Bridge(a,b,40,.28,true),std::logic_error);
  const autoware::motion_utils::AccelerationLimits disabled;
  EXPECT_THROW(detail::makeC2Bridge(a,b,40,.28,true,&disabled),std::logic_error);
  const auto on=detail::makeC2Bridge(a,b,40,.28,true,&calibration);
  for(size_t i=on.begin+1;i<=on.end;++i) {
    const auto &p=on.points[i-1],&q=on.points[i];
    const double ds=std::hypot(q.pose.position.x-p.pose.position.x,q.pose.position.y-p.pose.position.y);
    const double accel=(double(q.longitudinal_velocity_mps)*q.longitudinal_velocity_mps-
      double(p.longitudinal_velocity_mps)*p.longitudinal_velocity_mps)/(2*ds);
    EXPECT_LE(accel,calibration.at(q.longitudinal_velocity_mps)+1e-8);
  }
}

TEST(C2Bridge, FallingThenRisingCannotWorsenUnchangedSuffixSeam)
{
  auto a=prefix(),b=line();
  b[35].longitudinal_velocity_mps=1.;
  bool adopted=true;std::string reason;
  const auto legacy=detail::makeC2Bridge(a,b,40,.28);
  const auto result=detail::adoptValidatedBridgeOrBaseline(legacy.points,[&]{
    return detail::makeC2Bridge(a,b,40,.28,true,&calibration).points;
  },adopted,reason);
  EXPECT_FALSE(adopted);EXPECT_EQ(result,legacy.points);
  EXPECT_EQ(reason,"C2 bridge worsened suffix velocity seam");
}

int main(int argc,char ** argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
