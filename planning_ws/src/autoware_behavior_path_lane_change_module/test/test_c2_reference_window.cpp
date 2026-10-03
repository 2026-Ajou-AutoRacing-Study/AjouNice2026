#include "autoware/behavior_path_lane_change_module/utils/c2_reference_window.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include <gtest/gtest.h>
#include <array>
#include <limits>
namespace cw=autoware::behavior_path_planner::utils::lane_change::c2_reference_window;
namespace {
cw::Path raw() {
 cw::Path p;
 const std::vector<double>x{0,12,40,45,72,110,150,200},y{0,.01,.12,.08,.02,0,0,0};
 for(size_t i=0;i<x.size();++i){autoware_internal_planning_msgs::PathPointWithLaneId q;
  q.point.pose.position.x=x[i];q.point.pose.position.y=y[i];q.point.pose.position.z=2;
  q.point.pose.orientation.w=1;q.point.longitudinal_velocity_mps=20;q.lane_ids={100+int64_t(i)};p.points.push_back(q);}
 return p;
}
cw::Path native(const cw::Path&p){return autoware::behavior_path_planner::utils::resamplePathWithSpline(p,5,true,{0.,100.});}
}
TEST(C2ReferenceWindow, OffIsExactEvenForInvalidInput) {
 auto p=raw(),n=native(p);p.points[2].point.pose.position.x=std::numeric_limits<double>::quiet_NaN();
 const auto r=cw::apply(p,n,-1,100,false);EXPECT_FALSE(r.applied);EXPECT_EQ(r.path,n);EXPECT_EQ(r.reason,"disabled");
}
TEST(C2ReferenceWindow, ScopeRequiresEveryGuardAndOriginalApiRemainsOff) {
 const auto p=raw(),n=native(p);
 EXPECT_TRUE(cw::enabled_for_candidate(true,true,true,true,true,true));
 for(size_t i=0;i<6;++i){std::array<bool,6> flags{true,true,true,true,true,true};flags[i]=false;
  const bool on=cw::enabled_for_candidate(flags[0],flags[1],flags[2],flags[3],flags[4],flags[5]);
  EXPECT_FALSE(on);EXPECT_EQ(cw::apply(p,n,5,100,on).path,n);}
 // Both external-request and avoidance modules pass normal=false.
 EXPECT_FALSE(cw::enabled_for_candidate(true,false,true,true,true,true));
}
TEST(C2ReferenceWindow, KeepsSamplesMetadataAndProtectedCompletePoses) {
 const auto p=raw(),n=native(p);const auto r=cw::apply(p,n,5,100,true);
 ASSERT_TRUE(r.applied)<<r.reason;ASSERT_EQ(r.path.points.size(),n.points.size());
 for(size_t i=0;i<n.points.size();++i) {
  const auto&a=n.points[i];const auto&b=r.path.points[i];
  EXPECT_EQ(a.lane_ids,b.lane_ids);EXPECT_EQ(a.point.longitudinal_velocity_mps,b.point.longitudinal_velocity_mps);
  EXPECT_EQ(a.point.pose.position.z,b.point.pose.position.z);
  EXPECT_LE(std::hypot(a.point.pose.position.x-b.point.pose.position.x,a.point.pose.position.y-b.point.pose.position.y),.25);
  if(a.point.pose.position.x<=9.9||a.point.pose.position.x>=100.)EXPECT_EQ(a,b);
 }
 EXPECT_EQ(n.points.front(),r.path.points.front());EXPECT_EQ(n.points.back(),r.path.points.back());
}
TEST(C2ReferenceWindow, NativeParityMismatchFallsBackAtomically) {
 const auto p=raw();auto n=native(p);n.points[3].lane_ids.push_back(999);
 const auto r=cw::apply(p,n,5,100,true);EXPECT_FALSE(r.applied);EXPECT_EQ(r.path,n);EXPECT_EQ(r.reason,"native_sample_parity_mismatch");
}
TEST(C2ReferenceWindow, StopInsideWindowIsNotMoved) {
 auto p=raw();for(size_t i=3;i<p.points.size();++i)p.points[i].point.longitudinal_velocity_mps=0;
 const auto n=native(p);const auto r=cw::apply(p,n,5,100,true);
 EXPECT_FALSE(r.applied);EXPECT_EQ(r.path,n);EXPECT_EQ(r.reason,"stop_or_reverse_in_window");
}
TEST(C2ReferenceWindow, CanonicalSuffixStopsAreExactlyPreserved) {
 auto p=raw();p.points[6].point.longitudinal_velocity_mps=0;p.points[7].point.longitudinal_velocity_mps=0;
 const auto n=native(p);const auto r=cw::apply(p,n,5,100,true);ASSERT_TRUE(r.applied)<<r.reason;
 for(size_t i=0;i<n.points.size();++i)if(n.points[i].point.longitudinal_velocity_mps==0)EXPECT_EQ(n.points[i],r.path.points[i]);
}
TEST(C2ReferenceWindow, InvalidOrStationaryGeometryRetainsNative) {
 const auto original=raw(),n=native(original);auto p=original;p.points[2].point.pose.position=p.points[1].point.pose.position;
 EXPECT_FALSE(cw::apply(p,n,5,100,true).applied);
 p=original;p.points[2].point.pose.position.y=std::numeric_limits<double>::quiet_NaN();
 EXPECT_EQ(cw::apply(p,n,5,100,true).path,n);
 EXPECT_FALSE(cw::apply(original,n,5,20,true).applied);
 EXPECT_FALSE(cw::apply(original,n,5,300,true).applied);
}
TEST(C2ReferenceWindow, LargeReferenceDeformationDoesNotEscapeBound) {
 auto p=raw();p.points[2].point.pose.position.y=20;p.points[3].point.pose.position.y=-20;
 const auto n=native(p);const auto r=cw::apply(p,n,5,100,true);
 ASSERT_TRUE(r.applied)<<r.reason;
 double maximum=0;
 for(size_t i=0;i<n.points.size();++i)maximum=std::max(maximum,std::hypot(
   n.points[i].point.pose.position.x-r.path.points[i].point.pose.position.x,
   n.points[i].point.pose.position.y-r.path.points[i].point.pose.position.y));
 EXPECT_NEAR(maximum,.25,1e-10);
 EXPECT_EQ(r.path.points.front(),n.points.front());EXPECT_EQ(r.path.points.back(),n.points.back());
}
TEST(C2ReferenceWindow, DeduplicatedMandatoryEndpointAndSuffixAreCompleteMessages) {
 auto p=raw();
 // Select an existing native input station 0.1 m before requested length.
 std::vector<autoware_planning_msgs::PathPoint> points;
 for(const auto&q:p.points)points.push_back(q.point);
 const auto arc=autoware::motion_utils::calcSignedArcLengthPartialSum(points,0,points.size());
 const double length=arc[4]+.1;
 const auto n=autoware::behavior_path_planner::utils::resamplePathWithSpline(p,5,true,{0.,length});
 const auto r=cw::apply(p,n,5,length,true);ASSERT_TRUE(r.applied)<<r.reason;
 const auto endpoint=autoware::motion_utils::findNearestIndex(n.points,p.points[4].point.pose.position);
 ASSERT_LT(endpoint,n.points.size());
 for(size_t i=endpoint;i<n.points.size();++i)EXPECT_EQ(r.path.points[i],n.points[i]);
 EXPECT_EQ(r.path.points[endpoint].point.pose,n.points[endpoint].point.pose);
}
