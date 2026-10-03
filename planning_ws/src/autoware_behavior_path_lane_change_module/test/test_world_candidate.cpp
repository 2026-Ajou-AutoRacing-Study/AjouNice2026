#include "autoware/behavior_path_lane_change_module/utils/world_candidate.hpp"
#include <gtest/gtest.h>
namespace wc=autoware::behavior_path_planner::utils::lane_change::world_candidate;
TEST(WorldCandidateDraft, DefaultOffAndNormalOnly) {
 EXPECT_FALSE(wc::enabled(false,true,true,true,true,true,true));
 EXPECT_FALSE(wc::enabled(true,false,true,true,true,true,true));
 EXPECT_TRUE(wc::enabled(true,true,true,true,true,true,true));
 for(size_t absent=2;absent<7;++absent){bool gates[7]={true,true,true,true,true,true,true};gates[absent]=false;
  EXPECT_FALSE(wc::enabled(gates[0],gates[1],gates[2],gates[3],gates[4],gates[5],gates[6]));}
}
TEST(WorldCandidateDraft, MissingContextNeverInvokesBorrowedCallbacks) {
 wc::Input input;input.deadline=ros::WallTime::now()+ros::WallDuration(1.);
 wc::Services services;bool called=false;services.native_reference=[&](const auto&){called=true;return autoware_internal_planning_msgs::PathWithLaneId{};};
 EXPECT_THROW(wc::make(input,services),std::logic_error);EXPECT_FALSE(called);
}
TEST(WorldCandidateDraft, ExpiredAndAbsentDeadlineFailClosed) {
 wc::Input input;wc::Services services;
 EXPECT_THROW(wc::make(input,services),std::logic_error);
 input.deadline=ros::WallTime::now()-ros::WallDuration(1.);
 EXPECT_THROW(wc::make(input,services),std::logic_error);
}

namespace bpp=autoware::behavior_path_planner;
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace {
bpp::PathPointWithLaneId point(double x,double y,double yaw=0.) {
 bpp::PathPointWithLaneId p;p.point.pose.position.x=x;p.point.pose.position.y=y;
 tf2::Quaternion q;q.setRPY(0,0,yaw);p.point.pose.orientation=tf2::toMsg(q);
 p.point.longitudinal_velocity_mps=15;p.lane_ids={42};return p;
}
struct Fixture {
 bpp::PathWithLaneId prep,ref;geometry_msgs::Pose end;size_t start{},finish{};
 Fixture(){prep.points={point(-4,0),point(-2,0),point(0,0)};
  ref.points={point(0,3.5),point(20,3.5),point(40,3.5),point(60,3.5),point(80,3.5)};
  end=point(39.5,3.5).point.pose;}
 auto make(){return wc::detail::construct_geometry(prep,ref,end,start,finish);}
};
}
TEST(WorldGeometryDraft, ValidStraightPreservesPrefixEndpointAndSuffix) {
 Fixture f;const auto out=f.make();ASSERT_GT(f.finish,f.start);EXPECT_EQ(f.start,2u);
 for(size_t i=0;i<f.prep.points.size();++i)EXPECT_EQ(out.points[i],f.prep.points[i]);
 EXPECT_EQ(out.points[f.finish].point.pose,f.end);
 ASSERT_EQ(out.points.size()-f.finish-1,3u);
 for(size_t i=0;i<3;++i)EXPECT_EQ(out.points[f.finish+1+i],f.ref.points[2+i]);
}
TEST(WorldGeometryDraft, ValidCurvedFiniteGeometryAndEndpoint) {
 Fixture f;f.prep.points={point(-4,.04,-.02),point(-2,.01,-.01),point(0,0)};
 f.ref.points={point(0,3.5),point(20,3.7,.02),point(40,4.3,.04),point(60,5.3,.06),point(80,6.7,.08)};
 f.end=point(39.5,4.285,.04).point.pose;const auto out=f.make();
 EXPECT_EQ(out.points[f.finish].point.pose,f.end);
 for(const auto&p:out.points)EXPECT_TRUE(std::isfinite(p.point.pose.position.x+p.point.pose.position.y+tf2::getYaw(p.point.pose.orientation)));
}
TEST(WorldGeometryDraft, InternalStopRejectedAndSuffixStopExact) {
 Fixture f;f.ref.points[1].point.longitudinal_velocity_mps=0;EXPECT_THROW(f.make(),std::logic_error);
 Fixture g;g.ref.points[3].point.longitudinal_velocity_mps=0;const auto out=g.make();
 EXPECT_EQ(out.points[g.finish+2],g.ref.points[3]);
}
TEST(WorldGeometryDraft, ExactVertexDoesNotDuplicateSuffixJoin) {
 Fixture f;f.end=f.ref.points[2].point.pose;const auto out=f.make();
 ASSERT_LT(f.finish+1,out.points.size());EXPECT_EQ(out.points[f.finish+1],f.ref.points[3]);
 EXPECT_NE(out.points[f.finish].point.pose.position,out.points[f.finish+1].point.pose.position);
}
TEST(WorldGeometryDraft, DegenerateAndMissingEndpointReject) {
 Fixture f;f.prep.points[1]=f.prep.points[2];EXPECT_THROW(f.make(),std::logic_error);
 Fixture g;g.ref.points[2]=g.ref.points[1];EXPECT_THROW(g.make(),std::logic_error);
 Fixture h;h.end.position.y+=1;EXPECT_THROW(h.make(),std::logic_error);
}
TEST(WorldGeometryDraft, NonfinitePoseAndQuaternionRejectBeforeAllocation) {
 for(double bad:{NAN,INFINITY}) {
  Fixture a;a.end.position.x=bad;EXPECT_THROW(a.make(),std::logic_error);
  Fixture b;b.prep.points[0].point.pose.position.y=bad;EXPECT_THROW(b.make(),std::logic_error);
  Fixture c;c.ref.points[2].point.pose.orientation.w=bad;EXPECT_THROW(c.make(),std::logic_error);
 }
 Fixture d;d.end.orientation.w=0;EXPECT_THROW(d.make(),std::logic_error);
}
