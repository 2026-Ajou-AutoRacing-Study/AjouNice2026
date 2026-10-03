#include "autoware/behavior_path_lane_change_module/utils/preferred_path_hazard.hpp"
#include <autoware_internal_planning_msgs/PathWithLaneId.h>
#include <gtest/gtest.h>
namespace hazard=autoware::behavior_path_planner::lane_change::preferred_path_hazard;
using Polygon=boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>;
struct VehicleBox {double min_longitudinal_offset_m=-.79,max_longitudinal_offset_m=3.845;
  double min_lateral_offset_m=-.945,max_lateral_offset_m=.945;};
namespace {
autoware_internal_planning_msgs::PathWithLaneId path() {
  autoware_internal_planning_msgs::PathWithLaneId p;p.points.resize(2);
  for(auto&v:p.points)v.point.pose.orientation.w=1;
  p.points.back().point.pose.position.x=100;return p;
}
geometry_msgs::Pose pose(double x,double y) {geometry_msgs::Pose p;p.position.x=x;p.position.y=y;p.orientation.w=1;return p;}
Polygon box(double x,double y) {Polygon p;
  for(const auto&v:{std::pair<double,double>{x-2.5,y-1},{x+2.5,y-1},{x+2.5,y+1},{x-2.5,y+1}})
    p.outer().emplace_back(v.first,v.second);
  boost::geometry::correct(p);return p;
}
bool eligible(bool enabled=true,bool normal=true,bool approved=false,bool fresh=true,bool epoch=true,
              double t=1.7,double H=2.884,double d=80.913,double gap=80.913) {
  return hazard::eligible(enabled,normal,approved,fresh,epoch,t,H,d,gap,77.517/3.6,11.07157,3.7,4.635);
}
}
TEST(PreferredPathHazard, DefaultOffExternalApprovedStaleOtherEpochCannotRelease) {
  EXPECT_TRUE(eligible());EXPECT_FALSE(eligible(false));EXPECT_FALSE(eligible(true,false));
  EXPECT_FALSE(eligible(true,true,true));EXPECT_FALSE(eligible(true,true,false,false));
  EXPECT_FALSE(eligible(true,true,false,true,false));
}
TEST(PreferredPathHazard, SameExistingHorizonAndRangeNoExpansion) {
  EXPECT_FALSE(eligible(true,true,false,true,true,3.));
  EXPECT_FALSE(eligible(true,true,false,true,true,-.01));
  EXPECT_FALSE(eligible(true,true,false,true,true,1.,2.884,200.));
  EXPECT_FALSE(eligible(true,true,false,true,true,1.,2.884,10.,-1.));
  EXPECT_FALSE(eligible(true,true,false,true,true,1.,std::numeric_limits<double>::quiet_NaN()));
}
TEST(PreferredPathHazard, RecordedR49EarlierOutOfRangeLaterPotentiallyConstraining) {
  EXPECT_FALSE(hazard::eligible(true,true,false,true,true,2.6,2.884,89.7543,89.7543,
    65.884/3.6,11.0425,3.7,4.635));
  EXPECT_TRUE(eligible()); // Scalar public-packet proof, not private LC approval.
}
TEST(PreferredPathHazard, FasterObjectDoesNotRelease) {
  EXPECT_FALSE(hazard::eligible(true,true,false,true,true,1.,3.,20.,20.,20.,25.,3.7,4.635));
}
TEST(PreferredPathHazard, ActualVehicleFootprintIntersection) {
  EXPECT_TRUE(hazard::intersects_input_path(path(),pose(30,0),box(30,0),VehicleBox{},.785));
}
TEST(PreferredPathHazard, AdjacentNonArrivingObjectDoesNotRelease) {
  EXPECT_FALSE(hazard::intersects_input_path(path(),pose(30,3.6),box(30,3.6),VehicleBox{},.785));
}
TEST(PreferredPathHazard, FutureFootprintEntersSameInputPath) {
  EXPECT_FALSE(hazard::intersects_input_path(path(),pose(30,3.6),box(30,3.6),VehicleBox{},.785));
  EXPECT_TRUE(hazard::intersects_input_path(path(),pose(45,1.5),box(45,1.5),VehicleBox{},.785));
}
TEST(PreferredPathHazard, OncomingCrossingAndExtrapolatedEndpointRejected) {
  auto p=pose(30,0);p.orientation.z=1;p.orientation.w=0;
  EXPECT_FALSE(hazard::intersects_input_path(path(),p,box(30,0),VehicleBox{},.785));
  p.orientation.z=p.orientation.w=std::sqrt(.5);
  EXPECT_FALSE(hazard::intersects_input_path(path(),p,box(30,0),VehicleBox{},.785));
  EXPECT_FALSE(hazard::intersects_input_path(path(),pose(101,0),box(101,0),VehicleBox{},.785));
}
TEST(PreferredPathHazard, MalformedPathObjectBoxNeverCertifiesHazard) {
  auto p=path();p.points[0].point.pose.position.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(hazard::intersects_input_path(p,pose(30,0),box(30,0),VehicleBox{},.785));
  p=path();p.points[0].point.pose.orientation.w=0;
  EXPECT_FALSE(hazard::intersects_input_path(p,pose(30,0),box(30,0),VehicleBox{},.785));
  auto v=VehicleBox{};v.max_lateral_offset_m=v.min_lateral_offset_m;
  EXPECT_FALSE(hazard::intersects_input_path(path(),pose(30,0),box(30,0),v,.785));
}
