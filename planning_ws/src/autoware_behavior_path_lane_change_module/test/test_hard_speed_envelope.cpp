#include "autoware/behavior_path_lane_change_module/utils/hard_speed_envelope.hpp"
#include "autoware/behavior_path_lane_change_module/utils/bounded_speed_grid.hpp"
#include "autoware/behavior_path_lane_change_module/utils/prepare_speed_profile.hpp"
#include "autoware/behavior_path_lane_change_module/utils/maneuver_safety.hpp"
#include <gtest/gtest.h>
using namespace autoware::behavior_path_planner::utils::lane_change;
namespace {
HardEnvelopePath straight(double velocity,double end=100) {
  HardEnvelopePath result;
  for(double x=0;x<=end;x+=10) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=x;p.point.pose.orientation.w=1;
    p.point.longitudinal_velocity_mps=velocity;p.lane_ids={123};result.points.push_back(p);
  }
  return result;
}
geometry_msgs::Point origin() {return geometry_msgs::Point{};}
HardEnvelopePath recorded_shared_vertex() {
  // R29r1 first checked_braking_selected, stamp1790966065.420537,
  // exact target reference vertices88..93, lanelet201447.
  const std::vector<std::pair<double,double>> xy{
    {201.8666692073457,250.79381582792848},{201.86993517109659,250.2938273618929},
    {201.87320025038207,249.79383891448379},{201.87646533269435,249.2938504689373},
    {201.87779696309008,248.79385307664052},{201.87912862258963,248.29385679448023}};
  HardEnvelopePath path;
  for(const auto & pos:xy){autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=pos.first;p.point.pose.position.y=pos.second;
    p.point.longitudinal_velocity_mps=41.66666793823242;path.points.push_back(p);}
  return path;
}
geometry_msgs::Point recorded_offset_query() {
  geometry_msgs::Point q;q.x=203.4324533122101;q.y=249.2979944923279;q.z=28.3642;return q;
}
}
TEST(HardSpeedEnvelope, CoverageErrorNamesReferenceAndExactPhaseWithoutChangingPolicy) {
  const auto path=straight(20),short_ref=straight(20,20);
  for(const std::string name:{"current","target","upstream"}) {
    try {
      (void)make_hard_speed_envelope(path,name=="current"?short_ref:path,
        name=="target"?short_ref:path,name=="upstream"?short_ref:path,
        0,8,origin(),25,4.5,1,true);
      FAIL()<<"Missing coverage must still reject "<<name;
    } catch(const std::logic_error & error) {
      const std::string message=error.what();
      EXPECT_EQ(message.find("speed reference does not cover candidate"),0u);
      EXPECT_NE(message.find("reference="+name),std::string::npos);
      EXPECT_NE(message.find("i=3 point_xyz=30,0,0 phase=lane_change"),std::string::npos);
      EXPECT_NE(message.find("lc_start=1 lc_end=8 begin=0"),std::string::npos);
    }
  }
  // Upstream ending after LC remains optional on the canonical tail, as before.
  const auto caps=make_hard_speed_envelope(path,path,path,short_ref,0,2,origin(),25,4.5,1,true);
  ASSERT_EQ(caps.size(),path.points.size());EXPECT_DOUBLE_EQ(caps.back(),20);
}

TEST(HardSpeedEnvelope, PreparedReferencePreservesNativeVertexTieAndGradedIndex) {
  auto reference=straight(20,100);
  for(size_t i=0;i<reference.points.size();++i) {
    reference.points[i].point.pose.position.z=0.17*i;
    reference.points[i].point.longitudinal_velocity_mps=10+i;
  }
  PreparedSpeedReference prepared(reference);ASSERT_TRUE(prepared.direct());
  for(double x=-5;x<=105;x+=0.25)for(double y:{-4.0,0.0,4.0}) {
    geometry_msgs::Point q;q.x=x;q.y=y;q.z=99;
    EXPECT_EQ(prepared.nearest_segment(q),
      autoware::motion_utils::findNearestSegmentIndex(reference.points,q));
  }
  geometry_msgs::Point tie;tie.x=15;
  EXPECT_EQ(prepared.nearest_segment(tie),1u);
  auto other=reference;EXPECT_FALSE(prepared.matches(other));
}

TEST(HardSpeedEnvelope, PreparedReferenceRoadCapsAndExceptionsMatchLegacy) {
  auto reference=recorded_shared_vertex();
  reference.points[2].point.longitudinal_velocity_mps=0;
  PreparedSpeedReference prepared(reference);ASSERT_TRUE(prepared.direct());
  for(double x=199;x<207;x+=0.25)for(double y=246;y<255;y+=0.25) {
    geometry_msgs::Point q;q.x=x;q.y=y;
    std::string old_error,new_error;double old_cap=0,new_cap=0;
    try{old_cap=hard_road_speed_limit(reference,q);}catch(const std::exception&e){old_error=e.what();}
    try{new_cap=prepared_hard_road_speed_limit(reference,q,&prepared);}catch(const std::exception&e){new_error=e.what();}
    EXPECT_EQ(old_error,new_error);EXPECT_DOUBLE_EQ(old_cap,new_cap);
  }
}

TEST(HardSpeedEnvelope, PreparedMalformedAndOverlapKeepLegacyQueryBehavior) {
  for(int mode=0;mode<7;++mode) {
    auto reference=straight(20,100);
    if(mode==0)reference.points[3]=reference.points[2];
    if(mode==1)reference.points[3].point.pose.position.x=20+0.5e-8;
    if(mode==2)reference.points[3].point.pose.position.x=NAN;
    if(mode==3)reference.points[3].point.pose.position.z=INFINITY;
    if(mode==4)reference.points.resize(1);
    if(mode==5)reference.points.clear();
    if(mode==6)for(auto & p:reference.points)p.point.pose.position.x=0;
    PreparedSpeedReference prepared(reference);EXPECT_FALSE(prepared.direct());
    for(double x:{0.,25.,95.}) {
      geometry_msgs::Point q;q.x=x;
      std::string a,b;double xcap=0,ycap=0;
      try{xcap=hard_road_speed_limit(reference,q);}catch(const std::exception&e){a=e.what();}
      try{ycap=prepared_hard_road_speed_limit(reference,q,&prepared);}catch(const std::exception&e){b=e.what();}
      EXPECT_EQ(a,b);EXPECT_DOUBLE_EQ(xcap,ycap);
    }
  }
}

TEST(HardSpeedEnvelope, PreparedNonAdjacentOverlapDoesNotChangeOriginalIndex) {
  auto reference=straight(20,100);
  reference.points[4].point.pose.position=reference.points[1].point.pose.position;
  PreparedSpeedReference prepared(reference);ASSERT_TRUE(prepared.direct());
  for(double x=-1;x<105;x+=0.1) {
    geometry_msgs::Point q;q.x=x;q.y=1;
    EXPECT_EQ(prepared.nearest_segment(q),
      autoware::motion_utils::findNearestSegmentIndex(reference.points,q));
  }
}
TEST(HardSpeedEnvelope, CertifiedCurrentExitOnlyRepairsMissingOldLaneReference) {
  using Point=boost::geometry::model::d2::point_xy<double>;
  using Polygon=boost::geometry::model::polygon<Point>;
  using Multi=boost::geometry::model::multi_polygon<Polygon>;
  const auto rect=[](double xmin,double xmax,double ymin,double ymax) {
    Polygon p;p.outer()={{xmin,ymin},{xmax,ymin},{xmax,ymax},{xmin,ymax},{xmin,ymin}};
    boost::geometry::correct(p);return p;
  };
  struct Vehicle {double min_longitudinal_offset_m=-1,max_longitudinal_offset_m=3;
    double min_lateral_offset_m=-1,max_lateral_offset_m=1;} vehicle;
  const auto path=straight(20),old=straight(20,20);
  Multi current,target;current.push_back(rect(-5,22,-2,2));target.push_back(rect(22,110,-3,3));
  CurrentExitCertificate cert(path,current,target,vehicle);
  EXPECT_FALSE(cert.permits(2,path.points[2].point.pose));
  EXPECT_TRUE(cert.permits(3,path.points[3].point.pose));
  CurrentExitCertificate bounded(path,current,target,vehicle,3,4);
  EXPECT_TRUE(bounded.permits(3,path.points[3].point.pose));
  EXPECT_TRUE(bounded.permits(4,path.points[4].point.pose));
  EXPECT_FALSE(bounded.permits(2,path.points[2].point.pose));
  EXPECT_FALSE(bounded.permits(5,path.points[5].point.pose));
  CurrentExitCertificate invalid_range(path,current,target,vehicle,5,4);
  EXPECT_FALSE(invalid_range.permits(5,path.points[5].point.pose));
  auto changed=path.points[3].point.pose;changed.position.x+=1e-8;
  EXPECT_FALSE(cert.permits(3,changed));
  const auto call=[&](const HardEnvelopePath & p,const HardEnvelopePath & curr,
      const HardEnvelopePath & tar,const HardEnvelopePath & up,const CurrentExitCertificate * proof) {
    return make_hard_speed_envelope_certified(p,curr,tar,up,0,8,origin(),25,4.5,1,true,nullptr,0,nullptr,proof);
  };
  EXPECT_THROW(call(path,old,path,old,nullptr),std::logic_error);
  const auto caps=call(path,old,path,old,&cert);EXPECT_DOUBLE_EQ(caps[3],20);
  EXPECT_THROW(call(path,old,old,old,&cert),std::logic_error); // Target stays mandatory.
  auto invalid=old;invalid.points.back().point.longitudinal_velocity_mps=std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(call(path,invalid,path,old,&cert),std::logic_error);
  auto current_zero=old;current_zero.points.back().point.longitudinal_velocity_mps=0;
  EXPECT_THROW(call(path,current_zero,path,old,&cert),std::logic_error);
  auto stopped=path;stopped.points[3].point.longitudinal_velocity_mps=0;
  EXPECT_DOUBLE_EQ(call(stopped,old,path,old,&cert)[3],0); // Exact pose, original stop survives.
  auto upstream_stop=old;upstream_stop.points[1].point.longitudinal_velocity_mps=0;
  EXPECT_THROW(call(path,old,path,upstream_stop,&cert),std::logic_error);
  Multi partial;partial.push_back(rect(22,110,-.5,.5));
  CurrentExitCertificate too_narrow(path,current,partial,vehicle);
  EXPECT_FALSE(too_narrow.permits(3,path.points[3].point.pose));
  EXPECT_THROW(call(path,old,path,old,&too_narrow),std::logic_error);
  Multi overlapping;overlapping.push_back(rect(-5,32,-2,2));
  CurrentExitCertificate overlap(path,overlapping,target,vehicle);
  EXPECT_FALSE(overlap.permits(3,path.points[3].point.pose));
  Multi touching;touching.push_back(rect(22,110,-1,1));
  CurrentExitCertificate boundary(path,current,touching,vehicle);
  EXPECT_FALSE(boundary.permits(3,path.points[3].point.pose));
}
HardEnvelopePath internal_vertex_fixture(){HardEnvelopePath r;
for(auto xy:std::vector<std::pair<double,double>>{{203.89846397883957,300.0},{203.61318751215003,293.47219375660643},{203.68502480816096,293.21221060911193},{203.63424102938734,292.71479729702696},{203.5834563334938,292.2173828952946}}){autoware_internal_planning_msgs::PathPointWithLaneId p;p.point.pose.position.x=xy.first;p.point.pose.position.y=xy.second;p.point.longitudinal_velocity_mps=25;r.points.push_back(p);}return r;}
geometry_msgs::Point internal_vertex_query(){geometry_msgs::Point p;p.x=206.81800355084624;p.y=292.98739356754294;p.z=28.382128791815273;return p;}
TEST(InternalVertexRoadSpeed, RecordedNormalConeOldRejectNewConservative){auto r=internal_vertex_fixture();r.points[1].point.longitudinal_velocity_mps=20;r.points[3].point.longitudinal_velocity_mps=15;
EXPECT_DOUBLE_EQ(hard_road_speed_limit(r,internal_vertex_query()),15);}
TEST(InternalVertexRoadSpeed, IncidentZeroRetained){auto r=internal_vertex_fixture();r.points[3].point.longitudinal_velocity_mps=0;EXPECT_DOUBLE_EQ(hard_road_speed_limit(r,internal_vertex_query()),0);}
TEST(InternalVertexRoadSpeed, IncidentNaNRejected){auto r=internal_vertex_fixture();r.points[1].point.longitudinal_velocity_mps=NAN;EXPECT_THROW(hard_road_speed_limit(r,internal_vertex_query()),std::logic_error);}
TEST(InternalVertexRoadSpeed, ReferenceEndpointStillRejected){auto r=internal_vertex_fixture();r.points.erase(r.points.begin(),r.points.begin()+2);EXPECT_THROW(hard_road_speed_limit(r,internal_vertex_query()),std::logic_error);}
TEST(InternalVertexRoadSpeed, DistanceThresholdUnchanged){auto p=internal_vertex_query();p.x+=3;EXPECT_THROW(hard_road_speed_limit(internal_vertex_fixture(),p),std::logic_error);}
TEST(InternalVertexRoadSpeed, CoveredSegmentExactOldResult){auto r=internal_vertex_fixture();auto p=internal_vertex_query();p.x=203.7;p.y=295.;EXPECT_DOUBLE_EQ(hard_road_speed_limit(r,p),25.);}
TEST(InternalVertexRoadSpeed, NonfiniteQueryRejected){auto p=internal_vertex_query();p.x=NAN;EXPECT_THROW(hard_road_speed_limit(internal_vertex_fixture(),p),std::logic_error);}

TEST(HardSpeedEnvelope, RecordedSharedVertexUsesValidAdjacentSegment) {
  const auto ref=recorded_shared_vertex();auto q=recorded_offset_query();
  const auto index=autoware::motion_utils::findNearestSegmentIndex(ref.points,q);
  EXPECT_EQ(index,2u); // Original helper rejects this preceding segment.
  const auto&a=ref.points[2].point.pose.position;const auto&b=ref.points[3].point.pose.position;
  const double dx=b.x-a.x,dy=b.y-a.y;
  EXPECT_GT(((q.x-a.x)*dx+(q.y-a.y)*dy)/(dx*dx+dy*dy),1.01);
  EXPECT_DOUBLE_EQ(hard_road_speed_limit(ref,q),41.66666793823242);
  q.y+=.2;EXPECT_DOUBLE_EQ(hard_road_speed_limit(ref,q),41.66666793823242);
  q.y-=.4;EXPECT_DOUBLE_EQ(hard_road_speed_limit(ref,q),41.66666793823242);
}
TEST(HardSpeedEnvelope, ExplicitPassedEndTailRetainsCanonicalCapsAndStops) {
  auto source=straight(20),ref=straight(30);source.points.back().point.longitudinal_velocity_mps=0;
  auto ego=source.points[3].point.pose.position;ego.x+=2;
  const auto anchored=anchor_maneuver_at_ego(source,3,ego,true);
  const auto end=source.points[3].point.pose.position;
  EXPECT_THROW(make_hard_speed_envelope(anchored.first,ref,ref,ref,anchored.second,3,ego,
    25,4.5,1,true,&source,anchored.second),std::logic_error);
  const auto caps=make_hard_speed_envelope(anchored.first,ref,ref,ref,anchored.second,3,ego,
    25,4.5,1,true,&source,anchored.second,&end);
  EXPECT_EQ(caps.front(),20);EXPECT_EQ(caps.back(),0);
  auto slow=ref;for(auto & p:slow.points)p.point.longitudinal_velocity_mps=10;
  const auto limited=make_hard_speed_envelope(anchored.first,ref,slow,ref,anchored.second,3,ego,
    25,4.5,1,true,&source,anchored.second,&end);
  EXPECT_EQ(limited.front(),10);EXPECT_EQ(limited.back(),0);
}
TEST(HardSpeedEnvelope, PassedTailRejectsUnverifiedEndpointAndNewUnrepresentedStop) {
  auto source=straight(20),ref=straight(30);auto ego=source.points[3].point.pose.position;ego.x+=2;
  const auto anchored=anchor_maneuver_at_ego(source,3,ego,true);
  auto end=source.points[3].point.pose.position;end.x+=.01;
  EXPECT_THROW(make_hard_speed_envelope(anchored.first,ref,ref,ref,anchored.second,3,ego,
    25,4.5,1,true,&source,anchored.second,&end),std::logic_error);
  end=source.points[3].point.pose.position;
  auto upstream=ref;auto stop=upstream.points[5];stop.point.pose.position.x=55;
  stop.point.longitudinal_velocity_mps=0;upstream.points.insert(upstream.points.begin()+6,stop);
  EXPECT_THROW(make_hard_speed_envelope(anchored.first,ref,ref,upstream,anchored.second,3,ego,
    25,4.5,1,true,&source,anchored.second,&end),std::logic_error);
  auto short_target=ref;short_target.points.pop_back();
  EXPECT_THROW(make_hard_speed_envelope(anchored.first,ref,short_target,ref,anchored.second,3,ego,
    25,4.5,1,true,&source,anchored.second,&end),std::logic_error);
}
TEST(HardSpeedEnvelope, CollinearAnchorPreservesExactRecordedCurvature) {
  auto source=straight(20,40);
  const std::vector<std::pair<double,double>> xy{
    {196.2,694.0},{196.68415417042607,690.132657103182},
    {197.15056389370588,686.4934685417594},
    {197.2838796022434,684.9564083284102},{197.693239586234,679.719179436719}};
  for(size_t i=0;i<xy.size();++i){source.points[i].point.pose.position.x=xy[i].first;
    source.points[i].point.pose.position.y=xy[i].second;}
  auto anchored=source;auto anchor=source.points[1];
  anchor.point.pose.position.x=196.75290366501224;
  anchor.point.pose.position.y=689.5962352367118;
  anchored=anchor_maneuver_at_ego(source,1,anchor.point.pose.position,true).first;
  const auto caps=immutable_lateral_speed_caps(source,anchored,2,4.5);
  EXPECT_NEAR(lateral_speed_cap(source.points,2,4.5),16.923008031878044,1e-8);
  EXPECT_NEAR(lateral_speed_cap(anchored.points,3,4.5),16.02085840085917,1e-8);
  for(size_t i=0;i<source.points.size();++i)
    EXPECT_DOUBLE_EQ(caps[i+(i>=2)],lateral_speed_cap(source.points,i,4.5));
  EXPECT_DOUBLE_EQ(caps[2],std::min(caps[1],caps[3]));
}
TEST(HardSpeedEnvelope, ImmutableMappingRejectsShapeMetadataAndEndpointChanges) {
  auto source=straight(20,40),anchored=source;auto anchor=source.points[1];
  anchor.point.pose.position.x=15;anchored=anchor_maneuver_at_ego(source,1,anchor.point.pose.position,true).first;
  EXPECT_NO_THROW(immutable_lateral_speed_caps(source,anchored,2,4.5));
  auto bad=anchored;bad.points[2].point.pose.position.y=.001;
  EXPECT_THROW(immutable_lateral_speed_caps(source,bad,2,4.5),std::logic_error);
  bad=anchored;bad.points[3].lane_ids={999};
  EXPECT_THROW(immutable_lateral_speed_caps(source,bad,2,4.5),std::logic_error);
  bad=anchored;bad.points.back().point.pose.position.x+=.1;
  EXPECT_THROW(immutable_lateral_speed_caps(source,bad,2,4.5),std::logic_error);
  bad=anchored;bad.points[1].point.pose.orientation.x+=.1;
  EXPECT_THROW(immutable_lateral_speed_caps(source,bad,2,4.5),std::logic_error);
  bad=anchored;bad.points[3].point.pose.orientation.y+=.1;
  EXPECT_THROW(immutable_lateral_speed_caps(source,bad,2,4.5),std::logic_error);
  bad=anchored;bad.points[2].lane_ids={999};
  EXPECT_THROW(immutable_lateral_speed_caps(source,bad,2,4.5),std::logic_error);
  EXPECT_THROW(immutable_lateral_speed_caps(source,anchored,0,4.5),std::logic_error);
  EXPECT_NO_THROW(immutable_lateral_speed_caps(source,source,1,4.5));
}
TEST(HardSpeedEnvelope, TransportKeepsPhaseRoadZeroAndCanonicalTailContracts) {
  auto source=straight(20,60),ref=straight(30,60);
  source.points[1].point.pose.position.y=1;
  source.points[3].point.longitudinal_velocity_mps=0;
  source.points[6].point.longitudinal_velocity_mps=4;
  auto anchored=source;auto anchor=source.points[0];
  anchor.point.pose.position.x=5;anchor.point.pose.position.y=.5;
  anchored=anchor_maneuver_at_ego(source,0,anchor.point.pose.position,true).first;
  const auto caps=make_hard_speed_envelope(anchored,ref,ref,ref,0,5,origin(),25,4.5,3,true,&source,1);
  EXPECT_DOUBLE_EQ(caps[1],25); // PREPARE has no LC-only curvature restriction.
  EXPECT_DOUBLE_EQ(caps[4],0);  // Existing zero unchanged.
  EXPECT_DOUBLE_EQ(caps.back(),4); // Canonical tail unchanged.
  EXPECT_LE(caps[3],lateral_speed_cap(source.points,2,4.5));
  auto slow=ref;for(auto & p:slow.points)p.point.longitudinal_velocity_mps=3;
  const auto limited=make_hard_speed_envelope(anchored,ref,ref,slow,0,5,origin(),25,4.5,3,true,&source,1);
  EXPECT_DOUBLE_EQ(limited[2],3);
  EXPECT_DOUBLE_EQ(limited[4],0);
}
TEST(HardSpeedEnvelope, SharedVertexRetainsConservativeIncidentSpeed) {
  auto ref=recorded_shared_vertex();auto q=recorded_offset_query();
  ref.points[2].point.longitudinal_velocity_mps=10;
  ref.points[3].point.longitudinal_velocity_mps=20;
  ref.points[4].point.longitudinal_velocity_mps=30;
  EXPECT_DOUBLE_EQ(hard_road_speed_limit(ref,q),10);
  q.y-=.2;EXPECT_DOUBLE_EQ(hard_road_speed_limit(ref,q),20);
}
TEST(HardSpeedEnvelope, AdjacentSearchDoesNotRelaxCoverageOrInvalidData) {
  auto ref=recorded_shared_vertex();auto q=recorded_offset_query();
  auto outside=q;outside.x+=5;
  EXPECT_THROW(hard_road_speed_limit(ref,outside),std::logic_error);
  outside=ref.points.back().point.pose.position;outside.y-=1;
  EXPECT_THROW(hard_road_speed_limit(ref,outside),std::logic_error);
  outside=ref.points.front().point.pose.position;outside.y+=1;
  EXPECT_THROW(hard_road_speed_limit(ref,outside),std::logic_error);
  outside=q;outside.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(hard_road_speed_limit(ref,outside),std::logic_error);
  ref.points[3].point.longitudinal_velocity_mps=std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(hard_road_speed_limit(ref,q),std::logic_error);
}
TEST(HardSpeedEnvelope, OldPlanIsNotHardLimitButAllZerosAndTailRemain) {
  auto path=straight(5),ref=straight(20);path.points[2].point.longitudinal_velocity_mps=0;
  path.points[9].point.longitudinal_velocity_mps=0;
  const auto before=path;
  const auto caps=make_hard_speed_envelope(path,ref,ref,ref,0,8,origin(),25,4.5);
  EXPECT_EQ(caps[0],20);EXPECT_EQ(caps[1],20);EXPECT_EQ(caps[2],0);
  EXPECT_EQ(caps[8],20);EXPECT_EQ(caps[9],0);EXPECT_EQ(caps[10],5);
  EXPECT_EQ(path,before);
}
TEST(HardSpeedEnvelope, MinimumOfAllFreshReferencesAndMaximumVelocity) {
  auto path=straight(5),current=straight(18),target=straight(20),upstream=straight(17);
  auto caps=make_hard_speed_envelope(path,current,target,upstream,0,8,origin(),16,4.5);
  EXPECT_EQ(caps[1],16);
  caps=make_hard_speed_envelope(path,current,target,upstream,0,8,origin(),25,4.5);
  EXPECT_EQ(caps[1],17);EXPECT_EQ(caps.back(),5);
}
TEST(HardSpeedEnvelope, PrepareDoesNotRequireFutureTargetLaneCoverage) {
  auto path=straight(5),current=straight(18),target=straight(20),upstream=straight(17);
  target.points.erase(target.points.begin(),target.points.begin()+3); // target begins at physical LC start30m
  path.points[1].point.longitudinal_velocity_mps=0;
  path.points[5].point.longitudinal_velocity_mps=0;
  path.points[9].point.longitudinal_velocity_mps=0;
  EXPECT_THROW(make_hard_speed_envelope(path,current,target,upstream,0,8,origin(),25,4.5),std::logic_error);
  const auto caps=make_hard_speed_envelope(path,current,target,upstream,0,8,origin(),25,4.5,3);
  EXPECT_EQ(caps[0],17);EXPECT_EQ(caps[1],0);EXPECT_EQ(caps[2],17);
  EXPECT_EQ(caps[3],17);EXPECT_EQ(caps[5],0);EXPECT_EQ(caps[9],0);EXPECT_EQ(caps[10],5);
}
TEST(HardSpeedEnvelope, ActualLaneChangeStillRequiresTargetAndStartMayPrecedeEgo) {
  auto path=straight(5),ref=straight(20),target=ref;
  target.points.erase(target.points.begin(),target.points.begin()+4);
  EXPECT_THROW(make_hard_speed_envelope(path,ref,target,ref,0,8,origin(),25,4.5,3),std::logic_error);
  EXPECT_THROW(make_hard_speed_envelope(path,ref,ref,ref,0,8,origin(),25,4.5,9),std::logic_error);
  auto ego=origin();ego.x=50;
  const auto caps=make_hard_speed_envelope(path,ref,ref,ref,5,8,ego,25,4.5,3);
  EXPECT_EQ(caps.front(),20);EXPECT_EQ(caps.back(),5);
}
TEST(HardSpeedEnvelope, UpstreamStopBetweenVerticesAndPositiveResumptionRejected) {
  auto path=straight(5),ref=straight(20),upstream=ref;
  auto stop=upstream.points[2];stop.point.pose.position.x=25;stop.point.longitudinal_velocity_mps=0;
  upstream.points.insert(upstream.points.begin()+3,stop);
  EXPECT_THROW(make_hard_speed_envelope(path,ref,ref,upstream,0,3,origin(),25,4.5),std::logic_error);
  // Exactly LC end +10 is protected, not only candidate-aligned stops.
  upstream=ref;upstream.points[4].point.longitudinal_velocity_mps=0;
  EXPECT_THROW(make_hard_speed_envelope(path,ref,ref,upstream,0,3,origin(),25,4.5),std::logic_error);
}
TEST(HardSpeedEnvelope, MissingCoverageNeverClampsReferenceEndpoint) {
  auto path=straight(5),ref=straight(20),short_ref=straight(20,30);
  EXPECT_THROW(make_hard_speed_envelope(path,HardEnvelopePath{},ref,ref,0,3,origin(),25,4.5),std::logic_error);
  EXPECT_THROW(make_hard_speed_envelope(path,ref,short_ref,ref,0,3,origin(),25,4.5),std::logic_error);
  EXPECT_THROW(make_hard_speed_envelope(path,short_ref,ref,ref,0,8,origin(),25,4.5),std::logic_error);
  // Past LC end, B2 allows the old upstream to end; canonical tail remains.
  const auto caps=make_hard_speed_envelope(path,ref,ref,short_ref,0,3,origin(),25,4.5);
  EXPECT_EQ(caps.back(),5);
}
TEST(HardSpeedEnvelope, InvalidIndexNaNAndActualOverspeedFailClosed) {
  auto path=straight(5),ref=straight(20);
  EXPECT_THROW(make_hard_speed_envelope(path,ref,ref,ref,3,2,origin(),25,4.5),std::logic_error);
  EXPECT_THROW(make_hard_speed_envelope(path,ref,ref,ref,0,30,origin(),25,4.5),std::logic_error);
  auto invalid=ref;invalid.points[2].point.longitudinal_velocity_mps=std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(make_hard_speed_envelope(path,ref,ref,invalid,0,8,origin(),25,4.5),std::logic_error);
  auto caps=make_hard_speed_envelope(path,ref,ref,ref,0,8,origin(),25,4.5);
  std::vector<double> arc;for(size_t i=0;i<caps.size();++i)arc.push_back(i*10.);
  EXPECT_THROW(bounded_speed_grid::make(arc,caps,20.0001,0,0,3,-4,-10,100,[](double){return 3.;}),std::logic_error);
  const auto proposal=bounded_speed_grid::make(arc,caps,10,0,0,3,-4,-10,100,[](double){return 3.;});
  EXPECT_DOUBLE_EQ(proposal.speeds.front(),10); // Actual state, never clamped to old plan5.
  EXPECT_LE(proposal.speeds.back(),5);
}
TEST(HardSpeedEnvelope, CurvatureLimitIdenticalToGenerationAndRefresh) {
  HardEnvelopePath path;
  const double radius=1/.017551297484834125;
  for(double angle:{-.1,-.05,0.,.05,.1}) {
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=radius*std::sin(angle);
    p.point.pose.position.y=radius*(1-std::cos(angle));p.point.pose.orientation.w=1;
    p.point.longitudinal_velocity_mps=30;path.points.push_back(p);
  }
  const auto caps=make_hard_speed_envelope(path,path,path,path,0,3,path.points[0].point.pose.position,40,4.5);
  EXPECT_NEAR(caps[2]*3.6,57.64400458,1e-5);
  EXPECT_DOUBLE_EQ(caps[2],lateral_speed_cap(path.points,2,4.5));
}
TEST(HardSpeedEnvelope, PhaseCurvatureDomainPreservesNormalPrepareAndCanonicalTail) {
  HardEnvelopePath path;
  const double radius=1/.017551297484834125;
  for(int i=0;i<9;++i) {
    const double angle=(i-4)*.2;
    autoware_internal_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x=radius*std::sin(angle);
    p.point.pose.position.y=radius*(1-std::cos(angle));p.point.pose.orientation.w=1;
    p.point.longitudinal_velocity_mps=30;path.points.push_back(p);
  }
  const auto ego=path.points.front().point.pose.position;
  const auto legacy=make_hard_speed_envelope(path,path,path,path,0,5,ego,40,4.5,3);
  const auto caps=make_hard_speed_envelope(path,path,path,path,0,5,ego,40,4.5,3,true);
  EXPECT_LT(legacy[1],30);EXPECT_LT(legacy[7],30);
  EXPECT_EQ(caps[1],30);EXPECT_EQ(caps[7],30);
  EXPECT_DOUBLE_EQ(caps[3],lateral_speed_cap(path.points,3,4.5));
  EXPECT_DOUBLE_EQ(caps[5],lateral_speed_cap(path.points,5,4.5));
  std::vector<double> arc{0};
  for(size_t i=1;i<path.points.size();++i)
    arc.push_back(arc.back()+autoware_utils::calc_distance2d(path.points[i-1],path.points[i]));
  const auto speeds=make_maneuver_speed_profile(arc,caps,18,arc[3],3.25,3.25,4);
  EXPECT_DOUBLE_EQ(speeds.front(),18);
  EXPECT_LE(speeds[3],caps[3]);
  EXPECT_LT(speeds[2],std::sqrt(18*18+2*3.25*arc[2])); // LC bound reaches backwards into PREPARE.
  auto with_zero=path;
  for(size_t i:{1u,4u,7u})with_zero.points[i].point.longitudinal_velocity_mps=0;
  const auto zeros=make_hard_speed_envelope(with_zero,path,path,path,0,5,ego,40,4.5,3,true);
  EXPECT_EQ(zeros[1],0);EXPECT_EQ(zeros[4],0);EXPECT_EQ(zeros[7],0);
}
