#include <autoware/behavior_path_planner_common/utils/path_safety_checker/prepared_path_projection.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp>
#include <gtest/gtest.h>
namespace native=autoware::behavior_path_planner::utils::path_safety_checker;
using Path=autoware_internal_planning_msgs::PathWithLaneId;
Path sample(){Path p;for(size_t i=0;i<40;++i){decltype(p.points)::value_type q;q.point.pose.position.x=.7*i;q.point.pose.position.y=.005*i*i;q.point.pose.position.z=.03*i;q.point.pose.orientation.w=1.;q.lane_ids={static_cast<long>(100+i/10)};q.point.longitudinal_velocity_mps=i==20?0:15;p.points.push_back(q);}return p;}
void same_pose(const std::optional<geometry_msgs::Pose>&a,const std::optional<geometry_msgs::Pose>&b){ASSERT_EQ(bool(a),bool(b));if(a)EXPECT_EQ(*a,*b);}
void same_polygon(const autoware_utils_geometry::Polygon2d&a,const autoware_utils_geometry::Polygon2d&b){ASSERT_EQ(a.outer().size(),b.outer().size());for(size_t i=0;i<a.outer().size();++i){EXPECT_DOUBLE_EQ(a.outer()[i].x(),b.outer()[i].x());EXPECT_DOUBLE_EQ(a.outer()[i].y(),b.outer()[i].y());}}
TEST(PreparedRSSProjection, CurvedGradedOffsetsAndEndpointNulloptRemainExact){
 auto path=sample();native::PreparedPathProjection projection(path);
 for(const auto&q:path.points)for(double x:{-2.,0.,.35,2.})for(double d:{-100.,-5.,0.,.1,4.,100.}){
  auto p=q.point.pose.position;p.x+=x;
  EXPECT_EQ(projection.nearest(p),autoware::motion_utils::findNearestSegmentIndex(path.points,p));
  same_pose(projection.offset_pose(p,d),autoware::motion_utils::calcLongitudinalOffsetPose(path.points,p,d));
 }
}
TEST(PreparedRSSProjection, NativePolygonVerticesOrderAndZeroStopSemanticsRemainExact){
 auto path=sample();native::PreparedPathProjection projection(path);
 const auto vehicle=autoware::vehicle_info_utils::createVehicleInfo(.37,.23,3.,1.65,.8,1.,.1,.1,1.6,.6);
 for(size_t i:{0u,1u,19u,38u,39u})for(double d:{0.,2.,15.,100.})for(bool stopped:{false,true})for(double lateral:{0.,1.}) {
   auto pose=path.points[i].point.pose;pose.position.y+=.2;
   same_polygon(native::create_extended_ego_polygon_along_path(path,pose,vehicle,d,lateral,stopped),native::create_extended_ego_polygon_along_path(path,pose,vehicle,d,lateral,stopped,&projection));
 }
}
TEST(PreparedRSSProjection, AdjacentDuplicatesAndTinyOverlapUseOriginalIndex){
 for(int mode:{0,1,2}){auto path=sample();path.points[10]=path.points[9];if(mode==1)path.points[10].point.pose.position.x+=.5e-8;if(mode==2)path.points[20]=path.points[3];
 native::PreparedPathProjection projection(path);for(size_t i:{0u,9u,10u,11u,20u,39u})for(double d:{-10.,0.,1.,30.}){
  const auto p=path.points[i].point.pose.position;
  EXPECT_EQ(projection.nearest(p),autoware::motion_utils::findNearestSegmentIndex(path.points,p));
  same_pose(projection.offset_pose(p,d),autoware::motion_utils::calcLongitudinalOffsetPose(path.points,p,d));
 }}
}
TEST(PreparedRSSProjection, EmptySingletonAndUnusedInvalidPoseKeepLegacyResult){
 for(int mode:{0,1,2,3}){auto path=sample();if(mode==0)path.points.clear();if(mode==1)path.points.resize(1);if(mode==2)path.points.back().point.pose.position.z=INFINITY;if(mode==3)path.points.back().point.pose.position.x=NAN;
 native::PreparedPathProjection projection(path);geometry_msgs::Point p;p.x=1;
 for(double d:{0.,1.,3.}){std::optional<geometry_msgs::Pose>a,b;std::string e1,e2;
  try{a=autoware::motion_utils::calcLongitudinalOffsetPose(path.points,p,d);}catch(const std::exception&e){e1=e.what();}
  try{b=projection.offset_pose(p,d);}catch(const std::exception&e){e2=e.what();}
  EXPECT_EQ(e1,e2);same_pose(a,b);
 }}
}

TEST(PreparedRSSProjection, WorkspaceIdentityMismatchFallsBackWithoutChangingGeometry){
 auto path=sample(),other=sample();for(auto&p:other.points)p.point.pose.position.x+=100.;
 native::PreparedPathProjection projection(other);EXPECT_FALSE(projection.matches(path));
 const auto vehicle=autoware::vehicle_info_utils::createVehicleInfo(.37,.23,3.,1.65,.8,1.,.1,.1,1.6,.6);
 const auto pose=path.points[10].point.pose;
 same_polygon(native::create_extended_ego_polygon_along_path(path,pose,vehicle,10.,1.,false),
   native::create_extended_ego_polygon_along_path(path,pose,vehicle,10.,1.,false,&projection));
}

TEST(PreparedRSSProjection, NativeRssSafePhysicalAndExtendedFailureDiagnosticsRemainExact){
 auto path=sample();for(auto&p:path.points)p.point.pose.position.y=0;
 native::PreparedPathProjection projection(path);
 const auto vehicle=autoware::vehicle_info_utils::createVehicleInfo(.37,.23,3.,1.65,.8,1.,.1,.1,1.6,.6);
 geometry_msgs::Pose ego;ego.orientation.w=1;
 const std::vector<native::PoseWithVelocityStamped> prediction{{0.,ego,10.},{1.,ego,10.}};
 native::RSSparams policy;policy.extended_polygon_policy="along_path";
 for(double position:{0.,8.,100.}) {
   native::ExtendedPredictedObject object;object.uuid.uuid[0]=1;object.initial_pose=ego;
   object.initial_pose.position.x=position;
   object.initial_polygon.outer()={{position-1.,-1.},{position-1.,1.},
     {position+1.,1.},{position+1.,-1.},{position-1.,-1.}};
   native::PredictedPathWithPolygon future;future.confidence=1.;
   future.path.emplace_back(0.,object.initial_pose,0.,object.initial_polygon);
   future.path.emplace_back(1.,object.initial_pose,0.,object.initial_polygon);object.predicted_paths={future};
   native::CollisionCheckDebugMap a,b;
   const bool old=native::check_safety_with_rss(path,prediction,vehicle,{object},a,policy,true,1.,1.57);
   const bool prepared=native::check_safety_with_rss(path,prediction,vehicle,{object},b,policy,true,1.,1.57,&projection);
   EXPECT_EQ(old,prepared);EXPECT_EQ(old,position==100.);
   ASSERT_EQ(a.size(),1u);ASSERT_EQ(b.size(),1u);
   const auto & x=a.begin()->second;const auto & y=b.begin()->second;
   EXPECT_EQ(x.unsafe_reason,y.unsafe_reason);EXPECT_EQ(x.is_safe,y.is_safe);
   EXPECT_EQ(x.expected_ego_pose,y.expected_ego_pose);EXPECT_EQ(x.expected_object_pose,y.expected_object_pose);
   same_polygon(x.extended_ego_polygon,y.extended_ego_polygon);
   same_polygon(x.extended_object_polygon,y.extended_object_polygon);
   if(position==0.)EXPECT_EQ(x.unsafe_reason,"overlap_polygon");
   if(position==8.)EXPECT_EQ(x.unsafe_reason,"overlap_extended_polygon");
 }
}
