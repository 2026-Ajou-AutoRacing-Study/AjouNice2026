#include "collision_front_check.hpp"
#include "obstacle_cruise_module.hpp"
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <gtest/gtest.h>
#include <vector>

using autoware::motion_velocity_planner::collision_is_ahead_of_object;
using Points = std::vector<autoware_planning_msgs::TrajectoryPoint>;
static Points line(double begin, double step, size_t count)
{
  Points p(count);
  for (size_t i=0; i<count; ++i) p[i].pose.position.x=begin+step*i;
  return p;
}
TEST(CollisionFrontCheck, DecimatedIndexMustNotIndexDenseTrajectoryWithBehindPrefix)
{
  const auto full=line(-30.,.2,700), decimated=line(0.,2.,60);
  geometry_msgs::Point object; object.x=36.;
  // Collision at +78m: object index18 and collision index39 in decimated space.
  EXPECT_TRUE(collision_is_ahead_of_object(decimated,object,39,2.5));
  EXPECT_FALSE(collision_is_ahead_of_object(full,object,39,2.5)); // old mixed-space bug
  EXPECT_TRUE(collision_is_ahead_of_object(full,object,540,2.5));
}
TEST(CollisionFrontCheck, SamplingOriginAndResolutionPreserveNativeAheadBehindRule)
{
  geometry_msgs::Point object; object.x=36.;
  for(double step : {.2, 1., 2.}) {
    const auto p=line(-20.,step,static_cast<size_t>(140./step)+1);
    EXPECT_TRUE(collision_is_ahead_of_object(p,object,static_cast<size_t>(98./step),2.5));
    EXPECT_FALSE(collision_is_ahead_of_object(p,object,static_cast<size_t>(46./step),2.5));
  }
}
TEST(CollisionFrontCheck, ShapeToleranceAndBoundsRemainConservative)
{
  const auto p=line(0.,1.,50); geometry_msgs::Point object; object.x=20.;
  EXPECT_TRUE(collision_is_ahead_of_object(p,object,18,2.5));
  EXPECT_FALSE(collision_is_ahead_of_object(p,object,17,2.5));
  EXPECT_FALSE(collision_is_ahead_of_object(p,object,50,2.5));
  EXPECT_FALSE(collision_is_ahead_of_object(Points{},object,0,2.5));
}

namespace autoware::motion_velocity_planner {
class CruiseFutureAdmissionTestAccess {
public:
  static bool evaluate(bool enabled, bool approaches, double speed=10., bool designated=true,
                       double overlap_threshold=.2, bool forward=true) {
    ros::Time::init();
    ObstacleCruiseModule module;
    auto & p=module.obstacle_filtering_param_;
    p.inside_object_types={1};p.outside_object_types={1};p.max_lat_margin=1.;
    p.crossing_obstacle_traj_angle_threshold=.785;
    p.max_lateral_time_margin=3.;p.outside_obstacle_velocity_threshold=1.5;
    p.ego_obstacle_overlap_time_threshold=overlap_threshold;
    p.max_prediction_time_for_collision_check=10.;p.num_of_predicted_paths_for_outside_cruise_obstacle=1;
    module.debug_data_ptr_=std::make_shared<ObstacleCruiseDebugData>();
    auto points=line(0.,2.,76);
    for(auto & point:points)point.pose.orientation.w=1.;
    nav_msgs::Odometry ego;ego.pose.pose.orientation.w=1.;ego.twist.twist.linear.x=15.;
    auto vi=autoware::vehicle_info_utils::createVehicleInfo(.3705,.235,3.,1.651,.845,.79,.1195,.1195,1.6,.7);
    auto polygons=polygon_utils::createOneStepPolygons(points,vi,ego.pose.pose,0.,false,.5);
    autoware_perception_msgs::PredictedObject raw;
    raw.classification.resize(1);raw.classification[0].label=designated?1:7;
    raw.shape.type=0;raw.shape.dimensions.x=4.8;raw.shape.dimensions.y=1.9;raw.shape.dimensions.z=1.5;
    auto & pose=raw.kinematics.initial_pose_with_covariance.pose;
    pose.position.x=30.;pose.position.y=4.;pose.orientation.w=1.;
    raw.kinematics.initial_twist_with_covariance.twist.linear.x=speed;
    raw.kinematics.predicted_paths.resize(1);auto & pred=raw.kinematics.predicted_paths[0];
    pred.confidence=1.;pred.time_step.sec=0;pred.time_step.nanosec=100000000;
    for(int i=0;i<=30;++i){auto q=pose;q.position.x+=speed*.1*i;
      if(approaches)q.position.y=std::max(0.,4.-.25*i);pred.path.push_back(q);}
    auto object=std::make_shared<PlannerData::Object>(raw);
    TrajectoryPolygonCollisionCheck check;check.decimate_trajectory_step_length=2.;
    return module.create_cruise_obstacle(ego,points,points,polygons,object,ros::Time::now(),
      2.,forward,vi,check,enabled,true).has_value();
  }
};
}
using autoware::motion_velocity_planner::CruiseFutureAdmissionTestAccess;
TEST(CruiseFutureAdmission, ExactFutureCollisionRequiredAndDisabledNativeGatePreserved)
{
  EXPECT_FALSE(CruiseFutureAdmissionTestAccess::evaluate(false,true));
  EXPECT_TRUE(CruiseFutureAdmissionTestAccess::evaluate(true,true));
  EXPECT_FALSE(CruiseFutureAdmissionTestAccess::evaluate(true,false));
}
TEST(CruiseFutureAdmission, NativeTypeSpeedOverlapAndDrivingDirectionStillRequired)
{
  EXPECT_FALSE(CruiseFutureAdmissionTestAccess::evaluate(true,true,10.,false));
  EXPECT_FALSE(CruiseFutureAdmissionTestAccess::evaluate(true,true,1.));
  EXPECT_FALSE(CruiseFutureAdmissionTestAccess::evaluate(true,true,10.,true,5.));
  EXPECT_FALSE(CruiseFutureAdmissionTestAccess::evaluate(true,true,10.,true,.2,false));
}
