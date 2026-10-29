#pragma once
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <boost/geometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::motion_utils
{
// Conservative swept footprints over each provided prediction interval. This
// deliberately checks all prediction branches, not just the most likely one.
// Check prediction age and hold each branch at its terminal pose afterwards.
// Existing velocity planners still decide whether/when movement is permissible.
template<class Objects, class Vehicle>
bool recoveryCollisionFree(const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
                          const geometry_msgs::Pose & measured, double speed,
                          const Objects & objects, const Vehicle & vehicle, double prediction_age = 0.0)
{
  namespace bg = boost::geometry;
  using P = bg::model::d2::point_xy<double>;
  using Poly = bg::model::polygon<P>;
  using Cloud = bg::model::multi_point<P>;
  if (points.size()<2 || !std::isfinite(speed) || !std::isfinite(prediction_age) || prediction_age<0) return false;
  const auto validPose=[](const geometry_msgs::Pose & pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
      std::isfinite(pose.position.z) && std::isfinite(tf2::getYaw(pose.orientation));
  };
  if (!validPose(measured) || !std::isfinite(vehicle.vehicle_width_m) ||
      vehicle.vehicle_width_m<=0 || vehicle.wheel_base_m+vehicle.front_overhang_m<=0) return false;
  for (const auto & point:points)
    if (!validPose(point.pose) || !std::isfinite(point.longitudinal_velocity_mps)) return false;
  auto rectangle=[](const geometry_msgs::Pose & pose,double front,double rear,double half) {
    Poly polygon;
    const double yaw=tf2::getYaw(pose.orientation),c=std::cos(yaw),s=std::sin(yaw);
    for(const auto & p:std::vector<P>{{front,half},{front,-half},{-rear,-half},{-rear,half}})
      polygon.outer().emplace_back(pose.position.x+c*p.x()-s*p.y(),pose.position.y+s*p.x()+c*p.y());
    bg::correct(polygon);return polygon;
  };
  auto footprint=[&](const geometry_msgs::Pose & pose) {
    return rectangle(pose,vehicle.wheel_base_m+vehicle.front_overhang_m,
                     vehicle.rear_overhang_m,vehicle.vehicle_width_m/2.0);
  };
  auto swept=[](const Poly & a,const Poly & b) {
    Cloud cloud;for(const auto & p:a.outer())cloud.push_back(p);
    for(const auto & p:b.outer())cloud.push_back(p);
    Poly hull;bg::convex_hull(cloud,hull);return hull;
  };
  auto interpolate=[](const geometry_msgs::Pose & a,const geometry_msgs::Pose & b,double ratio) {
    geometry_msgs::Pose p=a;
    p.position.x+=(b.position.x-a.position.x)*ratio;
    p.position.y+=(b.position.y-a.position.y)*ratio;
    p.position.z+=(b.position.z-a.position.z)*ratio;
    const double ya=tf2::getYaw(a.orientation),yb=tf2::getYaw(b.orientation);
    const double y=ya+std::atan2(std::sin(yb-ya),std::cos(yb-ya))*ratio;
    p.orientation.x=p.orientation.y=0;p.orientation.z=std::sin(y/2);p.orientation.w=std::cos(y/2);
    return p;
  };
  size_t ego=0;double best=1e30;
  for(size_t i=0;i<points.size();++i) {
    const double d=std::hypot(points[i].pose.position.x-measured.position.x,
                              points[i].pose.position.y-measured.position.y);
    if(d<best){best=d;ego=i;}
  }
  if(best>1.0)return false;
  std::vector<double> times{0};
  std::vector<geometry_msgs::Pose> poses{measured};
  double velocity=std::abs(speed);
  for(size_t i=ego+1;i<points.size();++i) {
    const double ds=std::hypot(points[i].pose.position.x-poses.back().position.x,
                                points[i].pose.position.y-poses.back().position.y);
    if(!std::isfinite(ds)||!std::isfinite(points[i].longitudinal_velocity_mps))return false;
    if(ds<1e-5)continue;
    const double next=std::abs(double(points[i].longitudinal_velocity_mps));
    if(velocity+next<1e-3)break;
    times.push_back(times.back()+2*ds/(velocity+next));poses.push_back(points[i].pose);velocity=next;
    if(times.back()>8.5)break;
  }
  auto egoAt=[&](double t) {
    const auto it=std::lower_bound(times.begin(),times.end(),t);
    if(it==times.end())return poses.back();
    const size_t j=it-times.begin();
    return j==0?poses.front():interpolate(poses[j-1],poses[j],(t-times[j-1])/(times[j]-times[j-1]));
  };
  double ego_extent=0;
  for(const auto & pose:poses)ego_extent=std::max(ego_extent,
    std::hypot(pose.position.x-measured.position.x,pose.position.y-measured.position.y));
  ego_extent+=vehicle.wheel_base_m+vehicle.front_overhang_m+vehicle.rear_overhang_m+vehicle.vehicle_width_m;
  for(const auto & object:objects.objects) {
    const auto & initial=object.kinematics.initial_pose_with_covariance.pose;
    if(!validPose(initial))return false;
    const double length=object.shape.dimensions.x;
    // Cylinder dimensions.x is its diameter; dimensions.y may be unset.
    const double width=object.shape.type==1 ? length : object.shape.dimensions.y;
    if (!std::isfinite(length)||!std::isfinite(width)||!std::isfinite(object.shape.dimensions.z)) return false;
    if(object.shape.type!=2 && (length<=0||width<=0))return false;
    const auto & initial_twist=object.kinematics.initial_twist_with_covariance.twist;
    const double object_speed=std::hypot(initial_twist.linear.x,initial_twist.linear.y);
    if(!std::isfinite(object_speed))return false;
    double object_extent=object_speed*(8.0+prediction_age)+std::hypot(length,width);
    for(const auto & path:object.kinematics.predicted_paths)
      for(const auto & pose:path.path) {
        if(!validPose(pose))return false;
        object_extent=std::max(object_extent,std::hypot(pose.position.x-initial.position.x,
          pose.position.y-initial.position.y)+std::hypot(length,width));
      }
    for(const auto & point:object.shape.footprint.points)
      object_extent=std::max(object_extent,object_speed*(8.0+prediction_age)+std::hypot(point.x,point.y));
    if(std::hypot(initial.position.x-measured.position.x,initial.position.y-measured.position.y)>
       ego_extent+object_extent)continue;
    if(std::abs(initial.position.z-measured.position.z)>vehicle.vehicle_height_m+object.shape.dimensions.z)continue;
    auto polygonAt=[&](const geometry_msgs::Pose & pose) {
      if(object.shape.type==2 && object.shape.footprint.points.size()>=3) {
        Poly p;const double yaw=tf2::getYaw(pose.orientation),c=std::cos(yaw),s=std::sin(yaw);
        for(const auto & q:object.shape.footprint.points)
          p.outer().emplace_back(pose.position.x+c*q.x-s*q.y,pose.position.y+s*q.x+c*q.y);
        bg::correct(p);return p;
      }
      return rectangle(pose,length/2,length/2,width/2);
    };
    const auto first=polygonAt(initial);
    if(!bg::is_valid(first))return false;
    if(bg::intersects(footprint(measured),first))return false;
    const auto & paths=object.kinematics.predicted_paths;
    if(paths.empty()) {
      const auto & twist=object.kinematics.initial_twist_with_covariance.twist;
      if(std::hypot(twist.linear.x,twist.linear.y)>0.1) return false;
      for(size_t i=1;i<poses.size();++i)
        if(bg::intersects(swept(footprint(poses[i-1]),footprint(poses[i])),first))return false;
      continue;
    }
    for(const auto & path:paths) {
      if(path.path.empty())return false;
      for(const auto & pose:path.path)
        if(!std::isfinite(pose.position.x)||!std::isfinite(pose.position.y)||
           !std::isfinite(tf2::getYaw(pose.orientation)))return false;
      const double dt=path.time_step.sec+path.time_step.nanosec*1e-9;
      if(!std::isfinite(dt)||dt<=0)return false;
      auto objectAt=[&](double t) {
        const double index=std::max(0.0,(t+prediction_age)/dt);const size_t i=std::min(size_t(index),path.path.size()-1);
        return i+1==path.path.size()?path.path[i]:interpolate(path.path[i],path.path[i+1],index-i);
      };
      const double horizon=8.0;
      if (!bg::is_valid(polygonAt(objectAt(0)))) return false;
      for(double t=0;t<horizon;t+=0.05) {
        const double end=std::min(horizon,t+0.05);
        if(bg::intersects(swept(footprint(egoAt(t)),footprint(egoAt(end))),
                          swept(polygonAt(objectAt(t)),polygonAt(objectAt(end)))))return false;
      }
    }
  }
  return true;
}
}
