#pragma once
#include "autoware/behavior_path_lane_change_module/utils/maneuver_safety.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_core.hpp"
#include <nav_msgs/Odometry.h>

namespace autoware::behavior_path_planner::utils::lane_change
{
struct PreparedSafetyWorld
{
  using Polygon=decltype(get_ego_footprint(geometry_msgs::Pose{},
    autoware::vehicle_info_utils::VehicleInfo{}));
  using Corridor=boost::geometry::model::multi_polygon<Polygon>;
  const void * planner=nullptr,*common=nullptr,*raw=nullptr,*odom=nullptr,*acceleration=nullptr;
  const void * route=nullptr,*map=nullptr;
  ros::Time raw_stamp,odom_stamp;
  unique_identifier_msgs::UUID route_uuid;
  std_msgs::Header route_header;
  std::string raw_frame,odom_frame;
  std::vector<lanelet::Id> current_ids,target_ids;
  double resolution=0,raw_age=0,horizon=0;
  path_safety_checker::ExtendedPredictedObjects objects;
  Corridor corridor;
};

inline std::pair<path_safety_checker::ExtendedPredictedObjects,double> prepare_safety_observations(
  const autoware_perception_msgs::PredictedObjects & raw,
  const nav_msgs::Odometry & odom,const double resolution)
{
  namespace lc=autoware::behavior_path_planner::utils::lane_change;
  const double raw_age=(odom.header.stamp-raw.header.stamp).toSec();
  if(raw.header.frame_id!=odom.header.frame_id || !std::isfinite(raw_age) ||
     raw_age<-.010 || raw_age>.5)throw std::logic_error("stale_or_wrong_frame_objects");
    path_safety_checker::ExtendedPredictedObjects checked_objects;
    double horizon = 3.0;
    const double age=std::max(0.0,raw_age);
    for (const auto & object : raw.objects) {
      auto transformed = lc::observed_maneuver_object(object,
        resolution);
      if (transformed.predicted_paths.empty())
        throw std::logic_error("object_prediction_missing");
      for (auto & prediction : transformed.predicted_paths) {
        if (prediction.path.size()<2 || prediction.path.front().time>1e-6)
          throw std::logic_error("object_prediction_invalid");
        for (size_t i=0;i<prediction.path.size();++i) {
          if (!std::isfinite(prediction.path[i].time) ||
              (i && prediction.path[i].time<=prediction.path[i-1].time))
            throw std::logic_error("object_prediction_invalid_time");
        }
        horizon = std::min(horizon,prediction.path.back().time-age);
        const auto initial = utils::path_safety_checker::calc_interpolated_object_pose(
          prediction.path,age,transformed.shape);
        if (!initial) throw std::logic_error("object_prediction_time_alignment_failed");
        auto aligned = prediction.path;
        aligned.clear();
        aligned.push_back(*initial);
        aligned.front().time = 0.0;
        for (const auto & point : prediction.path) {
          if (point.time <= age) continue;
          aligned.push_back(point);
          aligned.back().time -= age;
        }
        prediction.path = std::move(aligned);
      }
      checked_objects.push_back(std::move(transformed));
    }
    if (horizon < 2.5) {
      ROS_INFO_THROTTLE(1.0,"Strict LC observation insufficient: horizon=%.3f age=%.3f",horizon,age);
      throw std::logic_error("object_prediction_horizon_below_2p5s");
    }
    for (auto & object : checked_objects) {
      for (auto & prediction : object.predicted_paths) {
        const auto endpoint = utils::path_safety_checker::calc_interpolated_object_pose(
          prediction.path,horizon,object.shape);
        if (!endpoint) throw std::logic_error("object_prediction_endpoint_missing");
        prediction.path.erase(std::remove_if(prediction.path.begin(),prediction.path.end(),
          [&](const auto & point){return point.time>=horizon;}),prediction.path.end());
        prediction.path.push_back(*endpoint);
      }
    }
  return {std::move(checked_objects),horizon};
}
}
