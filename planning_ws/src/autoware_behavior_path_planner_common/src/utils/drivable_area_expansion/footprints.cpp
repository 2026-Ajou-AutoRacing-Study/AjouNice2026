#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering_core.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <tf2/utils.h>

#include <limits>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
Polygon2d translate_polygon(const Polygon2d & polygon, const double x, const double y)
{
  Polygon2d translated;
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> transform(x, y);
  boost::geometry::transform(polygon, translated, transform);
  return translated;
}
Polygon2d create_footprint(const geometry_msgs::Pose & pose, const Polygon2d & base)
{
  const double angle = tf2::getYaw(pose.orientation);
  return translate_polygon(
    autoware_utils::rotate_polygon(base, angle), pose.position.x, pose.position.y);
}
MultiPolygon2d create_object_footprints(const autoware_perception_msgs::PredictedObjects & objects, const DrivableAreaExpansionParameters & params)
{
  using utils::path_safety_checker::filter::velocity_filter;
  MultiPolygon2d footprints;
  if (!params.object_exclusion.exclude_dynamic && !params.object_exclusion.exclude_static) return footprints;
  for (const auto & object : objects.objects) {
    const double front = object.shape.dimensions.x / 2 + params.object_exclusion.front_offset;
    const double rear = -object.shape.dimensions.x / 2 - params.object_exclusion.rear_offset;
    const double left = object.shape.dimensions.y / 2 + params.object_exclusion.left_offset;
    const double right = -object.shape.dimensions.y / 2 - params.object_exclusion.right_offset;
    Polygon2d base; base.outer() = {Point2d{front,left}, Point2d{front,right}, Point2d{rear,right}, Point2d{rear,left}, Point2d{front,left}};
    if (params.object_exclusion.exclude_dynamic) { for (const auto & path : object.kinematics.predicted_paths) for (const auto & pose : path.path) footprints.push_back(create_footprint(pose, base)); continue; }
    if (params.object_exclusion.exclude_static && velocity_filter(object.kinematics.initial_twist_with_covariance.twist, -std::numeric_limits<double>::epsilon(), params.object_exclusion.stopped_obj_vel_th)) footprints.push_back(create_footprint(object.kinematics.initial_pose_with_covariance.pose, base));
  }
  return footprints;
}
}
