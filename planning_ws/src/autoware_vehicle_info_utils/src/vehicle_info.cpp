#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <ros/ros.h>

#include <cmath>
#include <limits>

namespace autoware::vehicle_info_utils
{
autoware_utils_geometry::LinearRing2d VehicleInfo::createFootprint(const double margin) const
{
  return createFootprint(margin, margin);
}

autoware_utils_geometry::LinearRing2d VehicleInfo::createFootprint(
  const double lat_margin, const double lon_margin) const
{
  return createFootprint(lat_margin, lat_margin, lat_margin, lon_margin, lon_margin);
}

autoware_utils_geometry::LinearRing2d VehicleInfo::createFootprint(
  const double front_lat_margin, const double center_lat_margin, const double rear_lat_margin,
  const double front_lon_margin, const double rear_lon_margin, const bool center_at_base_link) const
{
  using autoware_utils_geometry::LinearRing2d;
  const double x_front = front_overhang_m + wheel_base_m + front_lon_margin;
  const double x_center = center_at_base_link ? 0.0 : wheel_base_m / 2.0;
  const double x_rear = -(rear_overhang_m + rear_lon_margin);
  const double y_left_front = wheel_tread_m * 0.5 + left_overhang_m + front_lat_margin;
  const double y_right_front = -(wheel_tread_m * 0.5 + right_overhang_m + front_lat_margin);
  const double y_left_center = wheel_tread_m * 0.5 + left_overhang_m + center_lat_margin;
  const double y_right_center = -(wheel_tread_m * 0.5 + right_overhang_m + center_lat_margin);
  const double y_left_rear = wheel_tread_m * 0.5 + left_overhang_m + rear_lat_margin;
  const double y_right_rear = -(wheel_tread_m * 0.5 + right_overhang_m + rear_lat_margin);
  LinearRing2d footprint;
  footprint.reserve(7);
  footprint.emplace_back(x_front, y_left_front);
  footprint.emplace_back(x_front, y_right_front);
  footprint.emplace_back(x_center, y_right_center);
  footprint.emplace_back(x_rear, y_right_rear);
  footprint.emplace_back(x_rear, y_left_rear);
  footprint.emplace_back(x_center, y_left_center);
  footprint.emplace_back(x_front, y_left_front);
  return footprint;
}

VehicleInfo createVehicleInfo(
  const double wheel_radius_m, const double wheel_width_m, const double wheel_base_m_arg,
  const double wheel_tread_m, const double front_overhang_m, const double rear_overhang_m,
  const double left_overhang_m, const double right_overhang_m, const double vehicle_height_m,
  const double max_steer_angle_rad_arg)
{
  constexpr double kMinWheelBaseM = 1e-6;
  double wheel_base_m = wheel_base_m_arg;
  if (std::abs(wheel_base_m) < kMinWheelBaseM) {
    ROS_ERROR("wheel_base_m %.17g is almost 0.0, clamping to %.17g", wheel_base_m, kMinWheelBaseM);
    wheel_base_m = kMinWheelBaseM;
  }
  constexpr double kMinSteerAngleRad = 1e-6;
  double max_steer_angle_rad = max_steer_angle_rad_arg;
  if (std::abs(max_steer_angle_rad) < kMinSteerAngleRad) {
    ROS_ERROR("max_steer_angle_rad %.17g is almost 0.0, clamping to %.17g", max_steer_angle_rad, kMinSteerAngleRad);
    max_steer_angle_rad = kMinSteerAngleRad;
  }
  if (wheel_radius_m <= 0.0 || wheel_width_m <= 0.0 || wheel_base_m <= 0.0 ||
      wheel_tread_m <= 0.0 || front_overhang_m <= 0.0 || rear_overhang_m <= 0.0 ||
      left_overhang_m <= 0.0 || right_overhang_m <= 0.0 || vehicle_height_m <= 0.0 ||
      max_steer_angle_rad <= 0.0) {
    ROS_ERROR("given parameters contain non positive values");
  }
  return VehicleInfo{wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m,
    front_overhang_m, rear_overhang_m, left_overhang_m, right_overhang_m, vehicle_height_m,
    max_steer_angle_rad, front_overhang_m + wheel_base_m + rear_overhang_m,
    wheel_tread_m + left_overhang_m + right_overhang_m, -rear_overhang_m,
    front_overhang_m + wheel_base_m, -(wheel_tread_m / 2.0 + right_overhang_m),
    wheel_tread_m / 2.0 + left_overhang_m, 0.0, vehicle_height_m};
}

double VehicleInfo::calcMaxCurvature() const { return 1.0 / (wheel_base_m / std::tan(max_steer_angle_rad)); }

double VehicleInfo::calcCurvatureFromSteerAngle(const double steer_angle) const
{
  constexpr double kMinWheelBaseM = 1e-6;
  if (wheel_base_m < kMinWheelBaseM) {
    ROS_ERROR("wheel_base_m %.17g should not be 0 or negative", wheel_base_m);
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::tan(steer_angle) / wheel_base_m;
}

double VehicleInfo::calcSteerAngleFromCurvature(const double curvature) const
{
  if (std::abs(curvature) < 1e-6) return 0.0;
  return std::atan2(wheel_base_m, 1.0 / curvature);
}
}  // namespace autoware::vehicle_info_utils
