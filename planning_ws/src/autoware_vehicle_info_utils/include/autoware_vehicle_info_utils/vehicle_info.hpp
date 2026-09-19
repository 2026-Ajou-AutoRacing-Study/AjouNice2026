#ifndef AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_
#define AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <cstddef>

namespace autoware::vehicle_info_utils
{
struct VehicleInfo
{
  double wheel_radius_m{};
  double wheel_width_m{};
  double wheel_base_m{};
  double wheel_tread_m{};
  double front_overhang_m{};
  double rear_overhang_m{};
  double left_overhang_m{};
  double right_overhang_m{};
  double vehicle_height_m{};
  double max_steer_angle_rad{};
  double vehicle_length_m{};
  double vehicle_width_m{};
  double min_longitudinal_offset_m{};
  double max_longitudinal_offset_m{};
  double min_lateral_offset_m{};
  double max_lateral_offset_m{};
  double min_height_offset_m{};
  double max_height_offset_m{};

  static constexpr size_t FrontLeftIndex = 0;
  static constexpr size_t FrontRightIndex = 1;
  static constexpr size_t RearRightIndex = 3;
  static constexpr size_t RearLeftIndex = 4;

  [[nodiscard]] autoware_utils_geometry::LinearRing2d createFootprint(double margin = 0.0) const;
  [[nodiscard]] autoware_utils_geometry::LinearRing2d createFootprint(
    double lat_margin, double lon_margin) const;
  [[nodiscard]] autoware_utils_geometry::LinearRing2d createFootprint(
    double front_lat_margin, double center_lat_margin, double rear_lat_margin,
    double front_lon_margin, double rear_lon_margin, bool center_at_base_link = false) const;
  [[nodiscard]] double calcMaxCurvature() const;
  [[nodiscard]] double calcCurvatureFromSteerAngle(double steer_angle) const;
  [[nodiscard]] double calcSteerAngleFromCurvature(double curvature) const;
};

[[nodiscard]] VehicleInfo createVehicleInfo(
  double wheel_radius_m, double wheel_width_m, double wheel_base_m, double wheel_tread_m,
  double front_overhang_m, double rear_overhang_m, double left_overhang_m,
  double right_overhang_m, double vehicle_height_m, double max_steer_angle_rad);
}  // namespace autoware::vehicle_info_utils

#endif
