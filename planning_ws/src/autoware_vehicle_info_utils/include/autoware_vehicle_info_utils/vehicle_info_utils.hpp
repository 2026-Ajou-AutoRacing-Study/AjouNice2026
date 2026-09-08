#ifndef AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_
#define AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <ros/node_handle.h>

namespace autoware::vehicle_info_utils
{
class VehicleInfoUtils
{
public:
  explicit VehicleInfoUtils(const ros::NodeHandle & node);
  [[nodiscard]] VehicleInfo getVehicleInfo() const;

private:
  VehicleInfo vehicle_info_;
};
}  // namespace autoware::vehicle_info_utils

#endif
