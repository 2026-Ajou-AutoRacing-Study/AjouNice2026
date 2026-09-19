#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <xmlrpcpp/XmlRpcValue.h>

#include <stdexcept>
#include <string>

namespace
{
double requiredDouble(const ros::NodeHandle & node, const char * name)
{
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value)) {
    throw std::runtime_error("Missing required double parameter `" + std::string(name) + "` in `" + node.getNamespace() + "`");
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    throw std::runtime_error("Required parameter `" + std::string(name) + "` in `" + node.getNamespace() + "` is not a double");
  }
  return static_cast<double>(value);
}
}  // namespace

namespace autoware::vehicle_info_utils
{
VehicleInfoUtils::VehicleInfoUtils(const ros::NodeHandle & node)
{
  vehicle_info_ = createVehicleInfo(
    requiredDouble(node, "wheel_radius"), requiredDouble(node, "wheel_width"),
    requiredDouble(node, "wheel_base"), requiredDouble(node, "wheel_tread"),
    requiredDouble(node, "front_overhang"), requiredDouble(node, "rear_overhang"),
    requiredDouble(node, "left_overhang"), requiredDouble(node, "right_overhang"),
    requiredDouble(node, "vehicle_height"), requiredDouble(node, "max_steer_angle"));
}

VehicleInfo VehicleInfoUtils::getVehicleInfo() const { return vehicle_info_; }
}  // namespace autoware::vehicle_info_utils
