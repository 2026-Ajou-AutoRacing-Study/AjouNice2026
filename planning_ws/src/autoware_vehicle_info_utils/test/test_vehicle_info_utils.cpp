#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>
#include <stdexcept>

namespace
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware::vehicle_info_utils::VehicleInfoUtils;

void setAllDoubleParams(const ros::NodeHandle & nh)
{
  nh.setParam("wheel_radius", 0.39); nh.setParam("wheel_width", 0.42);
  nh.setParam("wheel_base", 2.74); nh.setParam("wheel_tread", 1.63);
  nh.setParam("front_overhang", 1.0); nh.setParam("rear_overhang", 1.03);
  nh.setParam("left_overhang", 0.1); nh.setParam("right_overhang", 0.1);
  nh.setParam("vehicle_height", 2.5); nh.setParam("max_steer_angle", 0.70);
}

TEST(VehicleInfoLaunch, LoadsRos1YamlAtExplicitNamespace)
{
  const ros::NodeHandle nh("vehicle_ns");
  const VehicleInfo info = VehicleInfoUtils(nh).getVehicleInfo();
  EXPECT_DOUBLE_EQ(0.39, info.wheel_radius_m);
  EXPECT_DOUBLE_EQ(2.74, info.wheel_base_m);
  EXPECT_DOUBLE_EQ(2.74 + 1.0 + 1.03, info.vehicle_length_m);
  EXPECT_DOUBLE_EQ(1.63 + 0.1 + 0.1, info.vehicle_width_m);
}

TEST(VehicleInfoGeometry, FootprintAndCurvatureMatchRos2Equations)
{
  const VehicleInfo info = autoware::vehicle_info_utils::createVehicleInfo(0.39, 0.42, 2.74, 1.63, 1.0, 1.03, 0.1, 0.1, 2.5, 0.7);
  const auto footprint = info.createFootprint();
  ASSERT_EQ(7U, footprint.size());
  EXPECT_DOUBLE_EQ(3.74, footprint.at(VehicleInfo::FrontLeftIndex).x());
  EXPECT_DOUBLE_EQ(0.915, footprint.at(VehicleInfo::FrontLeftIndex).y());
  EXPECT_DOUBLE_EQ(3.74, footprint.at(VehicleInfo::FrontRightIndex).x());
  EXPECT_DOUBLE_EQ(-0.915, footprint.at(VehicleInfo::FrontRightIndex).y());
  EXPECT_DOUBLE_EQ(-1.03, footprint.at(VehicleInfo::RearRightIndex).x());
  EXPECT_DOUBLE_EQ(-1.03, footprint.at(VehicleInfo::RearLeftIndex).x());
  EXPECT_NEAR(std::tan(0.7) / 2.74, info.calcMaxCurvature(), 1e-15);
  EXPECT_NEAR(std::tan(0.7) / 2.74, info.calcCurvatureFromSteerAngle(0.7), 1e-15);
  EXPECT_NEAR(0.7, info.calcSteerAngleFromCurvature(std::tan(0.7) / 2.74), 1e-15);
  EXPECT_DOUBLE_EQ(0.0, info.calcSteerAngleFromCurvature(1e-8));
}

TEST(VehicleInfoGeometry, NearZeroWheelBaseAndSteerClampExactly)
{
  const VehicleInfo info = autoware::vehicle_info_utils::createVehicleInfo(0.39, 0.42, 0.0, 1.63, 1.0, 1.03, 0.1, 0.1, 2.5, 0.0);
  EXPECT_DOUBLE_EQ(1e-6, info.wheel_base_m);
  EXPECT_DOUBLE_EQ(1e-6, info.max_steer_angle_rad);
}

TEST(VehicleInfoParameters, MissingRequiredKeyRejectsBeforeDerivedResult)
{
  ros::NodeHandle nh("strict_missing");
  setAllDoubleParams(nh);
  ASSERT_TRUE(nh.deleteParam("vehicle_height"));
  EXPECT_THROW((void)VehicleInfoUtils(nh), std::runtime_error);
}

TEST(VehicleInfoParameters, IntegerIsRejectedRatherThanCoercedToDouble)
{
  ros::NodeHandle nh("strict_wrong_type");
  setAllDoubleParams(nh);
  nh.setParam("wheel_radius", 1);
  EXPECT_THROW((void)VehicleInfoUtils(nh), std::runtime_error);
}
}  // namespace
