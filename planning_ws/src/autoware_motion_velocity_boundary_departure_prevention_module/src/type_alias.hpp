// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include <autoware/deprecated/boundary_departure_checker/parameters.hpp>
#include <autoware/deprecated/boundary_departure_checker/uncrossable_boundary_checker.hpp>
#include <autoware/deprecated/boundary_departure_checker/utils.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <autoware_adapi_v1_msgs/OperationModeState.h>
#include <autoware_control_msgs/Control.h>
#include <autoware_internal_debug_msgs/Float64Stamped.h>
#include <autoware_map_msgs/LaneletMapBin.h>
#include <autoware_planning_msgs/LaneletRoute.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <autoware_vehicle_msgs/SteeringReport.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

namespace autoware::motion_velocity_planner::experimental
{
using autoware_map_msgs::LaneletMapBin;
using autoware_planning_msgs::LaneletRoute;
using autoware_planning_msgs::Trajectory;
using autoware_planning_msgs::TrajectoryPoint;
using autoware_planning_msgs::TrajectoryConstPtr;
using autoware_planning_msgs::LaneletRouteConstPtr;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::PoseWithCovariance;
using nav_msgs::Odometry;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_adapi_v1_msgs::OperationModeState;
using autoware_adapi_v1_msgs::OperationModeStateConstPtr;
using BoundaryThreshold = boundary_departure_checker::Side<double>;
using autoware_control_msgs::Control;
using autoware_control_msgs::ControlConstPtr;
using autoware_internal_debug_msgs::Float64Stamped;
using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Segment2d;
using autoware_vehicle_msgs::SteeringReport;
using autoware_vehicle_msgs::SteeringReportConstPtr;
using boundary_departure_checker::BoundarySideWithIdx;
using boundary_departure_checker::EgoSide;
using boundary_departure_checker::EgoSides;
using boundary_departure_checker::Footprint;
using boundary_departure_checker::Footprints;
using boundary_departure_checker::g_side_keys;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using BDCParam = boundary_departure_checker::Param;
using SegmentWithIdx = boundary_departure_checker::SegmentWithIdx;
using DiagStatus = diagnostic_msgs::DiagnosticStatus;

namespace bg = boost::geometry;                             // NOLINT
namespace bgi = boost::geometry::index;                     // NOLINT
namespace trajectory = autoware::experimental::trajectory;  // NOLINT
namespace bdc_utils = boundary_departure_checker::utils;    // NOLINT
namespace polling_policy = autoware_utils::polling_policy;  // NOLINT

using autoware_utils::create_default_marker;     // NOLINT
using autoware_utils::create_marker_color;       // NOLINT
using autoware_utils::create_marker_scale;       // NOLINT
using autoware_utils::get_or_declare_parameter;  // NOLINT
using autoware_utils::ProcessingTimePublisher;   // NOLINT
using autoware_utils::to_msg;                    // NOLINT
using autoware_utils_geometry::Point2d;          // NOLINT
using autoware_utils_system::StopWatch;          // NOLINT
using vehicle_info_utils::VehicleInfo;           // NOLINT

using boundary_departure_checker::DeparturePoint;                       // NOLINT
using boundary_departure_checker::DeparturePoints;                      // NOLINT
using boundary_departure_checker::DepartureType;                        // NOLINT
using boundary_departure_checker::FootprintMap;                         // NOLINT
using boundary_departure_checker::FootprintMargin;                      // NOLINT
using boundary_departure_checker::FootprintType;                        // NOLINT
using boundary_departure_checker::ProjectionToBound;                    // NOLINT
using boundary_departure_checker::Side;                                 // NOLINT
using boundary_departure_checker::SideKey;                              // NOLINT
using boundary_departure_checker::UncrossableBoundaryDepartureChecker;  // NOLINT

using boundary_departure_checker::AbnormalitiesConfigs;
using boundary_departure_checker::AbnormalityConfig;
using boundary_departure_checker::DepartureData;
using boundary_departure_checker::LocalizationConfig;
using boundary_departure_checker::LongitudinalConfig;
using boundary_departure_checker::NormalConfig;
using boundary_departure_checker::SteeringConfig;
using boundary_departure_checker::TriggerThreshold;
}  // namespace autoware::motion_velocity_planner::experimental

#endif  // TYPE_ALIAS_HPP_
