// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_velocity_planner/test_utils.hpp"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace {
XmlRpc::XmlRpcValue fixture_value(const YAML::Node & node) {
  if(node.IsSequence()) {
    XmlRpc::XmlRpcValue result;result.setSize(static_cast<int>(node.size()));
    for(size_t i=0;i<node.size();++i)result[static_cast<int>(i)]=fixture_value(node[i]);
    return result;
  }
  if(!node.IsScalar())throw std::invalid_argument("BVP fixture requires scalar or array leaf");
  if(node.Tag()=="!" || node.Tag()=="tag:yaml.org,2002:str")return XmlRpc::XmlRpcValue(node.as<std::string>());
  bool boolean;int integer;double number;
  if(YAML::convert<bool>::decode(node,boolean))return XmlRpc::XmlRpcValue(boolean);
  if(YAML::convert<int>::decode(node,integer))return XmlRpc::XmlRpcValue(integer);
  if(YAML::convert<double>::decode(node,number))return XmlRpc::XmlRpcValue(number);
  return XmlRpc::XmlRpcValue(node.as<std::string>());
}
void load_fixture_parameters(ros::NodeHandle & target,const std::string & prefix,const YAML::Node & node) {
  if(node.IsMap()) {
    for(const auto & entry:node) {
      auto key=entry.first.as<std::string>();std::replace(key.begin(),key.end(),'.','/');
      load_fixture_parameters(target,prefix.empty()?key:prefix+"/"+key,entry.second);
    }
  } else target.setParam(prefix,fixture_value(node));
}
}


std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: behavior_velocity_planner → test_node_
  test_manager->subscribeOutput<autoware_planning_msgs::Path>(
    "/behavior_velocity_planner_node/output/path");

  return test_manager;
}

std::shared_ptr<FullBehaviorVelocityPlannerNode> generateNode(
  const std::vector<PluginInfo> & plugin_info_vec)
{
  ros::NodeHandle private_node("/behavior_velocity_planner_node");

  const auto autoware_test_utils_dir =
    ros::package::getPath("autoware_test_utils");
  const auto behavior_velocity_planner_common_dir =
    ros::package::getPath("autoware_behavior_velocity_planner_common");
  const auto behavior_velocity_planner_dir =
    ros::package::getPath("autoware_behavior_velocity_planner");
  const auto velocity_smoother_dir =
    ros::package::getPath("autoware_velocity_smoother");

  const auto get_behavior_velocity_module_config = [](const std::string & module) {
    const auto package_name = "autoware_behavior_velocity_" + module + "_module";
    const auto package_path = ros::package::getPath(package_name);
    return package_path + "/config/" + module + ".param.yaml";
  };

  std::vector<std::string> plugin_names;
  for (const auto & plugin_info : plugin_info_vec) {
    plugin_names.emplace_back(plugin_info.plugin_name);
  }



  auto yaml_files = std::vector<std::string>{
    autoware_test_utils_dir + "/config/test_common.param.yaml",
    autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
    autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
    velocity_smoother_dir + "/config/ros1/default_velocity_smoother.yaml",
    velocity_smoother_dir + "/config/ros1/Analytical.yaml",
    behavior_velocity_planner_common_dir + "/config/behavior_velocity_planner_common.param.yaml",
    behavior_velocity_planner_dir + "/config/behavior_velocity_planner.param.yaml"};
  for (const auto & plugin_info : plugin_info_vec) {
    yaml_files.push_back(get_behavior_velocity_module_config(plugin_info.module_name));
  }

  for (const auto & file : yaml_files) {
    const auto document = YAML::LoadFile(file);
    const auto parameters = document["/**"] ? document["/**"]["ros__parameters"] : document;
    if (!parameters.IsMap()) throw std::runtime_error("BVP fixture missing parameter map: " + file);
    load_fixture_parameters(private_node, "", parameters);
  }
  private_node.setParam("launch_modules", plugin_names);
  private_node.setParam("is_simulation", false);

  // TODO(Takagi, Isamu): set launch_modules
  // TODO(Kyoichi Sugahara) set to true launch_virtual_traffic_light
  // TODO(Kyoichi Sugahara) set to true launch_occlusion_spot
  // TODO(Kyoichi Sugahara) set to true launch_speed_bump

  return std::make_shared<FullBehaviorVelocityPlannerNode>(private_node);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<FullBehaviorVelocityPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    *test_target_node, "/tf", autoware::test_utils::makeTFMsg(ros::Time::now(), "base_link", "map"));
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/accel",
    geometry_msgs::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/dynamic_objects",
    autoware_perception_msgs::PredictedObjects{});
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/no_ground_pointcloud",
    [] { sensor_msgs::PointCloud2 cloud; cloud.header.frame_id="base_link"; return cloud; }());
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/accel",
    geometry_msgs::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/vector_map",
    autoware::test_utils::makeMapBinMsg());
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/traffic_signals",
    autoware_perception_msgs::TrafficLightGroupArray{});
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/external_velocity_limit_mps",
    autoware_internal_planning_msgs::VelocityLimit{});
  test_manager->publishInput(
    *test_target_node, "/behavior_velocity_planner_node/input/occupancy_grid",
    autoware::test_utils::makeCostMapMsg());
}
}  // namespace autoware::behavior_velocity_planner
