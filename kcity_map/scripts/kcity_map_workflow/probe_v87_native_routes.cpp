// Offline/private-master route audit using the installed production DefaultPlanner plugin.
#include <autoware/mission_planner_universe/mission_planner_plugin.hpp>
#include <pluginlib/class_loader.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <iostream>
int main(int argc,char**argv) {
 ros::init(argc,argv,"v87_native_route_probe");ros::NodeHandle node("~");
 YAML::Node report;bool passed=true;
 try {
  const auto package=ros::package::getPath("autoware_mission_planner_universe");
  pluginlib::ClassLoader<autoware::mission_planner_universe::PlannerPlugin> loader(
   "autoware_mission_planner_universe","autoware::mission_planner_universe::PlannerPlugin",
   "plugin",{package+"/plugins/plugin_description.xml"});
  auto planner=loader.createInstance("autoware::mission_planner_universe::lanelet2::DefaultPlanner");
  planner->initialize(&node);
  auto map=ros::topic::waitForMessage<autoware_map_msgs::LaneletMapBin>("/map/vector_map",node,ros::Duration(30));
  if(!map)throw std::runtime_error("map timeout");
  planner->initialize(&node,map);
  XmlRpc::XmlRpcValue cases;if(!node.getParam("route_policy_cases",cases))throw std::runtime_error("no cases");
  for(int i=0;i<cases.size();++i) {
   YAML::Node result;result["name"]=static_cast<std::string>(cases[i]["name"]);
   autoware::mission_planner_universe::PlannerPlugin::RoutePoints points;
   for(int j=0;j<cases[i]["points"].size();++j) {
    const auto &c=cases[i]["points"][j];geometry_msgs::Pose p;
    p.position.x=static_cast<double>(c[0]);p.position.y=static_cast<double>(c[1]);p.position.z=static_cast<double>(c[2]);
    const auto yaw=static_cast<double>(c[3]);p.orientation.z=std::sin(yaw/2);p.orientation.w=std::cos(yaw/2);points.push_back(p);
   }
   planner->clearRoute();const auto route=planner->plan(points);
   result["nonempty"]=!route.segments.empty();if(route.segments.empty())passed=false;
   for(const auto&s:route.segments) {
    YAML::Node section;section["preferred"]=s.preferred_primitive.id;
    for(const auto&p:s.primitives)section["primitives"].push_back(p.id);
    result["sections"].push_back(section);
   }
   result["goal_x"]=route.goal_pose.position.x;result["goal_y"]=route.goal_pose.position.y;
   report["cases"].push_back(result);
  }
  report["status"]=passed?"PASS":"FAIL";
 }catch(const std::exception&e){passed=false;report["status"]="FAIL";report["error"]=e.what();}
 std::string path;node.param<std::string>("output_file",path,"/tmp/v87_native_routes.yaml");
 std::ofstream(path)<<report;std::cout<<"native_routes="<<(passed?"PASS":"FAIL")<<std::endl;
 return passed?0:1;
}
