// Copyright 2026 The Autoware Contributors
// Licensed under the Apache License, Version 2.0.
// Re-evaluate the measured entry gate using the finalized bag frame exports.
#include <autoware/behavior_path_lane_change_module/utils/overtake.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

using Json = nlohmann::json;
namespace lc = autoware::behavior_path_planner::utils::lane_change;

Json read(const std::filesystem::path & path)
{
  std::ifstream file(path);
  return Json::parse(file);
}

int main(int argc, char ** argv)
{
  if (argc != 2) return 2;
  const std::filesystem::path root(argv[1]);
  const auto findings = read(root/"rear_findings.json");
  const auto vehicle = autoware::vehicle_info_utils::createVehicleInfo(
    .39,.23,3,1.651,.845,.790,.1195,.1195,1.8,.6);
  autoware::behavior_path_planner::lane_change::OvertakeParameters policy;
  policy.enable=true;
  policy.assume_cooperative_npc=true;
  Json report{{"kind","recorded_measured_entry_gate_replay"}, {"cases",Json::array()},
    {"limits","Recorded vehicle positions are fixed. This checks the entry gate, not a closed-loop outcome. Road reference is the local Ego tangent. UUIDs use the prior GT/perception association."}};
  bool all_rejected=true;
  for (const auto & item: findings.at("cases")) {
    const auto frames=read(root/item.at("case").get<std::string>()/"frames.json");
    const double time=item.at("first_recorded_rear_rejection_for_collision_object").at("t");
    const auto frame=std::min_element(frames.begin(),frames.end(),[time](const auto & a,const auto & b){
      return std::abs(a.at("t").template get<double>()-time)<std::abs(b.at("t").template get<double>()-time);
    });
    const auto & e=frame->at("ego");
    const auto & objects=frame->at("objects");
    const auto object=std::find_if(objects.begin(),objects.end(),[&](const auto & o){
      return o.at("uuid")==item.at("associated_uuid");
    });
    if (object==objects.end()) return 3;
    geometry_msgs::Pose pose;
    pose.position.x=e.at("x"); pose.position.y=e.at("y");
    const double yaw=e.at("yaw");
    pose.orientation.z=std::sin(yaw/2); pose.orientation.w=std::cos(yaw/2);
    autoware_internal_planning_msgs::PathWithLaneId reference;
    for (int s=-150;s<=150;++s) {
      autoware_internal_planning_msgs::PathPointWithLaneId point;
      point.point.pose=pose;
      point.point.pose.position.x+=s*std::cos(yaw);
      point.point.pose.position.y+=s*std::sin(yaw);
      reference.points.push_back(point);
    }
    autoware_utils_geometry::Polygon2d polygon;
    for (const auto & point:object->at("polygon")) polygon.outer().emplace_back(point.at(0),point.at(1));
    polygon.outer().push_back(polygon.outer().front());
    const double gap=lc::rear_body_gap(reference,pose,polygon,vehicle);
    const double npc_speed=std::hypot(object->at("vx").get<double>(),object->at("vy").get<double>());
    const double required=lc::cooperative_rear_distance(e.at("speed"),npc_speed,policy);
    const bool ready=gap>=required;
    all_rejected &= !ready;
    report["cases"].push_back({{"case",item.at("case")},{"frame_t",frame->at("t")},
      {"object_uuid",object->at("uuid")},{"rear_body_gap_m",gap},
      {"required_gap_m",required},{"entry_allowed",ready}});
  }
  report["all_previous_bad_entries_rejected"]=all_rejected;
  std::cout<<report.dump(2)<<std::endl;
  return all_rejected ? 0 : 1;
}
