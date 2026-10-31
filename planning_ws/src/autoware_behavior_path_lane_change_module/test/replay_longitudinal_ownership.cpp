// Re-evaluate exported bag hypotheses with the production LC safety checker.
#include <autoware/behavior_path_lane_change_module/scene.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
namespace bpp=autoware::behavior_path_planner;
namespace safety=bpp::utils::path_safety_checker;
using Json=nlohmann::json;
geometry_msgs::Pose pose(const Json & j) {
  geometry_msgs::Pose p;p.position.x=j[0];p.position.y=j[1];
  const double yaw=j[2];p.orientation.z=std::sin(yaw/2);p.orientation.w=std::cos(yaw/2);return p;
}
class Harness:public bpp::NormalLaneChange {
public:
  Harness(const std::shared_ptr<bpp::LaneChangeParameters> & p):NormalLaneChange(p,
    bpp::LaneChangeModuleType::EXTERNAL_REQUEST,autoware::route_handler::Direction::LEFT) {}
  using NormalLaneChange::is_colliding;
  using NormalLaneChange::common_data_ptr_;
};
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  ros::init(argc,argv,"lc_ownership_snapshot_replay",ros::init_options::NoSigintHandler);
  ros::Time::setNow(ros::Time(100));std::ifstream f(argv[1]);Json j=Json::parse(f);
  auto parameters=std::make_shared<bpp::LaneChangeParameters>();
  parameters->overtake.enable=false;
  const auto read_rss=[](const Json & p) {
    safety::RSSparams r;r.front_vehicle_deceleration=p["expected_front_deceleration"];
    r.rear_vehicle_deceleration=p["expected_rear_deceleration"];
    r.rear_vehicle_reaction_time=p["rear_vehicle_reaction_time"];
    r.rear_vehicle_safety_time_margin=p["rear_vehicle_safety_time_margin"];
    r.longitudinal_distance_min_threshold=p["longitudinal_distance_min_threshold"];
    r.lateral_distance_max_threshold=p["lateral_distance_max_threshold"];
    r.extended_polygon_policy=p["extended_polygon_policy"];return r;
  };
  parameters->safety.rss_params=read_rss(j["rss"]);
  parameters->safety.rss_params_for_prepare=parameters->safety.rss_params;
  parameters->safety.collision_check.th_yaw_diff=3.14159265;
  const auto & v=j["vehicle"];
  auto data=std::make_shared<bpp::PlannerData>();
  data->parameters.vehicle_info=autoware::vehicle_info_utils::createVehicleInfo(
    v["wheel_radius"],v["wheel_width"],v["wheel_base"],v["wheel_tread"],v["front_overhang"],
    v["rear_overhang"],v["left_overhang"],v["right_overhang"],v["vehicle_height"],v["max_steer_angle"]);
  auto odom=boost::make_shared<nav_msgs::Odometry>();odom->pose.pose=pose(j["ego_pose"]);
  odom->twist.twist.linear.x=j["ego_speed"];data->self_odometry=odom;
  data->self_acceleration=boost::make_shared<geometry_msgs::AccelWithCovarianceStamped>();
  Harness module(parameters);module.setData(data);
  bpp::LaneChangePath candidate;candidate.path.header.frame_id="map";
  for(const auto & p:j["path"]) {
    bpp::PathPointWithLaneId point;point.point.pose=pose(p);candidate.path.points.push_back(point);
  }
  module.common_data_ptr_->current_lanes_path=candidate.path;
  candidate.info.lane_changing_start=pose(j["lc_start"]);
  candidate.info.lane_changing_end=pose(j["lc_end"]);
  candidate.info.duration={0.0,j["duration"].get<double>()};
  Json report={{"kind","production_checker_on_reconstructed_bag_hypotheses"},{"checks",Json::array()}};
  for(const auto & o:j["objects"]) {
    safety::ExtendedPredictedObject obj;obj.uuid.uuid[0]=o["actor"];
    obj.initial_pose=pose(o["pose"]);obj.initial_twist.linear.x=o["speed"];
    obj.shape.type=autoware_perception_msgs::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x=o["dimensions"][0];obj.shape.dimensions.y=o["dimensions"][1];
    obj.shape.dimensions.z=o["dimensions"][2];
    for(const auto & pp:o["paths"]) {
      safety::PredictedPathWithPolygon path;path.confidence=pp["confidence"];
      for(const auto & p:pp["points"]) {
        const auto at=pose({p[1],p[2],p[3]});path.path.emplace_back(p[0],at,p[4],autoware_utils::to_polygon2d(at,obj.shape));
      }
      obj.predicted_paths.push_back(path);
    }
    for(const auto & hypothesis:j["hypotheses"]) {
      std::vector<safety::PoseWithVelocityStamped> prediction;
      for(const auto & p:hypothesis["points"]) prediction.emplace_back(p[0],pose({p[1],p[2],p[3]}),p[4]);
      safety::CollisionCheckDebugMap debug;
      const bool unsafe=module.is_colliding(candidate,obj,prediction,parameters->safety.rss_params,debug,false);
      const auto reason=debug.empty()?"none":debug.begin()->second.unsafe_reason;
      report["checks"].push_back({{"actor",o["actor"]},{"hypothesis",hypothesis["name"]},{"unsafe",unsafe},{"reason",reason}});
    }
  }
  std::cout<<report.dump(2)<<std::endl;
}
