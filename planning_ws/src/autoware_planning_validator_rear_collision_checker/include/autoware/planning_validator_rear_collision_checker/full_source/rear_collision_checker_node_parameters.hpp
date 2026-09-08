// Generated from the retained original schema. Do not relax validation here.
#pragma once
#include <autoware/planning_validator/full_source/native_support.hpp>
#include <XmlRpcValue.h>
#include <mutex>
#include <type_traits>
namespace autoware::planning_validator::full_source::rear_collision_checker_node {
struct Params {
struct Common {
  double on_time_buffer{};
  double off_time_buffer{};
  bool check_on_unstoppable{};
struct Pointcloud {
struct Range {
  double dead_zone{};
  double buffer{};
} range;
struct CropBoxFilter {
struct X {
  double max{};
  double min{};
} x;
struct Z {
  double max{};
  double min{};
} z;
} crop_box_filter;
struct VoxelGridFilter {
  double x{};
  double y{};
  double z{};
} voxel_grid_filter;
struct Clustering {
  double cluster_tolerance{};
  double min_cluster_height{};
  int min_cluster_size{};
  int max_cluster_size{};
} clustering;
struct VelocityEstimation {
  double observation_time{};
  double max_acceleration{};
} velocity_estimation;
  double latency{};
} pointcloud;
struct Filter {
  double min_velocity{};
  double moving_time{};
} filter;
struct TimeToCollision {
  double margin{};
} time_to_collision;
struct Ego {
  double reaction_time{};
  double min_velocity{};
  double max_velocity{};
  double max_acceleration{};
  double max_deceleration{};
  double max_positive_jerk{};
  double max_negative_jerk{};
  double nominal_deceleration{};
  double nominal_positive_jerk{};
  double nominal_negative_jerk{};
} ego;
struct BlindSpot {
  double lookahead_time{};
  std::string metric{};
struct Check {
  bool front{};
  bool left{};
  bool right{};
  double yaw_th{};
} check;
struct Offset {
  double inner{};
  double outer{};
} offset;
struct Participants {
  double reaction_time{};
  double max_velocity{};
  double max_deceleration{};
} participants;
} blind_spot;
struct AdjacentLane {
  double lookahead_time{};
  std::string metric{};
struct Check {
  bool front{};
} check;
struct Offset {
  double left{};
  double right{};
} offset;
struct Participants {
  double reaction_time{};
  double max_velocity{};
  double max_deceleration{};
} participants;
} adjacent_lane;
} common;
};
class ParamListener {
  ros::NodeHandle node_;
  Params cached_;
  std::mutex mutex_;
  template<class T> static T read(XmlRpc::XmlRpcValue tree,const std::string & key) {
    size_t begin=0;
    while(begin<key.size()) {
      const auto end=key.find('/',begin);
      const auto part=key.substr(begin,end==std::string::npos?end:end-begin);
      if(tree.getType()!=XmlRpc::XmlRpcValue::TypeStruct || !tree.hasMember(part))
        throw std::invalid_argument("Missing parameter: "+key);
      XmlRpc::XmlRpcValue child=tree[part]; tree=child;
      if(end==std::string::npos)break;
      begin=end+1;
    }
    if constexpr(std::is_same_v<T,double>) {
      double result;
      if(tree.getType()==XmlRpc::XmlRpcValue::TypeDouble)result=static_cast<double>(tree);
      else if(tree.getType()==XmlRpc::XmlRpcValue::TypeInt)result=static_cast<int>(tree);
      else throw std::invalid_argument("Expected numeric parameter: "+key);
      if(!std::isfinite(result))throw std::invalid_argument("Nonfinite parameter: "+key);
      return result;
    } else if constexpr(std::is_same_v<T,int>) {
      if(tree.getType()!=XmlRpc::XmlRpcValue::TypeInt)throw std::invalid_argument("Expected integer: "+key);
      return static_cast<int>(tree);
    } else if constexpr(std::is_same_v<T,bool>) {
      if(tree.getType()!=XmlRpc::XmlRpcValue::TypeBoolean)throw std::invalid_argument("Expected boolean: "+key);
      return static_cast<bool>(tree);
    } else {
      if(tree.getType()!=XmlRpc::XmlRpcValue::TypeString)throw std::invalid_argument("Expected string: "+key);
      return static_cast<std::string>(tree);
    }
  }
  Params snapshot() const {
    XmlRpc::XmlRpcValue tree;
    if(!node_.getParam(node_.getNamespace(),tree))throw std::invalid_argument("Missing validator parameter namespace");
    Params result;
    result.common.on_time_buffer=read<double>(tree,"common/on_time_buffer");
    if(!(result.common.on_time_buffer>=0.0))throw std::invalid_argument("Parameter constraint: common/on_time_buffer");
    result.common.off_time_buffer=read<double>(tree,"common/off_time_buffer");
    if(!(result.common.off_time_buffer>=0.0))throw std::invalid_argument("Parameter constraint: common/off_time_buffer");
    result.common.check_on_unstoppable=read<bool>(tree,"common/check_on_unstoppable");
    result.common.pointcloud.range.dead_zone=read<double>(tree,"common/pointcloud/range/dead_zone");
    if(!(result.common.pointcloud.range.dead_zone>=0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/range/dead_zone");
    result.common.pointcloud.range.buffer=read<double>(tree,"common/pointcloud/range/buffer");
    if(!(result.common.pointcloud.range.buffer>=0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/range/buffer");
    result.common.pointcloud.crop_box_filter.x.max=read<double>(tree,"common/pointcloud/crop_box_filter/x/max");
    result.common.pointcloud.crop_box_filter.x.min=read<double>(tree,"common/pointcloud/crop_box_filter/x/min");
    result.common.pointcloud.crop_box_filter.z.max=read<double>(tree,"common/pointcloud/crop_box_filter/z/max");
    result.common.pointcloud.crop_box_filter.z.min=read<double>(tree,"common/pointcloud/crop_box_filter/z/min");
    result.common.pointcloud.voxel_grid_filter.x=read<double>(tree,"common/pointcloud/voxel_grid_filter/x");
    if(!(result.common.pointcloud.voxel_grid_filter.x>0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/voxel_grid_filter/x");
    result.common.pointcloud.voxel_grid_filter.y=read<double>(tree,"common/pointcloud/voxel_grid_filter/y");
    if(!(result.common.pointcloud.voxel_grid_filter.y>0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/voxel_grid_filter/y");
    result.common.pointcloud.voxel_grid_filter.z=read<double>(tree,"common/pointcloud/voxel_grid_filter/z");
    if(!(result.common.pointcloud.voxel_grid_filter.z>0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/voxel_grid_filter/z");
    result.common.pointcloud.clustering.cluster_tolerance=read<double>(tree,"common/pointcloud/clustering/cluster_tolerance");
    if(!(result.common.pointcloud.clustering.cluster_tolerance>0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/clustering/cluster_tolerance");
    result.common.pointcloud.clustering.min_cluster_height=read<double>(tree,"common/pointcloud/clustering/min_cluster_height");
    if(!(result.common.pointcloud.clustering.min_cluster_height>0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/clustering/min_cluster_height");
    result.common.pointcloud.clustering.min_cluster_size=read<int>(tree,"common/pointcloud/clustering/min_cluster_size");
    if(!(result.common.pointcloud.clustering.min_cluster_size>=1 && result.common.pointcloud.clustering.min_cluster_size<=2147483647))throw std::invalid_argument("Parameter constraint: common/pointcloud/clustering/min_cluster_size");
    result.common.pointcloud.clustering.max_cluster_size=read<int>(tree,"common/pointcloud/clustering/max_cluster_size");
    if(!(result.common.pointcloud.clustering.max_cluster_size>=1 && result.common.pointcloud.clustering.max_cluster_size<=2147483647))throw std::invalid_argument("Parameter constraint: common/pointcloud/clustering/max_cluster_size");
    result.common.pointcloud.velocity_estimation.observation_time=read<double>(tree,"common/pointcloud/velocity_estimation/observation_time");
    if(!(result.common.pointcloud.velocity_estimation.observation_time>=0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/velocity_estimation/observation_time");
    result.common.pointcloud.velocity_estimation.max_acceleration=read<double>(tree,"common/pointcloud/velocity_estimation/max_acceleration");
    if(!(result.common.pointcloud.velocity_estimation.max_acceleration>=0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/velocity_estimation/max_acceleration");
    result.common.pointcloud.latency=read<double>(tree,"common/pointcloud/latency");
    if(!(result.common.pointcloud.latency>=0.0))throw std::invalid_argument("Parameter constraint: common/pointcloud/latency");
    result.common.filter.min_velocity=read<double>(tree,"common/filter/min_velocity");
    result.common.filter.moving_time=read<double>(tree,"common/filter/moving_time");
    result.common.time_to_collision.margin=read<double>(tree,"common/time_to_collision/margin");
    if(!(result.common.time_to_collision.margin>0.0))throw std::invalid_argument("Parameter constraint: common/time_to_collision/margin");
    result.common.ego.reaction_time=read<double>(tree,"common/ego/reaction_time");
    if(!(result.common.ego.reaction_time>0.0))throw std::invalid_argument("Parameter constraint: common/ego/reaction_time");
    result.common.ego.min_velocity=read<double>(tree,"common/ego/min_velocity");
    if(!(result.common.ego.min_velocity>0.0))throw std::invalid_argument("Parameter constraint: common/ego/min_velocity");
    result.common.ego.max_velocity=read<double>(tree,"common/ego/max_velocity");
    if(!(result.common.ego.max_velocity>0.0))throw std::invalid_argument("Parameter constraint: common/ego/max_velocity");
    result.common.ego.max_acceleration=read<double>(tree,"common/ego/max_acceleration");
    if(!(result.common.ego.max_acceleration>0.0))throw std::invalid_argument("Parameter constraint: common/ego/max_acceleration");
    result.common.ego.max_deceleration=read<double>(tree,"common/ego/max_deceleration");
    result.common.ego.max_positive_jerk=read<double>(tree,"common/ego/max_positive_jerk");
    if(!(result.common.ego.max_positive_jerk>0.0))throw std::invalid_argument("Parameter constraint: common/ego/max_positive_jerk");
    result.common.ego.max_negative_jerk=read<double>(tree,"common/ego/max_negative_jerk");
    result.common.ego.nominal_deceleration=read<double>(tree,"common/ego/nominal_deceleration");
    result.common.ego.nominal_positive_jerk=read<double>(tree,"common/ego/nominal_positive_jerk");
    if(!(result.common.ego.nominal_positive_jerk>0.0))throw std::invalid_argument("Parameter constraint: common/ego/nominal_positive_jerk");
    result.common.ego.nominal_negative_jerk=read<double>(tree,"common/ego/nominal_negative_jerk");
    result.common.blind_spot.lookahead_time=read<double>(tree,"common/blind_spot/lookahead_time");
    if(!(result.common.blind_spot.lookahead_time>0.0))throw std::invalid_argument("Parameter constraint: common/blind_spot/lookahead_time");
    result.common.blind_spot.metric=read<std::string>(tree,"common/blind_spot/metric");
    if(!(result.common.blind_spot.metric=="ttc" || result.common.blind_spot.metric=="rss"))throw std::invalid_argument("Parameter constraint: common/blind_spot/metric");
    result.common.blind_spot.check.front=read<bool>(tree,"common/blind_spot/check/front");
    result.common.blind_spot.check.left=read<bool>(tree,"common/blind_spot/check/left");
    result.common.blind_spot.check.right=read<bool>(tree,"common/blind_spot/check/right");
    result.common.blind_spot.check.yaw_th=read<double>(tree,"common/blind_spot/check/yaw_th");
    if(!(result.common.blind_spot.check.yaw_th>0.0))throw std::invalid_argument("Parameter constraint: common/blind_spot/check/yaw_th");
    result.common.blind_spot.offset.inner=read<double>(tree,"common/blind_spot/offset/inner");
    result.common.blind_spot.offset.outer=read<double>(tree,"common/blind_spot/offset/outer");
    result.common.blind_spot.participants.reaction_time=read<double>(tree,"common/blind_spot/participants/reaction_time");
    if(!(result.common.blind_spot.participants.reaction_time>0.0))throw std::invalid_argument("Parameter constraint: common/blind_spot/participants/reaction_time");
    result.common.blind_spot.participants.max_velocity=read<double>(tree,"common/blind_spot/participants/max_velocity");
    if(!(result.common.blind_spot.participants.max_velocity>0.0))throw std::invalid_argument("Parameter constraint: common/blind_spot/participants/max_velocity");
    result.common.blind_spot.participants.max_deceleration=read<double>(tree,"common/blind_spot/participants/max_deceleration");
    result.common.adjacent_lane.lookahead_time=read<double>(tree,"common/adjacent_lane/lookahead_time");
    if(!(result.common.adjacent_lane.lookahead_time>0.0))throw std::invalid_argument("Parameter constraint: common/adjacent_lane/lookahead_time");
    result.common.adjacent_lane.metric=read<std::string>(tree,"common/adjacent_lane/metric");
    if(!(result.common.adjacent_lane.metric=="ttc" || result.common.adjacent_lane.metric=="rss"))throw std::invalid_argument("Parameter constraint: common/adjacent_lane/metric");
    result.common.adjacent_lane.check.front=read<bool>(tree,"common/adjacent_lane/check/front");
    result.common.adjacent_lane.offset.left=read<double>(tree,"common/adjacent_lane/offset/left");
    result.common.adjacent_lane.offset.right=read<double>(tree,"common/adjacent_lane/offset/right");
    result.common.adjacent_lane.participants.reaction_time=read<double>(tree,"common/adjacent_lane/participants/reaction_time");
    if(!(result.common.adjacent_lane.participants.reaction_time>0.0))throw std::invalid_argument("Parameter constraint: common/adjacent_lane/participants/reaction_time");
    result.common.adjacent_lane.participants.max_velocity=read<double>(tree,"common/adjacent_lane/participants/max_velocity");
    if(!(result.common.adjacent_lane.participants.max_velocity>0.0))throw std::invalid_argument("Parameter constraint: common/adjacent_lane/participants/max_velocity");
    result.common.adjacent_lane.participants.max_deceleration=read<double>(tree,"common/adjacent_lane/participants/max_deceleration");
    return result;
  }
public:
  explicit ParamListener(ros::NodeHandle * node):node_(*node),cached_(snapshot()) {}
  Params get_params() {
    std::lock_guard<std::mutex> lock(mutex_);
    try { auto next=snapshot(); cached_=std::move(next); }
    catch(const std::exception & error) {
      ROS_ERROR_THROTTLE(1.0,"Validator parameter update rejected; retaining last valid snapshot: %s",error.what());
    }
    return cached_;
  }
};
}
