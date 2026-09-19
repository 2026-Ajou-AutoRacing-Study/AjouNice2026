// Generated from the retained original schema. Do not relax validation here.
#pragma once
#include <autoware/planning_validator/full_source/native_support.hpp>
#include <XmlRpcValue.h>
#include <mutex>
#include <type_traits>
namespace autoware::planning_validator::full_source::intersection_collision_checker_node {
struct Params {
struct IccParameters {
  bool enable{};
  double detection_range{};
  double ttc_threshold{};
  double ego_deceleration{};
  double min_time_horizon{};
  double on_time_buffer{};
  double off_time_buffer{};
  double close_distance_th{};
  double close_time_th{};
struct RightTurn {
  bool enable{};
  bool check_traffic_signal{};
  bool check_crossing_lanes{};
  bool check_turning_lanes{};
  double crossing_lane_angle_th{};
} right_turn;
struct LeftTurn {
  bool enable{};
  bool check_traffic_signal{};
  bool check_turning_lanes{};
} left_turn;
struct Pointcloud {
  double height_buffer{};
  double min_height{};
struct VoxelGridFilter {
  double x{};
  double y{};
  double z{};
  int min_size{};
} voxel_grid_filter;
struct Clustering {
  double tolerance{};
  double min_height{};
  int min_size{};
  int max_size{};
} clustering;
struct VelocityEstimation {
  double max_acceleration{};
  double max_velocity{};
  double observation_time{};
  double max_history_time{};
  int buffer_size{};
} velocity_estimation;
  double latency{};
} pointcloud;
struct Filter {
  double min_velocity{};
  double moving_time{};
} filter;
} icc_parameters;
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
    result.icc_parameters.enable=read<bool>(tree,"icc_parameters/enable");
    result.icc_parameters.detection_range=read<double>(tree,"icc_parameters/detection_range");
    if(!(result.icc_parameters.detection_range>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/detection_range");
    result.icc_parameters.ttc_threshold=read<double>(tree,"icc_parameters/ttc_threshold");
    if(!(result.icc_parameters.ttc_threshold>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/ttc_threshold");
    result.icc_parameters.ego_deceleration=read<double>(tree,"icc_parameters/ego_deceleration");
    if(!(result.icc_parameters.ego_deceleration>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/ego_deceleration");
    result.icc_parameters.min_time_horizon=read<double>(tree,"icc_parameters/min_time_horizon");
    if(!(result.icc_parameters.min_time_horizon>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/min_time_horizon");
    result.icc_parameters.on_time_buffer=read<double>(tree,"icc_parameters/on_time_buffer");
    if(!(result.icc_parameters.on_time_buffer>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/on_time_buffer");
    result.icc_parameters.off_time_buffer=read<double>(tree,"icc_parameters/off_time_buffer");
    if(!(result.icc_parameters.off_time_buffer>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/off_time_buffer");
    result.icc_parameters.close_distance_th=read<double>(tree,"icc_parameters/close_distance_th");
    if(!(result.icc_parameters.close_distance_th>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/close_distance_th");
    result.icc_parameters.close_time_th=read<double>(tree,"icc_parameters/close_time_th");
    if(!(result.icc_parameters.close_time_th>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/close_time_th");
    result.icc_parameters.right_turn.enable=read<bool>(tree,"icc_parameters/right_turn/enable");
    result.icc_parameters.right_turn.check_traffic_signal=read<bool>(tree,"icc_parameters/right_turn/check_traffic_signal");
    result.icc_parameters.right_turn.check_crossing_lanes=read<bool>(tree,"icc_parameters/right_turn/check_crossing_lanes");
    result.icc_parameters.right_turn.check_turning_lanes=read<bool>(tree,"icc_parameters/right_turn/check_turning_lanes");
    result.icc_parameters.right_turn.crossing_lane_angle_th=read<double>(tree,"icc_parameters/right_turn/crossing_lane_angle_th");
    if(!(result.icc_parameters.right_turn.crossing_lane_angle_th>0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/right_turn/crossing_lane_angle_th");
    result.icc_parameters.left_turn.enable=read<bool>(tree,"icc_parameters/left_turn/enable");
    result.icc_parameters.left_turn.check_traffic_signal=read<bool>(tree,"icc_parameters/left_turn/check_traffic_signal");
    result.icc_parameters.left_turn.check_turning_lanes=read<bool>(tree,"icc_parameters/left_turn/check_turning_lanes");
    result.icc_parameters.pointcloud.height_buffer=read<double>(tree,"icc_parameters/pointcloud/height_buffer");
    if(!(result.icc_parameters.pointcloud.height_buffer>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/height_buffer");
    result.icc_parameters.pointcloud.min_height=read<double>(tree,"icc_parameters/pointcloud/min_height");
    if(!(result.icc_parameters.pointcloud.min_height>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/min_height");
    result.icc_parameters.pointcloud.voxel_grid_filter.x=read<double>(tree,"icc_parameters/pointcloud/voxel_grid_filter/x");
    if(!(result.icc_parameters.pointcloud.voxel_grid_filter.x>0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/voxel_grid_filter/x");
    result.icc_parameters.pointcloud.voxel_grid_filter.y=read<double>(tree,"icc_parameters/pointcloud/voxel_grid_filter/y");
    if(!(result.icc_parameters.pointcloud.voxel_grid_filter.y>0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/voxel_grid_filter/y");
    result.icc_parameters.pointcloud.voxel_grid_filter.z=read<double>(tree,"icc_parameters/pointcloud/voxel_grid_filter/z");
    if(!(result.icc_parameters.pointcloud.voxel_grid_filter.z>0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/voxel_grid_filter/z");
    result.icc_parameters.pointcloud.voxel_grid_filter.min_size=read<int>(tree,"icc_parameters/pointcloud/voxel_grid_filter/min_size");
    if(!(result.icc_parameters.pointcloud.voxel_grid_filter.min_size>=1 && result.icc_parameters.pointcloud.voxel_grid_filter.min_size<=2147483647))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/voxel_grid_filter/min_size");
    result.icc_parameters.pointcloud.clustering.tolerance=read<double>(tree,"icc_parameters/pointcloud/clustering/tolerance");
    if(!(result.icc_parameters.pointcloud.clustering.tolerance>0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/clustering/tolerance");
    result.icc_parameters.pointcloud.clustering.min_height=read<double>(tree,"icc_parameters/pointcloud/clustering/min_height");
    if(!(result.icc_parameters.pointcloud.clustering.min_height>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/clustering/min_height");
    result.icc_parameters.pointcloud.clustering.min_size=read<int>(tree,"icc_parameters/pointcloud/clustering/min_size");
    if(!(result.icc_parameters.pointcloud.clustering.min_size>=1 && result.icc_parameters.pointcloud.clustering.min_size<=2147483647))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/clustering/min_size");
    result.icc_parameters.pointcloud.clustering.max_size=read<int>(tree,"icc_parameters/pointcloud/clustering/max_size");
    if(!(result.icc_parameters.pointcloud.clustering.max_size>=1 && result.icc_parameters.pointcloud.clustering.max_size<=2147483647))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/clustering/max_size");
    result.icc_parameters.pointcloud.velocity_estimation.max_acceleration=read<double>(tree,"icc_parameters/pointcloud/velocity_estimation/max_acceleration");
    if(!(result.icc_parameters.pointcloud.velocity_estimation.max_acceleration>0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/velocity_estimation/max_acceleration");
    result.icc_parameters.pointcloud.velocity_estimation.max_velocity=read<double>(tree,"icc_parameters/pointcloud/velocity_estimation/max_velocity");
    if(!(result.icc_parameters.pointcloud.velocity_estimation.max_velocity>0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/velocity_estimation/max_velocity");
    result.icc_parameters.pointcloud.velocity_estimation.observation_time=read<double>(tree,"icc_parameters/pointcloud/velocity_estimation/observation_time");
    if(!(result.icc_parameters.pointcloud.velocity_estimation.observation_time>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/velocity_estimation/observation_time");
    result.icc_parameters.pointcloud.velocity_estimation.max_history_time=read<double>(tree,"icc_parameters/pointcloud/velocity_estimation/max_history_time");
    if(!(result.icc_parameters.pointcloud.velocity_estimation.max_history_time>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/velocity_estimation/max_history_time");
    result.icc_parameters.pointcloud.velocity_estimation.buffer_size=read<int>(tree,"icc_parameters/pointcloud/velocity_estimation/buffer_size");
    if(!(result.icc_parameters.pointcloud.velocity_estimation.buffer_size>=1 && result.icc_parameters.pointcloud.velocity_estimation.buffer_size<=2147483647))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/velocity_estimation/buffer_size");
    result.icc_parameters.pointcloud.latency=read<double>(tree,"icc_parameters/pointcloud/latency");
    if(!(result.icc_parameters.pointcloud.latency>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/pointcloud/latency");
    result.icc_parameters.filter.min_velocity=read<double>(tree,"icc_parameters/filter/min_velocity");
    if(!(result.icc_parameters.filter.min_velocity>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/filter/min_velocity");
    result.icc_parameters.filter.moving_time=read<double>(tree,"icc_parameters/filter/moving_time");
    if(!(result.icc_parameters.filter.moving_time>=0.0))throw std::invalid_argument("Parameter constraint: icc_parameters/filter/moving_time");
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
