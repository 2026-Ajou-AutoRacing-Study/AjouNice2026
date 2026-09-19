#include <autoware/planning_validator_intersection_collision_checker/full_source/intersection_collision_checker_node_parameters.hpp>
#include <autoware/planning_validator_rear_collision_checker/full_source/rear_collision_checker_node_parameters.hpp>
#include <gtest/gtest.h>
#include <cstdlib>
#include <limits>
namespace full=autoware::planning_validator::full_source;
TEST(FullValidatorParameters,intersectionAllRequiredTypesAndRanges) {
  ros::NodeHandle node("/full_intersection_parameters");
  using Listener=full::intersection_collision_checker_node::ParamListener;
  ASSERT_NO_THROW({ Listener good(&node); });
  { SCOPED_TRACE("icc_parameters/enable"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/enable",saved));
    node.deleteParam("icc_parameters/enable"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/enable",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/enable",saved);
  }
  { SCOPED_TRACE("icc_parameters/detection_range"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/detection_range",saved));
    node.deleteParam("icc_parameters/detection_range"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/detection_range",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/detection_range",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/detection_range",saved);
  }
  { SCOPED_TRACE("icc_parameters/ttc_threshold"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/ttc_threshold",saved));
    node.deleteParam("icc_parameters/ttc_threshold"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/ttc_threshold",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/ttc_threshold",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/ttc_threshold",saved);
  }
  { SCOPED_TRACE("icc_parameters/ego_deceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/ego_deceleration",saved));
    node.deleteParam("icc_parameters/ego_deceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/ego_deceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/ego_deceleration",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/ego_deceleration",saved);
  }
  { SCOPED_TRACE("icc_parameters/min_time_horizon"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/min_time_horizon",saved));
    node.deleteParam("icc_parameters/min_time_horizon"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/min_time_horizon",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/min_time_horizon",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/min_time_horizon",saved);
  }
  { SCOPED_TRACE("icc_parameters/on_time_buffer"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/on_time_buffer",saved));
    node.deleteParam("icc_parameters/on_time_buffer"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/on_time_buffer",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/on_time_buffer",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/on_time_buffer",saved);
  }
  { SCOPED_TRACE("icc_parameters/off_time_buffer"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/off_time_buffer",saved));
    node.deleteParam("icc_parameters/off_time_buffer"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/off_time_buffer",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/off_time_buffer",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/off_time_buffer",saved);
  }
  { SCOPED_TRACE("icc_parameters/close_distance_th"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/close_distance_th",saved));
    node.deleteParam("icc_parameters/close_distance_th"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/close_distance_th",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/close_distance_th",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/close_distance_th",saved);
  }
  { SCOPED_TRACE("icc_parameters/close_time_th"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/close_time_th",saved));
    node.deleteParam("icc_parameters/close_time_th"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/close_time_th",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/close_time_th",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/close_time_th",saved);
  }
  { SCOPED_TRACE("icc_parameters/right_turn/enable"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/right_turn/enable",saved));
    node.deleteParam("icc_parameters/right_turn/enable"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/enable",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/enable",saved);
  }
  { SCOPED_TRACE("icc_parameters/right_turn/check_traffic_signal"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/right_turn/check_traffic_signal",saved));
    node.deleteParam("icc_parameters/right_turn/check_traffic_signal"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/check_traffic_signal",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/check_traffic_signal",saved);
  }
  { SCOPED_TRACE("icc_parameters/right_turn/check_crossing_lanes"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/right_turn/check_crossing_lanes",saved));
    node.deleteParam("icc_parameters/right_turn/check_crossing_lanes"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/check_crossing_lanes",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/check_crossing_lanes",saved);
  }
  { SCOPED_TRACE("icc_parameters/right_turn/check_turning_lanes"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/right_turn/check_turning_lanes",saved));
    node.deleteParam("icc_parameters/right_turn/check_turning_lanes"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/check_turning_lanes",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/check_turning_lanes",saved);
  }
  { SCOPED_TRACE("icc_parameters/right_turn/crossing_lane_angle_th"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/right_turn/crossing_lane_angle_th",saved));
    node.deleteParam("icc_parameters/right_turn/crossing_lane_angle_th"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/crossing_lane_angle_th",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/crossing_lane_angle_th",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/right_turn/crossing_lane_angle_th",saved);
  }
  { SCOPED_TRACE("icc_parameters/left_turn/enable"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/left_turn/enable",saved));
    node.deleteParam("icc_parameters/left_turn/enable"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/left_turn/enable",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/left_turn/enable",saved);
  }
  { SCOPED_TRACE("icc_parameters/left_turn/check_traffic_signal"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/left_turn/check_traffic_signal",saved));
    node.deleteParam("icc_parameters/left_turn/check_traffic_signal"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/left_turn/check_traffic_signal",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/left_turn/check_traffic_signal",saved);
  }
  { SCOPED_TRACE("icc_parameters/left_turn/check_turning_lanes"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/left_turn/check_turning_lanes",saved));
    node.deleteParam("icc_parameters/left_turn/check_turning_lanes"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/left_turn/check_turning_lanes",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/left_turn/check_turning_lanes",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/height_buffer"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/height_buffer",saved));
    node.deleteParam("icc_parameters/pointcloud/height_buffer"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/height_buffer",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/height_buffer",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/height_buffer",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/min_height"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/min_height",saved));
    node.deleteParam("icc_parameters/pointcloud/min_height"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/min_height",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/min_height",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/min_height",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/voxel_grid_filter/x"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/voxel_grid_filter/x",saved));
    node.deleteParam("icc_parameters/pointcloud/voxel_grid_filter/x"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/x",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/x",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/x",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/voxel_grid_filter/y"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/voxel_grid_filter/y",saved));
    node.deleteParam("icc_parameters/pointcloud/voxel_grid_filter/y"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/y",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/y",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/y",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/voxel_grid_filter/z"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/voxel_grid_filter/z",saved));
    node.deleteParam("icc_parameters/pointcloud/voxel_grid_filter/z"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/z",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/z",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/z",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/voxel_grid_filter/min_size"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/voxel_grid_filter/min_size",saved));
    node.deleteParam("icc_parameters/pointcloud/voxel_grid_filter/min_size"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/min_size",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/min_size",0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/voxel_grid_filter/min_size",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/clustering/tolerance"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/clustering/tolerance",saved));
    node.deleteParam("icc_parameters/pointcloud/clustering/tolerance"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/tolerance",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/tolerance",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/tolerance",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/clustering/min_height"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/clustering/min_height",saved));
    node.deleteParam("icc_parameters/pointcloud/clustering/min_height"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/min_height",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/min_height",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/min_height",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/clustering/min_size"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/clustering/min_size",saved));
    node.deleteParam("icc_parameters/pointcloud/clustering/min_size"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/min_size",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/min_size",0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/min_size",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/clustering/max_size"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/clustering/max_size",saved));
    node.deleteParam("icc_parameters/pointcloud/clustering/max_size"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/max_size",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/max_size",0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/clustering/max_size",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/velocity_estimation/max_acceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/velocity_estimation/max_acceleration",saved));
    node.deleteParam("icc_parameters/pointcloud/velocity_estimation/max_acceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_acceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_acceleration",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_acceleration",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/velocity_estimation/max_velocity"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/velocity_estimation/max_velocity",saved));
    node.deleteParam("icc_parameters/pointcloud/velocity_estimation/max_velocity"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_velocity",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_velocity",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_velocity",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/velocity_estimation/observation_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/velocity_estimation/observation_time",saved));
    node.deleteParam("icc_parameters/pointcloud/velocity_estimation/observation_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/observation_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/observation_time",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/observation_time",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/velocity_estimation/max_history_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/velocity_estimation/max_history_time",saved));
    node.deleteParam("icc_parameters/pointcloud/velocity_estimation/max_history_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_history_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_history_time",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/max_history_time",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/velocity_estimation/buffer_size"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/velocity_estimation/buffer_size",saved));
    node.deleteParam("icc_parameters/pointcloud/velocity_estimation/buffer_size"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/buffer_size",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/buffer_size",0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/velocity_estimation/buffer_size",saved);
  }
  { SCOPED_TRACE("icc_parameters/pointcloud/latency"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/pointcloud/latency",saved));
    node.deleteParam("icc_parameters/pointcloud/latency"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/latency",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/latency",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/pointcloud/latency",saved);
  }
  { SCOPED_TRACE("icc_parameters/filter/min_velocity"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/filter/min_velocity",saved));
    node.deleteParam("icc_parameters/filter/min_velocity"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/filter/min_velocity",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/filter/min_velocity",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/filter/min_velocity",saved);
  }
  { SCOPED_TRACE("icc_parameters/filter/moving_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("icc_parameters/filter/moving_time",saved));
    node.deleteParam("icc_parameters/filter/moving_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("icc_parameters/filter/moving_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("icc_parameters/filter/moving_time",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("icc_parameters/filter/moving_time",saved);
  }
  Listener listener(&node);
  const auto initial=listener.get_params();
  node.setParam("icc_parameters/detection_range",initial.icc_parameters.detection_range+1.0);
  EXPECT_DOUBLE_EQ(listener.get_params().icc_parameters.detection_range,initial.icc_parameters.detection_range+1.0);
  node.setParam("icc_parameters/detection_range",-10.0);
  EXPECT_DOUBLE_EQ(listener.get_params().icc_parameters.detection_range,initial.icc_parameters.detection_range+1.0);
  node.setParam("icc_parameters/detection_range",initial.icc_parameters.detection_range);
}
TEST(FullValidatorParameters,rearAllRequiredTypesAndRanges) {
  ros::NodeHandle node("/full_rear_parameters");
  using Listener=full::rear_collision_checker_node::ParamListener;
  ASSERT_NO_THROW({ Listener good(&node); });
  { SCOPED_TRACE("common/on_time_buffer"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/on_time_buffer",saved));
    node.deleteParam("common/on_time_buffer"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/on_time_buffer",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/on_time_buffer",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/on_time_buffer",saved);
  }
  { SCOPED_TRACE("common/off_time_buffer"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/off_time_buffer",saved));
    node.deleteParam("common/off_time_buffer"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/off_time_buffer",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/off_time_buffer",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/off_time_buffer",saved);
  }
  { SCOPED_TRACE("common/check_on_unstoppable"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/check_on_unstoppable",saved));
    node.deleteParam("common/check_on_unstoppable"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/check_on_unstoppable",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/check_on_unstoppable",saved);
  }
  { SCOPED_TRACE("common/pointcloud/range/dead_zone"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/range/dead_zone",saved));
    node.deleteParam("common/pointcloud/range/dead_zone"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/range/dead_zone",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/range/dead_zone",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/range/dead_zone",saved);
  }
  { SCOPED_TRACE("common/pointcloud/range/buffer"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/range/buffer",saved));
    node.deleteParam("common/pointcloud/range/buffer"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/range/buffer",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/range/buffer",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/range/buffer",saved);
  }
  { SCOPED_TRACE("common/pointcloud/crop_box_filter/x/max"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/crop_box_filter/x/max",saved));
    node.deleteParam("common/pointcloud/crop_box_filter/x/max"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/x/max",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/x/max",saved);
  }
  { SCOPED_TRACE("common/pointcloud/crop_box_filter/x/min"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/crop_box_filter/x/min",saved));
    node.deleteParam("common/pointcloud/crop_box_filter/x/min"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/x/min",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/x/min",saved);
  }
  { SCOPED_TRACE("common/pointcloud/crop_box_filter/z/max"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/crop_box_filter/z/max",saved));
    node.deleteParam("common/pointcloud/crop_box_filter/z/max"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/z/max",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/z/max",saved);
  }
  { SCOPED_TRACE("common/pointcloud/crop_box_filter/z/min"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/crop_box_filter/z/min",saved));
    node.deleteParam("common/pointcloud/crop_box_filter/z/min"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/z/min",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/crop_box_filter/z/min",saved);
  }
  { SCOPED_TRACE("common/pointcloud/voxel_grid_filter/x"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/voxel_grid_filter/x",saved));
    node.deleteParam("common/pointcloud/voxel_grid_filter/x"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/x",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/x",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/x",saved);
  }
  { SCOPED_TRACE("common/pointcloud/voxel_grid_filter/y"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/voxel_grid_filter/y",saved));
    node.deleteParam("common/pointcloud/voxel_grid_filter/y"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/y",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/y",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/y",saved);
  }
  { SCOPED_TRACE("common/pointcloud/voxel_grid_filter/z"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/voxel_grid_filter/z",saved));
    node.deleteParam("common/pointcloud/voxel_grid_filter/z"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/z",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/z",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/voxel_grid_filter/z",saved);
  }
  { SCOPED_TRACE("common/pointcloud/clustering/cluster_tolerance"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/clustering/cluster_tolerance",saved));
    node.deleteParam("common/pointcloud/clustering/cluster_tolerance"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/cluster_tolerance",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/cluster_tolerance",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/cluster_tolerance",saved);
  }
  { SCOPED_TRACE("common/pointcloud/clustering/min_cluster_height"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/clustering/min_cluster_height",saved));
    node.deleteParam("common/pointcloud/clustering/min_cluster_height"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/min_cluster_height",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/min_cluster_height",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/min_cluster_height",saved);
  }
  { SCOPED_TRACE("common/pointcloud/clustering/min_cluster_size"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/clustering/min_cluster_size",saved));
    node.deleteParam("common/pointcloud/clustering/min_cluster_size"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/min_cluster_size",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/min_cluster_size",0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/min_cluster_size",saved);
  }
  { SCOPED_TRACE("common/pointcloud/clustering/max_cluster_size"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/clustering/max_cluster_size",saved));
    node.deleteParam("common/pointcloud/clustering/max_cluster_size"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/max_cluster_size",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/max_cluster_size",0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/clustering/max_cluster_size",saved);
  }
  { SCOPED_TRACE("common/pointcloud/velocity_estimation/observation_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/velocity_estimation/observation_time",saved));
    node.deleteParam("common/pointcloud/velocity_estimation/observation_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/velocity_estimation/observation_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/velocity_estimation/observation_time",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/velocity_estimation/observation_time",saved);
  }
  { SCOPED_TRACE("common/pointcloud/velocity_estimation/max_acceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/velocity_estimation/max_acceleration",saved));
    node.deleteParam("common/pointcloud/velocity_estimation/max_acceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/velocity_estimation/max_acceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/velocity_estimation/max_acceleration",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/velocity_estimation/max_acceleration",saved);
  }
  { SCOPED_TRACE("common/pointcloud/latency"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/pointcloud/latency",saved));
    node.deleteParam("common/pointcloud/latency"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/pointcloud/latency",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/pointcloud/latency",-1.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/pointcloud/latency",saved);
  }
  { SCOPED_TRACE("common/filter/min_velocity"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/filter/min_velocity",saved));
    node.deleteParam("common/filter/min_velocity"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/filter/min_velocity",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/filter/min_velocity",saved);
  }
  { SCOPED_TRACE("common/filter/moving_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/filter/moving_time",saved));
    node.deleteParam("common/filter/moving_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/filter/moving_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/filter/moving_time",saved);
  }
  { SCOPED_TRACE("common/time_to_collision/margin"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/time_to_collision/margin",saved));
    node.deleteParam("common/time_to_collision/margin"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/time_to_collision/margin",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/time_to_collision/margin",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/time_to_collision/margin",saved);
  }
  { SCOPED_TRACE("common/ego/reaction_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/reaction_time",saved));
    node.deleteParam("common/ego/reaction_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/reaction_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/reaction_time",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/ego/reaction_time",saved);
  }
  { SCOPED_TRACE("common/ego/min_velocity"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/min_velocity",saved));
    node.deleteParam("common/ego/min_velocity"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/min_velocity",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/min_velocity",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/ego/min_velocity",saved);
  }
  { SCOPED_TRACE("common/ego/max_velocity"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/max_velocity",saved));
    node.deleteParam("common/ego/max_velocity"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/max_velocity",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/max_velocity",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/ego/max_velocity",saved);
  }
  { SCOPED_TRACE("common/ego/max_acceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/max_acceleration",saved));
    node.deleteParam("common/ego/max_acceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/max_acceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/max_acceleration",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/ego/max_acceleration",saved);
  }
  { SCOPED_TRACE("common/ego/max_deceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/max_deceleration",saved));
    node.deleteParam("common/ego/max_deceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/max_deceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/max_deceleration",saved);
  }
  { SCOPED_TRACE("common/ego/max_positive_jerk"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/max_positive_jerk",saved));
    node.deleteParam("common/ego/max_positive_jerk"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/max_positive_jerk",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/max_positive_jerk",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/ego/max_positive_jerk",saved);
  }
  { SCOPED_TRACE("common/ego/max_negative_jerk"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/max_negative_jerk",saved));
    node.deleteParam("common/ego/max_negative_jerk"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/max_negative_jerk",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/max_negative_jerk",saved);
  }
  { SCOPED_TRACE("common/ego/nominal_deceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/nominal_deceleration",saved));
    node.deleteParam("common/ego/nominal_deceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/nominal_deceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/nominal_deceleration",saved);
  }
  { SCOPED_TRACE("common/ego/nominal_positive_jerk"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/nominal_positive_jerk",saved));
    node.deleteParam("common/ego/nominal_positive_jerk"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/nominal_positive_jerk",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/nominal_positive_jerk",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/ego/nominal_positive_jerk",saved);
  }
  { SCOPED_TRACE("common/ego/nominal_negative_jerk"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/ego/nominal_negative_jerk",saved));
    node.deleteParam("common/ego/nominal_negative_jerk"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/ego/nominal_negative_jerk",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/ego/nominal_negative_jerk",saved);
  }
  { SCOPED_TRACE("common/blind_spot/lookahead_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/lookahead_time",saved));
    node.deleteParam("common/blind_spot/lookahead_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/lookahead_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/lookahead_time",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/blind_spot/lookahead_time",saved);
  }
  { SCOPED_TRACE("common/blind_spot/metric"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/metric",saved));
    node.deleteParam("common/blind_spot/metric"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/metric",123); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/metric",std::string("unknown metric")); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/blind_spot/metric",saved);
  }
  { SCOPED_TRACE("common/blind_spot/check/front"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/check/front",saved));
    node.deleteParam("common/blind_spot/check/front"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/check/front",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/check/front",saved);
  }
  { SCOPED_TRACE("common/blind_spot/check/left"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/check/left",saved));
    node.deleteParam("common/blind_spot/check/left"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/check/left",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/check/left",saved);
  }
  { SCOPED_TRACE("common/blind_spot/check/right"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/check/right",saved));
    node.deleteParam("common/blind_spot/check/right"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/check/right",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/check/right",saved);
  }
  { SCOPED_TRACE("common/blind_spot/check/yaw_th"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/check/yaw_th",saved));
    node.deleteParam("common/blind_spot/check/yaw_th"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/check/yaw_th",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/check/yaw_th",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/blind_spot/check/yaw_th",saved);
  }
  { SCOPED_TRACE("common/blind_spot/offset/inner"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/offset/inner",saved));
    node.deleteParam("common/blind_spot/offset/inner"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/offset/inner",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/offset/inner",saved);
  }
  { SCOPED_TRACE("common/blind_spot/offset/outer"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/offset/outer",saved));
    node.deleteParam("common/blind_spot/offset/outer"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/offset/outer",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/offset/outer",saved);
  }
  { SCOPED_TRACE("common/blind_spot/participants/reaction_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/participants/reaction_time",saved));
    node.deleteParam("common/blind_spot/participants/reaction_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/participants/reaction_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/participants/reaction_time",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/blind_spot/participants/reaction_time",saved);
  }
  { SCOPED_TRACE("common/blind_spot/participants/max_velocity"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/participants/max_velocity",saved));
    node.deleteParam("common/blind_spot/participants/max_velocity"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/participants/max_velocity",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/participants/max_velocity",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/blind_spot/participants/max_velocity",saved);
  }
  { SCOPED_TRACE("common/blind_spot/participants/max_deceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/blind_spot/participants/max_deceleration",saved));
    node.deleteParam("common/blind_spot/participants/max_deceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/blind_spot/participants/max_deceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/blind_spot/participants/max_deceleration",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/lookahead_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/lookahead_time",saved));
    node.deleteParam("common/adjacent_lane/lookahead_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/lookahead_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/lookahead_time",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/adjacent_lane/lookahead_time",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/metric"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/metric",saved));
    node.deleteParam("common/adjacent_lane/metric"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/metric",123); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/metric",std::string("unknown metric")); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/adjacent_lane/metric",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/check/front"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/check/front",saved));
    node.deleteParam("common/adjacent_lane/check/front"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/check/front",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/check/front",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/offset/left"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/offset/left",saved));
    node.deleteParam("common/adjacent_lane/offset/left"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/offset/left",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/offset/left",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/offset/right"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/offset/right",saved));
    node.deleteParam("common/adjacent_lane/offset/right"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/offset/right",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/offset/right",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/participants/reaction_time"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/participants/reaction_time",saved));
    node.deleteParam("common/adjacent_lane/participants/reaction_time"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/reaction_time",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/reaction_time",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/reaction_time",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/participants/max_velocity"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/participants/max_velocity",saved));
    node.deleteParam("common/adjacent_lane/participants/max_velocity"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/max_velocity",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/max_velocity",0.0); EXPECT_THROW({Listener outside(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/max_velocity",saved);
  }
  { SCOPED_TRACE("common/adjacent_lane/participants/max_deceleration"); XmlRpc::XmlRpcValue saved; ASSERT_TRUE(node.getParam("common/adjacent_lane/participants/max_deceleration",saved));
    node.deleteParam("common/adjacent_lane/participants/max_deceleration"); EXPECT_THROW({Listener missing(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/max_deceleration",std::string("invalid type")); EXPECT_THROW({Listener wrong(&node);},std::exception);
    node.setParam("common/adjacent_lane/participants/max_deceleration",saved);
  }
  Listener listener(&node);
  const auto initial=listener.get_params();
  node.setParam("common/on_time_buffer",initial.common.on_time_buffer+1.0);
  EXPECT_DOUBLE_EQ(listener.get_params().common.on_time_buffer,initial.common.on_time_buffer+1.0);
  node.setParam("common/on_time_buffer",-10.0);
  EXPECT_DOUBLE_EQ(listener.get_params().common.on_time_buffer,initial.common.on_time_buffer+1.0);
  node.setParam("common/on_time_buffer",initial.common.on_time_buffer);
}
int main(int argc,char **argv){
  ros::init(argc,argv,"full_validator_parameter_tests");
  if(std::system("rosparam load \"$(rospack find autoware_planning_validator_intersection_collision_checker)/config/ros1/intersection_collision_checker.yaml\" /full_intersection_parameters")!=0)return 1;
  if(std::system("rosparam load \"$(rospack find autoware_planning_validator_rear_collision_checker)/config/ros1/rear_collision_checker.yaml\" /full_rear_parameters")!=0)return 1;
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
