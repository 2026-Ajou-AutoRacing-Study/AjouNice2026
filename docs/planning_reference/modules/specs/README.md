> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

# Planning Package Specs

One source-derived summary per release-workspace planning package.

| Package | Package path | Spec |
|---|---|---|
| `autoware_core_planning` | `core/autoware_core/planning/autoware_core_planning` | [`core_planning.md`](core_planning.md) |
| `autoware_mission_planner` | `core/autoware_core/planning/autoware_mission_planner` | [`mission_planner.md`](mission_planner.md) |
| `autoware_objects_of_interest_marker_interface` | `core/autoware_core/planning/autoware_objects_of_interest_marker_interface` | [`objects_of_interest_marker_interface.md`](objects_of_interest_marker_interface.md) |
| `autoware_path_generator` | `core/autoware_core/planning/autoware_path_generator` | [`path_generator.md`](path_generator.md) |
| `autoware_planning_factor_interface` | `core/autoware_core/planning/autoware_planning_factor_interface` | [`planning_factor_interface.md`](planning_factor_interface.md) |
| `autoware_planning_topic_converter` | `core/autoware_core/planning/autoware_planning_topic_converter` | [`planning_topic_converter.md`](planning_topic_converter.md) |
| `autoware_route_handler` | `core/autoware_core/planning/autoware_route_handler` | [`route_handler.md`](route_handler.md) |
| `autoware_velocity_smoother` | `core/autoware_core/planning/autoware_velocity_smoother` | [`velocity_smoother.md`](velocity_smoother.md) |
| `autoware_behavior_velocity_planner` | `core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner` | [`behavior_velocity_planner.md`](behavior_velocity_planner.md) |
| `autoware_behavior_velocity_planner_common` | `core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_planner_common` | [`behavior_velocity_planner_common.md`](behavior_velocity_planner_common.md) |
| `autoware_behavior_velocity_stop_line_module` | `core/autoware_core/planning/behavior_velocity_planner/autoware_behavior_velocity_stop_line_module` | [`behavior_velocity_stop_line_module.md`](behavior_velocity_stop_line_module.md) |
| `autoware_motion_velocity_obstacle_stop_module` | `core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module` | [`motion_velocity_obstacle_stop_module.md`](motion_velocity_obstacle_stop_module.md) |
| `autoware_motion_velocity_planner` | `core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner` | [`motion_velocity_planner.md`](motion_velocity_planner.md) |
| `autoware_motion_velocity_planner_common` | `core/autoware_core/planning/motion_velocity_planner/autoware_motion_velocity_planner_common` | [`motion_velocity_planner_common.md`](motion_velocity_planner_common.md) |
| `autoware_planning_test_manager` | `core/autoware_core/testing/autoware_planning_test_manager` | [`planning_test_manager.md`](planning_test_manager.md) |
| `autoware_internal_planning_msgs` | `core/autoware_internal_msgs/autoware_internal_planning_msgs` | [`internal_planning_msgs.md`](internal_planning_msgs.md) |
| `autoware_planning_msgs` | `core/autoware_msgs/autoware_planning_msgs` | [`planning_msgs.md`](planning_msgs.md) |
| `autoware_planning_rviz_plugin` | `core/autoware_rviz_plugins/autoware_planning_rviz_plugin` | [`planning_rviz_plugin.md`](planning_rviz_plugin.md) |
| `tier4_planning_launch` | `launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch` | [`tier4_planning_launch.md`](tier4_planning_launch.md) |
| `autoware_planning_data_analyzer` | `tools/planning/autoware_planning_data_analyzer` | [`planning_data_analyzer.md`](planning_data_analyzer.md) |
| `autoware_route_client` | `tools/planning/autoware_route_client` | [`route_client.md`](route_client.md) |
| `autoware_route_history` | `tools/planning/autoware_route_history` | [`route_history.md`](route_history.md) |
| `autoware_rtc_replayer` | `tools/planning/autoware_rtc_replayer` | [`rtc_replayer.md`](rtc_replayer.md) |
| `autoware_static_centerline_generator` | `tools/planning/autoware_static_centerline_generator` | [`static_centerline_generator.md`](static_centerline_generator.md) |
| `planning_debug_tools` | `tools/planning/planning_debug_tools` | [`planning_debug_tools.md`](planning_debug_tools.md) |
| `autoware_planning_evaluator` | `universe/autoware_universe/evaluator/autoware_planning_evaluator` | [`planning_evaluator.md`](planning_evaluator.md) |
| `autoware_costmap_generator` | `universe/autoware_universe/planning/autoware_costmap_generator` | [`costmap_generator.md`](costmap_generator.md) |
| `autoware_diffusion_planner` | `universe/autoware_universe/planning/autoware_diffusion_planner` | [`diffusion_planner.md`](diffusion_planner.md) |
| `autoware_external_velocity_limit_selector` | `universe/autoware_universe/planning/autoware_external_velocity_limit_selector` | [`external_velocity_limit_selector.md`](external_velocity_limit_selector.md) |
| `autoware_freespace_planner` | `universe/autoware_universe/planning/autoware_freespace_planner` | [`freespace_planner.md`](freespace_planner.md) |
| `autoware_freespace_planning_algorithms` | `universe/autoware_universe/planning/autoware_freespace_planning_algorithms` | [`freespace_planning_algorithms.md`](freespace_planning_algorithms.md) |
| `autoware_hazard_lights_selector` | `universe/autoware_universe/planning/autoware_hazard_lights_selector` | [`hazard_lights_selector.md`](hazard_lights_selector.md) |
| `autoware_manual_lane_change_handler` | `universe/autoware_universe/planning/autoware_manual_lane_change_handler` | [`manual_lane_change_handler.md`](manual_lane_change_handler.md) |
| `autoware_mission_planner_universe` | `universe/autoware_universe/planning/autoware_mission_planner_universe` | [`mission_planner_universe.md`](mission_planner_universe.md) |
| `autoware_path_optimizer` | `universe/autoware_universe/planning/autoware_path_optimizer` | [`path_optimizer.md`](path_optimizer.md) |
| `autoware_path_smoother` | `universe/autoware_universe/planning/autoware_path_smoother` | [`path_smoother.md`](path_smoother.md) |
| `autoware_remaining_distance_time_calculator` | `universe/autoware_universe/planning/autoware_remaining_distance_time_calculator` | [`remaining_distance_time_calculator.md`](remaining_distance_time_calculator.md) |
| `autoware_rtc_interface` | `universe/autoware_universe/planning/autoware_rtc_interface` | [`rtc_interface.md`](rtc_interface.md) |
| `autoware_scenario_selector` | `universe/autoware_universe/planning/autoware_scenario_selector` | [`scenario_selector.md`](scenario_selector.md) |
| `autoware_surround_obstacle_checker` | `universe/autoware_universe/planning/autoware_surround_obstacle_checker` | [`surround_obstacle_checker.md`](surround_obstacle_checker.md) |
| `autoware_trajectory_adapter` | `universe/autoware_universe/planning/autoware_trajectory_adapter` | [`trajectory_adapter.md`](trajectory_adapter.md) |
| `autoware_trajectory_concatenator` | `universe/autoware_universe/planning/autoware_trajectory_concatenator` | [`trajectory_concatenator.md`](trajectory_concatenator.md) |
| `autoware_trajectory_modifier` | `universe/autoware_universe/planning/autoware_trajectory_modifier` | [`trajectory_modifier.md`](trajectory_modifier.md) |
| `autoware_trajectory_optimizer` | `universe/autoware_universe/planning/autoware_trajectory_optimizer` | [`trajectory_optimizer.md`](trajectory_optimizer.md) |
| `autoware_trajectory_ranker` | `universe/autoware_universe/planning/autoware_trajectory_ranker` | [`trajectory_ranker.md`](trajectory_ranker.md) |
| `autoware_trajectory_validator` | `universe/autoware_universe/planning/autoware_trajectory_validator` | [`trajectory_validator.md`](trajectory_validator.md) |
| `autoware_behavior_path_avoidance_by_lane_change_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_avoidance_by_lane_change_module` | [`behavior_path_avoidance_by_lane_change_module.md`](behavior_path_avoidance_by_lane_change_module.md) |
| `autoware_behavior_path_bidirectional_traffic_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_bidirectional_traffic_module` | [`behavior_path_bidirectional_traffic_module.md`](behavior_path_bidirectional_traffic_module.md) |
| `autoware_behavior_path_dynamic_obstacle_avoidance_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module` | [`behavior_path_dynamic_obstacle_avoidance_module.md`](behavior_path_dynamic_obstacle_avoidance_module.md) |
| `autoware_behavior_path_external_request_lane_change_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_external_request_lane_change_module` | [`behavior_path_external_request_lane_change_module.md`](behavior_path_external_request_lane_change_module.md) |
| `autoware_behavior_path_goal_planner_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_goal_planner_module` | [`behavior_path_goal_planner_module.md`](behavior_path_goal_planner_module.md) |
| `autoware_behavior_path_lane_change_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_lane_change_module` | [`behavior_path_lane_change_module.md`](behavior_path_lane_change_module.md) |
| `autoware_behavior_path_planner` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner` | [`behavior_path_planner.md`](behavior_path_planner.md) |
| `autoware_behavior_path_planner_common` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common` | [`behavior_path_planner_common.md`](behavior_path_planner_common.md) |
| `autoware_behavior_path_sampling_planner_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_sampling_planner_module` | [`behavior_path_sampling_planner_module.md`](behavior_path_sampling_planner_module.md) |
| `autoware_behavior_path_side_shift_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_side_shift_module` | [`behavior_path_side_shift_module.md`](behavior_path_side_shift_module.md) |
| `autoware_behavior_path_start_planner_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_start_planner_module` | [`behavior_path_start_planner_module.md`](behavior_path_start_planner_module.md) |
| `autoware_behavior_path_static_obstacle_avoidance_module` | `universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module` | [`behavior_path_static_obstacle_avoidance_module.md`](behavior_path_static_obstacle_avoidance_module.md) |
| `autoware_behavior_velocity_blind_spot_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module` | [`behavior_velocity_blind_spot_module.md`](behavior_velocity_blind_spot_module.md) |
| `autoware_behavior_velocity_crosswalk_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_crosswalk_module` | [`behavior_velocity_crosswalk_module.md`](behavior_velocity_crosswalk_module.md) |
| `autoware_behavior_velocity_detection_area_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_detection_area_module` | [`behavior_velocity_detection_area_module.md`](behavior_velocity_detection_area_module.md) |
| `autoware_behavior_velocity_intersection_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_intersection_module` | [`behavior_velocity_intersection_module.md`](behavior_velocity_intersection_module.md) |
| `autoware_behavior_velocity_no_drivable_lane_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_drivable_lane_module` | [`behavior_velocity_no_drivable_lane_module.md`](behavior_velocity_no_drivable_lane_module.md) |
| `autoware_behavior_velocity_no_stopping_area_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_no_stopping_area_module` | [`behavior_velocity_no_stopping_area_module.md`](behavior_velocity_no_stopping_area_module.md) |
| `autoware_behavior_velocity_occlusion_spot_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_occlusion_spot_module` | [`behavior_velocity_occlusion_spot_module.md`](behavior_velocity_occlusion_spot_module.md) |
| `autoware_behavior_velocity_roundabout_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_roundabout_module` | [`behavior_velocity_roundabout_module.md`](behavior_velocity_roundabout_module.md) |
| `autoware_behavior_velocity_rtc_interface` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_rtc_interface` | [`behavior_velocity_rtc_interface.md`](behavior_velocity_rtc_interface.md) |
| `autoware_behavior_velocity_speed_bump_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module` | [`behavior_velocity_speed_bump_module.md`](behavior_velocity_speed_bump_module.md) |
| `autoware_behavior_velocity_template_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_template_module` | [`behavior_velocity_template_module.md`](behavior_velocity_template_module.md) |
| `autoware_behavior_velocity_traffic_light_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_traffic_light_module` | [`behavior_velocity_traffic_light_module.md`](behavior_velocity_traffic_light_module.md) |
| `autoware_behavior_velocity_virtual_traffic_light_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_virtual_traffic_light_module` | [`behavior_velocity_virtual_traffic_light_module.md`](behavior_velocity_virtual_traffic_light_module.md) |
| `autoware_behavior_velocity_walkway_module` | `universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_walkway_module` | [`behavior_velocity_walkway_module.md`](behavior_velocity_walkway_module.md) |
| `autoware_motion_velocity_boundary_departure_prevention_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_boundary_departure_prevention_module` | [`motion_velocity_boundary_departure_prevention_module.md`](motion_velocity_boundary_departure_prevention_module.md) |
| `autoware_motion_velocity_dynamic_obstacle_stop_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_dynamic_obstacle_stop_module` | [`motion_velocity_dynamic_obstacle_stop_module.md`](motion_velocity_dynamic_obstacle_stop_module.md) |
| `autoware_motion_velocity_obstacle_cruise_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module` | [`motion_velocity_obstacle_cruise_module.md`](motion_velocity_obstacle_cruise_module.md) |
| `autoware_motion_velocity_obstacle_slow_down_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_slow_down_module` | [`motion_velocity_obstacle_slow_down_module.md`](motion_velocity_obstacle_slow_down_module.md) |
| `autoware_motion_velocity_obstacle_velocity_limiter_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_velocity_limiter_module` | [`motion_velocity_obstacle_velocity_limiter_module.md`](motion_velocity_obstacle_velocity_limiter_module.md) |
| `autoware_motion_velocity_out_of_lane_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_out_of_lane_module` | [`motion_velocity_out_of_lane_module.md`](motion_velocity_out_of_lane_module.md) |
| `autoware_motion_velocity_road_user_stop_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_road_user_stop_module` | [`motion_velocity_road_user_stop_module.md`](motion_velocity_road_user_stop_module.md) |
| `autoware_motion_velocity_run_out_module` | `universe/autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_run_out_module` | [`motion_velocity_run_out_module.md`](motion_velocity_run_out_module.md) |
| `autoware_planning_validator` | `universe/autoware_universe/planning/planning_validator/autoware_planning_validator` | [`planning_validator.md`](planning_validator.md) |
| `autoware_planning_validator_intersection_collision_checker` | `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_intersection_collision_checker` | [`planning_validator_intersection_collision_checker.md`](planning_validator_intersection_collision_checker.md) |
| `autoware_planning_validator_latency_checker` | `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_latency_checker` | [`planning_validator_latency_checker.md`](planning_validator_latency_checker.md) |
| `autoware_planning_validator_rear_collision_checker` | `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_rear_collision_checker` | [`planning_validator_rear_collision_checker.md`](planning_validator_rear_collision_checker.md) |
| `autoware_planning_validator_test_utils` | `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_test_utils` | [`planning_validator_test_utils.md`](planning_validator_test_utils.md) |
| `autoware_planning_validator_trajectory_checker` | `universe/autoware_universe/planning/planning_validator/autoware_planning_validator_trajectory_checker` | [`planning_validator_trajectory_checker.md`](planning_validator_trajectory_checker.md) |
| `autoware_bezier_sampler` | `universe/autoware_universe/planning/sampling_based_planner/autoware_bezier_sampler` | [`bezier_sampler.md`](bezier_sampler.md) |
| `autoware_frenet_planner` | `universe/autoware_universe/planning/sampling_based_planner/autoware_frenet_planner` | [`frenet_planner.md`](frenet_planner.md) |
| `autoware_path_sampler` | `universe/autoware_universe/planning/sampling_based_planner/autoware_path_sampler` | [`path_sampler.md`](path_sampler.md) |
| `autoware_sampler_common` | `universe/autoware_universe/planning/sampling_based_planner/autoware_sampler_common` | [`sampler_common.md`](sampler_common.md) |
| `autoware_simple_planning_simulator` | `universe/autoware_universe/simulator/autoware_simple_planning_simulator` | [`simple_planning_simulator.md`](simple_planning_simulator.md) |
| `tier4_planning_factor_rviz_plugin` | `universe/autoware_universe/visualization/tier4_planning_factor_rviz_plugin` | [`tier4_planning_factor_rviz_plugin.md`](tier4_planning_factor_rviz_plugin.md) |
| `tier4_planning_msgs` | `universe/external/tier4_autoware_msgs/tier4_planning_msgs` | [`tier4_planning_msgs.md`](tier4_planning_msgs.md) |
