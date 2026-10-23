// Fresh-process replay of the unchanged production EKF core, no ROS node or
// scheduler. Node-side yaw canonicalization/TF and policy are separate scopes.
#include "algorithm/ekf_multi_object_tracking.hpp"
#include <iomanip>
#include <iostream>
#include <memory>

int main() {
  ros::Time::init();
  MultiClassObjectTrackingConfig config;
  config.global_coord_track = true;
  config.prediction_model = mc_mot::PredictionModel::CTRV;
  config.max_association_dist_m = 5.0;
  config.pedestrian_gaussian_association_ranking = true;
  config.time_aware_track_lifecycle = true;
  config.confirmed_track_max_coast_time_sec = .50;
  config.confirmed_stationary_track_max_coast_time_sec = 1.00;
  config.lifecycle_stationary_speed_threshold_mps = 2.0;
  config.suppress_duplicate_track_birth = true;
  config.duplicate_birth_suppression_distance_m = .75;
  config.duplicate_birth_minimum_max_dimension_m = 1.0;
  config.duplicate_birth_maximum_dimension_ratio = 2.0;
  config.system_noise_std_xy_m = .01;
  config.system_noise_std_yaw_deg = .5;
  config.system_noise_std_vx_vy_ms = 1.0;
  config.system_noise_std_yaw_rate_degs = 1.0;
  config.system_noise_std_ax_ay_ms2 = .6;
  config.meas_noise_std_xy_m = .4;
  config.meas_noise_std_yaw_deg = 10.0;
  config.detection_velocity_fusion_mode = 0;
  config.center_motion_velocity_fusion = false;
  config.dimension_filter_alpha = .3;
  config.use_kinematic_model = false;
  config.use_yaw_rate_filtering = true;
  auto tracker = std::make_unique<EkfMultiObjectTracking>(config);
  double timestamp = 0., previous = 0.;
  int count = 0;
  std::cout << std::setprecision(17);
  while (std::cin >> timestamp >> count) {
    if (count < 0 || count > MAX_TRACKS || (previous && timestamp <= previous)) return 2;
    mc_mot::Meastructs measurements;
    measurements.time_stamp = timestamp;
    for (int i = 0; i < count; ++i) {
      int label;
      mc_mot::Meastruct d;
      d.state = {};
      if (!(std::cin >> label >> d.state.x >> d.state.y >> d.state.z >> d.state.yaw
            >> d.dimension.length >> d.dimension.width >> d.dimension.height >> d.detection_confidence)) return 3;
      if (label < 0 || label > 2) return 4;
      d.classification = label == 0 ? mc_mot::ObjectClass::CAR :
          label == 1 ? mc_mot::ObjectClass::PEDESTRIAN : mc_mot::ObjectClass::UNKNOWN;
      d.state.time_stamp = timestamp;
      measurements.meas.push_back(d);
    }
    if (previous) tracker->RunPrediction(timestamp - previous);
    tracker->RunUpdate(measurements);
    const auto result = tracker->GetTrackResults();
    int initialized = 0;
    for (const auto& track : result.track) if (track.is_init) ++initialized;
    std::cout << "FRAME " << timestamp << " " << initialized << "\n";
    for (const auto& track : result.track) {
      if (!track.is_init) continue;
      std::cout << "TRACK " << track.track_id << " " << track.getRepClass() << " "
          << track.is_confirmed << " " << track.is_associated << " "
          << track.state_vec(S_X) << " " << track.state_vec(S_Y) << " " << track.object_z << " "
          << track.state_vec(S_YAW) << " " << track.state_vec(S_VX) << " " << track.state_vec(S_VY) << " "
          << track.state_vec(S_YAW_RATE) << " "
          << track.dimension.length << " " << track.dimension.width << " " << track.dimension.height << " "
          << track.detection_confidence << " " << track.age << " "
          << track.last_measurement_time << " " << track.initialization_time << "\n";
    }
    previous = timestamp;
  }
}
