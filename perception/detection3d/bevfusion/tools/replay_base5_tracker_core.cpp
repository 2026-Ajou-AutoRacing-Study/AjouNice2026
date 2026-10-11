// Offline numerical replay of the real EKF implementation. No ROS master,
// publishers, TF, node scheduler or planning certification. Linked source is
// untouched; measurements are transformed to the same world frame by Python.
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
  config.system_noise_std_xy_m = .01;
  config.system_noise_std_yaw_deg = .5;
  config.system_noise_std_vx_vy_ms = 1.0;
  config.system_noise_std_yaw_rate_degs = 1.0;
  config.system_noise_std_ax_ay_ms2 = .6;
  config.meas_noise_std_xy_m = .4;
  config.meas_noise_std_yaw_deg = 10.0;
  config.detection_velocity_fusion_mode = 0;
  auto tracker = std::make_unique<EkfMultiObjectTracking>(config);
  double timestamp = 0.0, previous = 0.0;
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
    if (previous) tracker->RunPrediction(timestamp-previous);
    tracker->RunUpdate(measurements);
    const auto result = tracker->GetTrackResults();
    int initialized = 0;
    for (const auto& track : result.track) if (track.is_init) ++initialized;
    std::cout << "FRAME " << timestamp << " " << initialized << "\n";
    for (const auto& track : result.track) {
      if (!track.is_init) continue;
      std::cout << "TRACK " << track.track_id << " " << track.getRepClass() << " "
          << track.is_confirmed << " " << track.is_associated << " "
          << track.state_vec(S_X) << " " << track.state_vec(S_Y) << " "
          << track.state_vec(S_VX) << " " << track.state_vec(S_VY) << " "
          << track.state_vec(S_YAW) << "\n";
    }
    previous = timestamp;
  }
}
