#include <netinet/in.h>
#include <sys/socket.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>

#include "sensor_interface/morai_udp_common.hpp"

namespace {

#pragma pack(push, 1)
struct EgoCtrlCmdPacket {
  char header[14];
  int32_t data_length;
  int32_t aux_data[3];
  int8_t ctrl_mode;
  int8_t gear;
  int8_t cmd_type;
  float velocity;
  float acceleration;
  float accel;
  float brake;
  float steer;
  char tail[2];
};
#pragma pack(pop)

static_assert(sizeof(EgoCtrlCmdPacket) == 55, "unexpected Ctrl Cmd packet layout");

}  // namespace

class MoraiCtrlCmdUdpSender {
 public:
  MoraiCtrlCmdUdpSender() : private_nh_("~") {
    private_nh_.param<std::string>("network_config", network_config_, std::string());
    private_nh_.param("ctrl_mode", ctrl_mode_, 2);
    private_nh_.param("gear", gear_, 4);
    if (!private_nh_.getParam("vehicle/max_steer_rad", max_steer_rad_)) {
      double max_wheel_angle_deg = 40.0;
      private_nh_.param("vehicle/max_wheel_angle_deg", max_wheel_angle_deg, 40.0);
      max_steer_rad_ = max_wheel_angle_deg * M_PI / 180.0;
    }
    if (network_config_.empty()) throw std::runtime_error("~network_config is required");
    if (!std::isfinite(max_steer_rad_) || max_steer_rad_ <= 0.0) {
      throw std::runtime_error("vehicle max steering angle must be finite and positive");
    }

    const auto root = morai_udp::readJson(network_config_);
    const auto& ego = root.get_child("egoNetworkData").front().second;
    bool found = false;
    for (const auto& item : ego.get_child("listEgoNetworkInfo")) {
      const auto& entry = item.second;
      if (entry.get<int>("PUBSUB_TYPE") != 771) continue;
      const auto& config = entry.get_child("EgoCtrlConfig");
      if (config.get<int>("commType", 0) != 1) {
        throw std::runtime_error("Ego Ctrl Cmd entry is not UDP (commType 1)");
      }
      const auto local = morai_udp::udpEndpoint(config, "destinationIP", "destinationPort");
      const auto remote = morai_udp::udpEndpoint(config, "hostIP", "hostPort");
      socket_ = morai_udp::bindReceiver(local);
      remote_address_ = morai_udp::remoteAddress(remote);
      ROS_INFO("Ctrl Cmd ROS /ctrl_cmd -> UDP %s:%d (local %s:%d, longi type 1, "
               "steering %.6f rad -> normalized 1.0)",
               remote.ip.c_str(), remote.port, local.ip.c_str(), local.port,
               max_steer_rad_);
      found = true;
      break;
    }
    if (!found) throw std::runtime_error("UDP PUBSUB_TYPE 771 not found in " + network_config_);
    subscriber_ = nh_.subscribe("/ctrl_cmd", 10, &MoraiCtrlCmdUdpSender::callback, this);
  }

  ~MoraiCtrlCmdUdpSender() {
    if (socket_ >= 0) ::close(socket_);
  }

 private:
  void callback(const morai_msgs::CtrlCmd::ConstPtr& message) {
    if (message->longlCmdType != 1) {
      ROS_WARN_THROTTLE(1.0, "/ctrl_cmd longlCmdType=%d received; competition UDP output forces type 1",
                        message->longlCmdType);
    }
    EgoCtrlCmdPacket packet{};
    std::memcpy(packet.header, "#MoraiCtrlCmd$", sizeof(packet.header));
    packet.data_length = 23;
    packet.ctrl_mode = static_cast<int8_t>(ctrl_mode_);
    packet.gear = static_cast<int8_t>(gear_);
    packet.cmd_type = 1;
    packet.velocity = static_cast<float>(message->velocity);
    packet.acceleration = static_cast<float>(message->acceleration);
    packet.accel = static_cast<float>(std::max(0.0, std::min(1.0, message->accel)));
    packet.brake = static_cast<float>(std::max(0.0, std::min(1.0, message->brake)));
    // ROS control uses tire steering angle [rad], while MORAI Ego Ctrl Cmd
    // expects a normalized steering input in [-1, 1].
    packet.steer = static_cast<float>(
        std::max(-1.0, std::min(1.0, message->steering / max_steer_rad_)));
    packet.tail[0] = '\r';
    packet.tail[1] = '\n';

    const ssize_t sent = ::sendto(socket_, &packet, sizeof(packet), 0,
                                  reinterpret_cast<const sockaddr*>(&remote_address_),
                                  sizeof(remote_address_));
    if (sent != static_cast<ssize_t>(sizeof(packet))) {
      ROS_ERROR_THROTTLE(1.0, "Ctrl Cmd UDP send failed: %s", std::strerror(errno));
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  std::string network_config_;
  int socket_ = -1;
  int ctrl_mode_ = 2;
  int gear_ = 4;
  double max_steer_rad_ = 0.0;
  sockaddr_in remote_address_{};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_ctrl_cmd_udp_sender");
  try {
    MoraiCtrlCmdUdpSender sender;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("MORAI Ctrl Cmd UDP sender: %s", error.what());
    return 1;
  }
  return 0;
}
