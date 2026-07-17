#include <poll.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <morai_msgs/CollisionData.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/ObjectStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <ros/ros.h>

#include "sensor_interface/morai_udp_common.hpp"

namespace {

#pragma pack(push, 1)
struct EgoVehiclePacket {
  char header[11];
  int32_t data_length;
  int32_t aux_data[3];
  int32_t sec;
  int32_t nsec;
  int8_t ctrl_mode;
  int8_t gear;
  float signed_vel;
  int32_t map_data_id;
  float accel;
  float brake;
  float size_x, size_y, size_z;
  float overhang, wheelbase, rear_overhang;
  float pos_x, pos_y, pos_z;
  float roll, pitch, yaw;
  float vel_x, vel_y, vel_z;
  float ang_vel_x, ang_vel_y, ang_vel_z;
  float accel_x, accel_y, accel_z;
  float steer;
  char link_id[38];
  float tire_lateral_force_fl, tire_lateral_force_fr;
  float tire_lateral_force_rl, tire_lateral_force_rr;
  float side_slip_angle_fl, side_slip_angle_fr;
  float side_slip_angle_rl, side_slip_angle_rr;
  float tire_cornering_stiffness_fl, tire_cornering_stiffness_fr;
  float tire_cornering_stiffness_rl, tire_cornering_stiffness_rr;
  char tail[2];
};

// Competition Vehicle Status uses the original 181-byte Ego status protocol.
// It ends after current link ID and omits the newer 48-byte tire extension.
struct CompetitionVehiclePacket {
  char header[11];
  int32_t data_length;
  int32_t aux_data[3];
  int32_t sec;
  int32_t nsec;
  int8_t ctrl_mode;
  int8_t gear;
  float signed_vel;
  int32_t map_data_id;
  float accel;
  float brake;
  float size_x, size_y, size_z;
  float overhang, wheelbase, rear_overhang;
  float pos_x, pos_y, pos_z;
  float roll, pitch, yaw;
  float vel_x, vel_y, vel_z;
  float ang_vel_x, ang_vel_y, ang_vel_z;
  float accel_x, accel_y, accel_z;
  float steer;
  char link_id[38];
  char tail[2];
};

struct ObjectPacketData {
  int16_t obj_id;
  int16_t obj_type;
  float pose_x, pose_y, pose_z;
  float heading;
  float size_x, size_y, size_z;
  float overhang, wheelbase, rear_overhang;
  float vel_x, vel_y, vel_z;
  float accel_x, accel_y, accel_z;
  char link_id[38];
};

struct ObjectPacket {
  char header[14];
  int32_t data_length;
  int32_t aux_data[3];
  int32_t sec;
  int32_t nsec;
  ObjectPacketData data[20];
  char tail[2];
};

struct CollisionPacketData {
  int16_t obj_type;
  int16_t obj_id;
  float pose_x, pose_y, pose_z;
  float global_offset_x, global_offset_y, global_offset_z;
};

struct CollisionPacket {
  char header[15];
  int32_t data_length;
  int32_t aux_data[3];
  int32_t sec;
  int32_t nsec;
  CollisionPacketData data[5];
  char tail[2];
};
#pragma pack(pop)

static_assert(sizeof(EgoVehiclePacket) == 229, "unexpected Ego packet layout");
static_assert(sizeof(CompetitionVehiclePacket) == 181,
              "unexpected Competition Vehicle Status packet layout");
static_assert(sizeof(ObjectPacket) == 2160, "unexpected Object packet layout");
static_assert(sizeof(CollisionPacket) == 181, "unexpected Collision packet layout");

struct NetworkChannel {
  int fd = -1;
  int type = 0;
  int port = 0;
};

ros::Time packetTime(int32_t sec, int32_t nsec) {
  if (sec > 0 && nsec >= 0 && nsec < 1000000000) {
    return ros::Time(static_cast<uint32_t>(sec), static_cast<uint32_t>(nsec));
  }
  return ros::Time::now();
}

std::string fixedString(const char* data, std::size_t length) {
  const auto* end = static_cast<const char*>(std::memchr(data, '\0', length));
  return std::string(data, end ? static_cast<std::size_t>(end - data) : length);
}

morai_msgs::ObjectStatus objectMessage(const ObjectPacketData& object) {
  morai_msgs::ObjectStatus message;
  message.unique_id = object.obj_id;
  message.type = object.obj_type;
  message.name = fixedString(object.link_id, sizeof(object.link_id));
  message.heading = object.heading;
  message.position.x = object.pose_x;
  message.position.y = object.pose_y;
  message.position.z = object.pose_z;
  message.size.x = object.size_x;
  message.size.y = object.size_y;
  message.size.z = object.size_z;
  message.velocity.x = object.vel_x;
  message.velocity.y = object.vel_y;
  message.velocity.z = object.vel_z;
  message.acceleration.x = object.accel_x;
  message.acceleration.y = object.accel_y;
  message.acceleration.z = object.accel_z;
  return message;
}

template <typename Packet>
morai_msgs::EgoVehicleStatus egoMessage(const Packet& packet, const std::string& frame_id) {
  morai_msgs::EgoVehicleStatus message;
  message.header.stamp = packetTime(packet.sec, packet.nsec);
  message.header.frame_id = frame_id;
  message.unique_id = packet.aux_data[0];
  message.position.x = packet.pos_x;
  message.position.y = packet.pos_y;
  message.position.z = packet.pos_z;
  message.velocity.x = packet.vel_x;
  message.velocity.y = packet.vel_y;
  message.velocity.z = packet.vel_z;
  message.acceleration.x = packet.accel_x;
  message.acceleration.y = packet.accel_y;
  message.acceleration.z = packet.accel_z;
  message.heading = packet.yaw;
  message.accel = packet.accel;
  message.brake = packet.brake;
  message.wheel_angle = packet.steer;
  return message;
}

}  // namespace

class MoraiEgoUdpReceiver {
 public:
  MoraiEgoUdpReceiver() : private_nh_("~") {
    private_nh_.param<std::string>("network_config", network_config_, std::string());
    private_nh_.param<std::string>("frame_id", frame_id_, "morai_map");
    if (network_config_.empty()) throw std::runtime_error("~network_config is required");

    const auto root = morai_udp::readJson(network_config_);
    const auto& ego = root.get_child("egoNetworkData").front().second;
    for (const auto& item : ego.get_child("listEgoNetworkInfo")) {
      const auto& entry = item.second;
      const int type = entry.get<int>("PUBSUB_TYPE");
      if (type != 770 && type != 772 && type != 261 && type != 295) continue;
      const auto& config = entry.get_child("config");
      if (config.get<int>("commType", 0) != 1) continue;
      const auto endpoint = morai_udp::udpEndpoint(config, "destinationIP", "destinationPort");
      NetworkChannel channel;
      channel.type = type;
      channel.port = endpoint.port;
      channel.fd = morai_udp::bindReceiver(endpoint);
      channels_.push_back(channel);

      const std::string topic = config.get<std::string>("rosConfig.Topic");
      if (type == 770) ego_publisher_ = nh_.advertise<morai_msgs::EgoVehicleStatus>(topic, 20);
      if (type == 295) {
        competition_publisher_ = nh_.advertise<morai_msgs::EgoVehicleStatus>(topic, 20);
      }
      if (type == 772) object_publisher_ = nh_.advertise<morai_msgs::ObjectStatusList>(topic, 10);
      if (type == 261) collision_publisher_ = nh_.advertise<morai_msgs::CollisionData>(topic, 10);
      ROS_INFO("Ego network type %d UDP %s:%d -> %s", type, endpoint.ip.c_str(),
               endpoint.port, topic.c_str());
    }
    if (channels_.size() != 4) {
      throw std::runtime_error("expected UDP entries 770, 772, 261 and 295 in " + network_config_);
    }
  }

  ~MoraiEgoUdpReceiver() {
    for (const auto& channel : channels_) if (channel.fd >= 0) ::close(channel.fd);
  }

  void spin() {
    std::array<uint8_t, 4096> buffer{};
    while (ros::ok()) {
      std::vector<pollfd> poll_fds;
      for (const auto& channel : channels_) poll_fds.push_back({channel.fd, POLLIN, 0});
      const int ready = ::poll(poll_fds.data(), poll_fds.size(), 20);
      if (ready < 0 && errno != EINTR) {
        ROS_ERROR_THROTTLE(1.0, "ego UDP poll failed: %s", std::strerror(errno));
      }
      for (std::size_t i = 0; i < channels_.size(); ++i) {
        if (poll_fds[i].revents & POLLIN) receive(channels_[i], buffer);
      }
      ros::spinOnce();
    }
  }

 private:
  void receive(const NetworkChannel& channel, std::array<uint8_t, 4096>& buffer) {
    while (true) {
      const ssize_t length = ::recv(channel.fd, buffer.data(), buffer.size(), 0);
      if (length < 0) return;
      if (channel.type == 770) parseEgo(buffer.data(), length);
      if (channel.type == 295) parseCompetition(buffer.data(), length);
      if (channel.type == 772) parseObjects(buffer.data(), length);
      if (channel.type == 261) parseCollision(buffer.data(), length);
    }
  }

  void parseEgo(const uint8_t* bytes, ssize_t length) {
    if (length != static_cast<ssize_t>(sizeof(EgoVehiclePacket))) {
      ROS_WARN_THROTTLE(1.0, "Ego packet size %zd, expected %zu", length,
                        sizeof(EgoVehiclePacket));
      return;
    }
    EgoVehiclePacket packet{};
    std::memcpy(&packet, bytes, sizeof(packet));
    if (packet.tail[0] != '\r' || packet.tail[1] != '\n') return;
    auto message = egoMessage(packet, frame_id_);
    message.tire_lateral_force_fl = packet.tire_lateral_force_fl;
    message.tire_lateral_force_fr = packet.tire_lateral_force_fr;
    message.tire_lateral_force_rl = packet.tire_lateral_force_rl;
    message.tire_lateral_force_rr = packet.tire_lateral_force_rr;
    message.side_slip_angle_fl = packet.side_slip_angle_fl;
    message.side_slip_angle_fr = packet.side_slip_angle_fr;
    message.side_slip_angle_rl = packet.side_slip_angle_rl;
    message.side_slip_angle_rr = packet.side_slip_angle_rr;
    message.tire_cornering_stiffness_fl = packet.tire_cornering_stiffness_fl;
    message.tire_cornering_stiffness_fr = packet.tire_cornering_stiffness_fr;
    message.tire_cornering_stiffness_rl = packet.tire_cornering_stiffness_rl;
    message.tire_cornering_stiffness_rr = packet.tire_cornering_stiffness_rr;
    ego_publisher_.publish(message);
  }

  void parseCompetition(const uint8_t* bytes, ssize_t length) {
    if (length != static_cast<ssize_t>(sizeof(CompetitionVehiclePacket))) {
      ROS_WARN_THROTTLE(1.0, "Competition Vehicle Status packet size %zd, expected %zu",
                        length, sizeof(CompetitionVehiclePacket));
      return;
    }
    CompetitionVehiclePacket packet{};
    std::memcpy(&packet, bytes, sizeof(packet));
    if (packet.tail[0] != '\r' || packet.tail[1] != '\n') return;
    competition_publisher_.publish(egoMessage(packet, frame_id_));
  }

  void parseObjects(const uint8_t* bytes, ssize_t length) {
    if (length != static_cast<ssize_t>(sizeof(ObjectPacket))) {
      ROS_WARN_THROTTLE(1.0, "Object packet size %zd, expected %zu", length,
                        sizeof(ObjectPacket));
      return;
    }
    ObjectPacket packet{};
    std::memcpy(&packet, bytes, sizeof(packet));
    if (packet.tail[0] != '\r' || packet.tail[1] != '\n') return;
    morai_msgs::ObjectStatusList message;
    message.header.stamp = packetTime(packet.sec, packet.nsec);
    message.header.frame_id = frame_id_;
    for (const auto& object : packet.data) {
      if (object.obj_id == 0 && object.obj_type == 0) continue;
      const auto converted = objectMessage(object);
      if (object.obj_type == 1) message.npc_list.push_back(converted);
      else if (object.obj_type == 0) message.pedestrian_list.push_back(converted);
      else message.obstacle_list.push_back(converted);
    }
    message.num_of_npcs = static_cast<int32_t>(message.npc_list.size());
    message.num_of_pedestrian = static_cast<int32_t>(message.pedestrian_list.size());
    message.num_of_obstacle = static_cast<int32_t>(message.obstacle_list.size());
    object_publisher_.publish(message);
  }

  void parseCollision(const uint8_t* bytes, ssize_t length) {
    if (length != static_cast<ssize_t>(sizeof(CollisionPacket))) {
      ROS_WARN_THROTTLE(1.0, "Collision packet size %zd, expected %zu", length,
                        sizeof(CollisionPacket));
      return;
    }
    CollisionPacket packet{};
    std::memcpy(&packet, bytes, sizeof(packet));
    if (packet.tail[0] != '\r' || packet.tail[1] != '\n') return;
    morai_msgs::CollisionData message;
    message.header.stamp = packetTime(packet.sec, packet.nsec);
    message.header.frame_id = frame_id_;
    bool have_offset = false;
    for (const auto& collision : packet.data) {
      // MORAI fills unused collision slots with obj_type=-1.  They are
      // placeholders, not collision objects.
      if (collision.obj_type < 0) continue;
      if (collision.obj_id == 0 && collision.obj_type == 0) continue;
      if (!have_offset) {
        message.global_offset_x = collision.global_offset_x;
        message.global_offset_y = collision.global_offset_y;
        message.global_offset_z = collision.global_offset_z;
        have_offset = true;
      }
      morai_msgs::ObjectStatus object;
      object.unique_id = collision.obj_id;
      object.type = collision.obj_type;
      object.position.x = collision.pose_x;
      object.position.y = collision.pose_y;
      object.position.z = collision.pose_z;
      message.collision_object.push_back(object);
    }
    collision_publisher_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string network_config_;
  std::string frame_id_;
  std::vector<NetworkChannel> channels_;
  ros::Publisher ego_publisher_;
  ros::Publisher competition_publisher_;
  ros::Publisher object_publisher_;
  ros::Publisher collision_publisher_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_ego_udp_receiver");
  try {
    MoraiEgoUdpReceiver receiver;
    receiver.spin();
  } catch (const std::exception& error) {
    ROS_FATAL("MORAI ego UDP receiver: %s", error.what());
    return 1;
  }
  return 0;
}
