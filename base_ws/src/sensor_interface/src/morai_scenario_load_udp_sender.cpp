#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <morai_msgs/MoraiScenarioLoadSrv.h>
#include <ros/ros.h>

#include "sensor_interface/morai_udp_common.hpp"

namespace
{

#pragma pack(push, 1)
struct ScenarioLoadPacket
{
  char header[14];
  int32_t data_length;
  int32_t aux_data[3];
  char filename[30];
  uint8_t delete_all;
  uint8_t network;
  uint8_t ego;
  uint8_t npc;
  uint8_t pedestrian;
  uint8_t object;
  uint8_t pause;
  char tail[2];
};
#pragma pack(pop)

static_assert(sizeof(ScenarioLoadPacket) == 69, "unexpected Scenario Load packet layout");

std::string scenarioName(std::string filename)
{
  const std::string suffix = ".json";
  if (filename.size() >= suffix.size() &&
      filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    filename.erase(filename.size() - suffix.size());
  }
  return filename;
}

}  // namespace

class MoraiScenarioLoadUdpSender
{
public:
  MoraiScenarioLoadUdpSender() : private_nh_("~")
  {
    private_nh_.param<std::string>("network_config", network_config_, std::string());
    private_nh_.param<std::string>("service_name", service_name_,
                                   "/morai/scenario_load_udp");
    if (network_config_.empty())
    {
      throw std::runtime_error("~network_config is required");
    }

    const auto root = morai_udp::readJson(network_config_);
    const auto& ego = root.get_child("egoNetworkData").front().second;
    bool found = false;
    for (const auto& item : ego.get_child("listEgoNetworkInfo"))
    {
      const auto& entry = item.second;
      if (entry.get<int>("PUBSUB_TYPE") != 783)
      {
        continue;
      }
      const auto& config = entry.get_child("config");
      if (config.get<int>("commType", 0) != 1)
      {
        throw std::runtime_error("Scenario Load entry is not UDP (commType 1)");
      }

      const auto local =
          morai_udp::udpEndpoint(config, "destinationIP", "destinationPort");
      const auto remote = morai_udp::udpEndpoint(config, "hostIP", "hostPort");
      socket_ = morai_udp::bindReceiver(local);
      remote_address_ = morai_udp::remoteAddress(remote);
      ROS_INFO("Scenario Load service %s -> UDP %s:%d (local %s:%d)",
               service_name_.c_str(), remote.ip.c_str(), remote.port,
               local.ip.c_str(), local.port);
      found = true;
      break;
    }
    if (!found)
    {
      throw std::runtime_error("UDP PUBSUB_TYPE 783 not found in " +
                               network_config_);
    }

    service_ = nh_.advertiseService(
        service_name_, &MoraiScenarioLoadUdpSender::callback, this);
  }

  ~MoraiScenarioLoadUdpSender()
  {
    if (socket_ >= 0)
    {
      ::close(socket_);
    }
  }

private:
  bool callback(morai_msgs::MoraiScenarioLoadSrv::Request& request,
                morai_msgs::MoraiScenarioLoadSrv::Response& response)
  {
    const std::string filename = scenarioName(request.request.file_name);
    if (filename.empty() || filename.size() > sizeof(ScenarioLoadPacket::filename) ||
        filename.find('\0') != std::string::npos)
    {
      ROS_ERROR("Scenario Load filename must contain 1 to 30 bytes");
      response.response.result = false;
      return true;
    }

    ScenarioLoadPacket packet{};
    std::memcpy(packet.header, "#ScenarioLoad$", sizeof(packet.header));
    packet.data_length = 37;
    std::memset(packet.filename, ' ', sizeof(packet.filename));
    std::memcpy(packet.filename, filename.data(), filename.size());
    packet.delete_all = request.request.delete_all;
    packet.network = request.request.load_network_connection_data;
    packet.ego = request.request.load_ego_vehicle_data;
    packet.npc = request.request.load_surrounding_vehicle_data;
    packet.pedestrian = request.request.load_pedestrian_data;
    packet.object = request.request.load_obstacle_data;
    packet.pause = request.request.set_pause;
    packet.tail[0] = '\r';
    packet.tail[1] = '\n';

    const ssize_t sent =
        ::sendto(socket_, &packet, sizeof(packet), 0,
                 reinterpret_cast<const sockaddr*>(&remote_address_),
                 sizeof(remote_address_));
    response.response.result = sent == static_cast<ssize_t>(sizeof(packet));
    if (!response.response.result)
    {
      ROS_ERROR("Scenario Load UDP send failed: %s", std::strerror(errno));
      return true;
    }

    ROS_INFO("Scenario Load UDP sent: filename=%s network=%d ego=%d npc=%d "
             "pedestrian=%d object=%d pause=%d",
             filename.c_str(), packet.network, packet.ego, packet.npc,
             packet.pedestrian, packet.object, packet.pause);
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer service_;
  std::string network_config_;
  std::string service_name_;
  int socket_ = -1;
  sockaddr_in remote_address_{};
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "morai_scenario_load_udp_sender");
  try
  {
    MoraiScenarioLoadUdpSender sender;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("MORAI Scenario Load UDP sender: %s", error.what());
    return 1;
  }
  return 0;
}
