#include <poll.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

#include "sensor_interface/morai_udp_common.hpp"

namespace {

#pragma pack(push, 1)
struct MoraiImuPacket {
  char header[9];
  int32_t data_length;
  int32_t aux_data[3];
  int32_t sec;
  int32_t nsec;
  double ori_w;
  double ori_x;
  double ori_y;
  double ori_z;
  double ang_vel_x;
  double ang_vel_y;
  double ang_vel_z;
  double lin_acc_x;
  double lin_acc_y;
  double lin_acc_z;
  char tail[2];
};
#pragma pack(pop)

static_assert(sizeof(MoraiImuPacket) == 115, "unexpected IMU packet layout");

struct CameraChannel {
  int fd = -1;
  std::string name;
  std::string frame_id;
  int port = 0;
  ros::Publisher publisher;
  std::vector<uint8_t> jpeg;
  int32_t next_index = 0;
  ros::Time received_at;
};

struct SensorSocket {
  int fd = -1;
  int port = 0;
};

std::vector<std::string> splitCsv(const std::string& sentence) {
  std::vector<std::string> fields;
  std::stringstream stream(sentence);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

double nmeaDegrees(const std::string& value, const std::string& direction) {
  if (value.empty()) {
    return 0.0;
  }
  const double raw = std::stod(value);
  const double degrees = std::floor(raw / 100.0);
  double result = degrees + (raw - degrees * 100.0) / 60.0;
  if (direction == "S" || direction == "W") {
    result = -result;
  }
  return result;
}

const boost::property_tree::ptree& sensorConfig(const boost::property_tree::ptree& sensor,
                                                const char* key) {
  return sensor.get_child(key);
}

}  // namespace

class MoraiSensorUdpReceiver {
 public:
  MoraiSensorUdpReceiver() : private_nh_("~") {
    private_nh_.param<std::string>("sensor_config", sensor_config_, std::string());
    private_nh_.param("gps_east_offset", gps_east_offset_, 0.0);
    private_nh_.param("gps_north_offset", gps_north_offset_, 0.0);
    if (sensor_config_.empty()) {
      throw std::runtime_error("~sensor_config is required");
    }

    const auto root = morai_udp::readJson(sensor_config_);
    loadCameras(root);
    loadGps(root);
    loadImu(root);
    logLidar(root);
    if (cameras_.empty() && gps_.fd < 0 && imu_.fd < 0) {
      throw std::runtime_error("no enabled UDP camera/GPS/IMU entries in " + sensor_config_);
    }
  }

  ~MoraiSensorUdpReceiver() {
    for (auto& camera : cameras_) {
      if (camera.fd >= 0) ::close(camera.fd);
    }
    if (gps_.fd >= 0) ::close(gps_.fd);
    if (imu_.fd >= 0) ::close(imu_.fd);
  }

  void spin() {
    std::array<uint8_t, 65536> buffer{};
    while (ros::ok()) {
      std::vector<pollfd> poll_fds;
      poll_fds.reserve(cameras_.size() + 2);
      for (const auto& camera : cameras_) poll_fds.push_back({camera.fd, POLLIN, 0});
      if (gps_.fd >= 0) poll_fds.push_back({gps_.fd, POLLIN, 0});
      if (imu_.fd >= 0) poll_fds.push_back({imu_.fd, POLLIN, 0});

      const int ready = ::poll(poll_fds.data(), poll_fds.size(), 20);
      if (ready < 0 && errno != EINTR) {
        ROS_ERROR_THROTTLE(1.0, "UDP poll failed: %s", std::strerror(errno));
      }
      std::size_t poll_index = 0;
      for (std::size_t i = 0; i < cameras_.size(); ++i, ++poll_index) {
        if (poll_fds[poll_index].revents & POLLIN) {
          receiveCamera(cameras_[i], buffer);
        }
      }
      if (gps_.fd >= 0) {
        if (poll_fds[poll_index].revents & POLLIN) receiveGps(buffer);
        ++poll_index;
      }
      if (imu_.fd >= 0 && (poll_fds[poll_index].revents & POLLIN)) receiveImu(buffer);
      ros::spinOnce();
    }
  }

 private:
  void loadCameras(const boost::property_tree::ptree& root) {
    for (const auto& item : root.get_child("cameraList")) {
      const auto& sensor = item.second;
      const auto& config = sensorConfig(sensor, "cc");
      if (config.get<int>("commType", 0) != 1) continue;
      const auto endpoint = morai_udp::udpEndpoint(config, "destinationIP", "destinationPort");
      CameraChannel camera;
      camera.name = config.get<std::string>("sensorName", "camera");
      camera.frame_id = config.get<std::string>("rosConfig.frameID", camera.name);
      camera.port = endpoint.port;
      camera.fd = morai_udp::bindReceiver(endpoint, 8 * 1024 * 1024);
      const std::string topic = config.get<std::string>("rosConfig.Topic");
      camera.publisher = nh_.advertise<sensor_msgs::CompressedImage>(topic, 2);
      ROS_INFO("Camera UDP %s:%d -> %s (%s)", endpoint.ip.c_str(), endpoint.port,
               topic.c_str(), camera.frame_id.c_str());
      cameras_.push_back(std::move(camera));
    }
  }

  void loadGps(const boost::property_tree::ptree& root) {
    for (const auto& item : root.get_child("GPSList")) {
      const auto& config = sensorConfig(item.second, "gc");
      if (config.get<int>("commType", 0) != 1) continue;
      const auto endpoint = morai_udp::udpEndpoint(config, "destinationIP", "destinationPort");
      gps_.fd = morai_udp::bindReceiver(endpoint);
      gps_.port = endpoint.port;
      gps_frame_ = config.get<std::string>("rosConfig.frameID", "gps");
      const std::string topic = config.get<std::string>("rosConfig.Topic", "/sensor/gps");
      gps_publisher_ = nh_.advertise<morai_msgs::GPSMessage>(topic, 10);
      ROS_INFO("GPS UDP %s:%d -> %s", endpoint.ip.c_str(), endpoint.port, topic.c_str());
      break;
    }
  }

  void loadImu(const boost::property_tree::ptree& root) {
    for (const auto& item : root.get_child("IMUList")) {
      const auto& config = sensorConfig(item.second, "ic");
      if (config.get<int>("commType", 0) != 1) continue;
      const auto endpoint = morai_udp::udpEndpoint(config, "destinationIP", "destinationPort");
      imu_.fd = morai_udp::bindReceiver(endpoint);
      imu_.port = endpoint.port;
      imu_frame_ = config.get<std::string>("rosConfig.frameID", "imu");
      const std::string topic = config.get<std::string>("rosConfig.Topic", "/sensor/imu");
      imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(topic, 50);
      ROS_INFO("IMU UDP %s:%d -> %s", endpoint.ip.c_str(), endpoint.port, topic.c_str());
      break;
    }
  }

  void logLidar(const boost::property_tree::ptree& root) {
    for (const auto& item : root.get_child("Lidar3DList")) {
      const auto& config = sensorConfig(item.second, "lc");
      if (config.get<int>("commType", 0) != 1) continue;
      const auto endpoint = morai_udp::udpEndpoint(config, "destinationIP", "destinationPort");
      ROS_INFO("LiDAR UDP declared at %s:%d; handled by velodyne_driver",
               endpoint.ip.c_str(), endpoint.port);
    }
  }

  void receiveCamera(CameraChannel& camera, std::array<uint8_t, 65536>& buffer) {
    while (true) {
      const ssize_t length = ::recv(camera.fd, buffer.data(), buffer.size(), 0);
      if (length < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          ROS_WARN_THROTTLE(1.0, "camera port %d recv failed: %s", camera.port,
                            std::strerror(errno));
        }
        return;
      }
      const ros::Time received_at = ros::Time::now();
      if (length < 21 || std::memcmp(buffer.data(), "MOR", 3) != 0) continue;

      int32_t sec = 0, nsec = 0, index = 0;
      std::memcpy(&sec, buffer.data() + 3, sizeof(sec));
      std::memcpy(&nsec, buffer.data() + 7, sizeof(nsec));
      std::memcpy(&index, buffer.data() + 11, sizeof(index));
      if (index == 0 || index != camera.next_index || camera.jpeg.empty()) {
        camera.jpeg.clear();
        // Frame timestamp is the first fragment's ROS-host receipt time.
        camera.received_at = received_at;
      }
      camera.next_index = index + 1;

      const std::size_t payload_begin = 19;
      const std::size_t payload_end = static_cast<std::size_t>(length) - 2;
      if (camera.jpeg.size() + payload_end - payload_begin > 20 * 1024 * 1024) {
        camera.jpeg.clear();
        ROS_WARN_THROTTLE(1.0, "discarding oversized JPEG on camera port %d", camera.port);
        continue;
      }
      camera.jpeg.insert(camera.jpeg.end(), buffer.begin() + payload_begin,
                         buffer.begin() + payload_end);

      if (buffer[length - 2] == 'E' && buffer[length - 1] == 'I') {
        for (std::size_t i = camera.jpeg.size(); i >= 2; --i) {
          if (camera.jpeg[i - 2] == 0xff && camera.jpeg[i - 1] == 0xd9) {
            camera.jpeg.resize(i);
            break;
          }
        }
        if (camera.jpeg.size() >= 4 && camera.jpeg[0] == 0xff && camera.jpeg[1] == 0xd8) {
          sensor_msgs::CompressedImage message;
          message.header.stamp = camera.received_at;
          message.header.frame_id = camera.frame_id;
          message.format = "jpeg";
          message.data.swap(camera.jpeg);
          camera.publisher.publish(message);
        } else {
          ROS_WARN_THROTTLE(1.0, "invalid JPEG received on camera port %d", camera.port);
          camera.jpeg.clear();
        }
        camera.next_index = 0;
      }
    }
  }

  void receiveGps(std::array<uint8_t, 65536>& buffer) {
    while (true) {
      const ssize_t length = ::recv(gps_.fd, buffer.data(), buffer.size() - 1, 0);
      if (length < 0) return;
      const ros::Time received_at = ros::Time::now();
      buffer[length] = 0;
      std::string sentence(reinterpret_cast<char*>(buffer.data()), static_cast<std::size_t>(length));
      const auto nul = sentence.find('\0');
      if (nul != std::string::npos) sentence.resize(nul);
      if (sentence.compare(0, 6, "$GPGGA") != 0 && sentence.compare(0, 6, "$GNGGA") != 0) {
        continue;
      }
      try {
        const auto fields = splitCsv(sentence);
        if (fields.size() < 10) continue;
        morai_msgs::GPSMessage message;
        message.header.stamp = received_at;
        message.header.frame_id = gps_frame_;
        message.latitude = nmeaDegrees(fields[2], fields[3]);
        message.longitude = nmeaDegrees(fields[4], fields[5]);
        message.status = static_cast<int16_t>(std::stoi(fields[6]));
        message.altitude = fields[9].empty() ? 0.0 : std::stod(fields[9]);
        message.eastOffset = gps_east_offset_;
        message.northOffset = gps_north_offset_;
        gps_publisher_.publish(message);
      } catch (const std::exception& error) {
        ROS_WARN_THROTTLE(1.0, "invalid GPS NMEA packet: %s", error.what());
      }
    }
  }

  void receiveImu(std::array<uint8_t, 65536>& buffer) {
    while (true) {
      const ssize_t length = ::recv(imu_.fd, buffer.data(), buffer.size(), 0);
      if (length < 0) return;
      const ros::Time received_at = ros::Time::now();
      if (length != static_cast<ssize_t>(sizeof(MoraiImuPacket))) {
        ROS_WARN_THROTTLE(1.0, "IMU packet size %zd, expected %zu", length,
                          sizeof(MoraiImuPacket));
        continue;
      }
      MoraiImuPacket packet{};
      std::memcpy(&packet, buffer.data(), sizeof(packet));
      if (packet.tail[0] != '\r' || packet.tail[1] != '\n') continue;
      sensor_msgs::Imu message;
      message.header.stamp = received_at;
      message.header.frame_id = imu_frame_;
      message.orientation.w = packet.ori_w;
      message.orientation.x = packet.ori_x;
      message.orientation.y = packet.ori_y;
      message.orientation.z = packet.ori_z;
      message.angular_velocity.x = packet.ang_vel_x;
      message.angular_velocity.y = packet.ang_vel_y;
      message.angular_velocity.z = packet.ang_vel_z;
      message.linear_acceleration.x = packet.lin_acc_x;
      message.linear_acceleration.y = packet.lin_acc_y;
      message.linear_acceleration.z = packet.lin_acc_z;
      message.orientation_covariance[0] = -1.0;
      message.angular_velocity_covariance[0] = -1.0;
      message.linear_acceleration_covariance[0] = -1.0;
      imu_publisher_.publish(message);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string sensor_config_;
  std::vector<CameraChannel> cameras_;
  SensorSocket gps_;
  SensorSocket imu_;
  ros::Publisher gps_publisher_;
  ros::Publisher imu_publisher_;
  std::string gps_frame_ = "gps";
  std::string imu_frame_ = "imu";
  double gps_east_offset_ = 0.0;
  double gps_north_offset_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_sensor_udp_receiver");
  try {
    MoraiSensorUdpReceiver receiver;
    receiver.spin();
  } catch (const std::exception& error) {
    ROS_FATAL("MORAI sensor UDP receiver: %s", error.what());
    return 1;
  }
  return 0;
}
