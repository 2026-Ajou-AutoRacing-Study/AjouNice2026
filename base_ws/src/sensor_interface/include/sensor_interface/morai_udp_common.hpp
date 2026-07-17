#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace morai_udp {

struct Endpoint {
  std::string ip;
  int port = 0;
};

inline boost::property_tree::ptree readJson(const std::string& path) {
  boost::property_tree::ptree tree;
  boost::property_tree::read_json(path, tree);
  return tree;
}

inline Endpoint udpEndpoint(const boost::property_tree::ptree& config,
                            const std::string& ip_key,
                            const std::string& port_key) {
  Endpoint endpoint;
  endpoint.ip = config.get<std::string>("udpConfig." + ip_key);
  endpoint.port = config.get<int>("udpConfig." + port_key);
  return endpoint;
}

inline int bindReceiver(const Endpoint& endpoint, int receive_buffer = 4 * 1024 * 1024) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    throw std::runtime_error("socket: " + std::string(std::strerror(errno)));
  }

  int enabled = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
  ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(endpoint.port));
  if (::inet_pton(AF_INET, endpoint.ip.c_str(), &address.sin_addr) != 1) {
    ::close(fd);
    throw std::runtime_error("invalid IPv4 address: " + endpoint.ip);
  }
  if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
    const std::string error = std::strerror(errno);
    ::close(fd);
    throw std::runtime_error("bind " + endpoint.ip + ":" +
                             std::to_string(endpoint.port) + ": " + error);
  }
  const int flags = ::fcntl(fd, F_GETFL, 0);
  ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  return fd;
}

inline sockaddr_in remoteAddress(const Endpoint& endpoint) {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(endpoint.port));
  if (::inet_pton(AF_INET, endpoint.ip.c_str(), &address.sin_addr) != 1) {
    throw std::runtime_error("invalid IPv4 address: " + endpoint.ip);
  }
  return address;
}

inline bool startsWith(const char* data, std::size_t length, const char* prefix,
                       std::size_t prefix_length) {
  return length >= prefix_length && std::memcmp(data, prefix, prefix_length) == 0;
}

}  // namespace morai_udp
