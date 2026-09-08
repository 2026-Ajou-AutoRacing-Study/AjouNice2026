#pragma once
#include <autoware/map_loader/native_node.hpp>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
inline std::filesystem::path owned_map_test_dir() {
  const char * value=std::getenv("ROS_HOME");
  if(!value)throw std::runtime_error("Map test requires owned ROS_HOME");
  return std::filesystem::path(value);
}
inline std::string owned_map_test_file(const char * name) {return (owned_map_test_dir()/name).string();}
