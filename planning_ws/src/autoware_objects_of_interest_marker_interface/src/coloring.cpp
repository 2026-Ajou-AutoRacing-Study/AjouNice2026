// Copyright 2023 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware/objects_of_interest_marker_interface/coloring.hpp"

#include <cstdint>

namespace
{
std_msgs::ColorRGBA convertFromColorCode(const uint64_t code, float alpha)
{
  const float r = static_cast<float>(static_cast<int>(code >> 16) / 255.0);
  const float g = static_cast<float>(static_cast<int>((code << 48) >> 56) / 255.0);
  const float b = static_cast<float>(static_cast<int>((code << 56) >> 56) / 255.0);
  return autoware_utils_visualization::create_marker_color(r, g, b, alpha);
}
}  // namespace

namespace autoware::objects_of_interest_marker_interface::coloring
{
std_msgs::ColorRGBA getGreen(float alpha) { return convertFromColorCode(0x00e676, alpha); }
std_msgs::ColorRGBA getAmber(float alpha) { return convertFromColorCode(0xffea00, alpha); }
std_msgs::ColorRGBA getRed(float alpha) { return convertFromColorCode(0xff3d00, alpha); }
std_msgs::ColorRGBA getGray(float alpha) { return convertFromColorCode(0xbdbdbd, alpha); }
}  // namespace autoware::objects_of_interest_marker_interface::coloring
