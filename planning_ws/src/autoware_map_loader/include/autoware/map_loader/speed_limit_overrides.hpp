#pragma once
#include <lanelet2_core/LaneletMap.h>
#include <string>
namespace autoware::map_loader
{
// Validate the complete file before mutating any lanelets. Empty path opts out.
void apply_speed_limit_overrides(
  const lanelet::LaneletMapPtr & map, const std::string & map_file,
  const std::string & overrides_file);
}
