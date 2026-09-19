#pragma once

#include <filesystem>

namespace autoware::test_utils
{

inline std::filesystem::path lanelet2_utils_source_root()
{
  return AUTOWARE_LANELET2_UTILS_SOURCE_DIR;
}

}  // namespace autoware::test_utils
