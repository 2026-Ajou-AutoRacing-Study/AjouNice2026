# Exported catkin consumer language-floor contract for public lru_cache.hpp.
# catkin exports libraries as variables, not imported targets; consequently this
# config extra is loaded by find_package(catkin COMPONENTS autoware_utils_system).
if(NOT DEFINED CMAKE_CXX_STANDARD OR CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
