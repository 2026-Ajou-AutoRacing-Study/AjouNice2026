cmake_minimum_required(VERSION 3.0.2)
project(trajectory_follower)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  message_generation
  morai_msgs
  nav_msgs
  roscpp
  std_msgs
)

add_message_files(FILES PurePursuitDebug.msg TerminalState.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adaptive_pure_pursuit_core
  CATKIN_DEPENDS
    autoware_planning_msgs
    geometry_msgs
    message_runtime
    morai_msgs
    nav_msgs
    roscpp
    std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(purepursuit src/purepursuit.cpp)
add_executable(longi_pi src/longi_pi.cpp)
add_library(adaptive_pure_pursuit_core src/adaptive_pure_pursuit_core.cpp)
add_executable(purepursuit_autoware src/purepursuit_autoware.cpp)
add_executable(longi_pi_autoware src/longi_pi_autoware.cpp)
add_dependencies(purepursuit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(longi_pi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(purepursuit_autoware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(longi_pi_autoware ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(purepursuit ${catkin_LIBRARIES})
target_link_libraries(longi_pi ${catkin_LIBRARIES})
target_link_libraries(purepursuit_autoware adaptive_pure_pursuit_core ${catkin_LIBRARIES})
target_link_libraries(longi_pi_autoware ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_control_terminal.py)
  catkin_add_gtest(test_adaptive_pure_pursuit_core test/test_adaptive_pure_pursuit_core.cpp)
  if(TARGET test_adaptive_pure_pursuit_core)
    target_link_libraries(test_adaptive_pure_pursuit_core adaptive_pure_pursuit_core)
  endif()
  catkin_add_gtest(test_steering_rate_limiter test/test_steering_rate_limiter.cpp)
  catkin_add_gtest(test_distance_speed_limit test/test_distance_speed_limit.cpp)
  catkin_add_gtest(test_longitudinal_pi_utils test/test_longitudinal_pi_utils.cpp)
  find_package(rostest REQUIRED)
  add_rostest(test/autoware_freshness.test)
  add_rostest(test/control_terminal.test)
endif()

install(TARGETS purepursuit longi_pi purepursuit_autoware longi_pi_autoware adaptive_pure_pursuit_core
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/control_terminal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
