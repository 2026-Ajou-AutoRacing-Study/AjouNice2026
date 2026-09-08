# This public header exposes std::optional, so every downstream consumer must
# compile as C++17 or newer.  catkin includes this file from the installed
# package Config.cmake after find_package(autoware_rtc_interface).
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
