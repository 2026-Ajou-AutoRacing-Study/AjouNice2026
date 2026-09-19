# Public headers use std::optional; make the required language standard part of
# the exported Catkin contract so downstream targets compile by default.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
