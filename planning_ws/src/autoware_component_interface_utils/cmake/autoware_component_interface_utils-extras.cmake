# The installed public API uses std::optional.  Propagate this requirement to
# Catkin consumers rather than depending on their local default standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
