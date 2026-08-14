cmake_minimum_required(VERSION 3.8)
project(mgg_kdtree)

# Default to Release.
#
# catkin took the build type from the workspace (the ROS 1 image runs
# "catkin config -DCMAKE_BUILD_TYPE=Release"); colcon has no equivalent, so a
# plain "colcon build" produces an unoptimised binary. That is not a small
# difference here: the planner is Eigen-heavy, and measured on the dynamic time
# warping inner loop of the direction penalty, -O0 costs 199 ns per cell
# against 4.8 ns with -O2, a factor of 41. An unoptimised planner misses its
# planning cycle by an order of magnitude.
#
# An explicit -DCMAKE_BUILD_TYPE still wins.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)

# Third-party C, built as-is. No -Wall -Wextra: this is not our code to tidy.
add_library(${PROJECT_NAME} src/kdtree.c)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(${PROJECT_NAME} PROPERTIES C_STANDARD 99)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
