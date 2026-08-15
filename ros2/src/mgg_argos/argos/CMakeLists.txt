# The ARGoS half of mgg_argos: a controller and a loop function loaded into
# the simulator.
#
# Buildable two ways, because the two ends of the bridge normally live in
# different places: ARGoS is installed on the host, ROS 2 in a container.
#
#   Standalone, on a machine with ARGoS:
#     cmake -B build -S . && cmake --build build
#
#   As part of the ament package, when ARGoS happens to be present:
#     the parent CMakeLists add_subdirectory()s this file.
#
# Either way these targets link against ARGoS only. Nothing here includes a
# ROS header; the sole shared file is ../include/mgg_argos/protocol.h, which
# depends on nothing but <cstdint>.

cmake_minimum_required(VERSION 3.8)

if(NOT DEFINED PROJECT_NAME)
  project(mgg_argos_plugins CXX)
  set(MGG_ARGOS_STANDALONE TRUE)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

list(APPEND CMAKE_MODULE_PATH /usr/local/share/argos3/cmake
                             /usr/share/argos3/cmake)
find_package(ARGoS REQUIRED COMPONENTS genericrobot)

# The lidar control interface arrived with the photorealism plugin; without it
# the controller cannot compile, and the error from a missing header is far
# less useful than this one. find_file rather than EXISTS: ARGOS_INCLUDE_DIRS
# is a list (it carries Lua's and Qt's directories too), so pasting a path
# onto it would never match.
find_file(MGG_ARGOS_LIDAR_HEADER
  argos3/plugins/robots/generic/control_interface/ci_photorealistic_lidar_sensor.h
  HINTS ${ARGOS_INCLUDE_DIRS})
if(NOT MGG_ARGOS_LIDAR_HEADER)
  message(FATAL_ERROR
    "The installed ARGoS has no photorealistic_lidar sensor. It comes from the "
    "photorealism plugin; rebuild and reinstall ARGoS with it enabled.")
endif()

# ARGOS_INCLUDE_DIRS already carries Lua's include directory, which is needed
# because ARGoS's control-interface headers include lua.h unconditionally even
# though none of this code touches Lua.
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})

add_library(mgg_footbot MODULE mgg_footbot.cpp)
target_link_libraries(mgg_footbot
  argos3core_${ARGOS_BUILD_FOR} argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

# The loop function needs the controller's symbols: it downcasts each robot's
# controller to read the same sensor readings the controller sees, rather than
# taking a second copy of a 28800-ray scan.
add_library(mgg_bridge MODULE mgg_bridge.cpp mgg_footbot.cpp)
target_include_directories(mgg_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(mgg_bridge
  argos3core_${ARGOS_BUILD_FOR} argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)

if(MGG_ARGOS_STANDALONE)
  message(STATUS
    "Built standalone. Point ARGOS_PLUGIN_PATH at ${CMAKE_CURRENT_BINARY_DIR} "
    "so the simulator can load these.")
else()
  install(TARGETS mgg_footbot mgg_bridge DESTINATION lib/${PROJECT_NAME})
endif()
