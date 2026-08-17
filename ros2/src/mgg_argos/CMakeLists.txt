cmake_minimum_required(VERSION 3.8)
project(mgg_argos)

# Default to Release; see the note in mgg_core/CMakeLists.txt.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(mgg_argos_bridge src/bridge_node.cpp src/bridge_main.cpp)
target_include_directories(mgg_argos_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(mgg_argos_bridge
  rclcpp nav_msgs sensor_msgs geometry_msgs rosgraph_msgs tf2_ros
  visualization_msgs)
install(TARGETS mgg_argos_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config experiments DESTINATION share/${PROJECT_NAME})

#
# The ARGoS half: a controller and a loop function, loaded into the
# simulator. These deliberately do not link against ROS 2 (see
# include/mgg_argos/protocol.h); they are built here only so that both ends of
# the protocol live in one place and change together. ARGoS is not a ROS
# package, so this is skipped when it is not installed and the ROS side still
# builds - which is what happens in a container that runs the planner but not
# the simulator.
#
find_path(ARGOS3_INCLUDE_DIR argos3/core/simulator/simulator.h)

if(ARGOS3_INCLUDE_DIR)
  message(STATUS "mgg_argos: ARGoS found, building the simulator plugins")
  add_subdirectory(argos)
else()
  message(STATUS "mgg_argos: ARGoS not found, skipping the simulator plugins "
                 "(the ROS 2 bridge still builds). Build them separately with "
                 "cmake -B build -S argos on a machine that has ARGoS.")
endif()

ament_package()
