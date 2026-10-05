cmake_minimum_required(VERSION 3.8)
project(mgg_ros)

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

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(mgg_core REQUIRED)
find_package(mgg_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mgg_map_octomap REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_library(${PROJECT_NAME}
  src/param_loader.cpp
  src/conversions.cpp
  src/fleet_conversions.cpp
  src/global_guidance.cpp
  src/planner_guidance.cpp
  src/keyframe_trajectory.cpp
  src/local_events.cpp
  src/local_path_conversions.cpp
  src/local_planner_node.cpp
  src/local_planning_core.cpp
  src/local_replay.cpp
  src/planner_node.cpp
  src/scan_input.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} mgg_core::mgg_core
  mgg_map_octomap::mgg_map_octomap Eigen3::Eigen nlohmann_json::nlohmann_json OpenSSL::Crypto)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_msgs tf2_ros
  tf2_geometry_msgs mgg_msgs nav_msgs sensor_msgs std_srvs rosbag2_cpp)

add_executable(mggplanner_node src/planner_main.cpp)
target_link_libraries(mggplanner_node ${PROJECT_NAME})
ament_target_dependencies(mggplanner_node rclcpp)
install(TARGETS mggplanner_node DESTINATION lib/${PROJECT_NAME})

add_executable(mgg_local_planner_node src/local_planner_main.cpp)
target_link_libraries(mgg_local_planner_node ${PROJECT_NAME})
ament_target_dependencies(mgg_local_planner_node rclcpp)
install(TARGETS mgg_local_planner_node DESTINATION lib/${PROJECT_NAME})

add_executable(mgg_local_replay src/local_replay_main.cpp)
target_link_libraries(mgg_local_replay ${PROJECT_NAME})
install(TARGETS mgg_local_replay DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mgg_core mgg_msgs mgg_map_octomap rclcpp
  geometry_msgs visualization_msgs tf2_msgs tf2_ros nav_msgs sensor_msgs std_srvs
  Eigen3 nlohmann_json rosbag2_cpp OpenSSL)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_global_guidance test/test_global_guidance.cpp)
  target_link_libraries(test_global_guidance ${PROJECT_NAME})
  ament_target_dependencies(test_global_guidance mgg_msgs)
  ament_add_gtest(test_exploration_event test/test_exploration_event.cpp)
  target_link_libraries(test_exploration_event ${PROJECT_NAME})
  ament_add_gtest(test_param_loader test/test_param_loader.cpp)
  target_link_libraries(test_param_loader ${PROJECT_NAME})
  ament_target_dependencies(test_param_loader rclcpp mgg_msgs)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME})
  ament_target_dependencies(test_conversions rclcpp mgg_msgs)
  ament_add_gtest(test_fleet_conversions test/test_fleet_conversions.cpp)
  target_link_libraries(test_fleet_conversions ${PROJECT_NAME})
  ament_target_dependencies(test_fleet_conversions rclcpp mgg_msgs)
  ament_add_gtest(test_keyframe_trajectory test/test_keyframe_trajectory.cpp)
  target_link_libraries(test_keyframe_trajectory ${PROJECT_NAME})
  ament_add_gtest(test_no_path_streak test/test_no_path_streak.cpp)
  target_link_libraries(test_no_path_streak ${PROJECT_NAME})
  ament_add_gtest(test_planner_config_state test/test_planner_config_state.cpp)
  target_link_libraries(test_planner_config_state ${PROJECT_NAME})
  ament_target_dependencies(test_planner_config_state rclcpp mgg_msgs)
  ament_add_gtest(test_map_backend test/test_map_backend.cpp)
  target_link_libraries(test_map_backend ${PROJECT_NAME})
  ament_target_dependencies(test_map_backend rclcpp)
  # NAVIGATE for a Bunker-sized robot on MOLA products (always built).
  ament_add_gtest(test_planner_navigation test/test_planner_navigation.cpp
    TIMEOUT 300)
  target_link_libraries(test_planner_navigation ${PROJECT_NAME})
  ament_target_dependencies(test_planner_navigation rclcpp mgg_msgs)
  ament_add_gtest(test_local_path_conversions test/test_local_path_conversions.cpp)
  target_link_libraries(test_local_path_conversions ${PROJECT_NAME})
  ament_target_dependencies(test_local_path_conversions mgg_msgs)
  ament_add_gtest(test_local_replay test/test_local_replay.cpp TIMEOUT 300)
  target_link_libraries(test_local_replay ${PROJECT_NAME} OpenSSL::Crypto)
  ament_target_dependencies(test_local_replay rosbag2_cpp)
  ament_add_gtest(test_local_events test/test_local_events.cpp)
  target_link_libraries(test_local_events ${PROJECT_NAME})
  ament_target_dependencies(test_local_events mgg_msgs)
  ament_add_gtest(test_scan_input test/test_scan_input.cpp)
  target_link_libraries(test_scan_input ${PROJECT_NAME})
  ament_target_dependencies(test_scan_input sensor_msgs tf2)
  ament_add_gtest(test_local_planning_core test/test_local_planning_core.cpp
    TIMEOUT 300)
  target_link_libraries(test_local_planning_core ${PROJECT_NAME})
  ament_add_gtest(test_local_planner_node test/test_local_planner_node.cpp
    TIMEOUT 300)
  target_link_libraries(test_local_planner_node ${PROJECT_NAME})
  ament_target_dependencies(test_local_planner_node rclcpp mgg_msgs
    sensor_msgs nav_msgs)
  ament_add_gtest(test_native_scene_map test/test_native_scene_map.cpp)
  target_link_libraries(test_native_scene_map ${PROJECT_NAME})
  ament_add_gtest(test_planner_node test/test_planner_node.cpp)
  target_link_libraries(test_planner_node ${PROJECT_NAME})
  ament_target_dependencies(test_planner_node rclcpp mgg_msgs)
  # Two planners exploring one corridor over real topics: seconds, not
  # milliseconds.
  ament_add_gtest(test_fleet_exploration test/test_fleet_exploration.cpp
    TIMEOUT 600)
  target_link_libraries(test_fleet_exploration ${PROJECT_NAME})
  ament_target_dependencies(test_fleet_exploration rclcpp mgg_msgs)
endif()

ament_package()
