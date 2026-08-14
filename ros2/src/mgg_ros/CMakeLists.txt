cmake_minimum_required(VERSION 3.8)
project(mgg_ros)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mgg_map_octomap REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}
  src/param_loader.cpp
  src/conversions.cpp
  src/planner_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} mgg_core::mgg_core
  mgg_map_octomap::mgg_map_octomap Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros
  tf2_geometry_msgs mgg_msgs nav_msgs sensor_msgs std_srvs)

add_executable(mggplanner_node src/planner_main.cpp)
target_link_libraries(mggplanner_node ${PROJECT_NAME})
ament_target_dependencies(mggplanner_node rclcpp)
install(TARGETS mggplanner_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mgg_core mgg_msgs mgg_map_octomap rclcpp
  geometry_msgs visualization_msgs tf2_ros nav_msgs sensor_msgs std_srvs
  Eigen3)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_param_loader test/test_param_loader.cpp)
  target_link_libraries(test_param_loader ${PROJECT_NAME})
  ament_target_dependencies(test_param_loader rclcpp mgg_msgs)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions ${PROJECT_NAME})
  ament_target_dependencies(test_conversions rclcpp mgg_msgs)
endif()

ament_package()
