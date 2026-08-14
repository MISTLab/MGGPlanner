cmake_minimum_required(VERSION 3.0.2)
project(map_baseline)

add_definitions(-std=c++14 -Wall -Wno-sign-compare -Werror=return-type)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  planner_common
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf2_ros
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(map_baseline_node src/map_baseline_node.cpp)
target_link_libraries(map_baseline_node ${catkin_LIBRARIES})

install(TARGETS map_baseline_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
