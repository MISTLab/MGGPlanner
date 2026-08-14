cmake_minimum_required(VERSION 3.0.2)
project(adaptive_obb)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(planner_common REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(
  ${PROJECT_NAME}
  src/adaptive_obb.cpp
)

cs_install()
cs_export(LIBRARIES)
