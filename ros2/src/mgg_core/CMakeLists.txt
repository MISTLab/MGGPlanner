cmake_minimum_required(VERSION 3.8)
project(mgg_core)

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
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)  # Boost.Graph, header-only
find_package(mgg_kdtree REQUIRED)

# Deliberately no ROS dependency: that is what makes the core testable and
# deployable on its own. Grows as the algorithm moves across from
# planner_common.
add_library(${PROJECT_NAME}
  src/graph.cpp
  src/sensor_params.cpp
  src/graph_base.cpp
  src/graph_manager.cpp
  src/graph_merge.cpp
  src/params.cpp
  src/geofence_manager.cpp
  src/ground_projection.cpp
  src/graph_expansion.cpp
  src/grid_graph.cpp
  src/gain.cpp
  src/trajectory.cpp
  src/path_selection.cpp
  src/path_turns.cpp
  src/global_graph.cpp
  src/random_sampler.cpp
  src/log.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen mgg_kdtree::mgg_kdtree)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost mgg_kdtree)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_interface test/test_map_interface.cpp)
  target_link_libraries(test_map_interface ${PROJECT_NAME})
  ament_add_gtest(test_graph test/test_graph.cpp)
  target_link_libraries(test_graph ${PROJECT_NAME})
  ament_add_gtest(test_sensor_params test/test_sensor_params.cpp)
  target_link_libraries(test_sensor_params ${PROJECT_NAME})
  ament_add_gtest(test_graph_base test/test_graph_base.cpp)
  target_link_libraries(test_graph_base ${PROJECT_NAME})
  ament_add_gtest(test_graph_manager test/test_graph_manager.cpp)
  target_link_libraries(test_graph_manager ${PROJECT_NAME})
  ament_add_gtest(test_graph_merge test/test_graph_merge.cpp)
  target_link_libraries(test_graph_merge ${PROJECT_NAME})
  ament_add_gtest(test_params test/test_params.cpp)
  target_link_libraries(test_params ${PROJECT_NAME})
  ament_add_gtest(test_geofence test/test_geofence.cpp)
  target_link_libraries(test_geofence ${PROJECT_NAME})
  ament_add_gtest(test_ground_projection test/test_ground_projection.cpp)
  target_link_libraries(test_ground_projection ${PROJECT_NAME})
  ament_add_gtest(test_grid_graph test/test_grid_graph.cpp)
  target_link_libraries(test_grid_graph ${PROJECT_NAME})
  ament_add_gtest(test_gain test/test_gain.cpp)
  target_link_libraries(test_gain ${PROJECT_NAME})
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory ${PROJECT_NAME})
  ament_add_gtest(test_path_selection test/test_path_selection.cpp)
  target_link_libraries(test_path_selection ${PROJECT_NAME})
  ament_add_gtest(test_path_turns test/test_path_turns.cpp)
  target_link_libraries(test_path_turns ${PROJECT_NAME})
  ament_add_gtest(test_global_graph test/test_global_graph.cpp)
  target_link_libraries(test_global_graph ${PROJECT_NAME})
endif()

ament_package()
