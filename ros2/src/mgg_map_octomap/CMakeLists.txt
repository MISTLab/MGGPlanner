cmake_minimum_required(VERSION 3.8)
project(mgg_map_octomap)

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

# Persistent MOLA products and the rolling local voxel map.
find_package(ament_cmake REQUIRED)
find_package(mgg_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
add_library(${PROJECT_NAME} src/native_mola_grid.cpp src/mola_map.cpp
  src/rolling_voxel_map.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} mgg_core::mgg_core
  Eigen3::Eigen nlohmann_json::nlohmann_json OpenSSL::Crypto)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mgg_core Eigen3 nlohmann_json OpenSSL)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # A diagnostic, not a test: builds the lattice against a real product.
  add_executable(lattice_probe test/lattice_probe.cpp)
  target_link_libraries(lattice_probe ${PROJECT_NAME})
  # The navigation benchmark on a real planning product (nav_bench.cpp).
  add_executable(mgg_nav_bench test/nav_bench.cpp)
  target_link_libraries(mgg_nav_bench ${PROJECT_NAME})
  ament_add_gtest(test_nav_bench test/test_nav_bench.cpp TIMEOUT 300)
  target_link_libraries(test_nav_bench ${PROJECT_NAME})
  # The legacy exploration replay runner (C5 baseline, exploration_replay.cpp).
  add_executable(mgg_exploration_replay test/exploration_replay.cpp)
  target_link_libraries(mgg_exploration_replay ${PROJECT_NAME} OpenSSL::Crypto)
  ament_add_gtest(test_exploration_replay test/test_exploration_replay.cpp TIMEOUT 300)
  target_link_libraries(test_exploration_replay ${PROJECT_NAME} OpenSSL::Crypto)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mola_map test/test_mola_map.cpp test/reference_binary_grid.cpp)
  target_link_libraries(test_mola_map ${PROJECT_NAME})
  target_compile_definitions(test_mola_map PRIVATE
    MGG_MAP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  ament_add_gtest(test_gate_terrain test/test_gate_terrain.cpp)
  target_link_libraries(test_gate_terrain ${PROJECT_NAME})
  target_compile_definitions(test_gate_terrain PRIVATE
    MGG_MAP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  ament_add_gtest(test_native_mola_index test/test_native_mola_index.cpp)
  target_link_libraries(test_native_mola_index ${PROJECT_NAME})
  ament_add_gtest(test_aerial_exploration test/test_aerial_exploration.cpp)
  target_link_libraries(test_aerial_exploration ${PROJECT_NAME})
  ament_add_gtest(test_native_gain test/test_native_gain.cpp)
  target_link_libraries(test_native_gain ${PROJECT_NAME})
  ament_add_gtest(test_native_gain_memory test/test_native_gain_memory.cpp)
  target_link_libraries(test_native_gain_memory ${PROJECT_NAME})
  ament_add_gtest(test_rolling_voxel_map test/test_rolling_voxel_map.cpp)
  target_link_libraries(test_rolling_voxel_map ${PROJECT_NAME})
endif()

ament_package()
