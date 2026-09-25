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

# The package holds two map backends: the MOLA grid (MolaMap, fed a mapping
# process's product) and the OctoMap octree (OctomapMap, fed point clouds).
# A deployment that only uses the MOLA grid can build without OctoMap:
# -DMGG_WITH_OCTOMAP=OFF drops OctomapMap, octomap_map_node, their tests and
# the octomap dependency. Consumers read mgg_map_octomap_WITH_OCTOMAP after
# find_package(mgg_map_octomap).
option(MGG_WITH_OCTOMAP "Build the OctoMap map backend" ON)

find_package(ament_cmake REQUIRED)
find_package(mgg_core REQUIRED)
if(MGG_WITH_OCTOMAP)
  find_package(octomap REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} src/native_mola_grid.cpp src/mola_map.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} mgg_core::mgg_core
  Eigen3::Eigen nlohmann_json::nlohmann_json OpenSSL::Crypto)
if(MGG_WITH_OCTOMAP)
  target_sources(${PROJECT_NAME} PRIVATE src/octomap_map.cpp)
  target_include_directories(${PROJECT_NAME} PUBLIC ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})
  install(DIRECTORY include/ DESTINATION include)
else()
  install(DIRECTORY include/ DESTINATION include
    PATTERN octomap_map.h EXCLUDE)
endif()
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(MGG_WITH_OCTOMAP)
  # The ingest node is the only ROS-aware target here.
  add_executable(octomap_map_node src/octomap_map_node.cpp)
  target_link_libraries(octomap_map_node ${PROJECT_NAME})
  ament_target_dependencies(octomap_map_node
    rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
  install(TARGETS octomap_map_node DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
if(MGG_WITH_OCTOMAP)
  ament_export_dependencies(mgg_core octomap Eigen3 nlohmann_json OpenSSL)
else()
  ament_export_dependencies(mgg_core Eigen3 nlohmann_json OpenSSL)
endif()
ament_export_include_directories(include)

if(BUILD_TESTING)
  # A diagnostic, not a test: builds the lattice against a real product.
  add_executable(lattice_probe test/lattice_probe.cpp)
  target_link_libraries(lattice_probe ${PROJECT_NAME})
  find_package(ament_cmake_gtest REQUIRED)
  if(MGG_WITH_OCTOMAP)
    ament_add_gtest(test_octomap_map test/test_octomap_map.cpp)
    target_link_libraries(test_octomap_map ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_mola_map test/test_mola_map.cpp test/reference_binary_grid.cpp)
  target_link_libraries(test_mola_map ${PROJECT_NAME})
  target_compile_definitions(test_mola_map PRIVATE
    MGG_MAP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  ament_add_gtest(test_native_mola_index test/test_native_mola_index.cpp)
  target_link_libraries(test_native_mola_index ${PROJECT_NAME})
  ament_add_gtest(test_native_gain test/test_native_gain.cpp)
  target_link_libraries(test_native_gain ${PROJECT_NAME})
  ament_add_gtest(test_native_gain_memory test/test_native_gain_memory.cpp)
  target_link_libraries(test_native_gain_memory ${PROJECT_NAME})
endif()

ament_package(CONFIG_EXTRAS mgg_map_octomap-extras.cmake.in)
