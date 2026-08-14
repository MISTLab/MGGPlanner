cmake_minimum_required(VERSION 3.8)
project(mgg_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)  # Boost.Graph, header-only

# Deliberately no ROS dependency: that is what makes the core testable and
# deployable on its own. Grows as the algorithm moves across from
# planner_common.
add_library(${PROJECT_NAME}
  src/graph.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_interface test/test_map_interface.cpp)
  target_link_libraries(test_map_interface ${PROJECT_NAME})
  ament_add_gtest(test_graph test/test_graph.cpp)
  target_link_libraries(test_graph ${PROJECT_NAME})
endif()

ament_package()
