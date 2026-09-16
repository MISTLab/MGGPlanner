cmake_minimum_required(VERSION 3.8)
project(mgg_pci)

# Default to Release; see the note in mgg_core/CMakeLists.txt. A plain
# "colcon build" is unoptimised, which catkin's workspace configuration never
# was.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(mgg_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} src/pci_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp nav_msgs geometry_msgs std_srvs std_msgs mgg_msgs)

add_executable(mgg_pci_node src/pci_main.cpp)
target_link_libraries(mgg_pci_node ${PROJECT_NAME})
ament_target_dependencies(mgg_pci_node rclcpp)
install(TARGETS mgg_pci_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(mgg_msgs rclcpp nav_msgs geometry_msgs std_srvs std_msgs)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pci_deadlock test/test_pci_deadlock.cpp TIMEOUT 120)
  target_link_libraries(test_pci_deadlock ${PROJECT_NAME})
  ament_target_dependencies(test_pci_deadlock rclcpp std_srvs)
endif()

ament_package()
