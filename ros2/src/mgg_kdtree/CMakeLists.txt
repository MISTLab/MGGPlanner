cmake_minimum_required(VERSION 3.8)
project(mgg_kdtree)

find_package(ament_cmake REQUIRED)

# Third-party C, built as-is. No -Wall -Wextra: this is not our code to tidy.
add_library(${PROJECT_NAME} src/kdtree.c)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(${PROJECT_NAME} PROPERTIES C_STANDARD 99)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
