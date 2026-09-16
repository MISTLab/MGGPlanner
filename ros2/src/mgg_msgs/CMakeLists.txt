cmake_minimum_required(VERSION 3.8)
project(mgg_msgs)

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

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviourPlannerLogger.msg"
  "msg/BoundMode.msg"
  "msg/CoveragePlannerLogger.msg"
  "msg/Edge.msg"
  "msg/ExecutionPathMode.msg"
  "msg/Graph.msg"
  "msg/MappingSnapshot.msg"
  "msg/PlannerStatus.msg"
  "msg/PlanningBound.msg"
  "msg/PlanningMode.msg"
  "msg/RectangleShape.msg"
  "msg/RobotStatus.msg"
  "msg/SemanticClass.msg"
  "msg/SemanticPoint.msg"
  "msg/SemanticPolygon.msg"
  "msg/TriggerMode.msg"
  "msg/Vertex.msg"

  "srv/PciGeofence.srv"
  "srv/PciGlobal.srv"
  "srv/PciHomingTrigger.srv"
  "srv/PciInitialization.srv"
  "srv/PciSearch.srv"
  "srv/PciSetHomingPos.srv"
  "srv/PciStop.srv"
  "srv/PciToWaypoint.srv"
  "srv/PciTrigger.srv"
  "srv/PlannerDynamicGlobalBound.srv"
  "srv/PlannerGeofence.srv"
  "srv/PlannerGlobal.srv"
  "srv/PlannerGoToWaypoint.srv"
  "srv/PlannerHoming.srv"
  "srv/PlannerRequestPath.srv"
  "srv/PlannerSearch.srv"
  "srv/PlannerSetExpMode.srv"
  "srv/PlannerSetGlobalBound.srv"
  "srv/PlannerSetHomingPos.srv"
  "srv/PlannerSetPlanningMode.srv"
  "srv/PlannerSetSearchMode.srv"
  "srv/PlannerSetVel.srv"
  "srv/PlanObjective.srv"
  "srv/RefineObjectiveRoute.srv"
  "srv/QueryMapBatch.srv"
  "srv/ValidateObjectiveRoute.srv"
  "srv/PlannerSrv.srv"
  "srv/PlannerStringTrigger.srv"

  "action/PathFollower.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
