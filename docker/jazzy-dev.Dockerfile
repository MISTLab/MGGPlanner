# ROS 2 Jazzy development environment for the MGG planner port.
#
# Phase 0 of ROS2_PORT_PLAN.md. Jazzy is chosen to match the existing
# argos3-examples tooling (tools/swarm_slam_cslam and the bridge both source
# /opt/ros/jazzy), so the planner, the ARGoS bridge and Swarm-SLAM can share
# one DDS domain without a distro mismatch.
#
# Note what is NOT here: voxblox, protobuf_catkin, glog_catkin, gflags_catkin,
# minkindr and eigen_checks are all absent by design. They are voxblox's
# dependencies, not the planner's, and section 4 of the plan drops voxblox in
# favour of an OctoMap-backed ternary occupancy map.
#
# Build (from the repo root):
#   docker build -f docker/jazzy-dev.Dockerfile -t mgg:jazzy .

FROM ros:jazzy-perception

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      git \
      python3-colcon-common-extensions \
      python3-vcstool \
      libeigen3-dev \
      libyaml-cpp-dev \
      ros-jazzy-octomap \
      ros-jazzy-octomap-msgs \
      ros-jazzy-octomap-ros \
      ros-jazzy-pcl-ros \
      ros-jazzy-pcl-conversions \
      ros-jazzy-tf2 \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-eigen \
      ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-interactive-markers \
      ros-jazzy-visualization-msgs \
      ros-jazzy-rosidl-default-generators \
      ros-jazzy-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc

WORKDIR /ws
