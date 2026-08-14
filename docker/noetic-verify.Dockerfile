# ROS 1 Noetic build environment for MGGPlanner.
#
# Purpose (phase 0 of ROS2_PORT_PLAN.md): there is no ROS on the host, so
# this image is what lets us (a) verify that changes to the ROS 1 tree still
# compile and (b) capture the behavioural baseline that the ROS 2 mapping
# backend will be validated against.
#
# Only what mggplanner needs to build is installed. The Gazebo simulation
# stack (rotors_simulator, smb_simulator, lidar_simulator, subt_cave_sim)
# is deliberately omitted: it is large, it is being dropped in the port, and
# nothing in planner_common or mggplanner depends on it.
#
# Build (from the repo root):
#   docker build -f docker/noetic-verify.Dockerfile -t mgg:noetic .
#
# Verify a working tree:
#   docker run --rm -v "$PWD:/ws/src/exploration/MGGPlanner:ro" mgg:noetic \
#     bash -lc 'catkin build mggplanner --no-status'

FROM ros:noetic-perception

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-catkin-tools \
      python3-wstool \
      python3-osrf-pycommon \
      git \
      libgoogle-glog-dev \
      libgflags-dev \
      libeigen3-dev \
      protobuf-compiler \
      libprotobuf-dev \
      autoconf \
      libtool \
      ros-noetic-octomap-ros \
      ros-noetic-tf \
      ros-noetic-tf2-ros \
      ros-noetic-tf-conversions \
      ros-noetic-eigen-conversions \
      ros-noetic-cv-bridge \
      ros-noetic-pcl-ros \
      ros-noetic-interactive-markers \
      ros-noetic-rviz \
      ros-noetic-joy \
      ros-noetic-twist-mux \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src

# Build-system shims and support libraries. Pinned to the same revisions the
# repo's packages_https.rosinstall specifies, so this reproduces the
# environment the ROS 1 code was written against.
RUN git clone https://github.com/catkin/catkin_simple.git misc/catkin_simple \
      && git -C misc/catkin_simple checkout 0e62848b12da76c8cc58a1add42b4f894d1ac21e
RUN git clone https://github.com/ethz-asl/eigen_catkin.git misc/eigen_catkin \
      && git -C misc/eigen_catkin checkout 00b5eb254bad8de9cd68d238aa994e443062cf30
RUN git clone https://github.com/ethz-asl/eigen_checks.git misc/eigen_checks \
      && git -C misc/eigen_checks checkout 22a6247a3df11bc285d43d1a030f4e874a413997
RUN git clone https://github.com/ethz-asl/gflags_catkin.git misc/gflags_catkin \
      && git -C misc/gflags_catkin checkout 5324e74119996a6e2da12d20e5388c17480ebd79
RUN git clone https://github.com/ethz-asl/glog_catkin.git misc/glog_catkin \
      && git -C misc/glog_catkin checkout dcb4559be6b9f9248c543df6789d46271c13efcf
RUN git clone https://github.com/ethz-asl/minkindr.git misc/minkindr \
      && git -C misc/minkindr checkout bc4503c34970a13b7ef06f62505e3333395ce02c
RUN git clone https://github.com/ethz-asl/minkindr_ros.git misc/minkindr_ros \
      && git -C misc/minkindr_ros checkout 88e0bd476f82027453f04fdf7c40c7c9a358aa1b
RUN git clone https://github.com/ethz-asl/mav_comm.git misc/mav_comm
RUN git clone https://github.com/ethz-asl/yaml_cpp_catkin.git misc/yaml_cpp_catkin
RUN git clone https://github.com/ethz-asl/protobuf_catkin.git misc/protobuf_catkin

# Mapping and the two exploration dependencies that live outside this repo.
# voxblox_rviz_plugin cannot be skipped even though the planner never uses it:
# voxblox_ros find_package()s it, so rviz has to be installed above.
RUN git clone --branch dev/noetic https://github.com/ntnu-arl/voxblox.git mapping/voxblox
RUN git clone --branch main https://github.com/ntnu-arl/pci_general.git exploration/pci_general
# adaptive_obb is NOT cloned: it is vendored into MGGPlanner itself, because it
# build-depends on planner_common and was welded to the voxblox map manager.
# See MGGPlanner/adaptive_obb/PROVENANCE.md.

WORKDIR /ws
RUN source /opt/ros/noetic/setup.bash \
    && catkin init \
    && catkin config --extend /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release

# Pre-build everything that does not depend on MGGPlanner, so that iterating
# on the planner source is fast and does not rebuild voxblox each time.
RUN source /opt/ros/noetic/setup.bash \
    && catkin build voxblox_ros --no-status -j"$(nproc)"

RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && echo 'source /ws/devel/setup.bash 2>/dev/null || true' >> /root/.bashrc

WORKDIR /ws
