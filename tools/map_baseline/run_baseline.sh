#!/usr/bin/env bash
# Captures the ROS 1 map-layer baseline (phase 0 of ROS2_PORT_PLAN.md).
#
# Runs inside the mgg:noetic image with the repo mounted. Starts a private
# roscore, loads the planner's real voxblox settings so the baseline reflects
# the configuration the planner actually uses (0.20 m voxels, 0.6 m
# truncation), then runs the harness and writes a CSV.
#
# Usage, from the repo root:
#   docker run --rm \
#     -v "$PWD:/ws/src/exploration/MGGPlanner" \
#     -v "$PWD/baseline:/results" \
#     mgg:noetic bash /ws/src/exploration/MGGPlanner/tools/map_baseline/run_baseline.sh
set -euo pipefail

source /opt/ros/noetic/setup.bash
cd /ws

catkin build map_baseline --no-status -j"$(nproc)"
source /ws/devel/setup.bash

REPO=/ws/src/exploration/MGGPlanner
VOXBLOX_CFG="$REPO/mggplanner/config/smb/voxblox_sim_config_sim.yaml"
OUT=${OUT:-/results/baseline_ros1.csv}
mkdir -p "$(dirname "$OUT")"

roscore &
ROSCORE_PID=$!
trap 'kill $ROSCORE_PID 2>/dev/null || true' EXIT

# Wait for the master rather than guessing at a sleep duration.
until rostopic list >/dev/null 2>&1; do sleep 0.5; done

# The harness constructs MapManagerVoxblox with ~ as its private handle, so
# the voxblox settings have to land in the node's private namespace.
rosparam load "$VOXBLOX_CFG" /map_baseline_node
# SensorParams for the volumetric-gain section. Taken from the planner's own
# config so the baseline reflects the real VLP-16 model rather than an
# invented one.
rosparam load "$REPO/mggplanner/config/smb/mgg_simple.yaml" /map_baseline_node

rosrun map_baseline map_baseline_node \
    _out:="$OUT" \
    __name:=map_baseline_node

echo "=== baseline written to $OUT ==="
head -20 "$OUT"
echo "..."
grep '^gain,' "$OUT" || true
