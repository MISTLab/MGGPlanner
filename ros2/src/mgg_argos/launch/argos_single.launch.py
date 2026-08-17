"""One foot-bot exploring under the MGG planner, driven by ARGoS.

Start this first: the bridge creates the Unix socket and ARGoS connects to
it. Then run the simulator with the matching experiment file,

    argos3 -c $(ros2 pkg prefix mgg_argos)/share/mgg_argos/experiments/mgg_footbot.argos

with ARGOS_PLUGIN_PATH covering the directory holding libmgg_footbot.so and
libmgg_bridge.so.

Every node runs on simulated time. The bridge is the only source of /clock,
and it publishes a tick only once ARGoS has produced the data for it, so the
planner cannot run ahead of the simulator or vice versa.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mgg_argos')
    default_params = os.path.join(share, 'config', 'argos_footbot.yaml')

    params_arg = DeclareLaunchArgument(
        'params', default_value=default_params,
        description='Planner, PCI and bridge parameters.')
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='r0',
        description='Robot id; must match the mgg_footbot controller params '
                    'and the bridge robot list.')
    params = LaunchConfiguration('params')
    robot = LaunchConfiguration('robot')

    # use_sim_time everywhere. The bridge publishes /clock off the ARGoS tick
    # counter, so a node on wall-clock time would stamp its work in a
    # different era from the data it is working on and every TF lookup would
    # fail.
    sim_time = {'use_sim_time': True}

    bridge = Node(
        package='mgg_argos', executable='mgg_argos_bridge',
        name='mgg_argos_bridge', output='screen',
        parameters=[params, sim_time])

    # Topics are remapped rather than namespaced: the planner and the PCI are
    # single-robot nodes here, and the bridge publishes under the robot's own
    # prefix so several robots can share one bridge in phase 7.
    planner = Node(
        package='mgg_ros', executable='mggplanner_node',
        name='mggplanner_node', output='screen',
        parameters=[params, sim_time],
        remappings=[
            ('odometry', [robot, '/odometry']),
            ('pointcloud', [robot, '/pointcloud']),
            ('graph_markers', [robot, '/graph_markers']),
        ])


    pci = Node(
        package='mgg_pci', executable='mgg_pci_node',
        name='mgg_pci', output='screen',
        parameters=[params, sim_time],
        remappings=[
            ('odometry', [robot, '/odometry']),
            # The PCI's output is what the bridge forwards to the robot.
            ('command_path', [robot, '/path']),
        ])

    return LaunchDescription([params_arg, robot_arg, bridge, planner, pci])
