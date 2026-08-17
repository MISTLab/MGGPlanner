"""Four foot-bots exploring the Bistro street under the MGG planner.

Start this first: the bridge creates the Unix socket and ARGoS connects to it.
Then run the simulator with the generated experiment,

    argos3 -c $(ros2 pkg prefix mgg_argos)/share/mgg_argos/experiments/bistro_mgg.argos

with ARGOS_PLUGIN_PATH covering the directory holding libmgg_footbot.so and
libmgg_bridge.so. tools/run_argos_demo.sh --bistro does both.

One bridge serves all four robots: it is the single socket ARGoS talks to, and
it keys everything by robot id. Each robot then gets its own planner and its
own PCI, in its own namespace, because a planner instance owns one robot's map
and one robot's graph.

The graph exchange is a single shared topic rather than a mesh of point-to-
point ones. Every planner publishes its global graph to /mgg/graphs and
subscribes to the same topic; each drops messages carrying its own robot id, so
what arrives is exactly the other three. Wiring twelve directed topics would
say the same thing at four times the length, and would need rewiring for a
fifth robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mgg_argos')
    default_params = os.path.join(share, 'config', 'bistro.yaml')

    params_arg = DeclareLaunchArgument(
        'params', default_value=default_params,
        description='Planner, PCI and bridge parameters.')
    params = LaunchConfiguration('params')

    # use_sim_time everywhere. The bridge publishes /clock off the ARGoS tick
    # counter, and a node on wall-clock time would stamp its work in a
    # different era from the data it is working on, so every TF lookup would
    # fail.
    sim_time = {'use_sim_time': True}

    nodes = [Node(package='mgg_argos', executable='mgg_argos_bridge',
                  name='mgg_argos_bridge', output='screen',
                  parameters=[params, sim_time])]

    robots = ['r0', 'r1', 'r2', 'r3']
    for index, robot in enumerate(robots):
        # robot_id is 1-based and is what the merge keys vertices by, so it
        # has to be distinct per robot; everything else comes from the shared
        # wildcard section of the parameter file.
        robot_id = {'PlanningParams.robot_id': index + 1}
        nodes.append(Node(
            package='mgg_ros', executable='mggplanner_node',
            name='mggplanner_node', namespace=robot, output='screen',
            parameters=[params, sim_time, robot_id],
            remappings=[
                ('odometry', f'/{robot}/odometry'),
                ('pointcloud', f'/{robot}/pointcloud'),
                ('graph_markers', f'/{robot}/graph_markers'),
                # One topic, all four planners. Self-messages are discarded by
                # the planner, so this is a full mesh without the wiring.
                ('neighbour_graph_out', '/mgg/graphs'),
                ('neighbour_graph_in', '/mgg/graphs'),
            ]))
        nodes.append(Node(
            package='mgg_pci', executable='mgg_pci_node',
            name='mgg_pci', namespace=robot, output='screen',
            parameters=[params, sim_time],
            remappings=[
                ('odometry', f'/{robot}/odometry'),
                ('mggplanner', f'/{robot}/mggplanner'),
                # What the bridge forwards to this robot.
                ('command_path', f'/{robot}/path'),
            ]))

    return LaunchDescription([params_arg] + nodes)
