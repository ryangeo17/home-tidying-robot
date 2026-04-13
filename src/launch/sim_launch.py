"""Launch the home-tidying-robot simulation.

Starts Gazebo with the home world, spawns the robot, bridges Gz↔ROS 2
topics, publishes TF via robot_state_publisher, and runs the nav node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('home-tidying-robot')

    # --- Process xacro → URDF string ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'home-tidying-robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # --- Gazebo sim ---
    world_file = os.path.join(pkg_share, 'worlds', 'world.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # --- Spawn robot into Gazebo ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tidying_robot',
            '-topic', 'robot_description',
            '-x', '-4.0',
            '-y', '0.0',
            '-z', '0.05',
        ],
        output='screen',
    )

    # --- Robot state publisher (TF + /robot_description) ---
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # --- ros_gz_bridge ---
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # --- Arm init (tuck arm so it doesn't clip the base) ---
    arm_init = Node(
        package='home-tidying-robot',
        executable='arm_init_node',
        output='screen',
    )

    # --- Navigation node ---
    nav_node = Node(
        package='home-tidying-robot',
        executable='nav_node',
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_pub,
        spawn_robot,
        gz_bridge,
        arm_init,
        nav_node,
    ])
