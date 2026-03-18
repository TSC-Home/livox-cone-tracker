"""
ROS 2 launch file – FastTube simulation

Starts:
  1. Gazebo Classic with the cone track world
  2. The Livox HAP bridge node (PointCloud2 → UDP 57000)

Usage (after building and sourcing the workspace):
    ros2 launch fasttube_sim simulation.launch.py

The Rust detection app should be running with:
    cargo run --release -- --sim
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('fasttube_sim')

    # Paths
    world_file   = os.path.join(pkg, 'worlds', 'track.world')
    models_path  = os.path.join(pkg, 'models')

    # ── Gazebo Classic ─────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
    )

    # ── Livox bridge node ──────────────────────────────────────────────
    bridge_node = TimerAction(
        period=3.0,   # wait 3 s for Gazebo to start
        actions=[
            Node(
                package='fasttube_sim',
                executable='livox_bridge',
                name='livox_bridge',
                output='screen',
                parameters=[{'lidar_topic': '/lidar/points'}],
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        bridge_node,
    ])
