"""Build a map with slam_toolbox. Start sim.launch.py first, then drive with teleop.

ros2 launch forklift_nav slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/my_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('forklift_nav')
    slam_share = get_package_share_directory('slam_toolbox')
    nav2_share = get_package_share_directory('nav2_bringup')

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[os.path.join(share, 'config', 'scan_filter.yaml')],
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(share, 'config', 'slam_params.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([scan_filter, slam, rviz])
