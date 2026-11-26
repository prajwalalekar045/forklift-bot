"""Battery simulation and diagnostics for the forklift. Start sim.launch.py first.

ros2 launch forklift_nav monitor.launch.py
ros2 launch forklift_nav monitor.launch.py time_scale:=60.0 start_soc:=0.30
ros2 topic echo /battery_state
ros2 run rqt_robot_monitor rqt_robot_monitor
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_soc', default_value='0.95'),
        DeclareLaunchArgument('time_scale', default_value='1.0',
                              description='1.0 = real time; 60.0 drains an hour of energy per minute'),
        Node(
            package='forklift_nav',
            executable='robot_monitor',
            parameters=[{
                'use_sim_time': True,
                'start_soc': ParameterValue(LaunchConfiguration('start_soc'), value_type=float),
                'time_scale': ParameterValue(LaunchConfiguration('time_scale'), value_type=float),
            }],
            output='screen',
        ),
    ])
