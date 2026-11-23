"""Start Ignition Gazebo (Fortress) with the warehouse world, spawn the forklift and bridge to ROS 2.

ros2 launch forklift_gazebo sim.launch.py                       # AWS small warehouse (default)
ros2 launch forklift_gazebo sim.launch.py world:=warehouse      # the original generated rack world
ros2 launch forklift_gazebo sim.launch.py gui:=false x:=-3.5 y:=0.0 yaw:=0.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, OpaqueFunction, TimerAction)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# world:=<key>  ->  (file in worlds/, <world name> inside that file)
WORLDS = {
    'small_warehouse': ('small_warehouse.sdf', 'small_warehouse'),
    'warehouse': ('warehouse.world', 'warehouse'),
}
MODEL_NAME = 'forklift'    # must match /model/forklift/tf in the xacro DiffDrive plugin



def bridge_args(world_name):
    """Ignition topic @ ROS type [ign->ROS  ]ROS->ign  @ both ways"""
    return [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/model/{MODEL_NAME}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        f'/world/{world_name}/model/{MODEL_NAME}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/camera/front/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/front/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/fork/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double',
    ]


def remaps(world_name):
    return [
        (f'/model/{MODEL_NAME}/tf', '/tf'),
        (f'/world/{world_name}/model/{MODEL_NAME}/joint_state', '/joint_states'),
    ]


def launch_setup(context, *args, **kwargs):
    gz_share = get_package_share_directory('forklift_gazebo')
    desc_share = get_package_share_directory('forklift_description')

    key = LaunchConfiguration('world').perform(context)
    if key not in WORLDS:
        raise RuntimeError(f"world:='{key}' is not one of {sorted(WORLDS)}")
    world_file, world_name = WORLDS[key]
    world = os.path.join(gz_share, 'worlds', world_file)
    xacro_file = os.path.join(desc_share, 'urdf', 'forklift.urdf.xacro')
    headless = LaunchConfiguration('gui').perform(context).lower() == 'false'

    # -r: start running. gui:=false runs the server only (headless rendering for the sensors).
    gz_cmd = ['ign', 'gazebo', '-r', '-v', '3']
    if headless:
        gz_cmd += ['-s', '--headless-rendering']
    gazebo = ExecuteProcess(cmd=gz_cmd + [world], output='screen')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', world_name,
                   '-topic', 'robot_description',
                   '-name', MODEL_NAME,
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', '0.05',
                   '-Y', LaunchConfiguration('yaw')],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args(world_name),
        remappings=remaps(world_name),
        output='screen',
    )

    # The small warehouse loads ~25 meshes, so give the Ignition server longer before spawning.
    delayed_spawn = TimerAction(period=float(LaunchConfiguration('spawn_delay').perform(context)),
                                actions=[spawn])

    return [gazebo, rsp, bridge, delayed_spawn]


def generate_launch_description():
    gz_share = get_package_share_directory('forklift_gazebo')
    models = os.path.join(gz_share, 'models')

    args = [
        DeclareLaunchArgument('world', default_value='small_warehouse',
                              description='small_warehouse (AWS) or warehouse (generated racks)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_delay', default_value='8.0',
                              description='seconds to wait for the world before spawning the robot'),
    ]

    # Lets the world resolve model://... (AWS warehouse models and the ArUco marker textures).
    # IGN_* is read by Fortress; GZ_* by newer releases.
    env = [
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
    ]

    return LaunchDescription(args + env + [OpaqueFunction(function=launch_setup)])