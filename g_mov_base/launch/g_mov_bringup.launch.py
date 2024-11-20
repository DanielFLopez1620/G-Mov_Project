from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    launch_sensors = PathJoinSubstitution(
        [FindPackageShare('g_mov_base'), 'launch', 'sensors_bringup.launch.py']
    )
    launch_mosquitto = PathJoinSubstitution(
        [FindPackageShare('g_mov_base'), 'launch', 'mosquitto_nodes.launch.py']
    )
    launch_thingsboard = PathJoinSubstitution(
        [FindPackageShare('g_mov_base'), 'launch', 'thingsboard_nodes.launch.py']
    )
    
    # Include other launch files
    include_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_sensors),
    )
    include_mosquitto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_mosquitto),
    )
    include_thingsboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_thingsboard)
    )
    return LaunchDescription([
        include_sensors,
        include_mosquitto,
        include_thingsboard,
    ])