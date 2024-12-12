from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    tb3_model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type [burger, waffle, waffle_pi]'
    )

    launch_sensors = PathJoinSubstitution(
        [FindPackageShare('g_mov_base'), 'sensors_bringup.launch.py']
    )
    launch_mosquitto = PathJoinSubstitution(
        [FindPackageShare('g_mov_base'), 'mosquitto_nodes.launch.py']
    )
    launch_thingsboard = PathJoinSubstitution(
        [FindPackageShare('g_mov_base'), 'thingsboard_nodes.launch.py']
    )

    # Path to TurtleBot3 bringup launch file
    bringup_launch_file = PathJoinSubstitution(
        [FindPackageShare('turtlebot3_bringup'), 'launch', 'robot.launch.py']
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

    turtlebot3_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file),
            launch_arguments={
                'model': LaunchConfiguration('model'),
            }.items()
        )

    return LaunchDescription([
        tb3_model_arg,
        include_sensors,
        include_mosquitto,
        include_thingsboard,
        turtlebot3_bringup,
    ])