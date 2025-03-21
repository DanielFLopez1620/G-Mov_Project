import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from nav2_common.launch import ReplaceString   


def generate_launch_description():
    # ------------------------ Paths definitions ----------------------------
    pkg_gz = get_package_share_directory('g_mov_gz')
    spawn_file = os.path.join(pkg_gz, 'launch', 'spawn_module.launch.py')
    bridge_config_file_path = os.path.join(pkg_gz, 'config', 
        'ros_gz_bridge.yaml')

    # --------------------------- Configurations -----------------------------
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # -------------------------- Launch arguments -----------------------------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Specify the world file for Gazebo'
    )

    x_arg = DeclareLaunchArgument(
        'x', 
        default_value='0.0', 
        description='Initial X position'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Initial Y position'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.5',
        description='Initial Z position'
    )

    roll_arg = DeclareLaunchArgument(
        'R',
        default_value='0.0',
        description='Initial Roll'
    )

    pitch_arg = DeclareLaunchArgument(
        'P',
        default_value='0.0',
        description='Initial Pitch'
    )

    yaw_arg = DeclareLaunchArgument(
        'Y',
        default_value='0.0',
        description='Initial Yaw'
    )

    # ------------------------ Additional setups -------------------------------
    bridge_config = ReplaceString(
       source_file=bridge_config_file_path,
       replacements={'<entity>': 'g_mov_module'},
   )


    # -------------------------- Includes --------------------------------------
    spawn_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_file),
        launch_arguments= { "x": x, "y": y, "z": z,
                            "roll": roll, "pitch": pitch, 
                            "yaw": yaw, "world": world}.items(),
    )
    # ------------------------ Nodes --------------------------------------------
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config
        }],
        output='screen',
    )


    # -------------------------- Nodes ----------------------------------------
    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        spawn_include,
        bridge_node,
    ])