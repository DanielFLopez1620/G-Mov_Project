import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Define paths
    pkg_description = get_package_share_directory('g_mov_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'g_mov.urdf.xacro')

    # --------------------------- Configurations -----------------------------
    use_gui = LaunchConfiguration('use_gui')

    # -------------------------- Launch arguments -----------------------------

    gui_arg = DeclareLaunchArgument(
        "use_gui", 
        default_value="true", 
        description="Use joint_state_publisher_gui"
    )

    # -------------------------- Nodes ----------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_file, 
            ])
        }]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_config_file = os.path.join(pkg_description, 'rviz', 'model_viz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
