import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_argument(context, arg):
    return LaunchConfiguration(arg).perform(context)

def generate_robot_description(context):
    # -------------------------- Paths --------------------------------------
    pkg_gmov = get_package_share_directory('g_mov_description')
    xacro_file = os.path.join(pkg_gmov, 'urdf', 'g_mov.urdf.xacro')

    gazebo = get_argument(context, "gazebo")

    mappings={'gazebo': gazebo}

    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_desc = robot_description_config.toprettyxml(indent='  ')
    
    # Passing absolute path to the robot description due to Gazebo issues finding andino_description pkg path.
    robot_desc = robot_desc.replace(
        'package://g_mov_description/', f'file://{pkg_gmov}/'
    )

    # -------------------------- Nodes ----------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'rate': 200,
        }]
    )

    return [rsp_node]


def generate_launch_description():
    # -------------------------- Launch arguments -----------------------------

    gazebo_arg = DeclareLaunchArgument(
        'gazebo',
        default_value='true',
        description="True for using gazebo tags, false otherwise"
    )
    

    return LaunchDescription([
        gazebo_arg,
        OpaqueFunction(function=generate_robot_description),
    ])