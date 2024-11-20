# -------------------------- PYTHON DEPENDENCIES ------------------------------
from launch import LaunchDescription

# -------------------------- ROS 2 DEPENDENCIES  ------------------------------
from launch_ros.actions import Node

# -------------------------- LAUNCH DESCRIPTIONS ------------------------------
def generate_launch_description():
    """
    Oriented to deploy Thingsboard communication nodes over MQTT with ROS 2
    interaction.
    """
    return LaunchDescription([
        Node(
            package='g_mov_base',
            executable='thingsboard_accel_sub',
            name='thingsboard_accel_sub'
        ),
        Node(
            package='g_mov_base',
            executable='thingsboard_servo_sub',
            name='thingsboard_servo_sub'
        ),
        Node(
            package='g_mov_base',
            executable='thingsboard_fall_sub',
            name='thingsboard_fall_sub'
        ),
        Node(
            package='g_mov_base',
            executable='thingsboard_servo_pub',
            name='thingsboard_servo_pub'
        ),
    ])
