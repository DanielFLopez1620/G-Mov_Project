from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
    ])
