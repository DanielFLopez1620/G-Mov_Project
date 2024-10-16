from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g_mov_base',
            executable='accel_publisher',
            name='accel_publisher'
        ),
        Node(
            package='g_mov_base',
            executable='servo_subscriber',
            name='servo_subscriber'
        ),
        Node(
            package='g_mov_base',
            executable='thingspeak_accel_sub',
            name='thingspeak_accel_sub'
        ),
        Node(
            package='g_mov_base',
            executable='thingspeak_servo_sub',
            name='thingspeak_servo_sub'
        ),
        Node(
            package='g_mov_base',
            executable='thingspeak_cam_sub',
            name='thingspeak_cam_sub'
        )
    ])
