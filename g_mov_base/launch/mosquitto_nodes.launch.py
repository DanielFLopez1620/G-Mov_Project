from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g_mov_base',
            executable='mosquitto_accel_sub',
            name='mosquitto_accel_sub'
        ),
        Node(
            package='g_mov_base',
            executable='mosquitto_servo_sub',
            name='mosquitto_servo_sub'
        ),
        Node(
            package='g_mov_base',
            executable='mosquitto_fall_sub',
            name='mosquitto_fall_sub'
        ),
    ])
