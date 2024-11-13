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
            executable='mosquitto_accel_sub',
            name='mosquitto_accel_sub'
        ),
        Node(
            package='g_mov_base',
            executable='mosquitto_servo_sub',
            name='mosquitto_servo_sub'
        ),
        Node(
            package='g_mov_opencv_py',
            executable='fall_detect_img_raw_no_gui',
            name='fall_detect_img_raw_no_gui'
        ),
        Node(
            package='g_mov_base',
            executable='mosquitto_fall_sub',
            name='mosquitto_fall_sub'
        ),
    ])
