from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            
        ),
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
            package='g_mov_opencv_py',
            executable='fall_detect_img_raw_no_gui',
            name='fall_detect_img_raw_no_gui'
        ),
    ])
