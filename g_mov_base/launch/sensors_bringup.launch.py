# -------------------------- PYTHON DEPENDENCIES ------------------------------
from launch import LaunchDescription

# -------------------------- ROS 2 DEPENDENCIES  ------------------------------
from launch_ros.actions import Node

# -------------------------- LAUNCH DESCRIPTIONS ------------------------------
def generate_launch_description():
    """
    Oriented to deploy all the sensors nodes (publish/subscribe) that will
    interact with the mosquitto/thingsboard nodes.
    """
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera'
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
        # Node(
        #     package='g_mov_opencv_py',
        #     executable='get_people_centered',
        #     name='get_people'
	    #),
    ])
