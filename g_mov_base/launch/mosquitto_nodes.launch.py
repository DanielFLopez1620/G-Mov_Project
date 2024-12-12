# -------------------------- PYTHON DEPENDENCIES ------------------------------
from launch import LaunchDescription

# -------------------------- ROS 2 DEPENDENCIES  ------------------------------
from launch_ros.actions import Node

# -------------------------- LAUNCH DESCRIPTIONS ------------------------------
def generate_launch_description():
    """
    Oriented to deploy mosquitto communication nodes over MQTT with ROS 2
    interaction.
    """
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
